/*
 * central.c - the central controller. Runs alone on VM1.
 *
 * It monitors the whole network and issues pattern and override
 * commands. It never drives a lamp itself: that is always done by the
 * local controller, and it keeps working whether this process is
 * running or not.
 *
 * Nor does it drive a train or a gate. It can only tell the railway to
 * stop every train, or let them run again, and it acknowledges every
 * gate fault the railway reports.
 *
 * The pedestrian push buttons and the car loops are not here either:
 * they are keys on the VM2 panel (inter_panel.c), beside the
 * intersections they belong to. What they do still shows up here, in the
 * status every intersection sends on each lamp change.
 *
 * Threads, highest priority first:
 *   prio 12  t_srv       receives status from the six intersections and
 *                        crossing state and gate faults from the railway
 *   prio 10  t_hb        sends a heartbeat once a second to all seven
 *                        nodes and notices when one stops answering
 *   prio  8  t_op        single key operator input
 *   prio  6  t_display   redraws the network table
 *   prio  5  log writer
 *
 * Run:  ./central [-s speed]
 */
#include "rts_proto.h"
#include "rts_names.h"
#include "rts_timing.h"
#include "rts_util.h"
#include "rts_color.h"
#include "rts_log.h"

#include <errno.h>
#include <pthread.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include <sys/dispatch.h>
#include <sys/neutrino.h>

#define PRIO_SRV     12
#define PRIO_HB      10
#define PRIO_OP       8
#define PRIO_DISPLAY  6

/* How much green an override gives the phase it holds. */
#define OVERRIDE_S   40

typedef struct {
    inter_status_t st;
    int            online;
    int            ever_seen;
} inter_rec_t;

typedef struct {
    xing_status_t st;
    int           online;
} xing_rec_t;

static pthread_mutex_t g_lk;
static inter_rec_t     g_i[RTS_N_INTERSECTIONS];
static xing_rec_t      g_x[RTS_N_CROSSINGS];
static int             g_rail_online;

static int             g_selected = 1;      /* 0 = all six */

/* The green times u sends with the UPDATED pattern, one per phase, and
   the phase that - and + change. They start at the programmed times and
   stay as the operator leaves them. t_op writes them and t_display only
   reads them, the same as g_selected. */
#define DRAFT_MAX_S 99
static int             g_draft[PH_COUNT] = {
    (int)G_NS_S, (int)G_NS_RT_S, (int)G_EW_S, (int)G_EW_RT_S
};
static int             g_draft_ph = PH_A;

/* Random cars at the intersections, on or off. It rides on every
   heartbeat, so all six follow it within a second and a controller that
   restarts picks it up again. t_op writes it, t_hb and t_display read it. */
static int             g_random = 1;

/* The one-line message under the tables. It has its own lock because
   t_op sets it without holding g_lk, while t_srv and t_hb set it with
   g_lk held: the order is always g_lk first, then g_flash_lk. */
static pthread_mutex_t g_flash_lk;
static char            g_flash[120];
static const char     *g_flash_color = A_AMBER;
static uint64_t        g_flash_ns;

static struct termios  g_term_saved;
static int             g_term_raw;

/* Links owned by the operator thread. The heartbeat thread keeps its
   own set, so the two never have to share a connection. */
static rts_link_t      g_op_link[RTS_N_INTERSECTIONS];
static rts_link_t      g_op_rail;

/* The colour says what kind of news it is: green done, amber refused,
   red something failed, cyan only a selection. */
static void flash(const char *color, const char *fmt, ...)
{
    va_list ap;

    pthread_mutex_lock(&g_flash_lk);
    va_start(ap, fmt);
    vsnprintf(g_flash, sizeof(g_flash), fmt, ap);
    va_end(ap);
    g_flash_color = color;
    g_flash_ns    = rts_now_ns();
    pthread_mutex_unlock(&g_flash_lk);
}

/* One command to the railway on 'link', which must belong to the calling
   thread. Returns 0 accepted, 1 refused, -1 no answer. */
static int rail_send(rts_link_t *link, int action, int xing_id)
{
    rts_msg_t   m;
    rts_reply_t rep;

    memset(&m, 0, sizeof(m));
    m.hdr.type           = MSG_RAIL_CMD;
    m.hdr.sender         = SND_CENTRAL;
    m.u.rail_cmd.action  = (uint8_t)action;
    m.u.rail_cmd.xing_id = (uint8_t)xing_id;

    if (rts_link_send(link, &m, &rep) != 0) {
        return -1;
    }
    return (rep.result != 0) ? 1 : 0;
}

/* ------------------------------------------------------------------ */
/* terminal                                                            */
/* ------------------------------------------------------------------ */
static void term_restore(void)
{
    if (g_term_raw) {
        tcsetattr(STDIN_FILENO, TCSANOW, &g_term_saved);
        g_term_raw = 0;
    }
    printf("%s%s", C(A_SHOW), C(A_WRAP));
}

static void term_raw(void)
{
    struct termios t;
    if (tcgetattr(STDIN_FILENO, &g_term_saved) != 0) {
        return;
    }
    t = g_term_saved;
    t.c_lflag &= (tcflag_t)~(ICANON | ECHO);
    t.c_cc[VMIN]  = 1;
    t.c_cc[VTIME] = 0;
    if (tcsetattr(STDIN_FILENO, TCSANOW, &t) == 0) {
        g_term_raw = 1;
        atexit(term_restore);
    }
}

/* ------------------------------------------------------------------ */
/* t_srv : everything the network reports comes in here                */
/* ------------------------------------------------------------------ */
static void *t_srv(void *arg)
{
    name_attach_t *att;
    rts_rcv_t      rcv;
    rts_reply_t    rep;
    rts_link_t     ack_link;      /* this thread's own link to the railway */
    int            rcvid;
    int            ack_xing;
    (void)arg;

    att = name_attach(NULL, SVC_CENTRAL, 0);
    if (att == NULL) {
        printf("%scannot register %s%s\n", C(A_RED), SVC_CENTRAL, C(A_RESET));
        return NULL;
    }
    rts_link_init(&ack_link, rts_node_rail(), SVC_RAILWAY, "railway");
    printf("central service : %s\n", SVC_CENTRAL);

    while (rts_running) {
        rcvid = MsgReceive(att->chid, &rcv, sizeof(rcv), NULL);

        if (rcvid == -1) {
            if (errno == EINTR) continue;
            break;
        }
        if (rcvid == 0) {
            if (rcv.pulse.code == _PULSE_CODE_DISCONNECT) {
                ConnectDetach(rcv.pulse.scoid);
            }
            continue;
        }
        if (rts_server_housekeeping(rcvid, &rcv)) {
            continue;
        }

        memset(&rep, 0, sizeof(rep));
        rep.hdr.sender = SND_CENTRAL;
        rep.hdr.t_ns   = rts_now_ns();
        rep.result     = 0;
        ack_xing       = 0;

        pthread_mutex_lock(&g_lk);
        switch (rcv.msg.hdr.type) {

        case MSG_STATUS: {
            int id = rcv.msg.u.status.inter_id;
            if (id >= 1 && id <= RTS_N_INTERSECTIONS) {
                inter_rec_t *r = &g_i[id - 1];
                r->st = rcv.msg.u.status;
                if (!r->ever_seen) {
                    r->ever_seen = 1;
                    rts_log("I%d reported for the first time", id);
                }
                if (!r->online) {
                    r->online = 1;
                    flash(A_GREEN, "I%d is online", id);
                }
            } else {
                rep.result = -1;
            }
            break;
        }

        case MSG_XING_STATE: {
            int id = rcv.msg.u.xing.xing_id;
            if (id >= 1 && id <= RTS_N_CROSSINGS) {
                g_x[id - 1].st     = rcv.msg.u.xing;
                g_x[id - 1].online = 1;
                g_rail_online      = 1;
            } else {
                rep.result = -1;
            }
            break;
        }

        case MSG_GATE_FAULT:
            flash(A_RED, "GATE FAULT at crossing X%d",
                  rcv.msg.u.gate_fault.xing_id);
            rts_log("GATE FAULT reported at X%d",
                    rcv.msg.u.gate_fault.xing_id);
            ack_xing = rcv.msg.u.gate_fault.xing_id;
            break;

        default:
            rep.result = -1;
            break;
        }
        pthread_mutex_unlock(&g_lk);

        MsgReply(rcvid, EOK, &rep, sizeof(rep));

        /* A gate fault is acknowledged to the railway straight away. It
           goes after the reply, so the railway is never kept waiting on
           the control room while it reports. */
        if (ack_xing != 0) {
            if (rail_send(&ack_link, RC_FAULT_ACK, ack_xing) == 0) {
                rts_log("GATE FAULT at X%d acknowledged to the railway",
                        ack_xing);
            } else {
                rts_log("could not acknowledge the GATE FAULT at X%d",
                        ack_xing);
            }
        }
    }

    rts_link_close(&ack_link);
    name_detach(att, 0);
    return NULL;
}

/* ------------------------------------------------------------------ */
/* t_hb : one heartbeat a second to every node                         */
/* ------------------------------------------------------------------ */
static void *t_hb(void *arg)
{
    rts_link_t link[RTS_N_INTERSECTIONS];
    rts_link_t rail;
    rts_msg_t  m;
    char       svc[64];
    char       label[32];
    int        i;
    (void)arg;

    for (i = 0; i < RTS_N_INTERSECTIONS; i++) {
        rts_svc_inter(svc, sizeof(svc), i + 1);
        snprintf(label, sizeof(label), "I%d", i + 1);
        rts_link_init(&link[i], rts_node_inter(), svc, label);
    }
    rts_link_init(&rail, rts_node_rail(), SVC_RAILWAY, "railway");

    while (rts_running) {
        for (i = 0; i < RTS_N_INTERSECTIONS; i++) {
            memset(&m, 0, sizeof(m));
            m.hdr.type   = MSG_HEARTBEAT;
            m.hdr.sender = SND_CENTRAL;
            m.u.heartbeat.random_cars = (uint8_t)g_random;

            if (rts_link_send(&link[i], &m, NULL) != 0) {
                /*
                 * Three failures in a row is the 3 s rule: the local
                 * controller is declared failed and shown as offline.
                 * It is still controlling its own intersection; what we
                 * have lost is the ability to monitor it.
                 */
                if (link[i].misses >= 3) {
                    pthread_mutex_lock(&g_lk);
                    if (g_i[i].online) {
                        g_i[i].online = 0;
                        flash(A_RED, "I%d is not answering - it keeps running "
                              "on its own", i + 1);
                        rts_log("I%d declared offline after %d misses",
                                i + 1, link[i].misses);
                    }
                    pthread_mutex_unlock(&g_lk);
                }
            } else {
                pthread_mutex_lock(&g_lk);
                g_i[i].online = 1;
                pthread_mutex_unlock(&g_lk);
            }
        }

        memset(&m, 0, sizeof(m));
        m.hdr.type   = MSG_HEARTBEAT;
        m.hdr.sender = SND_CENTRAL;
        if (rts_link_send(&rail, &m, NULL) != 0 && rail.misses >= 3) {
            pthread_mutex_lock(&g_lk);
            if (g_rail_online) {
                g_rail_online = 0;
                flash(A_RED, "railway controller is not answering");
                rts_log("railway controller declared offline");
            }
            pthread_mutex_unlock(&g_lk);
        } else if (rail.online) {
            pthread_mutex_lock(&g_lk);
            g_rail_online = 1;
            pthread_mutex_unlock(&g_lk);
        }

        rts_sleep_ms(rts_ms(T_HEARTBEAT_S));
    }

    for (i = 0; i < RTS_N_INTERSECTIONS; i++) {
        rts_link_close(&link[i]);
    }
    rts_link_close(&rail);
    return NULL;
}

/* ------------------------------------------------------------------ */
/* operator commands                                                   */
/* ------------------------------------------------------------------ */
/* Returns 0 accepted, 1 refused (reason in *why), -1 no answer. */
static int send_to(int id, rts_msg_t *m, uint8_t *why)
{
    rts_reply_t rep;

    if (id < 1 || id > RTS_N_INTERSECTIONS) return -1;

    memset(&rep, 0, sizeof(rep));
    if (rts_link_send(&g_op_link[id - 1], m, &rep) != 0) {
        flash(A_RED, "I%d did not answer the command", id);
        return -1;
    }
    if (rep.result != 0) {
        flash(A_AMBER, "I%d REFUSED the command: %s", id,
              rts_reject_name(rep.reason));
        rts_log("I%d refused a command: %s", id, rts_reject_name(rep.reason));
        if (why != NULL) *why = rep.reason;
        return 1;
    }
    flash(A_GREEN, "I%d accepted the command", id);
    return 0;
}

static void send_selected(rts_msg_t *m)
{
    int     i, rc, ok = 0, refused = 0, lost = 0;
    uint8_t why = REJ_NONE;

    if (g_selected != 0) {
        send_to(g_selected, m, NULL);
        return;
    }
    for (i = 1; i <= RTS_N_INTERSECTIONS; i++) {
        rts_msg_t copy = *m;
        rc = send_to(i, &copy, &why);
        if (rc == 0)     ok++;
        else if (rc > 0) refused++;
        else             lost++;
    }
    /* One line for all six, in the colour of the worst answer. */
    if (ok == RTS_N_INTERSECTIONS) {
        flash(A_GREEN, "all six intersections accepted the command");
    } else if (refused > 0) {
        flash(lost > 0 ? A_RED : A_AMBER,
              "all six: %d accepted, %d REFUSED (%s), %d did not answer",
              ok, refused, rts_reject_name(why), lost);
    } else {
        flash(A_RED, "all six: %d accepted, %d did not answer", ok, lost);
    }
}

static void cmd_pattern(int pattern)
{
    rts_msg_t m;
    int       i;

    memset(&m, 0, sizeof(m));
    m.hdr.type              = MSG_SET_PATTERN;
    m.hdr.sender            = SND_CENTRAL;
    m.u.set_pattern.pattern = (uint8_t)pattern;

    if (pattern == PAT_UPDATED) {
        /* The operator's own green times, not checked here. The local
           controller refuses the whole update if one is unsafe, a green
           under 8 s for example, and keeps running what it had. */
        for (i = 0; i < PH_COUNT; i++) {
            m.u.set_pattern.green_s[i] = (uint16_t)g_draft[i];
        }
    }
    send_selected(&m);
}

/* Hold one phase green at the target for OVERRIDE_S seconds of green,
   or drop the hold. The local controller decides when: a phase that
   drives into the rail-side arm is held only once any train has gone. */
static void cmd_override(int phase, int cancel)
{
    rts_msg_t m;

    memset(&m, 0, sizeof(m));
    m.hdr.type                  = MSG_OVERRIDE;
    m.hdr.sender                = SND_CENTRAL;
    m.u.override_cmd.phase      = (uint8_t)phase;
    m.u.override_cmd.timeout_s  = OVERRIDE_S;
    m.u.override_cmd.cancel     = (uint8_t)cancel;
    send_selected(&m);
}

/* Stop every train on the line, or let them run again. */
static void cmd_line(int stop)
{
    int rc = rail_send(&g_op_rail, stop ? RC_STOP_TRAINS : RC_RESUME_TRAINS, 0);

    if (rc < 0) {
        flash(A_RED, "the railway controller did not answer");
    } else if (rc > 0) {
        flash(A_AMBER, "the railway REFUSED the command");
    } else {
        flash(stop ? A_RED : A_GREEN, "%s",
              stop ? "every train stopped by the control room"
                   : "trains released, the line runs again");
        rts_log("operator %s every train", stop ? "stopped" : "released");
    }
}

static void *t_op(void *arg)
{
    int  ch, esc = 0;
    char svc[64], label[32];
    int  i;
    (void)arg;

    for (i = 0; i < RTS_N_INTERSECTIONS; i++) {
        rts_svc_inter(svc, sizeof(svc), i + 1);
        snprintf(label, sizeof(label), "I%d", i + 1);
        rts_link_init(&g_op_link[i], rts_node_inter(), svc, label);
    }
    rts_link_init(&g_op_rail, rts_node_rail(), SVC_RAILWAY, "railway");

    while (rts_running) {
        ch = getchar();
        if (ch == EOF) {
            rts_sleep_ms(200);
            continue;
        }
        /* An arrow key arrives as ESC [ and a letter. Drop all three, or
           its [ would change the phase on the UPDATED row. */
        if (ch == 27) {
            esc = 1;
            continue;
        }
        if (esc) {
            esc = (esc == 1 && ch == '[') ? 2 : 0;
            continue;
        }

        switch (ch) {
        case '1': case '2': case '3':
        case '4': case '5': case '6':
            g_selected = ch - '0';
            flash(A_CYAN, "selected I%d", g_selected);
            break;
        case '0':
            g_selected = 0;
            flash(A_CYAN, "selected all six intersections");
            break;

        case 'f': cmd_pattern(PAT_FIXED);   break;
        case 's': cmd_pattern(PAT_SENSOR);  break;
        case 'u': cmd_pattern(PAT_UPDATED); break;

        /* The UPDATED row: [ and ] pick the phase, - and + change its
           green by one second. Nothing is sent until u. */
        case '[': g_draft_ph = (g_draft_ph + PH_COUNT - 1) % PH_COUNT; break;
        case ']': g_draft_ph = (g_draft_ph + 1) % PH_COUNT;            break;
        case '-':
            if (g_draft[g_draft_ph] > 1) g_draft[g_draft_ph]--;
            break;
        case '+': case '=':
            if (g_draft[g_draft_ph] < DRAFT_MAX_S) g_draft[g_draft_ph]++;
            break;

        /* Hold one phase green at the target, or drop the hold. */
        case 'A': cmd_override(PH_A, 0); break;
        case 'B': cmd_override(PH_B, 0); break;
        case 'C': cmd_override(PH_C, 0); break;
        case 'D': cmd_override(PH_D, 0); break;
        case 'x': cmd_override(0, 1);    break;

        /* Random cars at all six, on or off. The push buttons and the car
           loops are keys on the VM2 panel; this switch stays here, so a
           test can take the random traffic away. */
        case 'r':
            g_random = !g_random;
            flash(A_CYAN, "random cars %s at all six intersections",
                  g_random ? "ON" : "OFF");
            rts_log("operator turned random cars %s", g_random ? "on" : "off");
            break;

        /* The control room cannot put a train on the line or move a gate;
           those keys are on the railway. When something is wrong on the
           line it can stop every train, and let them run again. */
        case 'e': cmd_line(1); break;
        case 'E': cmd_line(0); break;

        case 'q': rts_running = 0; break;
        default:  break;
        }
    }
    return NULL;
}

/* ------------------------------------------------------------------ */
/* the dashboard                                                       */
/* ------------------------------------------------------------------ */
#define PANEL_W 103     /* the window is 105 columns; keep clear of the edge */

/* The crossing next to an intersection: X1 for I1 and I2, X2 for I3 and
   I4, X3 for I5 and I6, the same pairing the railway node uses. */
static int xing_of(int inter_id)
{
    return (inter_id + 1) / 2;
}

/* The two roads of an intersection, as intersection_iN.c names them. R1
   runs north-south on the west side of the tracks and R2 on the east
   side, so I1, I3 and I5 are on R1. R3, R4 and R5 run east-west and cross
   the tracks at X1, X2 and X3, so both intersections of a crossing are on
   the same one. */
static int road_ns(int inter_id)
{
    return (inter_id % 2 == 1) ? 1 : 2;
}

static int road_ew(int inter_id)
{
    return 2 + xing_of(inter_id);
}

/* One line under the intersection table: which roads each one joins. The
   north-south road runs phases A and B, the east-west road C and D. */
static void draw_roads(rts_frame_t *f)
{
    int i;

    rts_frame_add(f, "  %sroads%s", C(A_GREY), C(A_RESET));
    for (i = 1; i <= RTS_N_INTERSECTIONS; i++) {
        rts_frame_add(f, "  %sI%d%s R%dxR%d", C(A_WHITE), i, C(A_RESET),
                      road_ns(i), road_ew(i));
    }
    rts_frame_line(f, "   %s(R%d R%d R%d cross the tracks)", C(A_GREY),
                   road_ew(1), road_ew(3), road_ew(5));
}

/* One row of the intersection table. Every field has a fixed width on
   screen, and the heading in t_display() uses the same widths. */
static void draw_inter(rts_frame_t *f, int id, const inter_rec_t *r,
                       int targeted)
{
    const inter_status_t *st = &r->st;
    char                  label[16], lamps[256], peds[128], pattern[32];

    snprintf(label, sizeof(label), "I%d", id);
    rts_frame_add(f, " %s%s%s%s%-3s%s  ",
                  C(A_CYAN), targeted ? ">" : " ", C(A_RESET),
                  targeted ? C(A_CYAN) : C(A_WHITE), label, C(A_RESET));
    if (!r->ever_seen) {
        rts_frame_line(f, "%s%-4s  waiting for the first status message",
                       C(A_GREY), "----");
        return;
    }
    /* An override outranks the pattern, so it takes the column: the
       phase held, or the phase waiting for a train to pass. UPDATED
       shows the green times the intersection says it is running. */
    if (st->override_active == 2) {
        snprintf(pattern, sizeof(pattern), "OVR %s wait",
                 rts_phase_name(st->override_phase));
    } else if (st->override_active) {
        snprintf(pattern, sizeof(pattern), "OVERRIDE %s",
                 rts_phase_name(st->override_phase));
    } else if (st->pattern == PAT_UPDATED) {
        snprintf(pattern, sizeof(pattern), "%u/%u/%u/%u",
                 (unsigned)st->green_s[PH_A], (unsigned)st->green_s[PH_B],
                 (unsigned)st->green_s[PH_C], (unsigned)st->green_s[PH_D]);
    } else {
        snprintf(pattern, sizeof(pattern), "%s",
                 rts_pattern_name(st->pattern));
    }
    rts_frame_line(f,
        "%s%-4s%s  %s%-2s%s %s%-7s%s   %s   %s   %sX%d%s %s%-7s%s  "
        "%s%-11s%s  %s%s",
        r->online ? C(A_GREEN) : C(A_RED), r->online ? "up" : "DOWN",
        C(A_RESET),
        C(A_WHITE), rts_phase_name(st->phase), C(A_RESET),
        rts_state_color(st->state), rts_state_name(st->state), C(A_RESET),
        rts_lamps_str(lamps, sizeof(lamps), st->veh),
        rts_peds_str(peds, sizeof(peds), st->ped),
        C(A_GREY), xing_of(id), C(A_RESET),
        rts_xing_color(st->xing_state), rts_xing_name(st->xing_state),
        C(A_RESET),
        st->override_active == 2 ? C(A_AMBER)
            : st->override_active ? C(A_MAGENTA) : "", pattern, C(A_RESET),
        st->train_hold ? C(A_BG_RED) : "",
        st->train_hold ? " RAIL HOLD " : "");
}

/* One row of the crossing table, same widths as its heading. */
static void draw_xing(rts_frame_t *f, int id, const xing_rec_t *r)
{
    const xing_status_t *st = &r->st;
    char                 tracks[64];

    rts_frame_add(f, "  %sX%-2d%s  ", C(A_WHITE), id, C(A_RESET));
    if (!r->online) {
        rts_frame_line(f, "%sno data yet", C(A_GREY));
        return;
    }
    rts_frame_line(f,
        "%s%-7s%s  %s%-6s%s  %s     %s%-6s%s  %-6u  %sI%d I%d",
        rts_xing_color(st->state), rts_xing_name(st->state), C(A_RESET),
        rts_gate_color(st->gate_pos), rts_gate_name(st->gate_pos),
        C(A_RESET),
        rts_tracks_str(tracks, sizeof(tracks), st->track_busy),
        st->train_signal ? C(A_GREEN) : C(A_RED),
        st->train_signal ? "GREEN" : "RED", C(A_RESET),
        st->trains_served,
        C(A_GREY), 2 * id - 1, 2 * id);
}

/*
 * The green times u will send. The phase that - and + change is marked,
 * and a time the intersections will refuse is red: a green outside
 * 8..60 s, or a cycle over 150 s. Sending it anyway is allowed, which is
 * how the refusal is shown.
 */
static void draw_draft(rts_frame_t *f)
{
    double cycle = 0.0;
    int    i, bad, sel;

    rts_frame_add(f, "  %s%-9s%s", C(A_GREY), "UPDATED", C(A_RESET));
    for (i = 0; i < PH_COUNT; i++) {
        bad    = g_draft[i] < G_MIN_S || g_draft[i] > G_MAX_S;
        sel    = (i == g_draft_ph);
        cycle += g_draft[i] + T_AMBER_S + T_ALLRED_S;
        rts_frame_add(f, "%s%s%s %s%c%2d%c%s   ",
                      C(A_WHITE), rts_phase_name((uint8_t)i), C(A_RESET),
                      sel ? C(bad ? A_BG_RED : A_KEY)
                          : C(bad ? A_RED : A_WHITE),
                      sel ? '>' : ' ', g_draft[i], sel ? '<' : ' ',
                      C(A_RESET));
    }
    rts_frame_add(f, "%scycle %.0f s%s   ",
                  cycle > T_CYCLE_MAX_S ? C(A_RED) : C(A_GREEN), cycle,
                  C(A_RESET));
    rts_frame_key(f, "[ ]", "phase");
    rts_frame_add(f, "  ");
    rts_frame_key(f, "- +", "1 s");
    rts_frame_eol(f);
}

static void *t_display(void *arg)
{
    static rts_frame_t f;           /* 16 KB, kept off the thread stack */
    inter_rec_t        ir[RTS_N_INTERSECTIONS];
    xing_rec_t         xr[RTS_N_CROSSINGS];
    char               msg[sizeof(g_flash)];
    const char        *msg_color;
    uint64_t           msg_ns;
    char               right[80], target[8];
    int                rail_up, stopped, sel, i;
    (void)arg;

    while (rts_running) {
        rts_sleep_ms(400);

        /* Copy the tables and let go of the lock straight away. A write
           to the terminal can stall when the SSH link is slow, and t_srv
           must never be kept waiting for that. */
        pthread_mutex_lock(&g_lk);
        memcpy(ir, g_i, sizeof(ir));
        memcpy(xr, g_x, sizeof(xr));
        rail_up = g_rail_online;
        pthread_mutex_unlock(&g_lk);

        /* Every crossing reports whether the control room has stopped the
           trains, so any crossing that is online will do. */
        stopped = 0;
        for (i = 0; i < RTS_N_CROSSINGS; i++) {
            if (xr[i].online && xr[i].st.line_stopped) {
                stopped = 1;
            }
        }

        pthread_mutex_lock(&g_flash_lk);
        memcpy(msg, g_flash, sizeof(msg));
        msg_color = g_flash_color;
        msg_ns    = g_flash_ns;
        pthread_mutex_unlock(&g_flash_lk);

        sel = g_selected;
        if (sel == 0) snprintf(target, sizeof(target), "ALL");
        else          snprintf(target, sizeof(target), "I%d", sel);

        rts_frame_begin(&f);

        /* The bar turns red while the railway cannot be reached. */
        snprintf(right, sizeof(right), "node %s   speed %.1fx   railway %s ",
                 rts_hostname(), rts_speed(), rail_up ? "up" : "DOWN");
        rts_frame_bar(&f, rail_up ? A_BG_BLUE : A_BG_RED,
                      " CENTRAL CONTROL ROOM", right, PANEL_W);
        rts_frame_line(&f, "");

        rts_frame_line(&f, "  %s%-3s  %-4s  %-2s %-7s   %-26s   %-7s   "
                       "%-10s  %-11s  %-11s",
                       C(A_HEAD), "INT", "LINK", "PH", "STATE",
                       "  A      B      C      D", "PED", "CROSSING",
                       "PATTERN", "");
        for (i = 0; i < RTS_N_INTERSECTIONS; i++) {
            draw_inter(&f, i + 1, &ir[i], sel == 0 || sel == i + 1);
        }
        rts_frame_line(&f, "  %sA = N-S through   B = N-S right turn   "
                       "C = E-W through   D = E-W right turn", C(A_GREY));
        draw_roads(&f);
        rts_frame_line(&f, "");

        rts_frame_line(&f, "  %s%-3s  %-7s  %-6s  %-6s  %-6s  %-6s  %-6s",
                       C(A_HEAD), "X", "STATE", "GATES", "TRACKS", "SIGNAL",
                       "TRAINS", "SERVES");
        for (i = 0; i < RTS_N_CROSSINGS; i++) {
            draw_xing(&f, i + 1, &xr[i]);
        }
        rts_frame_line(&f, "");

        if (msg[0] != '\0' && rts_now_ns() - msg_ns < 8000000000ULL) {
            rts_frame_line(&f, "  %s>>%s %s%s", C(A_GREY), C(A_RESET),
                           C(msg_color), msg);
        } else {
            rts_frame_line(&f, "");
        }

        /* The key rows follow straight on, so the panel stays inside the
           24 rows of the window. */
        rts_frame_add(&f, "  %s%-9s%s%s%-5s%s", C(A_GREY), "TARGET",
                      C(A_RESET), C(A_CYAN), target, C(A_RESET));
        rts_frame_key(&f, "1-6", "pick one");
        rts_frame_add(&f, "  ");
        rts_frame_key(&f, "0", "all six");
        rts_frame_add(&f, "  ");
        rts_frame_key(&f, "q", "quit");
        rts_frame_eol(&f);
        rts_frame_keys(&f, 9, "PATTERN", "f", "fixed", "s", "sensor",
                       "u", "updated, times below", NULL);
        draw_draft(&f);
        rts_frame_keys(&f, 9, "COMMAND", "A", "hold A", "B", "hold B",
                       "C", "hold C", "D", "hold D", "x", "cancel", "r",
                       g_random ? "random cars ON" : "random cars OFF", NULL);
        rts_frame_add(&f, "  %s%-9s%s", C(A_GREY), "RAILWAY", C(A_RESET));
        rts_frame_key(&f, "e", "stop all trains");
        rts_frame_add(&f, "  ");
        rts_frame_key(&f, "E", "let them run");
        rts_frame_add(&f, "    %s%s", stopped ? C(A_RED) : C(A_GREEN),
                      stopped ? "trains STOPPED" : "trains running");
        rts_frame_eol(&f);
        rts_frame_end(&f);
    }
    printf("%s%s", C(A_SHOW), C(A_WRAP));
    return NULL;
}

/* ------------------------------------------------------------------ */
int main(int argc, char **argv)
{
    pthread_t th_srv, th_hb, th_op, th_disp;

    rts_start("central", argc, argv);
    rts_log_start("central");

    rts_mutex_init(&g_lk);
    rts_mutex_init(&g_flash_lk);
    memset(g_i, 0, sizeof(g_i));
    memset(g_x, 0, sizeof(g_x));

    term_raw();

    rts_thread(&th_srv, t_srv, NULL, PRIO_SRV);
    rts_thread(&th_hb,  t_hb,  NULL, PRIO_HB);
    rts_thread(&th_op,  t_op,  NULL, PRIO_OP);

    rts_sleep_ms(1500);
    printf("%s", C(A_CLEAR));
    rts_thread(&th_disp, t_display, NULL, PRIO_DISPLAY);

    while (rts_running) {
        rts_sleep_ms(200);
    }

    term_restore();
    printf("\n%scentral controller stopped%s\n", C(A_AMBER), C(A_RESET));
    rts_log("=== central stopped ===");
    rts_log_stop();
    return 0;
}
