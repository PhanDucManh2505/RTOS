/*
 * central.c - the central controller. Runs alone on VM1.
 *
 * It monitors the whole network and issues pattern, override and gate
 * commands. It never drives a lamp itself: that is always done by the
 * local controller, and it keeps working whether this process is
 * running or not.
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
#include "rts_safety.h"

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

typedef struct {
    inter_status_t st;
    int            online;
    int            ever_seen;
    uint32_t       updates;
    uint64_t       last_ns;
} inter_rec_t;

typedef struct {
    xing_status_t st;
    int           online;
    uint64_t      last_ns;
} xing_rec_t;

static pthread_mutex_t g_lk;
static inter_rec_t     g_i[RTS_N_INTERSECTIONS];
static xing_rec_t      g_x[RTS_N_CROSSINGS];
static int             g_rail_online;
static uint64_t        g_rail_last_ns;

static int             g_selected = 1;      /* 0 = all six */

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
    int            rcvid;
    (void)arg;

    att = name_attach(NULL, SVC_CENTRAL, 0);
    if (att == NULL) {
        printf("%scannot register %s%s\n", C(A_RED), SVC_CENTRAL, C(A_RESET));
        return NULL;
    }
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

        pthread_mutex_lock(&g_lk);
        switch (rcv.msg.hdr.type) {

        case MSG_STATUS: {
            int id = rcv.msg.u.status.inter_id;
            if (id >= 1 && id <= RTS_N_INTERSECTIONS) {
                inter_rec_t *r = &g_i[id - 1];
                r->st        = rcv.msg.u.status;
                r->last_ns   = rts_now_ns();
                r->updates++;
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
                g_x[id - 1].st      = rcv.msg.u.xing;
                g_x[id - 1].online  = 1;
                g_x[id - 1].last_ns = rts_now_ns();
                g_rail_online       = 1;
                g_rail_last_ns      = rts_now_ns();
            } else {
                rep.result = -1;
            }
            break;
        }

        case MSG_GATE_FAULT:
            flash(A_RED, "GATE FAULT at crossing X%d - train stopped",
                  rcv.msg.u.gate_fault.xing_id);
            rts_log("GATE FAULT reported at X%d, train stopped=%d",
                    rcv.msg.u.gate_fault.xing_id,
                    rcv.msg.u.gate_fault.train_stopped);
            break;

        default:
            rep.result = -1;
            break;
        }
        pthread_mutex_unlock(&g_lk);

        MsgReply(rcvid, EOK, &rep, sizeof(rep));
    }

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
                g_i[i].online  = 1;
                g_i[i].last_ns = rts_now_ns();
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
            g_rail_online  = 1;
            g_rail_last_ns = rts_now_ns();
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

static void cmd_pattern(int pattern, int use_custom)
{
    rts_msg_t m;

    memset(&m, 0, sizeof(m));
    m.hdr.type              = MSG_SET_PATTERN;
    m.hdr.sender            = SND_CENTRAL;
    m.u.set_pattern.pattern = (uint8_t)pattern;
    m.u.set_pattern.offset_s= (uint16_t)T_OFFSET_S;

    if (use_custom) {
        /* The advanced pattern: the control room supplies new green
           times, which the local controller validates before use. */
        m.u.set_pattern.green_s[PH_A] = 30;
        m.u.set_pattern.green_s[PH_B] = 10;
        m.u.set_pattern.green_s[PH_C] = 14;
        m.u.set_pattern.green_s[PH_D] = 10;
    }
    send_selected(&m);
}

static void cmd_bad_pattern(void)
{
    rts_msg_t m;

    /* Deliberately illegal: a green far below the pedestrian minimum.
       The local controller must refuse it and say why. */
    memset(&m, 0, sizeof(m));
    m.hdr.type                    = MSG_SET_PATTERN;
    m.hdr.sender                  = SND_CENTRAL;
    m.u.set_pattern.pattern       = PAT_UPDATED;
    m.u.set_pattern.green_s[PH_A] = 2;
    m.u.set_pattern.green_s[PH_B] = 2;
    m.u.set_pattern.green_s[PH_C] = 2;
    m.u.set_pattern.green_s[PH_D] = 2;
    send_selected(&m);
}

static void cmd_override(int on)
{
    rts_msg_t m;

    memset(&m, 0, sizeof(m));
    m.hdr.type                  = MSG_OVERRIDE;
    m.hdr.sender                = SND_CENTRAL;
    m.u.override_cmd.phase      = PH_A;
    m.u.override_cmd.timeout_s  = 40;
    m.u.override_cmd.cancel     = (uint8_t)(on ? 0 : 1);
    send_selected(&m);
}

static void cmd_button(int ped_id)
{
    rts_msg_t m;

    memset(&m, 0, sizeof(m));
    m.hdr.type          = MSG_PED_BUTTON;
    m.hdr.sender        = SND_CENTRAL;
    m.u.button.ped_id   = (uint8_t)ped_id;
    send_selected(&m);
}

static void cmd_gate(int xing_id, int action, const char *what)
{
    rts_msg_t   m;
    rts_reply_t rep;

    memset(&m, 0, sizeof(m));
    m.hdr.type              = MSG_GATE_CMD;
    m.hdr.sender            = SND_CENTRAL;
    m.u.gate_cmd.xing_id    = (uint8_t)xing_id;
    m.u.gate_cmd.action     = (uint8_t)action;

    if (rts_link_send(&g_op_rail, &m, &rep) != 0) {
        flash(A_RED, "the railway controller did not answer");
    } else if (rep.result != 0) {
        flash(A_AMBER, "the railway REFUSED %s at crossing X%d", what, xing_id);
    } else {
        flash(A_GREEN, "%s sent to crossing X%d", what, xing_id);
        rts_log("operator sent %s to X%d", what, xing_id);
    }
}

static void *t_op(void *arg)
{
    int  ch;
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

        case 'f': cmd_pattern(PAT_FIXED,   0); break;
        case 's': cmd_pattern(PAT_SENSOR,  0); break;
        case 'u': cmd_pattern(PAT_UPDATED, 1); break;
        case 'x': cmd_bad_pattern();           break;

        case 'o': cmd_override(1); break;
        case 'c': cmd_override(0); break;

        case 'p': cmd_button(PD_N); break;
        case 'P': cmd_button(PD_E); break;

        case 'a': cmd_gate(1, GC_REQUEST_TRAIN_A, "train on track A"); break;
        case 'b': cmd_gate(1, GC_REQUEST_TRAIN_B, "train on track B"); break;
        case 'g': cmd_gate(1, GC_INJECT_FAULT,    "gate fault");       break;
        case 'G': cmd_gate(1, GC_CLEAR_FAULT,     "clear fault");      break;
        case 'd': cmd_gate(1, GC_FORCE_DOWN,      "force gates down"); break;
        case 'n': cmd_gate(1, GC_NORMAL,          "return to normal"); break;

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

/* One row of the intersection table. Every field has a fixed width on
   screen, and the heading in t_display() uses the same widths. */
static void draw_inter(rts_frame_t *f, int id, const inter_rec_t *r,
                       int targeted)
{
    const inter_status_t *st = &r->st;
    char                  label[16], lamps[256], peds[128];

    snprintf(label, sizeof(label), "I%d", id);
    rts_frame_add(f, " %s%s%s%s%-3s%s  ",
                  C(A_CYAN), targeted ? ">" : " ", C(A_RESET),
                  targeted ? C(A_CYAN) : C(A_WHITE), label, C(A_RESET));
    if (!r->ever_seen) {
        rts_frame_line(f, "%s%-4s  waiting for the first status message",
                       C(A_GREY), "----");
        return;
    }
    rts_frame_line(f,
        "%s%-4s%s  %s%-2s%s %s%-7s%s   %s   %s   %sX%d%s %s%-7s%s  "
        "%s%-8s%s  %s%s",
        r->online ? C(A_GREEN) : C(A_RED), r->online ? "up" : "DOWN",
        C(A_RESET),
        C(A_WHITE), rts_phase_name(st->phase), C(A_RESET),
        rts_state_color(st->state), rts_state_name(st->state), C(A_RESET),
        rts_lamps_str(lamps, sizeof(lamps), st->veh),
        rts_peds_str(peds, sizeof(peds), st->ped),
        C(A_GREY), xing_of(id), C(A_RESET),
        rts_xing_color(st->xing_state), rts_xing_name(st->xing_state),
        C(A_RESET),
        /* an override outranks the pattern, so it takes the column */
        st->override_active ? C(A_MAGENTA) : "",
        st->override_active ? "OVERRIDE" : rts_pattern_name(st->pattern),
        C(A_RESET),
        st->train_hold ? C(A_BG_RED) : "",
        st->train_hold ? " RAIL HOLD " : "");
}

/* One row of the crossing table, same widths as its heading. */
static void draw_xing(rts_frame_t *f, int id, const xing_rec_t *r,
                      int targeted)
{
    const xing_status_t *st = &r->st;
    char                 label[16], tracks[64];

    snprintf(label, sizeof(label), "X%d", id);
    rts_frame_add(f, " %s%s%s%s%-3s%s  ",
                  C(A_CYAN), targeted ? ">" : " ", C(A_RESET),
                  targeted ? C(A_CYAN) : C(A_WHITE), label, C(A_RESET));
    if (!r->online) {
        rts_frame_line(f, "%sno data yet", C(A_GREY));
        return;
    }
    rts_frame_line(f,
        "%s%-7s%s  %s%-6s%s  %s     %s%-6s%s  %s%-6s%s  %-6u  %sI%d I%d",
        rts_xing_color(st->state), rts_xing_name(st->state), C(A_RESET),
        rts_gate_color(st->gate_pos), rts_gate_name(st->gate_pos),
        C(A_RESET),
        rts_tracks_str(tracks, sizeof(tracks), st->track_busy),
        st->train_signal ? C(A_GREEN) : C(A_RED),
        st->train_signal ? "GREEN" : "RED", C(A_RESET),
        st->manual ? C(A_MAGENTA) : C(A_GREY),
        st->manual ? "manual" : "auto", C(A_RESET),
        st->trains_served,
        C(A_GREY), 2 * id - 1, 2 * id);
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
    int                rail_up, sel, i;
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
                       "%-10s  %-8s  %-11s",
                       C(A_HEAD), "INT", "LINK", "PH", "STATE",
                       "  A      B      C      D", "PED", "CROSSING",
                       "PATTERN", "");
        for (i = 0; i < RTS_N_INTERSECTIONS; i++) {
            draw_inter(&f, i + 1, &ir[i], sel == 0 || sel == i + 1);
        }
        rts_frame_line(&f, "  %sA = R1 through   B = R1 right turn   "
                       "C = R3 through   D = R3 right turn   "
                       "(R3 crosses the tracks)", C(A_GREY));
        rts_frame_line(&f, "");

        rts_frame_line(&f, "  %s%-3s  %-7s  %-6s  %-6s  %-6s  %-6s  %-6s  "
                       "%-6s", C(A_HEAD), "X", "STATE", "GATES", "TRACKS",
                       "SIGNAL", "MODE", "TRAINS", "SERVES");
        for (i = 0; i < RTS_N_CROSSINGS; i++) {
            /* the gate keys below always act on X1 */
            draw_xing(&f, i + 1, &xr[i], i == 0);
        }
        rts_frame_line(&f, "");

        if (msg[0] != '\0' && rts_now_ns() - msg_ns < 8000000000ULL) {
            rts_frame_line(&f, "  %s>>%s %s%s", C(A_GREY), C(A_RESET),
                           C(msg_color), msg);
        } else {
            rts_frame_line(&f, "");
        }
        rts_frame_line(&f, "");

        rts_frame_add(&f, "  %s%-9s%s%s%-5s%s", C(A_GREY), "TARGET",
                      C(A_RESET), C(A_CYAN), target, C(A_RESET));
        rts_frame_key(&f, "1-6", "pick one");
        rts_frame_add(&f, "  ");
        rts_frame_key(&f, "0", "all six");
        rts_frame_add(&f, "  ");
        rts_frame_key(&f, "q", "quit");
        rts_frame_eol(&f);
        rts_frame_keys(&f, 9, "PATTERN", "f", "fixed", "s", "sensor",
                       "u", "updated", "x", "illegal, must be refused", NULL);
        rts_frame_keys(&f, 9, "COMMAND", "o", "override, hold A",
                       "c", "cancel", "p", "ped button N",
                       "P", "ped button E", NULL);
        rts_frame_keys(&f, 9, "X1 GATE", "a", "train track A",
                       "b", "train track B", "g", "fault", "G", "clear",
                       "d", "gates down", "n", "normal", NULL);
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
