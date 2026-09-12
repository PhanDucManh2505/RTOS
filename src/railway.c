/*
 * railway.c - the railway controller. Runs alone on VM3.
 *
 * It owns the three level crossings on the double track corridor and
 * the link to the train lines. Nothing on the traffic light side can
 * command a crossing: the only way in is a gate command from the
 * control room, which arrives here first. In the other direction this
 * process pushes the crossing state out to the two intersections
 * either side of each crossing, and to the control room for display.
 *
 * Why the state is pushed as a message rather than published in shared
 * memory: shared memory cannot cross a Qnet link, and the crossings and
 * the intersections now live on different machines. The intersections
 * therefore run a watchdog, and silence for longer than the watchdog is
 * treated exactly like a train being on the crossing.
 *
 * Threads, highest priority first:
 *   prio 20  t_xing x3   one per crossing, runs the crossing state
 *                        machine, drives the gates and publishes state
 *   prio 12  t_srv       receives gate commands and heartbeats
 *   prio 10  t_train     generates train arrivals
 *   prio  8  t_op        single key operator input
 *   prio  6  t_display   redraws the crossing table
 *   prio  5  log writer
 *
 * Run:  ./railway [-s speed]
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

#define PRIO_XING    20
#define PRIO_SRV     12
#define PRIO_TRAIN   10
#define PRIO_OP       8
#define PRIO_DISPLAY  6

#define TICK_MS 100

typedef struct {
    int             id;              /* 1..3 */
    int             inter_a;         /* the two intersections either side */
    int             inter_b;

    pthread_mutex_t lk;

    int             state;           /* xing_state_t                     */
    uint8_t         track_busy;      /* bit0 track A, bit1 track B       */
    int             gate_pos;        /* gate_pos_t                       */
    int             gate_target;     /* where the gates were told to go  */
    uint64_t        gate_move_end;   /* when a healthy gate would arrive */
    int             gate_fault;
    int             train_signal;    /* 0 red to the train, 1 green      */
    int             manual;          /* control room is holding it       */
    uint32_t        seq;
    uint32_t        trains_served;

    uint64_t        warn_end_ns;     /* when the gates must be down      */
    uint64_t        occupy_end_ns[2];/* per track                        */
    int             pending[2];      /* a train has been requested       */

    uint64_t        last_push_ns;
    int             last_pushed_state;

    rts_link_t      to_a, to_b, to_c;
} xing_t;

static xing_t          g_x[RTS_N_CROSSINGS];
static int             g_auto_trains = 1;
static int             g_night_mode  = 0;   /* 0 peak, 1 night */
static int             g_selected    = 1;
static struct termios  g_term_saved;
static int             g_term_raw    = 0;

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
        return;                    /* not a terminal, e.g. output piped */
    }
    t = g_term_saved;
    /* Read one key at a time and do not echo it, so the live display
       is never scrambled by what the operator types. */
    t.c_lflag &= (tcflag_t)~(ICANON | ECHO);
    t.c_cc[VMIN]  = 1;
    t.c_cc[VTIME] = 0;
    if (tcsetattr(STDIN_FILENO, TCSANOW, &t) == 0) {
        g_term_raw = 1;
        atexit(term_restore);
    }
}

/* ------------------------------------------------------------------ */
/* recent events, shown on the panel                                   */
/* ------------------------------------------------------------------ */
#define N_EVENTS 6

typedef struct {
    double      t_s;          /* seconds since start                 */
    const char *color;        /* escape code, put through C() later  */
    char        text[64];
} rail_event_t;

static pthread_mutex_t g_ev_lk;
static rail_event_t    g_ev[N_EVENTS];
static int             g_ev_count;          /* how many were ever added */

/*
 * Add a line to the event list on the panel. Nothing is printed here:
 * a line printed from another thread lands wherever the redraw left the
 * cursor and stays behind as stale text, and a crossing thread at
 * prio 20 must not wait on a terminal write. t_display shows the list.
 */
static void event(const char *color, const char *fmt, ...)
{
    rail_event_t *e;
    va_list       ap;

    pthread_mutex_lock(&g_ev_lk);
    e        = &g_ev[g_ev_count % N_EVENTS];
    e->t_s   = rts_uptime_s();
    e->color = color;
    va_start(ap, fmt);
    vsnprintf(e->text, sizeof(e->text), fmt, ap);
    va_end(ap);
    g_ev_count++;
    pthread_mutex_unlock(&g_ev_lk);
}

/* One line for each change of crossing state. */
static void state_event(const xing_status_t *st, int from)
{
    int id = st->xing_id;

    switch (st->state) {
    case XS_WARNING:
        event(A_AMBER, "X%d WARNING  train coming, gates still up", id);
        break;
    case XS_CLOSED:
        if (st->track_busy) {
            event(A_RED, "X%d CLOSED   gates down, train on the crossing", id);
        } else {
            event(A_RED, "X%d CLOSED   gates held down by the control room",
                  id);
        }
        break;
    case XS_CLEAR:
        if (from == XS_FAULT) {
            event(A_GREEN, "X%d CLEAR    fault cleared, gates going up", id);
        } else {
            event(A_GREEN, "X%d CLEAR    gates up, road open", id);
        }
        break;
    default:
        break;          /* FAULT gets its own line from report_fault() */
    }
}

/* ------------------------------------------------------------------ */
/* publishing                                                          */
/* ------------------------------------------------------------------ */
static void fill_xing(const xing_t *x, xing_status_t *st)
{
    memset(st, 0, sizeof(*st));
    st->xing_id       = (uint8_t)x->id;
    st->state         = (uint8_t)x->state;
    st->track_busy    = x->track_busy;
    st->gate_pos      = (uint8_t)x->gate_pos;
    st->gate_fault    = (uint8_t)x->gate_fault;
    st->train_signal  = (uint8_t)x->train_signal;
    st->manual        = (uint8_t)x->manual;
    st->seq           = x->seq;
    st->trains_served = x->trains_served;
    st->t_ns          = rts_now_ns();
}

/*
 * Send the crossing state to both intersections and to the control
 * room. Called on every change and once a second in between, so a
 * controller that restarts learns the state within one second.
 */
static void publish(xing_t *x, int changed)
{
    rts_msg_t     m;
    xing_status_t st;
    int           from;

    pthread_mutex_lock(&x->lk);
    if (changed) {
        x->seq++;
    }
    fill_xing(x, &st);
    x->last_push_ns      = rts_now_ns();
    from                 = x->last_pushed_state;
    x->last_pushed_state = x->state;
    pthread_mutex_unlock(&x->lk);

    memset(&m, 0, sizeof(m));
    m.hdr.type   = MSG_XING_STATE;
    m.hdr.sender = SND_RAILWAY;
    m.u.xing     = st;

    rts_link_send(&x->to_a, &m, NULL);
    rts_link_send(&x->to_b, &m, NULL);
    rts_link_send(&x->to_c, &m, NULL);

    if (changed) {
        rts_log("X%d -> %s gates=%d busy=%02x fault=%d train=%s",
                x->id, rts_xing_name((uint8_t)x->state), x->gate_pos,
                x->track_busy, x->gate_fault,
                x->train_signal ? "GREEN" : "RED");
    }
    if (st.state != from) {
        state_event(&st, from);
    }
}

static void report_fault(xing_t *x)
{
    rts_msg_t m;

    memset(&m, 0, sizeof(m));
    m.hdr.type                  = MSG_GATE_FAULT;
    m.hdr.sender                = SND_RAILWAY;
    m.u.gate_fault.xing_id      = (uint8_t)x->id;
    m.u.gate_fault.gate         = 1;
    m.u.gate_fault.train_stopped= 1;

    rts_link_send(&x->to_c, &m, NULL);
    rts_log("X%d GATE FAULT reported, train given a red signal", x->id);
    event(A_BG_RED, " X%d GATE FAULT  train stopped, control room told ",
          x->id);
}

/* ------------------------------------------------------------------ */
/* gates                                                               */
/* ------------------------------------------------------------------ */
static void gate_command(xing_t *x, int target)
{
    /* Idempotent on purpose. The state machine re-issues the same command
       on every tick until the gates arrive, so restarting the movement
       here would push gate_move_end forever out of reach: the gates would
       never land, the crossing would never close, and the timeout below
       would never turn a jammed gate into a fault. */
    if (x->gate_target == target) {
        return;
    }
    x->gate_target   = target;
    x->gate_pos      = GATE_MOVING;
    /* A jammed gate simply never reaches the position it was told to
       go to, and the timeout below turns that into a fault. */
    x->gate_move_end = rts_now_ns() +
                       rts_ns(x->gate_fault ? (T_GATE_TIMEOUT_S + 5.0)
                                            : T_GATE_MOVE_S);
}

/* Returns 1 if the gates moved or a fault appeared. */
static int gate_tick(xing_t *x)
{
    if (x->gate_pos != GATE_MOVING) {
        return 0;
    }
    if (rts_now_ns() >= x->gate_move_end) {
        x->gate_pos = x->gate_target;
        return 1;
    }
    /* Too slow: the gate has not reached its commanded position. */
    if (!x->gate_fault &&
        rts_now_ns() >= x->gate_move_end + rts_ns(T_GATE_TIMEOUT_S)) {
        x->gate_fault = 1;
        return 1;
    }
    if (x->gate_fault && x->state != XS_FAULT) {
        return 1;
    }
    return 0;
}

/* ------------------------------------------------------------------ */
/* the crossing state machine                                          */
/* ------------------------------------------------------------------ */
static void *t_xing(void *arg)
{
    xing_t  *x = (xing_t *)arg;
    int      changed;
    uint64_t now;

    while (rts_running) {
        rts_sleep_ms(TICK_MS);
        changed = 0;

        pthread_mutex_lock(&x->lk);
        now = rts_now_ns();

        changed |= gate_tick(x);

        /* A fault outranks everything. The train gets a red light and
           the crossing keeps reporting FAULT until the fault is cleared,
           which the traffic controllers treat exactly like a train. */
        if (x->gate_fault) {
            if (x->state != XS_FAULT) {
                x->state        = XS_FAULT;
                x->train_signal = 0;
                changed         = 1;
                pthread_mutex_unlock(&x->lk);
                report_fault(x);
                publish(x, 1);
                continue;
            }
        } else {
            switch (x->state) {

            case XS_CLEAR:
                if (x->manual && x->gate_target == GATE_DOWN) {
                    x->state = XS_CLOSED;
                    changed  = 1;
                    break;
                }
                if (x->pending[0] || x->pending[1]) {
                    /* A train has been detected on approach. The road
                       system is given the full warning time before the
                       gates start to move. */
                    x->state        = XS_WARNING;
                    x->warn_end_ns  = now + rts_ns(T_WARNING_S);
                    x->train_signal = 1;
                    changed         = 1;
                }
                break;

            case XS_WARNING:
                if (now >= x->warn_end_ns) {
                    gate_command(x, GATE_DOWN);
                    if (x->gate_pos == GATE_DOWN) {
                        int t;
                        x->state = XS_CLOSED;
                        for (t = 0; t < 2; t++) {
                            if (x->pending[t]) {
                                x->pending[t]      = 0;
                                x->track_busy     |= (uint8_t)(1u << t);
                                x->occupy_end_ns[t]= now +
                                                     rts_ns(T_TRAIN_OCCUPY_S);
                                x->trains_served++;
                            }
                        }
                        changed = 1;
                    }
                }
                break;

            case XS_CLOSED: {
                int t;
                for (t = 0; t < 2; t++) {
                    /* A second train may arrive while the first is still
                       on the crossing. The gates simply stay down. */
                    if (x->pending[t]) {
                        x->pending[t]       = 0;
                        x->track_busy      |= (uint8_t)(1u << t);
                        x->occupy_end_ns[t] = now + rts_ns(T_TRAIN_OCCUPY_S);
                        x->trains_served++;
                        changed             = 1;
                    }
                    if ((x->track_busy & (1u << t)) &&
                        now >= x->occupy_end_ns[t]) {
                        x->track_busy &= (uint8_t)~(1u << t);
                        changed        = 1;
                    }
                }
                if (x->track_busy == 0 && !x->manual) {
                    gate_command(x, GATE_UP);
                    if (x->gate_pos == GATE_UP) {
                        x->state = XS_CLEAR;
                        changed  = 1;
                    }
                }
                break;
            }

            case XS_FAULT:
                /* The operator cleared the fault. Start again from a
                   known safe position with the gates up. */
                x->state        = XS_CLEAR;
                x->train_signal = 1;
                gate_command(x, GATE_UP);
                changed         = 1;
                break;

            default:
                break;
            }
        }

        /* Repeat the state once a second even when nothing changed, so
           a controller that has just restarted picks it up quickly and
           the watchdog on the other side stays satisfied. */
        if (!changed &&
            rts_now_ns() - x->last_push_ns < rts_ns(T_XING_REFRESH_S)) {
            pthread_mutex_unlock(&x->lk);
            continue;
        }
        pthread_mutex_unlock(&x->lk);

        publish(x, changed);
    }
    return NULL;
}

/* ------------------------------------------------------------------ */
/* trains                                                              */
/* ------------------------------------------------------------------ */
/* who: "auto" or "operator", shown on the panel */
static void request_train(int xing_id, int track, const char *who)
{
    xing_t *x;

    if (xing_id < 1 || xing_id > RTS_N_CROSSINGS) return;
    if (track  < 0 || track  > 1)                 return;

    x = &g_x[xing_id - 1];
    pthread_mutex_lock(&x->lk);
    x->pending[track] = 1;
    pthread_mutex_unlock(&x->lk);

    rts_log("train requested on X%d track %c", xing_id, 'A' + track);
    event(A_CYAN, "X%d train approaching on track %c (%s)",
          xing_id, 'A' + track, who);
}

static void *t_train(void *arg)
{
    uint64_t next_ns;
    (void)arg;

    next_ns = rts_now_ns() + rts_ns(T_TRAIN_PEAK_S / 2.0);

    while (rts_running) {
        rts_sleep_ms(200);
        if (!g_auto_trains) {
            next_ns = rts_now_ns() +
                      rts_ns(g_night_mode ? T_TRAIN_NIGHT_S : T_TRAIN_PEAK_S);
            continue;
        }
        if (rts_now_ns() >= next_ns) {
            int xid   = 1 + (rand() % RTS_N_CROSSINGS);
            int track = rand() % 2;
            request_train(xid, track, "auto");
            next_ns = rts_now_ns() +
                      rts_ns(g_night_mode ? T_TRAIN_NIGHT_S : T_TRAIN_PEAK_S);
        }
    }
    return NULL;
}

/* ------------------------------------------------------------------ */
/* commands from the control room                                      */
/* ------------------------------------------------------------------ */
static void *t_srv(void *arg)
{
    name_attach_t *att;
    rts_rcv_t      rcv;
    rts_reply_t    rep;
    int            rcvid;
    (void)arg;

    att = name_attach(NULL, SVC_RAILWAY, 0);
    if (att == NULL) {
        printf("%scannot register %s%s\n", C(A_RED), SVC_RAILWAY, C(A_RESET));
        return NULL;
    }
    printf("railway service : %s\n", SVC_RAILWAY);

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
        rep.hdr.sender = SND_RAILWAY;
        rep.result     = 0;

        if (rcv.msg.hdr.type == MSG_GATE_CMD) {
            int     id  = rcv.msg.u.gate_cmd.xing_id;
            int     act = rcv.msg.u.gate_cmd.action;
            xing_t *x;

            if (id < 1 || id > RTS_N_CROSSINGS) {
                rep.result = -1;
            } else {
                const char *what  = NULL;          /* for the event list */
                const char *color = A_MAGENTA;     /* the operator acted */

                x = &g_x[id - 1];
                pthread_mutex_lock(&x->lk);
                switch (act) {
                case GC_NORMAL:       x->manual = 0;
                                      what = "back to automatic"; break;
                case GC_FORCE_DOWN:   x->manual = 1; gate_command(x, GATE_DOWN);
                                      what = "gates forced down"; break;
                case GC_FORCE_UP:     x->manual = 1; gate_command(x, GATE_UP);
                                      what = "gates held up"; break;
                case GC_INJECT_FAULT: x->gate_fault = 1;
                                      what = "gate fault injected"; color = A_RED;
                                      break;
                case GC_CLEAR_FAULT:  x->gate_fault = 0;
                                      what = "gate fault cleared"; color = A_GREEN;
                                      break;
                case GC_REQUEST_TRAIN_A: x->pending[0] = 1;
                                      what = "train approaching on track A";
                                      color = A_CYAN; break;
                case GC_REQUEST_TRAIN_B: x->pending[1] = 1;
                                      what = "train approaching on track B";
                                      color = A_CYAN; break;
                default: rep.result = -1; break;
                }
                pthread_mutex_unlock(&x->lk);
                rts_log("gate command %d on X%d from the control room", act, id);
                if (what != NULL) {
                    event(color, "X%d %s (control room)", id, what);
                }
            }
        } else if (rcv.msg.hdr.type != MSG_HEARTBEAT) {
            rep.result = -1;
        }

        MsgReply(rcvid, EOK, &rep, sizeof(rep));
    }

    name_detach(att, 0);
    return NULL;
}

/* ------------------------------------------------------------------ */
/* display                                                             */
/* ------------------------------------------------------------------ */
#define PANEL_W 64      /* the window is 67 columns; keep clear of the edge */

static void *t_display(void *arg)
{
    static rts_frame_t f;           /* 16 KB, kept off the thread stack */
    xing_status_t      st[RTS_N_CROSSINGS];
    rail_event_t       ev[N_EVENTS];
    char               right[80], id[8], tracks[64];
    int                ev_count, shown, first, sel, i;
    (void)arg;

    while (rts_running) {
        rts_sleep_ms(400);

        /* Copy first and draw after, so a crossing thread never waits
           while the terminal is being written. */
        for (i = 0; i < RTS_N_CROSSINGS; i++) {
            pthread_mutex_lock(&g_x[i].lk);
            fill_xing(&g_x[i], &st[i]);
            pthread_mutex_unlock(&g_x[i].lk);
        }
        pthread_mutex_lock(&g_ev_lk);
        memcpy(ev, g_ev, sizeof(ev));
        ev_count = g_ev_count;
        pthread_mutex_unlock(&g_ev_lk);
        sel = g_selected;

        rts_frame_begin(&f);
        snprintf(right, sizeof(right), "node %s  speed %.1fx  trains %s  %s ",
                 rts_hostname(), rts_speed(),
                 g_auto_trains ? "AUTO" : "MANUAL",
                 g_night_mode ? "NIGHT" : "PEAK");
        rts_frame_bar(&f, A_BG_BLUE, " RAILWAY CONTROLLER", right, PANEL_W);
        rts_frame_line(&f, "");

        /* The heading uses the same widths as the rows below it. */
        rts_frame_line(&f, "  %s%-3s  %-7s  %-6s  %-6s  %-6s  %-5s  %-6s  %-6s",
                       C(A_HEAD), "X", "STATE", "GATES", "TRACKS", "SIGNAL",
                       "FAULT", "MODE", "TRAINS");
        for (i = 0; i < RTS_N_CROSSINGS; i++) {
            const xing_status_t *x  = &st[i];
            int                  me = (i + 1 == sel);

            snprintf(id, sizeof(id), "X%d", x->xing_id);
            rts_frame_line(&f,
                " %s%s%s%s%-3s%s  %s%-7s%s  %s%-6s%s  %s     "
                "%s%-6s%s  %s%-5s%s  %s%-6s%s  %u",
                C(A_CYAN), me ? ">" : " ", C(A_RESET),
                me ? C(A_CYAN) : C(A_WHITE), id, C(A_RESET),
                rts_xing_color(x->state), rts_xing_name(x->state), C(A_RESET),
                rts_gate_color(x->gate_pos), rts_gate_name(x->gate_pos),
                C(A_RESET),
                rts_tracks_str(tracks, sizeof(tracks), x->track_busy),
                x->train_signal ? C(A_GREEN) : C(A_RED),
                x->train_signal ? "GREEN" : "RED", C(A_RESET),
                x->gate_fault ? C(A_RED) : C(A_GREY),
                x->gate_fault ? "YES" : "no", C(A_RESET),
                x->manual ? C(A_MAGENTA) : C(A_GREY),
                x->manual ? "manual" : "auto", C(A_RESET),
                x->trains_served);
        }
        rts_frame_add(&f, " %s", C(A_GREY));
        for (i = 0; i < RTS_N_CROSSINGS; i++) {
            rts_frame_add(&f, " X%d serves I%d I%d  ",
                          g_x[i].id, g_x[i].inter_a, g_x[i].inter_b);
        }
        rts_frame_eol(&f);
        rts_frame_line(&f, "");

        /* the last few events, oldest first, newest at the bottom */
        rts_frame_line(&f, "  %s%-59s", C(A_HEAD), "RECENT EVENTS");
        shown = (ev_count < N_EVENTS) ? ev_count : N_EVENTS;
        first = ev_count - shown;
        for (i = 0; i < N_EVENTS; i++) {
            if (i < shown) {
                const rail_event_t *e = &ev[(first + i) % N_EVENTS];
                rts_frame_line(&f, "  %s%8.1fs%s  %s%.51s",
                               C(A_GREY), e->t_s, C(A_RESET),
                               C(e->color), e->text);
            } else if (i == 0) {
                rts_frame_line(&f, "  %snothing yet", C(A_GREY));
            } else {
                rts_frame_line(&f, "");
            }
        }
        rts_frame_line(&f, "");

        rts_frame_keys(&f, 10, "CROSSING", "1", "X1", "2", "X2", "3", "X3",
                       NULL);
        rts_frame_keys(&f, 10, "TRAIN", "a", "on track A", "b", "on track B",
                       NULL);
        rts_frame_keys(&f, 10, "GATE", "f", "inject fault", "c", "clear fault",
                       NULL);
        rts_frame_keys(&f, 10, "SYSTEM", "t", "auto trains on/off",
                       "n", "peak / night", "q", "quit", NULL);
        rts_frame_end(&f);
    }
    printf("%s%s", C(A_SHOW), C(A_WRAP));
    return NULL;
}

/* ------------------------------------------------------------------ */
/* operator                                                            */
/* ------------------------------------------------------------------ */
static void *t_op(void *arg)
{
    int     ch;
    xing_t *x;
    (void)arg;

    while (rts_running) {
        ch = getchar();
        if (ch == EOF) {
            rts_sleep_ms(200);
            continue;
        }
        x = &g_x[g_selected - 1];

        switch (ch) {
        case '1': case '2': case '3':
            g_selected = ch - '0';
            break;
        case 'a':
            request_train(g_selected, 0, "operator");
            break;
        case 'b':
            request_train(g_selected, 1, "operator");
            break;
        case 'f':
            pthread_mutex_lock(&x->lk);
            x->gate_fault = 1;
            pthread_mutex_unlock(&x->lk);
            rts_log("operator injected a gate fault on X%d", g_selected);
            event(A_RED, "X%d gate fault injected (operator)", g_selected);
            break;
        case 'c':
            pthread_mutex_lock(&x->lk);
            x->gate_fault = 0;
            x->manual     = 0;
            pthread_mutex_unlock(&x->lk);
            rts_log("operator cleared the fault on X%d", g_selected);
            event(A_GREEN, "X%d fault cleared, back to automatic (operator)",
                  g_selected);
            break;
        case 't':
            g_auto_trains = !g_auto_trains;
            event(A_WHITE, "automatic trains %s", g_auto_trains ? "on" : "off");
            break;
        case 'n':
            g_night_mode = !g_night_mode;
            event(A_WHITE, "timetable %s", g_night_mode ? "NIGHT" : "PEAK");
            break;
        case 'q':
            rts_running = 0;
            break;
        default:
            break;
        }
    }
    return NULL;
}

/* ------------------------------------------------------------------ */
int main(int argc, char **argv)
{
    pthread_t th_x[RTS_N_CROSSINGS], th_srv, th_train, th_op, th_disp;
    char      svc[64];
    int       i;

    rts_start("railway", argc, argv);
    rts_log_start("railway");
    rts_mutex_init(&g_ev_lk);

    for (i = 0; i < RTS_N_CROSSINGS; i++) {
        xing_t *x = &g_x[i];

        memset(x, 0, sizeof(*x));
        x->id           = i + 1;
        x->inter_a      = 2 * i + 1;      /* X1 -> I1 and I2, X2 -> I3, I4 */
        x->inter_b      = 2 * i + 2;
        x->state        = XS_CLEAR;
        x->gate_pos     = GATE_UP;
        x->gate_target  = GATE_UP;
        x->train_signal = 1;
        rts_mutex_init(&x->lk);

        rts_svc_inter_evt(svc, sizeof(svc), x->inter_a);
        rts_link_init(&x->to_a, rts_node_inter(), svc, "I?");
        snprintf(x->to_a.label, sizeof(x->to_a.label), "I%d", x->inter_a);

        rts_svc_inter_evt(svc, sizeof(svc), x->inter_b);
        rts_link_init(&x->to_b, rts_node_inter(), svc, "I?");
        snprintf(x->to_b.label, sizeof(x->to_b.label), "I%d", x->inter_b);

        rts_link_init(&x->to_c, rts_node_central(), SVC_CENTRAL, "central");

        printf(" crossing X%d serves I%d and I%d\n",
               x->id, x->inter_a, x->inter_b);
    }

    srand((unsigned)(rts_now_ns() & 0xffffffu));
    term_raw();

    for (i = 0; i < RTS_N_CROSSINGS; i++) {
        rts_thread(&th_x[i], t_xing, &g_x[i], PRIO_XING);
    }
    rts_thread(&th_srv,   t_srv,     NULL, PRIO_SRV);
    rts_thread(&th_train, t_train,   NULL, PRIO_TRAIN);
    rts_thread(&th_op,    t_op,      NULL, PRIO_OP);

    rts_sleep_ms(1500);          /* let the banner be read before it goes */
    printf("%s", C(A_CLEAR));
    rts_thread(&th_disp, t_display, NULL, PRIO_DISPLAY);

    while (rts_running) {
        rts_sleep_ms(200);
    }

    term_restore();
    printf("\n%srailway controller stopped%s\n", C(A_AMBER), C(A_RESET));
    rts_log("=== railway stopped ===");
    rts_log_stop();
    return 0;
}
