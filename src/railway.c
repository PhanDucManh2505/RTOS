/*
 * railway.c - the railway controller. Runs alone on VM3.
 *
 * It owns the three level crossings on the double track line and the
 * trains that run on it. Nothing outside can drive a train or a gate:
 * the control room can only stop every train, let them run again, and
 * acknowledge a gate fault. In the other direction this process pushes
 * the crossing state out to the two intersections either side of each
 * crossing, and to the control room for display.
 *
 * The line runs X1 - X2 - X3, north to south. A train can only enter at
 * either end: a southbound (NS) train enters at X1 on track A and is then
 * detected at X2 and at X3, a northbound (SN) train enters at X3 on track
 * B and runs the other way. How long a train takes from one crossing to
 * the next is in rts_timing.h.
 *
 * Why the state is pushed as a message rather than published in shared
 * memory: shared memory cannot cross a Qnet link, and the crossings and
 * the intersections live on different machines. The intersections
 * therefore run a watchdog, and silence for longer than the watchdog is
 * treated exactly like a train being on the crossing.
 *
 * Threads, highest priority first:
 *   prio 20  t_xing x3   one per crossing, works out what its two tracks
 *                        need, drives the gates and publishes state
 *   prio 12  t_srv       receives control room commands and heartbeats
 *   prio 10  t_train     runs the timetable and moves trains along the line
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

/* ------------------------------------------------------------------ */
/* the line                                                            */
/* ------------------------------------------------------------------ */

/* A direction is also the track it runs on: A = 0, B = 1. */
enum { DIR_NS = 0, DIR_SN = 1 };

static const char *const dir_name[2] = { "NS", "SN" };

/* The crossings each direction passes, in order, and how many seconds
   after entering the line each of them detects the train. */
static const int route[2][RTS_N_CROSSINGS] = {
    { 1, 2, 3 },
    { 3, 2, 1 }
};
static const double route_s[2][RTS_N_CROSSINGS] = {
    { 0.0, T_X1_X2_S, T_X1_X2_S + T_X2_X3_S },
    { 0.0, T_X2_X3_S, T_X2_X3_S + T_X1_X2_S }
};

/* How long a train keeps one track of a crossing, from detection until
   it has left: warning, gates coming down, then the train itself. */
#define T_TRACK_S (T_WARNING_S + T_GATE_MOVE_S + T_TRAIN_OCCUPY_S)

/* What the timetable sends. */
enum { TT_NONE = 0, TT_RUSH, TT_OFFPEAK };

static const char *tt_name(int tt)
{
    switch (tt) {
    case TT_RUSH:    return "RUSH";
    case TT_OFFPEAK: return "OFF-PEAK";
    default:         return "NONE";
    }
}

static double tt_interval_s(int tt)
{
    return (tt == TT_RUSH) ? T_TRAIN_RUSH_S : T_TRAIN_OFFPEAK_S;
}

typedef struct {
    int             id;              /* 1..3 */
    int             inter_a;         /* the two intersections either side */
    int             inter_b;

    pthread_mutex_t lk;

    int             state;           /* xing_state_t                     */
    uint8_t         track_busy;      /* bit0 track A, bit1 track B: a
                                        train is on the crossing now     */
    int             gate_pos;        /* gate_pos_t                       */
    int             gate_target;     /* where the gates were told to go  */
    uint64_t        gate_move_end;   /* when a healthy gate would arrive */
    int             gate_fault;
    int             train_signal;    /* 0 red to the train, 1 green      */
    uint32_t        seq;
    uint32_t        trains_served;

    /* When the detector on each track saw the train that is coming, 0
       while that track is empty. Everything else about the train follows
       from this one time - see t_xing(). */
    uint64_t        detect_ns[2];

    uint64_t        last_push_ns;
    int             last_pushed_state;

    rts_link_t      to_a, to_b, to_c;
} xing_t;

#define N_TRAINS 8

typedef struct {
    int      active;
    int      dir;         /* DIR_NS or DIR_SN, which is also its track  */
    int      next;        /* how many crossings on its route have seen it */
    uint32_t no;          /* running number, for the panel and the log  */
    uint64_t enter_ns;    /* when the first crossing detected it        */
} train_t;

static xing_t          g_x[RTS_N_CROSSINGS];
static int             g_selected    = 1;
static struct termios  g_term_saved;
static int             g_term_raw    = 0;

/* The trains and the timetable, all under g_train_lk. When both are
   needed the order is always g_train_lk first, then a crossing's lk. */
static pthread_mutex_t g_train_lk;
static train_t         g_train[N_TRAINS];
static uint32_t        g_train_no;
static int             g_timetable = TT_RUSH;
static int             g_next_dir  = DIR_NS;   /* the timetable alternates */
static uint64_t        g_next_ns;              /* when it sends the next   */

/* The control room has stopped every train (see line_stop). Written under
   g_train_lk; the crossing threads only read it, to show a red signal. */
static int             g_line_stopped;
static uint64_t        g_stop_ns;              /* when it stopped them     */

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
            event(A_RED, "X%d CLOSED   gates down, train due", id);
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
    st->line_stopped  = (uint8_t)g_line_stopped;
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

    /* The event goes first: the control room acknowledges the report as
       soon as it has it, and that line has to land below this one. */
    event(A_BG_RED, " X%d GATE FAULT  control room told ", x->id);
    rts_log("X%d GATE FAULT reported, train given a red signal", x->id);
    rts_link_send(&x->to_c, &m, NULL);
}

/* ------------------------------------------------------------------ */
/* gates                                                               */
/* ------------------------------------------------------------------ */
static void gate_command(xing_t *x, int target)
{
    /* Idempotent on purpose. The crossing re-issues the same command on
       every tick until the gates arrive, so restarting the movement here
       would push gate_move_end forever out of reach: the gates would
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
/* the crossing                                                        */
/* ------------------------------------------------------------------ */
/*
 * Every tick the crossing works out from its two detector times what its
 * tracks need, and its state follows from that and from where the gates
 * are. For each train, counted from the moment it was detected:
 *
 *      0 .. 30 s   WARNING: the road is warned, the gates stay up
 *     30 .. 34 s   the gates come down
 *     34 .. 49 s   the train is on the crossing
 *     49 s         it has gone; the gates rise once no other train is coming
 *
 * The two tracks are timed separately, so when two trains meet at a
 * crossing - one each way - each gets its own warning and its own time on
 * the crossing, and the gates stay down from the first until the last.
 */
static void *t_xing(void *arg)
{
    xing_t  *x = (xing_t *)arg;
    int      changed, coming, want_down, state, go, t;
    uint8_t  busy;
    uint64_t now, lower, due;

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
                pthread_mutex_unlock(&x->lk);
                report_fault(x);
                publish(x, 1);
                continue;
            }
        } else {
            coming    = 0;
            want_down = 0;
            busy      = 0;
            for (t = 0; t < 2; t++) {
                if (x->detect_ns[t] == 0) {
                    continue;
                }
                if (now >= x->detect_ns[t] + rts_ns(T_TRACK_S)) {
                    x->detect_ns[t] = 0;              /* it has left */
                    changed         = 1;
                    continue;
                }
                coming = 1;
                lower  = x->detect_ns[t] + rts_ns(T_WARNING_S);
                due    = lower + rts_ns(T_GATE_MOVE_S);
                if (now >= lower) {
                    want_down = 1;
                }
                /* On the crossing once it is due and the gates are down.
                   A gate that is late only shortens what the panel shows. */
                if (now >= due && x->gate_pos == GATE_DOWN) {
                    if (!(x->track_busy & (1u << t))) {
                        x->trains_served++;           /* it has just arrived */
                    }
                    busy |= (uint8_t)(1u << t);
                }
            }
            if (busy != x->track_busy) {
                x->track_busy = busy;
                changed       = 1;
            }

            /* Down once any train is inside its gate time, and kept down
               while another one is still on its way, so two trains close
               together do not make the gates bounce. */
            if (want_down || (coming && x->gate_target == GATE_DOWN)) {
                gate_command(x, GATE_DOWN);
            } else if (!coming) {
                gate_command(x, GATE_UP);
            }

            if (coming) {
                state = (x->gate_pos == GATE_DOWN) ? XS_CLOSED : XS_WARNING;
            } else {
                state = (x->gate_pos == GATE_UP) ? XS_CLEAR : XS_CLOSED;
            }
            /* A cleared fault also lands here, with the signal still red.
               While the control room has the trains stopped every signal
               stays red. */
            go = !g_line_stopped;
            if (state != x->state || x->train_signal != go) {
                x->state        = state;
                x->train_signal = go;
                changed         = 1;
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

/*
 * The k-th crossing on the train's route detects it. The time given is
 * when the train was due at that detector, not when this thread got
 * round to it, so the gaps between the crossings stay exact.
 * Caller holds g_train_lk.
 */
static void detect(const train_t *tr, int k, uint64_t at_ns)
{
    int     id = route[tr->dir][k];
    xing_t *x  = &g_x[id - 1];

    pthread_mutex_lock(&x->lk);
    x->detect_ns[tr->dir] = at_ns;
    pthread_mutex_unlock(&x->lk);

    rts_log("train %u %s detected at X%d", tr->no, dir_name[tr->dir], id);
    event(A_CYAN, "X%d detects train %u %s on track %c", id, tr->no,
          dir_name[tr->dir], 'A' + tr->dir);
}

/*
 * Let every crossing the train has reached by now detect it. A train
 * keeps to its time even past a red signal: this models the detectors,
 * not the driver. Caller holds g_train_lk.
 */
static void train_advance(train_t *tr, uint64_t now)
{
    uint64_t at;

    while (tr->next < RTS_N_CROSSINGS) {
        at = tr->enter_ns;
        if (tr->next > 0) {
            at += rts_ns(route_s[tr->dir][tr->next]);
        }
        if (now < at) {
            return;
        }
        detect(tr, tr->next, at);
        tr->next++;
    }
    tr->active = 0;       /* the last crossing has it: it is off our line */
}

/*
 * Put a new train on the line, at the end its direction starts from.
 *
 * Refused while the train before it in the same direction still holds
 * that track at the first crossing. Once that one has gone they stay at
 * least that far apart all the way down the line, because every train
 * takes the same time between crossings.
 *
 * who is "operator" or "timetable". Returns 0 or -1, and -1 whenever
 * the control room has the trains stopped.
 * Caller holds g_train_lk.
 */
static int train_enter(int dir, const char *who)
{
    int      entry = route[dir][0];
    xing_t  *x     = &g_x[entry - 1];
    uint64_t now   = rts_now_ns();
    train_t *tr    = NULL;
    int      held, i;

    if (g_line_stopped) {
        rts_log("train %s from X%d refused (%s): line stopped", dir_name[dir],
                entry, who);
        event(A_AMBER, "X%d train %s refused, line stopped", entry,
              dir_name[dir]);
        return -1;
    }
    pthread_mutex_lock(&x->lk);
    held = x->detect_ns[dir] != 0 &&
           now < x->detect_ns[dir] + rts_ns(T_TRACK_S);
    pthread_mutex_unlock(&x->lk);

    for (i = 0; i < N_TRAINS && tr == NULL; i++) {
        if (!g_train[i].active) {
            tr = &g_train[i];
        }
    }
    if (held || tr == NULL) {
        rts_log("train %s from X%d refused (%s): %s", dir_name[dir], entry,
                who, held ? "track still occupied" : "too many trains");
        event(A_AMBER, "X%d train %s refused, track %c still busy", entry,
              dir_name[dir], 'A' + dir);
        return -1;
    }

    tr->active   = 1;
    tr->dir      = dir;
    tr->next     = 0;
    tr->no       = ++g_train_no;
    tr->enter_ns = now;

    rts_log("train %u %s enters at X%d on track %c (%s)", tr->no,
            dir_name[dir], entry, 'A' + dir, who);
    event(A_CYAN, "train %u %s enters at X%d (%s)", tr->no, dir_name[dir],
          entry, who);
    train_advance(tr, now);
    return 0;
}

/* A new timetable starts its own clock. Caller holds g_train_lk. */
static void timetable_set(int tt)
{
    g_timetable = tt;
    g_next_ns   = rts_now_ns() + rts_ns(tt_interval_s(tt));
}

/*
 * The control room stops every train, or lets them run again.
 *
 * Stopped, every crossing shows the trains a red signal, a train on the
 * line waits short of its next detector and no new train enters. A train
 * a crossing has already detected is inside braking distance, so it runs
 * on over that crossing.
 *
 * Let go, every train and the timetable carry on from where they were:
 * the time spent stopped is added to each, so the gaps between the
 * crossings stay exact. Caller holds g_train_lk.
 */
static void line_stop(int stop)
{
    uint64_t now = rts_now_ns();
    uint64_t lost;
    int      i;

    if (stop == g_line_stopped) {
        return;
    }
    if (stop) {
        g_stop_ns = now;
    } else {
        lost = now - g_stop_ns;
        for (i = 0; i < N_TRAINS; i++) {
            if (g_train[i].active) {
                g_train[i].enter_ns += lost;
            }
        }
        g_next_ns += lost;
    }
    g_line_stopped = stop;
    rts_log("control room %s every train", stop ? "stopped" : "released");
    event(stop ? A_RED : A_GREEN, "%s", stop
          ? "LINE STOPPED by the control room"
          : "line released by the control room");
}

static void *t_train(void *arg)
{
    uint64_t now;
    int      i;
    (void)arg;

    while (rts_running) {
        rts_sleep_ms(TICK_MS);

        pthread_mutex_lock(&g_train_lk);
        now = rts_now_ns();

        /* While the control room has the trains stopped nothing moves on
           and nothing new enters; line_stop() makes up the time after. */
        for (i = 0; i < N_TRAINS && !g_line_stopped; i++) {
            if (g_train[i].active) {
                train_advance(&g_train[i], now);
            }
        }

        if (!g_line_stopped && g_timetable != TT_NONE && now >= g_next_ns) {
            /* The direction only turns round for a train that actually
               left, so the timetable always alternates NS, SN, NS ... */
            if (train_enter(g_next_dir, "timetable") == 0) {
                g_next_dir = (g_next_dir == DIR_NS) ? DIR_SN : DIR_NS;
            }
            g_next_ns = now + rts_ns(tt_interval_s(g_timetable));
        }
        pthread_mutex_unlock(&g_train_lk);
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

        if (rcv.msg.hdr.type == MSG_RAIL_CMD) {
            int id = rcv.msg.u.rail_cmd.xing_id;

            switch (rcv.msg.u.rail_cmd.action) {
            case RC_STOP_TRAINS:
            case RC_RESUME_TRAINS:
                pthread_mutex_lock(&g_train_lk);
                line_stop(rcv.msg.u.rail_cmd.action == RC_STOP_TRAINS);
                pthread_mutex_unlock(&g_train_lk);
                break;
            case RC_FAULT_ACK:
                if (id < 1 || id > RTS_N_CROSSINGS) {
                    rep.result = -1;
                    break;
                }
                rts_log("X%d gate fault acknowledged by the control room", id);
                event(A_WHITE, "X%d fault acknowledged by the control room",
                      id);
                break;
            default:
                rep.result = -1;
                break;
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
    int                ev_count, shown, first, sel, i, tt, next_dir, stopped;
    uint64_t           next_ns, now;
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
        pthread_mutex_lock(&g_train_lk);
        tt       = g_timetable;
        next_ns  = g_next_ns;
        next_dir = g_next_dir;
        stopped  = g_line_stopped;
        pthread_mutex_unlock(&g_train_lk);
        pthread_mutex_lock(&g_ev_lk);
        memcpy(ev, g_ev, sizeof(ev));
        ev_count = g_ev_count;
        pthread_mutex_unlock(&g_ev_lk);
        sel = g_selected;
        now = rts_now_ns();

        rts_frame_begin(&f);
        snprintf(right, sizeof(right), "node %s  speed %.1fx  trains %s ",
                 rts_hostname(), rts_speed(), tt_name(tt));
        rts_frame_bar(&f, A_BG_BLUE, " RAILWAY CONTROLLER", right, PANEL_W);
        rts_frame_line(&f, "");

        /* The heading uses the same widths as the rows below it. */
        rts_frame_line(&f, "  %s%-3s  %-7s  %-6s  %-6s  %-6s  %-5s  %-6s",
                       C(A_HEAD), "X", "STATE", "GATES", "TRACKS", "SIGNAL",
                       "FAULT", "TRAINS");
        for (i = 0; i < RTS_N_CROSSINGS; i++) {
            const xing_status_t *x  = &st[i];
            int                  me = (i + 1 == sel);

            snprintf(id, sizeof(id), "X%d", x->xing_id);
            rts_frame_line(&f,
                " %s%s%s%s%-3s%s  %s%-7s%s  %s%-6s%s  %s     "
                "%s%-6s%s  %s%-5s%s  %u",
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
                x->trains_served);
        }
        rts_frame_add(&f, " %s", C(A_GREY));
        for (i = 0; i < RTS_N_CROSSINGS; i++) {
            rts_frame_add(&f, " X%d serves I%d I%d  ",
                          g_x[i].id, g_x[i].inter_a, g_x[i].inter_b);
        }
        rts_frame_eol(&f);

        /* what the timetable does next, in design seconds */
        if (stopped) {
            rts_frame_line(&f, "  %sTRAINS%s  %sLINE STOPPED by the control room",
                           C(A_HEAD), C(A_RESET), C(A_RED));
        } else if (tt == TT_NONE) {
            rts_frame_line(&f, "  %sTRAINS%s  no timetable (late night)",
                           C(A_HEAD), C(A_RESET));
        } else {
            rts_frame_line(&f, "  %sTRAINS%s  %s, one every %.0f min, next %s in %.0f s",
                           C(A_HEAD), C(A_RESET),
                           tt == TT_RUSH ? "rush hour" : "off peak",
                           tt_interval_s(tt) / 60.0, dir_name[next_dir],
                           next_ns > now
                               ? (double)(next_ns - now) / 1e9 * rts_speed()
                               : 0.0);
        }
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
        rts_frame_keys(&f, 10, "TRAIN", "a", "X1 > X3 track A",
                       "b", "X3 > X1 track B", NULL);
        rts_frame_keys(&f, 10, "TIMETABLE", "n", "none", "r", "rush hour",
                       "o", "off peak", NULL);
        rts_frame_keys(&f, 10, "GATE", "f", "inject fault", "c", "clear fault",
                       NULL);
        rts_frame_keys(&f, 10, "SYSTEM", "q", "quit", NULL);
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
    int     ch, tt;
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
            g_selected = ch - '0';            /* for the gate keys */
            break;
        case 'a':
        case 'b':
            /* A train enters at an end of the line - never at X2 -
               whichever crossing is selected. */
            pthread_mutex_lock(&g_train_lk);
            train_enter(ch == 'a' ? DIR_NS : DIR_SN, "operator");
            pthread_mutex_unlock(&g_train_lk);
            break;
        case 'n': case 'r': case 'o':
            tt = (ch == 'n') ? TT_NONE : (ch == 'r') ? TT_RUSH : TT_OFFPEAK;
            pthread_mutex_lock(&g_train_lk);
            timetable_set(tt);
            pthread_mutex_unlock(&g_train_lk);
            rts_log("operator set the timetable to %s", tt_name(tt));
            if (tt == TT_NONE) {
                event(A_WHITE, "timetable NONE: no trains");
            } else {
                event(A_WHITE, "timetable %s: a train every %.0f min",
                      tt_name(tt), tt_interval_s(tt) / 60.0);
            }
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
            pthread_mutex_unlock(&x->lk);
            rts_log("operator cleared the fault on X%d", g_selected);
            event(A_GREEN, "X%d fault cleared (operator)", g_selected);
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
    rts_mutex_init(&g_train_lk);

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

    /* The first timetabled train comes after half an interval, so a
       demonstration does not open with two silent minutes. */
    g_next_ns = rts_now_ns() + rts_ns(tt_interval_s(g_timetable) / 2.0);

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
