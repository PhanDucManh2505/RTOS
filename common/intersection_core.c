#include "intersection_core.h"
#include "rts_names.h"
#include "rts_timing.h"
#include "rts_util.h"
#include "rts_safety.h"
#include "rts_color.h"
#include "rts_log.h"
#include "rts_shm.h"

#include <errno.h>
#include <pthread.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/dispatch.h>
#include <sys/neutrino.h>

#define PRIO_PREEMPT 21
#define PRIO_PHASE   15
#define PRIO_SRV     12
#define PRIO_REPORT  10

/* Chance, per lane per sensor sample, that a vehicle pulls up to a red
   light. This stands in for real loop detectors; see t_phase(). */
#define ARRIVE_PCT   10

/* ------------------------------------------------------------------ */
/* All state shared by the threads of one controller.                  */
/* ------------------------------------------------------------------ */
typedef struct {
    const inter_cfg_t *cfg;

    pthread_mutex_t    lk;        /* priority inheritance, see rts_mutex_init */
    pthread_cond_t     report_cv; /* wakes t_report when something changed    */

    /* lamps: the output of the whole system */
    uint8_t  veh[MV_COUNT];
    uint8_t  ped[PD_COUNT];

    /* light state machine */
    int      phase;
    int      state;
    uint8_t  green_mask;  /* movements a train held back during this green */
    uint64_t state_end_ns;
    uint32_t cycle_count;
    uint64_t cycle_start_ns;

    /* pedestrians. The lamps are not kept here: a crossing always shows
       whatever the traffic beside it shows, so it is derived from the
       phase. This is only the set of buttons still asking for a phase. */
    uint8_t  ped_req;

    /* pattern in force */
    int      pattern;
    uint16_t green_s[PH_COUNT];
    double   offset_s;

    /* override from the control room */
    int      ov_active;
    int      ov_phase;
    uint64_t ov_until_ns;

    /* railway */
    int      xing_state;      /* last state read from the railway node */
    uint64_t xing_last_ns;    /* when we last heard from it            */
    int      xing_seen;       /* 1 once the railway has ever answered  */
    int      hold_rail;       /* 1 = phases C and D are shut down      */
    int      pre_pending;     /* 1 = pre-emption asked for, not done   */
    int      pre_done;        /* 1 = the clearing green has been given */

    /* One loop detector per phase, and it says one thing: someone is
       waiting on it, or nobody is. Used by the sensor driven pattern. */
    uint8_t  car[PH_COUNT];

    /* link to the control room */
    int      central_online;
    uint64_t last_hb_ns;

    /* reporting */
    int      status_dirty;
    uint32_t lamp_changes;

    rts_chan_t      phase_ch;   /* private channel owned by t_phase */
    rts_corridor_t *shm;
} ictl_t;

static ictl_t G;

/* ------------------------------------------------------------------ */
/* helpers                                                             */
/* ------------------------------------------------------------------ */

static uint16_t default_green(int phase)
{
    switch (phase) {
        case PH_A: return (uint16_t)G_R1_S;
        case PH_B: return (uint16_t)G_RT1_S;
        case PH_C: return (uint16_t)G_R3_S;
        default:   return (uint16_t)G_RT3_S;
    }
}

/* A crossing that reports FAULT, or that we cannot read at all, is
   treated exactly like a crossing with a train on it. */
static int xing_wants_hold(const ictl_t *s)
{
    return s->xing_state == XS_WARNING ||
           s->xing_state == XS_CLOSED  ||
           s->xing_state == XS_FAULT;
}

/*
 * The movements a train shuts down: everything that ends in the arm the
 * tracks cross, so no vehicle can drive towards the gates. Everything
 * else keeps running, including the movements that come out of that arm
 * and empty the road in front of the crossing.
 *
 * The mask is in force from the moment the crossing announces a train
 * until it reports itself clear again.
 */
static uint8_t rail_block(const ictl_t *s)
{
    if (!(s->hold_rail || s->pre_pending || s->pre_done ||
          xing_wants_hold(s))) {
        return 0;
    }
    return rts_rail_block_mask(s->cfg->rail_arm);
}

/*
 * The crossings a phase serves, as a bitmask.
 *
 * Phase A runs NS and SN, so the E and W crossings walk beside them;
 * phase C runs EW and WE, so N and S walk. The right turn phases B and
 * D serve none, because a turning vehicle would drive straight across
 * the crossing. rts_ped_phase() is the one place that mapping lives, so
 * it is read here rather than copied.
 */
static uint8_t ped_mask(int phase)
{
    uint8_t m = 0;
    int     i;

    for (i = 0; i < PD_COUNT; i++) {
        if (rts_ped_phase(i) == phase) {
            m |= (uint8_t)(1u << i);
        }
    }
    return m;
}

/* Is a button still waiting for a crossing this phase serves? */
static int ped_req_for(const ictl_t *s, int phase)
{
    return (s->ped_req & ped_mask(phase)) != 0;
}

/*
 * Show the crossings of one phase, and only those, in 'colour'.
 *
 * A crossing is the other face of its phase: it walks while the traffic
 * beside it is green and flashes while that traffic is amber, in every
 * pattern. There is no pedestrian timer and no pedestrian sub-machine -
 * the phase is the only clock either of them needs.
 */
static void ped_set(ictl_t *s, int phase, uint8_t colour)
{
    uint8_t m = ped_mask(phase);
    int     i;

    for (i = 0; i < PD_COUNT; i++) {
        s->ped[i] = (m & (1u << i)) ? colour : (uint8_t)PED_DONT;
    }
}

/*
 * Amber: a crossing that was walking now flashes, everything else stays
 * dark. It is written this way rather than by phase because the railway
 * clearing green leaves the phase set to C without ever having released
 * the N and S crossings, and they must not start flashing on the way
 * out of a green they were never part of.
 */
static void ped_flash(ictl_t *s)
{
    int i;

    for (i = 0; i < PD_COUNT; i++) {
        s->ped[i] = (s->ped[i] == PED_WALK) ? (uint8_t)PED_FLASH
                                            : (uint8_t)PED_DONT;
    }
}

/*
 * Print one line, "[I1]   123.45s  " and then the message, with a
 * single write(). The six controllers share one terminal on VM2 and
 * stdout is unbuffered, so a line printed in pieces could be cut into by
 * another controller's line. One write per line keeps every line whole.
 */
static void say(const char *label, const char *color, const char *fmt, ...)
{
    char    line[768];
    size_t  n;
    va_list ap;

    n = rts_append(line, sizeof(line), 0, "%s[%s]%s %8.2fs  %s",
                   C(A_CYAN), label, C(A_RESET), rts_uptime_s(), C(color));
    va_start(ap, fmt);
    n = rts_vappend(line, sizeof(line), n, fmt, ap);
    va_end(ap);
    /* keep room for the ending, so even a cut line resets and ends */
    if (n > sizeof(line) - 8) {
        n = sizeof(line) - 8;
    }
    n = rts_append(line, sizeof(line), n, "%s\n", C(A_RESET));
    rts_out(line, n);
}

/* Print one coloured line whenever the lamps change. Every field has a
   fixed width, so the lines of all six controllers line up. */
static void print_lamps(const ictl_t *s, const char *note)
{
    char    lamps[256];
    char    peds[128];
    uint8_t xs = (uint8_t)s->xing_state;

    say(s->cfg->label, "",
        "%sph%s %s%s%s  %s%-7s%s   %s   %sped%s %s   %sX%d%s %s%-7s%s  "
        "%s%s%s%s%s",
        C(A_GREY), C(A_RESET),
        C(A_BOLD), rts_phase_name((uint8_t)s->phase), C(A_RESET),
        rts_state_color((uint8_t)s->state),
        rts_state_name((uint8_t)s->state), C(A_RESET),
        rts_lamps_str(lamps, sizeof(lamps), s->veh),
        C(A_GREY), C(A_RESET), rts_peds_str(peds, sizeof(peds), s->ped),
        C(A_GREY), s->cfg->xing_id, C(A_RESET),
        rts_xing_color(xs), rts_xing_name(xs), C(A_RESET),
        /* the badge column keeps its width when a note follows */
        s->hold_rail ? C(A_BG_RED) : "",
        s->hold_rail ? " RAIL HOLD " : (note != NULL ? "           " : ""),
        s->hold_rail ? C(A_RESET) : "",
        note != NULL ? "  " : "",
        note != NULL ? note : "");
}

/*
 * Commit a set of lamp values. Everything that changes a light goes
 * through here, so the safety check cannot be bypassed.
 * The caller must already hold s->lk.
 */
static void lamps_commit(ictl_t *s, const char *note)
{
    uint8_t why = REJ_NONE;

    if (!rts_lamps_safe(s->veh, s->ped, rail_block(s), &why)) {
        /* This must never happen. If it does, the safe thing is to show
           nothing but red and say so loudly. */
        rts_lamps_all_red(s->veh, s->ped);
        rts_log("SAFETY VIOLATION blocked (%s) - forced all red",
                rts_reject_name(why));
        say(s->cfg->label, A_BG_RED,
            " SAFETY VIOLATION blocked (%s) - forced all red ",
            rts_reject_name(why));
    }

    s->lamp_changes++;
    s->status_dirty = 1;
    pthread_cond_signal(&s->report_cv);

    rts_slot_write(s->shm, s->cfg->id, (uint8_t)s->phase, (uint8_t)s->state,
                   (uint8_t)s->hold_rail, (uint8_t)s->pattern,
                   s->cycle_count, s->cycle_start_ns);

    /* mv= is every vehicle lamp in the order they are printed on the
       screen, NS SN NW SE EW WE WS EN, so a log line says exactly which
       movements were released and which the train held back. */
    rts_log("%s ph=%s st=%s NS=%d EW=%d ped=%d%d%d%d hold=%d x=%s "
            "mv=%d%d%d%d%d%d%d%d",
            s->cfg->label, rts_phase_name((uint8_t)s->phase),
            rts_state_name((uint8_t)s->state),
            s->veh[MV_NS], s->veh[MV_EW],
            s->ped[PD_N], s->ped[PD_S], s->ped[PD_E], s->ped[PD_W],
            s->hold_rail, rts_xing_name((uint8_t)s->xing_state),
            s->veh[MV_NS], s->veh[MV_SN], s->veh[MV_NW], s->veh[MV_SE],
            s->veh[MV_EW], s->veh[MV_WE], s->veh[MV_WS], s->veh[MV_EN]);

    print_lamps(s, note);
}

/* Fill a status message from the current state. Caller holds s->lk. */
static void fill_status(const ictl_t *s, inter_status_t *st)
{
    int i;
    memset(st, 0, sizeof(*st));
    st->inter_id       = (uint8_t)s->cfg->id;
    st->phase          = (uint8_t)s->phase;
    st->state          = (uint8_t)s->state;
    st->pattern        = (uint8_t)s->pattern;
    st->ped_req        = s->ped_req;
    st->train_hold     = (uint8_t)s->hold_rail;
    st->xing_state     = (uint8_t)s->xing_state;
    st->central_online = (uint8_t)s->central_online;
    st->override_active= (uint8_t)s->ov_active;
    st->cycle_count    = s->cycle_count;
    memcpy(st->veh, s->veh, MV_COUNT);
    memcpy(st->ped, s->ped, PD_COUNT);
    for (i = 0; i < PH_COUNT; i++) {
        st->green_s[i] = s->green_s[i];
    }
}

/* ------------------------------------------------------------------ */
/* the light state machine                                             */
/* ------------------------------------------------------------------ */

static timer_t tmr_phase;
static timer_t tmr_sensor;

static void fsm_enter(ictl_t *s, int newstate, const char *note);
static void fsm_enter_at(ictl_t *s, int newstate, const char *note,
                         uint64_t base);

/*
 * How long the green that is about to start must run.
 *
 * FIXED and UPDATED : the programmed time, and the phase ends there.
 *
 * SENSOR : there is no programmed length at all. The green holds until
 *          another lane asks for it, so this is only the shortest it may
 *          be before such a request can be acted on - without it the
 *          lights would flip on every sensor sample. A pedestrian who
 *          asked for this phase gets the whole crossing time instead.
 */
static double green_seconds(ictl_t *s, int phase)
{
    if (s->pattern == PAT_SENSOR) {
        return ped_req_for(s, phase) ? T_PED_TOTAL_S : G_SENSOR_MIN_S;
    }
    return (double)s->green_s[phase];
}

/* Is an override in force for the phase that is green right now? */
static int override_holds(const ictl_t *s)
{
    return s->ov_active && rts_now_ns() < s->ov_until_ns &&
           s->ov_phase == s->phase && !s->pre_pending &&
           (rts_phase_mask(s->phase) & ~rail_block(s)) != 0;
}

/* Ask t_report to send the full state now, not at the next lamp change. */
static void status_now(ictl_t *s)
{
    s->status_dirty = 1;
    pthread_cond_signal(&s->report_cv);
}

/*
 * Off peak the lights do not run a cycle at all: they sit where they
 * are until something asks them to move. This answers "who is asking",
 * or -1 for "nobody, stay put".
 *
 * A pedestrian asks outright and is granted the phase straight away,
 * because a person waiting at a kerb is not detected again later and
 * would otherwise wait for a gap that never comes.
 *
 * A vehicle is only granted its request once the lane that is green has
 * emptied, so a single car at a side road cannot cut off a road that
 * still has traffic on it. That is the whole rule.
 *
 * Phases are scanned starting from the one after the current phase, so
 * when two lanes ask at once they are still served in A B C D order.
 */
static int sensor_want_phase(const ictl_t *s, uint8_t blocked)
{
    int i, p;

    for (i = 1; i < PH_COUNT; i++) {
        p = (s->phase + i) % PH_COUNT;
        if ((rts_phase_mask(p) & ~blocked) == 0) continue;
        if (s->ped_req & ped_mask(p))            return p;
    }

    if (s->car[s->phase]) {
        return -1;            /* our own lane still has someone on it */
    }

    for (i = 1; i < PH_COUNT; i++) {
        p = (s->phase + i) % PH_COUNT;
        if ((rts_phase_mask(p) & ~blocked) == 0) continue;
        if (s->car[p])                           return p;
    }
    return -1;
}

/*
 * Pick the phase to run next.
 *
 * FIXED and UPDATED run the ring A -> B -> C -> D -> A. A phase is
 * skipped when a train has taken every movement it owns, never
 * reordered, so a driver always sees the same sequence of movements.
 *
 * SENSOR does not run a ring: it goes straight to whichever phase asked
 * for the green, and stays where it is when nothing did.
 */
static int next_phase(ictl_t *s)
{
    uint8_t blocked = rail_block(s);
    int     p = s->phase;
    int     tried;

    /* An override outranks the pattern but not railway pre-emption. */
    if (s->ov_active && rts_now_ns() < s->ov_until_ns) {
        if ((rts_phase_mask(s->ov_phase) & ~blocked) != 0) {
            return s->ov_phase;
        }
    }

    if (s->pattern == PAT_SENSOR) {
        p = sensor_want_phase(s, blocked);
        if (p >= 0) {
            return p;
        }
        /* Nobody asked, so stay where we are - unless a train has taken
           every movement this phase owns, in which case fall through to
           the ring below and find one that still has something to show. */
        if ((rts_phase_mask(s->phase) & ~blocked) != 0) {
            return s->phase;
        }
    }

    for (tried = 0; tried < PH_COUNT; tried++) {
        p = (p + 1) % PH_COUNT;

        /* A train is coming or the crossing cannot be read: run the
           phase only if it still has a movement that does not drive
           towards the gates. */
        if ((rts_phase_mask(p) & ~blocked) == 0) {
            continue;
        }
        return p;
    }

    /* Everything was skipped. Fall back to the first phase that still
       has a movement left; phase A never touches the rail-side arm. */
    for (tried = 0; tried < PH_COUNT; tried++) {
        if ((rts_phase_mask(tried) & ~blocked) != 0) {
            return tried;
        }
    }
    return PH_A;
}

/*
 * The green wave. At the start of each cycle we look at when our
 * partner across the crossing started its own cycle and bend this
 * cycle by at most a few seconds to hold the designed offset.
 */
static double offset_correction(ictl_t *s)
{
    rts_slot_t pair;
    double     cycle_ms, want_ms, have_ms, err_ms, fix_s;

    if (s->pattern == PAT_SENSOR || s->cfg->pair_id <= 0) {
        return 0.0;   /* only meaningful when both run a fixed cycle */
    }
    /* The lower-numbered controller of a pair leads and keeps its
       programmed timing; only its partner bends. When both bent, each
       chased the other and the pair never settled: phase A ran 17 or
       23 s instead of 20 s and the offset never reached 12 s. */
    if (s->cfg->id < s->cfg->pair_id) {
        return 0.0;
    }
    if (rts_slot_read(s->shm, s->cfg->pair_id, &pair) != 0) {
        return 0.0;   /* partner not running, nothing to align to */
    }
    /* Only line up with a partner running the same cycle, and never
       while a train is holding either side. */
    if (pair.pattern != (uint8_t)s->pattern || pair.train_hold || s->hold_rail) {
        return 0.0;
    }

    cycle_ms = (double)rts_ms(T_CYCLE_S);
    want_ms  = (double)rts_ms(s->offset_s);
    have_ms  = (double)((int64_t)s->cycle_start_ns -
                        (int64_t)pair.cycle_start_ns) / 1e6;

    /* Fold into one cycle so the answer is the shortest correction. */
    while (have_ms < 0)         have_ms += cycle_ms;
    while (have_ms >= cycle_ms) have_ms -= cycle_ms;

    err_ms = want_ms - have_ms;
    if (err_ms >  cycle_ms / 2.0) err_ms -= cycle_ms;
    if (err_ms < -cycle_ms / 2.0) err_ms += cycle_ms;

    fix_s = (err_ms / 1000.0) * rts_speed();
    if (fix_s >  T_OFFSET_FIX_MAX_S) fix_s =  T_OFFSET_FIX_MAX_S;
    if (fix_s < -T_OFFSET_FIX_MAX_S) fix_s = -T_OFFSET_FIX_MAX_S;
    return fix_s;
}

/*
 * Enter a new state: set the lamps, arm the timer, publish.
 *
 * base is when the state was planned to start: the planned end of the
 * state before it when a timer ended that one, or now when an event cut
 * in. An ordinary green ends on that plan, so the few milliseconds
 * every timer runs late do not add up into a cycle that drifts.
 *
 * Every interval that exists for safety - amber, all-red, start-up and
 * the clearing green - is timed from the moment the lamps actually
 * changed instead, so a late timer can only ever make one longer.
 */
static void fsm_enter_at(ictl_t *s, int newstate, const char *note,
                         uint64_t base)
{
    double   secs;
    uint64_t now     = rts_now_ns();
    uint8_t  blocked = rail_block(s);
    int      exit_mv;

    /* More than half a second behind the plan is not timer delay but
       an event (an override, or an off peak green with nobody asking
       for a change, kept the last phase), so start the plan from now. */
    if (now > base + rts_ns(0.5)) {
        base = now;
    }
    s->state = newstate;

    switch (newstate) {
    case ST_STARTUP:
        rts_lamps_all_red(s->veh, s->ped);
        secs = T_ALLRED_S;
        break;

    case ST_GREEN:
        /* The movements that drive into the rail-side arm stay red for
           the whole phase; the rest of the phase runs as normal. */
        s->green_mask = blocked;
        rts_lamps_phase_masked(s->veh, s->phase, LAMP_GREEN, blocked);
        secs = green_seconds(s, s->phase);
        if (s->phase == PH_A) {
            secs += offset_correction(s);
            if (secs < G_MIN_S) secs = G_MIN_S;
        }
        /* The crossings beside this phase walk with it, in every
           pattern. Anyone who pressed a button for one of them has now
           got what they asked for, so the request is cleared here. */
        ped_set(s, s->phase, PED_WALK);
        s->ped_req &= (uint8_t)~ped_mask(s->phase);
        break;

    case ST_AMBER:
        /*
         * Amber and all-red are fixed at compile time. No command from
         * the control room and no pre-emption may shorten them.
         *
         * The amber repeats the mask the green ran with, not the one in
         * force now: a movement that was never released must not flash
         * amber on its way to red, and a movement that was green when
         * the train was announced still gets its whole 4 s.
         */
        rts_lamps_phase_masked(s->veh, s->phase, LAMP_AMBER, s->green_mask);
        ped_flash(s);
        secs = T_AMBER_S;
        break;

    case ST_ALLRED:
        rts_lamps_all_red(s->veh, s->ped);
        secs = T_ALLRED_S;
        break;

    case ST_PRE_CLEAR:
        /* Railway pre-emption: a green long enough to empty the 50 m
           between the gates and the stop line. Only the movement that
           comes out of the rail-side arm is released - the one driving
           towards the gates would be filling the road we are trying to
           empty. The crossings stay dark through it. */
        s->phase = PH_C;
        exit_mv       = rts_rail_exit_mv(s->cfg->rail_arm);
        /* Geometry not configured: 0 falls back to the whole road. */
        s->green_mask = (exit_mv >= 0) ? (uint8_t)~(1u << exit_mv) : 0;
        rts_lamps_phase_masked(s->veh, s->phase, LAMP_GREEN,
                               s->green_mask);
        memset(s->ped, PED_DONT, PD_COUNT);
        secs = (s->pattern == PAT_FIXED) ? T_RAIL_CLEAR_PEAK_S
                                         : T_RAIL_CLEAR_OFF_S;
        break;

    default:
        rts_lamps_all_red(s->veh, s->ped);
        secs = T_ALLRED_S;
        break;
    }

    /* state_end_ns stays on the plan whichever way this state is timed,
       so the state after it still starts where the plan said it would
       and only the ordinary greens absorb the delay. */
    s->state_end_ns = base + rts_ns(secs);
    if (newstate == ST_GREEN) {
        rts_timer_at(tmr_phase, s->state_end_ns);
    } else {
        rts_timer_at(tmr_phase, now + rts_ns(secs));
    }
    lamps_commit(s, note);
}

/* An event changed the state: time it from now. */
static void fsm_enter(ictl_t *s, int newstate, const char *note)
{
    fsm_enter_at(s, newstate, note, rts_now_ns());
}

/* Keep the current green one more second, then look again. */
static void extend_green(ictl_t *s)
{
    s->state_end_ns = rts_now_ns() + rts_ns(1.0);
    rts_timer_at(tmr_phase, s->state_end_ns);
}

/* A phase timer expired: decide what comes next. Every state entered
   from here starts where the plan said the last one would end. */
static void fsm_advance(ictl_t *s)
{
    char     note[64];
    uint64_t plan = s->state_end_ns;

    switch (s->state) {
    case ST_STARTUP:
        s->phase          = PH_A;
        s->cycle_start_ns = plan;
        fsm_enter_at(s, ST_GREEN, "cycle start", plan);
        break;

    case ST_GREEN:
        /* An override holds its phase green until it expires or is
           cancelled; look again every second. A train still wins:
           fsm_reevaluate() cuts this green as soon as one is coming. */
        if (override_holds(s)) {
            extend_green(s);
            return;
        }
        /* Off peak the minimum is up, but the phase does not end on a
           clock: it holds until another lane asks for it. Look again in
           a second, and again after that, for as long as it takes. */
        if (s->pattern == PAT_SENSOR &&
            sensor_want_phase(s, rail_block(s)) < 0) {
            extend_green(s);
            return;
        }
        fsm_enter_at(s, ST_AMBER, NULL, plan);
        break;

    case ST_AMBER:
        fsm_enter_at(s, ST_ALLRED, NULL, plan);
        break;

    case ST_PRE_CLEAR:
        s->pre_done = 1;
        fsm_enter_at(s, ST_AMBER, "tracks cleared", plan);
        break;

    case ST_ALLRED:
        /* This is the only moment at which anything may change: no
           vehicle is inside the intersection. */
        if (s->pre_pending && !s->pre_done) {
            /* pre_pending stays set until the hold begins. Clearing it
               here let the once-a-second re-check in fsm_reevaluate()
               ask for a second pre-emption during this clearing green,
               and that stale request ran a second clearing green after
               the train had already gone. */
            fsm_enter_at(s, ST_PRE_CLEAR, "RAIL: clearing the tracks", plan);
            return;
        }
        if (s->pre_done) {
            s->pre_done    = 0;
            s->pre_pending = 0;
            if (xing_wants_hold(s)) {
                s->hold_rail = 1;
                snprintf(note, sizeof(note), "RAIL: holding, road over tracks red");
                s->phase          = PH_A;
                s->cycle_count++;
                s->cycle_start_ns = plan;
                fsm_enter_at(s, ST_GREEN, note, plan);
                return;
            }
            /* The crossing cleared while the tracks were being emptied,
               so there is nothing to hold: carry on with the cycle. */
        }
        {
            int np = next_phase(s);
            if (np <= s->phase) {           /* wrapped: a cycle finished */
                s->cycle_count++;
                s->cycle_start_ns = plan;
            }
            s->phase = np;
            fsm_enter_at(s, ST_GREEN, NULL, plan);
        }
        break;

    default:
        fsm_enter(s, ST_STARTUP, NULL);
        break;
    }
}

/*
 * Something happened that might mean the current state should be cut
 * short: a train, an override, or the crossing becoming readable again.
 */
static void fsm_reevaluate(ictl_t *s)
{
    int want_hold = xing_wants_hold(s);

    if (want_hold && !s->hold_rail && !s->pre_pending && !s->pre_done) {
        s->pre_pending = 1;
        rts_log("%s pre-emption requested, crossing %s",
                s->cfg->label, rts_xing_name((uint8_t)s->xing_state));

        if (s->state == ST_GREEN) {
            /* Cut the green short. Amber and all-red still run in full. */
            fsm_enter(s, ST_AMBER, "RAIL: train coming, ending phase");
        }
        /* Otherwise we are already in amber or all-red, and the change
           happens at the end of it. */
        return;
    }

    if (!want_hold && s->hold_rail) {
        s->hold_rail = 0;
        rts_log("%s crossing clear, normal cycle resumed", s->cfg->label);
        lamps_commit(s, "RAIL: crossing clear, resuming");
    }

    /* An override that has just been set, or has just expired. */
    if (s->ov_active && rts_now_ns() >= s->ov_until_ns) {
        s->ov_active = 0;
        rts_log("%s override expired", s->cfg->label);
        lamps_commit(s, "override expired");
    }
}

/* ------------------------------------------------------------------ */
/* t_phase : owns the lamps                                            */
/* ------------------------------------------------------------------ */
static void *t_phase(void *arg)
{
    ictl_t   *s = (ictl_t *)arg;
    rts_rcv_t rcv;
    int       rcvid;
    int       i;

    rts_timer_new(&tmr_phase,  &s->phase_ch, PRIO_PHASE, PULSE_PHASE_TICK,  0);
    rts_timer_new(&tmr_sensor, &s->phase_ch, PRIO_PHASE, PULSE_SENSOR_TICK, 0);

    /* One sensor sample per second, exactly as the design specifies. */
    rts_timer_every(tmr_sensor, rts_ms(1.0));

    pthread_mutex_lock(&s->lk);
    fsm_enter(s, ST_STARTUP, "start up, all red");
    pthread_mutex_unlock(&s->lk);

    while (rts_running) {
        rcvid = MsgReceive(s->phase_ch.chid, &rcv, sizeof(rcv), NULL);

        if (rcvid == -1) {
            if (errno == EINTR) continue;
            break;
        }
        if (rcvid > 0) {
            /* Nobody sends real messages here, only pulses. */
            MsgError(rcvid, ENOSYS);
            continue;
        }

        pthread_mutex_lock(&s->lk);
        switch (rcv.pulse.code) {
        case PULSE_PHASE_TICK:
            fsm_advance(s);
            break;

        case PULSE_SENSOR_TICK:
            /*
             * Stand in for the loop detectors the real intersection
             * would have. Each lane reads one bit: someone is waiting
             * on it, or nobody is.
             *
             * The green lane empties first, because whatever was on the
             * loop has driven through. Then every lane, that one
             * included, may pick up a new arrival - so a road that is
             * genuinely busy still reads occupied and goes on holding
             * its own green, which is what makes the "switch only when
             * the green lane is clear" rule mean anything.
             *
             * A lane at a red light is never cleared here: the car
             * stays on the loop until its phase is given the green.
             *
             * ARRIVE_PCT is the only tunable. Set it to 0 and the
             * lights freeze where they are until someone presses a
             * pedestrian button, which is the clearest way to see that
             * the off peak green really has no clock on it.
             */
            for (i = 0; i < PH_COUNT; i++) {
                if (i == s->phase && s->state == ST_GREEN) {
                    s->car[i] = 0;
                }
                if ((rand() % 100) < ARRIVE_PCT) {
                    s->car[i] = 1;
                }
            }
            /* The link is judged by silence, not by a failed send. */
            if (s->central_online &&
                rts_now_ns() - s->last_hb_ns > rts_ns(T_OFFLINE_S)) {
                s->central_online = 0;
                rts_log("%s central controller offline, running on the "
                        "last valid pattern", s->cfg->label);
                lamps_commit(s, "CENTRAL OFFLINE - running standalone");
            }
            /*
             * The crossing must keep talking to us. Silence for longer
             * than the watchdog means we cannot read it, and an
             * unreadable crossing is treated exactly like an occupied
             * one. This also covers the case where the railway node has
             * never come up at all.
             */
            if (rts_now_ns() - s->xing_last_ns > rts_ns(T_XING_WATCHDOG_S) &&
                s->xing_state != XS_FAULT) {
                rts_log("%s crossing unreadable, treating as FAULT",
                        s->cfg->label);
                s->xing_state = XS_FAULT;
            }
            /*
             * Re-check once a second as well as on every event. Events
             * can be missed while the machine is inside amber, all-red
             * or a pedestrian clearance, so this guarantees the
             * controller converges on the right answer within 1 s.
             */
            fsm_reevaluate(s);
            break;

        case PULSE_WAKE:
        case PULSE_XING_CHANGED:
            fsm_reevaluate(s);
            break;

        default:
            break;
        }
        pthread_mutex_unlock(&s->lk);
    }
    return NULL;
}

/* ------------------------------------------------------------------ */
/* t_preempt : watches the crossing                                    */
/* ------------------------------------------------------------------ */
static void *t_preempt(void *arg)
{
    ictl_t        *s = (ictl_t *)arg;
    name_attach_t *att;
    char           svc[64];
    rts_rcv_t      rcv;
    rts_reply_t    rep;
    int            rcvid;

    rts_svc_inter_evt(svc, sizeof(svc), s->cfg->id);
    att = name_attach(NULL, svc, 0);
    if (att == NULL) {
        say(s->cfg->label, A_RED, "cannot register %s", svc);
        return NULL;
    }
    say(s->cfg->label, "", "event channel  : %s", svc);

    memset(&rep, 0, sizeof(rep));
    rep.hdr.sender = s->cfg->sender_id;

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
            continue;                    /* never reply to a pulse */
        }
        if (rts_server_housekeeping(rcvid, &rcv)) {
            continue;
        }

        if (rcv.msg.hdr.type == MSG_XING_STATE) {
            int changed;

            pthread_mutex_lock(&s->lk);
            changed          = (s->xing_state != rcv.msg.u.xing.state);
            s->xing_state    = rcv.msg.u.xing.state;
            s->xing_last_ns  = rts_now_ns();
            s->xing_seen     = 1;
            pthread_mutex_unlock(&s->lk);

            /* Reply first, act second: the railway node must never be
               held up waiting for our state machine. */
            rep.result = 0;
            rep.reason = REJ_NONE;
            MsgReply(rcvid, EOK, &rep, sizeof(rep));

            if (changed) {
                rts_log("%s crossing %d -> %s", s->cfg->label,
                        rcv.msg.u.xing.xing_id,
                        rts_xing_name(rcv.msg.u.xing.state));
                /* Tell t_phase to look again. A pulse is used because
                   it never blocks the sender. */
                MsgSendPulse(s->phase_ch.coid, PRIO_PHASE,
                             PULSE_XING_CHANGED, 0);
            }
            continue;
        }

        rep.result = -1;
        rep.reason = REJ_BAD_PHASE;
        MsgReply(rcvid, EOK, &rep, sizeof(rep));
    }

    name_detach(att, 0);
    return NULL;
}

/* ------------------------------------------------------------------ */
/* t_srv : commands from the control room                              */
/* ------------------------------------------------------------------ */

/*
 * Validate a pattern command before it is used. A command that would
 * break a safety rule is refused and the reason is reported, which is
 * what the brief asks for.
 */
static int check_pattern(const ictl_t *s, const rts_msg_t *m, uint8_t *why)
{
    int    i;
    double total = 0.0;

    (void)s;
    if (m->u.set_pattern.pattern >= PAT_COUNT) {
        *why = REJ_UNKNOWN_PATTERN;
        return -1;
    }
    for (i = 0; i < PH_COUNT; i++) {
        double g = m->u.set_pattern.green_s[i];
        if (g == 0.0) {
            g = default_green(i);        /* 0 means "keep what we have" */
        }
        if (g < G_MIN_S) { *why = REJ_MIN_GREEN; return -1; }
        if (g > G_MAX_S) { *why = REJ_MAX_GREEN; return -1; }
        total += g + T_AMBER_S + T_ALLRED_S;
    }
    if (total > T_CYCLE_MAX_S) {
        *why = REJ_CYCLE_TOO_LONG;
        return -1;
    }
    *why = REJ_NONE;
    return 0;
}

static void *t_srv(void *arg)
{
    ictl_t        *s = (ictl_t *)arg;
    name_attach_t *att;
    char           svc[64];
    rts_rcv_t      rcv;
    rts_reply_t    rep;
    int            rcvid;
    int            wake;

    rts_svc_inter(svc, sizeof(svc), s->cfg->id);
    att = name_attach(NULL, svc, 0);
    if (att == NULL) {
        say(s->cfg->label, A_RED, "cannot register %s", svc);
        return NULL;
    }
    say(s->cfg->label, "", "command channel: %s", svc);

    while (rts_running) {
        rcvid = MsgReceive(att->chid, &rcv, sizeof(rcv), NULL);
        wake  = 0;

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
        rep.hdr.sender = s->cfg->sender_id;
        rep.hdr.t_ns   = rts_now_ns();
        rep.result     = 0;
        rep.reason     = REJ_NONE;

        pthread_mutex_lock(&s->lk);

        switch (rcv.msg.hdr.type) {

        case MSG_HEARTBEAT:
            s->last_hb_ns = rts_now_ns();
            if (!s->central_online) {
                s->central_online = 1;
                rts_log("%s central controller back online, sending full "
                        "state", s->cfg->label);
                s->status_dirty = 1;      /* S4: resync the control room */
                pthread_cond_signal(&s->report_cv);
                wake = 1;
            }
            break;

        case MSG_SET_PATTERN: {
            uint8_t why;
            if (check_pattern(s, &rcv.msg, &why) != 0) {
                rep.result = -1;
                rep.reason = why;
                rts_log("%s REJECTED pattern %s: %s", s->cfg->label,
                        rts_pattern_name(rcv.msg.u.set_pattern.pattern),
                        rts_reject_name(why));
                say(s->cfg->label, A_AMBER, "REJECTED pattern command: %s",
                    rts_reject_name(why));
            } else {
                int i;
                s->pattern = rcv.msg.u.set_pattern.pattern;
                for (i = 0; i < PH_COUNT; i++) {
                    if (rcv.msg.u.set_pattern.green_s[i] != 0) {
                        s->green_s[i] = rcv.msg.u.set_pattern.green_s[i];
                    } else if (s->pattern != PAT_UPDATED) {
                        /* FIXED and SENSOR always run the programmed
                           times; without this, green times sent with an
                           earlier UPDATED pattern stayed in force. */
                        s->green_s[i] = default_green(i);
                    }
                }
                if (rcv.msg.u.set_pattern.offset_s != 0) {
                    s->offset_s = rcv.msg.u.set_pattern.offset_s;
                }
                rts_log("%s accepted pattern %s (applies at end of cycle)",
                        s->cfg->label, rts_pattern_name((uint8_t)s->pattern));
                say(s->cfg->label, A_GREEN, "accepted pattern %s",
                    rts_pattern_name((uint8_t)s->pattern));
                status_now(s);
                wake = 1;
            }
            break;
        }

        case MSG_OVERRIDE:
            if (rcv.msg.u.override_cmd.cancel) {
                s->ov_active = 0;
                rts_log("%s override cancelled", s->cfg->label);
                status_now(s);
            } else if (rcv.msg.u.override_cmd.phase >= PH_COUNT) {
                rep.result = -1;
                rep.reason = REJ_BAD_PHASE;
            } else if ((rts_phase_mask(rcv.msg.u.override_cmd.phase) &
                        ~rail_block(s)) == 0) {
                /* Railway pre-emption outranks an operator override:
                   the phase asked for has nothing left to show. A phase
                   that keeps even one movement away from the tracks is
                   accepted, and the train still holds the rest red. */
                rep.result = -1;
                rep.reason = REJ_PREEMPT_ACTIVE;
                rts_log("%s REJECTED override: pre-emption active",
                        s->cfg->label);
            } else {
                s->ov_active   = 1;
                s->ov_phase    = rcv.msg.u.override_cmd.phase;
                s->ov_until_ns = rts_now_ns() +
                                 rts_ns(rcv.msg.u.override_cmd.timeout_s);
                rts_log("%s override: hold phase %s for %u s",
                        s->cfg->label, rts_phase_name((uint8_t)s->ov_phase),
                        rcv.msg.u.override_cmd.timeout_s);
                say(s->cfg->label, A_MAGENTA, "override: hold phase %s",
                    rts_phase_name((uint8_t)s->ov_phase));
                status_now(s);
            }
            wake = 1;
            break;

        case MSG_PED_BUTTON:
            if (rcv.msg.u.button.ped_id < PD_COUNT) {
                s->ped_req |= (uint8_t)(1u << rcv.msg.u.button.ped_id);
                rts_log("%s pedestrian button %d pressed",
                        s->cfg->label, rcv.msg.u.button.ped_id);
                wake = 1;
            } else {
                rep.result = -1;
            }
            break;

        case MSG_QUERY_STATE:
            fill_status(s, &rep.status);
            break;

        default:
            rep.result = -1;
            break;
        }

        pthread_mutex_unlock(&s->lk);

        MsgReply(rcvid, EOK, &rep, sizeof(rep));

        if (wake) {
            MsgSendPulse(s->phase_ch.coid, PRIO_PHASE, PULSE_WAKE, 0);
        }
    }

    name_detach(att, 0);
    return NULL;
}

/* ------------------------------------------------------------------ */
/* t_report : tells the control room about every lamp change           */
/* ------------------------------------------------------------------ */
static void *t_report(void *arg)
{
    ictl_t        *s = (ictl_t *)arg;
    rts_link_t     link;
    rts_msg_t      m;
    inter_status_t snap;

    rts_link_init(&link, rts_node_central(), SVC_CENTRAL, "central");

    while (rts_running) {
        pthread_mutex_lock(&s->lk);
        while (!s->status_dirty && rts_running) {
            struct timespec ts;
            clock_gettime(CLOCK_REALTIME, &ts);
            ts.tv_sec += 1;               /* wake now and then to retry */
            pthread_cond_timedwait(&s->report_cv, &s->lk, &ts);
        }
        s->status_dirty = 0;
        fill_status(s, &snap);
        pthread_mutex_unlock(&s->lk);

        memset(&m, 0, sizeof(m));
        m.hdr.type   = MSG_STATUS;
        m.hdr.sender = s->cfg->sender_id;
        m.u.status   = snap;

        if (rts_link_send(&link, &m, NULL) != 0) {
            /*
             * The control room cannot be reached. That costs monitoring,
             * not control: the intersection keeps cycling and the status
             * goes to /fs instead.
             */
            rts_log("%s status to /fs (central unreachable) ph=%s st=%s",
                    s->cfg->label, rts_phase_name(snap.phase),
                    rts_state_name(snap.state));
            rts_sleep_ms(200);
        }
    }

    rts_link_close(&link);
    return NULL;
}

/* ------------------------------------------------------------------ */
/* entry point                                                         */
/* ------------------------------------------------------------------ */
int intersection_run(const inter_cfg_t *cfg, int argc, char **argv)
{
    char      procname[32];
    pthread_t th_phase, th_pre, th_srv, th_rep;
    int       i;

    snprintf(procname, sizeof(procname), "intersection_%s", cfg->label);
    rts_start(procname, argc, argv);
    rts_log_start(procname);

    memset(&G, 0, sizeof(G));
    G.cfg = cfg;
    rts_mutex_init(&G.lk);
    pthread_cond_init(&G.report_cv, NULL);

    /* Sensible defaults so the intersection is safe and useful before
       the control room has ever spoken to it. */
    G.pattern    = PAT_SENSOR;
    G.offset_s   = cfg->offset_s;
    G.xing_state = XS_FAULT;   /* until proven otherwise, assume the worst */
    G.hold_rail  = 1;          /* so the tracks are never crossed on trust */
    G.state      = ST_STARTUP;
    G.phase      = PH_A;
    for (i = 0; i < PH_COUNT; i++) {
        G.green_s[i] = default_green(i);
    }
    rts_lamps_all_red(G.veh, G.ped);
    srand((unsigned)(cfg->id * 7919));

    G.shm = rts_shm_open();
    if (G.shm == NULL) {
        say(cfg->label, A_AMBER, "shared memory unavailable, green wave "
            "disabled");
    }

    if (rts_chan_open(&G.phase_ch) != 0) {
        say(cfg->label, A_RED, "cannot create the phase channel");
        return 1;
    }

    say(cfg->label, "", "crossing %d on the %s arm, partner I%d, offset %.0f s",
        cfg->xing_id, rts_arm_name(cfg->rail_arm), cfg->pair_id, cfg->offset_s);

    rts_thread(&th_pre,   t_preempt, &G, PRIO_PREEMPT);
    rts_thread(&th_phase, t_phase,   &G, PRIO_PHASE);
    rts_thread(&th_srv,   t_srv,     &G, PRIO_SRV);
    rts_thread(&th_rep,   t_report,  &G, PRIO_REPORT);

    while (rts_running) {
        rts_sleep_ms(200);
    }

    say(cfg->label, A_AMBER, "shutting down, %u lamp changes, %u log lines "
        "dropped", G.lamp_changes, rts_log_drops());
    rts_log("=== %s stopped ===", procname);
    rts_log_stop();
    rts_chan_close(&G.phase_ch);
    rts_shm_close(G.shm);
    return 0;
}
