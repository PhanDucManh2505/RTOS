#include "rts_util.h"
#include "rts_names.h"
#include "rts_timing.h"
#include "rts_color.h"
#include "rts_log.h"

#include <errno.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sched.h>
#include <sys/dispatch.h>
#include <sys/neutrino.h>

volatile int rts_running = 1;

static uint64_t g_start_ns;
static double   g_speed = 5.0;

/* ------------------------------------------------------------------ */
/* time                                                                */
/* ------------------------------------------------------------------ */

uint64_t rts_now_ns(void)
{
    struct timespec ts;
    /* CLOCK_MONOTONIC is used everywhere instead of time(), because it
       has nanosecond resolution and never jumps when the clock is set. */
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000000ULL + (uint64_t)ts.tv_nsec;
}

void rts_sleep_ms(uint64_t ms)
{
    struct timespec ts;
    ts.tv_sec  = (time_t)(ms / 1000ULL);
    ts.tv_nsec = (long)((ms % 1000ULL) * 1000000ULL);
    nanosleep(&ts, NULL);
}

double rts_uptime_s(void)
{
    return (double)(rts_now_ns() - g_start_ns) / 1e9;
}

void rts_speed_set(double speed)
{
    if (speed < 0.1)  speed = 0.1;
    if (speed > 60.0) speed = 60.0;
    g_speed = speed;
}

double rts_speed(void)
{
    return g_speed;
}

uint64_t rts_ms(double design_seconds)
{
    double ms = (design_seconds * 1000.0) / g_speed;
    if (ms < 1.0) {
        ms = 1.0;   /* never ask for a zero length interval */
    }
    return (uint64_t)ms;
}

uint64_t rts_ns(double design_seconds)
{
    return rts_ms(design_seconds) * 1000000ULL;
}

/* ------------------------------------------------------------------ */
/* start up                                                            */
/* ------------------------------------------------------------------ */

static void on_signal(int sig)
{
    (void)sig;
    rts_running = 0;
}

void rts_install_signals(void)
{
    struct sigaction sa;
    memset(&sa, 0, sizeof(sa));
    sa.sa_handler = on_signal;
    sigaction(SIGINT,  &sa, NULL);
    sigaction(SIGTERM, &sa, NULL);

    /* The six controllers on VM2 write their screen into a FIFO that the
       panel reads. If the panel goes away the write must fail, not kill
       the controller: losing the picture must never cost the lights. */
    sa.sa_handler = SIG_IGN;
    sigaction(SIGPIPE, &sa, NULL);
}

int rts_start(const char *proc_name, int argc, char **argv)
{
    char     host[128];
    const char *env;
    int      i;
    int      first_free = 1;

    /* Unbuffered output: every line appears the moment it is printed,
       which matters when the process is killed during a demonstration. */
    setvbuf(stdout, NULL, _IONBF, 0);

    g_start_ns = rts_now_ns();

    env = getenv("RTS_SPEED");
    if (env != NULL && env[0] != '\0') {
        rts_speed_set(atof(env));
    }

    for (i = 1; i < argc; i++) {
        if (strcmp(argv[i], "-s") == 0 && i + 1 < argc) {
            rts_speed_set(atof(argv[i + 1]));
            i++;
            first_free = i + 1;
        }
    }

    memset(host, 0, sizeof(host));
    if (gethostname(host, sizeof(host) - 1) != 0) {
        strcpy(host, "unknown");
    }

    printf("%s", C(A_CLEAR));
    printf("%s============================================================%s\n",
           C(A_CYAN), C(A_RESET));
    printf("%s EEET2588 Real-Time Systems - Traffic Light Control System%s\n",
           C(A_WHITE), C(A_RESET));
    printf(" Team ANK : Phan Duc Manh S4124156, Mai Quy Anh S4118973,\n");
    printf("            Vu Minh Khanh S4117146\n");
    printf("%s------------------------------------------------------------%s\n",
           C(A_CYAN), C(A_RESET));
    printf(" process  : %s%s%s\n", C(A_BOLD), proc_name, C(A_RESET));
    printf(" pid      : %d\n", (int)getpid());
    printf(" host     : %s\n", host);
    printf(" speed    : %.1fx real time (cycle %.1f s)\n",
           g_speed, T_CYCLE_S / g_speed);
    printf(" nodes    : central=%s  intersections=%s  railway=%s\n",
           rts_node_central(), rts_node_inter(), rts_node_rail());
    printf("%s============================================================%s\n",
           C(A_CYAN), C(A_RESET));

    rts_install_signals();
    return first_free;
}

/* ------------------------------------------------------------------ */
/* threads and locks                                                   */
/* ------------------------------------------------------------------ */

int rts_thread(pthread_t *tid, void *(*fn)(void *), void *arg, int prio)
{
    pthread_attr_t     attr;
    struct sched_param sp;
    int                rc;

    pthread_attr_init(&attr);
    /* EXPLICIT_SCHED means "use the priority I am about to give you",
       not the one inherited from the thread that created me. */
    pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);
    pthread_attr_setschedpolicy(&attr, SCHED_RR);

    memset(&sp, 0, sizeof(sp));
    sp.sched_priority = prio;
    pthread_attr_setschedparam(&attr, &sp);

    rc = pthread_create(tid, &attr, fn, arg);
    if (rc != 0) {
        /* Setting a priority needs privilege. If that is the only
           problem, fall back to the default priority and carry on. */
        rc = pthread_create(tid, NULL, fn, arg);
        if (rc == 0) {
            printf("%s[warn]%s could not set priority %d, using default\n",
                   C(A_AMBER), C(A_RESET), prio);
        }
    }
    pthread_attr_destroy(&attr);
    return rc;
}

void rts_mutex_init(pthread_mutex_t *m)
{
    pthread_mutexattr_t ma;
    pthread_mutexattr_init(&ma);
    pthread_mutexattr_setprotocol(&ma, PTHREAD_PRIO_INHERIT);
    pthread_mutex_init(m, &ma);
    pthread_mutexattr_destroy(&ma);
}

/* ------------------------------------------------------------------ */
/* channels and timers                                                 */
/* ------------------------------------------------------------------ */

int rts_chan_open(rts_chan_t *c)
{
    c->chid = ChannelCreate(0);
    if (c->chid == -1) {
        return -1;
    }
    /* _NTO_SIDE_CHANNEL lets this process connect to its own channel. */
    c->coid = ConnectAttach(0, 0, c->chid, _NTO_SIDE_CHANNEL, 0);
    if (c->coid == -1) {
        ChannelDestroy(c->chid);
        c->chid = -1;
        return -1;
    }
    return 0;
}

void rts_chan_close(rts_chan_t *c)
{
    if (c->coid != -1) { ConnectDetach(c->coid); c->coid = -1; }
    if (c->chid != -1) { ChannelDestroy(c->chid); c->chid = -1; }
}

int rts_timer_new(timer_t *t, const rts_chan_t *c, int prio, int code, int value)
{
    struct sigevent ev;
    SIGEV_PULSE_INIT(&ev, c->coid, prio, code, value);
    return timer_create(CLOCK_MONOTONIC, &ev, t);
}

int rts_timer_once(timer_t t, uint64_t ms)
{
    struct itimerspec its;
    memset(&its, 0, sizeof(its));
    its.it_value.tv_sec  = (time_t)(ms / 1000ULL);
    its.it_value.tv_nsec = (long)((ms % 1000ULL) * 1000000ULL);
    /* it_interval stays zero, so the timer fires once and stops. */
    return timer_settime(t, 0, &its, NULL);
}

int rts_timer_every(timer_t t, uint64_t ms)
{
    struct itimerspec its;
    memset(&its, 0, sizeof(its));
    its.it_value.tv_sec     = (time_t)(ms / 1000ULL);
    its.it_value.tv_nsec    = (long)((ms % 1000ULL) * 1000000ULL);
    its.it_interval.tv_sec  = its.it_value.tv_sec;
    its.it_interval.tv_nsec = its.it_value.tv_nsec;
    return timer_settime(t, 0, &its, NULL);
}

/* ------------------------------------------------------------------ */
/* links                                                               */
/* ------------------------------------------------------------------ */

void rts_link_init(rts_link_t *l, const char *node, const char *svc,
                   const char *label)
{
    memset(l, 0, sizeof(*l));
    snprintf(l->node,  sizeof(l->node),  "%s", node  != NULL ? node  : "");
    snprintf(l->svc,   sizeof(l->svc),   "%s", svc   != NULL ? svc   : "");
    snprintf(l->label, sizeof(l->label), "%s", label != NULL ? label : svc);
    l->coid   = -1;
    l->online = 0;
}

static int link_connect(rts_link_t *l)
{
    char path[192];
    rts_service_path(path, sizeof(path), l->node, l->svc);
    l->coid = name_open(path, 0);
    return l->coid;
}

int rts_link_send(rts_link_t *l, rts_msg_t *m, rts_reply_t *r)
{
    uint64_t    timeout;
    rts_reply_t scratch;
    int         rc;

    if (r == NULL) {
        r = &scratch;
    }

    if (l->coid == -1 && link_connect(l) == -1) {
        if (l->online) {
            l->online = 0;
            rts_log("LINK %s down (cannot connect)", l->label);
        }
        l->misses++;
        return -1;
    }

    l->seq++;
    m->hdr.seq  = l->seq;
    m->hdr.t_ns = rts_now_ns();

    /*
     * A node that has gone away must never block us. TimerTimeout()
     * applies to the very next blocking call only, so it is armed
     * immediately before every MsgSend().
     */
    timeout = (uint64_t)T_SEND_TIMEOUT_MS * 1000000ULL;
    TimerTimeout(CLOCK_MONOTONIC,
                 _NTO_TIMEOUT_SEND | _NTO_TIMEOUT_REPLY,
                 NULL, &timeout, NULL);

    rc = MsgSend(l->coid, m, (int)sizeof(*m), r, (int)sizeof(*r));
    if (rc == -1) {
        /* The connection is no longer usable, throw it away and let the
           next call open a fresh one. */
        name_close(l->coid);
        l->coid = -1;
        l->misses++;
        if (l->online) {
            l->online = 0;
            rts_log("LINK %s down (%s)", l->label, strerror(errno));
        }
        return -1;
    }

    if (!l->online) {
        l->online = 1;
        rts_log("LINK %s up", l->label);
    }
    l->misses     = 0;
    return 0;
}

void rts_link_close(rts_link_t *l)
{
    if (l->coid != -1) {
        name_close(l->coid);
        l->coid = -1;
    }
    l->online = 0;
}

/* ------------------------------------------------------------------ */
/* receiving                                                           */
/* ------------------------------------------------------------------ */

int rts_server_housekeeping(int rcvid, const rts_rcv_t *rcv)
{
    /*
     * When a client calls name_open() the kernel first sends the server
     * an _IO_CONNECT message. If we do not reply to it, the client
     * blocks forever. This is the single most common bug in a QNX
     * server that uses name_attach().
     */
    if (rcv->msg.hdr.type == _IO_CONNECT) {
        MsgReply(rcvid, EOK, NULL, 0);
        return 1;
    }

    /* Any other system I/O message is not meant for us. */
    if (rcv->msg.hdr.type > _IO_BASE && rcv->msg.hdr.type <= _IO_MAX) {
        MsgError(rcvid, ENOSYS);
        return 1;
    }

    return 0;
}
