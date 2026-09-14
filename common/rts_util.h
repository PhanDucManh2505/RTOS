/*
 * rts_util.h - the QNX plumbing every process needs.
 *
 * Nothing here knows anything about traffic lights. It is only the
 * small wrappers around channels, timers, threads and MsgSend() that
 * would otherwise be copied into all nine programs.
 */
#ifndef RTS_UTIL_H
#define RTS_UTIL_H

#include "rts_proto.h"
#include <pthread.h>
#include <stdint.h>
#include <time.h>

/* ---------------- start up ---------------------------------------- */

/*
 * Print the standard header (student, process, PID, hostname) and turn
 * off stdout buffering so nothing is lost if the process is killed.
 * Also reads -s <speed> and RTS_SPEED. Returns the index of the first
 * argument it did not consume.
 */
int rts_start(const char *proc_name, int argc, char **argv);

/* Set by the SIGINT/SIGTERM handler. Threads check it to shut down. */
extern volatile int rts_running;
void rts_install_signals(void);

/* ---------------- time -------------------------------------------- */

uint64_t rts_now_ns(void);              /* CLOCK_MONOTONIC, never jumps */
void     rts_sleep_ms(uint64_t ms);
/* Seconds since this process started, for readable log lines. */
double   rts_uptime_s(void);

/* ---------------- threads and locks -------------------------------- */

/*
 * Start a thread at a fixed priority instead of letting it inherit the
 * parent's. QNX priorities run 1 (low) to 63 (high).
 */
int rts_thread(pthread_t *tid, void *(*fn)(void *), void *arg, int prio);

/*
 * A mutex with priority inheritance. Without it a low priority thread
 * holding the lock could block a high priority one for an unbounded
 * time, which is the classic priority inversion problem.
 */
void rts_mutex_init(pthread_mutex_t *m);

/* ---------------- channels and timers ------------------------------ */

/*
 * A private channel plus a connection to it. Timers and other threads
 * send pulses to 'coid'; the owner blocks on MsgReceive(chid).
 */
typedef struct {
    int chid;
    int coid;
} rts_chan_t;

int  rts_chan_open(rts_chan_t *c);
void rts_chan_close(rts_chan_t *c);

/*
 * A POSIX timer that fires a pulse into a channel. 'code' says which
 * timer it was, 'value' carries whatever the owner wants to match on.
 */
int  rts_timer_new(timer_t *t, const rts_chan_t *c, int prio, int code, int value);
/* Arm a one shot. ms == 0 disarms the timer. */
int  rts_timer_once(timer_t t, uint64_t ms);
/* Arm a repeating timer. */
int  rts_timer_every(timer_t t, uint64_t ms);

/* ---------------- sending ------------------------------------------ */

/*
 * A named link to another process, local or on another node.
 * The connection is opened lazily and re-opened after a failure, so a
 * server can be started in any order and can be restarted mid demo.
 */
typedef struct {
    char     node[64];
    char     svc[64];
    char     label[32];
    int      coid;        /* -1 when not connected */
    int      online;
    int      misses;
    uint32_t seq;
} rts_link_t;

void rts_link_init(rts_link_t *l, const char *node, const char *svc,
                   const char *label);
/*
 * Send one message and wait for the reply, but never longer than
 * T_SEND_TIMEOUT_MS. Fills hdr.seq and hdr.t_ns for the caller.
 * Returns 0 on success, -1 if the other side did not answer.
 */
int  rts_link_send(rts_link_t *l, rts_msg_t *m, rts_reply_t *r);
void rts_link_close(rts_link_t *l);

/* ---------------- receiving ---------------------------------------- */

/*
 * The two housekeeping messages every named server must handle.
 * Returns 1 if the message was consumed and the caller should carry on
 * with the next MsgReceive(), 0 if it is a message for the caller.
 */
int rts_server_housekeeping(int rcvid, const rts_rcv_t *rcv);

#endif /* RTS_UTIL_H */
