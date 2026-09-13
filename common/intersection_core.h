/*
 * intersection_core.h - the local controller.
 *
 * All six intersections behave identically and independently; only their
 * identity and the crossing they sit next to differ. So the
 * behaviour lives here once and each intersection_iN.c is a short main()
 * that fills in a config and calls intersection_run(). Every one of the
 * six still builds into its own separate executable and runs as its own
 * independent process, exactly as the brief requires.
 *
 * Threads inside one local controller, highest priority first:
 *
 *   prio 21  t_preempt  owns the event channel "rts_iN_evt".
 *                       Receives crossing state from the railway node
 *                       and runs the crossing watchdog. A train reacted
 *                       to late is a collision, so this sits on top.
 *
 *   prio 15  t_phase    owns a private channel. Receives timer pulses
 *                       and wake pulses, runs the light state machine
 *                       and is the only thread that writes the lamps.
 *                       A phase that overruns is only a queue.
 *
 *   prio 12  t_srv      owns the command channel "rts_iN". Receives
 *                       commands from the control room, validates them
 *                       and answers accept or reject.
 *
 *   prio 10  t_report   sends a status message to the control room on
 *                       every lamp change. A status message that
 *                       arrives late is still only a message, so it is
 *                       the first thing to be starved if the CPU is
 *                       busy, and if the link stalls the intersection
 *                       carries on as though nothing happened.
 *
 *   prio  5  log writer drains the log ring buffer into /fs.
 *
 * Why not one thread: it would have to count down the current phase,
 * watch the crossing and wait for the control room at the same time,
 * and whichever one it blocked on, the other two would stop.
 */
#ifndef INTERSECTION_CORE_H
#define INTERSECTION_CORE_H

#include "rts_proto.h"

typedef struct {
    int         id;          /* 1 .. 6                                  */
    uint16_t    sender_id;   /* SND_I1 .. SND_I6                        */
    int         xing_id;     /* 1 .. 3, the crossing on this road       */
    int         rail_arm;    /* arm_t: the arm the tracks cross. The
                                crossing sits between the two controllers
                                of a pair, so it is on opposite sides of
                                them. Movements that end in this arm are
                                the ones a train forbids.               */
    const char *label;       /* "I1"                                     */
} inter_cfg_t;

int intersection_run(const inter_cfg_t *cfg, int argc, char **argv);

#endif /* INTERSECTION_CORE_H */
