/*
 * rts_timing.h - every timing value the system uses.
 *
 * All numbers below are "design seconds", the real world values worked
 * out in section 7 of the Initial Design Report. At run time they are
 * divided by a speed factor so a whole cycle fits inside a short
 * demonstration. The relationships between the numbers never change,
 * so the safety argument still holds at any speed.
 *
 *      speed 1 : real time,  90 s cycle
 *      speed 5 : default,    18 s cycle
 */
#ifndef RTS_TIMING_H
#define RTS_TIMING_H

#include <stdint.h>

/* Clearance intervals. Fixed at compile time. No command may shorten them. */
#define T_AMBER_S            4.0
#define T_ALLRED_S           2.0

/* Pedestrian intervals. */
#define T_WALK_S             6.0
#define T_PED_CLEAR_S       12.0
#define T_PED_TOTAL_S       (T_WALK_S + T_PED_CLEAR_S)   /* 18 s */

/* Off peak: how long a car given the green has to move off before the
   green can go to the next request. */
#define T_CAR_MOVE_S         5.0

/* Green times per phase, and the cycle they add up to. */
#define G_NS_S              20.0    /* phase A, N-S road through      */
#define G_NS_RT_S           13.0    /* phase B, N-S road right turns  */
#define G_EW_S              20.0    /* phase C, E-W road through      */
#define G_EW_RT_S           13.0    /* phase D, E-W road right turns  */
#define T_CYCLE_S           90.0

/* Limits used when the control room sends new green times. */
#define G_MIN_S              8.0
#define G_MAX_S             60.0
#define T_CYCLE_MAX_S      150.0

/* Railway pre-emption. */
#define T_WARNING_S         30.0    /* notice the rail system must give   */
#define T_RAIL_CLEAR_PEAK_S 21.0    /* green needed to empty 50 m of road */
#define T_RAIL_CLEAR_OFF_S   6.0    /* off peak the buffer is nearly empty */
#define T_GATE_MOVE_S        4.0    /* how long a healthy boom gate takes  */
#define T_GATE_TIMEOUT_S     8.0    /* no movement by now means a fault    */
#define T_TRAIN_OCCUPY_S    15.0    /* how long a train sits on the crossing */

/* Trains. The line runs X1 - X2 - X3, north to south, on double track:
   southbound (NS) trains enter at X1 on track A, northbound (SN) trains
   enter at X3 on track B. A train is detected at the next crossing after
   the time it takes to drive there, and that is the same in both
   directions. */
#define T_X1_X2_S           60.0    /* about 1.5 km at 90 km/h          */
#define T_X2_X3_S           90.0    /* about 2.25 km at 90 km/h         */
#define T_TRAIN_RUSH_S     120.0    /* rush hour, a train every 2 min   */
#define T_TRAIN_OFFPEAK_S  240.0    /* off peak, a train every 4 min    */

/* Links. */
#define T_HEARTBEAT_S        1.0
#define T_OFFLINE_S          3.0    /* three missed heartbeats         */
#define T_XING_REFRESH_S     1.0    /* railway repeats its state       */
#define T_XING_WATCHDOG_S    3.0    /* silence for this long = FAULT   */
#define T_SEND_TIMEOUT_MS  200      /* no MsgSend may block longer     */

/* Speed factor, set once at start up from -s or from RTS_SPEED. */
void     rts_speed_set(double speed);
double   rts_speed(void);

/* Convert design seconds into real milliseconds and nanoseconds. */
uint64_t rts_ms(double design_seconds);
uint64_t rts_ns(double design_seconds);

#endif /* RTS_TIMING_H */
