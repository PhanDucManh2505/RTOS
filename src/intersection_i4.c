/*
 * intersection_i4.c - local controller for intersection I4.
 *
 * Its own main(), its own executable, its own process. The behaviour is
 * shared with the other five controllers (see intersection_core.c);
 * only the identity, the level crossing it sits next to and the green
 * wave offset are different.
 *
 *   crossing X2 is between I4 and I3
 *   tracks cross its west arm
 *   offset   12 s behind its partner
 *
 * Run:  ./intersection_i4 [-s speed]
 */
#include "intersection_core.h"

int main(int argc, char **argv)
{
    static const inter_cfg_t cfg = {
        .id        = 4,
        .sender_id = SND_I4,
        .xing_id   = 2,
        .pair_id   = 3,
        .rail_arm  = ARM_W,       /* X2 is on the west side of I4 */
        .offset_s  = 12.0,
        .label     = "I4"
    };
    return intersection_run(&cfg, argc, argv);
}
