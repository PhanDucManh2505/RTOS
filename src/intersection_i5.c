/*
 * intersection_i5.c - local controller for intersection I5.
 *
 * Its own main(), its own executable, its own process. The behaviour is
 * shared with the other five controllers (see intersection_core.c);
 * only the identity, the level crossing it sits next to and the green
 * wave offset are different.
 *
 *   crossing X3 is between I5 and I6
 *   tracks cross its east arm
 *   offset   0 s behind its partner
 *
 * Run:  ./intersection_i5 [-s speed]
 */
#include "intersection_core.h"

int main(int argc, char **argv)
{
    static const inter_cfg_t cfg = {
        .id        = 5,
        .sender_id = SND_I5,
        .xing_id   = 3,
        .pair_id   = 6,
        .rail_arm  = ARM_E,       /* X3 is on the east side of I5 */
        .offset_s  = 0.0,
        .label     = "I5"
    };
    return intersection_run(&cfg, argc, argv);
}
