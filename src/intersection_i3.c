/*
 * intersection_i3.c - local controller for intersection I3.
 *
 * Its own main(), its own executable, its own process. The behaviour is
 * shared with the other five controllers (see intersection_core.c);
 * only the identity, the level crossing it sits next to and the green
 * wave offset are different.
 *
 *   crossing X2 is between I3 and I4
 *   tracks cross its east arm
 *   offset   0 s behind its partner
 *
 * Run:  ./intersection_i3 [-s speed]
 */
#include "intersection_core.h"

int main(int argc, char **argv)
{
    static const inter_cfg_t cfg = {
        .id        = 3,
        .sender_id = SND_I3,
        .xing_id   = 2,
        .pair_id   = 4,
        .rail_arm  = ARM_E,       /* X2 is on the east side of I3 */
        .offset_s  = 0.0,
        .label     = "I3"
    };
    return intersection_run(&cfg, argc, argv);
}
