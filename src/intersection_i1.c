/*
 * intersection_i1.c - local controller for intersection I1.
 *
 * Its own main(), its own executable, its own process. The behaviour is
 * shared with the other five controllers (see intersection_core.c);
 * only the identity and the level crossing it sits next to are
 * different.
 *
 *   crossing X1 is between I1 and I2
 *   tracks cross its east arm
 *
 * Run:  ./intersection_i1 [-s speed]
 */
#include "intersection_core.h"

int main(int argc, char **argv)
{
    static const inter_cfg_t cfg = {
        .id        = 1,
        .sender_id = SND_I1,
        .xing_id   = 1,
        .rail_arm  = ARM_E,       /* X1 is on the east side of I1 */
        .label     = "I1"
    };
    return intersection_run(&cfg, argc, argv);
}
