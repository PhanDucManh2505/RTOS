/*
 * intersection_i6.c - local controller for intersection I6.
 *
 * Its own main(), its own executable, its own process. The behaviour is
 * shared with the other five controllers (see intersection_core.c);
 * only the identity and the level crossing it sits next to are
 * different.
 *
 *   crossing X3 is between I6 and I5
 *   tracks cross its west arm
 *
 * Run:  ./intersection_i6 [-s speed]
 */
#include "intersection_core.h"

int main(int argc, char **argv)
{
    static const inter_cfg_t cfg = {
        .id        = 6,
        .sender_id = SND_I6,
        .xing_id   = 3,
        .rail_arm  = ARM_W,       /* X3 is on the west side of I6 */
        .label     = "I6"
    };
    return intersection_run(&cfg, argc, argv);
}
