/*
 * intersection_i5.c - local controller for intersection I5.
 *
 * Its own main(), its own executable, its own process. The behaviour is
 * shared with the other five controllers (see intersection_core.c);
 * only the identity and the level crossing it sits next to are
 * different.
 *
 *   crossing X3 is between I5 and I6
 *   tracks cross its east arm
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
        .rail_arm  = ARM_E,       /* X3 is on the east side of I5 */
        .label     = "I5"
    };
    return intersection_run(&cfg, argc, argv);
}
