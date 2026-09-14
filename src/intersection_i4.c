/*
 * intersection_i4.c - local controller for intersection I4.
 *
 * Its own main(), its own executable, its own process. The behaviour is
 * shared with the other five controllers (see intersection_core.c);
 * only the identity and the level crossing it sits next to are
 * different.
 *
 *   roads R2 (north-south) and R4 (east-west, over the tracks)
 *   crossing X2 is between I4 and I3
 *   tracks cross its west arm
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
        .road_ns   = "R2",
        .road_ew   = "R4",
        .rail_arm  = ARM_W,       /* X2 is on the west side of I4 */
        .label     = "I4"
    };
    return intersection_run(&cfg, argc, argv);
}
