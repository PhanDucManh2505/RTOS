/*
 * intersection_i2.c - local controller for intersection I2.
 *
 * Its own main(), its own executable, its own process. The behaviour is
 * shared with the other five controllers (see intersection_core.c);
 * only the identity, the level crossing it sits next to and the green
 * wave offset are different.
 *
 *   crossing X1 is between I2 and I1
 *   tracks cross its west arm
 *   offset   12 s behind its partner
 *
 * Run:  ./intersection_i2 [-s speed]
 */
#include "intersection_core.h"

int main(int argc, char **argv)
{
    static const inter_cfg_t cfg = {
        .id        = 2,
        .sender_id = SND_I2,
        .xing_id   = 1,
        .pair_id   = 1,
        .rail_arm  = ARM_W,       /* X1 is on the west side of I2 */
        .offset_s  = 12.0,
        .label     = "I2"
    };
    return intersection_run(&cfg, argc, argv);
}
