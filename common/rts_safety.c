#include "rts_safety.h"
#include <string.h>

/*
 * Phase A : R1 through          (MV_NS, MV_SN) + pedestrians E and W
 * Phase B : R1 right turns      (MV_NW, MV_SE)
 * Phase C : R3 through          (MV_EW, MV_WE) + pedestrians N and S
 * Phase D : R3 right turns      (MV_WS, MV_EN)
 *
 * R3 is the road that runs over the railway. The tracks cross it on one
 * side of the intersection only, so "the road over the tracks" is not a
 * whole phase: it is the two movements that end in that one arm. Those
 * are the movements a train forbids, and they sit in two different
 * phases - see rts_rail_block_mask().
 */
static const int mv_phase_tab[MV_COUNT] = {
    PH_A, PH_A,   /* NS, SN */
    PH_C, PH_C,   /* EW, WE */
    PH_B, PH_B,   /* NW, SE */
    PH_D, PH_D    /* WS, EN */
};

/* The two ends of every movement, read straight off its name. */
static const int mv_from_tab[MV_COUNT] = {
    ARM_N, ARM_S,   /* NS, SN */
    ARM_E, ARM_W,   /* EW, WE */
    ARM_N, ARM_S,   /* NW, SE */
    ARM_W, ARM_E    /* WS, EN */
};
static const int mv_to_tab[MV_COUNT] = {
    ARM_S, ARM_N,   /* NS, SN */
    ARM_W, ARM_E,   /* EW, WE */
    ARM_W, ARM_E,   /* NW, SE */
    ARM_S, ARM_N    /* WS, EN */
};

/* PED_E and PED_W cross R3, so they run alongside the R1 traffic in
   phase A. PED_N and PED_S cross R1, so they run in phase C. No
   pedestrian ever runs during a right turn phase, because a turning
   vehicle would cut straight across the crossing. */
static const int ped_phase_tab[PD_COUNT] = {
    PH_C,   /* PD_N */
    PH_C,   /* PD_S */
    PH_A,   /* PD_E */
    PH_A    /* PD_W */
};

int rts_mv_phase(int mv)
{
    if (mv < 0 || mv >= MV_COUNT) return -1;
    return mv_phase_tab[mv];
}

int rts_ped_phase(int pd)
{
    if (pd < 0 || pd >= PD_COUNT) return -1;
    return ped_phase_tab[pd];
}

int rts_mv_from(int mv)
{
    if (mv < 0 || mv >= MV_COUNT) return ARM_NONE;
    return mv_from_tab[mv];
}

int rts_mv_to(int mv)
{
    if (mv < 0 || mv >= MV_COUNT) return ARM_NONE;
    return mv_to_tab[mv];
}

uint8_t rts_phase_mask(int phase)
{
    uint8_t m = 0;
    int     i;
    for (i = 0; i < MV_COUNT; i++) {
        if (mv_phase_tab[i] == phase) {
            m |= (uint8_t)(1u << i);
        }
    }
    return m;
}

uint8_t rts_rail_block_mask(int rail_arm)
{
    uint8_t m = 0;
    int     i;

    if (rail_arm != ARM_E && rail_arm != ARM_W) {
        /* Geometry not configured: fall back to shutting the whole
           rail-side road down, which is what this controller did before
           it knew which of its arms the tracks cross. */
        return (uint8_t)(rts_phase_mask(PH_C) | rts_phase_mask(PH_D));
    }

    for (i = 0; i < MV_COUNT; i++) {
        /* A movement that ends in the rail-side arm drives towards the
           crossing, whether it came straight on or turned into it. */
        if (mv_to_tab[i] == rail_arm) {
            m |= (uint8_t)(1u << i);
        }
    }
    return m;
}

int rts_rail_exit_mv(int rail_arm)
{
    int i;

    if (rail_arm != ARM_E && rail_arm != ARM_W) {
        return -1;
    }
    for (i = 0; i < MV_COUNT; i++) {
        /* Out of the rail-side arm and straight on: this is the one
           that empties the road between the gates and the stop line. */
        if (mv_from_tab[i] == rail_arm && mv_phase_tab[i] == PH_C) {
            return i;
        }
    }
    return -1;
}

int rts_lamps_safe(const uint8_t veh[MV_COUNT],
                   const uint8_t ped[PD_COUNT],
                   uint8_t blocked,
                   uint8_t *why)
{
    int active = -1;   /* the one phase allowed to be showing anything */
    int i;

    /*
     * A movement that is green or amber still occupies the
     * intersection, so both colours count as active here.
     */
    for (i = 0; i < MV_COUNT; i++) {
        if (veh[i] == LAMP_GREEN || veh[i] == LAMP_AMBER) {
            /*
             * A train is expected and this movement drives into the arm
             * the tracks cross, so nothing may release it. Only green is
             * refused: a movement caught mid-green when the train was
             * announced still has to be given its full amber, and that
             * amber is already inside the pre-emption budget (amber 4 s
             * + all-red 2 s + clearing green 21 s < 30 s of warning).
             */
            if ((blocked & (uint8_t)(1u << i)) && veh[i] == LAMP_GREEN) {
                if (why != NULL) *why = REJ_RAIL_ENTRY;
                return 0;
            }
            if (active < 0) {
                active = mv_phase_tab[i];
            } else if (active != mv_phase_tab[i]) {
                if (why != NULL) *why = REJ_CONFLICT;
                return 0;
            }
        }
    }

    /* A pedestrian who is walking or clearing is inside the crossing. */
    for (i = 0; i < PD_COUNT; i++) {
        if (ped[i] == PED_WALK || ped[i] == PED_FLASH) {
            if (active < 0) {
                active = ped_phase_tab[i];
            } else if (active != ped_phase_tab[i]) {
                if (why != NULL) *why = REJ_CONFLICT;
                return 0;
            }
        }
    }

    if (why != NULL) *why = REJ_NONE;
    return 1;
}

void rts_lamps_all_red(uint8_t veh[MV_COUNT], uint8_t ped[PD_COUNT])
{
    memset(veh, LAMP_RED, MV_COUNT);
    memset(ped, PED_DONT, PD_COUNT);
}

void rts_lamps_phase_masked(uint8_t veh[MV_COUNT], int phase,
                            uint8_t colour, uint8_t blocked)
{
    int i;
    for (i = 0; i < MV_COUNT; i++) {
        int on = (mv_phase_tab[i] == phase) &&
                 !(blocked & (uint8_t)(1u << i));
        veh[i] = on ? colour : (uint8_t)LAMP_RED;
    }
}

void rts_lamps_phase(uint8_t veh[MV_COUNT], int phase, uint8_t colour)
{
    rts_lamps_phase_masked(veh, phase, colour, 0);
}
