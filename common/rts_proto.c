#include "rts_proto.h"

static const char *mv_names[MV_COUNT] = {
    "NS", "SN", "EW", "WE", "NW", "SE", "WS", "EN"
};

const char *rts_mv_name(int mv)
{
    if (mv < 0 || mv >= MV_COUNT) return "??";
    return mv_names[mv];
}

const char *rts_lamp_name(uint8_t lamp)
{
    switch (lamp) {
        case LAMP_GREEN: return "GREEN";
        case LAMP_AMBER: return "AMBER";
        default:         return "RED";
    }
}

const char *rts_phase_name(uint8_t phase)
{
    switch (phase) {
        case PH_A: return "A";
        case PH_B: return "B";
        case PH_C: return "C";
        case PH_D: return "D";
        default:   return "?";
    }
}

const char *rts_state_name(uint8_t state)
{
    switch (state) {
        case ST_STARTUP:   return "STARTUP";
        case ST_GREEN:     return "GREEN";
        case ST_AMBER:     return "AMBER";
        case ST_ALLRED:    return "ALLRED";
        case ST_PRE_CLEAR: return "RAILCLR";
        default:           return "?";
    }
}

const char *rts_pattern_name(uint8_t pattern)
{
    switch (pattern) {
        case PAT_FIXED:   return "FIXED";
        case PAT_SENSOR:  return "SENSOR";
        case PAT_UPDATED: return "UPDATED";
        default:          return "?";
    }
}

const char *rts_xing_name(uint8_t state)
{
    switch (state) {
        case XS_CLEAR:   return "CLEAR";
        case XS_WARNING: return "WARNING";
        case XS_CLOSED:  return "CLOSED";
        case XS_FAULT:   return "FAULT";
        default:         return "?";
    }
}

const char *rts_gate_name(uint8_t pos)
{
    switch (pos) {
        case GATE_UP:   return "UP";
        case GATE_DOWN: return "DOWN";
        default:        return "MOVING";
    }
}

const char *rts_arm_name(int arm)
{
    switch (arm) {
        case ARM_N: return "north";
        case ARM_S: return "south";
        case ARM_E: return "east";
        case ARM_W: return "west";
        default:    return "unknown";
    }
}

const char *rts_reject_name(uint8_t reason)
{
    switch (reason) {
        case REJ_NONE:            return "ok";
        case REJ_UNKNOWN_PATTERN: return "unknown pattern";
        case REJ_MIN_GREEN:       return "green below the safe minimum";
        case REJ_MAX_GREEN:       return "green above the allowed maximum";
        case REJ_CYCLE_TOO_LONG:  return "cycle too long";
        case REJ_CONFLICT:        return "would release a conflicting movement";
        case REJ_BAD_PHASE:       return "no such phase";
        case REJ_PREEMPT_ACTIVE:  return "railway pre-emption is active";
        case REJ_RAIL_ENTRY:      return "would drive into the railway area";
        default:                  return "?";
    }
}
