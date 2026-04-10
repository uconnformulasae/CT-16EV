#include "endurance_strategy.h"

/* Internal State */
static float s_bus_voltage_v;       /* latest bus voltage in volts */
static float s_bus_current_a;       /* latest bus current in amps */
static float s_energy_used_wh;      /* accumulated energy in Wh */
static float s_distance_m;          /* total distance from AiM GPS */
static uint32_t s_last_tick_ms;     /* last update timestamp */
static uint8_t s_initialized;       /* have we received first tick */

void es_init(void) {
    s_bus_voltage_v = 0.0f;
    s_bus_current_a = 0.0f;
    s_energy_used_wh = 0.0f;
    s_distance_m = 0.0f;
    s_last_tick_ms = 0;
    s_initialized = 0;
}

void es_feed_bus_voltage(uint16_t voltage_v) {
    s_bus_voltage_v = (float)voltage_v;
}

void es_feed_bus_current(int16_t current_da) {
    /* current_da is in 0.1A units from inverter CAN */
    s_bus_current_a = (float)current_da * 0.1f;
}

void es_feed_distance(uint32_t distance_m) {
    s_distance_m = (float)distance_m;
}

void es_update(uint32_t tick_ms) {
    if (!s_initialized) {
        s_last_tick_ms = tick_ms;
        s_initialized = 1;
        return;
    }

    /* Handle tick wraparound */
    uint32_t dt_ms;
    if (tick_ms >= s_last_tick_ms) {
        dt_ms = tick_ms - s_last_tick_ms;
    } else {
        /* Wraparound: assume short interval, skip this sample */
        s_last_tick_ms = tick_ms;
        return;
    }
    s_last_tick_ms = tick_ms;

    if (dt_ms == 0 || dt_ms > 1000) {
        /* Skip bogus intervals */
        return;
    }

    /* Power = V * I (watts), only count positive power (discharging) */
    float power_w = s_bus_voltage_v * s_bus_current_a;
    if (power_w < 0.0f) {
        power_w = 0.0f;
    }

    /* Energy = Power * time (convert ms to hours for Wh) */
    float dt_hours = (float)dt_ms / 3600000.0f;
    s_energy_used_wh += power_w * dt_hours;
}

es_recommendation_t es_get_status(void) {
    es_status_t status;
    status.energy_used_wh = s_energy_used_wh;
    status.distance_m = s_distance_m;

    /* CO2 hard limit at current distance */
    float co2_limit_at_distance_wh = ES_MAX_WH_PER_KM * (s_distance_m / 1000.0f);
    status.co2_limit_remaining_wh = co2_limit_at_distance_wh - s_energy_used_wh;

    /* Efficiency */
    if (s_distance_m > 100.0f) {
        status.efficiency_wh_per_km = s_energy_used_wh / (s_distance_m / 1000.0f);
    } else {
        status.efficiency_wh_per_km = 0.0f;
    }

    /* Budget comparison: where should we be vs where are we */
    float total_distance = (float)ES_ENDURANCE_DISTANCE_M;
    float fraction_complete = s_distance_m / total_distance;

    if (fraction_complete < 0.01f) {
        /* Less than 1% of race done, not enough data */
        status.budget_used_ratio = 0.0f;
        status.budget_remaining_wh = ES_ENERGY_BUDGET_WH;
        status.recommendation = ES_NO_DATA;
        return status.recommendation;
    }

    /* Budget energy at current distance */
    float budget_at_distance_wh = ES_ENERGY_BUDGET_WH * fraction_complete;
    status.budget_used_ratio = s_energy_used_wh / budget_at_distance_wh;
    status.budget_remaining_wh = ES_ENERGY_BUDGET_WH - s_energy_used_wh;

    /* Determine recommendation */
    if (status.budget_used_ratio < ES_THRESH_PUSH) {
        status.recommendation = ES_PUSH_MORE;
    } else if (status.budget_used_ratio > ES_THRESH_CONSERVE) {
        status.recommendation = ES_CONSERVE;
    } else {
        status.recommendation = ES_MAINTAIN;
    }

    /* Override: if approaching CO2 hard limit, always conserve */
    float co2_total_limit_wh = ES_MAX_WH_PER_KM * (total_distance / 1000.0f);
    if (s_energy_used_wh > co2_total_limit_wh * 0.95f) {
        status.recommendation = ES_CONSERVE;
    }

    return status.recommendation;
}
