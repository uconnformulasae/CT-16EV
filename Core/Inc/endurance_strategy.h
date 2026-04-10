#ifndef ENDURANCE_STRATEGY_H
#define ENDURANCE_STRATEGY_H

#include <stdint.h>

/* Pack Specs */
#define ES_PACK_SERIES          100
#define ES_PACK_PARALLEL        4
#define ES_CELL_NOMINAL_V       3.6f
#define ES_CELL_CAPACITY_AH     5.0f
#define ES_PACK_ENERGY_WH       7200.0f   /* 100s4p × 18 Wh/cell */

/* Max allowed energy (Wh) per km: (20.02 / 0.65) * 10 = 308 Wh/km */
/*(CO2 LIMIT KG/100KM) / (CO2/KWH for EV) */
#define ES_MAX_WH_PER_KM        308.0f

#define ES_ENDURANCE_DISTANCE_M 22000     /* 22 km total */
#define ES_ENDURANCE_LAPS       20        /* adjust to actual lap count */

/* Budget: 90% of CO2 limit as target, leaves 10% safety margin */
#define ES_BUDGET_MARGIN         0.90f
#define ES_ENERGY_BUDGET_WH     (ES_MAX_WH_PER_KM * (ES_ENDURANCE_DISTANCE_M / 1000.0f) * ES_BUDGET_MARGIN)

/* Recommendation Thresholds */
/* ratio = energy_used / budget_at_current_distance                */
#define ES_THRESH_PUSH          0.80f     /* below this: push harder */
#define ES_THRESH_CONSERVE      0.95f     /* above this: conserve    */

/* CAN IDs */
#define ES_AIM_GPS_DISTANCE_CAN_ID   0x650   /* AiM GPS distance broadcast */
#define ES_INVERTER_CURRENT_CAN_ID   0x0A6   /* Cascadia PM100 current info */


typedef enum {
    ES_PUSH_MORE,       /* under budget, driver can push harder */
    ES_MAINTAIN,        /* on target */
    ES_CONSERVE,        /* over budget, need to back off */
    ES_NO_DATA          /* not enough data yet */
} es_recommendation_t;

typedef struct {
    float energy_used_wh;           /* total energy consumed so far */
    float distance_m;               /* total distance traveled */
    float efficiency_wh_per_km;     /* current average efficiency */
    float budget_used_ratio;        /* energy_used / budget_at_distance */
    float budget_remaining_wh;      /* energy left in budget */
    float co2_limit_remaining_wh;   /* energy left before DQ */
    es_recommendation_t recommendation;
} es_status_t;


void es_init(void);

void es_feed_bus_voltage(uint16_t voltage_v);
void es_feed_bus_current(int16_t current_da);  /* deciamps (0.1A units) */
void es_feed_distance(uint32_t distance_m);    /* meters from AiM GPS */

/* Call periodically to integrate energy */
void es_update(uint32_t tick_ms);

/* Get current status */
es_status_t es_get_status(void);
es_recommendation_t es_get_recommendation(void);

#endif
