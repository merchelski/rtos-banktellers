/*
 * info.c
 *
 *  Created on: Oct 18, 2023
 *      Author: merchelski
 */

#include <inttypes.h>

#include "info.h"
#include "stdbool.h"
#include "rng.h"
#include "stm32l4xx_hal.h"

uint64_t idle_hook_count = 0;
uint16_t total_customer_queue_time = 0;
uint16_t max_customer_queue_time = 0;
uint16_t max_customer_queue_depth = 0;


void init_teller(TELLER_INFO* teller)
{
	teller->max_wait_time = 0;
	teller->max_service_time = 0;
	teller->max_break_time = 0;
	teller->min_break_time = (uint32_t)(-1); // Max value for uint32_t
	teller->total_service_time = 0;
	teller->total_wait_time = 0;
	teller->total_waits_taken = 0;
	teller->total_break_time = 0;
	teller->total_breaks_taken = 0;
	teller->total_customers_serviced = 0;
	teller->next_available_natural_break_time = rand_range(MIN_TELLER_BREAK_WAIT, MAX_TELLER_BREAK_WAIT);
	teller->status = status_didnt_show_up;
	teller->forced_break_flag = false;

}

void reset_and_init_customer(CUSTOMER_INFO* customer)
{
	customer->service_time = rand_range(MIN_SERVICE_TIME, MAX_SERVICE_TIME);
	customer->time_entered_queue = HAL_GetTick();
}
