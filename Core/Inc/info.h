/*
 * info.h
 *
 *  Created on: Oct 18, 2023
 *      Author: merchelski
 */

#ifndef INC_INFO_H_
#define INC_INFO_H_

#include <stdbool.h>

#define MIN_TELLER_BREAK_TIME ((5*1*60) / 3UL)
#define MAX_TELLER_BREAK_TIME ((5*4*60) / 3UL)

#define MIN_TELLER_BREAK_WAIT ((5*30*60) / 3UL)
#define MAX_TELLER_BREAK_WAIT ((5*60*60) / 3UL)

#define MIN_CUSTOMER_ARRIVAL_DELAY ((5*60) / 3UL)
#define MAX_CUSTOMER_ARRIVAL_DELAY ((5*4*60) / 3UL)

#define MIN_SERVICE_TIME ((5*30) / 3UL)
#define MAX_SERVICE_TIME ((5*8*60) / 3UL)

#define TELLER_WAIT_TOLERANCE (10)

extern uint64_t idle_hook_count;
extern uint32_t total_customer_queue_time;
extern uint32_t max_customer_queue_time;
extern uint32_t max_customer_queue_depth;

enum enum_teller_status
{
	status_working,
	status_waiting, // Waiting for customer.
	status_on_break,
	status_done_for_the_day,
	status_didnt_show_up
} ;

typedef struct TELLER_INFO_STRUCT
{
	uint32_t max_wait_time;
	uint32_t max_service_time;
	uint32_t max_break_time;
	uint32_t min_break_time;

	uint32_t next_available_natural_break_time;

	uint16_t total_service_time;
	uint16_t total_customers_serviced;

	uint16_t total_break_time;
	uint16_t total_breaks_taken;

	uint16_t total_wait_time;
	uint16_t total_waits_taken;

	bool forced_break_flag;

	enum enum_teller_status status;
} TELLER_INFO;

typedef struct CUSTOMER_INFO_STRUCT
{
	uint16_t service_time;
	uint32_t time_entered_queue;
} CUSTOMER_INFO;


extern void init_teller(TELLER_INFO* teller);
extern void reset_and_init_customer(CUSTOMER_INFO* customer);


#endif /* INC_INFO_H_ */
