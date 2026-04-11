/*
 * uros_init.h
 *
 *  Created on: Apr 9, 2025
 *      Author: stanly
 */

#ifndef INC_UROS_INIT_H_
#define INC_UROS_INIT_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "cmsis_os.h"

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <uxr/client/transport.h>
#include <rmw_microxrcedds_c/config.h>
#include <rmw_microros/rmw_microros.h>

#include "config.h"
#include "timers.h"



bool cubemx_transport_open(struct uxrCustomTransport * transport);
bool cubemx_transport_close(struct uxrCustomTransport * transport);
size_t cubemx_transport_write(struct uxrCustomTransport* transport, const uint8_t * buf, size_t len, uint8_t * err);
size_t cubemx_transport_read(struct uxrCustomTransport* transport, uint8_t* buf, size_t len, int timeout, uint8_t* err);

void * microros_allocate(size_t size, void * state);
void microros_deallocate(void * pointer, void * state);
void * microros_reallocate(void * pointer, size_t size, void * state);
void * microros_zero_allocate(size_t number_of_elements, size_t size_of_element, void * state);


void uros_init(void);

void uros_agent_status_check(void);


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);

#ifdef __cplusplus
}


void handle_state_agent_waiting(void);
void handle_state_agent_available(void);
void handle_state_agent_connected(void);
void handle_state_agent_trying(void);
void handle_state_agent_disconnected(void);


void uros_create_entities(void);
void uros_destroy_entities(void);

void subscription_init(void);
void publisher_init(void);

void start_pub_cb(rcl_timer_t * timer, int64_t last_call_time);


#endif
#endif /* INC_UROS_INIT_H_ */
