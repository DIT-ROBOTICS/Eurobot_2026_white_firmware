/*
 * uros_init.cpp
 *
 *  Created on: Apr 9, 2025
 *      Author: stanly
 */

#include "uros_init.h"
#include "stm32h723xx.h"
#include "stm32h7xx_hal.h"
#include "mission.h"
#include "uros_config.h"

extern Uros_Manager UrosMGR;

bool start_flag = 0;

int ping_fail_count = 0;
#define MAX_PING_FAIL_COUNT 5


uint32_t current_time = 0;  // Variable to store the current time
uint64_t heap_remain = 0.0; // Variable to store heap usage percentage

extern UART_HandleTypeDef USARTx;

/* for debugger */

/* for debugger */

void uros_init(void) {
  // Initialize micro-ROS
  rmw_uros_set_custom_transport(true, (void *)&USARTx, cubemx_transport_open,
                                cubemx_transport_close, cubemx_transport_write,
                                cubemx_transport_read);

  rcl_allocator_t freeRTOS_allocator = rcutils_get_zero_initialized_allocator();

  freeRTOS_allocator.allocate = microros_allocate;
  freeRTOS_allocator.deallocate = microros_deallocate;
  freeRTOS_allocator.reallocate = microros_reallocate;
  freeRTOS_allocator.zero_allocate = microros_zero_allocate;

  if (!rcutils_set_default_allocator(&freeRTOS_allocator)) {
    printf("Error on default allocators (line %d)\n", __LINE__);
  }
}

void uros_agent_status_check(void) {
  switch (UrosMGR.agent_status) {
  case AGENT_WAITING:
    handle_state_agent_waiting();
    break;
  case AGENT_AVAILABLE:
    handle_state_agent_available();
    break;
  case AGENT_CONNECTED:
    handle_state_agent_connected();
    break;
  case AGENT_TRYING:
    handle_state_agent_trying();
    break;
  case AGENT_DISCONNECTED:
    handle_state_agent_disconnected();
    break;
  default:
    break;
  }
}

void handle_state_agent_waiting(void) {
    UrosMGR.agent_status = (rmw_uros_ping_agent(100, 10) == RMW_RET_OK) ? AGENT_AVAILABLE : AGENT_WAITING;
}
void handle_state_agent_available(void) {
  uros_create_entities();
  UrosMGR.agent_status = AGENT_CONNECTED;
}
void handle_state_agent_connected(void) {
  if (rmw_uros_ping_agent(20, 5) == RMW_RET_OK) {
    rclc_executor_spin_some(&UrosMGR.executor, RCL_MS_TO_NS(10));
    ping_fail_count = 0; // Reset ping fail count
  } else {
    ping_fail_count++;
    if (ping_fail_count >= MAX_PING_FAIL_COUNT) {
      UrosMGR.agent_status = AGENT_TRYING;
    }
  }
  current_time = HAL_GetTick();
  static uint32_t prev_time = 0;
  if (current_time - prev_time >= 100) { // Every 100ms
    rmw_uros_sync_session(100);
    prev_time = current_time;
  }
}
void handle_state_agent_trying(void) {
  if (rmw_uros_ping_agent(50, 10) == RMW_RET_OK) {
    UrosMGR.agent_status = AGENT_CONNECTED;
    ping_fail_count = 0; // Reset ping fail count
  } else {
    ping_fail_count++;
    if (ping_fail_count >= MAX_PING_FAIL_COUNT) {
      UrosMGR.agent_status = AGENT_DISCONNECTED;
      ping_fail_count = 0;
    }
  }
}
void handle_state_agent_disconnected(void) {
  uros_destroy_entities();
  UrosMGR.agent_status = AGENT_WAITING;
}

void uros_create_entities(void) {
  UrosMGR.allocator = rcl_get_default_allocator();

  UrosMGR.init_options = rcl_get_zero_initialized_init_options();
  rcl_init_options_init(&UrosMGR.init_options, UrosMGR.allocator);
  rcl_init_options_set_domain_id(&UrosMGR.init_options, DOMAIN_ID);

  rclc_support_init_with_options(
      &UrosMGR.support, 0, NULL, &UrosMGR.init_options,
      &UrosMGR.allocator); // Initialize support structure

  rcl_init_options_fini(&UrosMGR.init_options);

  rclc_node_init_default(&UrosMGR.node, NODE_NAME, "robot",
                         &UrosMGR.support); // Initialize node

  rclc_executor_init(&UrosMGR.executor, &UrosMGR.support.context, SUB_COUNT,
                     &UrosMGR.allocator);

  subscription_init();

  publisher_init();
}
void uros_destroy_entities(void) {
  rmw_context_t *rmw_context =
      rcl_context_get_rmw_context(&UrosMGR.support.context);
  (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);
  // Destroy time
  // rcl_timer_fini(&start_pub_timer);

  // Destroy publisher
  for (int i = 0; i < PUB_COUNT; i++) {
    rcl_publisher_fini(pub_configs[i].pub, &UrosMGR.node);
  }

  // Destroy subscriber
  for (int i = 0; i < SUB_COUNT; i++) {
    rcl_subscription_fini(sub_configs[i].sub, &UrosMGR.node);
  }

  // Destroy executor
  rclc_executor_fini(&UrosMGR.executor);

  // Destroy node
  rcl_node_fini(&UrosMGR.node);
  rclc_support_fini(&UrosMGR.support);
}

void subscription_init(void) {
  for (int i = 0; i < SUB_COUNT; i++) {
    rclc_subscription_init_default(sub_configs[i].sub, &UrosMGR.node,
                                   sub_configs[i].type, sub_configs[i].topic);
    rclc_executor_add_subscription(&UrosMGR.executor, sub_configs[i].sub,
                                   sub_configs[i].msg, sub_configs[i].cb,
                                   ON_NEW_DATA);
  }
}

void publisher_init(void) {
  for (int i = 0; i < PUB_COUNT; i++) {
    rclc_publisher_init_default(pub_configs[i].pub, &UrosMGR.node,
                                pub_configs[i].type, pub_configs[i].topic);
    rmw_uros_set_publisher_session_timeout(
        rcl_publisher_get_rmw_handle(pub_configs[i].pub),
        pub_configs[i].session_timeout);
  }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if (GPIO_Pin == GPIO_PIN_13) { // Check if the interrupt is from PC13
        race_start();
    }
}

#include <sys/time.h>

extern "C" {
// 解決 _gettimeofday 警告
int _gettimeofday(struct timeval *tv, void *tzvp) {
    (void)tv;
    (void)tzvp;
    return 0; // 暫時回傳 0 即可
}

}