/*
 * uros_config.h
 *
 *  Created on: Apr 5, 2026
 *      Author: jason
 */

#ifndef INC_UROS_CONFIG_H_
#define INC_UROS_CONFIG_H_


#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/bool.h>
#include <std_msgs/msg/int16_multi_array.h>
#include <std_msgs/msg/int16.h>
#include <rclc/executor.h>
#include "type_define.h"

#define DetectTakeUpVr

#ifdef __cplusplus
extern "C" {
#endif


typedef enum {
    AGENT_WAITING,
    AGENT_AVAILABLE,
    AGENT_CONNECTED,
    AGENT_TRYING,
    AGENT_DISCONNECTED
} agent_status_t;


#ifdef __cplusplus
}
#endif


typedef struct {
    rcl_subscription_t* sub;
    const rosidl_message_type_support_t* type;
    const char* topic;
    void* msg;
    rclc_subscription_callback_t cb;
} uros_sub_config_t;



typedef struct {
    rcl_publisher_t* pub;                      // Publisher 實體指標
    const rosidl_message_type_support_t* type; // 訊息型別
    const char* topic;                         // Topic 名稱
    uint32_t session_timeout;                  // 你之前用的 10ms 設定
} uros_pub_config_t;



typedef struct {
    rclc_support_t      support;
    rcl_allocator_t     allocator;
    rcl_node_t          node;
    rcl_init_options_t  init_options;
    rclc_executor_t     executor;
    agent_status_t      agent_status;
} Uros_Manager;


void on_flip_sub_cb(const void * msgin);
void on_take_sub_cb(const void * msgin);
void on_put_sub_cb(const void * msgin);
void on_close_sub_cb(const void * msgin);
void on_align_sub_cb(const void * msgin);
void on_left_temp_sub_cb(const void * msgin);
void on_right_temp_sub_cb(const void * msgin);

int finish_take(SideIndex side);
int finish_flip(SideIndex side);
int finish_put(SideIndex side);

void race_start();

extern uros_sub_config_t sub_configs[];
extern const int SUB_COUNT;

extern uros_pub_config_t pub_configs[];
extern const int PUB_COUNT;


#endif /* INC_UROS_CONFIG_H_ */
