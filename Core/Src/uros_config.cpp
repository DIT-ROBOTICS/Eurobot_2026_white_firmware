/*
 * uros_config.cpp
 *
 *  Created on: Apr 5, 2026
 *      Author: jason
 */


#include "uros_config.h"
#include "mission.h"
#include "type_define.h"
#include "Debug_Mode.h"

Uros_Manager UrosMGR = {.agent_status = AGENT_WAITING};

/* -------------publisher--------------------------*/

rcl_publisher_t         start_pub;
std_msgs__msg__Bool     start_msg = {.data = false};
// rcl_timer_t             start_pub_timer;

rcl_publisher_t         off_flip_pub;
std_msgs__msg__Int16    off_flip_msg = {.data = 0};

rcl_publisher_t         off_take_pub;
std_msgs__msg__Int16    off_take_msg = {.data = 0};

rcl_publisher_t         off_put_pub;
std_msgs__msg__Int16    off_put_msg = {.data = 0};

uros_pub_config_t pub_configs[] = {
    {&start_pub,    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),  "startup/plug",          10},
    {&off_take_pub, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16), "firmware/finish/take",  10},
    {&off_flip_pub, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16), "firmware/finish/flip",  10},
    {&off_put_pub,  ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16), "firmware/finish/put",   10}
};
const int PUB_COUNT = sizeof(pub_configs) / sizeof(uros_pub_config_t);


/* -------------publisher--------------------------*/
/* -------------subscription--------------------------*/

#define Int16MultiArraySize 5
#ifndef DetectTakeUpVr
rcl_subscription_t             on_take_sub;
std_msgs__msg__Int16           on_take_msg = {.data = 0};

rcl_subscription_t             on_flip_sub;
int16_t                        on_flip_buffer[Int16MultiArraySize];
std_msgs__msg__Int16MultiArray on_flip_msg= {.data = {.data = on_flip_buffer, .size = 0, .capacity = Int16MultiArraySize}};
#else
rcl_subscription_t             on_flip_sub;
std_msgs__msg__Int16           on_flip_msg = {.data = 0};

rcl_subscription_t             on_take_sub;
int16_t                        on_take_buffer[Int16MultiArraySize];
std_msgs__msg__Int16MultiArray on_take_msg = {.data = {.data = on_take_buffer, .size = 0, .capacity = Int16MultiArraySize}};
#endif

rcl_subscription_t             on_put_sub;
std_msgs__msg__Int16           on_put_msg = {.data = 0};

rcl_subscription_t      on_close_sub;
std_msgs__msg__Int16    on_close_msg = {.data = 0};

rcl_subscription_t      on_align_sub;
std_msgs__msg__Bool     on_align_msg = {.data = false};

rcl_subscription_t      on_left_temp_sub;
std_msgs__msg__Bool     on_left_temp_msg = {.data = false};

rcl_subscription_t      on_right_temp_sub;
std_msgs__msg__Bool     on_right_temp_msg = {.data = false};


uros_sub_config_t sub_configs[] = {
    #ifndef DetectTakeUpVr
    {&on_flip_sub,          ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16MultiArray),    "on_flip",          &on_flip_msg,       on_flip_sub_cb      },
    {&on_take_sub,          ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16),              "on_take",          &on_take_msg,       on_take_sub_cb      },
    #else
    {&on_flip_sub,          ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16),              "on_flip",          &on_flip_msg,       on_flip_sub_cb      },
    {&on_take_sub,          ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16MultiArray),    "on_take",          &on_take_msg,       on_take_sub_cb      },
    #endif
    {&on_put_sub,           ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16),              "on_put",           &on_put_msg,        on_put_sub_cb       },
    {&on_close_sub,         ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16),              "close",            &on_close_msg,      on_close_sub_cb     },
    {&on_align_sub,         ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),               "on_tidy",          &on_align_msg,      on_align_sub_cb     },
    {&on_left_temp_sub,     ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),               "on_cursor_left",   &on_left_temp_msg,  on_left_temp_sub_cb },
    {&on_right_temp_sub,    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),               "on_cursor_right",  &on_right_temp_msg, on_right_temp_sub_cb}
};
const int SUB_COUNT = sizeof(sub_configs) / sizeof(uros_sub_config_t);



/* -------------subscription--------------------------*/
/* -------------subscription recall--------------------------*/


#ifndef DetectTakeUpVr
void on_flip_sub_cb(const void * msgin) {
#else
void on_take_sub_cb(const void * msgin) {
#endif
    const std_msgs__msg__Int16MultiArray * msg = (const std_msgs__msg__Int16MultiArray *)msgin;

    if (msg->data.size < Int16MultiArraySize) return;

    MissionCaller mission_caller;

#ifndef DetectTakeUpVr
  mission_caller.mission_type = FlipBlock;
#else
    mission_caller.mission_type = TakeUpBlock;
#endif
	for (int i = 0; i < 4; i++) {
        mission_caller.pinecone[i] = (int)msg->data.data[i];
    }
    mission_deliveryman(msg->data.data[4],&mission_caller);

}
#ifndef DetectTakeUpVr
void on_take_sub_cb(const void * msgin) {
  const std_msgs__msg__Int16 * msg = (const std_msgs__msg__Int16 *) msgin;
  MissionCaller mission_caller;
  mission_caller.mission_type = TakeUpBlock;
  mission_controller(msg->data,&mission_caller);
}
#else
void on_flip_sub_cb(const void * msgin) {
    const std_msgs__msg__Int16 * msg = (const std_msgs__msg__Int16 *) msgin;
    MissionCaller mission_caller;
    mission_caller.mission_type = FlipBlock;
    mission_deliveryman(msg->data,&mission_caller);
}
#endif

void on_put_sub_cb(const void * msgin) {
    const std_msgs__msg__Int16 * msg = (const std_msgs__msg__Int16 *) msgin;
    MissionCaller mission_caller;
    mission_caller.mission_type = PutDownBlock;
    mission_deliveryman(msg->data,&mission_caller);
}
void on_close_sub_cb(const void * msgin) {
    const std_msgs__msg__Int16 * msg = (const std_msgs__msg__Int16 *) msgin;
    MissionCaller mission_caller;
    mission_caller.mission_type = CloseMission;
    mission_deliveryman(msg->data,&mission_caller);
}
void on_align_sub_cb(const void * msgin) {
    const std_msgs__msg__Bool * msg = (const std_msgs__msg__Bool *) msgin;
    MissionCaller mission_caller;
    mission_caller.mission_type = AlignBlock;
    mission_caller.align_block = msg->data;
    mission_deliveryman(0,&mission_caller);
}
void on_left_temp_sub_cb(const void * msgin) {
    const std_msgs__msg__Bool * msg = (const std_msgs__msg__Bool *) msgin;
    MissionCaller mission_caller;
    mission_caller.mission_type = PushTemperture;
    mission_caller.push_temp_side = SIDE_L;
    mission_caller.push_temp = msg->data;
    mission_deliveryman(0,&mission_caller);
}
void on_right_temp_sub_cb(const void * msgin) {
    const std_msgs__msg__Bool * msg = (const std_msgs__msg__Bool *) msgin;
    MissionCaller mission_caller;
    mission_caller.mission_type = PushTemperture;
    mission_caller.push_temp_side = SIDE_R;
    mission_caller.push_temp = msg->data;
    mission_deliveryman(0,&mission_caller);
}



/* -------------subscription recall--------------------------*/
/* -------------publish func---------------------------------*/
int finish_take(SideIndex side) {
    if (*is_in_debugger_mode) return 1020;
    
    off_take_msg.data = (int16_t)side;
    rcl_ret_t ret;
    int max_attempts = 3; // 總共嘗試 3 次

    for (int i = 0; i < max_attempts; i++) {
        ret = rcl_publish(&off_take_pub, &off_take_msg, NULL);

        if (ret == RCL_RET_OK) return 1010;

        // 檢查是否是值得重試的錯誤
        if (ret == RCL_RET_TIMEOUT || ret == RCL_RET_ERROR) {
            if (i < max_attempts - 1) {
                osDelay(10);
                continue; 
            }
        } 
        else  break; 
    }
    return 1050;
}

int finish_flip(SideIndex side) {
    if (*is_in_debugger_mode) return 2020;
    
    off_flip_msg.data = (int16_t)side;
    rcl_ret_t ret;
    int max_attempts = 3; // 總共嘗試 3 次

    for (int i = 0; i < max_attempts; i++) {
        ret = rcl_publish(&off_flip_pub, &off_flip_msg, NULL);

        if (ret == RCL_RET_OK) return 2010;

        // 檢查是否是值得重試的錯誤
        if (ret == RCL_RET_TIMEOUT || ret == RCL_RET_ERROR) {
            if (i < max_attempts - 1) {
                osDelay(10);
                continue; 
            }
        } 
        else  break; 
    }
    return 2050;
}

int finish_put(SideIndex side) {
    if (*is_in_debugger_mode) return 3020;

    off_put_msg.data = (int16_t)side;
    rcl_ret_t ret;
    int max_attempts = 3; // 總共嘗試 3 次

    for (int i = 0; i < max_attempts; i++) {
        ret = rcl_publish(&off_put_pub, &off_put_msg, NULL);

        if (ret == RCL_RET_OK) return 3010;

        // 檢查是否是值得重試的錯誤
        if (ret == RCL_RET_TIMEOUT || ret == RCL_RET_ERROR) {
            if (i < max_attempts - 1) {
                osDelay(10);
                continue; 
            }
        } 
        else  break; 
    }
    return 3050;
}

void race_start() {
	start_msg.data = true;
    rcl_ret_t ret;
    int max_attempts = 3; // 總共嘗試 3 次

    for (int i = 0; i < max_attempts; i++) {
        ret = rcl_publish(&start_pub, &start_msg, NULL);

        if (ret == RCL_RET_OK) return;

        // 檢查是否是值得重試的錯誤
        if (ret == RCL_RET_TIMEOUT || ret == RCL_RET_ERROR) {
            if (i < max_attempts - 1) {
                osDelay(10);
                continue; 
            }
        } 
        else  break; 
    }
    return;
}



/* -------------publish func---------------------------------*/
