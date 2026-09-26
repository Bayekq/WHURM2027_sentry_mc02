/**
 * @file UserTask.cpp
 * @author BayekQ  RM2026 (206858817@qq.com)
 * @brief Create user tasks with cpp support
 * @version 0.1
 * @date 2026/09/25
 *
 * @copyright WHU-LuojiaFox (c) 2027
 */
#include "FreeRTOS.h" // Include FreeRTOS.h
#include "main.h"
#include "task.h" // Include task
#include "UserTask.h" // Include user task header]

bool init_finished = false;

/**
 * @brief User task function(Template)
 */
void userTask(void*) {
    /* Your user layer codes begin here*/
    /*=================================================*/
    
    /* Your user layer codes end here*/
    /*=================================================*/
    while (true) {
        /* Your user layer codes in loop begin here*/
        /*=================================================*/
        static bool init_finished = true;


        // HAL_IWDG_Refresh(&hiwdg1); // Refresh the IWDG to prevent reset
        /* Your user layer codes in loop end here*/
        /*=================================================*/

        vTaskDelay(1); // Delay and block the task for 1ms.
    }
}
