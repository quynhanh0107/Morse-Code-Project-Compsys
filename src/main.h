#ifndef MAIN_H
#define MAIN_H

#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <stdbool.h>

#include "buzzer.h"
#include "tkjhat/sdk.h"
#include "tkjhat/pins.h"

#define BUFFER_SIZE 40

// Global buffers for sending/receiving
extern char send_buffer[BUFFER_SIZE];
extern int buffer_index;

extern char recv_buffer[BUFFER_SIZE];
extern int recv_index;

extern QueueHandle_t inputQueue;
extern TaskHandle_t imuTaskHandle;

// Movement detection shared state
extern float peak_value;
extern bool peak_reached;
extern bool accepted;

#endif
