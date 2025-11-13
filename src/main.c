
#include <stdio.h>
#include <string.h>

#include <pico/stdlib.h>

#include <FreeRTOS.h>
#include <queue.h>
#include <task.h>

#include "tkjhat/sdk.h"


#define BUFFER_SIZE 40

#define TILT_THRESHOLD 0.98
#define FLAT_THRESHOLD 0.98
#define SHAKE_THRESHOLD

#define SW1_PIN 02
#define SW2_PIN 22
#define BUTTON_SW1 SW1_PIN
#define BUTTON_SW2 SW2_PIN

char send_buffer[BUFFER_SIZE] = {0}; // add characters to a buffer
int buffer_index = 0;

char recv_buffer[BUFFER_SIZE];
int  recv_index = 0;

// Introducing state
enum state {IDLE=1, SEND};

// Global state variable, initialized to waiting state
enum state myState = IDLE;


// Button task to go in send state
// Need an interrupt in main
void button_callback(uint gpio, uint32_t events) {
    // Button 1 pressed: send the message
    
    if (gpio == BUTTON_SW1) {
        printf("hi");
        if (myState == IDLE) {
            myState = SEND;
    }
    } else if (gpio == BUTTON_SW2) {
        printf("hello");
        if (buffer_index < BUFFER_SIZE - 1) {
            send_buffer[buffer_index++] = ' ';
    }
    }
    
}

// Communications task
void commTask(void *pvParameters) {
    (void)pvParameters;
    
    while (1) {
        if (myState == SEND) {
            
            
            //maybe for loop to go through send_buffer
            printf("Sending");
            //printf(send_buffer);
            for (int i = 0; i < sizeof(send_buffer); i++) {
                printf("%c",send_buffer[i]);
            }
            
            // maybe a \n at the end of msg

            // Clear buffer
            memset(send_buffer, 0, sizeof(send_buffer[0])*BUFFER_SIZE);
            buffer_index = 0;

            // then change the state back to idle
            myState = IDLE;
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}


void imu_task(void *pvParameters) {
    (void)pvParameters;
    
    float ax, ay, az, gx, gy, gz, t;
    // Setting up the sensor. 
    if (init_ICM42670() == 0) {
        printf("ICM-42670P initialized successfully!\n");
        if (ICM42670_start_with_default_values() != 0){
            printf("ICM-42670P could not initialize accelerometer or gyroscope");
        }
    } else {
        printf("Failed to initialize ICM-42670P.\n");
    }
    
    //init_red_led();
    
    // Start collection data here. Infinite loop.
    while (1)
    {
        if (ICM42670_read_sensor_data(&ax, &ay, &az, &gx, &gy, &gz, &t) == 0) {
            if (buffer_index < (BUFFER_SIZE-1)) {
                if (az > FLAT_THRESHOLD) {
                    printf("dot");
                    send_buffer[buffer_index++] = '.';
                }
                //DASH: rotating 90 degrees
                else if (ay > TILT_THRESHOLD) {
                    printf("dash");
                    send_buffer[buffer_index++] = '-';
                }
            }
            else {
                printf("buffer is full\n");
            }
            
            printf("Accel: X=%.2f, Y=%.2f, Z=%.2f | Gyro: X=%.2f, Y=%.2f, Z=%.2f \n", ax, ay, az, gx, gy, gz);

        } else {
            printf("Failed to read imu data\n");
        }
        vTaskDelay(pdMS_TO_TICKS(400));
    }

}

// not needed until receiving
// Handling display update
/*void displayTask(void *pvParameters) {

    while (1) {
    
        if (myState == UPDATE) {
        
            // Functionality of state
            update_screen();
            
            // State transition UPDATE -> IDLE
            myState = IDLE;
        }
    
        vTaskDelay(..);
    }
}*/

int main() {
    stdio_init_all();
    sleep_ms(2000); //Wait to see the output.
    init_hat_sdk();
    init_sw1();
    init_sw2();
    while (!stdio_usb_connected()){
        sleep_ms(10);
    } 
    printf("Start tests\n");
        
    // Setup buttons (already initialized in SDK, we only attach callbacks)
    gpio_set_irq_enabled_with_callback(BUTTON_SW1, GPIO_IRQ_EDGE_FALL, true, button_callback);
    gpio_set_irq_enabled(BUTTON_SW2, GPIO_IRQ_EDGE_FALL, true);

    // Create tasks
    xTaskCreate(imu_task, "IMUTask", 256, NULL, 1, NULL);
    xTaskCreate(commTask, "CommTask", 512, NULL, 2, NULL);
    
    // Start the FreeRTOS scheduler
    vTaskStartScheduler();

    return 0;
}
