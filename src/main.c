
#include <stdio.h>
#include <string.h>


#include <pico/stdlib.h>

#include <FreeRTOS.h>
#include <queue.h>
#include <task.h>
#include "tusb.h"

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
enum state {IDLE=1, SEND, RECEIVE, UPDATE};

// Global state variable, initialized to waiting state
enum state myState = IDLE;
volatile bool imu_flag = true;
bool allow_input = true;

// Button task to go in send state
// Need an interrupt in main
void button_callback(uint gpio, uint32_t events) {
    // Button 1 pressed: send the message
    static int space_count = 0;
    if (gpio == BUTTON_SW1) {
        if (myState == IDLE) {
            myState = RECEIVE;
            allow_input = true;
    }
    } else if (gpio == BUTTON_SW2) {
        if (imu_flag) {
            send_buffer[buffer_index++] = ' ';
            printf("space\n");
        }
        space_count++;
        if (space_count >= 3) {
            imu_flag = false;
            space_count = 0;
            if (myState == IDLE) {
                myState = SEND;
            }
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
            write_text(send_buffer);
            vTaskDelay(pdMS_TO_TICKS(10000));

            // Clear buffer
            memset(send_buffer, 0, sizeof(send_buffer[0])*BUFFER_SIZE);
            buffer_index = 0;

            // then change the state back to idle
            myState = IDLE;
            clear_display();

        } else if (myState == RECEIVE  && allow_input) {
            printf("Type your message: ");
            fflush(stdout);
            if (fgets(recv_buffer, sizeof(recv_buffer), stdin)) {
                recv_buffer[strcspn(recv_buffer, "\n")] = '\0';
                printf("something");
                fflush(stdout);
                if (strlen(recv_buffer) > 0) {
                    printf("Message typed: %s\n", recv_buffer);

                    write_text(recv_buffer);
                    allow_input = false;
                } else {
                    printf("Empty message.\n");
                }
            }
            myState = IDLE;
            printf("End message");
            clear_display();
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
    
    init_buzzer();
    init_red_led();
    
    // Start collection data here. Infinite loop.
    while (1)
    {
        if (imu_flag) {
            if (ICM42670_read_sensor_data(&ax, &ay, &az, &gx, &gy, &gz, &t) == 0) {
                if (buffer_index < (BUFFER_SIZE-3)) {
                    if (az > FLAT_THRESHOLD) {
                        printf("dot\n");
                        send_buffer[buffer_index++] = '.';
                        blink_red_led(1);
                        buzzer_play_tone(440,500);
                        vTaskDelay(pdMS_TO_TICKS(10000));
                    }
                    //DASH: rotating 90 degrees
                    else if (ay > TILT_THRESHOLD) {
                        printf("dash\n");
                        send_buffer[buffer_index++] = '-';
                        blink_red_led(2);
                        buzzer_play_tone(800,200);
                        vTaskDelay(pdMS_TO_TICKS(10000));
                    }
                }
                else {
                    printf("buffer is full\n");
                }
            
                // printf("Accel: X=%.2f, Y=%.2f, Z=%.2f | Gyro: X=%.2f, Y=%.2f, Z=%.2f \n", ax, ay, az, gx, gy, gz);

            } else {
                printf("Failed to read imu data\n");
            }
            vTaskDelay(pdMS_TO_TICKS(400));
        }
        
    }

}

static void usbTask(void *arg) {
    (void)arg;
    while (1) {
        tud_task();   // handle USB stack events
    }
}

void tud_cdc_rx_cb(uint8_t itf) {
    uint8_t buf[BUFFER_SIZE + 1];
    uint32_t count = tud_cdc_n_read(itf, buf, sizeof(buf) - 1);

    // Always read from both interfaces to avoid stalling CDC0
    if (itf == 0) tud_cdc_read_flush();  

    // Null-terminate to make it a string
    buf[count] = '\0';

    // Show on the display
    write_text((char *)buf);

    // Send acknowledgment back to host
    tud_cdc_n_write(itf, (uint8_t const *)"OK\n", 3);
    tud_cdc_n_write_flush(itf);
}

// not needed until receiving
// Handling display update
/*void displayTask(void *pvParameters) {
    init_display();
    while (1) {
    
        if (myState == UPDATE) {
        
            // Functionality of state
            update_screen();
            
            // State transition UPDATE -> IDLE
            myState = IDLE;
        }
    
        vTaskDelay(400);
    }
}*/

int main() {
    stdio_init_all();
    sleep_ms(2000); //Wait to see the output.
    init_hat_sdk();
    init_sw1();
    init_sw2();
    init_display();
    while (!stdio_usb_connected()){
        sleep_ms(10);
    } 
    printf("Start tests\n");
    /*write_text("Start...");
    vTaskDelay(400);
    clear_display();*/
    // Setup buttons (already initialized in SDK, we only attach callbacks)
    gpio_set_irq_enabled_with_callback(BUTTON_SW1, GPIO_IRQ_EDGE_FALL, true, button_callback);
    gpio_set_irq_enabled(BUTTON_SW2, GPIO_IRQ_EDGE_FALL, true);

    // Create tasks
    xTaskCreate(imu_task, "IMUTask", 256, NULL, 1, NULL);
    xTaskCreate(commTask, "CommTask", 512, NULL, 2, NULL);
    TaskHandle_t hUsb = NULL;
    xTaskCreate(usbTask, "usb", 1024, NULL, 3, &hUsb);
    //xTaskCreate(displayTask, "displayTask", 512, NULL, 2, NULL);
    #if (configNUMBER_OF_CORES > 1)
        vTaskCoreAffinitySet(hUsb, 1u << 0);
    #endif
    tusb_init();
    //usb_serial_init();
    // Start the FreeRTOS scheduler
    vTaskStartScheduler();

    return 0;
}
