
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>


#include <pico/stdlib.h>

#include <FreeRTOS.h>
#include <queue.h>
#include <task.h>
#include "tusb.h"

#include "tkjhat/sdk.h"
#include "buzzer.h"
#include "morse_translate.h"


#define BUFFER_SIZE 40

#define TILT_THRESHOLD 0.98
#define FLAT_THRESHOLD 0.98
#define SHAKE_THRESHOLD 50.0
#define FREEFALL_THRESHOLD 0.30
#define DEBOUNCE_TIME 5000

#define SW1_PIN 02
#define SW2_PIN 22
#define BUTTON_SW1 SW1_PIN
#define BUTTON_SW2 SW2_PIN

QueueHandle_t inputQueue;

TaskHandle_t imuTaskHandle = NULL;

char send_buffer[BUFFER_SIZE] = {0}; // add characters to a buffer
int buffer_index = 0;

char recv_buffer[BUFFER_SIZE]= {0};
int  recv_index = 0;

int happy_birthday[][2] = {
    {NOTE_C4, 400}, {NOTE_C4, 200}, {NOTE_D4, 400}, {NOTE_C4, 400},
    {NOTE_F4, 400}, {NOTE_E4, 800},
    {NOTE_C4, 400}, {NOTE_C4, 200}, {NOTE_D4, 400}, {NOTE_C4, 400},
    {NOTE_G4, 400}, {NOTE_F4, 800},
    {REST, 0}
};

int iphone_alarm[][2] = {
    {NOTE_C5, 300}, {NOTE_E5, 300}, {NOTE_G5, 300}, {NOTE_C6, 600},
    {NOTE_G5, 300}, {NOTE_E5, 300}, {NOTE_C5, 600},
    {NOTE_C5, 300}, {NOTE_E5, 300}, {NOTE_G5, 300}, {NOTE_C6, 600},
    {REST, 200},  // Short pause
    {NOTE_C6, 400}, {NOTE_B5, 400}, {NOTE_A5, 400}, {NOTE_G5, 400},
    {REST, 0}      // End
};

// Introducing state
enum state {IDLE=1, SEND, RECEIVE, UPDATE};

// Global state variable, initialized to waiting state
enum state myState = IDLE;
volatile bool imu_flag = true;

// Button task to go in send state
// Need an interrupt in main
void button_callback(uint gpio, uint32_t events) {
    // Button 1 pressed: send the message
    static int space_count = 0;
    if (gpio == BUTTON_SW1) {
        if (myState == RECEIVE) {
            printf("RECEIVE and then to SEND");
            myState = SEND;
            //xTaskNotifyGive(imuTaskHandle);
        } else if (myState == IDLE) {
            printf("IDLE and then to RECEIVE");
            myState = RECEIVE;
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

void feedback(const char *text) {
    for (int i = 0; i < strlen(text) + 1; i++) {
        if (text[i] == '.') {
            buzzer_play_tone(440,500);
        } else if (text[i] == '-') {
            buzzer_play_tone(800,200);
        } else if (text[i] == ' ') {
            buzzer_play_tone(200,800);
        }
    }
}

void addChar(char *s, char c) {
    while (*s) {
        s++;
    }
    *s = c;   
    *(s + 1) = '\0';
}

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
            memset(send_buffer, 0, BUFFER_SIZE);
            buffer_index = 0;

            // then change the state back to idle
            clear_display();
            myState = IDLE;
            xTaskNotifyGive(imuTaskHandle);

        } else if (myState == RECEIVE) {
            char c;

            if (xQueueReceive(inputQueue, &c, 0) == pdTRUE) {

                if (c == '\n' || c == '\r') {
                    recv_buffer[recv_index] = '\0';

                    init_buzzer();
                    printf("Received: %s\n", recv_buffer);
                    buzzer_play_melody(iphone_alarm);

                    // check if the received message is in morse
                    bool is_morse = true;
                    char text[BUFFER_SIZE] = "";
                    for (int i = 0; recv_buffer[i]; i++) {
                        if (recv_buffer[i] != '.' && recv_buffer[i] != '-' && recv_buffer[i] != ' ') {
                            is_morse = false;
                            break;
                        }
                        addChar(text, recv_buffer[i]);
                    }
                    if (strstr(text, "  .clear  ")) {
                            printf("helloooo");
                            //function for clearin the terminal
                            is_morse = true;
                            system("clear");
                    } else if (strstr(text, "  .stop  ")) {
                        printf("bye");
                        //function for stopping the program
                        is_morse = true;
                        exit(0);
                    }
                    if (is_morse) {
                        char decoded_text[BUFFER_SIZE] = {0};
                        decode_morse_message(recv_buffer, decoded_text);

                        printf("Decoded text: %s\n", decoded_text);
                        write_text(decoded_text);    // display decoded text
                        feedback(recv_buffer);
                        vTaskDelay(pdMS_TO_TICKS(10000));
                    } else {
                        // regular text
                        printf("not morse");
                        vTaskDelay(pdMS_TO_TICKS(10000));
                    }

                    /*write_text(recv_buffer);
                    feedback(recv_buffer);
                    vTaskDelay(pdMS_TO_TICKS(10000));*/   // show message

                    memset(recv_buffer, 0, BUFFER_SIZE);
                    recv_index = 0;

                    
                    clear_display();
                    vTaskDelay(pdMS_TO_TICKS(200));
                }
                else if (recv_index < BUFFER_SIZE - 1) {
                    recv_buffer[recv_index++] = c;
                }
            }
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
    for(;;) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        printf("IMU activated");
        float peak_value = 0;
        bool peak_reached = false;
        bool accepted = false;
        while (1)
        {
            if (ICM42670_read_sensor_data(&ax, &ay, &az, &gx, &gy, &gz, &t) == 0) {
                if (buffer_index < (BUFFER_SIZE-3)) {
                    if (((fabs(gx) > 50 || fabs(gy) > 50 || fabs(gz) > 100) && fabs(ax) > 0.9) && (ax != -4.00 && ay != -4.00 && az != -4.00))  {
                        printf("current values: gx %.2f, gy %.2f, gz %.2f \n", gx, gy, gz);
                        printf("dot\n");
                        send_buffer[buffer_index++] = '.';
                        blink_red_led(1);
                        buzzer_play_tone(440,500);
                        peak_reached = false;
                        accepted = false;
                    }
                    //DASH: rotating 90 degrees
                    /*else if (ay > TILT_THRESHOLD) {
                        printf("dash\n");
                        send_buffer[buffer_index++] = '-';
                        blink_red_led(2);
                        buzzer_play_tone(800,200);
                        vTaskDelay(pdMS_TO_TICKS(2000));
                    }*/
                    /*else if (fabs(az) > 0.6 && fabs(az) < 2.0 && fabs(ax) < 1.0 && fabs(ay) < 1.0 && (fabs(gx) > 5 && fabs(gy) > 5) && (fabs(gx) < 70 && fabs(gy) < 100)) {
                        lift_count++;
                        printf("count %d \n", lift_count);
                        if (lift_count >= 3) { // require two consecutive readings below threshold
                            printf("dash\n");
                            send_buffer[buffer_index++] = '-';
                            blink_red_led(1);
                            buzzer_play_tone(440,500);
                            lift_count = 0;
                            vTaskDelay(pdMS_TO_TICKS(2000));
                            
                        }
                    } */
                    else if ((fabs(ay) > 0.15 && (fabs(gz) > 3.0 || fabs(gx) > 3.0 || fabs(gy) > 3.0) && (fabs(gz) < 100.0 && fabs(gx) < 50.0 && fabs(gy) < 50.0)) || accepted) {
                        printf("hello\n");
                        accepted = true;
                        float distance_one = ay;
                        if (fabs(ay) >= peak_value && !peak_reached) {
                            printf("bigger than peak\n");
                            peak_value = fabs(ay);
                            printf("peak value: %.2f and ay: %.2f \n", peak_value, ay);
                        } else {
                            printf("peak reach\n");
                            peak_reached = true;
                            printf("current values: gx %.2f, gy %.2f, gz %.2f \n", gx, gy, gz);
                        }
                        float distance = peak_value - distance_one;
                        printf("%.2f, %.2f, %.2f, %.2f,%.2f, %.2f \n", ax, ay, az, gx, gy, gz);
                        printf("distance: %.2f", distance);
                        if (peak_reached && fabs(az - 1.0) < 0.2 && (fabs(gz) < 24.0) && distance > 0.3) {
                            printf("dash\n");
                            send_buffer[buffer_index++] = '-';
                            blink_red_led(1);
                            buzzer_play_tone(440,500);
                            peak_reached = false;
                            peak_value = 0;
                            accepted = false;
                        }          
                    }
                }
                else {
                    printf("buffer is full\n");
                }

                printf("%.2f, %.2f, %.2f, %.2f,%.2f, %.2f \n", ax, ay, az, gx, gy, gz);

            } else {
                printf("Failed to read imu data\n");
            }
            vTaskDelay(pdMS_TO_TICKS(400));
            if (myState != IDLE) { //maybe change it to if (myState != SEND)
                break;
            }
        }
    }
    // Start collection data here. Infinite loop.


}

static void usbTask(void *arg) {
    (void)arg;
    while (1) {
        tud_task();   // handle USB stack events
    }
}


void tud_cdc_rx_cb(uint8_t itf) {
    uint8_t buf[64];
    uint32_t count = tud_cdc_n_read(itf, buf, sizeof(buf));

    for (uint32_t i = 0; i < count; i++) {
        char c = buf[i];
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xQueueSendFromISR(inputQueue, &c, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

int main() {
    stdio_init_all();
    sleep_ms(2000); //Wait to see the output.
    init_hat_sdk();
    init_sw1();
    init_sw2();
    init_display();
    init_buzzer();
    while (!stdio_usb_connected()){
        sleep_ms(10);
    } 
    printf("Start tests\n");
    buzzer_play_melody(happy_birthday);
    /*write_text("Start...");
    vTaskDelay(400);
    clear_display();*/
    // Setup buttons (already initialized in SDK, we only attach callbacks)
    gpio_set_irq_enabled_with_callback(BUTTON_SW1, GPIO_IRQ_EDGE_FALL, true, button_callback);
    gpio_set_irq_enabled(BUTTON_SW2, GPIO_IRQ_EDGE_FALL, true);

    // Create tasks
    xTaskCreate(imu_task, "IMUTask", 256, NULL, 1, &imuTaskHandle);
    xTaskNotifyGive(imuTaskHandle);
    xTaskCreate(commTask, "CommTask", 512, NULL, 2, NULL);
    TaskHandle_t hUsb = NULL;
    xTaskCreate(usbTask, "usb", 1024, NULL, 3, &hUsb);
    //xTaskCreate(displayTask, "displayTask", 512, NULL, 2, NULL);
    inputQueue = xQueueCreate(64, sizeof(char));

    #if (configNUMBER_OF_CORES > 1)
        vTaskCoreAffinitySet(hUsb, 1u << 0);
    #endif
    tusb_init();
    //usb_serial_init();
    // Start the FreeRTOS scheduler
    vTaskStartScheduler();
    return 0;
}
