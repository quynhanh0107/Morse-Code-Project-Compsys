/*


Computer Systems Module 2 course project
Created by Tinja Untinen, Le Phuong Ling, Le Quynh-Anh
Finalized 23.11.2025


*/
//include all necessary libraries
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

//Define global varibales and pins.

#define BUFFER_SIZE 40

//#define TILT_THRESHOLD 0.98
//#define FLAT_THRESHOLD 0.98
//#define SHAKE_THRESHOLD 50.0
//#define FREEFALL_THRESHOLD 0.30
//#define DEBOUNCE_TIME 5000

//Pins for the buttons in the pico. 
#define SW1_PIN 02
#define SW2_PIN 22
//Buttons of the pico
#define BUTTON_SW1 SW1_PIN
#define BUTTON_SW2 SW2_PIN

//Initialize the input queu used for when receiving messages.
QueueHandle_t inputQueue;
//Initialize and define the task handle.
TaskHandle_t imuTaskHandle = NULL;

//More varibales initialized
//send_buffer is for sending the values to the pico.
char send_buffer[BUFFER_SIZE] = {0};
int buffer_index = 0;
//As named recv_buffer is for the Receive state, it takes the values input by user in terminal.
char recv_buffer[BUFFER_SIZE]= {0};
int  recv_index = 0;

//songs/sounds to play:
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

// Introducing states
enum state {IDLE=1, SEND, RECEIVE, UPDATE};

// Global state variable, initialized to idle state
enum state myState = IDLE;
//imu_flag for checking the press of button and input of space
//!!!!!!!!
//!!!!!
//Make sure this imu_flag works correctly since we never set it back to true in the code.
volatile bool imu_flag = true;

// Button task to switch states and adding space
// Needs an interrupt in main
//Enable one of the buttons and define the events handler (interrupt) for button presses
void button_callback(uint gpio, uint32_t events) {
    // Since pressing three spaces triggers a send we set the space_count to 0 here
    static int space_count = 0;
    // Button 1 pressed: changing states
    if (gpio == BUTTON_SW1) {
        if (myState == RECEIVE) {
            //From state Receive to state Send.
            //!!!!!
            //Thoughts: Why don't we just have three space for to switch state to send? So the other button could just switch between Idle and Receive
            printf("RECEIVE and then to SEND");
            myState = SEND;
            //xTaskNotifyGive(imuTaskHandle);
        } else if (myState == IDLE) {
            //From state idle to state Receive
            printf("IDLE and then to RECEIVE");
            myState = RECEIVE;
        }
    //Button 2 pressed: if pressed once, considered space, if pressed three times back to back becomes a send
    } else if (gpio == BUTTON_SW2) {
        //Here we use imu_flag to make sure imu is active and if space count reaches three, won't be added to the buffer
        if (imu_flag) {
            //Adding space to send_buffer
            send_buffer[buffer_index++] = ' ';
            printf("space\n");
        }
        //Counting the spaces
        space_count++;
        if (space_count >= 3) {
            //Discontinue imu_flag
            imu_flag = false;
            //Reset the space_count
            space_count = 0;
            //Switching state from idle to sending, since data reading happends in IDLE
            if (myState == IDLE) {
                myState = SEND;
            } 
        }    
    }
    
}

//Function for buzzer sounds of different morse characters
void feedback(const char *text) {
    //Going through the morse sent.
    for (int i = 0; i < strlen(text) + 1; i++) {
        if (text[i] == '.') {
            //First value is about the frequency and next about the duration of the sound.
            buzzer_play_tone(440,500);
        } else if (text[i] == '-') {
            buzzer_play_tone(800,200);
        } else if (text[i] == ' ') {
            buzzer_play_tone(200,800);
        }
    }
}


//Adding characters from array into a string
void addChar(char *s, char c) {
    while (*s) {
        s++;
    }
    *s = c;   
    *(s + 1) = '\0';
}

//Communication task, used in both state receive and send.
void commTask(void *pvParameters) {
    (void)pvParameters;
    
    while (1) {
        //When state is send:
        if (myState == SEND) {
            printf("Sending");
            //for loop to go through the send_buffer
            for (int i = 0; i < sizeof(send_buffer); i++) {
                //printing the buffer to terminal
                printf("%c",send_buffer[i]);
            }
            //Printing the send_buffer to lcd
            write_text(send_buffer);
            //delay to display the text for a while
            vTaskDelay(pdMS_TO_TICKS(10000));

            // Clearing the buffer with memset
            memset(send_buffer, 0, BUFFER_SIZE);
            //Setting buffer_index back to zero for next sending
            buffer_index = 0;

            //Clearing display:
            clear_display();
            // then change the state back to idle, for reading next values to be sent
            myState = IDLE;
            //Back to the imu task.
            xTaskNotifyGive(imuTaskHandle);
            //When state is receive:
        } else if (myState == RECEIVE) {
            char c;

            if (xQueueReceive(inputQueue, &c, 0) == pdTRUE) {

                if (c == '\n' || c == '\r') {
                    recv_buffer[recv_index] = '\0';

                    //Initialize buzzer
                    init_buzzer();
                    //Show what was received from user's input to terminal
                    printf("Received: %s\n", recv_buffer);
                    //iphone alarm played to show that the pico received the message
                    buzzer_play_melody(iphone_alarm);

                    // check if the received message is in morse
                    bool is_morse = true;
                    //char text[BUFFER_SIZE] = "";
                    //Go through the buffer character by character to check for non morse characters.
                    for (int i = 0; recv_buffer[i]; i++) {
                        if (recv_buffer[i] != '.' && recv_buffer[i] != '-' && recv_buffer[i] != ' ') {
                            is_morse = false;
                            //Once there is a single incorrect character there's no need to check the rest.
                            break;
                        }
                        //addChar(text, recv_buffer[i]);
                    }
                    /*if (strstr(text, "  .clear  ")) {
                            printf("helloooo");
                            //function for clearin the terminal
                            is_morse = true;
                            system("clear");
                    } else if (strstr(text, "  .stop  ")) {
                        printf("bye");
                        //function for stopping the program
                        is_morse = true;
                        exit(0);
                    }*/
                    //If no wrong characters are found text is written to pico lcd
                    if (is_morse) {
                        char decoded_text[BUFFER_SIZE] = {0};
                        //Making the morse code message into abcs writing.
                        decode_morse_message(recv_buffer, decoded_text);
                        //Printing the decoded text to terminal
                        printf("Decoded text: %s\n", decoded_text);
                        // display decoded text
                        write_text(decoded_text);
                        //Play the tones of the message
                        feedback(recv_buffer);
                        //Show message for some time
                        vTaskDelay(pdMS_TO_TICKS(10000));
                    } else {
                        // regular text
                        printf(recv_buffer);
                        //more delays to slow things down
                        vTaskDelay(pdMS_TO_TICKS(10000));
                    }

                    /*write_text(recv_buffer);
                    feedback(recv_buffer);
                    vTaskDelay(pdMS_TO_TICKS(10000));*/   // show message

                    //Clearing the recv_buffer
                    memset(recv_buffer, 0, BUFFER_SIZE);
                    //Resetting index for next time
                    recv_index = 0;

                    //clearing display
                    clear_display();
                    vTaskDelay(pdMS_TO_TICKS(200));
                }
                //if max index is not reached the charactes is added to the recv_buffer
                else if (recv_index < BUFFER_SIZE - 1) {
                    recv_buffer[recv_index++] = c;
                }
            }
    }
        //Necessary delays
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    
}
//Task for reading the imu
void imu_task(void *pvParameters) {
    (void)pvParameters;
    //Defining the imu reading values
    //a: acceleration
    //g: gyroscope
    //t: time
    float ax, ay, az, gx, gy, gz, t;
    // Setting up the sensor.
    if (init_ICM42670() == 0) {
        //Make sure everything is initialize correctly with terminal messaging.
        printf("ICM-42670P initialized successfully!\n");
        if (ICM42670_start_with_default_values() != 0){
            printf("ICM-42670P could not initialize accelerometer or gyroscope");
        }
    } else {
        printf("Failed to initialize ICM-42670P.\n");
    }
    //Initialize buzzer and led for pico
    init_buzzer();
    init_red_led();
    for(;;) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        printf("IMU activated");
        //Setting some basic variables to read movements.
        float peak_value = 0;
        bool peak_reached = false;
        bool accepted = false;
        //Eternal loop!
        while (1)
        {
            if (ICM42670_read_sensor_data(&ax, &ay, &az, &gx, &gy, &gz, &t) == 0) {
                //Making sure buffer is not full, 3 out of the buffer for sending
                if (buffer_index < (BUFFER_SIZE-3)) {
                    //Shaking horizontally movement:
                    //Values based on readings done
                    if (((fabs(gx) > 50 || fabs(gy) > 50 || fabs(gz) > 100) && fabs(ax) > 0.9) && (ax != -4.00 && ay != -4.00 && az != -4.00))  {
                        //printf("current values: gx %.2f, gy %.2f, gz %.2f \n", gx, gy, gz);
                        printf("dot\n");
                        //When detects the dot adds . to send_buffer
                        send_buffer[buffer_index++] = '.';
                        //Blinking red led 
                        blink_red_led(1);
                        //Buzzer of same tone for . as in feedback function to keep things clear
                        buzzer_play_tone(440,500);
                        //Making sure that detecting dot resets lifting values
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
                    //Detecting lifting (up and down) movement
                    //Values based on readings of data done
                    else if ((fabs(ay) > 0.15 && (fabs(gz) > 3.0 || fabs(gx) > 3.0 || fabs(gy) > 3.0) && (fabs(gz) < 100.0 && fabs(gx) < 50.0 && fabs(gy) < 50.0)) || accepted) {
                        //printf("hello\n");
                        //Make sure that once value is once accepted then smaller values of ay are accepted as lifting motion will be going down
                        accepted = true;
                        //variable for counting how far up the lifting went to try make sure small lifting with shaking is not mistakenly detected as liftinig
                        float distance_one = ay;
                        //after initial lift accepted, looking for when the pico reaches the highest it goes, the peak
                        if (fabs(ay) >= peak_value && !peak_reached) {
                            //printf("bigger than peak\n");
                            //Save last ay value to the peak_value to track when pico stops rising
                            peak_value = fabs(ay);
                            //printf("peak value: %.2f and ay: %.2f \n", peak_value, ay);
                        } else {
                            //Once ay doesn't go above the last ay value peak has been reached
                            //printf("peak reach\n");
                            peak_reached = true;
                            //printf("current values: gx %.2f, gy %.2f, gz %.2f \n", gx, gy, gz);
                        }
                        //Calculate the distance from highes peak and the start value.
                        float distance = peak_value - distance_one;
                        //printf("%.2f, %.2f, %.2f, %.2f,%.2f, %.2f \n", ax, ay, az, gx, gy, gz);
                        //printf("distance: %.2f", distance);
                        //if peak was reached, the distance is a certain value and the pico isn't moving much it prints dash
                        if (peak_reached && fabs(az - 1.0) < 0.2 && (fabs(gz) < 24.0) && distance > 0.3) {
                            printf("dash\n");
                            //Adding a dash to the send_buffer
                            send_buffer[buffer_index++] = '-';
                            //Blinking red led
                            blink_red_led(1);
                            //Playing the buzzr
                            buzzer_play_tone(800,200);
                            //Resetting the variables for next reading
                            peak_reached = false;
                            peak_value = 0;
                            accepted = false;
                        }          
                    }
                }
                //Track to see when buffer is full.
                else {
                    printf("buffer is full\n");
                }
                //printf("%.2f, %.2f, %.2f, %.2f,%.2f, %.2f \n", ax, ay, az, gx, gy, gz);
            //Let the user know if imu data couldn't be read.
            } else {
                printf("Failed to read imu data\n");
            }
            //Necessary delays
            vTaskDelay(pdMS_TO_TICKS(400));
            //Make sure the state is still idle since reading is only done in that state.
            if (myState != IDLE) { //maybe change it to if (myState != SEND)
                break;
            }
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
    //Initialize everything necessary
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
    //Play song for starting the program
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
    //Queue for the receiving state
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
