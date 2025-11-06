
#include <stdio.h>
#include <string.h>

#include <pico/stdlib.h>

#include <FreeRTOS.h>
#include <queue.h>
#include <task.h>

#include "tkjhat/sdk.h"

#include <hardware/gpio.h>
#include <hardware/i2c.h>



#define BUFFER_SIZE 40

#define TILT_THRESHOLD 
#define FLAT_THRESHOLD 
#define SHAKE_THRESHOLD 


typedef enum IMU_states {
    STATE_IDLE = 1,    // In-between positions
    STATE_DOT,     // Being shaken 
    STATE_DASH,    // Tilted 90 degrees
    STATE_SPACE    // Flat on table
} imu_event;

//static enum IMU_states last_state = STATE_IDLE;
//static int space_count = 0; 

/*
// Introducing state
enum state {IDLE=1, READ_SENSOR, UPDATE, NEW_MSG};

// Global state variable, initialized to waiting state
enum state myState = IDLE;

// Timer interrupt once per second
void clkFxn(void *pvParameters) {
    // We change the state to a desired one
    // If-clause is used to check, that the state transition is possible
    // Now we allow only the state transition IDLE -> READ_SENSOR
    if (myState == IDLE) {
    
        // State transition IDLE -> READ_SENSOR
        myState = READ_SENSOR;
    }
}

// Communications task
void commTask(void *pvParameters) {

    while (1) { 
        // Function is_message_waiting is used to check
        // are there new messages in the buffer
        // Additionally, we allow only the state transition IDLE -> NEW_MSG
        if (is_message_waiting() == TRUE && myState == IDLE) {

            // State transition IDLE -> NEW_MSG
            myState = NEW_MSG;
        
            // Functionality of state
            handle_message();
            send_reply();			        
            
            // State transition NEW_MSG -> IDLE
            myState = IDLE;
        }
    }
}

// Handling sensors
void sensorTask(void *pvParameters) {
   
    //Start the I2C
    init_i2c_default();

    while (1) {
    
        if (myState == READ_SENSOR) {
        
            // Functionality of state
            read_sensor_values();
            
            // State transition READ_SENSOR -> UPDATE
            myState = UPDATE;				        
        }
    
        vTaskDelay(..);
    }
}

// Handling display update
void displayTask(void *pvParameters) {

    while (1) {
    
        if (myState == UPDATE) {
        
            // Functionality of state
            update_screen();
            
            // State transition UPDATE -> IDLE
            myState = IDLE;				        
        }
    
        vTaskDelay(..);
    }
}
*/

void imu_task(void *pvParameters) {
    (void)pvParameters;
    
    float ax, ay, az, gx, gy, gz, t;
    // Setting up the sensor. 
    if (init_ICM42670() == 0) {
        printf("ICM-42670P initialized successfully!\n");
        if (ICM42670_start_with_default_values() != 0){
            printf("ICM-42670P could not initialize accelerometer or gyroscope");
        }
        /*int _enablegyro = ICM42670_enable_accel_gyro_ln_mode();
        usb_serial_print ("Enable gyro: %d\n",_enablegyro);
        int _gyro = ICM42670_startGyro(ICM42670_GYRO_ODR_DEFAULT, ICM42670_GYRO_FSR_DEFAULT);
        usb_serial_print ("Gyro return:  %d\n", _gyro);
        int _accel = ICM42670_startAccel(ICM42670_ACCEL_ODR_DEFAULT, ICM42670_ACCEL_FSR_DEFAULT);
        usb_serial_print ("Accel return:  %d\n", _accel);*/
    } else {
        printf("Failed to initialize ICM-42670P.\n");
    }
    // Start collection data here. Infinite loop. 
    uint8_t buf[BUFFER_SIZE];
    while (1)
    {
        if (ICM42670_read_sensor_data(&ax, &ay, &az, &gx, &gy, &gz, &t) == 0) {

            /*enum IMU_states current_state = STATE_IDLE;

            //*******Determine the current state based on thresholds*******
            // STATE_SPACE: flat position
            if (az > FLAT_THRESHOLD) {
                current_state = STATE_SPACE;

            }
            //STATE_DASH: rotating 90 degrees 
            else if (ay > TILT_THRESHOLD) {
                current_state = STATE_DASH;
            }
            //STATE_DOT: shaking
            else if (fabs(gx) > SHAKE_THRESHOLD || fabs(gy) > SHAKE_THRESHOLD || fabs(gz) > SHAKE_THRESHOLD ) {
                current_state = STATE_DOT;
            }

            //*******Execute only if state changes*******
            if (current_state != last_state) {
                switch (current_state) {
                    case STATE_DOT:
                        printf(".");
                        space_count = 0; // reset space count 
                        break;
                    case STATE_DASH:
                        printf("-");
                        space_count = 0;
                        break;
                    case STATE_SPACE: 
                        printf("");
                        space_count ++;
                        break;
                    case STATE_IDLE:
                        break;

                }
                if (space_count >= 3) {
                    printf("End of message");
                    space_count = 0;
                }
            }
            last_state = current_state;
           */
            printf("Accel: X=%.2f, Y=%.2f, Z=%.2f | Gyro: X=%.2f, Y=%.2f, Z=%.2f \n", ax, ay, az, gx, gy, gz);

        } else {
            printf("Failed to read imu data\n");
        }
        vTaskDelay(pdMS_TO_TICKS(60));
    }

}





int main() {
    stdio_init_all();
    sleep_ms(2000); //Wait to see the output.
    init_hat_sdk();
    while (!stdio_usb_connected()){
        sleep_ms(10);
    } 
    printf("Start tests\n");
    
    // Initialize LED
    init_red_led();
    printf("Initializing red led\r\n");
    
    //Testing RED LED
    // printf("Testing red led should be on\r\n");
   


    // Test SW1 
    //init_sw1();
    //printf("Initializing switch 1\r\n");

    //Test SW2
    //init_sw2();
    //printf("Initializing switch 2\r\n");

    //Test RGB
    //init_rgb_led();
    //printf("Initializing RGB LED\r\n");


   // Initialize Buzzer
   //init_buzzer();
   //printf("Initializing the buzzer\n");

   
    // Initialize I2C
    init_i2c_default();
    printf("Initializing the i2c\n");

    //Initialize Light Sesnsor VEML6030
    //veml6030_init();
    //printf("Initializing the light sensor\n");

    //Initialize the Temp and Humidity Sensor
    //hdc2021_init();
    //printf("Initializing the temp/humidity sensor\n");

    //Initialize the display
    //init_display();
    //printf("Initializating display\n");

    //Initialize the microphone
    //Microhpone test in test_microphone.c

    //Initialize IMU
    if (init_ICM42670() == 0) {
        printf("ICM-42670P initialized successfully!\n");
        if (ICM42670_startAccel(ICM42670_ACCEL_ODR_DEFAULT, ICM42670_ACCEL_FSR_DEFAULT) != 0){
            printf("Wrong values to init the accelerometer in ICM-42670P.\n");
        }
        if (ICM42670_startGyro(ICM42670_GYRO_ODR_DEFAULT, ICM42670_GYRO_FSR_DEFAULT) != 0){
            printf("Wrong values to init the gyroscope in ICM-42670P.\n");
        };
        ICM42670_enable_accel_gyro_ln_mode();
    } else {
        printf("Failed to initialize ICM-42670P.\n");
    }

        



    // Create tasks
    
    // xTaskCreate(sw2_task, "SW2Task", 256, NULL, 1, NULL);
    // xTaskCreate(rgb_task, "RGBTask", 256, NULL, 1, NULL);
    // xTaskCreate(buzzer_task, "BuzzerTask", 256, NULL, 4, NULL);
    // xTaskCreate(light_sensor_task, "LightSensorTask", 256, NULL, 3, NULL);
    // xTaskCreate(ths_task, "THSTask", 256, NULL, 1, NULL);
    xTaskCreate(imu_task, "IMUTask", 256, NULL, 1, NULL);
    // xTaskCreate(led_simple_task, "LEDSimpleTask", 64, NULL, 1, NULL);
    // xTaskCreate(sw1_task, "SW1Task", 64, NULL, 1, NULL);
    // xTaskCreate(led_task, "LEDTask", 64, NULL, 2, NULL);
 

    //  xTaskCreate(display_task, "DisplayTask", 256, NULL, 2, NULL);
    // // xTaskCreate(mic_task, "MicTask", 256, NULL, 1, NULL);
    
    // Start the FreeRTOS scheduler
    vTaskStartScheduler();

    return 0;
}
