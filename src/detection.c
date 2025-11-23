#include "detection.h"
#include "main.h"
#include <math.h>
#include <stdio.h>

/* =========================
 *  DOT DETECTION
 * ========================= */
void dot_detection(float gx, float gy, float gz)
{
    printf("dot\n");
    send_buffer[buffer_index++] = '.';
    blink_red_led(1);
    buzzer_play_tone(440, 500);
}


/* =========================
 *  DASH DETECTION
 * ========================= */
void dash_detection(float ay, float az, float gz,
                    float *peak_value, bool *peak_reached, bool *accepted)
{
    *accepted = true;
    float distance_one = ay;

    // peak detection
    if (fabs(ay) >= *peak_value && !(*peak_reached)) {
        *peak_value = fabs(ay);
    } else {
        *peak_reached = true;
    }

    float distance = *peak_value - distance_one;

    if (*peak_reached &&
        fabs(az - 1.0f) < 0.2f &&
        fabs(gz) < 24.0f &&
        distance > 0.3f) {

        printf("dash\n");
        send_buffer[buffer_index++] = '-';
        blink_red_led(1);
        buzzer_play_tone(800, 200);

        *accepted = false;
        *peak_reached = false;
        *peak_value = 0;
    }
}

/* =========================
 *  SHAKE CHECK  (DOT)
 * =========================*/
bool shaking_check(float ax, float ay, float az, float gx, float gy, float gz)
{
    return ((fabs(gx) > 50 || fabs(gy) > 50 || fabs(gz) > 100)
             && fabs(ax) > 0.9
             && ax != -4.0 && ay != -4.0 && az != -4.0);
}

/* =========================
 *  LIFT CHECK  (DASH)
 * =========================*/
bool lifting_check(float ay, float gx, float gy, float gz)
{
    return (fabs(ay) > 0.15 &&
            (fabs(gz) > 3.0 || fabs(gx) > 3.0 || fabs(gy) > 3.0) &&
            fabs(gz) < 100.0 && fabs(gx) < 50.0 && fabs(gy) < 50.0);
}
