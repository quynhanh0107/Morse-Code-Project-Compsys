#ifndef DETECTION_H
#define DETECTION_H

#include <stdbool.h>

void dot_detection(float gx, float gy, float gz);

void dash_detection(float ay, float az, float gz,
                    float *peak_value, bool *peak_reached, bool *accepted);

bool shaking_check(float ax, float ay, float az,
                   float gx, float gy, float gz);

bool lifting_check(float ay, float gx, float gy, float gz);

#endif
