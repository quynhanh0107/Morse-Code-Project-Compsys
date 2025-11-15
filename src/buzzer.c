//
//  buzzer.c
//  learnc
//
//  Created by Linh Lê on 15/11/25.
//

#include "buzzer.h"
#include "pico/stdlib.h"

void buzzer_init(uint pin) {
    gpio_init(pin);
    gpio_set_dir(pin, true);
}

void buzzer_play_tone(uint pin, int frequency, int duration_ms) {
    if(frequency == 0) {  // Rest
        sleep_ms(duration_ms);
        return;
    }

    int period_us = 1000000 / frequency;  // microseconds per cycle
    int cycles = (duration_ms * 1000) / period_us;

    for(int i = 0; i < cycles; i++) {
        gpio_put(pin, 1);
        sleep_us(period_us / 2);
        gpio_put(pin, 0);
        sleep_us(period_us / 2);
    }
}

void buzzer_play_melody(uint pin, int melody[][2]) {
    for(int i = 0; melody[i][1] != 0; i++) {
        buzzer_play_tone(pin, melody[i][0], melody[i][1]);
        sleep_ms(50);  // Short pause between notes
    }
}
