//
//  buzzer.c
//  learnc
//
//  Created by Linh Lê on 15/11/25.
//

#include "buzzer.h"
#include "pico/stdlib.h"
#include "tkjhat/sdk.h"

void buzzer_play_melody(int melody[][2]) {
    for (int i = 0; melody[i][1] != 0; i++) {
        buzzer_play_tone(melody[i][0], melody[i][1]);
        sleep_ms(50);  // short pause between notes
    }
}
