//
//  morse_translate.c
//  learnc
//
//  Created by Linh Lê on 15/11/25.
//

#include "morse_translate.h"

const size_t MORSE_TABLE_SIZE = sizeof(morse_table) / sizeof(morse_table[0]);

// This functon converts a morse symbol into a letter, if nothing matches then it returns a "?"
char morse_to_ascii(const char *morse) {
    for (size_t i = 0; i < MORSE_TABLE_SIZE; i++)
        if (strcmp(morse, morse_table[i].morse) == 0)
            return morse_table[i].ascii;

    return '?';
}

// This function takes a morse code message and turns it into normal text
void decode_morse_message(const char *morse, char *output) {

    char letter[10] = {0};   // stores one morse letter at a time
    int current_morse_length = 0;      // how many characters are inside "letter"
    int out_pos = 0;         // where we are in the output text

    // go through every character in the Morse message
    for (int i = 0; ; i++) {

        char c = morse[i];   // current character ('.' or '-' or ' ' or '\0')

        // if we see a dot or dash, it is part of a morse letter
        if (c == '.' || c == '-') {

            letter[current_morse_length] = c;   // store it in the small buffer
            current_morse_length++;             // move to next position
            letter[current_morse_length] = '\0';
        }

        // if we see a space or end of the whole string, the morse letter is finished
        else if (c == ' ' || c == '\0') {

            // only decode if we actually collected a symbol
            if (current_morse_length > 0) {

                // convert the morse symbol into a letter
                output[out_pos] = morse_to_ascii(letter);
                out_pos++;

                // reset for the next symbol
                current_morse_length = 0;
                letter[0] = '\0';
            }

            // if we hit the end of the whole message, stop
            if (c == '\0') {
                break;
            }
        }
    }
    // end the output string
    output[out_pos] = '\0';
}
