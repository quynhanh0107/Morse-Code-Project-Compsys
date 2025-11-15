//
//  morse_translate.h
//  learnc
//
//  Created by Linh Lê on 15/11/25.
//

#ifndef morse_translate_h
#define morse_translate_h

#include <stdio.h>
#include <string.h>

typedef struct {
  char ascii;
  char *morse;
} morse_table_t;

const morse_table_t morse_table[] = {
    {'A', ".-"},    {'B', "-..."},  {'C', "-.-."},  {'D', "-.."},
    {'E', "."},     {'F', "..-."},  {'G', "--."},   {'H', "...."},
    {'I', ".."},    {'J', ".---"},  {'K', "-.-"},   {'L', ".-.."},
    {'M', "--"},    {'N', "-."},    {'O', "---"},   {'P', ".--."},
    {'Q', "--.-"},  {'R', ".-."},   {'S', "..."},   {'T', "-"},
    {'U', "..-"},   {'V', "...-"},  {'W', ".--"},   {'X', "-..-"},
    {'Y', "-.--"},  {'Z', "--.."},  {'1', ".----"}, {'2', "..---"},
    {'3', "...--"}, {'4', "....-"}, {'5', "....."}, {'6', "-...."},
    {'7', "--..."}, {'8', "---.."}, {'9', "----."}, {'0', "-----"}};

char morse_to_ascii(const char *morse);
void decode_morse_message(const char *morse, char *output);

#endif   // morse_translate_h
