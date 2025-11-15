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

extern const morse_table_t morse_table[];
extern const size_t MORSE_TABLE_SIZE;

char morse_to_ascii(const char *morse);
void decode_morse_message(const char *morse, char *output);

#endif   // morse_translate_h
