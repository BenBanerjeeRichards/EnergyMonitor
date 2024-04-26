#include "StringUtil.h"
#include <cstring>
#include <cstdio>

void clampString(char* string, int max) {
  if (strlen(string) > max) {
    // Two dots to save on space
    string[max-2] = '.';
    string[max-1] = '.';
    string[max] = '\0';
  }
}

int intToString(char* string, int stringSize, int* stringOffset, int number, const char* fallback, int maxNumLength) {
    int sizeNeeded = snprintf(string+*stringOffset, stringSize, "%d", number);
    if (sizeNeeded > maxNumLength) {
        strcpy(string+*stringOffset, fallback);
        *stringOffset += strlen(fallback);
        return 1;
    } else {
        *stringOffset += sizeNeeded;
        return 0;
    }
}

void centerString(char* outputString, int width, char* stringToCenter) {
  int inputLen = strlen(stringToCenter);
  int diff = width - inputLen;
  if (diff < 0) {
    return;
  }
  int leftPad = diff / 2;
  for (int i = 0; i < leftPad; i++) {
    strlcpy(outputString+i, " ", width);
  }
  strlcpy(outputString+leftPad, stringToCenter, width);
}
