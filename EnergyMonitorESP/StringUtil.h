#ifndef STRING_H 
#define STRING_H 

void clampString(char* string, int max);

int intToString(char* string, int stringSize, int* stringOffset, int number, const char* fallback, int maxNumLength);

void centerString(char* outputString, int width, char* stringToCenter);

#endif 