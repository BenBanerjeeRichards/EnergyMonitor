#ifndef STRING_H 
#define STRING_H 

#define STR_OK 0
#define STR_ERR 1


void clampString(char* string, int max);

int intToString(char* string, int stringSize, int* stringOffset, int number, const char* fallback, int maxNumLength);

void centerString(char* outputString, int width, char* stringToCenter);

int startsWith(const char* prefix, char* str);

void readUntilNewline(char* str, int* position, char* value, int valueLength);

#endif 