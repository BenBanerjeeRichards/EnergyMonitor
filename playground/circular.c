#include <stdio.h>
#include <string.h>

const int CAPACITY = 5;
int mod(int a, int b) {
    int r = a % b;
    return r < 0 ? r + b : r;
}


struct circular_buffer {
    unsigned int buffer[CAPACITY];
    unsigned int writeIndex;
    unsigned int size;
};

void write(struct circular_buffer buffer);

struct circular_buffer circular_init() {
    struct circular_buffer buf;
    buf.writeIndex = 0;
    buf.size = 0;
    return buf;
}

struct circular_buffer circular_add(struct circular_buffer buf, unsigned int item) {
    buf.buffer[buf.writeIndex] = item;
    buf.writeIndex = mod((buf.writeIndex + 1), CAPACITY);

    if (buf.size < CAPACITY) {
        buf.size++;
    }
    return buf;
}


// Get items where index 0 is latest item added to buffer and CAPACITY-1 is the last item
unsigned int circular_get(struct circular_buffer buf, int idx) {
    return buf.buffer[mod((buf.writeIndex - (idx + 1)),CAPACITY)];
}

void test_assert(int condition, const char* message) {
    if (!condition) {
        printf("ASSERT FAILED: %s\n", message);
    }
}


#define VERSION "v0.0.1"
const char str[1024];


void test() {
    struct circular_buffer buf = circular_init();
    test_assert(buf.size == 0, "initial size is 0");
    test_assert(buf.writeIndex == 0, "initial startIndex is 0");

    buf = circular_add(buf, 10);
    test_assert(circular_get(buf, 0) == 10, "get first element is 10");
    buf = circular_add(buf, 20);
    buf = circular_add(buf, 30);
    buf = circular_add(buf, 40);
    buf = circular_add(buf, 50);

    test_assert(circular_get(buf, 0) == 50, "get first element now 50");
    test_assert(circular_get(buf, 1) == 40, "get second element is 40");
    test_assert(circular_get(buf, 4) == 10, "get last element is 10");
    buf = circular_add(buf, 60);

    test_assert(circular_get(buf, 0) == 60, "first element is now 60");
    test_assert(circular_get(buf, 1) == 50, "next item now is 50");

    test_assert(circular_get(buf, 4) == 20, "final item is now 20");
    
    buf = circular_add(buf, 70);

    test_assert(circular_get(buf, 0) == 70, "first becomes 70");
    test_assert(circular_get(buf, 1) == 60, "next still 60");
    test_assert(circular_get(buf, 4) == 30, "final item is now 30");


    write(buf);
    printf("%s\n", str);

}


void append(int *idx, const char* toAppend, int bufferSize) {
    strlcpy(str+*idx, toAppend, bufferSize);
    *idx += strlen(toAppend);
}

void writeKeyValue(int* idx, const char* key, const char* val, int bufferSize) {
    append(idx, key, bufferSize);
    append(idx, "=", bufferSize);
    append(idx, val, bufferSize);
    append(idx, ";", bufferSize);
}

void write(struct circular_buffer buffer) {
    const int N = 1024;
    int writeIndex = 0;
    writeKeyValue(&writeIndex,"version", VERSION, N);
    writeKeyValue(&writeIndex,"hello", "world", N);
    
    char sizeStr[8];
    sprintf(sizeStr, "%d", buffer.size);
    writeKeyValue(&writeIndex,"size", sizeStr, N);
    append(&writeIndex, "values=", N);
    for (int i =0; i < buffer.size; i++) {
        sprintf(sizeStr, "%d", circular_get(buffer, i));
        append(&writeIndex, sizeStr, N);
        append(&writeIndex, ",", N);
    }

}

void lcdCurrentUsage(int watts, int todayPounds, int todayPence) {
  char wattsStr[6]; 
  char poundsStr[5]; 
  char penceStr[2];
  int wattsSize = 0;
  if (watts > 99999) {
    strcpy(wattsStr, ">9999");
    wattsSize = 6;
  } else {
    wattsSize = snprintf(wattsStr, 6, "%d", watts);
  }

  // poundsStr = "--";
  // penceStr = "--";
  char topLine[16];
  const int leftSpaces = (16 - (3 + 1 + wattsSize + 1)) / 2; // Now(1) <space>(1) <Watts> W<2>
  int position =0;

  for (int i = 0; i < leftSpaces; i++) {
    strlcpy(topLine+position, " ", 16);
    position += 1;
  }
  strlcpy(topLine+position, "Now ", 16);
  position += 4;
  strlcpy(topLine+position, wattsStr, 16);
  position += wattsSize;
  strlcpy(topLine+position, "W", 16);
}

int intToString(const char* string, int stringSize, int* stringOffset, int number, const char* fallback, int maxNumLength) {
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


int main(int argc, char** args) {
    // test();
    lcdCurrentUsage(100230230, 20 ,10);

    char test[100];
    int offset = 0;
    intToString(test, 100, &offset, 1000, ">999", 3);

    printf("%s\n%d\n", test, offset);
    return 0;
}