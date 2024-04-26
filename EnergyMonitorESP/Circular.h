#ifndef CIRCULAR_H 
#define CIRCULAR_H

#define CAPACITY 100

struct circular_buffer {
    volatile unsigned int buffer[CAPACITY];
    volatile unsigned int writeIndex;
    volatile unsigned int size;
};

struct circular_buffer circular_init();

struct circular_buffer circular_add(struct circular_buffer buf, unsigned int item);

unsigned int circular_get(struct circular_buffer buf, int idx);

#endif 