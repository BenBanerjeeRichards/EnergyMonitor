#ifndef CIRCULAR_H 
#define CIRCULAR_H

// TODO refine below
// this is how large the capacity is
#define CAPACITY 50

struct circular_buffer {
    volatile unsigned int buffer[CAPACITY] = {0};
    volatile unsigned int writeIndex;
    volatile unsigned int size;
};

struct circular_buffer circular_init();

void circular_add(struct circular_buffer* buf, unsigned int item);

unsigned int circular_get(struct circular_buffer* buf, int idx);

int mod(int a, int b);

#endif 