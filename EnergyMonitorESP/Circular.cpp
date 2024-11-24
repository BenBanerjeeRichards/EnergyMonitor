#include "Circular.h"

int mod(int a, int b) {
    int r = a % b;
    return r < 0 ? r + b : r;
}

struct circular_buffer circular_init() {
    struct circular_buffer buf;
    buf.writeIndex = 0;
    buf.size = 0;
    return buf;
}

void circular_add(struct circular_buffer* buf, unsigned int item) {
    buf->buffer[buf->writeIndex] = item;
    buf->writeIndex = mod((buf->writeIndex + 1), CAPACITY);

    if (buf->size < CAPACITY) {
        buf->size++;
    }
}

// Get items where index 0 is latest item added to buffer and CAPACITY-1 is the last item
unsigned int circular_get(struct circular_buffer* buf, int idx) {
    return buf->buffer[mod((buf->writeIndex - (idx + 1)),CAPACITY)];
}

