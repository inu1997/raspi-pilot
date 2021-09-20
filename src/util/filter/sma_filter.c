#include "sma_filter.h"
#include <stdlib.h>
#include <string.h>

struct SMAFilter {
    float *buffer; // buffer for filtering.
    int index; // cycle index
    int length; // length of buffer
    bool has_cycled; // true if index has gone from 0 to length once.
};

/**
 * @brief Initiator of SMA Filter struct.
 * 
 * @param length
 *      Length of filter buffer.
 * @return Address of newly created SMAFilter struct
 */
struct SMAFilter *sma_init(int length) {
    struct SMAFilter *sma = malloc(sizeof(struct SMAFilter));
    
    sma->index = 0;

    sma->buffer = 0;
    
    sma_set_length(sma, length);

    sma->has_cycled = false;

    return sma;
}

/**
 * @brief Push new value into buffer.
 * 
 * @param sma
 *      The filter.
 * @param v
 *      The unfiltered value.
 * @return The new filtered value.
 */
float sma_update(struct SMAFilter *sma, float v) {
    sma->buffer[sma->index++] = v;
    if (sma->index >= sma->length) {
        sma->index = 0;
        sma->has_cycled = true;
    }
    
    int i;
    float sum = 0;

    for(i = 0; i < sma->length; i++) {
        sum += sma->buffer[i];
    }
    
    return sum / sma->length;
}

/**
 * @brief Clear the buffer and index.
 * 
 * @param sma
 *      The filter.
 */
void sma_reset(struct SMAFilter *sma){
    sma->index = 0;
    if (sma->buffer != 0) {
        memset(sma->buffer, 0, sma->length * sizeof(float));
    }
    sma->has_cycled = false;
}

/**
 * @brief Set the sma filter buffer length.
 * 
 * @param sma
 *      The filter.
 * @param length
 *      The length of buffer.
 * @return void
 */
void sma_set_length(struct SMAFilter *sma, int length){
    if (sma->buffer != 0) {
        // The old buffer exists.
        free(sma->buffer);
    }
    sma->length = length;
    
    sma->buffer = malloc(sma->length * sizeof(float));

    sma_reset(sma);
}

/**
 * @brief Getter of buffer length.
 * 
 * @param sma
 *      The filter.
 * @return length of buffer.
 */
int sma_get_length(struct SMAFilter *sma){
    return sma->length;
}

/**
 * @brief Check if the SMA filter buffer has cycled at least once.
 * 
 * @param sma 
 *      The SMA filter struct
 * @return true if it has cycled else false.
 */
bool sma_buffer_has_cycled(struct SMAFilter *sma) {
    return sma->has_cycled;
}