#include <stdint.h>
#include "common.h"

// functions

void Split_to_Bytes(uint32_t value, uint8_t *bytes){
	*(bytes+0) = (value >> 24) & 0xFF;  // Most Significant Byte (MSB)
    *(bytes+1) = (value >> 16) & 0xFF;
    *(bytes+2) = (value >> 8)  & 0xFF;
    *(bytes+3) = value         & 0xFF;  // Least Significant Byte (LSB)
}

uint32_t Assemble_from_Bytes(uint8_t *bytes) {
    return ((uint32_t)*(bytes+0) << 24) |
           ((uint32_t)*(bytes+1) << 16) |
           ((uint32_t)*(bytes+2) << 8)  |
           ((uint32_t)*(bytes+3));
}
