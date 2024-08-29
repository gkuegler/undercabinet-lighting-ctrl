#ifndef GRK_HELPER_HPP
#define GRK_HELPER_HPP

#define BYTE_0(x) ((x & 0x000000FF))
#define BYTE_1(x) ((x & 0x0000FF00) >> 8)
#define BYTE_2(x) ((x & 0x00FF0000) >> 16)
#define BYTE_3(x) ((x & 0xFF000000) >> 24)

#ifndef BIT64
#define BIT64(nr) (1ULL << (nr))
#endif

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define compare_types(T1, T2) _Generic(((T1){0}), T2 : true, default : false)

// Get number of elements in a static sized array.
// Will silently fail when a pointer is passed in.
#define ARRAY_LENGTH(arr) (sizeof(arr) / sizeof(arr[0]))

// Declare a parameter unused in a function body.
#define UNUSED(x) (void)(x)

void task_delay_ms(int ms);

#endif // GRK_HELPER_HPP