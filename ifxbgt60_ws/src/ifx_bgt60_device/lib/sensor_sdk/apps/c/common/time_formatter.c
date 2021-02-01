/* ===========================================================================
** Copyright (C) 2019-2020 Infineon Technologies AG. All rights reserved.
** ===========================================================================
**
** ===========================================================================
** Infineon Technologies AG (INFINEON) is supplying this file for use
** exclusively with Infineon's sensor products. This file can be freely
** distributed within development tools and software supporting such 
** products.
** 
** THIS SOFTWARE IS PROVIDED "AS IS".  NO WARRANTIES, WHETHER EXPRESS, IMPLIED
** OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
** MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.
** INFINEON SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR DIRECT, INDIRECT, 
** INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES, FOR ANY REASON 
** WHATSOEVER.
** ===========================================================================
*/
/**
 * @file time_formatter.c
 *
 * @brief This file implements the API for time formatting.
 *
 */

/*
==============================================================================
   1. INCLUDE FILES
==============================================================================
*/

#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#if defined(WIN32) || defined(_WIN32) || defined(__WIN32)
#include <windows.h>
#else
#include <sys/time.h>
#endif
#include "time_formatter.h"

/*
==============================================================================
   2. LOCAL DEFINITIONS
==============================================================================
*/

#ifdef _MSC_VER
#pragma warning(disable : 4996)
#endif

#define NUM_D_DIGITS 3
#define NUM_H_DIGITS 2
#define NUM_M_DIGITS 2
#define NUM_S_DIGITS 2
#define NUM_MS_DIGITS 3
#define NUM_ADDITIONAL_CHARS 6

#define STR(X) #X
#define XSTR(X) STR(X)
#define TIME_FORMAT "%0" XSTR(NUM_H_DIGITS) "d:%0" XSTR(NUM_M_DIGITS) \
                   "d:%0" XSTR(NUM_S_DIGITS) "d.%0" XSTR(NUM_MS_DIGITS) "d"

/*
==============================================================================
   3. LOCAL TYPES
==============================================================================
*/
struct ifx_Time_s
{

#if defined(WIN32) || defined(_WIN32) || defined(__WIN32)
    LARGE_INTEGER freq;
    LARGE_INTEGER start;
    LARGE_INTEGER end;
#else
    struct timeval start;
    struct timeval end;
#endif
    clock_t start_clock;
    clock_t end_clock;
    char *cur_time_str;
    size_t cur_time_size;
    ifx_Float_t cur_time_s;

};

/*
==============================================================================
   4. DATA
==============================================================================
*/

/*
==============================================================================
   5. LOCAL FUNCTION PROTOTYPES
==============================================================================
*/

static void ifx_time_init(ifx_Time_Handle_t handle);

/*
==============================================================================
  6. LOCAL FUNCTIONS
==============================================================================
*/

static void ifx_time_init(ifx_Time_Handle_t handle)
{
#if defined(WIN32) || defined(_WIN32) || defined(__WIN32)
    QueryPerformanceFrequency(&handle->freq);
    QueryPerformanceCounter(&handle->start);
#else
    gettimeofday(&handle->start, NULL);
#endif
}

/*
==============================================================================
   7. EXPORTED FUNCTIONS
==============================================================================
*/

ifx_Error_t ifx_time_create(ifx_Time_Handle_t* handle)
{
    *handle = malloc(sizeof(struct ifx_Time_s));

    if(*handle == NULL)
        return -1;

    ifx_time_init(*handle);

    (*handle)->cur_time_size = NUM_D_DIGITS + 2 + NUM_H_DIGITS +
            NUM_M_DIGITS + NUM_S_DIGITS +
            NUM_MS_DIGITS + NUM_ADDITIONAL_CHARS;
    (*handle)->cur_time_str = calloc(1, (*handle)->cur_time_size * sizeof(char));

    if((*handle)->cur_time_str == NULL)
        return -1;

    return 0;
}

//----------------------------------------------------------------------------

int64_t ifx_time_get_ms(ifx_Time_Handle_t handle)
{
    int64_t elapsed = 0;
#if defined(WIN32) || defined(_WIN32) || defined(__WIN32)
    QueryPerformanceCounter(&handle->end);
    elapsed = ((handle->end.QuadPart - handle->start.QuadPart) * 1000) / handle->freq.QuadPart;

#else
    gettimeofday(&handle->end, NULL);
    elapsed = (handle->end.tv_sec - handle->start.tv_sec) * 1000;
    elapsed += (handle->end.tv_usec - handle->start.tv_usec) / 1000;
#endif
    return elapsed;
}

//----------------------------------------------------------------------------

char* ifx_time_get_cstr(ifx_Time_Handle_t handle)
{
    int h, m, s, ms, d;
    int64_t time_ms = ifx_time_get_ms(handle);

    int64_t time_sec = time_ms / 1000;
    int64_t time_min = time_sec / 60;
    int64_t time_h = time_min / 60;
    ms =(int) (time_ms % 1000);
    s = (int) (time_sec % 60);
    m = (int) (time_min % 60);
    h = (int) (time_h % 24);
    d = (int) (time_h / 24);

    if(d > 999)
        d  = 0;

    if(d)
        sprintf(handle->cur_time_str, "%dd "TIME_FORMAT, d, h, m, s, ms);
    else
        sprintf(handle->cur_time_str, TIME_FORMAT, h, m, s, ms);

    return handle->cur_time_str;
}

//----------------------------------------------------------------------------

void ifx_time_destroy(ifx_Time_Handle_t handle)
{
    if(handle == NULL)
        return;

    free(handle->cur_time_str);
    free(handle);
}
