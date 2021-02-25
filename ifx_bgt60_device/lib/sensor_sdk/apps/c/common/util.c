/**
* \copyright
* MIT License
*
* Copyright (c) 2021 Infineon Technologies AG
*
* Permission is hereby granted, free of charge, to any person obtaining a copy
* of this software and associated documentation files (the "Software"), to deal
* in the Software without restriction, including without limitation the rights
* to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
* copies of the Software, and to permit persons to whom the Software is
* furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included in all
* copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
* OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
* SOFTWARE.
*
* \endcopyright
*
* \author Infineon Technologies AG
*
* \file util.c
*
* \brief   This file implements the APIs to check if files exist and are readable.
*
*
* @{
*/


#ifdef _MSC_VER
#define _CRT_SECURE_NO_WARNINGS
#endif

/*
==============================================================================
   1. INCLUDE FILES
==============================================================================
*/

#include <ctype.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>

#ifdef _WIN32
#include <direct.h>
#include <io.h>
#include <windows.h>
#else
#include <sys/types.h>
#include <unistd.h>
#endif

#include "util.h"

/*
==============================================================================
   2. LOCAL DEFINITIONS
==============================================================================
*/

/*
==============================================================================
   3. LOCAL TYPES
==============================================================================
*/

/*
==============================================================================
   4. LOCAL DATA
==============================================================================
*/

/*
==============================================================================
   5. LOCAL FUNCTION PROTOTYPES
==============================================================================
*/

/*
==============================================================================
   6. LOCAL FUNCTIONS
==============================================================================
*/

/*
==============================================================================
   7. EXPORTED FUNCTIONS
==============================================================================
*/

void disable_buffering(FILE* fh)
{
    fflush(fh);
    setvbuf(fh, NULL, _IONBF, 0);
}

//----------------------------------------------------------------------------

bool get_matrix_from_file_r(FILE* f, ifx_Matrix_R_t* matrix)
{
    char temp[24];
    char* cur_pos = NULL;
    for (uint32_t i = 0; i < IFX_MAT_ROWS(matrix); i++) {
        for (uint32_t j = 0; j < IFX_MAT_COLS(matrix); j++) {
            do {
                cur_pos = fgets(temp, 24, f);
                if (cur_pos == NULL)
                    return false;
            } while ((strlen(cur_pos) <= 2) && isspace((unsigned char)*cur_pos));
            ifx_Float_t value = (ifx_Float_t)atof(temp);
            ifx_mat_set_element_r(matrix, i, j, value);
        }
    }
    return true;
}

//----------------------------------------------------------------------------

void print_matrix_to_file_r(FILE* f, ifx_Matrix_R_t* matrix)
{
    ifx_Float_t cur_val;
    for (uint32_t i = 0; i < IFX_MAT_ROWS(matrix); i++) {
        for (uint32_t j = 0; j < IFX_MAT_COLS(matrix); j++) {
            cur_val = ifx_mat_get_element_r(matrix, i, j);
            fprintf(f, "%f\n", cur_val);
        }
        fprintf(f, "\n");
    }
}

//----------------------------------------------------------------------------
char* extract_filename_from_path(char* filepath) {
    char *p = strrchr(filepath, PATH_SEPARATOR);

    // if the file does not have a path, called from the same directory
    if (p == NULL)
        return filepath;
    return(p + 1);
}

bool change_to_dirname(char* filepath)
{
    char* p = strrchr(filepath, PATH_SEPARATOR);

    // we are already in the correct directory
    if(p == NULL)
        return true;

    // find the 
    *p = '\0';
#ifdef _WIN32
    int ret = _chdir(filepath);
#else
    int ret = chdir(filepath);
#endif

    *p = PATH_SEPARATOR;

    if(ret == 0)
        return true;
    else
        return false;
}

//----------------------------------------------------------------------------

bool file_executable(const char* pathname)
{
#ifndef _WIN32
    if(access(pathname, R_OK | X_OK) == 0)
        return true;
    else
        return false;
#else
    return file_readable(pathname);
#endif
}

//----------------------------------------------------------------------------

bool file_readable(const char* pathname)
{
#ifdef _WIN32
    if(_access(pathname, 0) == 0)
        return true;
    else
        return false;
#else
    if(access(pathname, R_OK) == 0)
        return true;
    else
        return false;
#endif
}

//----------------------------------------------------------------------------

void* file_slurp(const char* pathname)
{
    struct stat filestatus;

    if (stat(pathname, &filestatus) != 0)
        return NULL;

    FILE* fp = fopen(pathname, "rb");
    if (fp == NULL)
        return NULL;

    void* content = malloc(filestatus.st_size+(size_t)1);
    if (content == NULL)
    {
        fclose(fp);
        return NULL;
    }

    size_t ret = fread(content, 1, filestatus.st_size, fp);
    fclose(fp);

    if (ret != (size_t)filestatus.st_size)
    {
        free(content);
        return NULL;
    }

    ((char *)content)[ret] = '\0';

    return content;
}

//----------------------------------------------------------------------------

bool get_uuid_from_string(const char* uuidString, uint8_t uuid[16])
{
    // internal temporary buffer
    uint8_t uuid_internal[32];

    // uuid must have at least 32 characters
    if (strlen(uuidString) < 32)
        return false;

    // uuid is valid
    if (!uuid)
        return false;
    // index for uuid_internal
    size_t j = 0;

    for (size_t i = 0; i < strlen(uuidString); i++)
    {
        const char c = tolower(uuidString[i]);

        // ignore hyphens
        if (c == '-')
            continue;

        // not a valid uuid string if not a hexadecimal character [0-9a-f]
        if (!isxdigit(c))
            return false;

        // read too many characters
        if (j >= 32)
            return false;

        // convert to integer
        switch (c)
        {
        case '0': uuid_internal[j++] = 0; break;
        case '1': uuid_internal[j++] = 1; break;
        case '2': uuid_internal[j++] = 2; break;
        case '3': uuid_internal[j++] = 3; break;
        case '4': uuid_internal[j++] = 4; break;
        case '5': uuid_internal[j++] = 5; break;
        case '6': uuid_internal[j++] = 6; break;
        case '7': uuid_internal[j++] = 7; break;
        case '8': uuid_internal[j++] = 8; break;
        case '9': uuid_internal[j++] = 9; break;
        case 'a': uuid_internal[j++] = 10; break;
        case 'b': uuid_internal[j++] = 11; break;
        case 'c': uuid_internal[j++] = 12; break;
        case 'd': uuid_internal[j++] = 13; break;
        case 'e': uuid_internal[j++] = 14; break;
        case 'f': uuid_internal[j++] = 15; break;
        }
    }

    if (j != 32)
        return false;

    // copy to uuid
    for (size_t i = 0; i < 16; i++)
        uuid[i] = uuid_internal[2 * i + 0] << 4 | uuid_internal[2 * i + 1];

    return true;
}

/**
* @}
*/
