/**
* \copyright
* MIT License
*
* Copyright (c) 2020 Infineon Technologies AG
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
* SOFTWARE
*
* \endcopyright
*
* \author Infineon Technologies AG
*
* \file Types.h
*
* \brief   Definitions of data types used within the SDK.
*
*
* @{
*/


#ifndef IFX_RADAR_TYPES_H
#define IFX_RADAR_TYPES_H

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

/*
==============================================================================
   1. INCLUDE FILES
==============================================================================
*/

#include <stdint.h>

/*
==============================================================================
   2. DEFINITIONS
==============================================================================
*/

/* On Windows symbols (functions) in a DLL (shared library) are not exported by
 * default. You have to tell the linker to export the symbol when you build the
 * library and you have to tell the linker (?) to import the symbol when using
 * the library. This is done with __declspec:
 *     Compiling the library: __declspec(dllexport) void foo(void);
 *     Using the library:     __declspec(dllimport) void foo(void);
 * More information can be found here:
 * https://stackoverflow.com/questions/33062728/cmake-link-shared-library-on-windows/41618677
 *
 * In contrast, on Linux the default is to export all symbols. For gcc
 * (probably also clang), see https://gcc.gnu.org/wiki/Visibility.
 *
 * The build system has to set correct preprocessor defines (cmake works).
 * - If the library is build as a static library, no preprocessor name is
 *   needed.
 * - If the library is build as a dynamic library:
 *      - Windows: radar_sdk_EXPORTS must be set when compiling the library
 *      - Windows: radar_sdk_EXPORTS must not be set when linking
 *      - Linux: No preprocessor defines are needed
 */
#ifdef RADAR_SDK_BUILD_STATIC
    // build as static library; no visibility
    #define IFX_DLL_PUBLIC
    #define IFX_DLL_HIDDEN
#elif defined(_MSC_VER) || defined(__MINGW64__) || defined(__WIN32__)
    // default visibility is hidden, so IFX_DLL_HIDDEN is a noop
    #define IFX_DLL_HIDDEN

    #ifndef IFX_DLL_PUBLIC
        #ifdef radar_sdk_EXPORTS
            // We are building this library
            #define IFX_DLL_PUBLIC __declspec(dllexport)
        #else
            // We are using this library
            #define IFX_DLL_PUBLIC __declspec(dllimport)
        #endif
    #endif
#elif (__GNUC__ >= 4) || (__clang_major__ >= 5)
    // see https://gcc.gnu.org/wiki/Visibility
    #define IFX_DLL_PUBLIC __attribute__ ((visibility ("default")))
    #define IFX_DLL_HIDDEN __attribute__ ((visibility ("hidden")))
#else
    #define IFX_DLL_PUBLIC
    #define IFX_DLL_HIDDEN
#endif

/** Speed of light in m/s */
#define IFX_LIGHT_SPEED_MPS ((ifx_Float_t)(299792458U))

/*
==============================================================================
   3. TYPES
==============================================================================
*/

#ifdef IFX_USE_DOUBLE
typedef double ifx_Float_t;
#else
typedef float ifx_Float_t;
#endif

/**
 * @brief Defines the structure for Complex data core parameters.
 *        Use type ifx_Complex_t for this struct.
 */
struct ifx_Complex_s
{
    ifx_Float_t data[2];
};

/**
 * @brief Defines the structure for Polar form. 
 *        Use type ifx_Polar_t for this struct.
 */
struct ifx_Polar_s
{
    ifx_Float_t radius;     /**< Radius.*/
    ifx_Float_t angle;      /**< Angle.*/
};

/*
==============================================================================
   4. FUNCTION PROTOTYPES
==============================================================================
*/

#ifdef __cplusplus
} // extern "C"
#endif // __cplusplus

#endif /* IFX_RADAR_TYPES_H */

/**
* @}
*/