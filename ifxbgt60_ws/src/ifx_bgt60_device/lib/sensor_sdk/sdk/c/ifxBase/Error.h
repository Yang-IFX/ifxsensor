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
 * @file Error.h
 *
 * \brief \copybrief gr_error
 *
 * For details refer to \ref gr_error
 */

#ifndef IFX_BASE_ERROR_H
#define IFX_BASE_ERROR_H

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

/*
==============================================================================
   1. INCLUDE FILES
==============================================================================
*/

#include "ifxBase/Forward.h"
#include "ifxBase/Log.h"

/*
==============================================================================
   2. TYPES
==============================================================================
*/

#define IFX_ERROR_API_BASE      0x00010000
#define IFX_ERROR_DEV_BASE      0x00011000 
#define IFX_ERROR_APP_BASE      0x00020000

typedef enum ifx_Error_e
{
    // APIs related errors:
    // Any new API related errors should only be defined at the end of this block, using the next available offset from IFX_ERROR
    // This is to maintain backward compatibility of error codes on wrappers and GUI
    // If you add an error, make sure also to extend ifx_error_to_string.
    IFX_OK                                          = 0,                           /**< No error.*/
    IFX_ERROR                                       = IFX_ERROR_API_BASE,          /**< A generic error occurred in radar SDK API.*/
    IFX_ERROR_ARGUMENT_NULL                         = IFX_ERROR_API_BASE + 0x01,   /**< Argument Null error.*/
    IFX_ERROR_ARGUMENT_INVALID                      = IFX_ERROR_API_BASE + 0x02,   /**< Argument invalid error.*/
    IFX_ERROR_ARGUMENT_OUT_OF_BOUNDS                = IFX_ERROR_API_BASE + 0x03,   /**< Argument out of bounds.*/
    IFX_ERROR_ARGUMENT_INVALID_EXPECTED_REAL        = IFX_ERROR_API_BASE + 0x04,   /**< Argument invalid expected real.*/
    IFX_ERROR_ARGUMENT_INVALID_EXPECTED_COMPLEX     = IFX_ERROR_API_BASE + 0x05,   /**< Argument invalid expected complex.*/
    IFX_ERROR_INDEX_OUT_OF_BOUNDS                   = IFX_ERROR_API_BASE + 0x06,   /**< Index out of bounds.*/
    IFX_ERROR_DIMENSION_MISMATCH                    = IFX_ERROR_API_BASE + 0x07,   /**< Dimension mismatch.*/
    IFX_ERROR_MEMORY_ALLOCATION_FAILED              = IFX_ERROR_API_BASE + 0x08,   /**< Memory allocation failed.*/
    IFX_ERROR_IN_PLACE_CALCULATION_NOT_SUPPORTED    = IFX_ERROR_API_BASE + 0x09,   /**< In place calculation not supported.*/
    IFX_ERROR_MATRIX_SINGULAR                       = IFX_ERROR_API_BASE + 0x0A,   /**< Matrix is singular.*/
    IFX_ERROR_MATRIX_NOT_POSITIVE_DEFINITE          = IFX_ERROR_API_BASE + 0x0B,   /**< Matrix is not positive definite.*/
    IFX_ERROR_NOT_SUPPORTED                         = IFX_ERROR_API_BASE + 0x0C,   /**< Generic error for unsupported API.*/      
    IFX_ERROR_INTERNAL                              = IFX_ERROR_API_BASE + 0x0D,   /**< Generic internal logic error */

    // Device related errors:
    // Any new Device related errors should only be defined at the end of this block, using the next available 
    // offset from IFX_ERROR_NO_DEVICE. This is to maintain backward compatibility of error codes on wrappers and GUI    
    IFX_ERROR_NO_DEVICE                             = IFX_ERROR_DEV_BASE,           /**< No device compatible to Radar SDK was found.*/
    IFX_ERROR_DEVICE_BUSY                           = IFX_ERROR_DEV_BASE + 0x01,    /**< The connected device is busy and cannot
                                                                                          perform the requested action. This can happen
                                                                                          during device handle creation when the device
                                                                                          is in an undefined state. It is recommended to
                                                                                          unplug and replug the device.*/
    IFX_ERROR_COMMUNICATION_ERROR                   = IFX_ERROR_DEV_BASE + 0x02,    /**< The communication between host computer and
                                                                                          device is disturbed. This error is also
                                                                                          returned when the device sends an unexpected
                                                                                          error code.*/
    IFX_ERROR_NUM_SAMPLES_OUT_OF_RANGE              = IFX_ERROR_DEV_BASE + 0x03,    /**< The device does not support the requested
                                                                                          number of samples, because the requested
                                                                                          number is too high.*/
    IFX_ERROR_RX_ANTENNA_COMBINATION_NOT_ALLOWED    = IFX_ERROR_DEV_BASE + 0x04,    /**< The device does not support the requested
                                                                                          combination of RX antennas to be enabled.
                                                                                          This error typically occurs when a
                                                                                          non-existing antenna is requested to be
                                                                                          enabled.*/                                                
    IFX_ERROR_IF_GAIN_OUT_OF_RANGE                  = IFX_ERROR_DEV_BASE + 0x05,    /**< The device does not support the requested IF
                                                                                          gain, because the requested gain is either too
                                                                                          high or too low.*/
    IFX_ERROR_SAMPLERATE_OUT_OF_RANGE               = IFX_ERROR_DEV_BASE + 0x06,    /**< The device does not support the requested
                                                                                          sampling rate, because the requested rate is
                                                                                          either too high or too low.*/
    IFX_ERROR_RF_OUT_OF_RANGE                       = IFX_ERROR_DEV_BASE + 0x07,    /**< The requested FMCW start and end frequency are
                                                                                          not in the supported RF range of the device.*/
    IFX_ERROR_TX_POWER_OUT_OF_RANGE                 = IFX_ERROR_DEV_BASE + 0x08,    /**< The device does not support the requested TX
                                                                                          power, because the requested value is
                                                                                          too high.*/
    IFX_ERROR_CHIRP_RATE_OUT_OF_RANGE               = IFX_ERROR_DEV_BASE + 0x09,    /**< The requested chirp-to-chirp time cannot be
                                                                                          applied. This typically happens when the
                                                                                          requested time is shorter than the chirp
                                                                                          duration resulting from the specified sampling
                                                                                          rate and number of samples.*/
    IFX_ERROR_FRAME_RATE_OUT_OF_RANGE               = IFX_ERROR_DEV_BASE + 0x0A,    /**< The requested frame period cannot be applied.
                                                                                          This typically happens when the requested
                                                                                          period is shorter than the frame duration
                                                                                          resulting from the specified sampling
                                                                                          rate, number of samples and chirp-to-chirp
                                                                                          time.*/
    IFX_ERROR_NUM_CHIRPS_NOT_ALLOWED                = IFX_ERROR_DEV_BASE + 0x0B,    /**< The device does not support the requested
                                                                                          number of chirps per frame, because the
                                                                                          number is too high.*/
    IFX_ERROR_FRAME_SIZE_NOT_SUPPORTED              = IFX_ERROR_DEV_BASE + 0x0C,    /**< The device does not support the frame size
                                                                                          resulting from specified number of chirps,
                                                                                          number of samples and number of antennas.*/
    IFX_ERROR_TIMEOUT                               = IFX_ERROR_DEV_BASE + 0x0D,    /**< The device did not acquire a complete time
                                                                                          domain data frame within the expected time.
                                                                                         */
    IFX_ERROR_FIFO_OVERFLOW                         = IFX_ERROR_DEV_BASE + 0x0E,    /**< The device stopped acquisition of time domain
                                                                                          data due to an internal buffer overflow. This
                                                                                          happens when time domain data is acquired
                                                                                          faster than it is read from the device.*/
    IFX_ERROR_TX_ANTENNA_MODE_NOT_ALLOWED           = IFX_ERROR_DEV_BASE + 0x0F,    /**< The device does not support the requested
                                                                                          mode of TX antennas to be used.
                                                                                          This error typically occurs when a
                                                                                          the requested tx_mode is not supported by the
                                                                                          device due to non availability of TX antennas
                                                                                          for that mode.*/                                                                                                                      

    // App related errors:
    // Application errors are to be defined individually by the Apps, starting from this value.
    IFX_ERROR_APP                                   = IFX_ERROR_APP_BASE                    
} ifx_Error_t;

/// The pointer to callback function is saved in ifxRadar_Error.c
IFX_DLL_PUBLIC
extern void(*ifx_error_callback_pointer)(const char*, const char*, int, ifx_Error_t);

/*
==============================================================================
   3. DEFINITIONS
==============================================================================
*/

//----------------------------------------------------------------------------
// Macro definitions for logging error codes

#ifndef IFX_STDOUT
#define IFX_STDOUT stderr
#endif

#ifndef IFX_ERR_UNUSED
#define IFX_ERR_UNUSED(e) (void)(e)
#endif

/**
 * @brief Sets error code
 *
 * This macro sets the error to e and calls the callback function
 * if a callback function has been registered by \ref ifx_error_set_callback.
 */
#define ifx_error_set(e) do { \
        ifx_error_set_no_callback(e); \
        if((e) != IFX_OK && ifx_error_callback_pointer != NULL) \
            ifx_error_callback_pointer(__FILE__, __func__, __LINE__, (e)); \
    } while(0)

#define IFX_ERR_IS_ERROR(e) (e != IFX_OK)

//----------------------------------------------------------------------------
// Handler macros for calling some cleanup code in case an error occurs
// Useful while calling create methods of lower modules (like PPFFT) from the create method
// of a higher module (like range Doppler map)

#define IFX_ERR_HANDLE_R(stmt, cleanup)                            \
    do {                                                           \
        const ifx_Error_t old_error__ = ifx_error_get_and_clear(); \
        stmt;                                                      \
        const ifx_Error_t error__ = ifx_error_get();               \
        if(error__ != IFX_OK) {                                    \
            cleanup;                                               \
            return;                                                \
        }                                                          \
        ifx_error_set(old_error__);                                \
    } while(0)

#define IFX_ERR_HANDLE_E(stmt, cleanup)                            \
    do {                                                           \
        const ifx_Error_t old_error__ = ifx_error_get_and_clear(); \
        stmt;                                                      \
        const ifx_Error_t error__ = ifx_error_get();               \
        if(error__ != IFX_OK) {                                    \
            cleanup;                                               \
            return error__;                                        \
        }                                                          \
        ifx_error_set(old_error__);                                \
    } while(0)

#define IFX_ERR_HANDLE_N(stmt, cleanup)                            \
    do {                                                           \
        const ifx_Error_t old_error__ = ifx_error_get_and_clear(); \
        stmt;                                                      \
        const ifx_Error_t error__ = ifx_error_get();               \
        if(error__ != IFX_OK) {                                    \
            cleanup;                                               \
            return NULL;                                           \
        }                                                          \
        ifx_error_set(old_error__);                                \
    } while(0)

//----------------------------------------------------------------------------
// Condition check macros which set an error code.
// Useful for handling errors in various ways as follows:
// BRK=break,
// BRV=break with return value,
// BRF=break and goto a fail: code segment within the function,
// BRN=break with return null
#define IFX_ERR_DEBUG_MSG(cond, error_code) IFX_LOG_DEBUG("Condition failed: %s\n%s(%d)\n", #cond, ifx_error_to_string(error_code), error_code)

#define IFX_ERR_BRK_COND(cond, error_code)   \
    if(cond)                                 \
    {                                        \
        IFX_ERR_DEBUG_MSG(cond, error_code); \
        ifx_error_set(error_code);           \
        return;                              \
    }

#define IFX_ERR_BRV_COND(cond, error_code, v)\
    if(cond)                                 \
    {                                        \
        IFX_ERR_DEBUG_MSG(cond, error_code); \
        ifx_error_set(error_code);           \
        return (v);                          \
    }

#define IFX_ERR_BRF_COND(cond, error_code)   \
    if(cond)                                 \
    {                                        \
        IFX_ERR_DEBUG_MSG(cond, error_code); \
        ifx_error_set(error_code);           \
        goto fail;                           \
    }

#define IFX_ERR_BRN_COND(cond, error_code) \
    IFX_ERR_BRV_COND(cond, error_code, NULL)

//----------------------------------------------------------------------------
// Common applications of the Condition check macros

#define IFX_ERR_BRK_NULL(var)         IFX_ERR_BRK_COND(!(var), IFX_ERROR_ARGUMENT_NULL)
#define IFX_ERR_BRF_NULL(var)         IFX_ERR_BRF_COND(!(var), IFX_ERROR_ARGUMENT_NULL)
#define IFX_ERR_BRN_NULL(var)         IFX_ERR_BRN_COND(!(var), IFX_ERROR_ARGUMENT_NULL)
#define IFX_ERR_BRV_NULL(var, r)      IFX_ERR_BRV_COND(!(var), IFX_ERROR_ARGUMENT_NULL, r)

#define IFX_ERR_BRK_MEMALLOC(var)     IFX_ERR_BRK_COND(!(var), IFX_ERROR_MEMORY_ALLOCATION_FAILED)
#define IFX_ERR_BRF_MEMALLOC(var)     IFX_ERR_BRF_COND(!(var), IFX_ERROR_MEMORY_ALLOCATION_FAILED)
#define IFX_ERR_BRN_MEMALLOC(var)     IFX_ERR_BRN_COND(!(var), IFX_ERROR_MEMORY_ALLOCATION_FAILED)
#define IFX_ERR_BRV_MEMALLOC(var, r)  IFX_ERR_BRV_COND(!(var), IFX_ERROR_MEMORY_ALLOCATION_FAILED, r)

#define IFX_ERR_BRK_ARGUMENT(cond)    IFX_ERR_BRK_COND((cond), IFX_ERROR_ARGUMENT_INVALID)
#define IFX_ERR_BRN_ARGUMENT(cond)    IFX_ERR_BRN_COND((cond), IFX_ERROR_ARGUMENT_INVALID)
#define IFX_ERR_BRV_ARGUMENT(cond, r) IFX_ERR_BRV_COND((cond), IFX_ERROR_ARGUMENT_INVALID, r)

#define IFX_ERR_BRK_NEGATIVE(val)     IFX_ERR_BRK_COND((val) < 0, IFX_ERROR_ARGUMENT_OUT_OF_BOUNDS)
#define IFX_ERR_BRK_NEGATIVE0(val)    IFX_ERR_BRK_COND((val) <= 0, IFX_ERROR_ARGUMENT_OUT_OF_BOUNDS)

#define IFX_ERR_CHECK_BOUNDS(val, min, max) \
    ((val < min) || (val > max))

#define IFX_ERR_BRK_BOUNDS(val, min, max) \
    IFX_ERR_BRK_COND(IFX_ERR_CHECK_BOUNDS(val, min, max), IFX_ERROR_ARGUMENT_OUT_OF_BOUNDS)

#define IFX_ERR_BRK_LOWER(val, min) \
    IFX_ERR_BRK_COND((val < min), IFX_ERROR_ARGUMENT_OUT_OF_BOUNDS)

#define IFX_ERR_BRK_LOWER_INCL(val, min) \
    IFX_ERR_BRK_COND((val <= min), IFX_ERROR_ARGUMENT_OUT_OF_BOUNDS)

#define IFX_ERR_BRK_BIGGER(val, max) \
    IFX_ERR_BRK_COND((val > max), IFX_ERROR_ARGUMENT_OUT_OF_BOUNDS)

#define IFX_ERR_BRK_BIGGER_INCL(val, max) \
    IFX_ERR_BRK_COND((val >= max), IFX_ERROR_ARGUMENT_OUT_OF_BOUNDS)

/*
==============================================================================
   4. FUNCTION PROTOTYPES
==============================================================================
*/

/** @addtogroup gr_cat_SDK_base
  * @{
  */

/** @defgroup gr_error Error
  * @brief API for error handling
  *
  * Definitions of error codes and error handling macros.
  *
  * @{
  */

/**
 * @brief Sets error code
 *
 * This function only sets the error and does not call the callback function.
 *
 * Usually it is desired that when an error is set the error is also handed
 * to the callback function registered by \ref ifx_error_set_callback. In these
 * cases use the macro \ref ifx_error_set.
 *
 * In some situations, however, this behavior is not wanted. A typical example
 * is saving an error code and later restoring the saved error code.
 *
 * @param [in]     error     error code
 */
IFX_DLL_PUBLIC
void ifx_error_set_no_callback(ifx_Error_t error);

/**
 * @brief Get human-readable description of error code
 *
 * This function returns a human-readable description of the provided error
 * code. The string must not be modified or freed.
 *
 * @param [in]     error     error code
 * @retval         desc      human readable description of error
 */
IFX_DLL_PUBLIC
const char* ifx_error_to_string(ifx_Error_t error);

/**
 * @brief Clears errors
 *
 * Sets the last error to IFX_OK.
 */
IFX_DLL_PUBLIC
void ifx_error_clear(void);

/**
 * @brief Returns last error
 *
 * @retval error  last error code (may be IFX_OK)
 */
IFX_DLL_PUBLIC
ifx_Error_t ifx_error_get(void);

/**
 * @brief Returns and clear error
 *
 * Return the current error code and set the error code to IFX_OK.
 *
 * @retval error  last error code (may be IFX_OK)
 */
IFX_DLL_PUBLIC
ifx_Error_t ifx_error_get_and_clear(void);

/**
 * @brief Checks if an error is set
 *
 * @retval  1   if error != IFX_OK
 * @retval  0   if error == IFX_OK
 */
IFX_DLL_PUBLIC
int ifx_error_is_set(void);

/** @brief Registers callback function that gets called on errors
 *
 * Register callback as callback function. The function gets called whenever an
 * error occurs and before \ref ifx_error_set is called.
 *
 * The prototype of the callback is
 *      void callback(const char* filename, int line, ifx_Error_t error)
 * where filename and line are the name of the file and the line number where
 * the error occurred, and error is the error code. If callback is NULL, the old
 * callback will be unregistered.
 */
IFX_DLL_PUBLIC
void ifx_error_set_callback(void (callback)(const char* filename, const char* functionname, int line, ifx_Error_t error));

/**
  * @}
  */
 
/**
  * @}
  */ 

#ifdef __cplusplus
} // extern "C"
#endif // __cplusplus

#endif /* IFX_BASE_ERROR_H */
