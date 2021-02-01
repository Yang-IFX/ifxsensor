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
 * @internal
 * @file Shapes.h
 *
 * @brief API for Shape configuration
 */

#ifndef IFX_RADAR_INTERNAL_SHAPES_H
#define IFX_RADAR_INTERNAL_SHAPES_H

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

/*
==============================================================================
   2. DEFINITIONS
==============================================================================
*/

/*
==============================================================================
   3. TYPES
==============================================================================
*/

/**
 * @internal
 * @brief Defines the structure for the Chirp settings.
 */
struct ifx_Device_Chirp_Settings_s
{
    uint32_t num_samples;
    uint32_t end_delay_ns;
};

/**
 * @internal
 * @brief Defines the structure for the Antenna setup.
 */
struct ifx_Device_Antenna_Setup_s
{
    uint8_t rx_mask;
    uint8_t tx_mode;
};

typedef struct ifx_Device_Chirp_Config_s
{
    struct ifx_Device_Chirp_Settings_s chirp_setting;
    struct ifx_Device_Antenna_Setup_s antenna_setup;
} ifx_Device_Chirp_Config_t;

typedef enum
{
    DIR_UPCHIRP_ONLY = 0, /**< All chirps of a frame
                                         start at the lower FMCW
                                         frequency and ramp up
                                         to the upper FMCW
                                         frequency. */
    DIR_DOWNCHIRP_ONLY = 1, /**< All chirps of a frame
                                         start at the upper FMCW
                                         frequency and ramp down
                                         to the lower FMCW
                                         frequency. */
    DIR_ALTERNATING_FIRST_UP = 2, /**< Only the odd chirps of
                                         a frame start at the
                                         lower RF frequency and
                                         ramp up to the upper
                                         FMCW frequency. The
                                         even chirps of a frame
                                         go from the upper to
                                         the lower FMCW
                                         frequency. */
    DIR_ALTERNATING_FIRST_DOWN = 3  /**< Only the even chirps of
                                         a frame start at the
                                         lower RF frequency and
                                         ramp up to the upper
                                         FMCW frequency. The odd
                                         chirps of a frame go
                                         from the upper to the
                                         lower RF frequency. */
} ifx_chirp_direction_t;


/**
 * @internal
 * @brief Defines the structure for the Shape configuration.
 */
struct ifx_Device_Shape_Config_s
{
    ifx_chirp_direction_t direction;
    uint32_t lower_frequency_kHz;
    uint32_t upper_frequency_kHz;
    uint8_t tx_power;

    struct ifx_Device_Chirp_Config_s up;
    struct ifx_Device_Chirp_Config_s down;
};

enum ifx_Device_Shape_Power_Mode
{
    IFX_SHAPE_POWER_MODE_STAY_ACTIVE = 0,
    IFX_SHAPE_POWER_MODE_GO_IDLE = 1,
    IFX_SHAPE_POWER_MODE_GO_DEEP_SLEEP = 2,
    IFX_SHAPE_POWER_MODE_GO_DEEP_SLEEP_CONTINUE = 3
};

/**
 * @internal
 * @brief Defines the structure for the Shape.
 */
struct ifx_Device_Shape_s
{
    uint16_t num_repetition;
    uint8_t following_power_mode;
    uint64_t end_delay_100ps;
    struct ifx_Device_Shape_Config_s config;
};

typedef struct ifx_Device_Shape_s ifx_Device_Shape_t;

/**
 * @internal
 * @brief Defines the structure for the Shape set configuration.
 *
 * The shape set consists of up to 4 shapes. Each shape can be activated by setting the num_repetition
 * parameter to > 0. Once the num_repetition of a shape is set to 0 the following shapes will
 * be ignored.
 */
struct ifx_Device_Shape_Set_s
{
    uint16_t num_repetition;
    uint8_t following_power_mode;
    uint64_t end_delay_100ps;

    struct ifx_Device_Shape_s shape[4];
};

typedef struct ifx_Device_Shape_Set_s ifx_Device_Shape_Set_t;

/*
==============================================================================
   4. FUNCTION PROTOTYPES
==============================================================================
*/

/**
 * @brief Configures radar sensor Shapes settings.
 *
 * @param [in]     handle    A handle to the radar device object.
 * @param [out]    shape_set Shape set configuration to apply.
 *
 */
IFX_DLL_PUBLIC
void ifx_device_configure_shapes(ifx_Device_Handle_t handle,
                                 ifx_Device_Shape_Set_t* shape_set);

#ifdef __cplusplus
} // extern "C"
#endif // __cplusplus

#endif /* IFX_RADAR_INTERNAL_SHAPES_H */
