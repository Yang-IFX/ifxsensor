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
 * @file raw_data_defaults.h
 *
 * @brief This file defines the default configuration values for raw data app.
 *
 */

#ifndef RAW_DATA_DEFAULTS_H
#define RAW_DATA_DEFAULTS_H

// Device configuration

#define IFX_FRAME_RATE_HZ               (1)         /**< This is the frame acquisition rate for raw data.
                                                         Valid range is [0.016 - 100]Hz.*/

#define IFX_TX_POWER_LEVEL              (31)        /**< TX Power level to be used on the radar transceiver.
                                                         Valid range is [1 - 31].*/

#define IFX_RX_MASK             (4)         /**< Receive antenna mask defines which antennas to activate. 
                                                         Multiple antennas can be activated by masking.*/
#define IFX_TX_MASK             (1)         /**< Transmitting antenna mask defines which antennas to activate. */


#define IFX_IF_GAIN_DB                  (33)        /**< This is the amplification factor that is
                                                         applied to the IF signal before sampling.
                                                         Valid range is [18 - 60]dB.*/

#define IFX_SAMPLE_RATE_HZ           (1000000U)  /**< Sample rate of the ADC.
                                                      Valid range is [100k - 2M]Hz.*/

#define IFX_NUM_SAMPLES_PER_CHIRP       (128)       /**< Number of samples per chirp.*/

#define IFX_NUM_CHIRPS_PER_FRAME        (32)        /**< Number of chirps in a frame.*/

#define IFX_LOWER_FREQUENCY_HZ         ((uint64_t)60500000000)  /**< Lower frequency of the bandwidth the chirps start at.*/

#define IFX_UPPER_FREQUENCY_HZ         ((uint64_t)61500000000)  /**< Higher frequency of the bandwidth the chirps end with.*/

#define IFX_CHIRP_REPETITION_TIME_S ((ifx_Float_t)5.014928e-4)   /**< The time between consecutive chirps in 100 picoseconds steps.*/

#define IFX_FRAME_REPETITION_TIME_S             ((ifx_Float_t)1 / IFX_FRAME_RATE_HZ)  /**< Frame-rate in Hz to period in microseconds calculation */

#endif /* RAW_DATA_DEFAULTS_H */
