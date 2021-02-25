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
* \file DeviceConfig.h
*
* \brief   \copybrief gr_deviceconfig
*
* For details refer to \ref gr_deviceconfig
*
*
* @{
*/


#ifndef IFX_RADAR_DEVICE_CONFIG_H
#define IFX_RADAR_DEVICE_CONFIG_H

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

#include "ifxBase/Forward.h"
#include "ifxBase/Types.h"

#include "ifxRadar/DeviceConfig.h"

#define IFX_DEVICE_SAMPLE_RATE_HZ_LOWER ((uint32_t)80000)
#define IFX_DEVICE_SAMPLE_RATE_HZ_UPPER ((uint32_t)2800000)

#define IF_GAIN_DB_LOWER ((uint32_t)18)
#define IF_GAIN_DB_UPPER ((uint32_t)60)

#define TX_POWER_LEVEL_LOWER ((uint32_t)0)
#define TX_POWER_LEVEL_UPPER ((uint32_t)31)


/*
==============================================================================
   3. TYPES
==============================================================================
*/

/**
* @brief Defines possible values to set the tx_mode member of  \ref ifx_Device_Config_t.
*
* The tx mode defines which antenna is used to transmit the configured chirps. 
* Note that not every chip supports every mode.
*/
enum ifx_Device_TX_Mode
{
    IFX_TX_MODE_TX1 = 0, /**< Tx 1 is used for transmission */
    IFX_TX_MODE_TX2 = 1, /**< Tx 2 is used for transmission (only BGT60ATR24C)*/
    IFX_TX_MODE_TX1_TX2_TIMEMUX = 2 /**< Time-division multiplex, alternating 
                                         between Tx 1 and Tx 2 on each transmitted chirp (only BGT60ATR24C).*/
};

enum ifx_Device_MIMO_Mode
{
    IFX_MIMO_OFF = 0, /**< MIMO is deactivataed */
    IFX_MIMO_TDM = 1  /**< time-domain multiplexing MIMO */
};

enum ifx_Device_ADC_Tracking
{
    IFX_ADC_NO_SUBCONVERSIONS = 0,
    IFX_ADC_1_SUBCONVERSIONS  = 1,
    IFX_ADC_3_SUBCONVERSIONS  = 2,
    IFX_ADC_7_SUBCONVERSIONS  = 3
};

enum ifx_Device_ADC_SampleTime
{
    IFX_ADC_SAMPLETIME_50NS  = 0,
    IFX_ADC_SAMPLETIME_100NS = 1,
    IFX_ADC_SAMPLETIME_200NS = 2,
    IFX_ADC_SAMPLETIME_400NS = 3
};

enum ifx_Device_ADC_Oversampling
{
    IFX_ADC_OVERSAMPLING_OFF = 0,
    IFX_ADC_OVERSAMPLING_2x  = 1,
    IFX_ADC_OVERSAMPLING_4x  = 2,
    IFX_ADC_OVERSAMPLING_8x  = 3
};

typedef struct
{
    struct { uint32_t min; uint32_t max; } sample_rate_Hz;
    struct { uint32_t min; uint32_t max; } rx_mask;
    struct { uint32_t min; uint32_t max; } tx_mask;
    struct { uint32_t min; uint32_t max; } tx_power_level;
    struct { uint32_t min; uint32_t max; } if_gain_dB;
    struct { ifx_Float_t min; ifx_Float_t max; } max_range_m;
    struct { ifx_Float_t min; ifx_Float_t max; } range_resolution_m;
    struct { ifx_Float_t min; ifx_Float_t max; } speed_resolution_m_s;
    struct { ifx_Float_t min; ifx_Float_t max; } max_speed_m_s;
    struct { ifx_Float_t min; ifx_Float_t max; } frame_repetition_time_s;
} ifx_Device_Metrics_Limits_t;

typedef struct
{
    const char* description;               /**< A pointer to a null terminated
                                                string holding a human
                                                readable description of the
                                                device. */
    uint64_t         min_rf_frequency_Hz;  /**< The minimum RF frequency the
                                                the sensor device can emit. */
    uint64_t         max_rf_frequency_Hz;  /**< The maximum RF frequency the
                                                sensor device can emit. */
    uint8_t          num_tx_antennas;      /**< The number of RF antennas used
                                                for transmission. */
    uint8_t          num_rx_antennas;      /**< The number of RF antennas used
                                                for reception. */
    uint8_t          major_version_hw;      /**< The major version number of
                                                 the sensor hardware. */
    uint8_t          minor_version_hw;      /**< The minor version number of
                                                 the sensor hardware. */
} ifx_Device_Info_t;

typedef struct
{
    const char* description; /**< A pointer to a zero-terminated string
                                  containing a firmware description. */
    uint16_t version_major; /**< The firmware version major number. */
    uint16_t version_minor; /**< The firmware version minor number. */
    uint16_t version_build; /**< The firmware version build number. */
} ifx_Firmware_Info_t;

/**
* @brief Defines the structure for the metrics of the feature space used for presence detection.
*
* The presence detection algorithm analyzes the distance (range) and velocity (speed) of targets
* in the field of view, so the time domain input data must be transformed to range/speed feature
* space. Resolution and maximum values in this feature space depend on the time domain acquisition
* parameters and the presence sensing algorithm calculates them from the device configuration. To
* go the other way round and specify the metrics the function
* \ref ifx_device_translate_metrics_to_config can be used to derive those acquisition parameters
* from desired feature space metrics. This structure is used to specify the desired metrics to
* that function.
*/
typedef struct
{
    uint32_t sample_rate_Hz; /**< Sampling rate of the ADC used to acquire the samples
                                  during a chirp. The duration of a single churp depends
                                  on the number of samples and the sampling rate. */

    uint32_t rx_mask;        /**< Bitmask where each bit represents one RX antenna of
                                  the radar device. If a bit is set the according RX
                                  antenna is enabled during the chirps and the signal
                                  received through that antenna is captured. The least
                                  significant bit corresponds to antenna 1. */
    uint32_t tx_mask;        /**< Bitmask where each bit represents one TX antenna. */
    uint32_t tx_power_level; /**< This value controls the power of the transmitted RX
                                  signal. This is an abstract value between 0 and 31
                                  without any physical meaning. */
    uint32_t if_gain_dB;     /**< Amplification factor that is applied to the IF signal
                                  coming from the RF mixer before it is fed into the ADC. */

    ifx_Float_t range_resolution_m;   /**< The range resolution is the distance between two consecutive
-                                          bins of the range transform. Note that even though zero
-                                          padding before the range transform seems to increase this
-                                          resolution, the true resolution does not change but depends
-                                          only from the acquisition parameters. Zero padding is just
-                                          interpolation! */
    ifx_Float_t max_range_m;          /**< The bins of the range transform represent the range
                                           between 0m and this value. (If the time domain input data it
                                           is the range-max_range_m ... max_range_m.) */
    ifx_Float_t max_speed_m_s;        /**< The bins of the Doppler transform represent the speed values
                                           between -max_speed_m_s and max_speed_m_s. */
    ifx_Float_t speed_resolution_m_s; /**< The speed resolution is the distance between two consecutive
                                           bins of the Doppler transform. Note that even though zero
                                           padding before the speed transform seems to increase this
                                           resolution, the true resolution does not change but depends
                                           only from the acquisition parameters. Zero padding is just
                                           interpolation! */

    ifx_Float_t frame_repetition_time_s; /**< The desired frame repetition time in seconds (also known
                                              as frame time or frame period). The frame repetition time
                                              is the inverse of the frame rate */

    ifx_Float_t center_frequency_Hz; /**< Center frequency of the FMCW chirp. If the value is set to 0
                                          the center frequency will be determined from the device */
} ifx_Device_Metrics_t;

/**
 * @brief Defines the structure for acquisition of time domain data related settings.
 *
 * When a connection to sensor device is established, the device is configured according to the
 * parameters of this struct.
 */
typedef struct
{
    uint32_t sample_rate_Hz; /**< Sampling rate of the ADC used to acquire the samples
                                  during a chirp. The duration of a single chirp depends
                                  on the number of samples and the sampling rate. */

    uint32_t rx_mask;        /**< Bitmask where each bit represents one RX antenna of
                                  the radar device. If a bit is set the according RX
                                  antenna is enabled during the chirps and the signal
                                  received through that antenna is captured. The least
                                  significant bit corresponds to antenna 1. */
    uint32_t tx_mask;        /**< Bitmask where each bit represents one TX antenna. */
    uint32_t tx_power_level; /**< This value controls the power of the transmitted RX
                                  signal. This is an abstract value between 0 and 31
                                  without any physical meaning. */
    uint32_t if_gain_dB;     /**< Amplification factor that is applied to the IF signal
                                  coming from the RF mixer before it is fed into the ADC. */

    uint64_t lower_frequency_Hz; /**< Lower frequency (start frequency) of the FMCW chirp. */
    uint64_t upper_frequency_Hz; /**< Upper frequency (stop frequency) of the FMCW chirp. */

    uint32_t num_samples_per_chirp; /**< This is the number of samples acquired during each
                                         chirp of a frame. The duration of a single
                                         chirp depends on the number of samples and the
                                         sampling rate. */
    uint32_t num_chirps_per_frame;  /**< This is the number of chirps a single data frame
                                         consists of. */

    ifx_Float_t chirp_repetition_time_s; /**< This is the time period that elapses between the
                                              beginnings of two consecutive chirps in a frame.
                                              (Also commonly refered to as pulse repetition time
                                              or chirp-to-chirp-time.) */
    ifx_Float_t frame_repetition_time_s; /**< This is the time period that elapses between the
                                              beginnings of two consecutive frames. The reciprocal
                                              of this parameter is the frame rate. (Also commonly
                                              refered to as frame time or frame period.) */

    enum ifx_Device_MIMO_Mode mimo_mode; /**< Mode of MIMO, see definition of \ref ifx_Device_MIMO_Mode */
} ifx_Device_Config_t;

/**
 * @brief Defines the structure for the ADC configuration.
 */
typedef struct
{
    uint32_t samplerate_Hz;
    uint8_t tracking;
    uint8_t sample_time;
    uint8_t double_msb_time;
} ifx_Device_ADC_Config_t;

/*
==============================================================================
   4. FUNCTION PROTOTYPES
==============================================================================
*/

/** @addtogroup gr_cat_Device_Access
  * @{
  */

/** @defgroup gr_deviceconfig Device Configuration
  * @brief API for Radar device configuration
  * @{
  */

/**
* @brief Calculates the center frequency in use from given device configuration
*
* @param [in]     config    Device configuration structure
* @return Center frequency (in Hz) used by device configuration
*/
IFX_DLL_PUBLIC
ifx_Float_t ifx_device_get_center_frequency(const ifx_Device_Config_t* config);

/**
* @brief Calculates the bandwidth in use from given device configuration
*
* @param [in]     config    Device configuration structure
* @return Bandwidth (in Hz) used by device configuration
*/
IFX_DLL_PUBLIC
ifx_Float_t ifx_device_get_bandwidth(const ifx_Device_Config_t* config);

/**
* @brief Calculates the Chirp Repetition Time in use from given device configuration
*
* The chirp repetition time is also known as pulse repetition titme (PRT).
*
* @param [in]     config    Device configuration structure
* @return Pulse Repetition Time (in seconds) used by device configuration
*/
IFX_DLL_PUBLIC
ifx_Float_t ifx_device_get_chirp_repetition_time(const ifx_Device_Config_t* config);

/**
* @brief Counts the number of receive antennas based on the given device configuration
*
* If MIMO is active this function will return virtual receive antenna count. 
* 
* @param [in]     config    Device configuration structure
* @return Number of receive antennas activated by device configuration
*/
IFX_DLL_PUBLIC
uint8_t ifx_device_count_rx_antennas(const ifx_Device_Config_t* config);
/**
  * @}
  */
 
/**
  * @}
  */

#ifdef __cplusplus
} // extern "C"
#endif // __cplusplus

#endif /* IFX_RADAR_DEVICE_CONFIG_H */

/**
* @}
*/