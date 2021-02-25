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
*
* @{
*/

#ifndef APP_COMMON_JSON_H
#define APP_COMMON_JSON_H

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

/*
==============================================================================
    1. INCLUDE FILES
==============================================================================
*/

#include <stdbool.h>

#include "ifxRadar/DeviceControl.h"


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

struct ifxJsonConfigurationC;
typedef struct ifxJsonConfigurationC ifx_json_t;


/*
==============================================================================
    4. FUNCTION PROTOTYPES
==============================================================================
*/

/**
 * @brief Create new json configuration object
 *
 * Create an empty json configuration object
 *
 * @retval json_object  if successul
 * @retval NULL         otherwise
 */
ifx_json_t* ifx_json_create(void);

/**
 * @brief Create json configuration object from file
 *
 * Read the configuration from the JSON configuration filename. This is a
 * convenience function for \ref ifx_json_create and
 * \ref ifx_json_load_from_file.
 *
 * @retval json_object  if successul
 * @retval NULL         otherwise
 */
ifx_json_t* ifx_json_create_from_file(const char* filename);

/**
 * @brief Destroy json object
 *
 * @param [in]  j   json object (might be NULL)
 */
void ifx_json_destroy(ifx_json_t* j);

/**
 * @brief Load configuration from file
 *
 * Load the configuration from the file given by filename.
 *
 * @param [in] json         json object
 * @param [in] filename     filename of configuration file
 * @retval  true    if successful
 * @retval  false   otherwise
 */
bool ifx_json_load_from_file(ifx_json_t* json, const char* filename);

/**
 * @brief Save configuration to file
 *
 * Save the configuration to the file given by filename.
 *
 * @param [in] json         json object
 * @param [in] filename     filename of configuration file
 * @retval  true    if successful
 * @retval  false   otherwise
 */
bool ifx_json_save_to_file(ifx_json_t* json, const char* filename);

/**
 * @brief Get error message
 *
 * Return a human-readable description of the last error.
 *
 * @param [in]  json    json object
 * @retval error message
 */
const char* ifx_json_get_error(const ifx_json_t* json);

/** @brief Return true if fmcw_single_shape configuration is present */
bool ifx_json_has_config_single_shape(const ifx_json_t* json);
void ifx_json_set_device_config_single_shape(ifx_json_t* json, const ifx_Device_Config_t* config);
bool ifx_json_get_device_config_single_shape(ifx_json_t* json, ifx_Device_Config_t* config);

/** @brief Return true if fmcw_scene configuration is present */
bool ifx_json_has_config_scene(const ifx_json_t* json);
void ifx_json_set_device_config_scene(ifx_json_t* json, const ifx_Device_Metrics_t* metrics);
bool ifx_json_get_device_config_scene(ifx_json_t* json, ifx_Device_Metrics_t* metrics);

/** @brief Return true if segmentation configuration is present */
bool ifx_json_has_segmentation(const ifx_json_t* json);
void ifx_json_set_segmentation(ifx_json_t* json, const ifx_Segmentation_Config_t* segmentation_config);
bool ifx_json_get_segmentation(ifx_json_t* json, const ifx_Device_Config_t* device_config, ifx_Segmentation_Config_t* config_segmentation);

void ifx_json_set_presence_sensing(ifx_json_t* json, const ifx_PresenceSensing_Config_t* config_presence_sensing);
bool ifx_json_has_presence_sensing(const ifx_json_t* json);
bool ifx_json_get_presence_sensing(ifx_json_t* json, const ifx_Device_Config_t* device_config, ifx_PresenceSensing_Config_t* config_presence_sensing);

#ifdef __cplusplus
} // extern "C"
#endif // __cplusplus

#endif // #ifndef APP_COMMON_JSON_H

/**
* @}
*/