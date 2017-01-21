/* Copyright (c) 2012 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 */

/** @file
 *
 * @defgroup ble_sdk_srv_temp Temperature Service
 * @{
 * @ingroup ble_sdk_srv
 * @brief Temperature Service module.
 *
 * @details This module implements the Temperature Service with the Temperature Level characteristic.
 *          During initialization it adds the Temperature Service and Temperature Level characteristic
 *          to the BLE stack database. Optionally it can also add a Report Reference descriptor
 *          to the Temperature Level characteristic (used when including the Temperature Service in
 *          the HID service).
 *
 *          If specified, the module will support notification of the Temperature Level characteristic
 *          through the ble_temp_Temperature_level_update() function.
 *          If an event handler is supplied by the application, the Temperature Service will
 *          generate Temperature Service events to the application.
 *
 * @note The application must propagate BLE stack events to the Temperature Service module by calling
 *       ble_temp_on_ble_evt() from the @ref softdevice_handler callback.
 *
 * @note Attention! 
 *  To maintain compliance with Nordic Semiconductor ASA Bluetooth profile 
 *  qualification listings, this section of source code must not be modified.
 */

#ifndef BLE_TEMPERATURE_H__
#define BLE_TEMPERATURE_H__

#include <stdint.h>
#include <stdbool.h>
#include "ble.h"
#include "ble_srv_common.h"
		
#define BLE_UUID_TEMPERATURE_SERVICE		0xAA55

/**@brief Temperature Service event type. */
typedef enum
{
    BLE_TEMP_EVT_NOTIFICATION_ENABLED,                             /**< Temperature value notification enabled event. */
    BLE_TEMP_EVT_NOTIFICATION_DISABLED                             /**< Temperature value notification disabled event. */
} ble_temp_evt_type_t;

/**@brief Temperature Service event. */
typedef struct
{
    ble_temp_evt_type_t evt_type;                                  /**< Type of event. */
} ble_temp_evt_t;

// Forward declaration of the ble_temp_t type. 
typedef struct ble_temp_s ble_temp_t;

/**@brief Temperature Service event handler type. */
typedef void (*ble_temp_evt_handler_t) (ble_temp_t * p_temp, ble_temp_evt_t * p_evt);

/**@brief Temperature Service init structure. This contains all options and data needed for
 *        initialization of the service.*/
typedef struct
{
    ble_temp_evt_handler_t        evt_handler;                          /**< Event handler to be called for handling events in the Temperature Service. */
    bool                          support_notification;                 /**< TRUE if notification of Temperature Level measurement is supported. */
    ble_srv_report_ref_t *        p_report_ref;                         /**< If not NULL, a Report Reference descriptor with the specified value will be added to the Temperature Level characteristic */
    uint8_t                       initial_temp_level;                   /**< Initial Temperature level */
    ble_srv_cccd_security_mode_t  temperature_level_char_attr_md;       /**< Initial security level for Temperature characteristics attribute */
    ble_gap_conn_sec_mode_t       temperature_level_report_read_perm;   /**< Initial security level for Temperature report read attribute */
} ble_temp_init_t;

/**@brief Temperature Service structure. This contains various status information for the service. */
struct ble_temp_s
{
    ble_temp_evt_handler_t        evt_handler;                          /**< Event handler to be called for handling events in the Temperature Service. */
    uint16_t                      service_handle;                       /**< Handle of Temperature Service (as provided by the BLE stack). */
    ble_gatts_char_handles_t      temperature_level_handles;            /**< Handles related to the Temperature Level characteristic. */
    uint16_t                      report_ref_handle;                    /**< Handle of the Report Reference descriptor. */
    uint8_t                      temperature_level_last;               /**< Last Temperature Level measurement passed to the Temperature Service. */
    uint16_t                      conn_handle;                          /**< Handle of the current connection (as provided by the BLE stack, is BLE_CONN_HANDLE_INVALID if not in a connection). */
    bool                          is_notification_supported;            /**< TRUE if notification of Temperature Level is supported. */
};

/**@brief Function for initializing the Temperature Service.
 *
 * @param[out]  p_temp       Temperature Service structure. This structure will have to be supplied by
 *                          the application. It will be initialized by this function, and will later
 *                          be used to identify this particular service instance.
 * @param[in]   p_temp_init  Information needed to initialize the service.
 *
 * @return      NRF_SUCCESS on successful initialization of service, otherwise an error code.
 */
uint32_t ble_temp_init(ble_temp_t * p_temp, const ble_temp_init_t * p_temp_init);

/**@brief Function for handling the Application's BLE Stack events.
 *
 * @details Handles all events from the BLE stack of interest to the Temperature Service.
 *
 * @note For the requirements in the TEMP specification to be fulfilled,
 *       ble_temp_Temperature_level_update() must be called upon reconnection if the
 *       Temperature level has changed while the service has been disconnected from a bonded
 *       client.
 *
 * @param[in]   p_temp      Temperature Service structure.
 * @param[in]   p_ble_evt  Event received from the BLE stack.
 */
void ble_temp_on_ble_evt(ble_temp_t * p_temp, ble_evt_t * p_ble_evt);

/**@brief Function for updating the Temperature level.
 *
 * @details The application calls this function after having performed a Temperature measurement. If
 *          notification has been enabled, the Temperature level characteristic is sent to the client.
 *
 * @note For the requirements in the TEMP specification to be fulfilled,
 *       this function must be called upon reconnection if the Temperature level has changed
 *       while the service has been disconnected from a bonded client.
 *
 * @param[in]   p_temp          Temperature Service structure.
 * @param[in]   Temperature_level  New Temperature measurement value (in percent of full capacity).
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
uint32_t ble_temp_Temperature_level_update(ble_temp_t * p_temp, uint8_t Temperature_level);

#endif // BLE_TEMPERATURE_H__

/** @} */
