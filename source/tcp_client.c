/******************************************************************************
* File Name:   tcp_client.c
*
* Description: This file contains task and functions related to TCP client
* operation.
*
* Related Document: See README.md
*
*
*******************************************************************************
* Copyright 2019-2021, Cypress Semiconductor Corporation (an Infineon company) or
* an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
*
* This software, including source code, documentation and related
* materials ("Software") is owned by Cypress Semiconductor Corporation
* or one of its affiliates ("Cypress") and is protected by and subject to
* worldwide patent protection (United States and foreign),
* United States copyright laws and international treaty provisions.
* Therefore, you may use this Software only as provided in the license
* agreement accompanying the software package from which you
* obtained this Software ("EULA").
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software
* source code solely for use in connection with Cypress's
* integrated circuit products.  Any reproduction, modification, translation,
* compilation, or representation of this Software except as specified
* above is prohibited without the express written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
* reserves the right to make changes to the Software without notice. Cypress
* does not assume any liability arising out of the application or use of the
* Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use in any products where a malfunction or
* failure of the Cypress product may reasonably be expected to result in
* significant property damage, injury or death ("High Risk Product"). By
* including Cypress's product in a High Risk Product, the manufacturer
* of such system or application assumes all risk of such use and in doing
* so agrees to indemnify Cypress against all liability.
*******************************************************************************/

/* Header file includes. */
#include "cyhal.h"
#include "cybsp.h"
#include "cy_retarget_io.h"
#include "cyabs_rtos.h"

/* FreeRTOS header file. */
#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>

/* Standard C header file. */
#include <string.h>

/* Cypress secure socket header file. */
#include "cy_secure_sockets.h"

/* Wi-Fi connection manager header files. */
#include "cy_wcm.h"
#include "cy_wcm_error.h"

/* TCP client task header file. */
#include "tcp_client.h"
#include "scan_task.h"

#include "cy_lwip.h"

/* Standard C header files */
#include <inttypes.h>

/*******************************************************************************
* Macros
********************************************************************************/
/* Wi-Fi Credentials: Modify WIFI_SSID, WIFI_PASSWORD, and WIFI_SECURITY_TYPE
 * to match your Wi-Fi network credentials.
 * Note: Maximum length of the Wi-Fi SSID and password is set to
 * CY_WCM_MAX_SSID_LEN and CY_WCM_MAX_PASSPHRASE_LEN as defined in cy_wcm.h file.
 */
#define WIFI_SSID                             "MY_WIFI_SSID"
#define WIFI_PASSWORD                         "MY_WIFI_PASSWORD"

/* Security type of the Wi-Fi access point. See 'cy_wcm_security_t' structure
 * in "cy_wcm.h" for more details.
 */
#define WIFI_SECURITY_TYPE                    CY_WCM_SECURITY_WPA2_AES_PSK
/* Maximum number of connection retries to a Wi-Fi network. */
#define MAX_WIFI_CONN_RETRIES                 (10u)

/* Wi-Fi re-connection time interval in milliseconds */
#define WIFI_CONN_RETRY_INTERVAL_MSEC         (1000u)

/*******************************************************************************
* Function Prototypes
********************************************************************************/

static cy_rslt_t connect_to_wifi_ap(void);
static void scan_callback( cy_wcm_scan_result_t *result_ptr, void *user_data, cy_wcm_scan_status_t status );
static void print_scan_result(cy_wcm_scan_result_t *result);

/*******************************************************************************
* Global Variables
********************************************************************************/

uint32_t num_scan_result;
bool scan_completed = false;

cy_wcm_mac_t AP_BSSID;

/*******************************************************************************
 * Function Name: tcp_client_task
 *******************************************************************************
 * Summary:
 *  Task used to establish a connection to a remote TCP server and
 *  control the LED state (ON/OFF) based on the command received from TCP server.
 *
 * Parameters:
 *  void *args : Task parameter defined during task creation (unused).
 *
 * Return:
 *  void
 *
 *******************************************************************************/
void tcp_client_task(void *arg)
{
    cy_rslt_t result ;

    cy_wcm_config_t wifi_config = { .interface = CY_WCM_INTERFACE_TYPE_STA };

    /* Initialize Wi-Fi connection manager. */
    result = cy_wcm_init(&wifi_config);

    if (result != CY_RSLT_SUCCESS)
    {
        printf("Wi-Fi Connection Manager initialization failed! Error code: 0x%08"PRIx32"\n", (uint32_t)result);
        CY_ASSERT(0);
    }
    printf("Wi-Fi Connection Manager initialized.\r\n");

    printf("Start scan: %s\n", WIFI_SSID);
    cy_wcm_start_scan(scan_callback, NULL, NULL);
    while (!scan_completed)
    {
        cy_rtos_delay_milliseconds(3000);
    }

    /* Connect to Wi-Fi AP */
    result = connect_to_wifi_ap();
    if(result!= CY_RSLT_SUCCESS )
    {
        printf("\n Failed to connect to Wi-Fi AP! Error code: 0x%08"PRIx32"\n", (uint32_t)result);
        CY_ASSERT(0);
    }

    for(;;)
    {
        cy_wcm_wlan_statistics_t stat;
        memset(&stat, 0, sizeof(stat));
        result = cy_wcm_get_wlan_statistics(CY_WCM_INTERFACE_TYPE_STA, &stat);

        printf("rx_bytes=%lu, tx_bytes=%lu, rx_packets=%lu, tx_packets=%lu, tx_retries=%lu, tx_failed=%lu, tx_bitrate=%lu\r\n",
               stat.rx_bytes, stat.tx_bytes, stat.rx_packets, stat.tx_packets, stat.tx_retries, stat.tx_failed, stat.tx_bitrate);

        cy_rtos_delay_milliseconds(3000);
    }
 }

/*******************************************************************************
 * Function Name: connect_to_wifi_ap()
 *******************************************************************************
 * Summary:
 *  Connects to Wi-Fi AP using the user-configured credentials, retries up to a
 *  configured number of times until the connection succeeds.
 *
 *******************************************************************************/
cy_rslt_t connect_to_wifi_ap(void)
{
    cy_rslt_t result;

    /* Variables used by Wi-Fi connection manager.*/
    cy_wcm_connect_params_t wifi_conn_param;

    cy_wcm_ip_address_t ip_address;

     /* Set the Wi-Fi SSID, password and security type. */
    memset(&wifi_conn_param, 0, sizeof(cy_wcm_connect_params_t));
    memcpy(wifi_conn_param.ap_credentials.SSID, WIFI_SSID, sizeof(WIFI_SSID));
    memcpy(wifi_conn_param.ap_credentials.password, WIFI_PASSWORD, sizeof(WIFI_PASSWORD));

#if 0
    //MAC 1c:3b:f3:a3:2c:6b
    wifi_conn_param.BSSID[0] = 0x1c;
    wifi_conn_param.BSSID[1] = 0x3b;
    wifi_conn_param.BSSID[2] = 0xf3;
    wifi_conn_param.BSSID[3] = 0xa3;
    wifi_conn_param.BSSID[4] = 0x2c;
    wifi_conn_param.BSSID[5] = 0x6b;
#elif 1
    memcpy(wifi_conn_param.BSSID, AP_BSSID, 6);
#endif

    wifi_conn_param.ap_credentials.security = WIFI_SECURITY_TYPE;

    printf("\nConnecting to %s with MAC address %02X:%02X:%02X:%02X:%02X:%02X\n\n",WIFI_SSID,
           wifi_conn_param.BSSID[0],wifi_conn_param.BSSID[1],wifi_conn_param.BSSID[2],
           wifi_conn_param.BSSID[3],wifi_conn_param.BSSID[4],wifi_conn_param.BSSID[5]);

    /* Join the Wi-Fi AP. */
    for(uint32_t conn_retries = 0; conn_retries < MAX_WIFI_CONN_RETRIES; conn_retries++ )
    {
        result = cy_wcm_connect_ap(&wifi_conn_param, &ip_address);

        if(result == CY_RSLT_SUCCESS)
        {
            printf("Successfully connected to Wi-Fi network '%s'.\n",
                                wifi_conn_param.ap_credentials.SSID);
            printf("IP Address Assigned: %s\n",
                    ip4addr_ntoa((const ip4_addr_t *)&ip_address.ip.v4));
            return result;
        }

        printf("Connection to Wi-Fi network failed with error code %d."
               "Retrying in %d ms...\n", (int)result, WIFI_CONN_RETRY_INTERVAL_MSEC);

        vTaskDelay(pdMS_TO_TICKS(WIFI_CONN_RETRY_INTERVAL_MSEC));
    }

    /* Stop retrying after maximum retry attempts. */
    printf("Exceeded maximum Wi-Fi connection attempts\n");

    return result;
}

/*
 * Function Name: scan_callback
 *******************************************************************************
 * Summary: The callback function which accumulates the scan results. After
 * completing the scan, it sends a task notification to scan_task.
 *
 * Parameters:
 *   cy_wcm_scan_result_t *result_ptr: Pointer to the scan result
 *   void *user_data: User data.
 *   cy_wcm_scan_status_t status: Status of scan completion.
 *
 * Return:
 *  void
 *
 ******************************************************************************/
static void scan_callback(cy_wcm_scan_result_t *result_ptr, void *user_data, cy_wcm_scan_status_t status)
{
    if ((strlen((const char *)result_ptr->SSID) != 0) && (status == CY_WCM_SCAN_INCOMPLETE))
    {
        num_scan_result++;
        print_scan_result(result_ptr);

        if (strcmp((char*)result_ptr->SSID, WIFI_SSID) == 0)
        {
            printf("*** Found target SSID, save BSSID\n");
            memcpy(AP_BSSID, result_ptr->BSSID, 6);
        }
    }

    if ( (CY_WCM_SCAN_COMPLETE == status) )
    {
        /* Reset the number of scan results to 0 for the next scan.*/
        num_scan_result = 0;
        scan_completed = true;
    }
}


/*******************************************************************************
 * Function Name: print_scan_result
 *******************************************************************************
 * Summary: This function prints the scan result accumulated by the scan
 * handler.
 *
 *
 * Parameters:
 *  cy_wcm_scan_result_t *result: Pointer to the scan result.
 *
 * Return:
 *  void
 *
 ******************************************************************************/
static void print_scan_result(cy_wcm_scan_result_t *result)
{
    char* security_type_string;

    /* Convert the security type of the scan result to the corresponding
     * security string
     */
    switch (result->security)
    {
        case CY_WCM_SECURITY_OPEN:
            security_type_string = SECURITY_OPEN;
            break;
        case CY_WCM_SECURITY_WEP_PSK:
            security_type_string = SECURITY_WEP_PSK;
            break;
        case CY_WCM_SECURITY_WEP_SHARED:
            security_type_string = SECURITY_WEP_SHARED;
            break;
        case CY_WCM_SECURITY_WPA_TKIP_PSK:
            security_type_string = SECURITY_WEP_TKIP_PSK;
            break;
        case CY_WCM_SECURITY_WPA_AES_PSK:
            security_type_string = SECURITY_WPA_AES_PSK;
            break;
        case CY_WCM_SECURITY_WPA_MIXED_PSK:
            security_type_string = SECURITY_WPA_MIXED_PSK;
            break;
        case CY_WCM_SECURITY_WPA2_AES_PSK:
            security_type_string = SECURITY_WPA2_AES_PSK;
            break;
        case CY_WCM_SECURITY_WPA2_TKIP_PSK:
            security_type_string = SECURITY_WPA2_TKIP_PSK;
            break;
        case CY_WCM_SECURITY_WPA2_MIXED_PSK:
            security_type_string = SECURITY_WPA2_MIXED_PSK;
            break;
        case CY_WCM_SECURITY_WPA2_FBT_PSK:
            security_type_string = SECURITY_WPA2_FBT_PSK;
            break;
        case CY_WCM_SECURITY_WPA3_SAE:
            security_type_string = SECURITY_WPA3_SAE;
            break;
        case CY_WCM_SECURITY_WPA3_WPA2_PSK:
            security_type_string = SECURITY_WPA3_WPA2_PSK;
            break;
        case CY_WCM_SECURITY_IBSS_OPEN:
            security_type_string = SECURITY_IBSS_OPEN;
            break;
        case CY_WCM_SECURITY_WPS_SECURE:
            security_type_string = SECURITY_WPS_SECURE;
            break;
        case CY_WCM_SECURITY_UNKNOWN:
            security_type_string = SECURITY_UNKNOWN;
            break;
        default:
            security_type_string = SECURITY_UNKNOWN;
            break;
    }

    printf(" %2ld   %-32s     %4d     %2d      %2X:%2X:%2X:%2X:%2X:%2X         %-15s\n",
           num_scan_result, result->SSID,
           result->signal_strength, result->channel, result->BSSID[0], result->BSSID[1],
           result->BSSID[2], result->BSSID[3], result->BSSID[4], result->BSSID[5],
           security_type_string);
}

/* [] END OF FILE */
