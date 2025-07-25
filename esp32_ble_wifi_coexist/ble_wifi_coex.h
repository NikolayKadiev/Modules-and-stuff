#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <stdbool.h>
#include <stdio.h>
#include "nvs.h"
#include "nvs_flash.h"

#include "esp_bt.h"
#include "esp_gap_ble_api.h"
#include "esp_gattc_api.h"
#include "esp_gatt_defs.h"
#include "esp_bt_main.h"
#include "esp_gatt_common_api.h"
#include "esp_log.h"

#include "esp_wifi.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_netif.h"

// ADD PROTOCOL LIBRARIES

#include "protocol_examples_common.h"

#include "lwip/sockets.h"
#include "lwip/dns.h"
#include "lwip/netdb.h"

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/event_groups.h"

#include "driver/uart.h"
#include "esp_sleep.h"
#include "driver/gpio.h"

/* BLE part */
typedef struct{
    esp_gatt_if_t   gatt_if;
    uint16_t        conn_id;
    uint16_t        char_handle;
}esp_gatt_wr_elem_t; 

esp_gatt_wr_elem_t wr_elements;
QueueHandle_t uart_queue = NULL;
QueueHandle_t pin_queue = NULL;

typedef struct{
    uint8_t             ble_data_len;
    uint8_t             ble_adv_data[20];
    esp_bd_addr_t       ble_mac_adr;
    esp_ble_addr_type_t ble_ad_type;
}esp_ble_data_point;

#define NEW_DATA_POINT_NUM_ALL     10
uint8_t new_data_point_num = 0;
esp_ble_data_point new_data_points[NEW_DATA_POINT_NUM_ALL];

#define GATTC_TAG                  "NK_ESP_CLIENT"
#define REMOTE_SERVICE_UUID        0x00FF
#define REMOTE_NOTIFY_CHAR_UUID    0xFF01
#define PROFILE_NUM                1
#define PROFILE_A_APP_ID           0
#define INVALID_HANDLE             0

static const char new_device_name[] = "NK_ESP_DEV";
static bool ble_connect  = false;
static bool ble_rdy      = false;
static bool get_server   = false;
static bool ble_scan_rdy = false;
static bool ble_scan_cpl = false;
static esp_gattc_char_elem_t *char_elem_result   = NULL;
static esp_gattc_descr_elem_t *descr_elem_result = NULL;

/* Declare static functions */
static void esp_gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param);
static void esp_gattc_cb(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param);
static void gattc_profile_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param);


static esp_bt_uuid_t remote_filter_service_uuid = {
    .len = ESP_UUID_LEN_16,
    .uuid = {.uuid16 = REMOTE_SERVICE_UUID,},
};

static esp_bt_uuid_t remote_filter_char_uuid = {
    .len = ESP_UUID_LEN_16,
    .uuid = {.uuid16 = REMOTE_NOTIFY_CHAR_UUID,},
};

static esp_bt_uuid_t notify_descr_uuid = {
    .len = ESP_UUID_LEN_16,
    .uuid = {.uuid16 = ESP_GATT_UUID_CHAR_CLIENT_CONFIG,},
};

static esp_ble_scan_params_t ble_scan_params = {
    .scan_type              = BLE_SCAN_TYPE_ACTIVE,
    .own_addr_type          = BLE_ADDR_TYPE_PUBLIC,
    .scan_filter_policy     = BLE_SCAN_FILTER_ALLOW_ALL,
    .scan_interval          = 0x3000,     //0x0640  //0x0300
    .scan_window            = 0x3000,     //0x0640  //0x0300
    .scan_duplicate         = BLE_SCAN_DUPLICATE_ENABLE
};

struct gattc_profile_inst {
    esp_gattc_cb_t gattc_cb;
    uint16_t gattc_if;
    uint16_t app_id;
    uint16_t conn_id;
    uint16_t service_start_handle;
    uint16_t service_end_handle;
    uint16_t char_handle;
    esp_bd_addr_t remote_bda;
};

/* One gatt-based profile one app_id and one gattc_if, this array will store the gattc_if returned by ESP_GATTS_REG_EVT */
static struct gattc_profile_inst gl_profile_tab[PROFILE_NUM] = {
    [PROFILE_A_APP_ID] = {
        .gattc_cb = gattc_profile_event_handler,
        .gattc_if = ESP_GATT_IF_NONE,       /* Not get the gatt_if, so initial is ESP_GATT_IF_NONE */
    },
};

/* WiFi part*/
#define EXAMPLE_ESP_WIFI_SSID      "ESP32_NET"
#define EXAMPLE_ESP_WIFI_PASS      "SP32_PASS"
#define EXAMPLE_ESP_MAXIMUM_RETRY  5

// #define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_OPEN
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA2_PSK
/* FreeRTOS event group to signal when we are connected*/
static EventGroupHandle_t s_wifi_event_group;

#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1
#define HOST_IP_ADDR CONFIG_EXAMPLE_IPV4_ADDR

static int s_retry_num = 0;

uint8_t conn_status = 0, in_state = 0;

// ADD PROTOCOL VARIABLES AND STRUCTURES IF NEEDED