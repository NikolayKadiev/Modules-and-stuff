#include "ble_wifi_coex.h"

#define PIN_SEARCH      19
#define PIN_SCAN        18

uint8_t first_scan = 0, ble_num_now = 0;
/* BLE PART*/

static void gattc_profile_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param)
{
    esp_ble_gattc_cb_param_t *p_data = (esp_ble_gattc_cb_param_t *)param;

    switch (event) {
    case ESP_GATTC_REG_EVT:
        ESP_LOGI(GATTC_TAG, "ESP_GATTC_REG_EVT");
        esp_ble_gap_set_scan_params(&ble_scan_params);
        break;
    case ESP_GATTC_CONNECT_EVT:{
        ESP_LOGI(GATTC_TAG, "ESP_GATTC_CONNECT_EVT");
        memcpy(gl_profile_tab[PROFILE_A_APP_ID].remote_bda, p_data->connect.remote_bda, sizeof(esp_bd_addr_t));
        esp_ble_gattc_send_mtu_req (gattc_if, p_data->connect.conn_id);

        break;
    }
    case ESP_GATTC_OPEN_EVT:
        ESP_LOGI(GATTC_TAG, "ESP_GATTC_OPEN_EVT");
        if (param->open.status != ESP_GATT_OK){
            break;
        }
        break;
    case ESP_GATTC_DIS_SRVC_CMPL_EVT:
        ESP_LOGI(GATTC_TAG, "ESP_GATTC_DIS_SRVC_CMPL_EVT");
        if (param->dis_srvc_cmpl.status != ESP_GATT_OK){
            break;
        }
        esp_ble_gattc_search_service(gattc_if, param->cfg_mtu.conn_id, &remote_filter_service_uuid);
        break;
    case ESP_GATTC_CFG_MTU_EVT:
        ESP_LOGI(GATTC_TAG, "ESP_GATTC_CFG_MTU_EVT");
        break;
    case ESP_GATTC_SEARCH_RES_EVT: {
        ESP_LOGI(GATTC_TAG, "ESP_GATTC_SEARCH_RES_EVT");
        if (p_data->search_res.srvc_id.uuid.len == ESP_UUID_LEN_16 && p_data->search_res.srvc_id.uuid.uuid.uuid16 == REMOTE_SERVICE_UUID) {
            get_server = true;
            gl_profile_tab[PROFILE_A_APP_ID].service_start_handle = p_data->search_res.start_handle;
            gl_profile_tab[PROFILE_A_APP_ID].service_end_handle = p_data->search_res.end_handle;
        }
        break;
    }
    case ESP_GATTC_SEARCH_CMPL_EVT:
        ESP_LOGI(GATTC_TAG, "ESP_GATTC_SEARCH_CMPL_EVT");
        if (p_data->search_cmpl.status != ESP_GATT_OK){
            break;
        }
        if (get_server){
            uint16_t count = 0;
            esp_gatt_status_t status = esp_ble_gattc_get_attr_count( gattc_if,
                                                                     p_data->search_cmpl.conn_id,
                                                                     ESP_GATT_DB_CHARACTERISTIC,
                                                                     gl_profile_tab[PROFILE_A_APP_ID].service_start_handle,
                                                                     gl_profile_tab[PROFILE_A_APP_ID].service_end_handle,
                                                                     INVALID_HANDLE,
                                                                     &count);
            if (status != ESP_GATT_OK){
                break;
            }
            if (count > 0){
                char_elem_result = (esp_gattc_char_elem_t *)malloc(sizeof(esp_gattc_char_elem_t) * count);
                if (!char_elem_result){
                    break;
                }
                status = esp_ble_gattc_get_char_by_uuid( gattc_if,
                                                            p_data->search_cmpl.conn_id,
                                                            gl_profile_tab[PROFILE_A_APP_ID].service_start_handle,
                                                            gl_profile_tab[PROFILE_A_APP_ID].service_end_handle,
                                                            remote_filter_char_uuid,
                                                            char_elem_result,
                                                            &count);
                    if (status != ESP_GATT_OK){
                        free(char_elem_result);
                        char_elem_result = NULL;
                        break;
                    }
                /*  Every service have only one char in our 'ESP_GATTS_DEMO' demo, so we used first 'char_elem_result' */
                if (count > 0 && (char_elem_result[0].properties & ESP_GATT_CHAR_PROP_BIT_NOTIFY)){
                    gl_profile_tab[PROFILE_A_APP_ID].char_handle = char_elem_result[0].char_handle;
                    esp_ble_gattc_register_for_notify (gattc_if, gl_profile_tab[PROFILE_A_APP_ID].remote_bda, char_elem_result[0].char_handle);
                }
                /* free char_elem_result */
                free(char_elem_result);
            }
        }
         break;
    case ESP_GATTC_REG_FOR_NOTIFY_EVT: {
        ESP_LOGI(GATTC_TAG, "ESP_GATTC_REG_FOR_NOTIFY_EVT");
        uint16_t count = 0;
        uint16_t notify_en = 1;
        esp_gatt_status_t ret_status = esp_ble_gattc_get_attr_count( gattc_if,
                                                                    gl_profile_tab[PROFILE_A_APP_ID].conn_id,
                                                                    ESP_GATT_DB_DESCRIPTOR,
                                                                    gl_profile_tab[PROFILE_A_APP_ID].service_start_handle,
                                                                    gl_profile_tab[PROFILE_A_APP_ID].service_end_handle,
                                                                    gl_profile_tab[PROFILE_A_APP_ID].char_handle,
                                                                    &count);
        if (ret_status != ESP_GATT_OK){
            break;
        }
        if (count > 0){
            descr_elem_result = malloc(sizeof(esp_gattc_descr_elem_t) * count);
            if (!descr_elem_result){
                    break;
            }
            ret_status = esp_ble_gattc_get_descr_by_char_handle( gattc_if,
                                                                gl_profile_tab[PROFILE_A_APP_ID].conn_id,
                                                                p_data->reg_for_notify.handle,
                                                                notify_descr_uuid,
                                                                descr_elem_result,
                                                                &count);
            if (ret_status != ESP_GATT_OK){
                free(descr_elem_result);
                descr_elem_result = NULL;
                break;
            }
            /* Every char has only one descriptor in our 'ESP_GATTS_DEMO' demo, so we used first 'descr_elem_result' */
            if (count > 0 && descr_elem_result[0].uuid.len == ESP_UUID_LEN_16 && descr_elem_result[0].uuid.uuid.uuid16 == ESP_GATT_UUID_CHAR_CLIENT_CONFIG){
                esp_ble_gattc_write_char_descr( gattc_if,
                                                            gl_profile_tab[PROFILE_A_APP_ID].conn_id,
                                                            descr_elem_result[0].handle,
                                                            sizeof(notify_en),
                                                            (uint8_t *)&notify_en,
                                                            ESP_GATT_WRITE_TYPE_RSP,
                                                            ESP_GATT_AUTH_REQ_NONE);
            }

            /* free descr_elem_result */
            free(descr_elem_result);
            
        }
        
        break;
    }
    case ESP_GATTC_NOTIFY_EVT:
        ESP_LOGI(GATTC_TAG, "ESP_GATTC_NOTIFY_EVT");
        uart_write_bytes(UART_NUM_0, (char *)(p_data->notify.value), p_data->notify.value_len);
        // xQueueSend(msg_queue, &go_now, 0);
        break;
    case ESP_GATTC_WRITE_DESCR_EVT:
        ESP_LOGI(GATTC_TAG, "ESP_GATTC_WRITE_DESCR_EVT");
        if (p_data->write.status != ESP_GATT_OK){
            break;
        }
        uint8_t write_char_data = '\n';
        esp_ble_gattc_write_char( gattc_if,
                                  gl_profile_tab[PROFILE_A_APP_ID].conn_id,
                                  gl_profile_tab[PROFILE_A_APP_ID].char_handle,
                                  1,//sizeof(write_char_data),
                                  &write_char_data,
                                  ESP_GATT_WRITE_TYPE_NO_RSP,
                                  ESP_GATT_AUTH_REQ_NONE);
        ble_rdy = 1;
        break;
    case ESP_GATTC_SRVC_CHG_EVT: {
        ESP_LOGI(GATTC_TAG, "ESP_GATTC_SRVC_CHG_EVT");
        esp_bd_addr_t bda;
        memcpy(bda, p_data->srvc_chg.remote_bda, sizeof(esp_bd_addr_t));
        break;
    }
    case ESP_GATTC_WRITE_CHAR_EVT:
        ESP_LOGI(GATTC_TAG, "ESP_GATTC_WRITE_CHAR_EVT");
        wr_elements.char_handle = gl_profile_tab[PROFILE_A_APP_ID].char_handle;
        wr_elements.conn_id = gl_profile_tab[PROFILE_A_APP_ID].conn_id;//param->write.conn_id;
        wr_elements.gatt_if = gattc_if;
        ble_connect = 1;

        break;
    case ESP_GATTC_DISCONNECT_EVT:
        ESP_LOGI(GATTC_TAG, "ESP_GATTC_DISCONNECT_EVT");
        ble_connect = false;
        get_server = false;
        break;
    default:
        break;
    }
}

static void esp_gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    uint8_t *adv_name = NULL;
    uint8_t adv_name_len = 0;
    switch (event) {
    case ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT: {
        ESP_LOGI(GATTC_TAG, "ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT");
        //the unit of the duration is second
        esp_ble_gap_start_scanning(25);
        break;
    }
    case ESP_GAP_BLE_SCAN_START_COMPLETE_EVT:
        ESP_LOGI(GATTC_TAG, "ESP_GAP_BLE_SCAN_START_COMPLETE_EVT");
        //scan start complete event to indicate scan start successfully or failed
        if (param->scan_start_cmpl.status != ESP_BT_STATUS_SUCCESS) {
            break;
        }

        break;
    case ESP_GAP_BLE_SCAN_RESULT_EVT: {
        ESP_LOGI(GATTC_TAG, "ESP_GAP_BLE_SCAN_RESULT_EVT");
        esp_ble_gap_cb_param_t *scan_result = (esp_ble_gap_cb_param_t *)param;
        switch (scan_result->scan_rst.search_evt) {
        case ESP_GAP_SEARCH_INQ_RES_EVT:
            gpio_set_level(PIN_SCAN, 1);
            uint8_t ble_msg_len = 0;
            ESP_LOGI(GATTC_TAG, "ESP_GAP_SEARCH_INQ_RES_EVT");
            esp_log_buffer_hex(GATTC_TAG, scan_result->scan_rst.bda, 6);
            if(first_scan){
                for(uint8_t num_now = 0; num_now < new_data_point_num; num_now++){
                    if((new_data_points[num_now].ble_mac_adr[0] == scan_result->scan_rst.bda[0]) &&
                    (new_data_points[num_now].ble_mac_adr[1] == scan_result->scan_rst.bda[1]) &&
                    (new_data_points[num_now].ble_mac_adr[2] == scan_result->scan_rst.bda[2]) &&
                    (new_data_points[num_now].ble_mac_adr[3] == scan_result->scan_rst.bda[3]) &&
                    (new_data_points[num_now].ble_mac_adr[4] == scan_result->scan_rst.bda[4]) &&
                    (new_data_points[num_now].ble_mac_adr[5] == scan_result->scan_rst.bda[5])){
                        if(new_data_points[num_now].ble_data_len == 0){
                            ble_msg_len = scan_result->scan_rst.adv_data_len;
                            uint8_t *adv_data = scan_result->scan_rst.ble_adv;
                            uint8_t a1 = 0, a2 = 0;
                            ble_num_now --;
                            for(a1 = 0; a1 < ble_msg_len; a1 ++){
                                if((adv_data[a1+1] == 0xff) && (adv_data[a1+2] == 0xff) && (adv_data[a1+3] == 0xff)){
                                    new_data_points[num_now].ble_data_len = adv_data[a1]-1;
                                    for(a2 = 0; a2 < new_data_points[num_now].ble_data_len+1; a2++){
                                        new_data_points[num_now].ble_adv_data[a2] = adv_data[a2+a1+2];
                                    }
                                    break;
                                }
                            }
                            if(ble_num_now == 0){
                                ble_scan_rdy = 1;
                                esp_ble_gap_stop_scanning();
                            }
                        }
                    }
                }
            }
            else{
                adv_name = esp_ble_resolve_adv_data(scan_result->scan_rst.ble_adv, ESP_BLE_AD_TYPE_NAME_CMPL, &adv_name_len);
                if (adv_name != NULL) {
                    // ESP_LOGI(GATTC_TAG, "ESP_GAP_SEARCH name: %s", adv_name);
                    // if ((strlen(new_device_name) == adv_name_len) && 
                    // (strncmp((char *)adv_name, new_device_name, adv_name_len) == 0) && 
                    // (scan_result->scan_rst.adv_data_len > 26) )
                    if ((strlen(new_device_name) == adv_name_len) && 
                    (strncmp((char *)adv_name, new_device_name, adv_name_len) == 0)) {
                        memcpy(new_data_points[new_data_point_num].ble_mac_adr, scan_result->scan_rst.bda, 6);
                        new_data_point_num += 1;
                        if(new_data_point_num == NEW_DATA_POINT_NUM_ALL){
                            ble_scan_rdy = 1;
                            esp_ble_gap_stop_scanning();
                        }
                    }
                }
            }
            gpio_set_level(PIN_SCAN, 0);
            break;
        case ESP_GAP_SEARCH_INQ_CMPL_EVT:
            ESP_LOGI(GATTC_TAG, "ESP_GAP_SEARCH_INQ_CMPL_EVT");
            ble_scan_cpl = 1;
            break;
        default:
            break;
        }
        break;
    }
    case ESP_GAP_BLE_SCAN_STOP_COMPLETE_EVT:
        ESP_LOGI(GATTC_TAG, "ESP_GAP_BLE_SCAN_STOP_COMPLETE_EVT");
        if (param->scan_stop_cmpl.status != ESP_BT_STATUS_SUCCESS){
            break;
        }
        break;

    case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
        ESP_LOGI(GATTC_TAG, "ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT");
        if (param->adv_stop_cmpl.status != ESP_BT_STATUS_SUCCESS){
            break;
        }
        break;
    case ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT:
        ESP_LOGI(GATTC_TAG, "ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT");
        break;
    default:
        break;
    }
}

static void esp_gattc_cb(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param)
{
    /* If event is register event, store the gattc_if for each profile */
    if (event == ESP_GATTC_REG_EVT) {
        if (param->reg.status == ESP_GATT_OK) {
            gl_profile_tab[param->reg.app_id].gattc_if = gattc_if;
        } else {
            return;
        }
    }

    /* If the gattc_if equal to profile A, call profile A cb handler,
     * so here call each profile's callback */
    do {
        int idx;
        for (idx = 0; idx < PROFILE_NUM; idx++) {
            if (gattc_if == ESP_GATT_IF_NONE || /* ESP_GATT_IF_NONE, not specify a certain gatt_if, need to call every profile cb function */
                    gattc_if == gl_profile_tab[idx].gattc_if) {
                if (gl_profile_tab[idx].gattc_cb) {
                    gl_profile_tab[idx].gattc_cb(event, gattc_if, param);
                }
            }
        }
    } while (0);
}

void ble_init_fun(void)
{
    // Initialize NVS.
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );
    esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT);
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    esp_bt_controller_init(&bt_cfg);
    esp_bt_controller_enable(ESP_BT_MODE_BLE);
    esp_bluedroid_config_t bluedroid_cfg = BT_BLUEDROID_INIT_CONFIG_DEFAULT();
    ret = esp_bluedroid_init_with_cfg(&bluedroid_cfg);
    esp_bluedroid_enable();
    esp_ble_gap_register_callback(esp_gap_cb);
    esp_ble_gattc_register_callback(esp_gattc_cb);
    esp_ble_gattc_app_register(PROFILE_A_APP_ID);
    esp_ble_gatt_set_local_mtu(510);
}

void ble_msg_send(uint16_t value_len, uint8_t *value)
{
    esp_ble_gattc_write_char(wr_elements.gatt_if,
                                wr_elements.conn_id,
                                wr_elements.char_handle,
                                value_len,
                                value,
                                ESP_GATT_WRITE_TYPE_NO_RSP,
                                ESP_GATT_AUTH_REQ_NONE);
}

/*---------*/

/* WiFi Part*/

// ADD PROTOCOL FUNCTIONS HERE

/* STATION part */
static void event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < EXAMPLE_ESP_MAXIMUM_RETRY) {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(GATTC_TAG, "retry to connect to the AP");
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
        ESP_LOGI(GATTC_TAG,"connect to the AP fail");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(GATTC_TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

void wifi_init_sta(void)
{
    s_wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());

    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_got_ip));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = EXAMPLE_ESP_WIFI_SSID,
            .password = EXAMPLE_ESP_WIFI_PASS,
	     .threshold.authmode = ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD,
	     .sae_pwe_h2e = WPA3_SAE_PWE_BOTH,
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config) );
    ESP_ERROR_CHECK(esp_wifi_start() );

    ESP_LOGI(GATTC_TAG, "wifi_init_sta finished.");

    /* Waiting until either the connection is established (WIFI_CONNECTED_BIT) or connection failed for the maximum
     * number of re-tries (WIFI_FAIL_BIT). The bits are set by event_handler() (see above) */
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
            pdFALSE,
            pdFALSE,
            portMAX_DELAY);

    /* xEventGroupWaitBits() returns the bits before the call returned, hence we can test which event actually
     * happened. */
    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(GATTC_TAG, "connected to ap SSID:%s password:%s",
                 wifi_config.sta.ssid, wifi_config.sta.password);
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGI(GATTC_TAG, "Failed to connect to SSID:%s, password:%s",
                 wifi_config.sta.ssid, wifi_config.sta.password);
    } else {
        ESP_LOGE(GATTC_TAG, "UNEXPECTED EVENT");
    }

    /* The event will not be processed after unregister */
    ESP_ERROR_CHECK(esp_event_handler_instance_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, instance_got_ip));
    ESP_ERROR_CHECK(esp_event_handler_instance_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, instance_any_id));
    vEventGroupDelete(s_wifi_event_group);
}

/*---------*/

void pin_setup(void)
{
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_RTS,
        .rx_flow_ctrl_thresh = 122,
        .source_clk = UART_SCLK_APB,
    };
    uart_driver_install(UART_NUM_0, 4096, 8192, 10, NULL, 0);
    uart_param_config(UART_NUM_0, &uart_config);
    uart_set_pin(UART_NUM_0, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    gpio_set_direction(PIN_SCAN, GPIO_MODE_DEF_OUTPUT);
    gpio_set_level(PIN_SCAN, 0);
    gpio_set_direction(PIN_SEARCH, GPIO_MODE_DEF_OUTPUT);
    gpio_set_level(PIN_SEARCH, 0);
}

void app_main(void)
{
    uint8_t iter1 = 0, iter2 = 0;
    pin_setup();

    printf("Searching scan...\n");
    gpio_set_level(PIN_SEARCH, 1);

    ble_init_fun();
    while((!ble_scan_rdy) && (!ble_scan_cpl)){
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
    esp_ble_gap_stop_scanning();
    gpio_set_level(PIN_SEARCH, 0);
    first_scan = 1;
    printf("Devices found: %d\n", new_data_point_num);
    printf("\n-------------------------------------\n");
    for(uint8_t iter1 = 0; iter1 < new_data_point_num; iter1++){
        printf("Device MAC: %02X:%02X:%02X:%02X:%02X:%02X\n", new_data_points[iter1].ble_mac_adr[0], new_data_points[iter1].ble_mac_adr[1], new_data_points[iter1].ble_mac_adr[2], \
        new_data_points[iter1].ble_mac_adr[3], new_data_points[iter1].ble_mac_adr[4], new_data_points[iter1].ble_mac_adr[5]);
        printf("\n-------------------------------------\n");
    }

    ESP_LOGI(GATTC_TAG, "ESP_WIFI_MODE_STA");

    wifi_init_sta();
    
    // INIT PROTOCOL HERE
    
    while(1){
        ble_scan_rdy = 0;
        ble_scan_cpl = 0;
        ble_num_now = new_data_point_num;
        printf("\nScaning...\n");
        esp_ble_gap_start_scanning(20);
        while((!ble_scan_rdy) && (!ble_scan_cpl)){
            vTaskDelay(100 / portTICK_PERIOD_MS);
        }
        esp_ble_gap_stop_scanning();

        printf("Adverticemenet data results:\n");
        printf("\n-------------------------------------\n");
        for(iter1 = 0; iter1 < new_data_point_num; iter1++){
            printf("Data from: %02X:%02X:%02X:%02X:%02X:%02X\n", new_data_points[iter1].ble_mac_adr[0], new_data_points[iter1].ble_mac_adr[1], new_data_points[iter1].ble_mac_adr[2], \
            new_data_points[iter1].ble_mac_adr[3], new_data_points[iter1].ble_mac_adr[4], new_data_points[iter1].ble_mac_adr[5]);
            if(new_data_points[iter1].ble_data_len > 0){
                printf("Data size: %d\n", new_data_points[iter1].ble_data_len);
                printf("Data: \n");
                for(iter2 = 0; iter2 < new_data_points[iter1].ble_data_len; iter2++){
                    printf("%02X\t",new_data_points[iter1].ble_adv_data[iter2]);
                }
                sprintf(msg, "{\"Smarti_bot\":\"%d\",\"Smarti_mid\":\"%d\",\"Smarti_top\":\"%d\",\"Smarti_batt\":\"%d\",\"Smarti_temp\":\"%d\"}\n", 
                    new_data_points[iter1].ble_adv_data[7], 
                    new_data_points[iter1].ble_adv_data[8], 
                    new_data_points[iter1].ble_adv_data[9], 
                    new_data_points[iter1].ble_adv_data[6], 
                    new_data_points[iter1].ble_adv_data[3]);

                    // SEND ADV DATA TROUGH PROTOCOL

            }
            else{
                printf("No data on scan!\n");
            }
            // printf("\n");
            printf("\n-------------------------------------\n");
            new_data_points[iter1].ble_data_len = 0;
        }


        vTaskDelay(5000 / portTICK_PERIOD_MS);
    }
}
