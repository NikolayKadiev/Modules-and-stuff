#include  "ble_spp_client.h"

static void esp_gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    uint8_t *adv_name = NULL;
    uint8_t adv_name_len = 0;
    esp_err_t err;

    switch(event){
    case ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT: {
        if((err = param->scan_param_cmpl.status) != ESP_BT_STATUS_SUCCESS){
            break;
        }
        //the unit of the duration is second
        uint32_t duration = 0xFFFF;
        esp_ble_gap_start_scanning(duration);
        break;
    }
    case ESP_GAP_BLE_SCAN_START_COMPLETE_EVT:
        //scan start complete event to indicate scan start successfully or failed
        if ((err = param->scan_start_cmpl.status) != ESP_BT_STATUS_SUCCESS) {
            break;
        }
        break;
    case ESP_GAP_BLE_SCAN_STOP_COMPLETE_EVT:
        if ((err = param->scan_stop_cmpl.status) != ESP_BT_STATUS_SUCCESS) {
            break;
        }
        if (is_connect == false) {
            esp_ble_gattc_open(gl_profile_tab[PROFILE_APP_ID].gattc_if, scan_rst.scan_rst.bda, scan_rst.scan_rst.ble_addr_type, true);
        }
        break;
    case ESP_GAP_BLE_SCAN_RESULT_EVT: {
        esp_ble_gap_cb_param_t *scan_result = (esp_ble_gap_cb_param_t *)param;
        switch (scan_result->scan_rst.search_evt) {
        case ESP_GAP_SEARCH_INQ_RES_EVT:
        	adv_name = esp_ble_resolve_adv_data(scan_result->scan_rst.ble_adv, ESP_BLE_AD_TYPE_NAME_CMPL, &adv_name_len);
            if (adv_name != NULL) {
                if ( strncmp((char *)adv_name, device_name, adv_name_len) == 0) {
                    memcpy(&(scan_rst), scan_result, sizeof(esp_ble_gap_cb_param_t));
                    esp_ble_gap_stop_scanning();
                }
            }
            break;
        case ESP_GAP_SEARCH_INQ_CMPL_EVT:
            break;
        default:
            break;
        }
        break;
    }
    case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
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

static void gattc_profile_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param)
{
    esp_ble_gattc_cb_param_t *p_data = (esp_ble_gattc_cb_param_t *)param;

    switch (event) {
    case ESP_GATTC_REG_EVT:
    	esp_ble_gap_set_scan_params(&ble_scan_params);
        break;
    case ESP_GATTC_CONNECT_EVT:
    	spp_gattc_if = gattc_if;
        is_connect = true;
        spp_conn_id = p_data->connect.conn_id;
        memcpy(gl_profile_tab[PROFILE_APP_ID].remote_bda, p_data->connect.remote_bda, sizeof(esp_bd_addr_t));
        esp_ble_gattc_search_service(spp_gattc_if, spp_conn_id, &spp_service_uuid);
        break;
    case ESP_GATTC_DISCONNECT_EVT:
        // free_gattc_srv_db();
        is_connect = false;
        spp_gattc_if = 0xff;
        spp_conn_id = 0;
        spp_mtu_size = 23;
        cmd = 0;
        spp_srv_start_handle = 0;
        spp_srv_end_handle = 0;
        notify_value_p = NULL;
        notify_value_offset = 0;
        notify_value_count = 0;
        if(db){
            free(db);
            db = NULL;
        }
        esp_ble_gap_start_scanning(SCAN_ALL_THE_TIME);
        break;
    case ESP_GATTC_SEARCH_RES_EVT:
        spp_srv_start_handle = p_data->search_res.start_handle;
        spp_srv_end_handle = p_data->search_res.end_handle;
        break;
    case ESP_GATTC_SEARCH_CMPL_EVT:
        esp_ble_gattc_send_mtu_req(gattc_if, spp_conn_id);
        break;
    case ESP_GATTC_REG_FOR_NOTIFY_EVT: {
        if(p_data->reg_for_notify.status != ESP_GATT_OK){
           break;
        }
        uint16_t notify_en = 1;
        esp_ble_gattc_write_char_descr(
                spp_gattc_if,
                spp_conn_id,
                (db+cmd+1)->attribute_handle,
                sizeof(notify_en),
                (uint8_t *)&notify_en,
                ESP_GATT_WRITE_TYPE_RSP,
                ESP_GATT_AUTH_REQ_NONE);

        break;
    }
    case ESP_GATTC_NOTIFY_EVT:
        uart_write_bytes(UART_NUM_0, (char *)(p_data->notify.value), p_data->notify.value_len);
        //notify_event_handler(p_data);
        break;
    case ESP_GATTC_READ_CHAR_EVT:
        break;
    case ESP_GATTC_WRITE_CHAR_EVT:
        if(param->write.status != ESP_GATT_OK){
            break;
        }
        break;
    case ESP_GATTC_PREP_WRITE_EVT:
        break;
    case ESP_GATTC_EXEC_EVT:
        break;
    case ESP_GATTC_WRITE_DESCR_EVT:
        if(p_data->write.status != ESP_GATT_OK){
            break;
        }
        switch(cmd){
        case SPP_IDX_SPP_DATA_NTY_VAL:
            cmd = SPP_IDX_SPP_STATUS_VAL;
            xQueueSend(cmd_reg_queue, &cmd,10/portTICK_PERIOD_MS);
            break;
        case SPP_IDX_SPP_STATUS_VAL:
            break;
        default:
            break;
        };
        break;
    case ESP_GATTC_CFG_MTU_EVT:
        if(p_data->cfg_mtu.status != ESP_OK){
            break;
        }
        spp_mtu_size = p_data->cfg_mtu.mtu;

        db = (esp_gattc_db_elem_t *)malloc(count*sizeof(esp_gattc_db_elem_t));
        if(db == NULL){
            break;
        }
        if(esp_ble_gattc_get_db(spp_gattc_if, spp_conn_id, spp_srv_start_handle, spp_srv_end_handle, db, &count) != ESP_GATT_OK){
            break;
        }
        if(count != SPP_IDX_NB){
            break;
        }
        for(int i = 0;i < SPP_IDX_NB;i++){
            switch((db+i)->type){
            case ESP_GATT_DB_PRIMARY_SERVICE:
                break;
            case ESP_GATT_DB_SECONDARY_SERVICE:
                break;
            case ESP_GATT_DB_CHARACTERISTIC:
                break;
            case ESP_GATT_DB_DESCRIPTOR:
                break;
            case ESP_GATT_DB_INCLUDED_SERVICE:
                break;
            case ESP_GATT_DB_ALL:
                break;
            default:
                break;
            }
        }
        cmd = SPP_IDX_SPP_DATA_NTY_VAL;
        xQueueSend(cmd_reg_queue, &cmd, 10/portTICK_PERIOD_MS);
        break;
    case ESP_GATTC_SRVC_CHG_EVT:
        break;
    default:
        break;
    }
}

void spp_client_reg_task(void* arg)
{
    uint16_t cmd_id;
    for(;;) {
        vTaskDelay(100 / portTICK_PERIOD_MS);
        if(xQueueReceive(cmd_reg_queue, &cmd_id, portMAX_DELAY)) {
            if(db != NULL) {
                if(cmd_id == SPP_IDX_SPP_DATA_NTY_VAL){
                    esp_ble_gattc_register_for_notify(spp_gattc_if, gl_profile_tab[PROFILE_APP_ID].remote_bda, (db+SPP_IDX_SPP_DATA_NTY_VAL)->attribute_handle);
                }else if(cmd_id == SPP_IDX_SPP_STATUS_VAL){
                    esp_ble_gattc_register_for_notify(spp_gattc_if, gl_profile_tab[PROFILE_APP_ID].remote_bda, (db+SPP_IDX_SPP_STATUS_VAL)->attribute_handle);
                }
            }
        }
    }
}

void uart_task(void *pvParameters)
{
    uart_event_t event;
    for (;;) {
        //Waiting for UART event.
        if (xQueueReceive(spp_uart_queue, (void * )&event, (portTickType)portMAX_DELAY)) {
            switch (event.type) {
            //Event of UART receving data
            case UART_DATA:
                if (event.size && (is_connect == true) && (db != NULL) && ((db+SPP_IDX_SPP_DATA_RECV_VAL)->properties & (ESP_GATT_CHAR_PROP_BIT_WRITE_NR | ESP_GATT_CHAR_PROP_BIT_WRITE))) {
                    uint8_t * temp = NULL;
                    temp = (uint8_t *)malloc(sizeof(uint8_t)*event.size);
                    if(temp == NULL){
                        ESP_LOGE(GATTC_TAG, "malloc failed,%s L#%d\n", __func__, __LINE__);
                        break;
                    }
                    memset(temp, 0x0, event.size);
                    uart_read_bytes(UART_NUM_0,temp,event.size,portMAX_DELAY);
                    esp_ble_gattc_write_char( spp_gattc_if,
                                              spp_conn_id,
                                              (db+SPP_IDX_SPP_DATA_RECV_VAL)->attribute_handle,
                                              event.size,
                                              temp,
                                              ESP_GATT_WRITE_TYPE_RSP,
                                              ESP_GATT_AUTH_REQ_NONE);
                    free(temp);
                }
                break;
            default:
                break;
            }
        }
    }
    vTaskDelete(NULL);
}

void app_main(void)
{
    esp_err_t ret;

    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();

    nvs_flash_init();
    esp_bt_controller_init(&bt_cfg);
    esp_bt_controller_enable(ESP_BT_MODE_BLE);
    esp_bluedroid_init();
    esp_bluedroid_enable();
   
    //register the scan callback function to the gap module
    esp_ble_gap_register_callback(esp_gap_cb);
    //register the callback function to the gattc module
    esp_ble_gattc_register_callback(esp_gattc_cb);
    esp_ble_gattc_app_register(PROFILE_APP_ID);
    esp_ble_gatt_set_local_mtu(200);

    cmd_reg_queue = xQueueCreate(10, sizeof(uint32_t));
    xTaskCreate(spp_client_reg_task, "spp_client_reg_task", 2048, NULL, 10, NULL);

    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_RTS,
        .rx_flow_ctrl_thresh = 122,
        .source_clk = UART_SCLK_APB,
    };

    //Install UART driver, and get the queue.
    uart_driver_install(UART_NUM_0, 4096, 8192, 10, &spp_uart_queue, 0);
    //Set UART parameters
    uart_param_config(UART_NUM_0, &uart_config);
    //Set UART pins
    uart_set_pin(UART_NUM_0, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    xTaskCreate(uart_task, "uTask", 2048, (void*)UART_NUM_0, 8, NULL);
}
