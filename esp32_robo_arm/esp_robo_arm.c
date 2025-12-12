
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "lwip/err.h"
#include "lwip/sys.h"
#include <sys/param.h>
#include "esp_netif.h"
#include <esp_http_server.h>
#include "esp_mac.h"
#include "protocol_examples_common.h"
#include "lwip/sockets.h"
#include <lwip/netdb.h>

#include "driver/gpio.h"
#include "driver/ledc.h"

#define EXAMPLE_ESP_WIFI_SSID      "ESP32_NET"
#define EXAMPLE_ESP_WIFI_PASS      "ESP32_PASS"
#define EXAMPLE_ESP_WIFI_CHANNEL   1
#define EXAMPLE_MAX_STA_CONN       4

static const char *TAG = "ESP ROBO ARM";

#define SLIDER_COUNT 5

static const char base_html[] = {
"<!DOCTYPE html>"
"<html>"
"<head>"
"<meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">"
"<style>"
"body { font-family: Arial; padding: 20px; }"
".slider { width: 90%%; }"
"</style>"
"</head>"
"<body>"
"<h2>ESP32 ROBO-ARM</h2>"
"<div id='sliders'>@@SLIDERS@@</div>"
"</body>"
"</html>"};

/* Motor sliders */
uint16_t slider_min[SLIDER_COUNT] = {515, 515, 515, 515, 515};
uint16_t slider_max[SLIDER_COUNT] = {2048, 2048, 2048, 2048, 2048};
uint16_t slider_value[SLIDER_COUNT] = {1282, 1282, 1282, 1282, 1282};
char slidr_names[SLIDER_COUNT][25] = {"Hand", "Wrist", "Elbow", "Shoulder", "Shoulder Blade"};
uint8_t motor_pins[SLIDER_COUNT] = {18, 19, 21, 22, 23};
uint8_t motor_chans[SLIDER_COUNT] = {LEDC_CHANNEL_0, LEDC_CHANNEL_1, LEDC_CHANNEL_2, LEDC_CHANNEL_3, LEDC_CHANNEL_4};


/*-------------------------------HTTP SERVER----------------------------------------------------------------*/

static char sliders_html[2048];

static void build_sliders_html()
{
    sliders_html[0] = 0;

    for (int i = 0; i < SLIDER_COUNT; i++)
    {
        char line[256];
        snprintf(line, sizeof(line),
                 "<p>%s<br>"
                 "<input type='range' min='%d' max='%d' value='%d' "
                 "class='slider' id='s%d' "
                 "oninput='fetch(`/set?i=%d&v=${this.value}`)'></p>\n",
                 slidr_names[i],
                 slider_min[i],
                 slider_max[i],
                 slider_value[i],
                 i, i);

        strlcat(sliders_html, line, sizeof(sliders_html));
    }
}

static esp_err_t root_get_handler(httpd_req_t *req)
{
    build_sliders_html();

    /* Prepare output buffer */
    static char out_buf[8192];

    /* Replace @@SLIDERS@@ manually */
    const char *marker = "@@SLIDERS@@";
    const char *pos = strstr(base_html, marker);

    if (!pos)
        return httpd_resp_send_500(req);

    size_t before = pos - base_html;
    size_t after  = strlen(pos + strlen(marker));

    memcpy(out_buf, base_html, before);
    memcpy(out_buf + before, sliders_html, strlen(sliders_html));
    memcpy(out_buf + before + strlen(sliders_html),
           pos + strlen(marker), after);

    size_t total_len = before + strlen(sliders_html) + after;
    out_buf[total_len] = 0;

    /* Send page as chunk */
    httpd_resp_set_type(req, "text/html");
    httpd_resp_send_chunk(req, out_buf, total_len);
    httpd_resp_send_chunk(req, NULL, 0); // end chunk

    return ESP_OK;
}

static esp_err_t set_handler(httpd_req_t *req)
{
    char query[64];
    int slider = 0, value = 0;

    if (httpd_req_get_url_query_str(req, query, sizeof(query)) == ESP_OK)
    {
        char param[16];
        if (httpd_query_key_value(query, "i", param, sizeof(param)) == ESP_OK)
            slider = atoi(param);
        if (httpd_query_key_value(query, "v", param, sizeof(param)) == ESP_OK)
            value  = atoi(param);
    }

    if (slider >= 0 && slider < SLIDER_COUNT){
        slider_value[slider] = value;
        printf("%s = %d\n\r", slidr_names[slider], value);
        ledc_set_duty(LEDC_LOW_SPEED_MODE, motor_chans[slider], value);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, motor_chans[slider]);
    }

    httpd_resp_sendstr(req, "OK");
    return ESP_OK;
}

static httpd_handle_t start_webserver(void)
{
    httpd_handle_t server = NULL;

    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.stack_size = 12288;   // Important fix!

    ESP_LOGI(TAG, "Starting HTTP Server");

    if (httpd_start(&server, &config) == ESP_OK)
    {
        httpd_uri_t root = {
            .uri      = "/",
            .method   = HTTP_GET,
            .handler  = root_get_handler
        };

        httpd_uri_t set_val = {
            .uri      = "/set",
            .method   = HTTP_GET,
            .handler  = set_handler
        };

        httpd_register_uri_handler(server, &root);
        httpd_register_uri_handler(server, &set_val);
    }

    return server;
}

static void stop_webserver(httpd_handle_t server)
{
    // Stop the httpd server
    httpd_stop(server);
}

static void disconnect_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data)
{
    httpd_handle_t* server = (httpd_handle_t*) arg;
    if (*server) {
        ESP_LOGI(TAG, "Stopping webserver");
        stop_webserver(*server);
        *server = NULL;
    }
}

static void connect_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data)
{
    httpd_handle_t* server = (httpd_handle_t*) arg;
    if (*server == NULL) {
        ESP_LOGI(TAG, "Starting webserver");
        *server = start_webserver();
    }
}

/*----------------------------------------------------------------------------------------------------------*/


/*-------------------------------SOFT AP FUNC---------------------------------------------------------------*/

static void wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data)
{
    if (event_id == WIFI_EVENT_AP_STACONNECTED) {
        wifi_event_ap_staconnected_t* event = (wifi_event_ap_staconnected_t*) event_data;
        ESP_LOGI(TAG, "station "MACSTR" join, AID=%d",
                 MAC2STR(event->mac), event->aid);
    } else if (event_id == WIFI_EVENT_AP_STADISCONNECTED) {
        wifi_event_ap_stadisconnected_t* event = (wifi_event_ap_stadisconnected_t*) event_data;
        ESP_LOGI(TAG, "station "MACSTR" leave, AID=%d",
                 MAC2STR(event->mac), event->aid);
    }
}

void wifi_init_softap(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        NULL));

    wifi_config_t wifi_config = {
        .ap = {
            .ssid = EXAMPLE_ESP_WIFI_SSID,
            .ssid_len = strlen(EXAMPLE_ESP_WIFI_SSID),
            .channel = EXAMPLE_ESP_WIFI_CHANNEL,
            .password = EXAMPLE_ESP_WIFI_PASS,
            .max_connection = EXAMPLE_MAX_STA_CONN,
            .authmode = WIFI_AUTH_WPA_WPA2_PSK
        },
    };
    if (strlen(EXAMPLE_ESP_WIFI_PASS) == 0) {
        wifi_config.ap.authmode = WIFI_AUTH_OPEN;
    }

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "wifi_init_softap finished. SSID:%s password:%s channel:%d",
             EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS, EXAMPLE_ESP_WIFI_CHANNEL);
}

/*----------------------------------------------------------------------------------------------------------*/


/*-------------------------------PERIFERY INIT--------------------------------------------------------------*/

void set_motor(void){
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_LOW_SPEED_MODE,
        .timer_num        = LEDC_TIMER_0,
        .duty_resolution  = LEDC_TIMER_14_BIT,
        .freq_hz          = 50,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ledc_timer_config(&ledc_timer);

    // Prepare and then apply the LEDC PWM channel configuration
    ledc_channel_config_t ledc_channel = {
        .speed_mode     = LEDC_LOW_SPEED_MODE,
        .timer_sel      = LEDC_TIMER_0,
        .intr_type      = LEDC_INTR_DISABLE,
        .duty           = 0, 
        .hpoint         = 0
    };

    for(uint8_t i = 0; i < SLIDER_COUNT; i ++){
        ledc_channel.channel = motor_chans[i];
        ledc_channel.gpio_num = motor_pins[i];
        ledc_channel_config(&ledc_channel);
        ledc_set_duty(LEDC_LOW_SPEED_MODE, motor_chans[i], slider_value[i]);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, motor_chans[i]);
    }
}

/*----------------------------------------------------------------------------------------------------------*/

void app_main(void)
{
    static httpd_handle_t server = NULL;

    //Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    vTaskDelay(1000 / portTICK_PERIOD_MS);

    ESP_LOGI(TAG, "ESP_WIFI_MODE_AP");

    wifi_init_softap();

    set_motor();
    
    server = start_webserver();
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_AP_STAIPASSIGNED, &connect_handler, &server));
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, WIFI_EVENT_STA_DISCONNECTED, &disconnect_handler, &server));
}
