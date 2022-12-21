/*
 * Written by Thành Nhân <yesthanhnhan@gmail.com>
 * Copyright (C) 21/12/2022
 *
*/

#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include <stdarg.h>
#include <stdlib.h>
#include <errno.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/timers.h"
#include "freertos/event_groups.h"
#include <driver/gpio.h>
#include "esp_system.h"
#include "esp_task.h"
#include "esp_log.h"
#include "esp_event.h"
#include "esp_wifi.h"
#include "esp_http_client.h"
#include "nvs_flash.h"

#include "lwip/err.h"
#include <dht.h>

#define     BLYNK_AUTH_TOKEN        "K4HSc6ttnPha6dyF2CdN_A_4JsNfIxbD"
#define     SERVER                  "blynk.cloud"
#define     PORT                    "8080"

#define     MAX_HTTP_RECV_BUFFER    4

#define     SENSOR_TYPE             DHT_TYPE_AM2301
#define     SENSOR_PIN              15

#define     EXAMPLE_ESP_WIFI_SSID       "NhanSgu"
#define     EXAMPLE_ESP_WIFI_PASS       "123456789"
#define     EXAMPLE_ESP_MAXIMUM_RETRY   5

#define     WIFI_CONNECTED_BIT          BIT0
#define     WIFI_FAIL_BIT               BIT1

static const char               *TAG                    = "UPDATE_DATA";
static const char               *TAG_BUTTON_BLYNK       = "BUTTON_BLYNK";
static  const char              *TAG_WIFI               = "wifi station";

static  EventGroupHandle_t      s_wifi_event_group;
static  float                   temperature, humidity;
static  int                     s_retry_num             = 0;
int                             button_blynk_response;

void                            dht_test(void *pvParameters);

static  void            event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data);
void                    wifi_init_sta(void);
esp_err_t               client_event_post_handler(esp_http_client_event_handle_t evt);

char                    *form_http_request(char *pin);
char                    *send_http_request_and_parse_response(char *http_request);
void                    send_http_request_and_no_parse_response(char *http_request);
void                    write_http_request(char *pinTemperature, char *pinHumidity);
static void             check_button();
static void             update_data();

void dht_test(void *pvParameters)
{

    while (1)
    {
        if (dht_read_float_data(SENSOR_TYPE, 15, &humidity, &temperature) == ESP_OK)
            printf("Humidity: %.1f%% Temp: %.1fC\n", humidity, temperature);
        else
            printf("Could not read data from sensor\n");
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

static void event_handler(void *arg, esp_event_base_t event_base,
                          int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START)
    {
        esp_wifi_connect();
    }
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED)
    {
        if (s_retry_num < EXAMPLE_ESP_MAXIMUM_RETRY)
        {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAG_WIFI, "retry to connect to the AP");
        }
        else
        {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
        ESP_LOGI(TAG_WIFI, "connect to the AP fail");
    }
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)
    {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG_WIFI, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

void wifi_init_sta()
{
    s_wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());

    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    // có bất kỳ event nào của wifi thì chạy vào hàm event_handler
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &event_handler,
                                                        NULL,
                                                        NULL));

    // có bất kỳ event nào của IP thì chạy vào hàm event_handler
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &event_handler,
                                                        NULL,
                                                        NULL));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = EXAMPLE_ESP_WIFI_SSID,
            .password = EXAMPLE_ESP_WIFI_PASS,
            /* Setting a password implies station will connect to all security modes including WEP/WPA.
             * However these modes are deprecated and not advisable to be used. Incase your Access point
             * doesn't support WPA2, these mode can be enabled by commenting below line */
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
            .pmf_cfg = {
                .capable = true,
                .required = false},
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_LOGI(TAG_WIFI, "wifi_init_sta finished.");

    /* Waiting until either the connection is established (WIFI_CONNECTED_BIT) or connection failed for the maximum
     * number of re-tries (WIFI_FAIL_BIT). The bits are set by event_handler() (see above) */

    // đợi 2 bit WIFI_CONNECTED_BIT WIFI_FAIL_BIT được set
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
                                           WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                                           pdFALSE,
                                           pdFALSE,
                                           portMAX_DELAY);

    /* xEventGroupWaitBits() returns the bits before the call returned, hence we can test which event actually
     * happened. */
    if (bits & WIFI_CONNECTED_BIT)
    {
        ESP_LOGI(TAG_WIFI, "connected to ap SSID:%s password:%s",
                 EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS);
    }
    else if (bits & WIFI_FAIL_BIT)
    {
        ESP_LOGI(TAG_WIFI, "Failed to connect to SSID:%s, password:%s",
                 EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS);
    }
    else
    {
        ESP_LOGE(TAG_WIFI, "UNEXPECTED EVENT");
    }
}

esp_err_t client_event_post_handler(esp_http_client_event_handle_t evt)
{
    switch (evt->event_id)
    {
    case HTTP_EVENT_ON_DATA:
        printf("HTTP_EVENT_ON_DATA: %.*s\n", evt->data_len, (char *)evt->data);
        break;

    default:
        break;
    }
    return ESP_OK;
}

char *form_http_request(char *pin)
{
    static char http_request[500];
    bzero(http_request, sizeof(http_request));
    strcat(http_request, "http://");
    strcat(http_request, SERVER);
    strcat(http_request, ":");
    strcat(http_request, PORT);
    strcat(http_request, "/external/api/get");
    strcat(http_request, "?token=");
    strcat(http_request, BLYNK_AUTH_TOKEN);
    strcat(http_request, "&pin=");
    strcat(http_request, pin);
    return http_request;
}

void send_http_request_and_no_parse_response(char *http_request)
{
    esp_http_client_config_t config_get = {
        .url = http_request,
        .method = HTTP_METHOD_GET,
        .cert_pem = NULL,
        .event_handler = client_event_post_handler};

    esp_http_client_handle_t client = esp_http_client_init(&config_get);
    esp_http_client_perform(client);
    esp_http_client_cleanup(client);
}

void write_http_request(char *pinTemperature, char *pinHumidity)
{
    static char http_request[500];
    char data[8];
    char data2[8];

    sprintf(data, "%.1f", temperature);
    sprintf(data2, "%.1f", humidity);

    bzero(http_request, sizeof(http_request));
    strcat(http_request, "http://");
    strcat(http_request, SERVER);
    strcat(http_request, ":");
    strcat(http_request, PORT);
    strcat(http_request, "/external/api/batch/update");
    strcat(http_request, "?token=");
    strcat(http_request, BLYNK_AUTH_TOKEN);
    strcat(http_request, "&");
    strcat(http_request, pinTemperature);
    strcat(http_request, "=");
    strcat(http_request, data);
    strcat(http_request, "&");
    strcat(http_request, pinHumidity);
    strcat(http_request, "=");
    strcat(http_request, data2);
    strcat(http_request, "&");
    strcat(http_request, "v3");
    strcat(http_request, "=");
    strcat(http_request, data2);
    send_http_request_and_no_parse_response(http_request);
}

static void update_data()
{
    while (1)
    {
        if (button_blynk_response == 1)
        {
            write_http_request("v0", "v1");
        }
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

char *send_http_request_and_parse_response(char *http_request)
{
    static char res_buffer[MAX_HTTP_RECV_BUFFER];
    bzero(res_buffer, sizeof(res_buffer));

    if (res_buffer == NULL)
    {
        ESP_LOGI(TAG_BUTTON_BLYNK, "Cannot malloc http receive buffer");
        return "\0";
    }
    esp_http_client_config_t config = {
        .url = http_request,
        .method = HTTP_METHOD_GET,
        .cert_pem = NULL,
        .event_handler = client_event_post_handler};
    esp_http_client_handle_t client = esp_http_client_init(&config);
    esp_err_t err;
    if ((err = esp_http_client_open(client, 0)) != ESP_OK)
    {
        ESP_LOGI(TAG_BUTTON_BLYNK, "Failed to open HTTP connection: %s", esp_err_to_name(err));
        return "\0";
    }
    int content_length = esp_http_client_fetch_headers(client);
    int total_read_len = 0, read_len;
    if (total_read_len < content_length && content_length <= MAX_HTTP_RECV_BUFFER)
    {
        read_len = esp_http_client_read(client, res_buffer, content_length);
        if (read_len <= 0)
        {
            ESP_LOGI(TAG_BUTTON_BLYNK, "Error read data");
        }
        res_buffer[read_len] = 0;
    }

    esp_http_client_close(client);
    esp_http_client_cleanup(client);
    return res_buffer;
}

static void check_button()
{
    while (1)
    {
        char http_request[500];
        strcpy(http_request, form_http_request("v2"));
        button_blynk_response = atoi(send_http_request_and_parse_response(http_request));
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

void app_main(void)
{
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_LOGI(TAG, "ESP_WIFI_MODE_STA");
    wifi_init_sta();

    xTaskCreate(dht_test, "dht_test", configMINIMAL_STACK_SIZE * 3, NULL, 1, NULL);
    /* Start Blynk client task */
    xTaskCreate(check_button, "HTTP request for valve control", 4096, NULL, 1, NULL);
    xTaskCreate(update_data, "HTTP request for valve control", 4096, NULL, 1, NULL);
}
