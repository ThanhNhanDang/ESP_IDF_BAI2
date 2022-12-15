/*
 * Written by Thành Nhân <yesthanhnhan@gmail.com>
 * Copyright (C) 2022
 *
 * Chương trình đọc dữ liệu nhiệt độ và độ ẩm từ cảm biến DHT22. Hiển thị lên app Blynk.
 * App Blynk có:
 * -Hiển thi nhiệt độ và độ ẩm dạng biểu đồ gauge
 * -Biểu đồ đường thể hiện độ ẩm hiện tại
 * -Nút nhấn để dừng việc cập nhật dữ liệu
 *
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice, this permission notice, and the following
 * disclaimer shall be included in all copies or substantial portions of
 * the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OF OR OTHER
 * DEALINGS IN THE SOFTWARE.
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
#include "lwip/sys.h"

#define CHECK_ARG(VAL)                  \
    do                                  \
    {                                   \
        if (!(VAL))                     \
            return ESP_ERR_INVALID_ARG; \
    } while (0)

#define CHECK_LOGE(x, msg, ...)                \
    do                                         \
    {                                          \
        esp_err_t __;                          \
        if ((__ = x) != ESP_OK)                \
        {                                      \
            PORT_EXIT_CRITICAL();              \
            ESP_LOGE(TAG, msg, ##__VA_ARGS__); \
            return __;                         \
        }                                      \
    } while (0)

#define     PORT_ENTER_CRITICAL()   portENTER_CRITICAL(&mux)
#define     PORT_EXIT_CRITICAL()    portEXIT_CRITICAL(&mux)

#define     BLYNK_AUTH_TOKEN        "K4HSc6ttnPha6dyF2CdN_A_4JsNfIxbD"
#define     SERVER                  "blynk.cloud"
#define     PORT                    "8080"

#define     MAX_HTTP_RECV_BUFFER    4

#define     SENSOR_TYPE             DHT_TYPE_AM2301
#define     SENSOR_PIN              15
#define     DHT_TIMER_INTERVAL      2
#define     DHT_DATA_BITS           40
#define     DHT_DATA_BYTES          (DHT_DATA_BITS / 8)

#define     EXAMPLE_ESP_WIFI_SSID       "Hai Anh"
#define     EXAMPLE_ESP_WIFI_PASS       "123456789"
#define     EXAMPLE_ESP_MAXIMUM_RETRY   5

#define     WIFI_CONNECTED_BIT          BIT0
#define     WIFI_FAIL_BIT               BIT1

static const char               *TAG                    = "UPDATE_DATA";
static const char               *TAG_BUTTON_BLYNK       = "BUTTON_BLYNK";
static  const char              *TAG_WIFI               = "wifi station";
static portMUX_TYPE             mux                     = portMUX_INITIALIZER_UNLOCKED;

static  EventGroupHandle_t      s_wifi_event_group;
static  float                   temperature, humidity;
static  int                     s_retry_num             = 0;
int                             button_blynk_response;

typedef enum
{
    DHT_TYPE_AM2301, //!< AM2301 (DHT21, DHT22, AM2302, AM2321)
} dht_sensor_type_t;

static inline   int16_t         dht_convert_data(dht_sensor_type_t sensor_type, uint8_t msb, uint8_t lsb);
static inline   esp_err_t       dht_fetch_data(dht_sensor_type_t sensor_type, gpio_num_t pin, uint8_t data[DHT_DATA_BYTES]);
static  esp_err_t               dht_await_pin_state(gpio_num_t pin, uint32_t timeout, int expected_pin_state, uint32_t *duration);
esp_err_t                       dht_read_data(dht_sensor_type_t sensor_type, gpio_num_t pin, int16_t *humidity, int16_t *temperature);
esp_err_t                       dht_read_float_data(dht_sensor_type_t sensor_type, gpio_num_t pin, float *humidity, float *temperature);
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

static inline int16_t dht_convert_data(dht_sensor_type_t sensor_type, uint8_t msb, uint8_t lsb)
{
    int16_t data;

    data = msb & 0x7F;
    data <<= 8;
    data |= lsb;
    if (msb & BIT(7))
        data = -data; // convert it to negative

    return data;
}
static esp_err_t dht_await_pin_state(gpio_num_t pin, uint32_t timeout,
                                     int expected_pin_state, uint32_t *duration)
{
    /* XXX dht_await_pin_state() should save pin direction and restore
     * the direction before return. however, the SDK does not provide
     * gpio_get_direction().
     */
    gpio_set_direction(pin, GPIO_MODE_INPUT);
    for (uint32_t i = 0; i < timeout; i += DHT_TIMER_INTERVAL)
    {
        // need to wait at least a single interval to prevent reading a jitter
        ets_delay_us(DHT_TIMER_INTERVAL);
        if (gpio_get_level(pin) == expected_pin_state)
        {
            if (duration)
                *duration = i;
            return ESP_OK;
        }
    }

    return ESP_ERR_TIMEOUT;
}

static inline esp_err_t dht_fetch_data(dht_sensor_type_t sensor_type, gpio_num_t pin, uint8_t data[DHT_DATA_BYTES])
{
    uint32_t low_duration;
    uint32_t high_duration;

    // Phase 'A' pulling signal low to initiate read sequence
    gpio_set_direction(pin, GPIO_MODE_OUTPUT_OD);
    gpio_set_level(pin, 0);
    ets_delay_us(20000);
    gpio_set_level(pin, 1);

    // Step through Phase 'B', 40us
    CHECK_LOGE(dht_await_pin_state(pin, 40, 0, NULL),
               "Initialization error, problem in phase 'B'");
    // Step through Phase 'C', 88us
    CHECK_LOGE(dht_await_pin_state(pin, 88, 1, NULL),
               "Initialization error, problem in phase 'C'");
    // Step through Phase 'D', 88us
    CHECK_LOGE(dht_await_pin_state(pin, 88, 0, NULL),
               "Initialization error, problem in phase 'D'");

    // Read in each of the 40 bits of data...
    for (int i = 0; i < DHT_DATA_BITS; i++)
    {
        CHECK_LOGE(dht_await_pin_state(pin, 65, 1, &low_duration),
                   "LOW bit timeout");
        CHECK_LOGE(dht_await_pin_state(pin, 75, 0, &high_duration),
                   "HIGH bit timeout");

        uint8_t b = i / 8;
        uint8_t m = i % 8;
        if (!m)
            data[b] = 0;

        data[b] |= (high_duration > low_duration) << (7 - m);
    }

    return ESP_OK;
}

esp_err_t dht_read_data(dht_sensor_type_t sensor_type, gpio_num_t pin,
                        int16_t *humidity, int16_t *temperature)
{
    CHECK_ARG(humidity || temperature);

    uint8_t data[DHT_DATA_BYTES] = {0};

    gpio_set_direction(pin, GPIO_MODE_OUTPUT_OD);
    gpio_set_level(pin, 1);

    PORT_ENTER_CRITICAL();
    esp_err_t result = dht_fetch_data(sensor_type, pin, data);
    if (result == ESP_OK)
        PORT_EXIT_CRITICAL();

    /* restore GPIO direction because, after calling dht_fetch_data(), the
     * GPIO direction mode changes */
    gpio_set_direction(pin, GPIO_MODE_OUTPUT_OD);
    gpio_set_level(pin, 1);

    if (result != ESP_OK)
        return result;

    if (data[4] != ((data[0] + data[1] + data[2] + data[3]) & 0xFF))
    {
        ESP_LOGE(TAG, "Checksum failed, invalid data received from sensor");
        return ESP_ERR_INVALID_CRC;
    }

    if (humidity)
        *humidity = dht_convert_data(sensor_type, data[0], data[1]);
    if (temperature)
        *temperature = dht_convert_data(sensor_type, data[2], data[3]);

    ESP_LOGD(TAG, "Sensor data: humidity=%d, temp=%d", *humidity, *temperature);

    return ESP_OK;
}

esp_err_t dht_read_float_data(dht_sensor_type_t sensor_type, gpio_num_t pin,
                              float *humidity, float *temperature)
{
    CHECK_ARG(humidity || temperature);

    int16_t i_humidity, i_temp;

    esp_err_t res = dht_read_data(sensor_type, pin, humidity ? &i_humidity : NULL, temperature ? &i_temp : NULL);
    if (res != ESP_OK)
        return res;

    if (humidity)
        *humidity = i_humidity / 10.0;
    if (temperature)
        *temperature = i_temp / 10.0;

    return ESP_OK;
}

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
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}