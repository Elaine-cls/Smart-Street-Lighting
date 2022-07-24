/* ADC1 Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
// libs cod iot
#include <string.h>
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
//#include "protocol_examples_common.h"
#include "freertos/event_groups.h"

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include "lwip/netdb.h"
#include "lwip/dns.h"
#include "sdkconfig.h"

// fim libs

// lib queue
#include "freertos/queue.h"
// fim lib

#include <stdio.h>
#include <stdlib.h>
#include "string.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"


//libs do ADC
#include "driver/adc.h"
#include "esp_adc_cal.h"

//lib do DAC
#include "driver/dac.h"

// defines do cod iot

/* The examples use WiFi configuration that you can set via project configuration menu

   If you'd rather not, just change the below entries to strings with
   the config you want - ie #define EXAMPLE_WIFI_SSID "mywifissid"
*/
#define EXAMPLE_ESP_WIFI_SSID      CONFIG_ESP_WIFI_SSID
#define EXAMPLE_ESP_WIFI_PASS      CONFIG_ESP_WIFI_PASSWORD
#define EXAMPLE_ESP_MAXIMUM_RETRY  CONFIG_ESP_MAXIMUM_RETRY
#if CONFIG_ESP_WIFI_AUTH_OPEN
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_OPEN
#elif CONFIG_ESP_WIFI_AUTH_WEP
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WEP
#elif CONFIG_ESP_WIFI_AUTH_WPA_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA_PSK
#elif CONFIG_ESP_WIFI_AUTH_WPA2_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA2_PSK
#elif CONFIG_ESP_WIFI_AUTH_WPA_WPA2_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA_WPA2_PSK
#elif CONFIG_ESP_WIFI_AUTH_WPA3_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA3_PSK
#elif CONFIG_ESP_WIFI_AUTH_WPA2_WPA3_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA2_WPA3_PSK
#elif CONFIG_ESP_WIFI_AUTH_WAPI_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WAPI_PSK
#endif
/* FreeRTOS event group to signal when we are connected*/
static EventGroupHandle_t s_wifi_event_group;

/* The event group allows multiple bits for each event, but we only care about two events:
 * - we are connected to the AP with an IP
 * - we failed to connect after the maximum amount of retries */
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

/* Constants that aren't configurable in menuconfig */
#define WEB_SERVER "api.thingspeak.com"
#define WEB_PORT "80"
// The API key below is configurable in menuconfig
#define THINGSPEAK_WRITE_API_KEY CONFIG_THINGSPEAK_WRITE_API_KEY

// fim defines

#define LED DAC_CHANNEL_1   //GPIO25 fará a leitura

#define DEFAULT_VREF    1100        //Tensao de referencia
#define NO_OF_SAMPLES   64          //taxa de amostragem

// statics cod iot
static const char *TAG = "wifi station";

static int s_retry_num = 0;

static const char* get_request_start =
    "GET /update?key="
    THINGSPEAK_WRITE_API_KEY;

static const char* get_request_end =
    " HTTP/1.1\n"
    "Host: "WEB_SERVER"\n"
    "Connection: close\n"
    "User-Agent: esp32 / esp-idf\n"
    "\n";

// fim statics
static TaskHandle_t LumControlTask = NULL;

static QueueHandle_t data_manager_queue = NULL;
static QueueHandle_t luminosity_queue = NULL;

static esp_adc_cal_characteristics_t *adc_chars;
#if CONFIG_IDF_TARGET_ESP32
static const adc_channel_t channel = ADC_CHANNEL_6;     //GPIO34
static const adc_bits_width_t width = ADC_WIDTH_BIT_12; // 12 bits de resolucao
#elif CONFIG_IDF_TARGET_ESP32S2
static const adc_channel_t channel = ADC_CHANNEL_6;     // GPIO7 if ADC1, GPIO17 if ADC2
static const adc_bits_width_t width = ADC_WIDTH_BIT_13;
#endif
static const adc_atten_t atten = ADC_ATTEN_DB_6;
static const adc_unit_t unit = ADC_UNIT_1;

// func cod iot

static void event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < EXAMPLE_ESP_MAXIMUM_RETRY) {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAG, "retry to connect to the AP");
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
        ESP_LOGI(TAG,"connect to the AP fail");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
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
            /* Setting a password implies station will connect to all security modes including WEP/WPA.
             * However these modes are deprecated and not advisable to be used. Incase your Access point
             * doesn't support WPA2, these mode can be enabled by commenting below line */
	     .threshold.authmode = ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD,
	     .sae_pwe_h2e = 2,
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config) );
    ESP_ERROR_CHECK(esp_wifi_start() );

    ESP_LOGI(TAG, "wifi_init_sta finished.");

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
        ESP_LOGI(TAG, "connected to ap SSID:%s password:%s",
                 EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS);
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGI(TAG, "Failed to connect to SSID:%s, password:%s",
                 EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS);
    } else {
        ESP_LOGE(TAG, "UNEXPECTED EVENT");
    }
}

static void http_get_task(void *pvParameters)
{
    const struct addrinfo hints = {
        .ai_family = AF_INET,
        .ai_socktype = SOCK_STREAM,
    };
    struct addrinfo *res;
    struct in_addr *addr;
    int s, r;
    char recv_buf[64];

    while(1) {
        int arg = 0;
        int porcentagem_luminosidade = 0;
        vTaskDelay(20000 / portTICK_PERIOD_MS);
        xQueueReset(data_manager_queue);
        xQueueReset(luminosity_queue);
        if(xQueueReceive(data_manager_queue, &arg, portMAX_DELAY) && xQueueReceive(luminosity_queue, &porcentagem_luminosidade, portMAX_DELAY)){
            int n;
            // conversion of values to character strings
            n = snprintf(NULL, 0, "%lu", arg);
            char field1[n+1];
            sprintf(field1, "%lu", arg);

            printf("AQUIIII\n");
            printf("%d\n",porcentagem_luminosidade);
            n = snprintf(NULL, 0, "%lu", porcentagem_luminosidade);
            char field2[n+1];
            sprintf(field2, "%lu", porcentagem_luminosidade);
            
            int string_size = strlen(get_request_start);
            string_size += strlen("&field1=")*2;
            string_size += strlen(field1);
            string_size += strlen(field2);
            string_size += strlen(get_request_end);
            string_size += 1;  // '\0' - space for string termination character
            
            // request string assembly / concatenation
            char * get_request = malloc(string_size);
            
            strcpy(get_request, get_request_start);
            strcat(get_request, "&field1=");
            strcat(get_request, field1);
            strcat(get_request, "&field2=");
            strcat(get_request, field2);
            strcat(get_request, get_request_end);

            int err = getaddrinfo(WEB_SERVER, WEB_PORT, &hints, &res);
            if(err != 0 || res == NULL) {
                ESP_LOGE(TAG, "DNS lookup failed err=%d res=%p", err, res);
                vTaskDelay(1000 / portTICK_PERIOD_MS);
                continue;
            }

            /* Code to print the resolved IP.

            Note: inet_ntoa is non-reentrant, look at ipaddr_ntoa_r for "real" code */
            addr = &((struct sockaddr_in *)res->ai_addr)->sin_addr;
            ESP_LOGI(TAG, "DNS lookup succeeded. IP=%s", inet_ntoa(*addr));

            s = socket(res->ai_family, res->ai_socktype, 0);
            if(s < 0) {
                ESP_LOGE(TAG, "... Failed to allocate socket.");
                freeaddrinfo(res);
                vTaskDelay(1000 / portTICK_PERIOD_MS);
                continue;
            }
            ESP_LOGI(TAG, "... allocated socket");
            if(connect(s, res->ai_addr, res->ai_addrlen) != 0) {
                ESP_LOGE(TAG, "... socket connect failed errno=%d", errno);
                close(s);
                freeaddrinfo(res);
                vTaskDelay(4000 / portTICK_PERIOD_MS);
                continue;
            }

            ESP_LOGI(TAG, "... connected");
            freeaddrinfo(res);

            if (write(s, get_request, strlen(get_request)) < 0) {
                ESP_LOGE(TAG, "... socket send failed");
                close(s);
                vTaskDelay(4000 / portTICK_PERIOD_MS);
                continue;
            }
            ESP_LOGI(TAG, "... socket send success");

            struct timeval receiving_timeout;
            receiving_timeout.tv_sec = 5;
            receiving_timeout.tv_usec = 0;
            if (setsockopt(s, SOL_SOCKET, SO_RCVTIMEO, &receiving_timeout,
                    sizeof(receiving_timeout)) < 0) {
                ESP_LOGE(TAG, "... failed to set socket receiving timeout");
                close(s);
                vTaskDelay(4000 / portTICK_PERIOD_MS);
                continue;
            }
            ESP_LOGI(TAG, "... set socket receiving timeout success");

            /* Read HTTP response */
            do {
                bzero(recv_buf, sizeof(recv_buf));
                r = read(s, recv_buf, sizeof(recv_buf)-1);
                for(int i = 0; i < r; i++) {
                    putchar(recv_buf[i]);
                }
            } while(r > 0);

            ESP_LOGI(TAG, "... done reading from socket. Last read return=%d errno=%d.", r, errno);
            close(s);
            // for(int countdown = 10; countdown >= 0; countdown--) {
            //     ESP_LOGI(TAG, "%d... ", countdown);
            //     vTaskDelay(1000 / portTICK_PERIOD_MS);
            // }
            ESP_LOGI(TAG, "Starting again!");
            free(get_request);
        }
    }
}
// fim func


// Mensagens sobre o status/ funcionamento da tensao de referencia
static void check_efuse(void)
{
#if CONFIG_IDF_TARGET_ESP32
    //Check if TP is burned into eFuse
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_TP) == ESP_OK) {
        printf("eFuse Two Point: Supported\n");
    } else {
        printf("eFuse Two Point: NOT supported\n");
    }
    //Check Vref is burned into eFuse
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_VREF) == ESP_OK) {
        printf("eFuse Vref: Supported\n");
    } else {
        printf("eFuse Vref: NOT supported\n");
    }
#elif CONFIG_IDF_TARGET_ESP32S2
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_TP) == ESP_OK) {
        printf("eFuse Two Point: Supported\n");
    } else {
        printf("Cannot retrieve eFuse Two Point calibration values. Default calibration values will be used.\n");
    }
#else
#error "This example is configured for ESP32/ESP32S2."
#endif
}

// Tipo de tensao de referencia
static void print_char_val_type(esp_adc_cal_value_t val_type)
{
    if (val_type == ESP_ADC_CAL_VAL_EFUSE_TP) {
        printf("Characterized using Two Point Value\n");
    } else if (val_type == ESP_ADC_CAL_VAL_EFUSE_VREF) {
        printf("Characterized using eFuse Vref\n");
    } else {
        printf("Characterized using Default Vref\n");
    }
}

void LumControl(void *parameter){
    // configurando o DAC
    dac_output_enable(LED);
    dac_output_voltage(LED, 0);

    //Check if Two Point or Vref are burned into eFuse
    check_efuse();

    //Configurando o ADC1
    if (unit == ADC_UNIT_1) {
        adc1_config_width(width);
        adc1_config_channel_atten(channel, atten);
    } else {
        adc2_config_channel_atten((adc2_channel_t)channel, atten);
    }

    //Caracterizando o ADC
    adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_value_t val_type = esp_adc_cal_characterize(unit, atten, width, DEFAULT_VREF, adc_chars);
    print_char_val_type(val_type);

     //Amostragem do ADC1
    while (1) {
        uint32_t adc_reading = 0;
        //Amostragem
        int flag = 0;
        for (int i = 0; i < NO_OF_SAMPLES; i++) {
            if (unit == ADC_UNIT_1) {
                while(adc1_get_raw((adc1_channel_t)channel) == 0) {
                    flag = 10000;
                    xQueueSend(data_manager_queue, &flag, 0);
                    printf("FALHA NA LEITURA, VERIFIQUE CONEXAO DO PINO\n");
                    vTaskDelay(500 / portTICK_PERIOD_MS);
                }
                flag = 0;
                xQueueSend(data_manager_queue, &flag, 0);
                adc_reading += adc1_get_raw((adc1_channel_t)channel);
                //vTaskDelay(500 / portTICK_PERIOD_MS);
            } else {
                int raw;
                adc2_get_raw((adc2_channel_t)channel, width, &raw);
                adc_reading += raw;
            }
        }
        adc_reading /= NO_OF_SAMPLES; // efetua uma media dos valores amostrados

        //Converter valor lido em mV
        uint32_t voltage = esp_adc_cal_raw_to_voltage(adc_reading, adc_chars);
        xQueueSend(data_manager_queue, &voltage, 0);
        
        float linear = -1.0*(255.0/1299.0)*((float) voltage) + 372.0;

        int dimmer = (int) linear;

        if(voltage < 600) {
            dac_output_voltage(LED, 255);
        } else if (voltage >= 600) {
            //dimmer = -1*(255/1299)*voltage + 372;
            dac_output_voltage(LED, dimmer);
        }

        printf("Valor do dimmer %d\n", dimmer);

        int porcentagem_luminosidade = (dimmer/255)*100;
        printf("PORCENTAGEM LUMINOSIDAD%d\n", porcentagem_luminosidade);
        xQueueSend(luminosity_queue, &porcentagem_luminosidade, 0);
        printf("Raw: %d\tVoltage: %dmV\n", adc_reading, voltage);
        printf("\n");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

//Funcao principal
void app_main(void)
{
    // creating the queue
    data_manager_queue = xQueueCreate(1,sizeof(int));
    luminosity_queue = xQueueCreate(1,sizeof(int));

    //Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_LOGI(TAG, "ESP_WIFI_MODE_STA");
    wifi_init_sta();

    // ESP_ERROR_CHECK( nvs_flash_init() );
    // ESP_ERROR_CHECK(esp_netif_init());
    // ESP_ERROR_CHECK(esp_event_loop_create_default());

    /* This helper function configures Wi-Fi or Ethernet, as selected in menuconfig.
     * Read "Establishing Wi-Fi or Ethernet Connection" section in
     * examples/protocols/README.md for more information about this function.
     */
    //ESP_ERROR_CHECK(example_connect());

    xTaskCreate(&http_get_task, "http_get_task", 8192, NULL, 1, NULL);

    
    xTaskCreate(LumControl, "Sensor de luminosidade", 2048, NULL, 1, &LumControlTask);
}
