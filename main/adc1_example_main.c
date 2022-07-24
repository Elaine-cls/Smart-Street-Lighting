/* ADC1 Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"

//libs do ADC
#include "driver/adc.h"
#include "esp_adc_cal.h"

//lib do DAC
#include "driver/dac.h"

#define LED DAC_CHANNEL_1   //GPIO25 fará a leitura

#define DEFAULT_VREF    1100        //Tensao de referencia
#define NO_OF_SAMPLES   64          //taxa de amostragem

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

//Funcao principal
void app_main(void)
{
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
        for (int i = 0; i < NO_OF_SAMPLES; i++) {
            if (unit == ADC_UNIT_1) {
                // while(adc1_get_raw((adc1_channel_t)channel) == 0) {
                //     printf("FALHA NA LEITURA, VERIFIQUE CONEXAO DO PINO\n");
                //     vTaskDelay(1000 / portTICK_PERIOD_MS);
                // }
                adc_reading += adc1_get_raw((adc1_channel_t)channel);
            } else {
                int raw;
                adc2_get_raw((adc2_channel_t)channel, width, &raw);
                adc_reading += raw;
            }
        }
        adc_reading /= NO_OF_SAMPLES; // efetua uma media dos valores amostrados

        //Converter valor lido em mV
        uint32_t voltage = esp_adc_cal_raw_to_voltage(adc_reading, adc_chars);

        int pin = 0;

        int linear = (int) -1*(255/1299)*(esp_adc_cal_raw_to_voltage(adc_reading, adc_chars));
        uint8_t dimmer = linear + 372;


        if(voltage <= 600) {
            dac_output_voltage(LED, 255);
        } else {
            //dimmer = -1*(255/1299)*voltage + 372;
            dac_output_voltage(LED, dimmer_ok);
        }

        printf("Valor do dimmer %d\n", dimmer_ok);
        printf("Raw: %d\tVoltage: %dmV\n", adc_reading, voltage);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
