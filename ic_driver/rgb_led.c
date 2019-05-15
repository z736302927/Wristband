/*
 * rgb_led.c
 *
 *  Created on: 2019Äê5ÔÂ10ÈÕ
 *      Author: Administrator
 */

/* LEDC (LED Controller) fade example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/ledc.h"
#include "esp_err.h"
#include "sdkconfig.h"

/*
 * About this example
 *
 * 1. Start with initializing LEDC module:
 *    a. Set the timer of LEDC first, this determines the frequency
 *       and resolution of PWM.
 *    b. Then set the LEDC channel you want to use,
 *       and bind with one of the timers.
 *
 * 2. You need first to install a default fade function,
 *    then you can use fade APIs.
 *
 * 3. You can also set a target duty directly without fading.
 *
 * 4. This example uses GPIO18/19/4/5 as LEDC output,
 *    and it will change the duty repeatedly.
 *
 * 5. GPIO18/19 are from high speed channel group.
 *    GPIO4/5 are from low speed channel group.
 *
 */
#define LEDC_HS_TIMER          	LEDC_TIMER_0
#define LEDC_SPEED_MODE        	LEDC_HIGH_SPEED_MODE
#define LEDC_RED_GPIO       	(26)
#define LEDC_RED_CHANNEL    	LEDC_CHANNEL_0
#define LEDC_GREEN_GPIO       	(14)
#define LEDC_GREEN_CHANNEL    	LEDC_CHANNEL_1
#define LEDC_BLUE_GPIO       	(12)
#define LEDC_BLUE_CHANNEL    	LEDC_CHANNEL_1

#define LEDC_TEST_CH_NUM       	(3)
#define LEDC_TEST_DUTY         	(255)
#define LEDC_MAX_DUTY         	(255)
#define LEDC_TEST_FADE_TIME    	(1000)

#define LED_R GPIO_NUM_26
#define LED_G GPIO_NUM_14
#define LED_B GPIO_NUM_12

void blink_task(void *pvParameter)
{
    /* Configure the IOMUX register for pad BLINK_GPIO (some pads are
       muxed to GPIO on reset already, but some default to other
       functions and need to be switched to GPIO. Consult the
       Technical Reference for a list of pads and their default
       functions.)
    */
    gpio_pad_select_gpio(LED_R);
    gpio_pad_select_gpio(LED_G);
    gpio_pad_select_gpio(LED_B);
    /* Set the GPIO as a push/pull output */
    gpio_set_direction(LED_R, GPIO_MODE_OUTPUT);
    gpio_set_direction(LED_G, GPIO_MODE_OUTPUT);
    gpio_set_direction(LED_B, GPIO_MODE_OUTPUT);
    while(1) {
        gpio_set_level(LED_R, 0);
        gpio_set_level(LED_G, 1);
        gpio_set_level(LED_B, 1);
        vTaskDelay(1000 / portTICK_PERIOD_MS);

        gpio_set_level(LED_R, 1);
        gpio_set_level(LED_G, 0);
        gpio_set_level(LED_B, 1);
        vTaskDelay(1000 / portTICK_PERIOD_MS);

        gpio_set_level(LED_R, 1);
        gpio_set_level(LED_G, 1);
        gpio_set_level(LED_B, 0);
        vTaskDelay(1000 / portTICK_PERIOD_MS);

        gpio_set_level(LED_R, 1);
        gpio_set_level(LED_G, 1);
        gpio_set_level(LED_B, 1);
        vTaskDelay(1000 / portTICK_PERIOD_MS);

        gpio_set_level(LED_R, 0);
        gpio_set_level(LED_G, 0);
        gpio_set_level(LED_B, 0);
        vTaskDelay(1000 / portTICK_PERIOD_MS);

        gpio_set_level(LED_R, 0);
        gpio_set_level(LED_G, 0);
        gpio_set_level(LED_B, 1);
        vTaskDelay(1000 / portTICK_PERIOD_MS);

        gpio_set_level(LED_R, 0);
        gpio_set_level(LED_G, 1);
        gpio_set_level(LED_B, 0);
        vTaskDelay(1000 / portTICK_PERIOD_MS);

        gpio_set_level(LED_R, 1);
        gpio_set_level(LED_G, 0);
        gpio_set_level(LED_B, 0);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}
void RGB_LedInit(void)
{
    int ch;

    /*
     * Prepare and set configuration of timers
     * that will be used by LED Controller
     */
    ledc_timer_config_t ledc_timer = {
        .duty_resolution = LEDC_TIMER_8_BIT, // resolution of PWM duty
        .freq_hz = 5000,                      // frequency of PWM signal
        .speed_mode = LEDC_SPEED_MODE,           // timer mode
        .timer_num = LEDC_HS_TIMER            // timer index
    };
    // Set configuration of timer0 for high speed channels
    ledc_timer_config(&ledc_timer);

    /*
     * Prepare individual configuration
     * for each channel of LED Controller
     * by selecting:
     * - controller's channel number
     * - output duty cycle, set initially to 0
     * - GPIO number where LED is connected to
     * - speed mode, either high or low
     * - timer servicing selected channel
     *   Note: if different channels use one timer,
     *         then frequency and bit_num of these channels
     *         will be the same
     */
    ledc_channel_config_t ledc_channel[LEDC_TEST_CH_NUM] = {
        {
            .channel    = LEDC_RED_CHANNEL,
            .duty       = 0,
            .gpio_num   = LEDC_RED_GPIO,
            .speed_mode = LEDC_SPEED_MODE,
            .hpoint     = 0,
            .timer_sel  = LEDC_HS_TIMER
        },
        {
            .channel    = LEDC_GREEN_CHANNEL,
            .duty       = 0,
            .gpio_num   = LEDC_GREEN_GPIO,
            .speed_mode = LEDC_SPEED_MODE,
            .hpoint     = 0,
            .timer_sel  = LEDC_HS_TIMER
        },
        {
            .channel    = LEDC_BLUE_CHANNEL,
            .duty       = 0,
            .gpio_num   = LEDC_BLUE_GPIO,
            .speed_mode = LEDC_SPEED_MODE,
            .hpoint     = 0,
            .timer_sel  = LEDC_HS_TIMER
        },
    };

    // Set LED Controller with previously prepared configuration
    for (ch = 0; ch < LEDC_TEST_CH_NUM; ch++) {
        ledc_channel_config(&ledc_channel[ch]);
    }

    // Initialize fade service.
    ledc_fade_func_install(0);
}

void RGB_SetColor(RGB_LedTypeDef RGB_Led)
{
	ledc_set_duty(LEDC_SPEED_MODE, LEDC_RED_CHANNEL, LEDC_MAX_DUTY - 0);
	ledc_update_duty(LEDC_SPEED_MODE, LEDC_RED_CHANNEL);
	ledc_set_duty(LEDC_SPEED_MODE, LEDC_GREEN_CHANNEL, LEDC_MAX_DUTY - 255);
	ledc_update_duty(LEDC_SPEED_MODE, LEDC_GREEN_CHANNEL);
	ledc_set_duty(LEDC_SPEED_MODE, LEDC_BLUE_CHANNEL, LEDC_MAX_DUTY - 0);
	ledc_update_duty(LEDC_SPEED_MODE, LEDC_BLUE_CHANNEL);
}

void RGB_ColorTest(void)
{
	RGB_LedTypeDef RGB_Led;

//	ledc_set_fade_with_time(LEDC_SPEED_MODE,LEDC_RED_CHANNEL, LEDC_TEST_DUTY, LEDC_TEST_FADE_TIME);
//	ledc_fade_start(LEDC_SPEED_MODE,LEDC_RED_CHANNEL, LEDC_FADE_NO_WAIT);
//    vTaskDelay(LEDC_TEST_FADE_TIME / portTICK_PERIOD_MS);
//	ledc_set_fade_with_time(LEDC_SPEED_MODE,LEDC_RED_CHANNEL, 0, LEDC_TEST_FADE_TIME);
//	ledc_fade_start(LEDC_SPEED_MODE,LEDC_RED_CHANNEL, LEDC_FADE_NO_WAIT);
//    vTaskDelay(LEDC_TEST_FADE_TIME / portTICK_PERIOD_MS);
//	ledc_set_fade_with_time(LEDC_SPEED_MODE,LEDC_GREEN_CHANNEL, LEDC_TEST_DUTY, LEDC_TEST_FADE_TIME);
//	ledc_fade_start(LEDC_SPEED_MODE,LEDC_GREEN_CHANNEL, LEDC_FADE_NO_WAIT);
//    vTaskDelay(LEDC_TEST_FADE_TIME / portTICK_PERIOD_MS);
//	ledc_set_fade_with_time(LEDC_SPEED_MODE,LEDC_GREEN_CHANNEL, 0, LEDC_TEST_FADE_TIME);
//	ledc_fade_start(LEDC_SPEED_MODE,LEDC_GREEN_CHANNEL, LEDC_FADE_NO_WAIT);
//    vTaskDelay(LEDC_TEST_FADE_TIME / portTICK_PERIOD_MS);
//	ledc_set_fade_with_time(LEDC_SPEED_MODE,LEDC_BLUE_CHANNEL, LEDC_TEST_DUTY, LEDC_TEST_FADE_TIME);
//	ledc_fade_start(LEDC_SPEED_MODE,LEDC_RED_CHANNEL, LEDC_FADE_NO_WAIT);
//    vTaskDelay(LEDC_TEST_FADE_TIME / portTICK_PERIOD_MS);
//	ledc_set_fade_with_time(LEDC_SPEED_MODE,LEDC_BLUE_CHANNEL, 0, LEDC_TEST_FADE_TIME);
//	ledc_fade_start(LEDC_SPEED_MODE,LEDC_RED_CHANNEL, LEDC_FADE_NO_WAIT);
//    vTaskDelay(LEDC_TEST_FADE_TIME / portTICK_PERIOD_MS);

//    RGB_Led.R = (uint8_t)(COLOR_WHITE >> 16);
//    RGB_Led.G = (uint8_t)(COLOR_WHITE >> 8);
//    RGB_Led.B = (uint8_t)(COLOR_WHITE >> 0);
//    RGB_SetColor(RGB_Led);
//    vTaskDelay(1000 / portTICK_PERIOD_MS);
//    RGB_Led.R = (uint8_t)(COLOR_BLACK >> 16);
//    RGB_Led.G = (uint8_t)(COLOR_BLACK >> 8);
//    RGB_Led.B = (uint8_t)(COLOR_BLACK >> 0);
//    RGB_SetColor(RGB_Led);
//    vTaskDelay(1000 / portTICK_PERIOD_MS);
    RGB_Led.R = (uint8_t)(COLOR_RED >> 16);
    RGB_Led.G = (uint8_t)(COLOR_RED >> 8);
    RGB_Led.B = (uint8_t)(COLOR_RED >> 0);
    RGB_SetColor(RGB_Led);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
//    RGB_Led.R = (uint8_t)(COLOR_ORANGE >> 16);
//    RGB_Led.G = (uint8_t)(COLOR_ORANGE >> 8);
//    RGB_Led.B = (uint8_t)(COLOR_ORANGE >> 0);
//    RGB_SetColor(RGB_Led);
//    vTaskDelay(1000 / portTICK_PERIOD_MS);
//    RGB_Led.R = (uint8_t)(COLOR_YELLOW >> 16);
//    RGB_Led.G = (uint8_t)(COLOR_YELLOW >> 8);
//    RGB_Led.B = (uint8_t)(COLOR_YELLOW >> 0);
//    RGB_SetColor(RGB_Led);
//    vTaskDelay(1000 / portTICK_PERIOD_MS);
    RGB_Led.R = (uint8_t)(COLOR_GREEN >> 16);
    RGB_Led.G = (uint8_t)(COLOR_GREEN >> 8);
    RGB_Led.B = (uint8_t)(COLOR_GREEN >> 0);
    RGB_SetColor(RGB_Led);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
//    RGB_Led.R = (uint8_t)(COLOR_CYAN >> 16);
//    RGB_Led.G = (uint8_t)(COLOR_CYAN >> 8);
//    RGB_Led.B = (uint8_t)(COLOR_CYAN >> 0);
//    RGB_SetColor(RGB_Led);
//    vTaskDelay(1000 / portTICK_PERIOD_MS);
    RGB_Led.R = (uint8_t)(COLOR_BLUE >> 16);
    RGB_Led.G = (uint8_t)(COLOR_BLUE >> 8);
    RGB_Led.B = (uint8_t)(COLOR_BLUE >> 0);
    RGB_SetColor(RGB_Led);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
//    RGB_Led.R = (uint8_t)(COLOR_PURPLE >> 16);
//    RGB_Led.G = (uint8_t)(COLOR_PURPLE >> 8);
//    RGB_Led.B = (uint8_t)(COLOR_PURPLE >> 0);
//    RGB_SetColor(RGB_Led);
//    vTaskDelay(1000 / portTICK_PERIOD_MS);
}




