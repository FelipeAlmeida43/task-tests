/*
 * ws2812.c
 *
 *  Created on: 21 de jul. de 2024
 *      Author: FelipeAlmeida
 */
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/rmt.h"
#include "driver/rmt_types_legacy.h"
#include "driver/rmt_tx.h"
#include "ws2812.h"
// Predefined colors (Red, Green, Blue, Yellow, Cyan, Magenta, White, Off)
uint8_t colors[][3] = {
    {32, 0, 0},    // Red
    {0, 32, 0},    // Green
    {0, 0, 32},    // Blue
    {32, 32, 0},  // Yellow
    {0, 32, 32},  // Cyan
    {32, 0, 32},  // Magenta
    {32, 32, 32},// White
    {0, 0, 0}       // Off
};
void ws2812_set_color(uint8_t red, uint8_t green, uint8_t blue) {
    rmt_item32_t items[24];
    uint32_t color = ((green << 16) | (red << 8) | blue);
    // uint32_t color = ((red << 16) | (green << 8) | blue);  // Adjust the order to match GRB
    for (int i = 23; i >= 0; i--) {
        if (color & (1 << i)) {
            items[23 - i].level0 = 1;
            items[23 - i].duration0 = WS2812_T1H_CYCLES;
            items[23 - i].level1 = 0;
            items[23 - i].duration1 = WS2812_T1L_CYCLES;
        } else {
            items[23 - i].level0 = 1;
            items[23 - i].duration0 = WS2812_T0H_CYCLES;
            items[23 - i].level1 = 0;
            items[23 - i].duration1 = WS2812_T0L_CYCLES;
        }
    }
    
    rmt_write_items(RMT_TX_CHANNEL, items, 24, true);
    rmt_wait_tx_done(RMT_TX_CHANNEL, portMAX_DELAY);
}


void ws2812_init(int pin) {
    rmt_config_t config = RMT_DEFAULT_CONFIG_TX(pin, RMT_TX_CHANNEL);
    config.clk_div = 2; // Set clock divider, higher value means lower precision but more memory

    ESP_ERROR_CHECK(rmt_config(&config));
    ESP_ERROR_CHECK(rmt_driver_install(config.channel, 0, 0));
}


