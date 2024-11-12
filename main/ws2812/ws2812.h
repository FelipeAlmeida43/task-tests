/*
 * ws2812.h
 *
 *  Created on: 21 de jul. de 2024
 *      Author: FelipeAlmeida
 */

#ifndef MAIN_WS2812_WS2812_H_
#define MAIN_WS2812_WS2812_H_
#include <stdint.h>
void ws2812_set_color(uint8_t red, uint8_t green, uint8_t blue);
void ws2812_init(int pin);
#define WS2812_FREQ_HZ 800000
#define WS2812_T0H_NS 350
#define WS2812_T0L_NS 800
#define WS2812_T1H_NS 700
#define WS2812_T1L_NS 600
#define WS2812_RESET_US 50
#define RMT_CLK_DIV 2
#define WS2812_T0H_CYCLES (WS2812_T0H_NS / (1000 / 80) / RMT_CLK_DIV)
#define WS2812_T0L_CYCLES (WS2812_T0L_NS / (1000 / 80) / RMT_CLK_DIV)
#define WS2812_T1H_CYCLES (WS2812_T1H_NS / (1000 / 80) / RMT_CLK_DIV)
#define WS2812_T1L_CYCLES (WS2812_T1L_NS / (1000 / 80) / RMT_CLK_DIV)
#define RMT_TX_CHANNEL RMT_CHANNEL_0
typedef enum{
	RED =0,
	BLUE,
	GREEN,
	YELLOW
}LedColor;

#endif /* MAIN_WS2812_WS2812_H_ */
