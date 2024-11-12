#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "ws2812/ws2812.h"
#include "esp_log.h"
// Define the GPIO pin for the button (for ESP32, for example)
#define BUTTON_PIN 9
#define RGB_PIN 48
#define DEBOUNCE_TIME_MS 50  // Debounce time of 50ms
int currentColorIndex = 0;  // Start with the first color
extern uint8_t colors[8][3];
// Declare a binary semaphore handle
SemaphoreHandle_t buttonSemaphore;

// Task that waits for button press signal (triggered by semaphore)
// Task that waits for a valid button press and release
void buttonTask(void *pvParameters) {
    while (1) {
        // Wait for the semaphore to be given by the ISR
        if (xSemaphoreTake(buttonSemaphore, portMAX_DELAY)) {
            // Debounce: Wait for the button to stabilize
            vTaskDelay(pdMS_TO_TICKS(DEBOUNCE_TIME_MS));

            // Check if the button is still pressed (active low)
            if (gpio_get_level(BUTTON_PIN) == 0) {
                // Wait until the button is released (goes back to high)
                while (gpio_get_level(BUTTON_PIN) == 0) {
                    vTaskDelay(pdMS_TO_TICKS(10));  // Poll every 10ms
                }

                // Now the button is released, process the key event
                currentColorIndex = (currentColorIndex + 1) % (sizeof(colors) / sizeof(colors[0]));
                uint8_t red = colors[currentColorIndex][0];
                uint8_t green = colors[currentColorIndex][1];
                uint8_t blue = colors[currentColorIndex][2];

                // Set the new color
                ESP_LOGI(__func__,"Color %d: Red = %d, Green = %d, Blue = %d\n",
                currentColorIndex, colors[currentColorIndex][0], colors[currentColorIndex][1], colors[currentColorIndex][2]);
                printf("COLOR idx %d \r\n",currentColorIndex);
                ws2812_set_color(red, green, blue);
                printf("Button Released! Task running...\n");
            }
        }
    }
}

// Interrupt Service Routine (ISR) for the button press
void IRAM_ATTR buttonISR(void *arg) {
    // "Give" the semaphore to unblock the task
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(buttonSemaphore, &xHigherPriorityTaskWoken);

    // If giving the semaphore caused a higher priority task to unblock, request a context switch
    if (xHigherPriorityTaskWoken == pdTRUE) {
        portYIELD_FROM_ISR();
    }
}

void app_main(void) {
    // Create the binary semaphore
    buttonSemaphore = xSemaphoreCreateBinary();

    // Check if the semaphore was created successfully
    if (buttonSemaphore == NULL) {
        printf("Failed to create semaphore\n");
        return;
    }

    // Configure the GPIO for the button
    esp_rom_gpio_pad_select_gpio(BUTTON_PIN);
    gpio_set_direction(BUTTON_PIN, GPIO_MODE_INPUT);
    gpio_set_intr_type(BUTTON_PIN, GPIO_INTR_NEGEDGE); // Interrupt on falling edge (button press)

    // Install the ISR for the button
    gpio_install_isr_service(0);
    gpio_isr_handler_add(BUTTON_PIN, buttonISR, NULL);

    // Create the task that will respond to button presses
    xTaskCreate(buttonTask, "Button Task", 2048, NULL, 10, NULL);
    ws2812_init(RGB_PIN);
	ws2812_set_color(0, 0, 0);
    // The FreeRTOS scheduler will now run the tasks
}
