/* Starting point. Will eventually include
 * * blinking LED (done)
 * * WiFi support - associate with AP (done)
 * * MQTT support  - publish (added, not working)
 * * NTP support - synchronize time
 *
 * Some code shamelessly copied from some examples
 * * espidf-blink example.
 * * station_example_main.c from getting_started/station
 * */

extern "C"
{
#include <driver/gpio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/event_groups.h>
#include <freertos/task.h>
#include <esp_system.h>
#include <esp_event.h>
#include <esp_log.h>
#include <nvs_flash.h>
}

#include "wifi.h"
#include "mqtt.h"
#include "my_sntp.h"

static const bool chatty = false;

// Timer class/object, encapsulates an action and the information
// required to schedule execution. Timer:next is either the value
// returned by time(0) in seconds or some arbitrary timing mark 
// maintained by the program.

class Timer {
    protected:
        time_t  next;       // next time(0) the action should be executed
        time_t  interval;   // periodic interval to fire the action
        Timer() {};         // no public default ctor
    public:
        time_t get_next(void) { return next; }
        virtual bool do_action(void);  // return true to reschedule
        Timer(bool (*ptr)(void)):next(0), interval(0) {};
};


class PublishState:Timer {
    public:
        bool do_action(void);
};

bool PublishState::do_action(void) {
    return true;
}

PublishState pubState();

static Timer actions[1] = {pubState}; // actions to publish state and 'data' content


// LED ========================================

static const gpio_num_t blink_led = GPIO_NUM_2;

void setup_LED(gpio_num_t gpio)
{
    gpio_pad_select_gpio(gpio);
    /* Set the GPIO as a push/pull output */
    gpio_set_direction(gpio, GPIO_MODE_OUTPUT);
}

void blink_led_task(void *param)
{
    while (1)
    {
        /* Blink off (output low) */
        if (chatty)
            printf("Turning off the LED\n");
        gpio_set_level(blink_led, 0);
        vTaskDelay(990 / portTICK_PERIOD_MS);
        /* Blink on (output high) */
        if (chatty)
            printf("Turning on the LED\n");
        gpio_set_level(blink_led, 1);
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

// main =======================================

static const char *TAG = "esp32 main";
/* Variable holding number of times ESP32 restarted since first boot.
 * It is placed into RTC memory using RTC_DATA_ATTR and
 * maintains its value when ESP32 wakes from deep sleep.
 *
 * Assumes that WiFi connection is already established.
 */
RTC_DATA_ATTR static int boot_count = 0;

extern "C" void app_main()
{
    boot_count++;
    setup_LED(blink_led);

    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Note: 512 is insufficient stack for blink_led_task
    xTaskCreate(&blink_led_task, "blink_led_task", 1024, NULL, 5, NULL);

    // start WiFi
    ESP_LOGI(TAG, "ESP_WIFI_MODE_STA");
    init_wifi();
    mqtt_app_start();
    time_t t = init_sntp();
    ESP_LOGI(TAG, "init_sntp(): %ld", t);

    // Following loop is superfluous. App continues to execute
    // even if app_main() exits
    int loop_counter = 0;
    while (1)
    {
        // Blink off (output low)
        if (chatty)
            printf("looping main\n");
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        if ((++loop_counter % 10) == 0)
        {
            static const int buf_len = 100;
            char uptime_buff[buf_len];
            snprintf(uptime_buff, buf_len, "uptime %d, timestamp %ld, bootcount %d, heap %d, %d, %d, %d, %d, %d",
                    loop_counter, time(0), boot_count,
                    heap_caps_get_free_size(MALLOC_CAP_EXEC),
                    heap_caps_get_free_size(MALLOC_CAP_32BIT),
                    heap_caps_get_free_size(MALLOC_CAP_8BIT),
                    heap_caps_get_free_size(MALLOC_CAP_DMA),
                    heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
                    heap_caps_get_free_size(MALLOC_CAP_INTERNAL));
            mqtt_publish(NULL, uptime_buff);
        }
    }
}