#include <stdio.h>
// #include <espressif/user_interface.h>
#include <espressif/esp_wifi.h>
#include <espressif/esp_sta.h>
#include <espressif/esp_common.h>
#include <esp/uart.h>
#include <esp8266.h>
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <etstimer.h>
#include <esplibs/libmain.h>
#include <homekit/homekit.h>
#include <homekit/characteristics.h>
#include "wifi.h"


static void wifi_init() {
    struct sdk_station_config wifi_config = {
        .ssid = WIFI_SSID,
        .password = WIFI_PASSWORD,
	.bssid_set = 1,
	.bssid = {0x48, 0xf8, 0xb3, 0xd5, 0x5f, 0x46},
    };


    printf("Starting WIFI connection attempt...\n");
    sdk_wifi_set_opmode(STATION_MODE);
    sdk_wifi_station_set_config(&wifi_config);
    sdk_wifi_station_connect();
    printf("Finished WIFI connection attempt.\n");
}

const int led_gpio = 2;
const int btn_gpio = 0;
bool led_on = false;
const gpio_inttype_t int_type = GPIO_INTTYPE_EDGE_NEG;

void led_write(bool on) {
    gpio_write(led_gpio, on ? 0 : 1);
}

void led_init() {
    gpio_enable(led_gpio, GPIO_OUTPUT);
    led_write(led_on);
    printf("LED initialized\n");
}

void led_identify_task(void *_args) {
    for (int i=0; i<3; i++) {
        for (int j=0; j<2; j++) {
            led_write(true);
            vTaskDelay(100 / portTICK_PERIOD_MS);
            led_write(false);
            vTaskDelay(100 / portTICK_PERIOD_MS);
        }

        vTaskDelay(250 / portTICK_PERIOD_MS);
    }

    led_write(led_on);

    vTaskDelete(NULL);
}

void led_identify(homekit_value_t _value) {
    printf("LED identify\n");
    xTaskCreate(led_identify_task, "LED identify", 128, NULL, 2, NULL);
}

homekit_value_t led_on_get() {
    return HOMEKIT_BOOL(led_on);
}

void led_on_set(homekit_value_t value) {
    if (value.format != homekit_format_bool) {
        printf("Invalid value format: %d\n", value.format);
        return;
    }

    led_on = value.bool_value;
    led_write(led_on);
}

homekit_characteristic_t led_characteristic = HOMEKIT_CHARACTERISTIC_(
	ON, false,
	.getter=led_on_get,
	.setter=led_on_set
    );

homekit_accessory_t *accessories[] = {
    HOMEKIT_ACCESSORY(.id=1, .category=homekit_accessory_category_lightbulb, .services=(homekit_service_t*[]){
        HOMEKIT_SERVICE(ACCESSORY_INFORMATION, .characteristics=(homekit_characteristic_t*[]){
            HOMEKIT_CHARACTERISTIC(NAME, "ROD SWITCH"),
            HOMEKIT_CHARACTERISTIC(MANUFACTURER, "ROD"),
            HOMEKIT_CHARACTERISTIC(SERIAL_NUMBER, "037A2BABFA01"),
            HOMEKIT_CHARACTERISTIC(MODEL, "SWITCH"),
            HOMEKIT_CHARACTERISTIC(FIRMWARE_REVISION, "0.1"),
            HOMEKIT_CHARACTERISTIC(IDENTIFY, led_identify),
            NULL
        }),
        HOMEKIT_SERVICE(LIGHTBULB, .primary=true, .characteristics=(homekit_characteristic_t*[]){
            HOMEKIT_CHARACTERISTIC(NAME, "ROD SWITCH"),
	    &led_characteristic,
            NULL
        }),
        NULL
    }),
    NULL
};

homekit_server_config_t config = {
    .accessories = accessories,
    .password = "111-11-111"
};


ETSTimer hb_timer;

void hb_alarm(void *arg) {
	printf("%d Heartbeat\n", xTaskGetTickCount() * portTICK_PERIOD_MS );
}

void heartbeat_init(void) {
	sdk_os_timer_setfn(&hb_timer, hb_alarm, NULL);
	sdk_os_timer_arm(&hb_timer, 250, true); // id, ms, repeat
}


static QueueHandle_t tsqueue;

void gpio_intr_handler(uint8_t gpio_num);

void buttonIntTask(void *pvParameters)
{
    printf("Waiting for button press interrupt on gpio %d...\r\n", btn_gpio);
    QueueHandle_t *tsqueue = (QueueHandle_t *)pvParameters;
    gpio_set_interrupt(btn_gpio, int_type, gpio_intr_handler);

    uint32_t last = 0;
    while(1) {
        uint32_t button_ts;
        xQueueReceive(*tsqueue, &button_ts, portMAX_DELAY);
        button_ts *= portTICK_PERIOD_MS;
        if(last < button_ts-200) {
            printf("Button interrupt fired at %dms\r\n", button_ts);
            last = button_ts;
            /*btn action goes here*/
            led_on = !led_on;
            led_write(led_on);
            homekit_characteristic_notify(&led_characteristic, HOMEKIT_BOOL(led_on));
            // sdk_system_deep_sleep(30*1000*1000);
            // printf("It should go sleep\n");
        }
    }
}

void gpio_intr_handler(uint8_t gpio_num)
{
    uint32_t now = xTaskGetTickCountFromISR();
    xQueueSendToBackFromISR(tsqueue, &now, NULL);
}

void btn_init() {
    gpio_enable(btn_gpio, GPIO_INPUT);
    gpio_set_pullup(btn_gpio, true, true);
    tsqueue = xQueueCreate(2, sizeof(uint32_t));
    xTaskCreate(buttonIntTask, "buttonIntTask", 256, &tsqueue, 2, NULL);
    printf("BTN initialized\n");
}

void user_init(void) {
    uart_set_baud(0, 115200);

    heartbeat_init();
    wifi_init();
    led_init();
    homekit_server_init(&config);
    btn_init();
}
