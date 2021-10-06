#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "esp_netif.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"

#define TAG         "rebound"
#define AP_SSID     "esp32"
#define AP_PASS     "passw0rd"
#define AP_CHAN     1
#define AP_MAX_CONN 1

static void wifi_event_handler(void* arg, esp_event_base_t event_base,
		int32_t event_id, void* event_data)
{
	if (event_id == WIFI_EVENT_AP_STACONNECTED) {
		wifi_event_ap_staconnected_t *event = (wifi_event_ap_staconnected_t *)event_data;
		ESP_LOGI(TAG, "station "MACSTR" join, AID=%d",
				MAC2STR(event->mac), event->aid);
	} else if (event_id == WIFI_EVENT_AP_STADISCONNECTED) {
		wifi_event_ap_stadisconnected_t *event = (wifi_event_ap_stadisconnected_t *)event_data;
		ESP_LOGI(TAG, "station "MACSTR" leave, AID=%d",
				MAC2STR(event->mac), event->aid);
	}
}

void wifi_init_ap(void)
{
	ESP_ERROR_CHECK(esp_netif_init());
	ESP_ERROR_CHECK(esp_event_loop_create_default());
	esp_netif_create_default_wifi_ap();

	wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
	ESP_ERROR_CHECK(esp_wifi_init(&cfg));

	ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
				ESP_EVENT_ANY_ID,
				&wifi_event_handler,
				NULL,
				NULL));

	wifi_config_t wifi_config = {
		.ap = {
			.ssid = AP_SSID,
			.ssid_len = strlen(AP_SSID),
			.channel = AP_CHAN,
			.password = AP_PASS,
			.max_connection = AP_MAX_CONN,
			.authmode = WIFI_AUTH_WPA_WPA2_PSK
		},
	};
	if (strlen((const char *)wifi_config.ap.password) == 0) {
		wifi_config.ap.authmode = WIFI_AUTH_OPEN;
	}

	ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
	ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
	ESP_ERROR_CHECK(esp_wifi_set_country_code("GB", true));
	ESP_ERROR_CHECK(esp_wifi_start());

	ESP_LOGI(TAG, "wifi_init_ap done.");
}

void app_main(void)
{
	printf("Hello, world!");
	fflush(stdout);

	esp_err_t ret = nvs_flash_init();
	if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
		ESP_ERROR_CHECK(nvs_flash_erase());
		ret = nvs_flash_init();
	}
	ESP_ERROR_CHECK(ret);

	ESP_LOGI(TAG, "ESP_WIFI_MODE_AP");
	wifi_init_ap();

	while (1) {
		vTaskDelay(portMAX_DELAY);
	}
}
