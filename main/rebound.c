#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_netif.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "lwip/err.h"
#include "lwip/sys.h"
#include "lwip/sockets.h"
#include <lwip/netdb.h>

#define TAG         "rebound"
#define AP_SSID     "esp32"
#define AP_PASS     "passw0rd"
#define AP_CHAN     1
#define AP_MAX_CONN 1

#define TCP_PORT               9000
#define TCP_KEEPALIVE_IDLE     5
#define TCP_KEEPALIVE_INTERVAL 5
#define TCP_KEEPALIVE_COUNT    3

#define LED_PIN_R  16
#define LED_PIN_G  17
#define LED_PIN_B  18
#define LED_PIN_B2 19

#define UART_NUM      UART_NUM_1
#define UART_BUF_SIZE 1024
#define UART_TX_PIN   2
#define UART_RX_PIN   4
static QueueHandle_t event_queue;

struct connection {
	struct sockaddr_storage addr;
	socklen_t addr_len;
	int fd;
};

typedef enum {
	TCP_NEW_CONN = UART_EVENT_MAX + 1,
} tcp_event_type_t;

typedef struct {
	tcp_event_type_t type;
	void *data;
} tcp_event_t;

typedef union {
	uart_event_t uart;
	tcp_event_t tcp;
} event_t;
// We're piggy-backing off the UART event queue, so we can't control the event size
_Static_assert(sizeof(tcp_event_t) <= sizeof(uart_event_t), "TCP events can't be larger than UART events");

static int tcp_server_init(void)
{
	int err;
	struct sockaddr_storage dest_addr;

	struct sockaddr_in *dest_addr_ip4 = (struct sockaddr_in *)&dest_addr;
	dest_addr_ip4->sin_addr.s_addr = htonl(INADDR_ANY);
	dest_addr_ip4->sin_family = AF_INET;
	dest_addr_ip4->sin_port = htons(TCP_PORT);

	int listen_sock = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
	if (listen_sock < 0) {
		ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
		return -1;
	}

	int flags = fcntl(listen_sock, F_GETFL);
	if (flags < 0) {
		ESP_LOGE(TAG, "Unable to get flags: errno %d", errno);
		goto err;
	}

	err = fcntl(listen_sock, F_SETFL, flags | O_NONBLOCK);
	if (err != 0) {
		ESP_LOGE(TAG, "Unable to set nonblock: errno %d", errno);
		goto err;
	}

	int opt = 1;
	setsockopt(listen_sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

	ESP_LOGI(TAG, "Socket created");

	err = bind(listen_sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
	if (err != 0) {
		ESP_LOGE(TAG, "Socket unable to bind: errno %d", errno);
		ESP_LOGE(TAG, "IPPROTO: %d", AF_INET);
		goto err;
	}
	ESP_LOGI(TAG, "Socket bound, port %d", TCP_PORT);

	// TODO: Is backlog = 1 OK?
	err = listen(listen_sock, 1);
	if (err != 0) {
		ESP_LOGE(TAG, "Error occurred during listen: errno %d", errno);
		goto err;
	}

	return listen_sock;

err:
	close(listen_sock);
	return -1;
}

static void wifi_event_handler(void* arg, esp_event_base_t event_base,
		int32_t event_id, void* event_data)
{
	if (event_id == WIFI_EVENT_AP_STACONNECTED) {
		wifi_event_ap_staconnected_t *event = (wifi_event_ap_staconnected_t *)event_data;
		ESP_LOGI(TAG, "station "MACSTR" join, AID=%d",
				MAC2STR(event->mac), event->aid);
		gpio_set_level(LED_PIN_B2, 0);
	} else if (event_id == WIFI_EVENT_AP_STADISCONNECTED) {
		wifi_event_ap_stadisconnected_t *event = (wifi_event_ap_stadisconnected_t *)event_data;
		ESP_LOGI(TAG, "station "MACSTR" leave, AID=%d",
				MAC2STR(event->mac), event->aid);
		gpio_set_level(LED_PIN_B2, 1);
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

static void free_connection(struct connection *conn)
{
	shutdown(conn->fd, SHUT_RDWR);
	close(conn->fd);
	free(conn);
}

static void handler_task(void *pvParameters)
{
	bool led = 0;
	event_t event;
	struct connection *current_conn = NULL;
	static char data[1024];
	for(;;) {
		gpio_set_level(LED_PIN_R, led);
		led = !led;

		if (!(xQueueReceive(event_queue, (void * )&event, 30 / portTICK_PERIOD_MS))) {
			if (current_conn != NULL) {
				// Pump socket
				int len = recv(current_conn->fd, data, sizeof(data), 0);
				if (len > 0) {
					ESP_LOGI(TAG, "%d from socket", len);
					ESP_LOGI(TAG, "[UART TX DATA]:");
					ESP_LOG_BUFFER_HEXDUMP(TAG, data, len, ESP_LOG_INFO);
					gpio_set_level(LED_PIN_B, 0);
					uart_write_bytes(UART_NUM, (const char*)data, len);
					gpio_set_level(LED_PIN_B, 1);
				} else if (len == 0) {
					ESP_LOGI(TAG, "Socket closed");
					free_connection(current_conn);
					current_conn = NULL;
					gpio_set_level(LED_PIN_G, 1);
				} else if (errno != EAGAIN || errno != EWOULDBLOCK) {
					ESP_LOGE(TAG, "Socket error: %d", errno);
					free_connection(current_conn);
					current_conn = NULL;
					gpio_set_level(LED_PIN_G, 1);
				}
			}

			continue;
		}

		if (event.uart.type <= UART_EVENT_MAX) {
			ESP_LOGI(TAG, "uart[%d] event:", UART_NUM);
			switch(event.uart.type) {
			case UART_DATA:
				ESP_LOGI(TAG, "[UART DATA]: %d", event.uart.size);
				gpio_set_level(LED_PIN_B, 0);
				while (event.uart.size > 0) {
					int size = event.uart.size > sizeof(data) ? sizeof(data) : event.uart.size;
					uart_read_bytes(UART_NUM, data, size, portMAX_DELAY);
					ESP_LOGI(TAG, "[UART RX DATA]:");
					ESP_LOG_BUFFER_HEXDUMP(TAG, data, size, ESP_LOG_INFO);
					if (current_conn != NULL) {
						int to_write = size;
						while (to_write > 0) {
							int written = send(current_conn->fd, data + (size - to_write), to_write, 0);
							if (written < 0) {
								ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
								// TODO: Close conn here?
							}
							to_write -= written;
						}
					}

					event.uart.size -= size;
				}
				gpio_set_level(LED_PIN_B, 1);
				break;
			case UART_FIFO_OVF:
				ESP_LOGI(TAG, "hw fifo overflow");
				uart_flush_input(UART_NUM);
				// TODO: We can't just reset if there's more than UART
				// events in the queue
				xQueueReset(event_queue);
				free_connection(current_conn);
				current_conn = NULL;
				gpio_set_level(LED_PIN_G, 1);
				break;
			case UART_BUFFER_FULL:
				ESP_LOGI(TAG, "ring buffer full");
				uart_flush_input(UART_NUM);
				// TODO: We can't just reset if there's more than UART
				// events in the queue
				xQueueReset(event_queue);
				free_connection(current_conn);
				current_conn = NULL;
				gpio_set_level(LED_PIN_G, 1);
				break;
			case UART_BREAK:
				ESP_LOGI(TAG, "uart rx break");
				break;
			case UART_PARITY_ERR:
				ESP_LOGI(TAG, "uart parity error");
				break;
			case UART_FRAME_ERR:
				ESP_LOGI(TAG, "uart frame error");
				break;
			default:
				ESP_LOGI(TAG, "uart event type: %d", event.uart.type);
				break;
			}
		} else {
			switch (event.tcp.type) {
			case TCP_NEW_CONN:
				ESP_LOGI(TAG, "received connection");
				if (current_conn != NULL) {
					free_connection(current_conn);
				}
				current_conn = event.tcp.data;
				gpio_set_level(LED_PIN_G, 0);
				break;
			default:
				ESP_LOGI(TAG, "tcp event type: %d", event.tcp.type);
			}
		}

		if (current_conn != NULL) {
			// Pump socket
			int len = recv(current_conn->fd, data, sizeof(data), 0);
			if (len > 0) {
				ESP_LOGI(TAG, "%d from socket", len);
				ESP_LOGI(TAG, "[UART TX DATA]:");
				ESP_LOG_BUFFER_HEXDUMP(TAG, data, len, ESP_LOG_INFO);
				gpio_set_level(LED_PIN_B, 0);
				uart_write_bytes(UART_NUM, (const char*)data, len);
				gpio_set_level(LED_PIN_B, 1);
			} else if (len == 0) {
				ESP_LOGI(TAG, "Socket closed");
				free_connection(current_conn);
				current_conn = NULL;
				gpio_set_level(LED_PIN_G, 1);
			} else if (errno != EAGAIN || errno != EWOULDBLOCK) {
				ESP_LOGE(TAG, "Socket error: %d", errno);
				free_connection(current_conn);
				current_conn = NULL;
				gpio_set_level(LED_PIN_G, 1);
			}
		}
	}
	vTaskDelete(NULL);
}

void uart_init(void)
{
	uart_config_t uart_config = {
		.baud_rate = 115200,
		.data_bits = UART_DATA_8_BITS,
		.parity = UART_PARITY_DISABLE,
		.stop_bits = UART_STOP_BITS_1,
		.flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
		.source_clk = UART_SCLK_APB,
	};

	uart_driver_install(UART_NUM, UART_BUF_SIZE * 2, UART_BUF_SIZE * 2, 20, &event_queue, 0);
	uart_param_config(UART_NUM, &uart_config);

	uart_set_pin(UART_NUM, UART_TX_PIN, UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}

void app_main(void)
{
	esp_log_level_set(TAG, ESP_LOG_WARN);
	printf("Hello, world!\n");
	fflush(stdout);

	gpio_config_t gpio_conf = {
		.intr_type = GPIO_INTR_DISABLE,
		.mode = GPIO_MODE_OUTPUT,
		.pin_bit_mask = (1 << LED_PIN_R) | (1 << LED_PIN_G) | (1 << LED_PIN_B) | (1 << LED_PIN_B2),
		.pull_down_en = 0,
		.pull_up_en = 0,
	};
	gpio_config(&gpio_conf);

	gpio_set_level(LED_PIN_R, 1);
	gpio_set_level(LED_PIN_G, 1);
	gpio_set_level(LED_PIN_B, 1);
	gpio_set_level(LED_PIN_B2, 1);

	esp_err_t ret = nvs_flash_init();
	if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
		ESP_ERROR_CHECK(nvs_flash_erase());
		ret = nvs_flash_init();
	}
	ESP_ERROR_CHECK(ret);

	uart_init();
	xTaskCreate(handler_task, "handler_task", 2048, NULL, 12, NULL);

	ESP_LOGI(TAG, "ESP_WIFI_MODE_AP");
	wifi_init_ap();

	struct connection *conn = NULL;
	int listen_fd = tcp_server_init();
	if (listen_fd < 0) {
		ESP_LOGE(TAG, "tcp init failed.");
	}

	while (1) {
		if (conn == NULL) {
			conn = malloc(sizeof(*conn));
			*conn = (struct connection){ 0 };
			conn->addr_len = sizeof(conn->addr);
		}

		conn->fd = accept(listen_fd, (struct sockaddr *)&conn->addr, &conn->addr_len);
		if (conn->fd >= 0) {
			// Handle connection
			char addr_str[128];
			int keepAlive = 1;
			int keepIdle = TCP_KEEPALIVE_IDLE;
			int keepInterval = TCP_KEEPALIVE_INTERVAL;
			int keepCount = TCP_KEEPALIVE_COUNT;

			inet_ntoa_r(((struct sockaddr_in *)&conn->addr)->sin_addr, addr_str, sizeof(addr_str) - 1);
			ESP_LOGI(TAG, "Socket accepted ip address: %s", addr_str);

			setsockopt(conn->fd, SOL_SOCKET, SO_KEEPALIVE, &keepAlive, sizeof(int));
			setsockopt(conn->fd, IPPROTO_TCP, TCP_KEEPIDLE, &keepIdle, sizeof(int));
			setsockopt(conn->fd, IPPROTO_TCP, TCP_KEEPINTVL, &keepInterval, sizeof(int));
			setsockopt(conn->fd, IPPROTO_TCP, TCP_KEEPCNT, &keepCount, sizeof(int));

			int flags = fcntl(conn->fd, F_GETFL);
			if (flags < 0) {
				ESP_LOGE(TAG, "Unable to get flags: errno %d", errno);
			}

			int err = fcntl(conn->fd, F_SETFL, flags | O_NONBLOCK);
			if (err != 0) {
				ESP_LOGE(TAG, "Unable to set nonblock: errno %d", errno);
			}

			event_t event = {
				.tcp = {
					.type = TCP_NEW_CONN,
					.data = conn,
				},
			};
			// Pass off to handler thread.
			xQueueSend(event_queue, &event, portMAX_DELAY);
			conn = NULL;
		} else if (errno != EAGAIN && errno != EWOULDBLOCK) {
			ESP_LOGE(TAG, "Accept failed: errno %d", errno);
		}

		vTaskDelay(100 / portTICK_PERIOD_MS);
	}
}
