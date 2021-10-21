#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>
#include <stdbool.h>
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
#define LOG_LEVEL   ESP_LOG_WARN
#define AP_SSID     "esp32"
#define AP_PASS     "passw0rd"
#define AP_CHAN     1
#define AP_MAX_CONN 1

#define TCP_PORT               9000
#define TCP_KEEPALIVE_IDLE     5
#define TCP_KEEPALIVE_INTERVAL 5
#define TCP_KEEPALIVE_COUNT    3
#define TCP_MAX_RX_LEN         2048
#define TCP_RX_SIZE_T          uint32_t

#define LED_PIN_R  16
#define LED_PIN_G  17
#define LED_PIN_B  18
#define LED_PIN_B2 19

#define UART_NUM         UART_NUM_1
#define UART_BAUD        921600
#define UART_RX_BUF_SIZE 2048
#define UART_TX_BUF_SIZE 0          // No TX ring buffer, all sends are blocking
#define UART_TX_PIN      2
#define UART_RX_PIN      4
#define UART_RX_FLUSH_THRESHOLD (UART_RX_BUF_SIZE / 2)
#define UART_TIMEOUT_MS       1     // Time to wait for UART events
#define UART_TIMEOUT_LONG_MS  100   // Max time to wait for a UART response

static QueueHandle_t event_queue;
static QueueHandle_t txn_queue;

typedef struct {
	void *priv;
	uint8_t *tx_data;
	void (*rx_func)(void *priv, uint8_t *rx_data, uint16_t rx_len, bool error);
	uint16_t tx_len;
	uint16_t rx_len;
} txn_event_t;

struct tcp_txn_priv {
	int fd;
	TaskHandle_t tcp_task;
};

uint8_t tcp_rx_buf[4096];
uint8_t rx_data[4096];

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

static void uart_tx(uint8_t *data, int len)
{
	gpio_set_level(LED_PIN_B, 0);
	uart_write_bytes(UART_NUM, (const char*)data, len);
	gpio_set_level(LED_PIN_B, 1);
}

static uint16_t uart_rx_into(uint8_t *buf, uint16_t len)
{
	size_t avail = 0;

	gpio_set_level(LED_PIN_B, 0);

	uart_get_buffered_data_len(UART_NUM, &avail);

	avail = avail < len ? avail : len;

	ESP_LOGI(TAG, "uart_rx_into, read %d bytes", avail);
	uart_read_bytes(UART_NUM, buf, avail, portMAX_DELAY);

	ESP_LOG_BUFFER_HEXDUMP(TAG, buf, avail, ESP_LOG_INFO);

	gpio_set_level(LED_PIN_B, 1);

	return avail;
}

static void uart_handler_error()
{
	uart_flush_input(UART_NUM);
	xQueueReset(event_queue);
}

static void uart_handler_task(void *pvParameters)
{
	txn_event_t txn;
	uart_event_t uart_event;

	for ( ; ; ) {
		// Everything is driven by transactions
		ESP_LOGI(TAG, "Waiting for transaction");
		xQueueReceive(txn_queue, (void *)&txn, portMAX_DELAY);
		ESP_LOGI(TAG, "Transaction: %d bytes (%d)", txn.tx_len, txn.rx_len);

		// Start with an empty RX buffer for every transaction
		// TODO: Is this definitely what we want to do?
		uart_flush_input(UART_NUM);

		uart_tx(txn.tx_data, txn.tx_len);

		uint16_t rx_count = 0;
		bool error = false;

		TickType_t rx_start = xTaskGetTickCount();

		for (rx_count = 0, error = false; rx_count < txn.rx_len && !error ; ) {
			bool timeout = !xQueueReceive(event_queue, (void *)&uart_event, UART_TIMEOUT_MS / portTICK_PERIOD_MS);
			if (timeout) {
				TickType_t now = xTaskGetTickCount();

				uint16_t in_fifo = uart_rx_into(&rx_data[rx_count], txn.rx_len - rx_count);
				rx_count += in_fifo;

				if (in_fifo > 0) {
					// If there was data, wait for longer
					continue;
				} else if (rx_count > 0 || ((now - rx_start) > (UART_TIMEOUT_LONG_MS / portTICK_PERIOD_MS))) {
					// Timeout, send what we have
					ESP_LOGI(TAG, "UART timeout after %d bytes\n", rx_count);
					break;
				}

				// Haven't received any data yet, and the long
				// timeout hasn't been reached. Keep waiting
				continue;
			}

			switch(uart_event.type) {
			case UART_DATA:
				ESP_LOGI(TAG, "UART_DATA %d", uart_event.size);
				ESP_LOGI(TAG, "rx_count: %d, rx_len: %d\n", rx_count, txn.rx_len);
				rx_count += uart_rx_into(&rx_data[rx_count], txn.rx_len - rx_count);
				break;
			case UART_FIFO_OVF:
				// Fallthrough
			case UART_BUFFER_FULL:
				// Fallthrough
			case UART_PARITY_ERR:
				// Fallthrough
			case UART_FRAME_ERR:
				uart_handler_error();
				error = true;
				break;
			case UART_BREAK:
				// TODO: Are we going to use break?
				break;
			default:
				// Unexpected UART type
				break;
			}
		}

		// One last attempt to drain anything in the ring buffer
		rx_count += uart_rx_into(&rx_data[rx_count], txn.rx_len - rx_count);

		ESP_LOGI(TAG, "Calling rx_func %d bytes %d\n", rx_count, error);
		txn.rx_func(txn.priv, rx_data, rx_count, error);
	}

	vTaskDelete(NULL);
}

static void tcp_txn_rx_func(void *vpriv, uint8_t *rx_data, uint16_t rx_len, bool error)
{
	struct tcp_txn_priv *priv = (struct tcp_txn_priv *)vpriv;

	ESP_LOGI(TAG, "tcp_txn_rx_func %d bytes %d\n", rx_len, error);

	gpio_set_level(LED_PIN_B, 0);

	if (error) {
		shutdown(priv->fd, SHUT_RDWR);
		close(priv->fd);

		xTaskNotifyGive(priv->tcp_task);

		gpio_set_level(LED_PIN_B, 1);
		return;
	}

	int to_write = rx_len;
	while (to_write > 0) {
		ESP_LOGI(TAG, "writing %d bytes", to_write);
		int written = send(priv->fd, rx_data + (rx_len - to_write), to_write, 0);
		if (written < 0) {
			ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);

			shutdown(priv->fd, SHUT_RDWR);
			close(priv->fd);

			xTaskNotifyGive(priv->tcp_task);

			gpio_set_level(LED_PIN_B, 1);
			return;
		}
		to_write -= written;
	}

	ESP_LOGI(TAG, "Notify");
	xTaskNotifyGive(priv->tcp_task);

	gpio_set_level(LED_PIN_B, 1);
}

static void handle_connection(struct tcp_txn_priv *priv)
{
	gpio_set_level(LED_PIN_G, 0);
	for (;;) {
		int total_len = sizeof(TCP_RX_SIZE_T);
		int received = 0;
		for (received = 0; received < total_len; ) {
			int len = recv(priv->fd, &tcp_rx_buf[received], total_len - received, 0);
			if (len < 0) {
				// TODO: Error return?
				shutdown(priv->fd, SHUT_RDWR);
				close(priv->fd);

				gpio_set_level(LED_PIN_G, 1);
				return;
			}

			received += len;

			// TODO: Doing this every loop isn't strictly necessary.
			// We could add a variable to track if we've received
			// the length already.
			// But, this is probably really cheap.
			if (received >= sizeof(TCP_RX_SIZE_T)) {
				ESP_LOGI(TAG, "Received %d", received);
				total_len = sizeof(TCP_RX_SIZE_T) + ((TCP_RX_SIZE_T *)tcp_rx_buf)[0];
				ESP_LOGI(TAG, "Total len %d", total_len);
				if (total_len >= sizeof(tcp_rx_buf)) {
					ESP_LOGE(TAG, "Transaction too large: %d bytes", total_len);

					shutdown(priv->fd, SHUT_RDWR);
					close(priv->fd);

					gpio_set_level(LED_PIN_G, 1);
					return;
				}
			}
		}


		txn_event_t txn = {
			.priv = priv,
			// XXX: Careful of pointer arithmetic here.
			.tx_data = tcp_rx_buf + sizeof(TCP_RX_SIZE_T),
			.rx_func = tcp_txn_rx_func,
			.tx_len = total_len - sizeof(TCP_RX_SIZE_T),
			// TODO: Once we've plumbed in an rx_len, change this.
			// For now, we'll rely on timeout (which will kill throughput,
			// but means less changes to get this working).
			.rx_len = TCP_MAX_RX_LEN,
		};

		ESP_LOGI(TAG, "Transaction %d bytes", txn.tx_len);
		ESP_LOG_BUFFER_HEXDUMP(TAG, txn.tx_data, txn.tx_len, ESP_LOG_INFO);

		xQueueSend(txn_queue, &txn, portMAX_DELAY);

		// Block until complete, so we don't re-use fd or the buffer.
		// We'll change this to be better later.
		ESP_LOGI(TAG, "Waiting");
		ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
		ESP_LOGI(TAG, "Woken");
	}
	gpio_set_level(LED_PIN_G, 1);
}

static void tcp_handler_task(void *pvParameters)
{
	struct tcp_txn_priv priv = {
		.fd = -1,
		.tcp_task = xTaskGetCurrentTaskHandle()
	};

	int listen_fd = tcp_server_init();
	if (listen_fd < 0) {
		ESP_LOGE(TAG, "tcp init failed.");
		vTaskDelete(NULL);
		return;
	}

	struct sockaddr_storage addr;
	socklen_t addr_len;

	for (;;) {
		int fd = accept(listen_fd, (struct sockaddr *)&addr, &addr_len);
		if (fd >= 0) {
			char addr_str[128];
			int keepAlive = 1;
			int keepIdle = TCP_KEEPALIVE_IDLE;
			int keepInterval = TCP_KEEPALIVE_INTERVAL;
			int keepCount = TCP_KEEPALIVE_COUNT;

			inet_ntoa_r(((struct sockaddr_in *)&addr)->sin_addr, addr_str, sizeof(addr_str) - 1);
			ESP_LOGI(TAG, "Socket accepted ip address: %s", addr_str);

			setsockopt(fd, SOL_SOCKET, SO_KEEPALIVE, &keepAlive, sizeof(int));
			setsockopt(fd, IPPROTO_TCP, TCP_KEEPIDLE, &keepIdle, sizeof(int));
			setsockopt(fd, IPPROTO_TCP, TCP_KEEPINTVL, &keepInterval, sizeof(int));
			setsockopt(fd, IPPROTO_TCP, TCP_KEEPCNT, &keepCount, sizeof(int));

			priv.fd = fd;

			handle_connection(&priv);
		} else if (errno != EAGAIN && errno != EWOULDBLOCK) {
			ESP_LOGE(TAG, "Accept failed: errno %d", errno);
		}

		vTaskDelay(100 / portTICK_PERIOD_MS);
	}
}

void uart_init(void)
{
	uart_config_t uart_config = {
		.baud_rate = UART_BAUD,
		.data_bits = UART_DATA_8_BITS,
		.parity = UART_PARITY_DISABLE,
		.stop_bits = UART_STOP_BITS_1,
		.flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
		.source_clk = UART_SCLK_APB,
	};

	uart_driver_install(UART_NUM, UART_RX_BUF_SIZE, UART_TX_BUF_SIZE, 100, &event_queue, 0);
	uart_param_config(UART_NUM, &uart_config);
	uart_set_rx_full_threshold(UART_NUM, 100);

	uart_set_pin(UART_NUM, UART_TX_PIN, UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}

void app_main(void)
{
	esp_log_level_set(TAG, LOG_LEVEL);
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

	ESP_LOGI(TAG, "ESP_WIFI_MODE_AP");
	wifi_init_ap();

	txn_queue = xQueueCreate(5, sizeof(txn_event_t));

	xTaskCreate(uart_handler_task, "uart_task", 2048, NULL, 12, NULL);
	xTaskCreate(tcp_handler_task, "tcp_task", 4096, NULL, 10, NULL);

	while (1) {
		vTaskDelay(portMAX_DELAY);
	}
}
