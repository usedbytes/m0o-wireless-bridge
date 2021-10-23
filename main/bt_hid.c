/*
 * Derived from the btstack hid_host_demo:
 * Copyright (C) 2017 BlueKitchen GmbH
 *
 * Modifications Copyright (C) 2021 Brian Starkey <stark3y@gmail.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holders nor the names of
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 * 4. Any redistribution, use, or modification is done solely for
 *    personal benefit and not for any commercial purpose or for
 *    monetary gain.
 *
 * THIS SOFTWARE IS PROVIDED BY BLUEKITCHEN GMBH AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL MATTHIAS
 * RINGWALD OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
 * THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * Please inquire about commercial licensing options at
 * contact@bluekitchen-gmbh.com
 *
 */
#include <inttypes.h>
#include <stdio.h>

#include "esp_log.h"

#include "btstack_config.h"
#include "btstack.h"

#define TAG "bt_hid"
#define LOG_LEVEL   ESP_LOG_WARN

#define MAX_ATTRIBUTE_VALUE_SIZE 300

// SN30 Pro
static const char * remote_addr_string = "E4:17:D8:EE:73:0E";
// Real DS4
// static const char * remote_addr_string = "00:22:68:DB:D3:66";

static bd_addr_t remote_addr;
static bd_addr_t connected_addr;

static btstack_packet_callback_registration_t hci_event_callback_registration;

// SDP
static uint8_t hid_descriptor_storage[MAX_ATTRIBUTE_VALUE_SIZE];

static uint16_t hid_host_cid = 0;
static bool     hid_host_descriptor_available = false;
static hid_protocol_mode_t hid_host_report_mode = HID_PROTOCOL_MODE_REPORT;

static void packet_handler (uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size);

static void hid_host_setup(void){
	// Initialize L2CAP
	l2cap_init();

	// Initialize HID Host
	hid_host_init(hid_descriptor_storage, sizeof(hid_descriptor_storage));
	hid_host_register_packet_handler(packet_handler);

	// Allow sniff mode requests by HID device and support role switch
	gap_set_default_link_policy_settings(LM_LINK_POLICY_ENABLE_SNIFF_MODE | LM_LINK_POLICY_ENABLE_ROLE_SWITCH);

	// try to become master on incoming connections
	hci_set_master_slave_policy(HCI_ROLE_MASTER);

	// register for HCI events
	hci_event_callback_registration.callback = &packet_handler;
	hci_add_event_handler(&hci_event_callback_registration);
}

static void hid_host_handle_interrupt_report(const uint8_t * report, uint16_t report_len){
	// check if HID Input Report
	if (report_len < 1) return;
	if (*report != 0xa1) return;

	report++;
	report_len--;

	btstack_hid_parser_t parser;
	btstack_hid_parser_init(&parser,
			hid_descriptor_storage_get_descriptor_data(hid_host_cid),
			hid_descriptor_storage_get_descriptor_len(hid_host_cid),
			HID_REPORT_TYPE_INPUT, report, report_len);

	int shift = 0;
	while (btstack_hid_parser_has_more(&parser)){
		uint16_t usage_page;
		uint16_t usage;
		int32_t  value;
		btstack_hid_parser_get_field(&parser, &usage_page, &usage, &value);
		printf("Usage page: 0x%x, 0x%x, 0x%x\n", usage_page, usage, value);
		if (usage_page != 0x07) continue;
		switch (usage){
			case 0xe1:
			case 0xe6:
				if (value){
					shift = 1;
				}
				continue;
			case 0x00:
				continue;
			default:
				break;
		}
	}
}

static void bt_hid_disconnected(bd_addr_t addr)
{
	hid_host_cid = 0;
	hid_host_descriptor_available = false;
	gap_drop_link_key_for_bd_addr(addr);
}

static void packet_handler (uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size)
{
	UNUSED(channel);
	UNUSED(size);

	uint8_t   event;
	uint8_t   hid_event;
	bd_addr_t event_addr;
	uint8_t   status;

	if (packet_type != HCI_EVENT_PACKET) {
		return;
	}

	event = hci_event_packet_get_type(packet);
	switch (event) {
	case BTSTACK_EVENT_STATE:
		// On boot, we try a manual connection
		if (btstack_event_state_get_state(packet) == HCI_STATE_WORKING){
			ESP_LOGI(TAG, "Starting hid_host_connect (%s)", bd_addr_to_str(remote_addr));
			status = hid_host_connect(remote_addr, hid_host_report_mode, &hid_host_cid);
			if (status != ERROR_CODE_SUCCESS){
				ESP_LOGE(TAG, "hid_host_connect command failed: 0x%02x", status);
			}
		}
		break;
	case HCI_EVENT_PIN_CODE_REQUEST:
		ESP_LOGI(TAG, "Pin code request. Responding '0000'");
		hci_event_pin_code_request_get_bd_addr(packet, event_addr);
		gap_pin_code_response(event_addr, "0000");
		break;
	case HCI_EVENT_USER_CONFIRMATION_REQUEST:
		ESP_LOGI(TAG, "SSP User Confirmation Request: %d", little_endian_read_32(packet, 8));
		break;
	case HCI_EVENT_HID_META:
		hid_event = hci_event_hid_meta_get_subevent_code(packet);
		switch (hid_event) {
		case HID_SUBEVENT_INCOMING_CONNECTION:
			hid_subevent_incoming_connection_get_address(packet, event_addr);
			ESP_LOGI(TAG, "Accepting connection from %s", bd_addr_to_str(event_addr));
			hid_host_accept_connection(hid_subevent_incoming_connection_get_hid_cid(packet), hid_host_report_mode);
			break;
		case HID_SUBEVENT_CONNECTION_OPENED:
			status = hid_subevent_connection_opened_get_status(packet);
			hid_subevent_connection_opened_get_bd_addr(packet, event_addr);
			if (status != ERROR_CODE_SUCCESS) {
				ESP_LOGE(TAG, "Connection to %s failed: 0x%02x", bd_addr_to_str(event_addr), status);
				bt_hid_disconnected(event_addr);
				return;
			}
			hid_host_descriptor_available = false;
			hid_host_cid = hid_subevent_connection_opened_get_hid_cid(packet);
			ESP_LOGI(TAG, "Connected to %s", bd_addr_to_str(event_addr));
			bd_addr_copy(connected_addr, event_addr);
			break;
		case HID_SUBEVENT_DESCRIPTOR_AVAILABLE:
			status = hid_subevent_descriptor_available_get_status(packet);
			if (status == ERROR_CODE_SUCCESS){
				hid_host_descriptor_available = true;
				ESP_LOGI(TAG, "HID descriptor available");
			} else {
				ESP_LOGE(TAG, "Couldn't process HID Descriptor");
			}
			break;
		case HID_SUBEVENT_REPORT:
			if (hid_host_descriptor_available){
				hid_host_handle_interrupt_report(hid_subevent_report_get_report(packet), hid_subevent_report_get_report_len(packet));
			} else {
				printf_hexdump(hid_subevent_report_get_report(packet), hid_subevent_report_get_report_len(packet));
			}
			break;
		case HID_SUBEVENT_SET_PROTOCOL_RESPONSE:
			status = hid_subevent_set_protocol_response_get_handshake_status(packet);
			if (status != HID_HANDSHAKE_PARAM_TYPE_SUCCESSFUL){
				ESP_LOGE(TAG, "Protocol handshake error: 0x%02x", status);
				break;
			}
			hid_protocol_mode_t proto = hid_subevent_set_protocol_response_get_protocol_mode(packet);
			switch (proto) {
			case HID_PROTOCOL_MODE_BOOT:
				ESP_LOGI(TAG, "Negotiated protocol: BOOT");
				break;
			case HID_PROTOCOL_MODE_REPORT:
				ESP_LOGI(TAG, "Negotiated protocol: REPORT");
				break;
			default:
				ESP_LOGE(TAG, "Negotiated unknown protocol: 0x%x", proto);
				break;
			}
			break;
		case HID_SUBEVENT_CONNECTION_CLOSED:
			ESP_LOGI(TAG, "HID connection closed: %s", bd_addr_to_str(connected_addr));
			bt_hid_disconnected(connected_addr);
			break;
		default:
			ESP_LOGI(TAG, "Unknown HID subevent: 0x%x", hid_event);
			break;
		}
		break;
	default:
		ESP_LOGI(TAG, "Unknown HCI event: 0x%x", event);
		break;
	}
}

int btstack_main(int argc, const char * argv[]);
int btstack_main(int argc, const char * argv[]){

	(void)argc;
	(void)argv;

	esp_log_level_set(TAG, LOG_LEVEL);

	hid_host_setup();

	sscanf_bd_addr(remote_addr_string, remote_addr);
	bt_hid_disconnected(remote_addr);

	hci_power_control(HCI_POWER_ON);

	return 0;
}
