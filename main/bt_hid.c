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

#include "btstack_config.h"
#include "btstack.h"

#define MAX_ATTRIBUTE_VALUE_SIZE 300

// MBP 2016 static const char * remote_addr_string = "F4-0F-24-3B-1B-E1";
// iMpulse static const char * remote_addr_string = "64:6E:6C:C1:AA:B5";
// Logitec 
//static const char * remote_addr_string = "00:1F:20:86:DF:52";
// SN30
static const char * remote_addr_string = "E4:17:D8:EE:73:0E";
// Real ds4
// static const char * remote_addr_string = "00:22:68:DB:D3:66";
// E4:17:D8:EE:73:0E

static bd_addr_t remote_addr;

static btstack_packet_callback_registration_t hci_event_callback_registration;

// SDP
static uint8_t hid_descriptor_storage[MAX_ATTRIBUTE_VALUE_SIZE];

// App
static enum {
	APP_IDLE,
	APP_CONNECTED
} app_state = APP_IDLE;

static uint16_t hid_host_cid = 0;
static bool     hid_host_descriptor_available = false;
//static hid_protocol_mode_t hid_host_report_mode = HID_PROTOCOL_MODE_REPORT_WITH_FALLBACK_TO_BOOT;
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

	// Disable stdout buffering
	setbuf(stdout, NULL);
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
	gap_drop_link_key_for_bd_addr(addr);
}

static void packet_handler (uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size)
{
	/* LISTING_PAUSE */
	UNUSED(channel);
	UNUSED(size);

	uint8_t   event;
	bd_addr_t event_addr;
	uint8_t   status;

	/* LISTING_RESUME */
	switch (packet_type) {
		case HCI_EVENT_PACKET:
			event = hci_event_packet_get_type(packet);

			switch (event) {            
#if 1 //HAVE_BTSTACK_STDIN
				/* @text When BTSTACK_EVENT_STATE with state HCI_STATE_WORKING
				 * is received and the example is started in client mode, the remote SDP HID query is started.
				 */
				case BTSTACK_EVENT_STATE:
					if (btstack_event_state_get_state(packet) == HCI_STATE_WORKING){
						printf("Start hid_host_connect\n");
						status = hid_host_connect(remote_addr, hid_host_report_mode, &hid_host_cid);
						if (status != ERROR_CODE_SUCCESS){
							printf("HID host connect failed, status 0x%02x.\n", status);
						} else {
							printf("Connected.\n");
						}
					}
					break;
#endif
					/* LISTING_PAUSE */
				case HCI_EVENT_PIN_CODE_REQUEST:
					// inform about pin code request
					printf("Pin code request - using '0000'\n");
					hci_event_pin_code_request_get_bd_addr(packet, event_addr);
					gap_pin_code_response(event_addr, "0000");
					break;

				case HCI_EVENT_USER_CONFIRMATION_REQUEST:
					// inform about user confirmation request
					printf("SSP User Confirmation Request with numeric value '%"PRIu32"'\n", little_endian_read_32(packet, 8));
					printf("SSP User Confirmation Auto accept\n");
					break;

					/* LISTING_RESUME */
				case HCI_EVENT_HID_META:
					switch (hci_event_hid_meta_get_subevent_code(packet)){

						case HID_SUBEVENT_INCOMING_CONNECTION:
							// There is an incoming connection: we can accept it or decline it.
							// The hid_host_report_mode in the hid_host_accept_connection function 
							// allows the application to request a protocol mode. 
							// For available protocol modes, see hid_protocol_mode_t in btstack_hid.h file. 
							printf("Accept connection\n");
							hid_host_accept_connection(hid_subevent_incoming_connection_get_hid_cid(packet), hid_host_report_mode);
							break;

						case HID_SUBEVENT_CONNECTION_OPENED:
							// The status field of this event indicates if the control and interrupt
							// connections were opened successfully.
							printf("Connection opened\n");
							status = hid_subevent_connection_opened_get_status(packet);
							if (status != ERROR_CODE_SUCCESS) {
								bd_addr_t addr;
								hid_subevent_connection_opened_get_bd_addr(packet, addr);
								printf("Connection failed, status 0x%x, %s\n", status, bd_addr_to_str(addr));
								app_state = APP_IDLE;
								hid_host_cid = 0;
								bt_hid_disconnected(addr);
								return;
							}
							app_state = APP_CONNECTED;
							hid_host_descriptor_available = false;
							hid_host_cid = hid_subevent_connection_opened_get_hid_cid(packet);
							printf("HID Host connected.\n");
							break;

						case HID_SUBEVENT_DESCRIPTOR_AVAILABLE:
							printf("Descriptor available\n");
							// This event will follows HID_SUBEVENT_CONNECTION_OPENED event. 
							// For incoming connections, i.e. HID Device initiating the connection,
							// the HID_SUBEVENT_DESCRIPTOR_AVAILABLE is delayed, and some HID  
							// reports may be received via HID_SUBEVENT_REPORT event. It is up to 
							// the application if these reports should be buffered or ignored until 
							// the HID descriptor is available.
							status = hid_subevent_descriptor_available_get_status(packet);
							if (status == ERROR_CODE_SUCCESS){
								hid_host_descriptor_available = true;
								printf("HID Descriptor available, please start typing.\n");
							} else {
								printf("Cannot handle input report, HID Descriptor is not available.\n");
							}
							break;

						case HID_SUBEVENT_REPORT:
							printf("Report\n");
							// Handle input report.
							if (hid_host_descriptor_available){
								hid_host_handle_interrupt_report(hid_subevent_report_get_report(packet), hid_subevent_report_get_report_len(packet));
							} else {
								printf_hexdump(hid_subevent_report_get_report(packet), hid_subevent_report_get_report_len(packet));
							}
							break;

						case HID_SUBEVENT_SET_PROTOCOL_RESPONSE:
							printf("Set proto response\n");
							// For incoming connections, the library will set the protocol mode of the
							// HID Device as requested in the call to hid_host_accept_connection. The event 
							// reports the result. For connections initiated by calling hid_host_connect, 
							// this event will occur only if the established report mode is boot mode.
							status = hid_subevent_set_protocol_response_get_handshake_status(packet);
							if (status != HID_HANDSHAKE_PARAM_TYPE_SUCCESSFUL){
								printf("Error set protocol, status 0x%02x\n", status);
								break;
							}
							switch ((hid_protocol_mode_t)hid_subevent_set_protocol_response_get_protocol_mode(packet)){
								case HID_PROTOCOL_MODE_BOOT:
									printf("Protocol mode set: BOOT.\n");
									break;  
								case HID_PROTOCOL_MODE_REPORT:
									printf("Protocol mode set: REPORT.\n");
									break;
								default:
									printf("Unknown protocol mode.\n");
									break; 
							}
							break;

						case HID_SUBEVENT_CONNECTION_CLOSED:
							// The connection was closed.
							hid_host_cid = 0;
							hid_host_descriptor_available = false;
							printf("HID Host disconnected.\n");
							bt_hid_disconnected(remote_addr);
							break;

						default:
							break;
					}
					break;
				default:
					break;
			}
			break;
		default:
			break;
	}
}

int btstack_main(int argc, const char * argv[]);
int btstack_main(int argc, const char * argv[]){

    (void)argc;
    (void)argv;

    hid_host_setup();

    // parse human readable Bluetooth address
    sscanf_bd_addr(remote_addr_string, remote_addr);
    bt_hid_disconnected(remote_addr);

#if 0 //HAVE_BTSTACK_STDIN
    btstack_stdin_setup(stdin_process);
#endif

    // Turn on the device 
    hci_power_control(HCI_POWER_ON);

    printf("btstack_main returning\n");

    return 0;
}
