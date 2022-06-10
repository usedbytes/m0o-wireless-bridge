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
#ifndef __BT_HID_H__
#define __BT_HID_H__

#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#define BTN_BIT_A       0
#define BTN_BIT_B       1
#define BTN_BIT_X       2
#define BTN_BIT_Y       3
#define BTN_BIT_L1      4
#define BTN_BIT_L2      5
#define BTN_BIT_L3      6
#define BTN_BIT_R1      7
#define BTN_BIT_R2      8
#define BTN_BIT_R3      9
#define BTN_BIT_START   10
#define BTN_BIT_SELECT  11
#define BTN_BIT_HEART   12
#define BTN_BIT_STAR    13

struct bt_hid_state {
	uint16_t buttons;
	uint8_t lx;
	uint8_t ly;
	uint8_t rx;
	uint8_t ry;
	uint8_t hat;
	uint8_t pad;
};

struct bt_hid_task_params {
	QueueHandle_t state_queue;
};

void bt_hid_task(void *pvParameters);

#endif /* __BT_HID_H__ */
