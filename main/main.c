#include <stdint.h>

#include "driver/usb_serial_jtag.h"
#include "esp_err.h"
#include "esp_twai.h"
#include "esp_twai_onchip.h"
#include "freertos/FreeRTOS.h" // IWYU pragma: keep
#include "freertos/queue.h"
#include "freertos/task.h"
#include <unistd.h>

#define TWAI_PORT_RX 0
#define TWAI_PORT_TX 1

char hex_map[16] = {'0', '1', '2', '3', '4', '5', '6', '7',
                    '8', '9', 'A', 'B', 'C', 'D', 'E', 'F'};

enum twai_state_t {
  OPEN,
  LISTEN,
  CLOSED,
};

enum twai_bitrate_t {
  BITRATE_10K = 10000,
  BITRATE_20K = 20000,
  BITRATE_50K = 50000,
  BITRATE_100K = 100000,
  BITRATE_125K = 125000,
  BITRATE_250K = 250000,
  BITRATE_500K = 500000,
  BITRATE_800K = 800000,
  BITRATE_1M = 1000000,
};

enum twai_bitrate_t twai_bitrate[] = {BITRATE_10K,  BITRATE_20K,  BITRATE_50K,
                                      BITRATE_100K, BITRATE_125K, BITRATE_250K,
                                      BITRATE_500K, BITRATE_800K, BITRATE_1M};

enum twai_state_t twai_mode = CLOSED;

twai_node_handle_t node_hdl = NULL;
twai_onchip_node_config_t node_config = {
    .io_cfg.tx = TWAI_PORT_TX, // TWAI TX GPIO pin
    .io_cfg.rx = TWAI_PORT_RX, // TWAI RX GPIO pin
    .io_cfg.quanta_clk_out = GPIO_NUM_NC,
    .io_cfg.bus_off_indicator = GPIO_NUM_NC,
    .bit_timing.bitrate = BITRATE_250K, // 250 kbps bitrate
    .tx_queue_depth = 5,                // Transmit queue depth set to 5
};

static QueueHandle_t twai_rx_queue;
static QueueHandle_t twai_tx_queue;

void set_twai_mode(char *command) {
  char mode = command[0];
  switch (mode) {
  case 'S':
    if (twai_mode == CLOSED) {
      node_config.bit_timing.bitrate = twai_bitrate[command[2] + '0'];
    }
    break;
  case 'O':
    if (twai_mode == CLOSED) {
      ESP_ERROR_CHECK(twai_node_enable(node_hdl));
    }
    twai_mode = OPEN;
    break;
  case 'L':
    if (twai_mode == CLOSED) {
      ESP_ERROR_CHECK(twai_node_enable(node_hdl));
    }
    twai_mode = LISTEN;
    break;
  case 'C':
    if (twai_mode == OPEN) {
      ESP_ERROR_CHECK(twai_node_disable(node_hdl));
    }
    twai_mode = CLOSED;
    break;
  }
}

void send_frame_can_bus(void *pvParameter) {
  // send frame using can bus protocol to twai
  twai_frame_t twai_frame;
  int id_length;
  char message[64];
  char data[8];
  while (1) {
    if (xQueueReceive(twai_tx_queue, &message, portMAX_DELAY) == pdTRUE) {
      if (message[0] == 't') {
        id_length = 3;
        uint32_t id = 0;
        for (int i = 0; i < id_length; i++) {
          id |= ((9 * ((message[i + 1] >> 6) == 1) + ((message[i + 1] & 0x0F)))
                 << (4 * (id_length - i - 1)));
        }
        twai_frame.header.id = id;
        twai_frame.header.ide = false;
        twai_frame.header.rtr = false;
        int offset = id_length + 2;
        int dlc = (message[id_length + 1] & 0x0F);
        for (int i = 0; i < dlc; i++) {
          data[i] = 0;
          data[i] |= ((9 * ((message[2 * i + offset] >> 6) == 1) +
                       ((message[2 * i + offset] & 0x0F)))
                      << 4);
          data[i] |= (9 * ((message[2 * i + offset + 1] >> 6) == 1) +
                      ((message[2 * i + offset + 1] & 0x0F)));
        }
        twai_frame.header.dlc = dlc;
        twai_frame.buffer = (uint8_t *)data;
        twai_frame.buffer_len = dlc;
        ESP_ERROR_CHECK(twai_node_transmit(
            node_hdl, &twai_frame,
            0)); // Timeout = 0: returns immediately if queue is full
        ESP_ERROR_CHECK(twai_node_transmit_wait_all_done(
            node_hdl, -1)); // Wait for transmission to finish
      }
    }
  }
}

void send_frame_to_slcan(void *pvParameter) {
  // send frame using slcan protocol to usb serial jtag
  twai_frame_t message;
  while (1) {
    if (xQueueReceive(twai_rx_queue, &message, portMAX_DELAY) == pdTRUE) {
      char frame[32];
      int id_length;
      int frame_idx = 0;
      if (message.header.ide == true) {
        frame[0] = 'T';
        id_length = 8;
      } else {
        frame[0] = 't';
        id_length = 3;
      }

      frame_idx += 1;

      for (int i = frame_idx; i <= id_length; i++) {
        frame[i] = hex_map[(int)((message.header.id >> (4 * (id_length - i))) &
                                 0x0000000F)];
      }

      frame_idx += id_length;

      frame[frame_idx] = hex_map[(int)(message.header.dlc & 0x0000000F)];

      frame_idx += 1;

      for (int i = 0; i < message.buffer_len; i++) {
        frame[frame_idx + 2 * i] =
            hex_map[(int)((message.buffer[i] >> 4) & 0x0000000F)];
        frame[frame_idx + 2 * i + 1] =
            hex_map[(int)(message.buffer[i] & 0x0000000F)];
      }

      frame_idx += message.buffer_len * 2;
      frame[frame_idx] = '\r';
      frame_idx += 1;
      usb_serial_jtag_write_bytes((const uint8_t *)&frame, frame_idx,
                                  pdMS_TO_TICKS(100));
    }
  }
}

void read_serial(void *pvParameter) {
  char c;
  char buffer[64];
  int count = 0;

  while (1) {
    int len = usb_serial_jtag_read_bytes((uint8_t *)&c, 1, pdMS_TO_TICKS(100));

    if (len > 0) {
      buffer[count++] = c;
      if (c == '\r' || c == '\n' || c == '\0') {
        buffer[count] = '\0';
        // printf("command: %s\n", buffer);
        count = 0;
        if (buffer[0] != 'T' && buffer[0] != 't') {
          set_twai_mode(buffer);
        } else {
          xQueueSend(twai_tx_queue, &buffer, portMAX_DELAY);
        }
      }
    } else {
      vTaskDelay(pdMS_TO_TICKS(10));
    }
  }
}

static bool twai_rx_cb(twai_node_handle_t handle,
                       const twai_rx_done_event_data_t *edata, void *user_ctx) {
  uint8_t recv_buff[8];
  twai_frame_t rx_frame = {
      .buffer = recv_buff,
      .buffer_len = sizeof(recv_buff),
  };
  if (ESP_OK == twai_node_receive_from_isr(handle, &rx_frame)) {
    xQueueSendFromISR(twai_rx_queue, &rx_frame, NULL);
  }
  return false;
}

void app_main(void) {

  twai_rx_queue = xQueueCreate(16, sizeof(twai_frame_t));
  twai_tx_queue = xQueueCreate(16, sizeof(twai_frame_t));
  // Create a new TWAI controller driver instance
  ESP_ERROR_CHECK(twai_new_node_onchip(&node_config, &node_hdl));

  twai_event_callbacks_t user_cbs = {
      .on_rx_done = twai_rx_cb,
  };
  ESP_ERROR_CHECK(
      twai_node_register_event_callbacks(node_hdl, &user_cbs, NULL));

  // Start the TWAI controller
  //   ESP_ERROR_CHECK(twai_node_enable(node_hdl));
  if (!usb_serial_jtag_is_driver_installed()) {
    usb_serial_jtag_driver_config_t usb_serial_jtag_config =
        USB_SERIAL_JTAG_DRIVER_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(usb_serial_jtag_driver_install(&usb_serial_jtag_config));
  }

  //   xTaskCreate(collectSerial, "collectSerial", 1024, NULL, 10, NULL);
  xTaskCreate(read_serial, "read_serial", 1024, NULL, 10, NULL);
  xTaskCreate(send_frame_to_slcan, "send_frame_to_slcan", 1024, NULL, 10, NULL);
  xTaskCreate(send_frame_can_bus, "send_frame_can_bus", 1024, NULL, 10, NULL);
}
