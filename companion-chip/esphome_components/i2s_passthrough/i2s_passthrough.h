#pragma once

#include "esphome/core/component.h"
#include "esphome/core/log.h"

extern "C" {
  #include "freertos/FreeRTOS.h"
  #include "freertos/task.h"
  #include "esp_err.h"
  #include "driver/i2s_std.h"
}

namespace esphome {
namespace i2s_passthrough {

/*
 * I2SPassthrough — real-time I2S slave-to-master audio bridge
 *
 * Purpose:
 *   Receives I2S audio from an ESP32-S3 (Sendspin device) acting as I2S
 *   master, and retransmits it verbatim to an external DAC as a second
 *   I2S master.  Both ports run at 48 kHz, 16-bit stereo (Philips I2S).
 *
 * Signal routing:
 *   ESP32-S3  --[BCK/WS/DATA]--> ESP32 RX (I2S0, slave)
 *   ESP32 TX (I2S1, master)  --[BCK/WS/DATA]--> External DAC
 *
 * Pin assignments:
 *   RX from S3 : BCK = GPIO23, WS = GPIO18, DIN  = GPIO2
 *   TX to DAC  : BCK = GPIO25, WS = GPIO27, DOUT = GPIO26
 *   DAC unmute : GPIO32 (controlled separately via ESPHome switch)
 *
 * Clock handling:
 *   The RX port operates in slave mode, deriving its clock from the S3.
 *   If the S3 stops driving clocks (e.g. during boot or a reboot), the RX
 *   read will time out.  The bridge task detects this condition and disables
 *   the TX channel to silence the DAC, then re-enables it as soon as valid
 *   data resumes.  This avoids audible noise during clock-absent periods.
 *
 * Implementation notes:
 *   - The bridge loop runs in a dedicated FreeRTOS task pinned to core 1,
 *     keeping it clear of the ESPHome main loop on core 0.
 *   - Chunk size is 256 frames (1 KB at 16-bit stereo), matching the DMA
 *     descriptor count so that read/write operations stay DMA-aligned.
 *   - No sample-rate conversion or gain adjustment is applied; the bridge
 *     is intentionally transparent.
 */

static const char* TAG = "I2S_BRIDGE";

class I2SPassthrough : public Component {
 public:
  void setup() override {
    ESP_LOGI(TAG, "Starting I2S passthrough bridge");
    xTaskCreatePinnedToCore(
      &I2SPassthrough::task_trampoline_,
      "i2s_bridge",
      8192,    // stack size in bytes
      this,
      1,       // task priority
      &this->task_handle_,
      1        // pin to core 1 (ESPHome main loop runs on core 0)
    );
  }

  void loop() override {}  // all work is done in the bridge task

 private:
  static void task_trampoline_(void* arg) {
    static_cast<I2SPassthrough*>(arg)->task_main_();
  }

  // Returns the shared slot configuration used by both RX and TX channels.
  // 16-bit data in 16-bit slots, stereo, standard Philips I2S framing:
  //   bit_shift = true  : data MSB is one clock after the WS edge (Philips I2S)
  //   ws_pol    = false : WS low = left channel
  static i2s_std_slot_config_t slot_config_() {
    i2s_std_slot_config_t s{};
    s.data_bit_width = I2S_DATA_BIT_WIDTH_16BIT;
    s.slot_bit_width = I2S_SLOT_BIT_WIDTH_16BIT;
    s.slot_mode      = I2S_SLOT_MODE_STEREO;
    s.slot_mask      = I2S_STD_SLOT_BOTH;
    s.ws_width       = I2S_DATA_BIT_WIDTH_16BIT;
    s.ws_pol         = false;
    s.bit_shift      = true;   // Philips I2S: MSB one BCLK after WS edge
    s.msb_right      = false;
    return s;
  }

  void task_main_() {
    // Allow the S3 time to complete its boot sequence and begin driving clocks
    // before we attempt to open the RX channel.
    ESP_LOGI(TAG, "Waiting 3 s for S3 to start streaming...");
    vTaskDelay(pdMS_TO_TICKS(3000));

    // ── RX channel (I2S0, slave) — receives audio from the S3 ───────────────
    ESP_LOGI(TAG, "Configuring RX from S3 (BCK=GPIO23, WS=GPIO18, DIN=GPIO2)");

    i2s_chan_config_t rx_chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_SLAVE);
    rx_chan_cfg.dma_desc_num  = 8;
    rx_chan_cfg.dma_frame_num = 256;
    ESP_ERROR_CHECK(i2s_new_channel(&rx_chan_cfg, nullptr, &rx_chan_));

    i2s_std_config_t rx_std_cfg{};
    rx_std_cfg.clk_cfg  = I2S_STD_CLK_DEFAULT_CONFIG(48000);
    rx_std_cfg.slot_cfg = slot_config_();
    rx_std_cfg.gpio_cfg.mclk = I2S_GPIO_UNUSED;
    rx_std_cfg.gpio_cfg.bclk = GPIO_NUM_23;
    rx_std_cfg.gpio_cfg.ws   = GPIO_NUM_18;
    rx_std_cfg.gpio_cfg.dout = I2S_GPIO_UNUSED;
    rx_std_cfg.gpio_cfg.din  = GPIO_NUM_2;
    rx_std_cfg.gpio_cfg.invert_flags = {};

    ESP_ERROR_CHECK(i2s_channel_init_std_mode(rx_chan_, &rx_std_cfg));
    ESP_ERROR_CHECK(i2s_channel_enable(rx_chan_));
    ESP_LOGI(TAG, "RX channel enabled");

    // ── TX channel (I2S1, master) — drives the external DAC ─────────────────
    ESP_LOGI(TAG, "Configuring TX to DAC (BCK=GPIO25, WS=GPIO27, DOUT=GPIO26)");

    i2s_chan_config_t tx_chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_1, I2S_ROLE_MASTER);
    tx_chan_cfg.dma_desc_num  = 8;
    tx_chan_cfg.dma_frame_num = 256;
    ESP_ERROR_CHECK(i2s_new_channel(&tx_chan_cfg, &tx_chan_, nullptr));

    i2s_std_config_t tx_std_cfg{};
    tx_std_cfg.clk_cfg  = I2S_STD_CLK_DEFAULT_CONFIG(48000);
    tx_std_cfg.slot_cfg = slot_config_();
    tx_std_cfg.gpio_cfg.mclk = I2S_GPIO_UNUSED;
    tx_std_cfg.gpio_cfg.bclk = GPIO_NUM_25;
    tx_std_cfg.gpio_cfg.ws   = GPIO_NUM_27;
    tx_std_cfg.gpio_cfg.dout = GPIO_NUM_26;
    tx_std_cfg.gpio_cfg.din  = I2S_GPIO_UNUSED;
    tx_std_cfg.gpio_cfg.invert_flags = {};

    ESP_ERROR_CHECK(i2s_channel_init_std_mode(tx_chan_, &tx_std_cfg));
    ESP_ERROR_CHECK(i2s_channel_enable(tx_chan_));
    ESP_LOGI(TAG, "TX channel enabled");

    // ── Bridge loop ──────────────────────────────────────────────────────────
    // Reads 256-frame chunks from the RX DMA and writes them directly to the
    // TX DMA.  Short timeouts allow rapid detection of clock loss on the RX
    // side without blocking the task indefinitely.
    const size_t BYTES_PER_FRAME  = 4;    // 16-bit stereo = 2 ch x 2 bytes
    const size_t FRAMES_PER_CHUNK = 256;
    const size_t BUF_BYTES        = BYTES_PER_FRAME * FRAMES_PER_CHUNK;

    uint8_t* buf = static_cast<uint8_t*>(malloc(BUF_BYTES));

    const TickType_t READ_TIMEOUT  = pdMS_TO_TICKS(10);
    const TickType_t WRITE_TIMEOUT = pdMS_TO_TICKS(10);

    uint32_t total_frames = 0;
    bool     tx_enabled   = true;

    ESP_LOGI(TAG, "Bridge running — streaming audio");

    while (true) {
      size_t   rbytes = 0;
      esp_err_t r = i2s_channel_read(rx_chan_, buf, BUF_BYTES, &rbytes, READ_TIMEOUT);

      if (r == ESP_OK && rbytes > 0) {
        // Valid data received.  If the TX channel was previously stopped due
        // to a clock-loss condition, re-enable it before writing.
        if (!tx_enabled) {
          ESP_LOGI(TAG, "Signal resumed — re-enabling DAC output");
          i2s_channel_enable(tx_chan_);
          tx_enabled = true;
        }

        size_t wbytes = 0;
        i2s_channel_write(tx_chan_, buf, rbytes, &wbytes, WRITE_TIMEOUT);

        total_frames += rbytes / BYTES_PER_FRAME;
        // Log throughput roughly every 10 seconds at 48 kHz
        if (total_frames % 480000 == 0) {
          ESP_LOGI(TAG, "Streaming: %lu frames", total_frames);
        }

      } else if (r == ESP_ERR_TIMEOUT) {
        // No data within the timeout window — the S3 has stopped sending clocks.
        // Disable the TX channel so the DAC outputs silence rather than noise.
        if (tx_enabled) {
          ESP_LOGW(TAG, "Signal lost — disabling DAC output");
          i2s_channel_disable(tx_chan_);
          tx_enabled = false;
        }
        vTaskDelay(1);  // yield to avoid triggering the watchdog while idle

      } else {
        ESP_LOGW(TAG, "RX read error: %s", esp_err_to_name(r));
        vTaskDelay(1);
      }
    }
  }

  TaskHandle_t       task_handle_{nullptr};
  i2s_chan_handle_t  rx_chan_{nullptr};
  i2s_chan_handle_t  tx_chan_{nullptr};
};

}  // namespace i2s_passthrough
}  // namespace esphome