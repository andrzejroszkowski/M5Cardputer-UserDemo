// Copyright (c) M5Stack. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#include "Mic_Class.hpp"

#include <string.h>
#include <algorithm>

#include <sdkconfig.h>
#include <esp_log.h>
#include <esp_err.h>
#include <math.h>

#define __STDC_FORMAT_MACROS
#include <inttypes.h>

namespace m5
{
#define COMM_FORMAT_I2S (I2S_COMM_FORMAT_STAND_I2S)
#define COMM_FORMAT_MSB (I2S_COMM_FORMAT_STAND_MSB)
#define SAMPLE_RATE_TYPE uint32_t
#define MIC_CLASS_ADC_WIDTH_BITS ADC_WIDTH_BIT_12
#define MIC_CLASS_ADC_ATTEN_DB ADC_ATTEN_DB_11

  uint32_t Mic_Class::_calc_rec_rate(void) const
  {
    int rate = (_cfg.sample_rate * _cfg.over_sampling);
    return rate;
  }

  esp_err_t Mic_Class::_setup_i2s(void)
  {
    if (_cfg.pin_data_in < 0)
    {
      return ESP_FAIL;
    }

    SAMPLE_RATE_TYPE sample_rate = _calc_rec_rate();
    bool use_pdm = (_cfg.pin_bck < 0);
    if (_cfg.use_adc)
    {
      sample_rate >>= 4;
      use_pdm = false;
    }

    // i2s_start(self->_cfg.i2s_port);

    // https://github.com/espressif/esp-idf/blob/v5.0.6/examples/peripherals/i2s/i2s_basic/i2s_pdm/main/i2s_pdm_rx.c
    // https://github.com/espressif/esp-idf/blob/v5.0.6/examples/peripherals/i2s/i2s_basic/i2s_std/main/i2s_std_example_main.c
    // do we need pdm mode here ?    

    i2s_chan_handle_t mic_handle;

    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_AUTO, I2S_ROLE_MASTER);

    ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, NULL, &mic_handle));

    i2s_pdm_rx_config_t pdm_rx_cfg = {
        .clk_cfg = I2S_PDM_RX_CLK_DEFAULT_CONFIG(sample_rate),
        .slot_cfg = I2S_PDM_RX_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO),
        .gpio_cfg = {
            .clk = static_cast<gpio_num_t>(_cfg.pin_mck),
            .din = static_cast<gpio_num_t>(_cfg.pin_data_in),
            .invert_flags = {
                .clk_inv = false,
            },
        },
    };

    i2s_config.mode = use_pdm
                          //  ? (i2s_mode_t)( I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_PDM );
                          ? (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX | (0x1 << 6)) // 0x1<<6 is I2S_MODE_PDM
                          : (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX);
    i2s_config.channel_format = _cfg.stereo ? I2S_CHANNEL_FMT_RIGHT_LEFT : I2S_CHANNEL_FMT_ONLY_RIGHT;
    i2s_config.communication_format = (i2s_comm_format_t)(COMM_FORMAT_I2S);
    i2s_config.dma_buf_count = _cfg.dma_buf_count;
    i2s_config.dma_buf_len = _cfg.dma_buf_len;

    i2s_pin_config_t pin_config;
    // pin_config.mck_io_num = _cfg.pin_mck;
    pin_config.bck_io_num = _cfg.pin_bck;
    pin_config.ws_io_num = _cfg.pin_ws;
    // pin_config.data_in_num = _cfg.pin_data_in;

    esp_err_t err;
    if (ESP_OK != (err = i2s_driver_install(_cfg.i2s_port, &i2s_config, 0, nullptr)))
    {
      i2s_driver_uninstall(_cfg.i2s_port);
      err = i2s_driver_install(_cfg.i2s_port, &i2s_config, 0, nullptr);
    }
    if (err != ESP_OK)
    {
      return err;
    }

    err = i2s_set_pin(_cfg.i2s_port, &pin_config);

    return err;
  }

  void Mic_Class::mic_task(void *args)
  {
    auto self = (Mic_Class *)args;

    int oversampling = self->_cfg.over_sampling;
    if (oversampling < 1)
    {
      oversampling = 1;
    }
    else if (oversampling > 8)
    {
      oversampling = 8;
    }
    int32_t gain = self->_cfg.magnification;
    const float f_gain = (float)gain / oversampling;
    int32_t offset = self->_cfg.input_offset;
    size_t src_idx = ~0u;
    size_t src_len = 0;
    int32_t value = 0;
    int32_t prev_value[2] = {0, 0};
    bool flip = false;
    const bool stereo = self->_cfg.stereo;
    int32_t os_remain = oversampling;
    const size_t dma_buf_len = self->_cfg.dma_buf_len;
    int16_t *src_buf = (int16_t *)alloca(dma_buf_len * sizeof(int16_t));

    i2s_read(self->_cfg.i2s_port, src_buf, dma_buf_len, &src_len, portTICK_PERIOD_MS);
    i2s_read(self->_cfg.i2s_port, src_buf, dma_buf_len, &src_len, portTICK_PERIOD_MS);

    while (self->_task_running)
    {
      bool rec_flip = self->_rec_flip;
      recording_info_t *current_rec = &(self->_rec_info[!rec_flip]);
      recording_info_t *next_rec = &(self->_rec_info[rec_flip]);

      size_t dst_remain = current_rec->length;
      if (dst_remain == 0)
      {
        rec_flip = !rec_flip;
        self->_rec_flip = rec_flip;
        xSemaphoreGive(self->_task_semaphore);
        std::swap(current_rec, next_rec);
        dst_remain = current_rec->length;
        if (dst_remain == 0)
        {
          self->_is_recording = false;
          ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
          src_idx = ~0u;
          src_len = 0;
          value = 0;
          continue;
        }
      }
      self->_is_recording = true;

      for (;;)
      {
        if (src_idx >= src_len)
        {
          i2s_read(self->_cfg.i2s_port, src_buf, dma_buf_len, &src_len, 100 / portTICK_PERIOD_MS);
          src_len >>= 1;
          src_idx = 0;
        }

        if (self->_cfg.use_adc)
        {
          do
          {
            value += (src_buf[src_idx ^ 1] & 0x0FFF) + (offset - 2048);
            ++src_idx;
          } while (--os_remain && (src_idx < src_len));
        }
        else
        {
          do
          {
            value += src_buf[src_idx] + offset;
            ++src_idx;
          } while (--os_remain && (src_idx < src_len));
        }
        if (os_remain)
        {
          continue;
        }
        os_remain = oversampling;

        int32_t noise_filter = self->_cfg.noise_filter_level;
        if (noise_filter)
        {
          value = (value + prev_value[flip] * noise_filter) / (noise_filter + 1);
          prev_value[flip] = value;
          flip = stereo - flip;
        }

        value = value * f_gain;

        if (current_rec->is_16bit)
        {
          if (value < INT16_MIN + 16)
          {
            value = INT16_MIN + 16;
          }
          else if (value > INT16_MAX - 16)
          {
            value = INT16_MAX - 16;
          }
          auto dst = (int16_t *)(current_rec->data);
          *dst++ = value;
          current_rec->data = dst;
        }
        else
        {
          value = ((value + 128) >> 8) + 128;
          if (value < 0)
          {
            value = 0;
          }
          else if (value > 255)
          {
            value = 255;
          }
          auto dst = (uint8_t *)(current_rec->data);
          *dst++ = value;
          current_rec->data = dst;
        }
        value = 0;
        if (--dst_remain == 0)
        {
          current_rec->length = 0;
          break;
        }
      }
    }
    self->_is_recording = false;
    i2s_stop(self->_cfg.i2s_port);

    self->_task_handle = nullptr;
    vTaskDelete(nullptr);
  }

  bool Mic_Class::begin(void)
  {
    if (_task_running)
    {
      if (_rec_sample_rate == _cfg.sample_rate)
      {
        return true;
      }
      do
      {
        vTaskDelay(1);
      } while (isRecording());
      end();
      _rec_sample_rate = _cfg.sample_rate;
    }

    if (_task_semaphore == nullptr)
    {
      _task_semaphore = xSemaphoreCreateBinary();
    }

    bool res = true;
    if (_cb_set_enabled)
    {
      res = _cb_set_enabled(_cb_set_enabled_args, true);
    }

    res = (ESP_OK == _setup_i2s()) && res;
    if (res)
    {
      size_t stack_size = 1024 + (_cfg.dma_buf_len * sizeof(int16_t));
      _task_running = true;
#if portNUM_PROCESSORS > 1
      if (((size_t)_cfg.task_pinned_core) < portNUM_PROCESSORS)
      {
        xTaskCreatePinnedToCore(mic_task, "mic_task", stack_size, this, _cfg.task_priority, &_task_handle, _cfg.task_pinned_core);
      }
      else
#endif
      {
        xTaskCreate(mic_task, "mic_task", stack_size, this, _cfg.task_priority, &_task_handle);
      }
    }

    return res;
  }

  void Mic_Class::end(void)
  {
    if (!_task_running)
    {
      return;
    }
    _task_running = false;
    if (_task_handle)
    {
      if (_task_handle)
      {
        xTaskNotifyGive(_task_handle);
      }
      do
      {
        vTaskDelay(1);
      } while (_task_handle);
    }

    if (_cb_set_enabled)
    {
      _cb_set_enabled(_cb_set_enabled_args, false);
    }
    i2s_driver_uninstall(_cfg.i2s_port);
  }

  bool Mic_Class::_rec_raw(void *recdata, size_t array_len, bool flg_16bit, uint32_t sample_rate)
  {
    recording_info_t info;
    info.data = recdata;
    info.length = array_len;
    info.is_16bit = flg_16bit;

    _cfg.sample_rate = sample_rate;

    if (!begin())
    {
      return false;
    }
    if (array_len == 0)
    {
      return true;
    }
    while (_rec_info[_rec_flip].length)
    {
      xSemaphoreTake(_task_semaphore, 1);
    }
    _rec_info[_rec_flip] = info;
    if (this->_task_handle)
    {
      xTaskNotifyGive(this->_task_handle);
    }
    return true;
  }
}