/*
 * ESP32 Two-Way Intercom System
 * With configurable debug output
 *
 * NOTE: Debug controls are located in intercom/main/include/debu_config.h
 *
 * AUDIO OPTIMIZATION SUMMARY:
 * We achieved significant audio quality improvements through several key optimizations:
 * 
 * 1. Buffer Management:
 * - Reduced DMA frame size from 960 to 480 for lower latency
 * - Increased DMA descriptors from 4 to 8 for better buffering
 * - Increased ESP-NOW packet size from 120 to 240 bytes for more efficient transmission
 * 
 * 2. WiFi/ESP-NOW Configuration:
 * - Explicitly enabled 11b/g/n protocols for better throughput
 * - Set fixed channel and listen interval to reduce RF management overhead
 * - Optimized station mode configuration for ESP-NOW performance
 *
 * 3. Task Prioritization:
 * - Elevated audio task to maximum priority (configMAX_PRIORITIES - 1)
 * - Ensures audio processing gets CPU time when needed
 * 
 * These changes significantly reduced the previous 70ms dropouts that occurred every 700ms
 * by better balancing latency, buffering, and processing priorities.
 */

#include <stdio.h>
#include <math.h>
#include <stdint.h>
#include <string.h>
#include "esp_log.h"
#include "esp_now.h"
#include "esp_wifi.h"
#include "driver/i2s_std.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_check.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "debug_config.h"
#include "optimize.h"

//-----------------------------------------------------------------------------
// Configuration Constants
//-----------------------------------------------------------------------------

// I2S Configuration
#define SAMPLE_RATE         16000    // Sample rate in Hz
#define SAMPLE_BITS         32       // Sample bits
#define I2S_CH             I2S_SLOT_MODE_MONO  // Mono mode

// OPTIMIZATION: Enhanced buffer management for better audio streaming
#define DMA_DESC_NUM       8         // Increased from 4 to 8 descriptors
                                    // More descriptors provide better buffering against jitter
                                    // while keeping memory usage reasonable

#define DMA_FRAME_NUM      480       // Reduced from 960 to 480 frames
                                    // Smaller frames reduce latency while still maintaining
                                    // enough data for smooth playback

// OPTIMIZATION: Larger ESP-NOW packets for more efficient transmission
#define AUDIO_PACKET_SIZE  240       // Increased from 120 to 240 bytes
                                    // Larger packets reduce overhead and improve throughput
                                    // while staying well under ESP-NOW's limit

// GPIO Pin Configuration - Microphone
#define I2S_MIC_BCK_IO     26       // Bit clock pin
#define I2S_MIC_WS_IO      25       // Word select pin
#define I2S_MIC_DI_IO      22       // Data in pin
#define I2S_MIC_DO_IO      -1       // Data out pin (not used)

// GPIO Pin Configuration - DAC
#define I2S_DAC_BCK_IO     27       // Bit clock
#define I2S_DAC_WS_IO      14       // Word select
#define I2S_DAC_DO_IO      12       // Data out
#define I2S_DAC_DI_IO      -1       // Data in (not used)

// GPIO Pin Configuration - Controls
#define PTT_PIN            32       // Push-to-talk button pin
#define LED_PIN            2         // Status LED pin

// WiFi Configuration
#define WIFI_CHANNEL       1
#define MAX_PACKET_SIZE    250

#define MIN(a,b) ((a) < (b) ? (a) : (b))

// Device MAC addresses
const uint8_t MAC_ESP32_1[] = {0xC8, 0x2E, 0x18, 0xC3, 0x95, 0xB0};
const uint8_t MAC_ESP32_2[] = {0xC8, 0x2E, 0x18, 0xC3, 0x96, 0x38};

// VU Meter Configuration
#define VU_FILTER_ALPHA    0.28      // VU meter time constant
#define VU_MIN_DB         -90.0      // Minimum dB value to display
#define VU_SCALE_REF      INT32_MAX  // Full scale reference for dBFS

#if DEBUG_ENABLE
static const char *TAG = "ESP32_INTERCOM";
#endif

//-----------------------------------------------------------------------------
// Global Variables
//-----------------------------------------------------------------------------

// OPTIMIZATION: Handles for I2S channels are kept static and global
// This reduces overhead in passing handles between functions and
// ensures consistent access throughout the application
static i2s_chan_handle_t rx_handle;  // Microphone handle
static i2s_chan_handle_t tx_handle;  // DAC handle
static bool is_transmitting = false;
static bool peer_connected = false;
static bool is_device_1 = false;
static uint8_t peer_mac[ESP_NOW_ETH_ALEN];

//-----------------------------------------------------------------------------
// Type Definitions
//-----------------------------------------------------------------------------

typedef struct {
    bool ptt_active;  // true when transmitting
} ptt_status_t;

typedef struct {
    double filtered_rms;     // Filtered RMS value with VU ballistics
    double dc_offset;        // Current DC offset estimation
    double dc_alpha;         // DC offset filter coefficient
    int32_t min_sample;      // Minimum sample in current buffer
    int32_t max_sample;      // Maximum sample in current buffer
    bool initialized;        // Initialization state flag
} vu_meter_state_t;

// OPTIMIZATION: Audio packet structure optimized for new larger packet size
// The increased size (240 bytes) allows for more efficient transmission
// while maintaining good latency characteristics
typedef struct {
    uint16_t sequence;       // Packet sequence number
    uint16_t size;          // Actual data size
    int32_t samples[AUDIO_PACKET_SIZE / sizeof(int32_t)]; // Audio data
} audio_packet_t;

//-----------------------------------------------------------------------------
// Function Prototypes
//-----------------------------------------------------------------------------

static void vu_meter_init(vu_meter_state_t *state);
static void vu_meter_process(vu_meter_state_t *state, 
                           const int32_t *samples, 
                           size_t sample_count,
                           double *rms_out,
                           double *db_out,
                           int *level_out);
static esp_err_t init_i2s_mic(void);
static esp_err_t init_i2s_dac(void);
static esp_err_t init_espnow(void);
static void ptt_isr_handler(void* arg);
static void esp_now_send_cb(const uint8_t *mac_addr, esp_now_send_status_t status);
static void esp_now_recv_cb(const esp_now_recv_info_t *esp_now_info,
                           const uint8_t *data,
                           int len);
static void print_vu_data(const int32_t* buffer,
                         double dc_offset,
                         double rms,
                         double filtered_rms,
                         double db_value,
                         int level,
                         int32_t min_sample,
                         int32_t max_sample);
static void mic_task(void *arg);

//-----------------------------------------------------------------------------
// Function Implementations
//-----------------------------------------------------------------------------

static void vu_meter_init(vu_meter_state_t *state) {
    state->filtered_rms = 0.0;
    state->dc_offset = 0.0;
    state->dc_alpha = 0.95;  // DC offset filter coefficient
    state->min_sample = 0;
    state->max_sample = 0;
    state->initialized = false;
}

static void vu_meter_process(vu_meter_state_t *state, 
                           const int32_t *samples, 
                           size_t sample_count,
                           double *rms_out,
                           double *db_out,
                           int *level_out) {
    // Calculate DC offset
    double dc_sum = 0;
    int32_t min_sample = INT32_MAX;
    int32_t max_sample = INT32_MIN;
    
    for (size_t i = 0; i < sample_count; i++) {
        dc_sum += samples[i];
        if (samples[i] < min_sample) min_sample = samples[i];
        if (samples[i] > max_sample) max_sample = samples[i];
    }
    
    double current_dc = dc_sum / sample_count;
    
    if (!state->initialized) {
        state->dc_offset = current_dc;
    } else {
        state->dc_offset = (state->dc_alpha * state->dc_offset) + 
                          ((1.0 - state->dc_alpha) * current_dc);
    }

    state->min_sample = min_sample;
    state->max_sample = max_sample;

    // Calculate RMS value with DC offset removal
    double sum_squares = 0;
    for (size_t i = 0; i < sample_count; i++) {
        double centered_sample = samples[i] - state->dc_offset;
        sum_squares += (centered_sample * centered_sample);
    }
    
    double current_rms = sqrt(sum_squares / sample_count);

    // Apply VU meter ballistics
    if (!state->initialized) {
        state->filtered_rms = current_rms;
        state->initialized = true;
    } else {
        state->filtered_rms = (VU_FILTER_ALPHA * current_rms) + 
                             ((1.0 - VU_FILTER_ALPHA) * state->filtered_rms);
    }

    // Calculate dBFS value
    double db_value = 20 * log10(state->filtered_rms / VU_SCALE_REF);
    if (db_value < VU_MIN_DB) db_value = VU_MIN_DB;

    // Calculate VU meter level (0-100)
    int level = (int)((db_value + 90) * 1.25);
    if (level > 100) level = 100;
    if (level < 0) level = 0;

    *rms_out = current_rms;
    *db_out = db_value;
    *level_out = level;
}

static void print_vu_data(const int32_t* buffer,
                         double dc_offset,
                         double rms,
                         double filtered_rms,
                         double db_value,
                         int level,
                         int32_t min_sample,
                         int32_t max_sample) {
    if (!DEBUG_VU_METER) return;  // Skip if VU meter debug is disabled
    
    #if DEBUG_VU_METER
    char bar[21];
    const char* level_desc;
    
    // Create visual VU meter bar
    for (int i = 0; i < 20; i++) {
        bar[i] = (i < level * 20 / 100) ? '#' : ' ';
    }
    bar[20] = '\0';

    // Determine level description based on dBFS ranges
    if (db_value > -10) level_desc = "very loud";
    else if (db_value > -20) level_desc = "loud";
    else if (db_value > -40) level_desc = "conversation";
    else if (db_value > -60) level_desc = "quiet";
    else level_desc = "very quiet";

    // Print formatted output with transmission status
    DEBUG_VU_PRINT("VU: [%s] %.1f dBFS (%s) | DC: %.1f | RMS: %.1f | %s\n",
                   bar, 
                   db_value,
                   level_desc,
                   dc_offset,
                   filtered_rms,
                   is_transmitting ? "TRANSMITTING" : "LISTENING");
    #endif
}

// OPTIMIZATION: I2S initialization with optimized buffer settings
static esp_err_t init_i2s_mic(void) {
    esp_err_t ret = ESP_OK;
    
    // OPTIMIZATION: Increased DMA descriptors (8) and reduced frame size (480)
    // for better balance between latency and buffer protection
    i2s_chan_config_t chan_cfg = {
        .id = I2S_NUM_0,
        .role = I2S_ROLE_MASTER,
        .dma_desc_num = DMA_DESC_NUM,  // Now 8 descriptors
        .dma_frame_num = DMA_FRAME_NUM, // Now 480 frames
        .auto_clear = true
    };
    
    ret = i2s_new_channel(&chan_cfg, NULL, &rx_handle);
    if (ret != ESP_OK) {
        DEBUG_ERROR_PRINT(TAG, "Failed to create I2S RX channel");
        return ret;
    }

    i2s_std_config_t std_cfg = {
        .clk_cfg = {
            .sample_rate_hz = SAMPLE_RATE,
            .clk_src = I2S_CLK_SRC_DEFAULT,
            .mclk_multiple = I2S_MCLK_MULTIPLE_256,
        },
        .slot_cfg = {
            .data_bit_width = SAMPLE_BITS,
            .slot_bit_width = SAMPLE_BITS,
            .slot_mode = I2S_CH,
            .slot_mask = I2S_STD_SLOT_LEFT,
            .ws_width = SAMPLE_BITS,
            .ws_pol = false,
            .bit_shift = true
        },
        .gpio_cfg = {
            .mclk = -1,
            .bclk = I2S_MIC_BCK_IO,
            .ws = I2S_MIC_WS_IO,
            .dout = I2S_MIC_DO_IO,
            .din = I2S_MIC_DI_IO,
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv = false,
            },
        },
    };

    ret = i2s_channel_init_std_mode(rx_handle, &std_cfg);
    if (ret != ESP_OK) {
        DEBUG_ERROR_PRINT(TAG, "Failed to initialize I2S RX channel");
        i2s_del_channel(rx_handle);
        return ret;
    }

    return i2s_channel_enable(rx_handle);
}

// OPTIMIZATION: I2S DAC initialization with same optimized buffer settings as microphone
static esp_err_t init_i2s_dac(void) {
    esp_err_t ret = ESP_OK;
    
    // OPTIMIZATION: Using same optimized DMA settings as microphone channel
    // for consistent buffer handling throughout the audio path
    i2s_chan_config_t chan_cfg = {
        .id = I2S_NUM_1,
        .role = I2S_ROLE_MASTER,
        .dma_desc_num = DMA_DESC_NUM,  // 8 descriptors for better buffering
        .dma_frame_num = DMA_FRAME_NUM, // 480 frames for lower latency
        .auto_clear = true
    };
    
    ret = i2s_new_channel(&chan_cfg, &tx_handle, NULL);
    if (ret != ESP_OK) {
        DEBUG_ERROR_PRINT(TAG, "Failed to create I2S TX channel");
        return ret;
    }

    i2s_std_config_t std_cfg = {
        .clk_cfg = {
            .sample_rate_hz = SAMPLE_RATE,
            .clk_src = I2S_CLK_SRC_DEFAULT,
            .mclk_multiple = I2S_MCLK_MULTIPLE_256,
        },
        .slot_cfg = {
            .data_bit_width = SAMPLE_BITS,
            .slot_bit_width = SAMPLE_BITS,
            .slot_mode = I2S_CH,
            .slot_mask = I2S_STD_SLOT_LEFT,
            .ws_width = SAMPLE_BITS,
            .ws_pol = false,
            .bit_shift = true
        },
        .gpio_cfg = {
            .mclk = -1,
            .bclk = I2S_DAC_BCK_IO,
            .ws = I2S_DAC_WS_IO,
            .dout = I2S_DAC_DO_IO,
            .din = I2S_DAC_DI_IO,
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv = false,
            },
        },
    };

    ret = i2s_channel_init_std_mode(tx_handle, &std_cfg);
    if (ret != ESP_OK) {
        DEBUG_ERROR_PRINT(TAG, "Failed to initialize I2S TX channel");
        i2s_del_channel(tx_handle);
        return ret;
    }

    return i2s_channel_enable(tx_handle);
}

// OPTIMIZATION: Enhanced ESP-NOW initialization with optimized WiFi settings
static esp_err_t init_espnow(void) {
   esp_err_t ret;

   // Initialize NVS
   ret = nvs_flash_init();
   if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
       ESP_ERROR_CHECK(nvs_flash_erase());
       ret = nvs_flash_init();
   }
   ESP_ERROR_CHECK(ret);

   // Create and initialize the event loop
   ESP_ERROR_CHECK(esp_event_loop_create_default());

   // Initialize WiFi in Station mode
   wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
   ret = esp_wifi_init(&cfg);
   if (ret != ESP_OK) return ret;

   ret = esp_wifi_set_mode(WIFI_MODE_STA);
   if (ret != ESP_OK) return ret;

   // OPTIMIZATION: Enable all protocols (b/g/n) for better throughput
   // This allows the ESP32 to use the most efficient protocol available
   ret = esp_wifi_set_protocol(WIFI_IF_STA, 
                              WIFI_PROTOCOL_11B | 
                              WIFI_PROTOCOL_11G | 
                              WIFI_PROTOCOL_11N);
   if (ret != ESP_OK) return ret;

   // OPTIMIZATION: Configure WiFi for minimum latency
   // Fixed channel and shorter listen interval reduce RF management overhead
   wifi_config_t wifi_config = {
       .sta = {
           .channel = WIFI_CHANNEL,
           .listen_interval = 1,
       },
   };
   ret = esp_wifi_set_config(WIFI_IF_STA, &wifi_config);
   if (ret != ESP_OK) return ret;

   ret = esp_wifi_start();
   if (ret != ESP_OK) return ret;

   uint8_t mac[6];
   ret = esp_wifi_get_mac(WIFI_IF_STA, mac);
   if (ret != ESP_OK) return ret;

   DEBUG_INFO_PRINT(TAG, "----------------------------------------");
   DEBUG_INFO_PRINT(TAG, "Local MAC:  %02X:%02X:%02X:%02X:%02X:%02X",
            mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
   
   if (memcmp(mac, MAC_ESP32_1, ESP_NOW_ETH_ALEN) == 0) {
       memcpy(peer_mac, MAC_ESP32_2, ESP_NOW_ETH_ALEN);
       is_device_1 = true;
   } else if (memcmp(mac, MAC_ESP32_2, ESP_NOW_ETH_ALEN) == 0) {
       memcpy(peer_mac, MAC_ESP32_1, ESP_NOW_ETH_ALEN);
       is_device_1 = false;
   } else {
       DEBUG_ERROR_PRINT(TAG, "Unknown device MAC address");
       return ESP_FAIL;
   }
   
   DEBUG_INFO_PRINT(TAG, "Remote MAC: %02X:%02X:%02X:%02X:%02X:%02X",
            peer_mac[0], peer_mac[1], peer_mac[2], peer_mac[3], peer_mac[4], peer_mac[5]);
   DEBUG_INFO_PRINT(TAG, "----------------------------------------");

   ret = esp_now_init();
   if (ret != ESP_OK) return ret;

   esp_now_register_send_cb(esp_now_send_cb);
   esp_now_register_recv_cb(esp_now_recv_cb);

   esp_now_peer_info_t peer_info = {
       .channel = WIFI_CHANNEL,
       .encrypt = false,
   };
   memcpy(peer_info.peer_addr, peer_mac, ESP_NOW_ETH_ALEN);
   ret = esp_now_add_peer(&peer_info);

   if (ret == ESP_OK) {
       peer_connected = true;
   }

   return ret;
}

static void esp_now_send_cb(const uint8_t *mac_addr, esp_now_send_status_t status) {
   if (status != ESP_NOW_SEND_SUCCESS) {
       DEBUG_WARNING_PRINT(TAG, "Failed to send ESP-NOW packet");
   }
}

// OPTIMIZATION: Streamlined receive callback with zero-wait writes
static void esp_now_recv_cb(const esp_now_recv_info_t *esp_now_info,
                          const uint8_t *data,
                          int len) {
   // Check if it's a PTT status message
   if (len == sizeof(ptt_status_t)) {
       ptt_status_t *status = (ptt_status_t*)data;
       gpio_set_level(LED_PIN, status->ptt_active);
       return;
   }

   // Handle audio packet
   if (len == sizeof(audio_packet_t) && !is_transmitting) {
       audio_packet_t *packet = (audio_packet_t*)data;
       size_t bytes_written = 0;
       
       // OPTIMIZATION: Using no-wait write (timeout = 0) to prevent blocking
       // This helps maintain real-time performance
       esp_err_t ret = i2s_channel_write(tx_handle, packet->samples,
                                       packet->size,
                                       &bytes_written,
                                       0);  // No wait
       if (ret != ESP_OK) {
           DEBUG_WARNING_PRINT(TAG, "Failed to write to DAC: %s", esp_err_to_name(ret));
       }
   }
}

static void IRAM_ATTR ptt_isr_handler(void* arg) {
   bool ptt_pressed = !gpio_get_level(PTT_PIN);
   is_transmitting = ptt_pressed;
   gpio_set_level(LED_PIN, ptt_pressed);

   static ptt_status_t status;
   status.ptt_active = ptt_pressed;
   esp_now_send(peer_mac, (uint8_t*)&status, sizeof(ptt_status_t));
}

// OPTIMIZATION: Enhanced mic_task with optimized buffer handling
static void mic_task(void *arg) {
   // OPTIMIZATION: Using DMA-capable memory for audio buffer
   int32_t *buffer = heap_caps_malloc(DMA_FRAME_NUM * sizeof(int32_t), MALLOC_CAP_DMA);
   if (buffer == NULL) {
       DEBUG_ERROR_PRINT(TAG, "Failed to allocate buffer");
       vTaskDelete(NULL);
       return;
   }

   vu_meter_state_t vu_state;
   vu_meter_init(&vu_state);
   size_t bytes_read = 0;
   uint16_t sequence = 0;
   
   // Warm-up period
   DEBUG_INFO_PRINT(TAG, "Starting warm-up...");
   for (int i = 0; i < 10; i++) {
       esp_err_t ret = i2s_channel_read(rx_handle, buffer,
                                      DMA_FRAME_NUM * sizeof(int32_t),
                                      &bytes_read, portMAX_DELAY);
       if (ret == ESP_OK) {
           vTaskDelay(pdMS_TO_TICKS(10));
       }
   }
   DEBUG_INFO_PRINT(TAG, "Warm-up complete");
   
   while (1) {
       esp_err_t ret = i2s_channel_read(rx_handle, buffer,
                                      DMA_FRAME_NUM * sizeof(int32_t),
                                      &bytes_read, portMAX_DELAY);
       if (ret != ESP_OK) {
           DEBUG_ERROR_PRINT(TAG, "Error reading I2S data: %s", esp_err_to_name(ret));
           continue;
       }

       int samples = bytes_read / sizeof(int32_t);
       if (samples == 0) continue;

       // Process samples for VU meter
       double rms, db_value;
       int level;
       vu_meter_process(&vu_state, buffer, samples, &rms, &db_value, &level);

       // OPTIMIZATION: Efficient audio transmission with larger packet size
       if (is_transmitting && peer_connected) {
           // Send audio data in optimized fragment size
           for (int offset = 0; offset < samples; offset += AUDIO_PACKET_SIZE / sizeof(int32_t)) {
               audio_packet_t packet = {
                   .sequence = sequence++,
                   .size = MIN(AUDIO_PACKET_SIZE, (samples - offset) * sizeof(int32_t))
               };
               
               memcpy(packet.samples, &buffer[offset], packet.size);
               
               esp_err_t ret = esp_now_send(peer_mac, (uint8_t*)&packet, sizeof(packet));
               if (ret != ESP_OK) {
                   DEBUG_WARNING_PRINT(TAG, "Failed to send ESP-NOW packet: %s", esp_err_to_name(ret));
               }
           }
       }

       // Display VU meter
       print_vu_data(buffer,
                    vu_state.dc_offset,
                    rms,
                    vu_state.filtered_rms,
                    db_value,
                    level,
                    vu_state.min_sample,
                    vu_state.max_sample);
   }

   heap_caps_free(buffer);
   vTaskDelete(NULL);
}

// OPTIMIZATION: Enhanced app_main with optimized initialization sequence
void app_main(void) {
   DEBUG_INFO_PRINT(TAG, "Starting ESP32 Intercom...");
   print_debug_config();

   // Initialize GPIO for PTT button and LED
   gpio_config_t io_conf = {
       .pin_bit_mask = (1ULL << PTT_PIN),
       .mode = GPIO_MODE_INPUT,
       .pull_up_en = GPIO_PULLUP_ENABLE,
       .intr_type = GPIO_INTR_ANYEDGE,
   };
   gpio_config(&io_conf);
   
   io_conf.pin_bit_mask = (1ULL << LED_PIN);
   io_conf.mode = GPIO_MODE_OUTPUT;
   io_conf.intr_type = GPIO_INTR_DISABLE;
   gpio_config(&io_conf);

   // Install GPIO ISR service and add PTT handler
   gpio_install_isr_service(0);
   gpio_isr_handler_add(PTT_PIN, ptt_isr_handler, NULL);

   // Initialize ESP-NOW with optimized WiFi settings
   esp_err_t ret = init_espnow();
   if (ret != ESP_OK) {
       DEBUG_ERROR_PRINT(TAG, "Failed to initialize ESP-NOW: %s", esp_err_to_name(ret));
       return;
   }
   
   // OPTIMIZATION: Apply system optimizations after WiFi init but before I2S
   // This ensures WiFi and system settings are properly configured
   optimizeESP32ForAudio();
   
   // Initialize I2S interfaces with optimized buffer settings
   ESP_ERROR_CHECK(init_i2s_mic());  // Microphone
   ESP_ERROR_CHECK(init_i2s_dac());  // DAC

   // OPTIMIZATION: Create mic task with maximum priority
   // This ensures audio processing gets CPU time when needed
   BaseType_t task_created = xTaskCreate(mic_task,
                                       "mic_task",
                                       4096,
                                       NULL,
                                       configMAX_PRIORITIES - 1,  // Maximum priority for audio task
                                       NULL);
                                       
   if (task_created != pdPASS) {
       DEBUG_ERROR_PRINT(TAG, "Failed to create mic task");
       // Cleanup
       i2s_channel_disable(rx_handle);
       i2s_del_channel(rx_handle);
       i2s_channel_disable(tx_handle);
       i2s_del_channel(tx_handle);
       return;
   }

   DEBUG_INFO_PRINT(TAG, "Intercom initialized successfully");
}

