/* Play mp3 file by audio pipeline
   with possibility to start, stop, pause and resume playback
   as well as adjust volume

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

/**
 * [ build that will not crash ]
 *
 *
 */

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "audio_element.h"
#include "audio_pipeline.h"
#include "audio_event_iface.h"
#include "audio_mem.h"
#include "audio_common.h"
#include "i2s_stream.h"
#include "mp3_decoder.h"
#include "esp_peripherals.h"
#include "periph_touch.h"
#include "periph_adc_button.h"
#include "periph_button.h"
#include "board.h"

#include "nvs_flash.h"
#include "esp_vfs_fat.h"
#include "esp_flash_partitions.h"

static const char *TAG = "PLAY_FLASH_MP3_CONTROL";

#define WFM1_STRINGIFY(x) WFM1_STRINGIFY2(x)
#define WFM1_STRINGIFY2(x) #x

static const char *FATFS_PARTITION = "storage";
static const char *FATFS_MOUNT_DIR = "/storage";
static const char *TEST_FILE = "/storage/test.txt";
static const char *TEST_DATA = "hello world";
static const char *TASK_NAME = "FATFS";
#define TASK_STACK_SIZE 2048
#define TASK_PRIORITY 4
#define MP3_DECODER_CORE 0

static struct marker {
    int pos;
    const uint8_t *start;
    const uint8_t *end;
} file_marker;

// low rate mp3 audio
extern const uint8_t lr_mp3_start[] asm("_binary_music_16b_2c_8000hz_mp3_start");
extern const uint8_t lr_mp3_end[] asm("_binary_music_16b_2c_8000hz_mp3_end");

// medium rate mp3 audio
extern const uint8_t mr_mp3_start[] asm("_binary_music_16b_2c_22050hz_mp3_start");
extern const uint8_t mr_mp3_end[] asm("_binary_music_16b_2c_22050hz_mp3_end");

// high rate mp3 audio
extern const uint8_t hr_mp3_start[] asm("_binary_music_16b_2c_44100hz_mp3_start");
extern const uint8_t hr_mp3_end[] asm("_binary_music_16b_2c_44100hz_mp3_end");

static void set_next_file_marker() {
    static int idx = 0;

    switch (idx) {
    case 0:
        file_marker.start = lr_mp3_start;
        file_marker.end = lr_mp3_end;
        break;
    case 1:
        file_marker.start = mr_mp3_start;
        file_marker.end = mr_mp3_end;
        break;
    case 2:
        file_marker.start = hr_mp3_start;
        file_marker.end = hr_mp3_end;
        break;
    default:
        ESP_LOGE(TAG, "[ * ] Not supported index = %d", idx);
    }
    if (++idx > 2) {
        idx = 0;
    }
    file_marker.pos = 0;
}

int mp3_music_read_cb(audio_element_handle_t el, char *buf, int len, TickType_t wait_time, void *ctx) {
    int read_size = file_marker.end - file_marker.start - file_marker.pos;
    if (read_size == 0) {
        return AEL_IO_DONE;
    } else if (len < read_size) {
        read_size = len;
    }
    memcpy(buf, file_marker.start + file_marker.pos, read_size);
    file_marker.pos += read_size;
    return read_size;
}

/**
 * @brief Print macros from menuconfig
 */
void printMacro(const char *s1, const char *s2) {
    bool isSame = (s2 && !strcmp(s1, s2));
    ESP_LOGI(TAG, ">>> %-50s %s", s1, isSame ? "(not defined)" : s2);
}

/**
 * @brief Print app configs
 */
void printConfig(void) {
    ESP_LOGI(TAG, "=================================================");
    ESP_LOGI(TAG, ">>> main task priority=%d", uxTaskPriorityGet(NULL));
    printMacro("CONFIG_ESP32_REV_MIN", WFM1_STRINGIFY(CONFIG_ESP32_REV_MIN));
    printMacro("CONFIG_ESP32_SPIRAM_SUPPORT", WFM1_STRINGIFY(CONFIG_ESP32_SPIRAM_SUPPORT));
    printMacro("CONFIG_SPIRAM", WFM1_STRINGIFY(CONFIG_SPIRAM));
    printMacro("CONFIG_SPIRAM_BOOT_INIT", WFM1_STRINGIFY(CONFIG_SPIRAM_BOOT_INIT));
    printMacro("CONFIG_SPIRAM_USE_MALLOC", WFM1_STRINGIFY(CONFIG_SPIRAM_USE_MALLOC));
    printMacro("CONFIG_SPIRAM_MALLOC_ALWAYSINTERNAL", WFM1_STRINGIFY(CONFIG_SPIRAM_MALLOC_ALWAYSINTERNAL));
    printMacro("CONFIG_SPIRAM_MALLOC_RESERVE_INTERNAL", WFM1_STRINGIFY(CONFIG_SPIRAM_MALLOC_RESERVE_INTERNAL));
    printMacro("CONFIG_SPIRAM_ALLOW_BSS_SEG_EXTERNAL_MEMORY", WFM1_STRINGIFY(CONFIG_SPIRAM_ALLOW_BSS_SEG_EXTERNAL_MEMORY));
    printMacro("CONFIG_SPIRAM_ALLOW_STACK_EXTERNAL_MEMORY", WFM1_STRINGIFY(CONFIG_SPIRAM_ALLOW_STACK_EXTERNAL_MEMORY));
    ESP_LOGI(TAG, "=================================================");
}

/**
 * @brief forever loop, delay 1000ms for each iteration
 */
void foreverLoop() {
    while (1) {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

/**
 * @brief init NVS
 */
void init_nvs() {
    // init nvs
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);
    ESP_LOGI(TAG, ">>> init nvs OK");
}

/**
 * @brief Check if FAT FS is corrupted?
 * - The checking is performed by first writing a test file and the read the file to compare the content.
 * - Data read from the test file should be matched with the data written to the test file.
 * - If there is any error writing and reading, or data is NOT matched, FAT FS is most likely corrupted
 * @return 1=OK, 0=FAT FS may be corrupted
 */
uint8_t isFATFSCorrupted() {
    static uint16_t nTest = 0;
    uint8_t isTestPass = 0;
    FILE *fp = NULL;

    // break on any failed conditions
    do {
        // write test data
        ESP_LOGI(TAG, "before open for write %s", TEST_FILE);
        if (!(fp = fopen(TEST_FILE, "wb"))) {
            ESP_LOGE(TAG, "failed to create file %s", TEST_FILE);
            break;
        }
        ESP_LOGI(TAG, "before write %s", TEST_FILE);
        fwrite(TEST_DATA, 1, strlen(TEST_DATA), fp);
        fclose(fp);

        // read test data
        ESP_LOGI(TAG, "before open for read %s", TEST_FILE);
        if (!(fp = fopen(TEST_FILE, "r"))) {
            ESP_LOGE(TAG, "failed to open file %s", TEST_FILE);
            break;
        }
        char buffer[strlen(TEST_DATA) + 1];
        ESP_LOGI(TAG, "before read %s", TEST_FILE);
        uint16_t nRead = fread(buffer, 1, strlen(TEST_DATA), fp);
        fclose(fp);

        // compare data
        ESP_LOGI(TAG, "before compare");
        if (nRead == strlen(TEST_DATA) && !strncmp(TEST_DATA, (const char *)buffer, strlen(TEST_DATA))) {
            isTestPass = 1;
            nTest += 1;
        }
        break;
    } while (0); // loop only once

    if (isTestPass) {
        ESP_LOGI(TAG, ">>> [%3d] FAT FS check passed", nTest);
    } else {
        ESP_LOGE(TAG, ">>> FAT FS test failed");
    }
    ESP_LOGI(TAG, "before return ");
    return isTestPass;
}

/**
 * @brief Mount FAT FS
 * @return wl_handle_t
 */
wl_handle_t mountFATFS() {
    // init FAT FS
    esp_vfs_fat_mount_config_t mountConfig = {};
    mountConfig.max_files = 4,
    mountConfig.format_if_mount_failed = true,
    mountConfig.allocation_unit_size = CONFIG_WL_SECTOR_SIZE; // 4096

    // partition name is "storage"
    wl_handle_t wearCtx = WL_INVALID_HANDLE;
    esp_err_t err = esp_vfs_fat_spiflash_mount(FATFS_MOUNT_DIR, FATFS_PARTITION, &mountConfig, &wearCtx);
    if (err != ESP_OK)
        ESP_LOGE(TAG, "failed to mount FATFS");
    ESP_ERROR_CHECK(err);
    ESP_LOGI(TAG, ">>> mounted FAT FS");
    return wearCtx;
}

/**
 * @brief Unmount FAT FS
 */
void unmountFATFS(wl_handle_t wearCtx) {
    esp_err_t err = esp_vfs_fat_spiflash_unmount(FATFS_MOUNT_DIR, wearCtx);
    if (err == ESP_ERR_INVALID_STATE)
        ESP_LOGE(TAG, "failed to unmount FATFS (ESP_ERR_INVALID_STATE), partition=%s", FATFS_PARTITION);
    if (err != ESP_OK)
        ESP_LOGE(TAG, "failed to unmount FATFS, err=%s", esp_err_to_name(err));
    ESP_ERROR_CHECK(err);
    ESP_LOGI(TAG, ">>> un-mounted FAT FS");
}

void eraseFATPartition() {
    // find the storage partition
    esp_partition_iterator_t it;
    if (!(it = esp_partition_find(ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_DATA_FAT, FATFS_PARTITION))) {
        ESP_LOGE(TAG, ">>> partition not found, partition=%s", FATFS_PARTITION);
        ESP_ERROR_CHECK(-1);
    }
    const esp_partition_t *FATPart = esp_partition_get(it);
    esp_partition_iterator_release(it);

    // erase the partition
    esp_err_t err;
    ESP_LOGI(TAG, "Erasing partition=%s....", FATFS_PARTITION);
    if ((err = esp_partition_erase_range(FATPart, 0, FATPart->size))) {
        ESP_LOGE(TAG, "esp_partition_erase_range() failed, err=%s", esp_err_to_name(err));
        ESP_ERROR_CHECK(-1);
    }
    ESP_LOGI(TAG, "Erase partition OK, partition=%s", FATFS_PARTITION);
}

/**
 * @brief Init and mount FAT FS.
 * Before test, partition is erased.  This makes sure partition is clean and formatted before mount.
 */
void init_fatfs() {
    // erase partition
    eraseFATPartition();

    // mount the FAT FS
    wl_handle_t wearCtx = mountFATFS();
}

/**
 * @brief Task worker function to check FAT FS periodically
 */
void worker(void *ctx) {
    // wait for 5 sec before running FAT FS check
    ESP_LOGI(TAG, ">>> worker waiting to start..., priority=%d", uxTaskPriorityGet(NULL));
    vTaskDelay(5000 / portTICK_PERIOD_MS);
    const uint16_t delayMs = 2000;
    ESP_LOGI(TAG, ">>> worker started");
    while (1) {
        if (!isFATFSCorrupted()) {
            ESP_LOGE(TAG, ">>> stopped checking FAT FS");
            foreverLoop();
        }
        vTaskDelay(delayMs / portTICK_PERIOD_MS);
    }
}

/**
 * @brief A task that perform FAT check regularly.
 * The purpose is to check if file read/write can be performed when there are I2S communication
 */
void createTaskCheckFATFS() {
    TaskHandle_t taskCtx;
    const uint8_t cpuId = 0;
    if (xTaskCreatePinnedToCore(worker, TASK_NAME, TASK_STACK_SIZE, NULL, TASK_PRIORITY, &taskCtx, cpuId) != pdPASS) {
        ESP_LOGE(TAG, ">>> failed creating task");
        foreverLoop();
    }
}

void app_main(void) {
    audio_pipeline_handle_t pipeline;
    audio_element_handle_t i2s_stream_writer, mp3_decoder;

    printConfig();
    init_nvs();
    init_fatfs();
    createTaskCheckFATFS();

    esp_log_level_set("*", ESP_LOG_WARN);
    esp_log_level_set(TAG, ESP_LOG_INFO);

    ESP_LOGI(TAG, "[ 0 ] program started");

    ESP_LOGI(TAG, "[ 1 ] Start audio codec chip");
    audio_board_handle_t board_handle = audio_board_init();
    audio_hal_ctrl_codec(board_handle->audio_hal, AUDIO_HAL_CODEC_MODE_BOTH, AUDIO_HAL_CTRL_START);

    int player_volume;
    audio_hal_get_volume(board_handle->audio_hal, &player_volume);

    ESP_LOGI(TAG, "[ 2 ] Create audio pipeline, add all elements to pipeline, and subscribe pipeline event");
    audio_pipeline_cfg_t pipeline_cfg = DEFAULT_AUDIO_PIPELINE_CONFIG();
    pipeline = audio_pipeline_init(&pipeline_cfg);
    mem_assert(pipeline);

    ESP_LOGI(TAG, "[2.1] Create mp3 decoder to decode mp3 file and set custom read callback");
    mp3_decoder_cfg_t mp3_cfg = DEFAULT_MP3_DECODER_CONFIG();
    mp3_cfg.task_core = MP3_DECODER_CORE;
    mp3_decoder = mp3_decoder_init(&mp3_cfg);
    audio_element_set_read_cb(mp3_decoder, mp3_music_read_cb, NULL);

    ESP_LOGI(TAG, "[2.2] Create i2s stream to write data to codec chip");
    i2s_stream_cfg_t i2s_cfg = I2S_STREAM_CFG_DEFAULT();
    i2s_cfg.type = AUDIO_STREAM_WRITER;
    i2s_stream_writer = i2s_stream_init(&i2s_cfg);

    ESP_LOGI(TAG, "[2.3] Register all elements to audio pipeline");
    audio_pipeline_register(pipeline, mp3_decoder, "mp3");
    audio_pipeline_register(pipeline, i2s_stream_writer, "i2s");

    ESP_LOGI(TAG, "[2.4] Link it together [mp3_music_read_cb]-->mp3_decoder-->i2s_stream-->[codec_chip]");
    const char *link_tag[2] = {"mp3", "i2s"};
    audio_pipeline_link(pipeline, &link_tag[0], 2);

    ESP_LOGI(TAG, "[ 3 ] Initialize peripherals");
    esp_periph_config_t periph_cfg = DEFAULT_ESP_PERIPH_SET_CONFIG();
    esp_periph_set_handle_t set = esp_periph_set_init(&periph_cfg);

    ESP_LOGI(TAG, "[3.1] Initialize keys on board");
    audio_board_key_init(set);

    ESP_LOGI(TAG, "[ 4 ] Set up  event listener");
    audio_event_iface_cfg_t evt_cfg = AUDIO_EVENT_IFACE_DEFAULT_CFG();
    audio_event_iface_handle_t evt = audio_event_iface_init(&evt_cfg);

    ESP_LOGI(TAG, "[4.1] Listening event from all elements of pipeline");
    audio_pipeline_set_listener(pipeline, evt);

    ESP_LOGI(TAG, "[4.2] Listening event from peripherals");
    audio_event_iface_set_listener(esp_periph_set_get_event_iface(set), evt);

    ESP_LOGW(TAG, "[ 5 ] Tap touch buttons to control music player:");
    ESP_LOGW(TAG, "      [Play] to start, pause and resume, [Set] to stop.");
    ESP_LOGW(TAG, "      [Vol-] or [Vol+] to adjust volume.");

    ESP_LOGI(TAG, "[ 5.1 ] Start audio_pipeline");
    set_next_file_marker();
    audio_pipeline_run(pipeline);

    while (1) {
        audio_event_iface_msg_t msg;
        esp_err_t ret = audio_event_iface_listen(evt, &msg, portMAX_DELAY);
        if (ret != ESP_OK) {
            continue;
        }

        if (msg.source_type == AUDIO_ELEMENT_TYPE_ELEMENT && msg.source == (void *)mp3_decoder && msg.cmd == AEL_MSG_CMD_REPORT_MUSIC_INFO) {
            audio_element_info_t music_info = {0};
            audio_element_getinfo(mp3_decoder, &music_info);
            ESP_LOGI(TAG, "[ * ] Receive music info from mp3 decoder, sample_rates=%d, bits=%d, ch=%d",
                     music_info.sample_rates, music_info.bits, music_info.channels);
            audio_element_setinfo(i2s_stream_writer, &music_info);
            i2s_stream_set_clk(i2s_stream_writer, music_info.sample_rates, music_info.bits, music_info.channels);
            continue;
        }

        if ((msg.source_type == PERIPH_ID_TOUCH || msg.source_type == PERIPH_ID_BUTTON || msg.source_type == PERIPH_ID_ADC_BTN) && (msg.cmd == PERIPH_TOUCH_TAP || msg.cmd == PERIPH_BUTTON_PRESSED || msg.cmd == PERIPH_ADC_BUTTON_PRESSED)) {
            if ((int)msg.data == get_input_play_id()) {
                ESP_LOGI(TAG, "[ * ] [Play] touch tap event");
                audio_element_state_t el_state = audio_element_get_state(i2s_stream_writer);
                switch (el_state) {
                case AEL_STATE_INIT:
                    ESP_LOGI(TAG, "[ * ] Starting audio pipeline");
                    audio_pipeline_run(pipeline);
                    break;
                case AEL_STATE_RUNNING:
                    ESP_LOGI(TAG, "[ * ] Pausing audio pipeline");
                    audio_pipeline_pause(pipeline);
                    break;
                case AEL_STATE_PAUSED:
                    ESP_LOGI(TAG, "[ * ] Resuming audio pipeline");
                    audio_pipeline_resume(pipeline);
                    break;
                case AEL_STATE_FINISHED:
                    ESP_LOGI(TAG, "[ * ] Rewinding audio pipeline");
                    audio_pipeline_reset_ringbuffer(pipeline);
                    audio_pipeline_reset_elements(pipeline);
                    audio_pipeline_change_state(pipeline, AEL_STATE_INIT);
                    set_next_file_marker();
                    audio_pipeline_run(pipeline);
                    break;
                default:
                    ESP_LOGI(TAG, "[ * ] Not supported state %d", el_state);
                }
            } else if ((int)msg.data == get_input_set_id()) {
                ESP_LOGI(TAG, "[ * ] [Set] touch tap event");
                ESP_LOGI(TAG, "[ * ] Stopping audio pipeline");
                break;
            } else if ((int)msg.data == get_input_mode_id()) {
                ESP_LOGI(TAG, "[ * ] [mode] tap event");
                audio_pipeline_stop(pipeline);
                audio_pipeline_wait_for_stop(pipeline);
                audio_pipeline_terminate(pipeline);
                audio_pipeline_reset_ringbuffer(pipeline);
                audio_pipeline_reset_elements(pipeline);
                set_next_file_marker();
                audio_pipeline_run(pipeline);
            } else if ((int)msg.data == get_input_volup_id()) {
                ESP_LOGI(TAG, "[ * ] [Vol+] touch tap event");
                player_volume += 10;
                if (player_volume > 100) {
                    player_volume = 100;
                }
                audio_hal_set_volume(board_handle->audio_hal, player_volume);
                ESP_LOGI(TAG, "[ * ] Volume set to %d %%", player_volume);
            } else if ((int)msg.data == get_input_voldown_id()) {
                ESP_LOGI(TAG, "[ * ] [Vol-] touch tap event");
                player_volume -= 10;
                if (player_volume < 0) {
                    player_volume = 0;
                }
                audio_hal_set_volume(board_handle->audio_hal, player_volume);
                ESP_LOGI(TAG, "[ * ] Volume set to %d %%", player_volume);
            }
        }
    }

    ESP_LOGI(TAG, "[ 6 ] Stop audio_pipeline");
    audio_pipeline_stop(pipeline);
    audio_pipeline_wait_for_stop(pipeline);
    audio_pipeline_terminate(pipeline);
    audio_pipeline_unregister(pipeline, mp3_decoder);
    audio_pipeline_unregister(pipeline, i2s_stream_writer);

    /* Terminate the pipeline before removing the listener */
    audio_pipeline_remove_listener(pipeline);

    /* Make sure audio_pipeline_remove_listener is called before destroying event_iface */
    audio_event_iface_destroy(evt);

    /* Release all resources */
    audio_pipeline_deinit(pipeline);
    audio_element_deinit(i2s_stream_writer);
    audio_element_deinit(mp3_decoder);
}
