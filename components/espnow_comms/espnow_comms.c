#include "espnow_comms.h"
#include "board_config.h"

#include "esp_now.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "string.h"

static const char *TAG = "espnow";

#define ESPNOW_CHANNEL   1

// Broadcast MAC — all ESP-NOW peers receive
static const uint8_t k_broadcast_mac[ESP_NOW_ETH_ALEN] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};

// Called from WiFi task — no blocking calls allowed, just set GPIO
static void send_cb(const esp_now_send_info_t *tx_info, esp_now_send_status_t status)
{
    (void)tx_info;
    if (status == ESP_NOW_SEND_SUCCESS) {
        gpio_set_level(HAL_LED_TX_PIN, 1);
    } else {
        gpio_set_level(HAL_LED_TX_PIN, 0);
    }
}

// Called from WiFi task — no blocking calls allowed, just set GPIO + copy data
static void recv_cb(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len)
{
    (void)recv_info;
    if (len != sizeof(telemetry_packet_t)) {
        return;
    }

    const telemetry_packet_t *pkt = (const telemetry_packet_t *)data;

    // Safe to call from WiFi task — xSemaphoreTakeFromISR is not needed here
    // since recv_cb runs at task level (not true ISR), but keep timeout minimal
    if (xSemaphoreTake(g_telem_mutex, 0) == pdTRUE) {
        memcpy(&g_remote, data, sizeof(telemetry_packet_t));
        xSemaphoreGive(g_telem_mutex);
    }

    // Drive RX LED directly — high if remote button pressed, brief pulse otherwise
    gpio_set_level(HAL_LED_RX_PIN, pkt->button_state ? 1 : 0);

    ESP_LOGI(TAG, "RX node=%d seq=%lu btn=%d",
             pkt->node_id, (unsigned long)pkt->seq, pkt->button_state);
}

void espnow_comms_init(void)
{
    // WiFi — STA mode required for ESP-NOW
    wifi_init_config_t wifi_init = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&wifi_init));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());

    // Fix channel — both boards must be on the same channel
    ESP_ERROR_CHECK(esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE));

    // ESP-NOW
    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_send_cb(send_cb));
    ESP_ERROR_CHECK(esp_now_register_recv_cb(recv_cb));

    // Register broadcast peer on the fixed channel
    esp_now_peer_info_t peer = {
        .channel = ESPNOW_CHANNEL,
        .ifidx   = WIFI_IF_STA,
        .encrypt = false,
    };
    memcpy(peer.peer_addr, k_broadcast_mac, ESP_NOW_ETH_ALEN);
    ESP_ERROR_CHECK(esp_now_add_peer(&peer));

    ESP_LOGI(TAG, "ESP-NOW initialised (ch=%d, node_id=%d)", ESPNOW_CHANNEL, NODE_ID);
}

void espnow_tx_task(void *pvParameters)
{
    while (1) {
        if (xSemaphoreTake(g_telem_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            g_local.node_id      = NODE_ID;
            g_local.timestamp_ms = (uint32_t)(esp_timer_get_time() / 1000ULL);
            g_local.button_state = (uint8_t)(!gpio_get_level(HAL_BUTTON_PIN)); // active low
            g_local.seq++;

            esp_now_send(k_broadcast_mac, (uint8_t *)&g_local, sizeof(telemetry_packet_t));

            // Turn TX LED off after brief on from send_cb
            vTaskDelay(pdMS_TO_TICKS(50));
            gpio_set_level(HAL_LED_TX_PIN, 0);

            if (g_local.seq % 25 == 0) {
                ESP_LOGI(TAG, "TX seq=%lu btn=%d", (unsigned long)g_local.seq, g_local.button_state);
            } else if (g_local.button_state) {
                ESP_LOGI(TAG, "TX seq=%lu BTN PRESSED", (unsigned long)g_local.seq);
            }

            xSemaphoreGive(g_telem_mutex);
        }
        vTaskDelay(pdMS_TO_TICKS(150)); // 50ms LED on + 150ms = 200ms total cycle
    }
}
