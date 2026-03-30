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

// Local uptime (ms) at last successful packet receive — defined here, extern'd in header
volatile uint32_t g_remote_rx_ms = 0;
// Local uptime (ms) of last remote button press — latches so display can't miss a quick press
volatile uint32_t g_remote_btn_ms = 0;
// RSSI (dBm) of the most recently received packet — used by distance_estimator
volatile int8_t g_remote_rssi = -100;

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

    // Record OUR local time of receipt — used for link staleness (not remote clock)
    g_remote_rx_ms  = (uint32_t)(esp_timer_get_time() / 1000ULL);
    g_remote_rssi   = (int8_t)recv_info->rx_ctrl->rssi;

    // Drive RX LED directly — high if remote button pressed, brief pulse otherwise
    gpio_set_level(HAL_LED_RX_PIN, pkt->button_state ? 1 : 0);

    // Latch button-press timestamp so display task can't miss a quick press
    if (pkt->button_state) {
        g_remote_btn_ms = (uint32_t)(esp_timer_get_time() / 1000ULL);
    }

    if (pkt->button_state || pkt->seq % 25 == 0) {
        ESP_LOGI(TAG, "RX node=%d seq=%lu btn=%d",
                 pkt->node_id, (unsigned long)pkt->seq, pkt->button_state);
    }
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
    uint8_t  prev_btn     = 0;
    uint32_t last_send_ms = 0;
    uint32_t led_off_ms   = 0;

    while (1) {
        uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000ULL);
        uint8_t  btn    = (uint8_t)(!gpio_get_level(HAL_BUTTON_PIN)); // active low

        // Turn TX LED off 50 ms after last send
        if (led_off_ms && now_ms >= led_off_ms) {
            gpio_set_level(HAL_LED_TX_PIN, 0);
            led_off_ms = 0;
        }

        // Send immediately on button edge (press or release), or every 200 ms heartbeat
        bool btn_edge   = (btn != prev_btn);
        bool time_to_tx = (now_ms - last_send_ms) >= 200;

        if (btn_edge || time_to_tx) {
            uint32_t seq = 0;
            if (xSemaphoreTake(g_telem_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                g_local.node_id      = NODE_ID;
                g_local.timestamp_ms = now_ms;
                g_local.button_state = btn;
                g_local.seq++;
                seq = g_local.seq;
                esp_now_send(k_broadcast_mac, (uint8_t *)&g_local, sizeof(telemetry_packet_t));
                xSemaphoreGive(g_telem_mutex);
            }
            last_send_ms = now_ms;
            led_off_ms   = now_ms + 50;

            if (btn_edge) {
                ESP_LOGI(TAG, "TX seq=%lu btn=%s", (unsigned long)seq, btn ? "PRESS" : "release");
            } else if (seq % 25 == 0) {
                ESP_LOGI(TAG, "TX seq=%lu btn=%d", (unsigned long)seq, btn);
            }
        }

        prev_btn = btn;
        vTaskDelay(pdMS_TO_TICKS(20)); // 20 ms poll — detects press within one cycle
    }
}
