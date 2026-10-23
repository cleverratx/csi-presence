// Передатчик, плата номер 1.
//
// Задача у нее одна: 100 раз в секунду кидать в эфир маленький пакет.
// Сам пакет никому не интересен, внутри просто счетчик. Важно другое:
// пока пакет летит через комнату, он отражается от стен, мебели и людей.
// Приемник смотрит, как именно сигнал исказился, и по этому понимает,
// что в комнате что-то поменялось.

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "esp_now.h"
#include "esp_idf_version.h"

// В ESP-IDF 6 переименовали WIFI_BW_HT20 в WIFI_BW20.
// Чтоб код собирался и на старых версиях, выбираем имя тут.
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(6, 0, 0)
#define BW_20MHZ WIFI_BW20
#else
#define BW_20MHZ WIFI_BW_HT20
#endif

// Канал должен быть одинаковый на обеих платах.
// Если домашний роутер тоже сидит на 11, поставь 1 или 6, там будет тише.
#define WIFI_CHANNEL   11
#define SEND_HZ        100   // сколько пакетов в секунду

// MAC передатчика задаем руками. По нему приемник отличает наши пакеты
// от всего остального, что летает в эфире.
static const uint8_t TX_MAC[6] = {0x1a, 0x00, 0x00, 0x00, 0x00, 0x00};
static const uint8_t BROADCAST[6] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff};
static const char *TAG = "csi_tx";

static void wifi_init(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_bandwidth(WIFI_IF_STA, BW_20MHZ));
    ESP_ERROR_CHECK(esp_wifi_start());

    // Энергосбережение выключаем, иначе радио иногда засыпает
    // и пакеты идут рваным потоком.
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
    ESP_ERROR_CHECK(esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE));
    ESP_ERROR_CHECK(esp_wifi_set_mac(WIFI_IF_STA, TX_MAC));
}

static void espnow_init(void)
{
    ESP_ERROR_CHECK(esp_now_init());
    esp_now_peer_info_t peer = {0};
    memcpy(peer.peer_addr, BROADCAST, 6);
    peer.channel = WIFI_CHANNEL;
    peer.ifidx = WIFI_IF_STA;
    peer.encrypt = false;
    ESP_ERROR_CHECK(esp_now_add_peer(&peer));

    // Тут главная ловушка. По умолчанию ESP-NOW шлет пакеты на 1 Мбит/с,
    // это старый 802.11b. Для таких пакетов чип CSI вообще не считает,
    // и приемник будет молчать. Поэтому переключаем на MCS0, это уже OFDM,
    // и тогда CSI приходит на каждый пакет.
    esp_now_rate_config_t rate = {
        .phymode = WIFI_PHY_MODE_HT20,
        .rate = WIFI_PHY_RATE_MCS0_LGI,
        .ersu = false,
        .dcm = false,
    };
    ESP_ERROR_CHECK(esp_now_set_peer_rate_config(BROADCAST, &rate));
}

void app_main(void)
{
    // NVS нужна Wi-Fi драйверу. Если раздел старый или битый, просто чистим его.
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    wifi_init();
    espnow_init();
    ESP_LOGI(TAG, "TX started: channel %d, %d packets/s", WIFI_CHANNEL, SEND_HZ);

    uint32_t count = 0;
    TickType_t last = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(1000 / SEND_HZ);
    while (true) {
        esp_now_send(BROADCAST, (const uint8_t *)&count, sizeof(count));
        count++;

        // Раз в 5 секунд пишем в лог, чтоб было видно, что плата живая
        if (count % (SEND_HZ * 5) == 0) {
            ESP_LOGI(TAG, "sent %lu packets", (unsigned long)count);
        }

        // vTaskDelayUntil держит ровный шаг 10 мс, даже если отправка
        // заняла какое-то время. Обычный vTaskDelay так не умеет.
        vTaskDelayUntil(&last, period);
    }
}
