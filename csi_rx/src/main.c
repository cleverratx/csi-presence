// Приемник, плата номер 2. Тут вся логика детектора.
//
// Как это работает, если по-простому. Передатчик шлет 100 пакетов в секунду.
// На каждый пакет Wi-Fi чип сам считает, как комната исказила сигнал
// на каждой из 64 частот (поднесущих). Этот набор чисел и называют CSI.
//
// Когда в комнате никого нет, картинка почти не меняется от пакета к пакету.
// Когда человек ходит, он меняет пути, по которым летят отражения,
// и амплитуды на поднесущих начинают дрожать. Мы меряем, насколько сильно
// они дрожат за последние полсекунды, и сравниваем с порогом. Порог
// подбирается сам на старте, пока комната пустая.

#include <stdio.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "esp_timer.h"
#include "esp_mac.h"
#include "esp_idf_version.h"
#include "driver/rmt_tx.h"

// В ESP-IDF 6 переименовали WIFI_BW_HT20 в WIFI_BW20.
// Чтоб код собирался и на старых версиях, выбираем имя тут.
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(6, 0, 0)
#define BW_20MHZ WIFI_BW20
#else
#define BW_20MHZ WIFI_BW_HT20
#endif

// Настройки. Крутить в основном THRESH_K и HOLD_MS.
#define WIFI_CHANNEL   11      // обязательно такой же, как у передатчика
#define LED_GPIO       48      // RGB на DevKitC-1 v1.0. На v1.1 он на 38
#define WINDOW         50      // окно в пакетах. 50 пакетов = 0.5 с
#define EVAL_EVERY     10      // пересчитываем оценку раз в 10 пакетов, то есть 10 раз в секунду
#define CALIB_SEC      10      // сколько секунд калибруемся на старте
#define THRESH_K       4.0f    // порог = фон + K * разброс фона. Меньше K = чувствительнее
#define HOLD_MS        3000    // сколько держать "человек есть" после последнего движения

static const uint8_t TX_MAC[6] = {0x1a, 0x00, 0x00, 0x00, 0x00, 0x00};
static const char *TAG = "csi_rx";

// Что лежит в буфере CSI. Сначала идет LLTF, это 64 поднесущих по 2 байта:
// мнимая часть, потом действительная. Индексы 0..31 это поднесущие 0..31,
// а индексы 32..63 это поднесущие -32..-1.
// Реальные данные есть только на поднесущих от -26 до +26. Остальные
// защитные или нулевые, их выкидываем. Индекс 1 тоже пропускаем:
// на S3 первые 4 байта буфера иногда приходят мусорные.
#define N_RAW  64
#define N_SC   51
static int s_sc_idx[N_SC];

typedef struct { int8_t buf[N_RAW * 2]; } csi_pkt_t;
static QueueHandle_t s_queue;

// ---------- RGB светодиод ----------
// На плате стоит адресный WS2812. Обычным digitalWrite его не зажечь,
// ему нужны импульсы с точностью до долей микросекунды. Генерируем их
// через RMT, это аппаратный блок как раз для таких сигналов.

static rmt_channel_handle_t s_led_chan;
static rmt_encoder_handle_t s_led_enc;

static void led_init(void)
{
    rmt_tx_channel_config_t ch = {
        .gpio_num = LED_GPIO,
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = 10 * 1000 * 1000,   // 10 МГц, один тик 0.1 мкс
        .mem_block_symbols = 64,
        .trans_queue_depth = 4,
    };
    ESP_ERROR_CHECK(rmt_new_tx_channel(&ch, &s_led_chan));

    // Бит "0": 0.3 мкс высокий уровень, потом 0.9 мкс низкий.
    // Бит "1": наоборот, 0.9 высокий и 0.3 низкий. Так WS2812 их и различает.
    rmt_bytes_encoder_config_t enc = {
        .bit0 = {.level0 = 1, .duration0 = 3, .level1 = 0, .duration1 = 9},
        .bit1 = {.level0 = 1, .duration0 = 9, .level1 = 0, .duration1 = 3},
        .flags.msb_first = 1,
    };
    ESP_ERROR_CHECK(rmt_new_bytes_encoder(&enc, &s_led_enc));
    ESP_ERROR_CHECK(rmt_enable(s_led_chan));
}

static void led_set(uint8_t r, uint8_t g, uint8_t b)
{
    // WS2812 ждет цвета не в RGB, а в порядке G, R, B.
    // Буфер static, потому что RMT читает его уже после возврата из rmt_transmit.
    static uint8_t grb[3];
    grb[0] = g; grb[1] = r; grb[2] = b;
    rmt_transmit_config_t tx = {.loop_count = 0};
    rmt_transmit(s_led_chan, s_led_enc, grb, sizeof(grb), &tx);
    rmt_tx_wait_all_done(s_led_chan, 100);
}

// ---------- Wi-Fi и CSI ----------

// Этот колбэк дергает сам Wi-Fi драйвер на каждый пакет из эфира.
// Считать тут ничего нельзя: если колбэк будет долгим, драйвер начнет
// терять пакеты. Поэтому только проверяем, что пакет от нашего передатчика,
// и копируем сырые байты в очередь. Считает отдельная задача ниже.
static void csi_cb(void *ctx, wifi_csi_info_t *info)
{
    if (!info || !info->buf || info->len < N_RAW * 2) return;
    if (memcmp(info->mac, TX_MAC, 6) != 0) return;
    csi_pkt_t pkt;
    memcpy(pkt.buf, info->buf, sizeof(pkt.buf));
    xQueueSend(s_queue, &pkt, 0);    // если очередь забита, пакет просто теряем, не страшно
}

static void wifi_csi_init(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_bandwidth(WIFI_IF_STA, BW_20MHZ));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
    ESP_ERROR_CHECK(esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE));

    // Promiscuous это режим "слушаю всё". Ни к какой сети не подключаемся,
    // просто ловим все пакеты на своем канале.
    ESP_ERROR_CHECK(esp_wifi_set_promiscuous(true));

    // Берем только LLTF. Это 64 поднесущих, и они есть в любом OFDM пакете,
    // так что формат буфера всегда одинаковый и его легко разбирать.
    // Сглаживание соседних поднесущих выключаем, нам нужны сырые данные.
    wifi_csi_config_t csi = {
        .lltf_en = true,
        .htltf_en = false,
        .stbc_htltf2_en = false,
        .ltf_merge_en = false,
        .channel_filter_en = false,
        .manu_scale = false,
        .shift = 0,
    };
    ESP_ERROR_CHECK(esp_wifi_set_csi_config(&csi));
    ESP_ERROR_CHECK(esp_wifi_set_csi_rx_cb(csi_cb, NULL));
    ESP_ERROR_CHECK(esp_wifi_set_csi(true));
}

// ---------- обработка ----------

// Окно последних WINDOW пакетов. Строка это пакет, столбец это поднесущая.
static float s_win[WINDOW][N_SC];

// Уровень движения. Для каждой поднесущей считаем, насколько ее амплитуда
// гуляла за окно (стандартное отклонение), и усредняем по всем поднесущим.
// Пустая комната дает маленькое число, человек на ходу дает большое.
static float motion_score(void)
{
    float total = 0;
    for (int k = 0; k < N_SC; k++) {
        float sum = 0, sumsq = 0;
        for (int t = 0; t < WINDOW; t++) {
            float v = s_win[t][k];
            sum += v;
            sumsq += v * v;
        }
        float mean = sum / WINDOW;
        float var = sumsq / WINDOW - mean * mean;
        total += var > 0 ? sqrtf(var) : 0;
    }
    return total / N_SC;
}

static void process_task(void *arg)
{
    csi_pkt_t pkt;
    int head = 0, filled = 0, since_eval = 0;

    // Для калибровки считаем среднее и разброс фона "на лету", без массива.
    // Это алгоритм Уэлфорда, он не копит ошибку округления.
    int64_t calib_end_us = 0;
    int calib_n = 0;
    double calib_mean = 0, calib_m2 = 0;
    float threshold = 0;
    bool calibrated = false;

    int64_t last_motion_us = -1000000000LL;
    int last_presence = -1;
    uint32_t pkts = 0;

    printf("\nКалибровка %d с. Выйди из комнаты или замри.\n", CALIB_SEC);
    led_set(0, 0, 24);   // синий, идет калибровка

    while (true) {
        if (xQueueReceive(s_queue, &pkt, pdMS_TO_TICKS(2000)) != pdTRUE) {
            printf("Нет пакетов от передатчика. Проверь, что TX включен и каналы совпадают\n");
            continue;
        }
        pkts++;

        // Шаг 1. Амплитуда на каждой полезной поднесущей: sqrt(re^2 + im^2)
        float amp[N_SC], mean_amp = 0;
        for (int k = 0; k < N_SC; k++) {
            int i = s_sc_idx[k];
            float im = pkt.buf[2 * i], re = pkt.buf[2 * i + 1];
            amp[k] = sqrtf(re * re + im * im);
            mean_amp += amp[k];
        }
        mean_amp /= N_SC;
        if (mean_amp < 1e-3f) continue;

        // Шаг 2. Делим на среднюю амплитуду пакета. У приемника есть
        // автоматическое усиление, оно иногда скачком меняет громкость
        // всего сигнала. Без этой нормировки такой скачок выглядел бы
        // как движение. А так остается только форма, она нам и нужна.
        for (int k = 0; k < N_SC; k++) s_win[head][k] = amp[k] / mean_amp;
        head = (head + 1) % WINDOW;
        if (filled < WINDOW) { filled++; continue; }
        if (++since_eval < EVAL_EVERY) continue;
        since_eval = 0;

        // Шаг 3. Уровень движения за последние полсекунды
        float score = motion_score();
        int64_t now = esp_timer_get_time();

        // Первые CALIB_SEC секунд только копим статистику фона
        if (!calibrated) {
            if (calib_end_us == 0) calib_end_us = now + (int64_t)CALIB_SEC * 1000000;
            calib_n++;
            double d = score - calib_mean;
            calib_mean += d / calib_n;
            calib_m2 += d * (score - calib_mean);
            if (now >= calib_end_us) {
                double sd = calib_n > 1 ? sqrt(calib_m2 / (calib_n - 1)) : 0;
                threshold = (float)(calib_mean + THRESH_K * sd);
                // Если фон был совсем ровный, порог выйдет впритык к нему
                // и будет ловить любой шум. Поэтому не даем ему быть ниже 1.3 фона.
                if (threshold < calib_mean * 1.3) threshold = (float)(calib_mean * 1.3);
                calibrated = true;
                printf("Калибровка готова: фон=%.4f, разброс=%.4f, порог=%.4f (пакетов: %lu)\n",
                       calib_mean, sd, threshold, (unsigned long)pkts);
            }
            continue;
        }

        // Шаг 4. Решение. Человек не двигается постоянно, поэтому после
        // последнего всплеска держим "есть человек" еще HOLD_MS миллисекунд.
        // Иначе светодиод мигал бы на каждой паузе.
        if (score > threshold) last_motion_us = now;
        int presence = (now - last_motion_us) < (int64_t)HOLD_MS * 1000;

        // Строки с '>' в начале понимает расширение Teleplot в VS Code
        // и рисует из них живой график
        printf(">score:%.4f\n>threshold:%.4f\n>presence:%d\n", score, threshold, presence);

        // Светодиод и сообщение меняем только когда статус поменялся
        if (presence != last_presence) {
            last_presence = presence;
            if (presence) { led_set(32, 0, 0); printf("*** ЧЕЛОВЕК ОБНАРУЖЕН ***\n"); }
            else          { led_set(0, 24, 0); printf("--- пусто ---\n"); }
        }
    }
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

    // Какие индексы буфера берем: поднесущие +2..+26 и -26..-1 (это индексы 38..63)
    int n = 0;
    for (int i = 2; i <= 26; i++) s_sc_idx[n++] = i;
    for (int i = 38; i <= 63; i++) s_sc_idx[n++] = i;

    // Порядок важен: очередь и задача должны появиться раньше,
    // чем Wi-Fi начнет вызывать колбэк
    led_init();
    s_queue = xQueueCreate(32, sizeof(csi_pkt_t));
    xTaskCreate(process_task, "csi_proc", 6144, NULL, 5, NULL);
    wifi_csi_init();
    ESP_LOGI(TAG, "RX started on channel %d, waiting for TX " MACSTR, WIFI_CHANNEL, MAC2STR(TX_MAC));
}
