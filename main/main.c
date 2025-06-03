#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>
#include <queue.h>

#include <math.h>
#include <stdio.h>
#include "pico/stdlib.h"   // stdlib
#include "hardware/irq.h"  // interrupts
#include "hardware/pwm.h"  // pwm
#include "hardware/sync.h" // wait for interrupt
#include "hardware/pll.h"
#include "hardware/clocks.h"
#include "hardware/adc.h" // adc
#include <stdlib.h>
#include "ring.h"

// Inclusões para MQTT e WiFi
#include "pico/cyw43_arch.h"

// Inclua o arquivo lwipopts.h primeiro
#include "lwipopts.h"
#include "lwip/apps/mqtt.h"
#include "lwip/dns.h"

#define AUDIO_OUT_PIN 26
#define AUDIO_IN_PIN 27

#define SAMPLE_RATE 8000
#define DATA_LENGTH SAMPLE_RATE * 4 // WAV_DATA_LENGTH //16000
#define FREQ 8000

// Configurações WiFi e MQTT
#define WIFI_SSID "PICO_TEST"
#define WIFI_PASSWORD "cauezao123"
#define MQTT_SERVER "172.20.10.13"
// Usando a porta padrão MQTT definida no lwip
// #define MQTT_PORT       1883
#define MQTT_TOPIC "/audio_data"
#define MQTT_DEVICE_NAME "pico_audio"
#define MQTT_PUB_QOS 1
#define MQTT_KEEP_ALIVE_S 60

char audio[DATA_LENGTH];

int wav_position = 0;

SemaphoreHandle_t xSemaphorePlayInit;
SemaphoreHandle_t xSemaphorePlayDone;
SemaphoreHandle_t xSemaphoreRecordDone;

// Estado do cliente MQTT
typedef struct {
    mqtt_client_t *mqtt_client;
    struct mqtt_connect_client_info_t mqtt_client_info;
    ip_addr_t mqtt_server_address;
    bool connected;
} mqtt_state_t;

mqtt_state_t mqtt_state;
SemaphoreHandle_t xSemaphoreMQTTConnected;

int audio_pin_slice;

// Callback para publicação MQTT
static void mqtt_pub_request_cb(void *arg, err_t err) {
    if (err != ERR_OK) {
        printf("MQTT publish failed: %d\n", err);
    } else {
        printf("MQTT publish success\n");
    }
    xSemaphoreGive(xSemaphorePlayDone);
}

// Callback de conexão MQTT
static void mqtt_connection_cb(mqtt_client_t *client, void *arg, mqtt_connection_status_t status) {
    mqtt_state_t *state = (mqtt_state_t *)arg;

    if (status == MQTT_CONNECT_ACCEPTED) {
        printf("MQTT conectado com sucesso!\n");
        state->connected = true;
        xSemaphoreGive(xSemaphoreMQTTConnected);
    } else {
        printf("MQTT falha na conexão: %d\n", status);
        state->connected = false;
    }
}

// Callback para resolução DNS
static void dns_found(const char *name, const ip_addr_t *ip, void *arg) {
    mqtt_state_t *state = (mqtt_state_t *)arg;
    if (!ip) {
        printf("DNS lookup falhou\n");
        return;
    }

    state->mqtt_server_address = *ip;

    // Conectando ao servidor MQTT
    cyw43_arch_lwip_begin();
    err_t err = mqtt_client_connect(
        state->mqtt_client,
        &state->mqtt_server_address,
        LWIP_IANA_PORT_MQTT, // Usando a porta padrão MQTT do LWIP
        mqtt_connection_cb,
        state,
        &state->mqtt_client_info);
    cyw43_arch_lwip_end();

    if (err != ERR_OK) {
        printf("MQTT connect falhou: %d\n", err);
    }
}

// Função para inicializar MQTT
void mqtt_init() {
    // Inicializa WiFi
    if (cyw43_arch_init()) {
        printf("Falha ao inicializar CYW43\n");
        return;
    }

    cyw43_arch_enable_sta_mode();
    printf("Conectando ao WiFi...\n");

    if (cyw43_arch_wifi_connect_timeout_ms(
            WIFI_SSID, WIFI_PASSWORD,
            CYW43_AUTH_WPA2_AES_PSK,
            30000)) {
        printf("Falha ao conectar ao WiFi\n");
        return;
    }

    printf("WiFi conectado!\n");

    // Inicializa o cliente MQTT
    mqtt_state.mqtt_client = mqtt_client_new();
    if (!mqtt_state.mqtt_client) {
        printf("Falha ao criar cliente MQTT\n");
        return;
    }

    // Configura client ID
    static char client_id[32];
    snprintf(client_id, sizeof(client_id), "%s_%lu", MQTT_DEVICE_NAME, to_ms_since_boot(get_absolute_time()));
    mqtt_state.mqtt_client_info.client_id = client_id;
    mqtt_state.mqtt_client_info.keep_alive = MQTT_KEEP_ALIVE_S;

    // Resolve o endereço do servidor MQTT
    cyw43_arch_lwip_begin();
    err_t err = dns_gethostbyname(
        MQTT_SERVER,
        &mqtt_state.mqtt_server_address,
        dns_found,
        &mqtt_state);
    cyw43_arch_lwip_end();

    if (err == ERR_OK) {
        // IP já estava disponível
        dns_found(MQTT_SERVER, &mqtt_state.mqtt_server_address, &mqtt_state);
    } else if (err != ERR_INPROGRESS) {
        printf("Erro DNS: %d\n", err);
    }

    // Aguarda a conexão MQTT (com timeout)
    if (xSemaphoreTake(xSemaphoreMQTTConnected, pdMS_TO_TICKS(10000)) != pdTRUE) {
        printf("Timeout ao aguardar conexão MQTT\n");
    }
}

/*
 * PWM Interrupt Handler which outputs PWM level and advances the
 * current sample.
 *
 * We repeat the same value for 8 cycles this means sample rate etc
 * adjust by factor of 8 (this is what bitshifting <<3 is doing)
 *
 */
void pwm_interrupt_handler() {
    pwm_clear_irq(pwm_gpio_to_slice_num(AUDIO_OUT_PIN));
    if (wav_position < (DATA_LENGTH << 3) - 1) {
        // set pwm level
        // allow the pwm value to repeat for 8 cycles this is >>3
        pwm_set_gpio_level(AUDIO_OUT_PIN, audio[wav_position >> 3]);
        wav_position++;
    } else {
        // Acabou de reproduzir o áudio
        // wav_position = 0;
        xSemaphoreGiveFromISR(xSemaphorePlayDone, 0);
    }
}

bool timer_0_callback(repeating_timer_t *rt) {
    if (wav_position < DATA_LENGTH) {
        audio[wav_position++] = adc_read() / 16;
        return true; // keep repeating
    } else {
        xSemaphoreGiveFromISR(xSemaphoreRecordDone, 0);
        return false; // stop repeating
    }
}

void mic_task() {
    adc_gpio_init(AUDIO_IN_PIN);
    adc_init();
    adc_select_input(AUDIO_IN_PIN - 26);

    repeating_timer_t timer_0;

    while (1) {
        // ======== DETECÇÃO DE FALA ========
        printf("Aguardando detecção de voz...\n");
        int detectado = 0;

        while (!detectado) {
            int sum = 0;
            int values[50];

            for (int i = 0; i < 50; i++) {
                values[i] = adc_read();
                sum += values[i];
                sleep_us(100);
            }

            int mean = sum / 50;

            int energy = 0;
            for (int i = 0; i < 50; i++) {
                energy += abs(values[i] - mean);
            }

            energy /= 50;
            printf("Signal: %d\n", energy);

            if (energy > 200) { // limiar
                detectado = 1;
                printf("Voz detectada!\n");
            }
        }

        // ======== gravacao ========
        wav_position = 0;
        if (!add_repeating_timer_us(1000000 / SAMPLE_RATE,
                                    timer_0_callback,
                                    NULL,
                                    &timer_0)) {
            printf("Erro ao iniciar timer\n");
        }

        if (xSemaphoreTake(xSemaphoreRecordDone, portMAX_DELAY) == pdTRUE) {
            cancel_repeating_timer(&timer_0);
        }

        // ======== filtro ========
        // Média móvel
        // for (int i = 2; i < DATA_LENGTH; i++) {
        //     audio[i] = (audio[i] + audio[i - 1] + audio[i - 2]) / 3;
        // }
        for (int i = 4; i < DATA_LENGTH; i++) {
            audio[i] = (audio[i] + audio[i - 1] + audio[i - 2] + audio[i - 3] + audio[i - 4]) / 5;
        }

        // Notifica a task de play para enviar o áudio gravado
        xSemaphoreGive(xSemaphorePlayInit);

        // Aguarda a conclusão do envio
        if (xSemaphoreTake(xSemaphorePlayDone, portMAX_DELAY) == pdTRUE) {
            printf("Áudio enviado com sucesso!\n");
        }

        printf("\n\nGravação concluída e enviada!\n");
    }
}

void play_task() {
    // Aguarda a inicialização do MQTT
    mqtt_init();

    while (1) {
        // Aguarda sinal para enviar o áudio
        if (xSemaphoreTake(xSemaphorePlayInit, portMAX_DELAY) == pdTRUE) {
            printf("Enviando áudio para o servidor via MQTT...\n");

            // Verifica se o cliente MQTT está conectado
            if (!mqtt_state.connected) {
                printf("MQTT não está conectado, tentando reconectar...\n");
                mqtt_init();
                if (!mqtt_state.connected) {
                    printf("Falha na reconexão MQTT\n");
                    xSemaphoreGive(xSemaphorePlayDone);
                    continue;
                }
            }

// Divide o áudio em chunks para envio (MQTT tem limite de tamanho)
#define CHUNK_SIZE 128
            int num_chunks = (DATA_LENGTH + CHUNK_SIZE - 1) / CHUNK_SIZE;

            for (int i = 0; i < num_chunks; i++) {
                int offset = i * CHUNK_SIZE;
                int remaining = DATA_LENGTH - offset;
                int size = (remaining > CHUNK_SIZE) ? CHUNK_SIZE : remaining;

                // Cria tópico com índice do chunk
                char topic[64];
                snprintf(topic, sizeof(topic), "%s/%d", MQTT_TOPIC, i);

                // Publica o chunk
                cyw43_arch_lwip_begin();
                err_t err = mqtt_publish(
                    mqtt_state.mqtt_client,
                    topic,
                    &audio[offset],
                    size,
                    MQTT_PUB_QOS,
                    0,                                                // não reter
                    i == num_chunks - 1 ? mqtt_pub_request_cb : NULL, // callback apenas no último chunk
                    i == num_chunks - 1 ? NULL : NULL);
                cyw43_arch_lwip_end();

                if (err != ERR_OK) {
                    printf("Erro ao publicar chunk %d: %d\n", i, err);
                }

                // Pequeno delay entre chunks para não sobrecarregar
                vTaskDelay(pdMS_TO_TICKS(50));
            }

            // Se chegou aqui, o callback do último chunk irá dar o semáforo xSemaphorePlayDone
            // ou damos o semáforo se houve erro no envio
            if (num_chunks == 0) {
                xSemaphoreGive(xSemaphorePlayDone);
            }
        }
    }
}

int main() {
    stdio_init_all();
    printf("oi\n");

    xSemaphorePlayInit = xSemaphoreCreateBinary();
    xSemaphorePlayDone = xSemaphoreCreateBinary();
    xSemaphoreRecordDone = xSemaphoreCreateBinary();
    xSemaphoreMQTTConnected = xSemaphoreCreateBinary();

    xTaskCreate(play_task, "Play Task", 4095, NULL, 1, NULL);
    xTaskCreate(mic_task, "Mic Task", 4095, NULL, 1, NULL);

    vTaskStartScheduler();

    while (true)
        ;
}