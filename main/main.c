#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>
#include <queue.h>

#include <math.h>
#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"   // stdlib
#include "hardware/irq.h"  // interrupts
#include "hardware/pwm.h"  // pwm
#include "hardware/sync.h" // wait for interrupt
#include "hardware/pll.h"
#include "hardware/clocks.h"
#include "hardware/adc.h" // adc
#include "hardware/i2c.h" // i2c para o LCD
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

// Definições para o LCD I2C
#define I2C_PORT i2c0
#define SDA_PIN 4         // Pino GP4 para SDA
#define SCL_PIN 5         // Pino GP5 para SCL
#define LCD_I2C_ADDR 0x27 // Endereço I2C padrão do LCD

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

// Adicione estas definições junto com as outras no início do arquivo
#define MQTT_TRANSCRIPTION_TOPIC "/transcription"
#define MAX_TRANSCRIPTION_LENGTH 1024
#define SCROLL_DELAY_MS 1000 // Tempo entre rolagens de texto (1 segundo)
#define SCROLL_STEP 2        // Número de linhas para rolar por vez

// Variável para sinalizar quando uma nova transcrição chega durante a rolagem
volatile bool interrupt_scroll = false;

// Variáveis e definições para o LCD
// Comandos LCD
#define LCD_CLEARDISPLAY 0x01
#define LCD_RETURNHOME 0x02
#define LCD_ENTRYMODESET 0x04
#define LCD_DISPLAYCONTROL 0x08
#define LCD_CURSORSHIFT 0x10
#define LCD_FUNCTIONSET 0x20
#define LCD_SETCGRAMADDR 0x40
#define LCD_SETDDRAMADDR 0x80

// Flags para controle do display
#define LCD_DISPLAYON 0x04
#define LCD_DISPLAYOFF 0x00
#define LCD_CURSORON 0x02
#define LCD_CURSOROFF 0x00
#define LCD_BLINKON 0x01
#define LCD_BLINKOFF 0x00

// Flags para function set
#define LCD_8BITMODE 0x10
#define LCD_4BITMODE 0x00
#define LCD_2LINE 0x08
#define LCD_1LINE 0x00
#define LCD_5x10DOTS 0x04
#define LCD_5x8DOTS 0x00

// Bits de controle do LCD
#define LCD_EN 0x04
#define LCD_RW 0x02
#define LCD_RS 0x01
#define LCD_BACKLIGHT 0x08

// Dimensões do LCD
#define LCD_ROWS 2
#define LCD_COLS 16

// Funções LCD
void lcd_write_byte(uint8_t val, uint8_t mode) {
    uint8_t high = mode | (val & 0xF0) | LCD_BACKLIGHT;
    uint8_t low = mode | ((val << 4) & 0xF0) | LCD_BACKLIGHT;

    uint8_t data[1];

    // Envio do byte alto
    data[0] = high;
    i2c_write_blocking(I2C_PORT, LCD_I2C_ADDR, data, 1, false);

    // Pulso de Enable
    data[0] = high | LCD_EN;
    i2c_write_blocking(I2C_PORT, LCD_I2C_ADDR, data, 1, false);
    sleep_us(1);

    data[0] = high & ~LCD_EN;
    i2c_write_blocking(I2C_PORT, LCD_I2C_ADDR, data, 1, false);
    sleep_us(50);

    // Envio do byte baixo
    data[0] = low;
    i2c_write_blocking(I2C_PORT, LCD_I2C_ADDR, data, 1, false);

    // Pulso de Enable
    data[0] = low | LCD_EN;
    i2c_write_blocking(I2C_PORT, LCD_I2C_ADDR, data, 1, false);
    sleep_us(1);

    data[0] = low & ~LCD_EN;
    i2c_write_blocking(I2C_PORT, LCD_I2C_ADDR, data, 1, false);
    sleep_us(50);
}

void lcd_command(uint8_t val) {
    lcd_write_byte(val, 0);
}

void lcd_write_char(char val) {
    lcd_write_byte(val, LCD_RS);
}

void lcd_init() {
    sleep_ms(50); // Espera para estabilização

    // Sequência de inicialização
    uint8_t data = LCD_BACKLIGHT;
    i2c_write_blocking(I2C_PORT, LCD_I2C_ADDR, &data, 1, false);
    sleep_ms(1000);

    // Inicialização em modo 4 bits, conforme datasheet
    lcd_command(0x33);
    sleep_ms(5);

    lcd_command(0x32);
    sleep_ms(5);

    // Configuração do LCD
    lcd_command(LCD_FUNCTIONSET | LCD_2LINE | LCD_5x8DOTS | LCD_4BITMODE);
    sleep_ms(5);

    lcd_command(LCD_DISPLAYCONTROL | LCD_DISPLAYON | LCD_CURSOROFF | LCD_BLINKOFF);
    sleep_ms(5);

    lcd_command(LCD_CLEARDISPLAY);
    sleep_ms(5);

    lcd_command(LCD_ENTRYMODESET | 0x02); // Incrementa cursor, sem shift
    sleep_ms(5);
}

void lcd_clear() {
    lcd_command(LCD_CLEARDISPLAY);
    sleep_ms(2);
}

void lcd_set_cursor(uint8_t row, uint8_t col) {
    static uint8_t offsets[] = {0x00, 0x40, 0x14, 0x54};
    lcd_command(LCD_SETDDRAMADDR | (col + offsets[row]));
}

void lcd_write_string(const char *str) {
    while (*str) {
        lcd_write_char(*str++);
    }
}

void lcd_write_message(const char *message) {
    lcd_clear();

    size_t len = strlen(message);

    if (len <= LCD_COLS) {
        // Mensagem cabe em uma linha
        lcd_set_cursor(0, 0);
        lcd_write_string(message);
    } else if (len <= LCD_COLS * 2) {
        // Mensagem cabe em duas linhas
        lcd_set_cursor(0, 0);
        char linha1[LCD_COLS + 1];
        strncpy(linha1, message, LCD_COLS);
        linha1[LCD_COLS] = '\0';
        lcd_write_string(linha1);

        lcd_set_cursor(1, 0);
        lcd_write_string(message + LCD_COLS);
    } else {
        // Mensagem muito grande, exibe apenas primeiras duas linhas
        lcd_set_cursor(0, 0);
        char linha1[LCD_COLS + 1];
        strncpy(linha1, message, LCD_COLS);
        linha1[LCD_COLS] = '\0';
        lcd_write_string(linha1);

        lcd_set_cursor(1, 0);
        char linha2[LCD_COLS + 1];
        strncpy(linha2, message + LCD_COLS, LCD_COLS);
        linha2[LCD_COLS] = '\0';
        lcd_write_string(linha2);
    }
}

// Função para rolar texto grande no LCD mostrando duas linhas por vez
void lcd_scroll_message(const char *message) {
    extern bool new_transcription_received;
    extern volatile bool interrupt_scroll;

    // Resetar a flag de interrupção no início da rolagem
    interrupt_scroll = false;

    size_t len = strlen(message);

    // Se a mensagem for pequena, apenas exibimos sem rolagem
    if (len <= LCD_COLS * 2) {
        lcd_write_message(message);
        return;
    }

    printf("Iniciando rolagem do texto (tamanho: %d caracteres)\n", len);

    // Calcular quantas páginas (de 2 linhas) são necessárias
    int total_paginas = (len + (LCD_COLS * 2) - 1) / (LCD_COLS * 2);
    printf("Total de páginas a exibir: %d\n", total_paginas);

    // Preparar cópia do texto
    char *texto = malloc(len + 1);
    if (texto == NULL) {
        lcd_write_message(message);
        return;
    }

    strcpy(texto, message);

    // Posição atual da página
    int pagina_atual = 0;

    // Loop de rolagem infinito até que nova mensagem seja recebida
    while (!interrupt_scroll) {
        // Calcular a posição inicial da página atual
        int pos_inicio = pagina_atual * (LCD_COLS * 2);

        // Limpar o display
        lcd_clear();

        // Primeira linha da página
        lcd_set_cursor(0, 0);
        if (pos_inicio < len) {
            char linha1[LCD_COLS + 1];
            int copiar_len = (pos_inicio + LCD_COLS <= len) ? LCD_COLS : (len - pos_inicio);
            strncpy(linha1, texto + pos_inicio, copiar_len);
            linha1[copiar_len] = '\0';
            lcd_write_string(linha1);

            // Imprimir log da linha exibida
            printf("Página %d/Linha 1: '%s'\n", pagina_atual + 1, linha1);
        }

        // Segunda linha da página
        lcd_set_cursor(1, 0);
        int pos_linha2 = pos_inicio + LCD_COLS;
        if (pos_linha2 < len) {
            char linha2[LCD_COLS + 1];
            int copiar_len = (pos_linha2 + LCD_COLS <= len) ? LCD_COLS : (len - pos_linha2);
            strncpy(linha2, texto + pos_linha2, copiar_len);
            linha2[copiar_len] = '\0';
            lcd_write_string(linha2);

            // Imprimir log da linha exibida
            printf("Página %d/Linha 2: '%s'\n", pagina_atual + 1, linha2);
        }

        // Aguardar usando vTaskDelay dividido em pequenos intervalos para resposta mais rápida
        for (int i = 0; i < SCROLL_DELAY_MS / 50; i++) {
            vTaskDelay(pdMS_TO_TICKS(50));
            // Verificar com mais frequência se a rolagem deve ser interrompida
            if (interrupt_scroll) {
                printf("Rolagem interrompida por nova mensagem\n");
                break;
            }
        }

        // Verificar novamente se a rolagem deve ser interrompida
        if (interrupt_scroll) {
            printf("Rolagem interrompida por nova mensagem\n");
            break;
        }

        // Avançar para a próxima página (volta ao início quando chega no final)
        pagina_atual = (pagina_atual + 1) % total_paginas;
        
        // Debug: mostrar quando volta ao início
        if (pagina_atual == 0 && total_paginas > 1) {
            printf("Voltando ao início da rolagem\n");
        }
    }

    // Limpar recursos
    free(texto);
    printf("Rolagem finalizada\n");
}

// Adicione esta variável global para armazenar a transcrição
char received_transcription[MAX_TRANSCRIPTION_LENGTH];
bool new_transcription_received = false;

// Callback para subscrição MQTT
static void mqtt_sub_request_cb(void *arg, err_t err) {
    if (err != ERR_OK) {
        printf("Falha na subscrição MQTT: %d\n", err);
    } else {
        printf("Subscrição MQTT bem-sucedida\n");
    }
}

// Callbacks para dados recebidos via MQTT
static void mqtt_incoming_publish_cb(void *arg, const char *topic, u32_t tot_len) {
    printf("Recebendo publicação no tópico: %s, tamanho total: %lu\n", topic, tot_len);
}

static void mqtt_incoming_data_cb(void *arg, const u8_t *data, u16_t len, u8_t flags) {
    // Copiar a transcrição recebida para o buffer (com limite de tamanho)
    size_t copy_len = len < (MAX_TRANSCRIPTION_LENGTH - 1) ? len : (MAX_TRANSCRIPTION_LENGTH - 1);
    memcpy(received_transcription, data, copy_len);
    received_transcription[copy_len] = '\0'; // Garantir que termine com null

    // Primeiro, interromper qualquer rolagem em andamento
    interrupt_scroll = true;

    // Sinalizar que uma nova transcrição foi recebida
    new_transcription_received = true;

    printf("Nova transcrição recebida MQTT: %s\n", received_transcription);
}

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

        // Configurar callbacks para receber mensagens
        mqtt_set_inpub_callback(client, mqtt_incoming_publish_cb, mqtt_incoming_data_cb, arg);

        // Subscrever ao tópico de transcrição
        err_t err = mqtt_subscribe(client, MQTT_TRANSCRIPTION_TOPIC, 1, mqtt_sub_request_cb, arg);
        if (err != ERR_OK) {
            printf("Falha ao subscrever ao tópico %s: %d\n", MQTT_TRANSCRIPTION_TOPIC, err);
        } else {
            printf("Subscrito ao tópico %s\n", MQTT_TRANSCRIPTION_TOPIC);
        }
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

void process_transcription_task(void *pvParameters) {
    extern volatile bool interrupt_scroll;

    // Inicializa o display LCD I2C
    printf("Inicializando o LCD I2C...\n");

    // Configurações do I2C para o LCD
    i2c_init(I2C_PORT, 100 * 1000); // 100 kHz
    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(SDA_PIN);
    gpio_pull_up(SCL_PIN);

    // Inicializa o LCD
    lcd_init();

    // Exibe mensagem inicial
    lcd_clear();
    lcd_set_cursor(0, 0);
    lcd_write_string("Aguardando");
    lcd_set_cursor(1, 0);
    lcd_write_string("transcricao...");

    while (1) {
        if (new_transcription_received) {
            printf("Nova transcrição recebida: %s\n", received_transcription);
            printf("Tamanho da transcrição: %d caracteres\n", strlen(received_transcription));

            // Aguardar um pouco para garantir que a interrupção seja processada
            vTaskDelay(pdMS_TO_TICKS(100));

            // Resetar a flag antes de iniciar nova exibição
            new_transcription_received = false;

            // Reiniciar a rolagem com o novo texto
            if (strlen(received_transcription) > LCD_COLS * 2) {
                printf("Iniciando rolagem do texto...\n");
                lcd_scroll_message(received_transcription);
                printf("Rolagem finalizada\n");
            } else {
                // Para textos curtos, apenas exibir normalmente
                printf("Texto curto, não necessita rolagem\n");
                lcd_write_message(received_transcription);
            }
        }

        // Verificar com frequência maior para responder rapidamente a novas transcrições
        vTaskDelay(pdMS_TO_TICKS(50));
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
    xTaskCreate(process_transcription_task, "Transcription Task", 2048, NULL, 1, NULL);

    vTaskStartScheduler();

    while (true)
        ;
}