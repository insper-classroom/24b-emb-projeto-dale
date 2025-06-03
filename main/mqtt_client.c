/**
 * Copyright (c) 2022 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "pico/unique_id.h"

#include "hardware/gpio.h"
#include "hardware/adc.h"

#include "lwip/apps/mqtt.h"
#include "lwip/apps/mqtt_priv.h"   // para setar hostname em TLS
#include "lwip/dns.h"
#include "lwip/altcp_tls.h"

#include <stdio.h>
#include <string.h>
#include <ctype.h>

#define WIFI_SSID            "PICO_TEST"
#define WIFI_PASSWORD        "cauezao123"
#define MQTT_SERVER          "172.20.10.11"

// — parâm. MQTT —
#define SENSOR_WORKER_TIME_S 2    // intervalo de publicação (s)
#define MQTT_KEEP_ALIVE_S    60
#define MQTT_SUB_QOS         1
#define MQTT_PUB_QOS         1
#define MQTT_PUB_RETAIN      1

// tópicos
#define TOPIC_LIGHT_SENSOR   "/light_sensor"
#define TOPIC_LIGHT_CMD      "/light"
#define TOPIC_WILL           "/online"
#define WILL_MESSAGE         "0"
#define WILL_QOS             1

#define MQTT_DEVICE_NAME     "pico"

// pinos
#define LIGHT_SENSOR_ADC_CH  0     // ADC0 → GPIO26
#define LIGHT_SENSOR_GPIO    26
#define LOCAL_LED_PIN        15    // LED externo

typedef struct {
    mqtt_client_t*                    mqtt_client_inst;
    struct mqtt_connect_client_info_t mqtt_client_info;
    char                              data[MQTT_OUTPUT_RINGBUF_SIZE];
    char                              topic[128];
    uint32_t                          len;
    ip_addr_t                         mqtt_server_address;
    bool                              connect_done;
    int                               subscribe_count;
    bool                              stop_client;
} MQTT_CLIENT_DATA_T;

// prints condicionais
#ifndef DEBUG_printf
#define DEBUG_printf(...)   // nada
#endif
#ifndef INFO_printf
#define INFO_printf         printf
#endif
#ifndef ERROR_printf
#define ERROR_printf        printf
#endif

// — lê valor bruto do LDR via ADC0 —
static uint16_t read_light_sensor_raw(void) {
    return adc_read();
}

// callback de publicação
static void pub_request_cb(__unused void *arg, err_t err) {
    if (err != ERR_OK) {
        ERROR_printf("MQTT publish failed: %d\n", err);
    }
}

// monta tópico (com client-ID, se habilitado)
static const char *full_topic(MQTT_CLIENT_DATA_T *s, const char *name) {
#if MQTT_UNIQUE_TOPIC
    static char buf[128];
    snprintf(buf, sizeof(buf), "/%s%s", s->mqtt_client_info.client_id, name);
    return buf;
#else
    return name;
#endif
}

// acende/apaga o LED local
static void control_local_led(bool on) {
    gpio_put(LOCAL_LED_PIN, on ? 1 : 0);
    INFO_printf("Local LED turned %s\n", on ? "ON" : "OFF");
}

// publica no tópico /light_sensor
static void publish_light_sensor(MQTT_CLIENT_DATA_T *s) {
    static uint16_t old = 0;
    uint16_t v = read_light_sensor_raw();
    if (v != old) {
        old = v;
        char payload[16];
        int len = snprintf(payload, sizeof(payload), "%u", v);
        INFO_printf("Publishing %s to %s\n", payload, TOPIC_LIGHT_SENSOR);
        mqtt_publish(
            s->mqtt_client_inst,
            full_topic(s, TOPIC_LIGHT_SENSOR),
            payload, len,
            MQTT_PUB_QOS,
            MQTT_PUB_RETAIN,
            pub_request_cb,
            s
        );
    }
}

// callbacks de subscribe/unsubscribe
static void sub_request_cb(void *arg, err_t err) {
    if (err) panic("subscribe failed %d\n", err);
    ((MQTT_CLIENT_DATA_T*)arg)->subscribe_count++;
}
static void unsub_request_cb(void *arg, err_t err) {
    if (err) panic("unsubscribe failed %d\n", err);
    MQTT_CLIENT_DATA_T *s = (MQTT_CLIENT_DATA_T*)arg;
    s->subscribe_count--;
    if (s->subscribe_count <= 0 && s->stop_client) {
        mqtt_disconnect(s->mqtt_client_inst);
    }
}
static void sub_unsub_topics(MQTT_CLIENT_DATA_T* s, bool sub) {
    mqtt_request_cb_t cb = sub ? sub_request_cb : unsub_request_cb;
    mqtt_sub_unsub(
        s->mqtt_client_inst,
        full_topic(s, TOPIC_LIGHT_CMD),
        MQTT_SUB_QOS,
        cb,
        s,
        sub
    );
}

// início de pub — grava tópico
static void mqtt_incoming_publish_cb(void *arg, const char *topic, u32_t tot_len) {
    MQTT_CLIENT_DATA_T* s = (MQTT_CLIENT_DATA_T*)arg;
    strncpy(s->topic, topic, sizeof(s->topic)-1);
    s->topic[sizeof(s->topic)-1] = '\0';
}

// dados de uma pub recebida
static void mqtt_incoming_data_cb(void *arg, const u8_t *data, u16_t len, u8_t flags) {
    MQTT_CLIENT_DATA_T* state = (MQTT_CLIENT_DATA_T*)arg;

    // determina qual é mesmo o “nível” do tópico recebido
    #if MQTT_UNIQUE_TOPIC
    // se você estiver prefixando com client-ID, remova-o aqui…
    const char *basic_topic = state->topic + strlen(state->mqtt_client_info.client_id) + 1;
    #else
    const char *basic_topic = state->topic;
    #endif

    // copia o payload para state->data (e termina em '\0')
    int n = len < (int)sizeof(state->data)-1 ? len : (int)sizeof(state->data)-1;
    memcpy(state->data, data, n);
    state->data[n] = '\0';
    state->len = n;

    DEBUG_printf("Topic: %s, Message: %s\n", basic_topic, state->data);

    // **Aqui**: compare com o seu tópico de comando "/light"
    if (strcmp(basic_topic, TOPIC_LIGHT_CMD) == 0) {
        // “On” ou “1” → acende o LED externo
        if (!strncasecmp(state->data, "on", 2) || strcmp(state->data, "1") == 0) {
            control_local_led(true);
        }
        // “Off” ou “0” → apaga
        else if (!strncasecmp(state->data, "off", 3) || strcmp(state->data, "0") == 0) {
            control_local_led(false);
        }
    }
}

// worker que dispara publish_light_sensor()
static void sensor_worker_fn(async_context_t *ctx, async_at_time_worker_t *w) {
    MQTT_CLIENT_DATA_T* s = (MQTT_CLIENT_DATA_T*)w->user_data;
    publish_light_sensor(s);
    async_context_add_at_time_worker_in_ms(ctx, w, SENSOR_WORKER_TIME_S * 1000);
}
static async_at_time_worker_t sensor_worker = {
    .do_work = sensor_worker_fn
};

// callback de status da conexão MQTT
static void mqtt_connection_cb(mqtt_client_t *client, void *arg, mqtt_connection_status_t status) {
    MQTT_CLIENT_DATA_T* s = (MQTT_CLIENT_DATA_T*)arg;
    if (status == MQTT_CONNECT_ACCEPTED) {
        INFO_printf(">>> MQTT_CONNECT_ACCEPTED\n");
        s->connect_done = true;
        sub_unsub_topics(s, true);

        // publica “online”
        mqtt_publish(
            s->mqtt_client_inst,
            full_topic(s, TOPIC_WILL),
            "1", 1,
            WILL_QOS,
            true,
            pub_request_cb,
            s
        );

        // agenda o worker de luminosidade
        sensor_worker.user_data = s;
        async_context_add_at_time_worker_in_ms(
            cyw43_arch_async_context(),
            &sensor_worker,
            0
        );
    }
    else if (status == MQTT_CONNECT_DISCONNECTED) {
        if (!s->connect_done) panic("Failed to connect to MQTT\n");
    }
    else {
        panic("Unexpected MQTT status %d\n", status);
    }
}

// chamado pelo LWIP quando o DNS retorna
static void dns_found(const char *name, const ip_addr_t *ip, void *arg) {
    MQTT_CLIENT_DATA_T *s = (MQTT_CLIENT_DATA_T*)arg;
    if (!ip) {
        panic("DNS lookup failed\n");
    }
    s->mqtt_server_address = *ip;

    // —— seção crítica LWIP para abrir o TCP/TLS ——
    cyw43_arch_lwip_begin();
    err_t err = mqtt_client_connect(
        s->mqtt_client_inst,
        &s->mqtt_server_address,
        MQTT_PORT,
        mqtt_connection_cb,
        s,
        &s->mqtt_client_info
    );
    cyw43_arch_lwip_end();
    // ——————————————————————————————————————————

    if (err != ERR_OK) {
        panic("mqtt connect failed: %d\n", err);
    }
}

int main(void) {
    stdio_init_all();
    INFO_printf("Starting MQTT light-sensor client\n");

    // === ADC + LDR ===
    adc_init();
    adc_gpio_init(LIGHT_SENSOR_GPIO);         // mapeia GPIO26 → ADC0
    adc_set_temp_sensor_enabled(false);
    adc_select_input(LIGHT_SENSOR_ADC_CH);

    // === LED externo ===
    gpio_init(LOCAL_LED_PIN);
    gpio_set_dir(LOCAL_LED_PIN, GPIO_OUT);
    gpio_put(LOCAL_LED_PIN, 0);

    // === Wi-Fi ===
    if (cyw43_arch_init()) panic("CYW43 init failed\n");
    cyw43_arch_enable_sta_mode();
    if ( cyw43_arch_wifi_connect_timeout_ms(
            WIFI_SSID, WIFI_PASSWORD,
            CYW43_AUTH_WPA2_AES_PSK,
            30000
         ) )
        panic("Wi-Fi connect failed\n");
    INFO_printf("Connected to Wi-Fi\n");

    // === Estado MQTT ===
    static MQTT_CLIENT_DATA_T state = { 0 };
    // monta client‐ID único
    char uid[9];
    pico_get_unique_board_id_string(uid, sizeof(uid));
    for (int i = 0; i < 8; i++) uid[i] = tolower(uid[i]);
    static char cli[sizeof(MQTT_DEVICE_NAME) + 8];
    snprintf(cli, sizeof(cli), "%s%s", MQTT_DEVICE_NAME, uid);
    state.mqtt_client_info.client_id    = cli;
    state.mqtt_client_info.keep_alive   = MQTT_KEEP_ALIVE_S;
    state.mqtt_client_info.will_topic   = TOPIC_WILL;
    state.mqtt_client_info.will_msg     = WILL_MESSAGE;
    state.mqtt_client_info.will_qos     = WILL_QOS;
    state.mqtt_client_info.will_retain  = true;

    // cria o cliente
    state.mqtt_client_inst = mqtt_client_new();
    if (!state.mqtt_client_inst) panic("MQTT client alloc failed\n");
    mqtt_set_inpub_callback(
        state.mqtt_client_inst,
        mqtt_incoming_publish_cb,
        mqtt_incoming_data_cb,
        &state
    );

    // DNS + conexão
    cyw43_arch_lwip_begin();
    err_t r = dns_gethostbyname(
        MQTT_SERVER,
        &state.mqtt_server_address,
        dns_found,
        &state
    );
    cyw43_arch_lwip_end();

    if (r == ERR_OK) {
        // IP já estava em state.mqtt_server_address → chama direto
        dns_found(MQTT_SERVER, &state.mqtt_server_address, &state);
    }
    else if (r != ERR_INPROGRESS) {
        panic("DNS error %d\n", r);
    }

    // loop de eventos
    while (!state.connect_done || mqtt_client_is_connected(state.mqtt_client_inst)) {
        cyw43_arch_poll();
        cyw43_arch_wait_for_work_until(make_timeout_time_ms(10000));
    }

    INFO_printf("Client exiting\n");
    return 0;
}
