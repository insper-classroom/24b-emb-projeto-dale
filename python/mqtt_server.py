#!/usr/bin/env python3
"""
Servidor MQTT para receber áudio da Raspberry Pi Pico W
"""

import paho.mqtt.client as mqtt
import numpy as np
import soundfile as sf
import time
import os
from datetime import datetime

from openai import OpenAI

# Inicializar o cliente da OpenAI (substitua 'sua_api_key' pela sua chave de API válida)
client_openai = OpenAI(api_key="")


# Configurar cliente MQTT
client = mqtt.Client()


import telebot
# import time
# import datetime
# import os

# Configurações
MQTT_BROKER = "172.20.10.13"  # Endereço IP do seu computador
MQTT_PORT = 1883
MQTT_TOPIC = "/audio_data/#"
SAMPLE_RATE = 8000
AUDIO_DIR = "audio_recordings"


MQTT_TRANSCRIPTION_TOPIC = "/transcription"


# Criar diretório para salvar os arquivos de áudio se não existir
if not os.path.exists(AUDIO_DIR):
    os.makedirs(AUDIO_DIR)

# Variáveis globais para armazenar os chunks de áudio
audio_chunks = {}
current_recording = None
last_chunk_time = 0
timeout = 3  # segundos para timeout


class BotTelegram():
    def __init__(self):
        self.token = '7732963685:AAFWrNnTSSsWlNyKiVCvo6ZHnOnYP7H7NXs'
        self.chat_id = 861453175
        self.bot = telebot.TeleBot(self.token)
        
    def send_messagem(self, message):
        self.bot.send_message(self.chat_id, message, parse_mode='Markdown')
    
    def send_audio(self, audio_path):
        with open(audio_path, 'rb') as audio_file:
            self.bot.send_audio(self.chat_id, audio_file, caption="Áudio gravado", parse_mode='Markdown')

# Importar a classe BotTelegram
bot_telegram = BotTelegram()


def on_connect(client, userdata, flags, rc):
    """Função chamada quando o cliente se conecta ao broker MQTT"""
    print(f"Conectado ao broker MQTT com código {rc}")
    client.subscribe(MQTT_TOPIC)
    print(f"Inscrito no tópico {MQTT_TOPIC}")
    print("Aguardando áudio da Pico W...")

def on_message(client, userdata, msg):
    """Função chamada quando uma mensagem é recebida"""
    global audio_chunks, current_recording, last_chunk_time
    
    topic = msg.topic
    
    # Extrair o índice do chunk do tópico
    try:
        # Formato esperado: /audio_data/N onde N é o índice
        chunk_index = int(topic.split('/')[-1])
    except ValueError:
        print(f"Formato de tópico inválido: {topic}")
        return
    
    # Se este é o primeiro chunk (0), iniciar uma nova gravação
    if chunk_index == 0:
        # Se havia uma gravação anterior em andamento, salvar se possível
        if current_recording is not None and audio_chunks:
            save_audio()
        
        # Iniciar nova gravação
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        current_recording = timestamp
        audio_chunks = {}
        print(f"\nIniciando nova gravação: {timestamp}")
    
    # Armazenar o chunk
    # Converter de bytes para array de inteiros de 8 bits
    chunk_data = np.frombuffer(msg.payload, dtype=np.uint8)
    audio_chunks[chunk_index] = chunk_data
    
    # Atualizar o timestamp do último chunk recebido
    last_chunk_time = time.time()
    
    # Imprimir progresso
    print(f"Recebido chunk {chunk_index} ({len(chunk_data)} bytes)", end="\r")

def check_timeout():
    """Verifica se houve timeout desde o último chunk recebido"""
    global last_chunk_time, audio_chunks, current_recording
    
    if current_recording is not None and audio_chunks and time.time() - last_chunk_time > timeout:
        print(f"\nTimeout após {timeout} segundos. Salvando áudio...")
        save_audio()

def save_audio():
    """Salva os chunks de áudio em um arquivo WAV"""
    global audio_chunks, current_recording
    
    if not audio_chunks or current_recording is None:
        return
    
    # Verificar se temos chunks sequenciais
    expected_indices = set(range(max(audio_chunks.keys()) + 1))
    actual_indices = set(audio_chunks.keys())
    
    missing_indices = expected_indices - actual_indices
    if missing_indices:
        print(f"Atenção: Faltando chunks: {sorted(missing_indices)}")
    
    # Ordenar os chunks pelo índice
    sorted_chunks = [audio_chunks[i] for i in sorted(audio_chunks.keys())]
    
    # Concatenar todos os chunks
    audio_data = np.concatenate(sorted_chunks)
    
    # Normalizar para valores entre -1 e 1 para salvar como WAV
    audio_normalized = (audio_data.astype(np.float32) / 128) - 1.0
    
    # Salvar como arquivo WAV
    filename = os.path.join(AUDIO_DIR, f"audio_{current_recording}.wav")
    sf.write(filename, audio_normalized, SAMPLE_RATE)
    
    print(f"Áudio salvo como {filename}")
    print(f"Tamanho total: {len(audio_normalized)} amostras ({len(audio_normalized)/SAMPLE_RATE:.2f} segundos)")
    
    
    time.sleep(2)
    
    # Enviar áudio para o Telegram

    bot_telegram.send_audio(filename)
    print("Áudio enviado para o Telegram.")
    
    time.sleep(2)
    
    # enviar para o whisper
    with open(filename, "rb") as audio_file:
        transcription = client_openai.audio.transcriptions.create(
            model="whisper-1",        
            file=audio_file,  # Passa o objeto de arquivo em vez do caminho
            response_format="text"    
        )

    print("Transcrição obtida pelo Whisper:")
    
    bot_telegram.send_messagem(transcription)
    
    # remova todos os caracteres com acento ou especiais para enviar
    import unicodedata
    
    def remover_acentos(texto):
        # Normaliza o texto para decomposição Unicode e remove acentos
        normalizado = unicodedata.normalize('NFD', texto)
        # Remove caracteres não ASCII (acentos, caracteres especiais, etc.)
        sem_acentos = ''.join(c for c in normalizado if unicodedata.category(c) != 'Mn')
        # Remove outros caracteres especiais
        sem_especiais = ''.join(c for c in sem_acentos if c.isalnum() or c.isspace() or c in '.,!?:-')
        return sem_especiais
    
    transcription_limpa = remover_acentos(transcription)
    print(f"Transcrição original: {transcription}")
    print(f"Transcrição sem acentos e caracteres especiais: {transcription_limpa}")
    
    # Publicar a transcrição sem acentos de volta para a Pico W
    client.publish(MQTT_TRANSCRIPTION_TOPIC, transcription_limpa, qos=1)
    print(f"Transcrição enviada para a Pico W via MQTT no tópico {MQTT_TRANSCRIPTION_TOPIC}\n\n")
    
    
    
    # Limpar para a próxima gravação
    audio_chunks = {}
    current_recording = None

def main():
    """Função principal"""
    print(f"Iniciando servidor MQTT em {MQTT_BROKER}:{MQTT_PORT}")
    
    
    client.on_connect = on_connect
    client.on_message = on_message
    
    # Tentar conectar ao broker MQTT
    try:
        client.connect(MQTT_BROKER, MQTT_PORT, 60)
    except Exception as e:
        print(f"Erro ao conectar ao broker MQTT: {e}")
        print(f"Certifique-se de que o broker MQTT está rodando em {MQTT_BROKER}:{MQTT_PORT}")
        print("Você pode instalar e iniciar o Mosquitto broker:")
        print("  brew install mosquitto")
        print("  mosquitto -v")
        return
    
    # Iniciar loop em segundo plano
    client.loop_start()
    
    try:
        # Loop principal
        while True:
            # Verificar timeout
            check_timeout()
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("\nEncerrando...")
        if audio_chunks:
            save_audio()
        client.loop_stop()
        client.disconnect()

if __name__ == "__main__":
    main()
