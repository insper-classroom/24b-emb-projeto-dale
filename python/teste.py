import telebot

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
            
            

bot_telegram = BotTelegram()

bot_telegram.send_audio("audio_recordings/audio_20250603_151743.wav")
# envie uma mensagem teste

