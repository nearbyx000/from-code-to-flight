#!/usr/bin/env python3
import requests
import subprocess
import json
import sys

# ---------------------------------------------------------------
# --- КОНФИГУРАЦИЯ ---
# ---------------------------------------------------------------

# ⚠️ ВАЖНО: Замените "192.168.1.50" на IP-адрес вашего ПК с Windows 11
PC_SERVER_IP = "100.0.0.2"

# Порт, который использует LM Studio (по умолчанию 1234)
LM_STUDIO_PORT = 1234

# ⚠️ Убедитесь, что этот путь совпадает с тем, куда вы скачали модель
PIPER_MODEL_PATH = "/home/pi/models/ru_RU-irina-medium.onnx"
# Адрес API сервера на вашем ПК
LLM_API_URL = f"http://{PC_SERVER_IP}:{LM_STUDIO_PORT}/v1/chat/completions"

def get_llm_response(user_prompt: str) -> str:
    """
    Отправляет ОДИН ЕДИНСТВЕННЫЙ запрос на ПК.
    (Временно без истории чата для теста)
    """
    global chat_history # Мы пока не используем историю, но оставим для будущего
    print("[LLM]... Отправка запроса на ПК (режим теста) ...")

    # [INST] ... [/INST] - это формат, который понимает Mistral.
    # Мы добавляем его вручную, так как отключили эту функцию в LM Studio.
    # Мы также добавляем наш системный промпт прямо сюда.

    system_prompt = "Ты — полезный ИИ-ассистент по имени Мистраль. Ты отвечаешь на русском языке, кратко и по делу."
    full_prompt_for_model = f"[INST] {system_prompt} {user_prompt} [/INST]"

    # Формируем тело запроса
    payload = {
        "model": "mistral-7b", # Имя не важно, но обязательно
        "messages": [
            # Мы отправляем ТОЛЬКО ОДНО сообщение с ролью "user"
            {"role": "user", "content": full_prompt_for_model}
        ],
        "temperature": 0.7,
        "max_tokens": 512,
        "stream": False
    }

    try:
        response = requests.post(LLM_API_URL, json=payload, timeout=90)
        response.raise_for_status()  # Проверка на HTTP ошибки

        data = response.json()

        # Извлекаем текст ответа из JSON
        assistant_response = data['choices'][0]['message']['content']

        # (Пока не сохраняем в историю, мы ее отключили)

        return assistant_response.strip()

    except requests.exceptions.Timeout:
        print("[Ошибка] Сервер на ПК не ответил (timeout).", file=sys.stderr)
        return "Ошибка. Сервер не отвечает."
    except requests.exceptions.ConnectionError:
        print(f"[Ошибка] Не удалось подключиться к {LLM_API_URL}.", file=sys.stderr)
        print("Проверьте IP-адрес и Брандмауэр на ПК.", file=sys.stderr)
        return "Ошибка подключения к серверу."
    except Exception as e:
        print(f"[Ошибка] Неизвестная ошибка при запросе к LLM: {e}", file=sys.stderr)
        return "Неизвестная ошибка."


# --- ЗАМЕНИТЕ ВАШУ ФУНКЦИЮ 'speak_text' НА ЭТУ ---
def speak_text(text: str):
    """
    Озвучивает текст, используя espeak-ng, и напрямую передает аудио в 'aplay'.
    """
    if not text:
        print("[TTS] Нечего озвучивать.", file=sys.stderr)
        return

    print("[TTS]... Генерация речи (espeak-ng) ...")
    try:
        # -v ru  : использовать русский голос
        # -s 160 : скорость (160 слов в минуту)
        # --stdout: выводить WAV в stdout
        espeak_cmd = [
            'espeak-ng',
            '-v', 'ru',
            '-s', '160',
            '--stdout'
        ]
        
        # aplay читает WAV из stdin
        aplay_cmd = ['aplay', '-q', '-t', 'wav', '-']

        # Запускаем espeak
        p_espeak = subprocess.Popen(espeak_cmd, stdin=subprocess.PIPE, stdout=subprocess.PIPE)
        
        # Запускаем aplay, который "слушает" вывод espeak
        p_aplay = subprocess.Popen(aplay_cmd, stdin=p_espeak.stdout)
        
        # Передаем текст в espeak
        p_espeak.communicate(input=text.encode('utf-8'))
        
        # Ждем завершения aplay
        p_aplay.wait()

    except FileNotFoundError:
        print("[Ошибка] 'espeak-ng' или 'aplay' не найдены.", file=sys.stderr)
    except Exception as e:
        print(f"[Ошибка] Не удалось озвучить текст: {e}", file=sys.stderr)
def main():
    """
    Главный цикл диалога.
    """
    print("--- Локальный Голосовой Ассистент ---")
    print(f"Подключение к серверу: {PC_SERVER_IP}:{LM_STUDIO_PORT}")
    print("Введите 'выход' или 'exit' для завершения.")
    
    while True:
        try:
            user_input = input("\nВы: ")
            if user_input.lower() in ('выход', 'exit', 'quit'):
                print("До свидания!")
                break
            
            # 1. Получаем ответ от LLM
            mistral_response = get_llm_response(user_input)
            print(f"Ассистент: {mistral_response}")
            
            # 2. Озвучиваем ответ
            speak_text(mistral_response)
            
        except KeyboardInterrupt:
            print("\nДо свидания!")
            break

if __name__ == "__main__":
    main()