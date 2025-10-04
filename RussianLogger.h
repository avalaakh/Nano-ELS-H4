#ifndef RUSSIAN_LOGGER_H
#define RUSSIAN_LOGGER_H

#include <Arduino.h>
#include "Config.h"

/**
 * @class RussianLogger
 * @brief Система логирования на русском языке для отладки NanoELS
 * 
 * Записывает подробные сообщения об ошибках и событиях системы на русском языке.
 * Позволяет отслеживать работу системы в реальном времени через Serial порт.
 */
class RussianLogger {
private:
    bool enabled;                       // Активна ли система логирования
    unsigned long logStartTime;         // Время начала логирования
    String logBuffer;                   // Буфер для хранения логов
    static constexpr int BUFFER_SIZE = 4096; // Максимальный размер буфера

public:
    /**
     * @brief Уровни важности сообщений
     */
    enum LogLevel {
        LOG_ERROR,      // Критические ошибки требующие немедленного внимания
        LOG_WARNING,    // Предупреждения о потенциальных проблемах
        LOG_INFO,       // Информационные сообщения о работе системы
        LOG_DEBUG,      // Отладочная информация для разработчиков
        LOG_MOTION      // Данные о движении осей (высокая частота)
    };
    
    /**
     * @brief Конструктор системы логирования
     */
    RussianLogger() : enabled(true), logStartTime(millis()) {
        logBuffer.reserve(BUFFER_SIZE);
    }
    
    /**
     * @brief Запись сообщения в лог
     * @param level Уровень важности сообщения
     * @param module Модуль-источник сообщения (например "Ось Z", "Энкодер")
     * @param message Текст сообщения на русском языке
     */
    void log(LogLevel level, const String& module, const String& message) {
        if (!enabled) return;
        
        // Преобразование уровня в русскую строку
        String levelStr;
        switch(level) {
            case LOG_ERROR: levelStr = "ОШИБКА"; break;
            case LOG_WARNING: levelStr = "ПРЕДУПР"; break;
            case LOG_INFO: levelStr = "ИНФО"; break;
            case LOG_DEBUG: levelStr = "ОТЛАДКА"; break;
            case LOG_MOTION: levelStr = "ДВИЖЕНИЕ"; break;
        }
        
        // Формирование записи лога с временной меткой
        unsigned long time = millis() - logStartTime;
        String logEntry = "[" + String(time) + "ms][" + levelStr + "][" + module + "] " + message;
        
        // Вывод в Serial порт
        Serial.println(logEntry);
        
        // Сохранение в буфер (с ограничением размера)
        if (logBuffer.length() < BUFFER_SIZE) {
            logBuffer += logEntry + "\n";
        } else {
            // Если буфер переполнен, сохраняем только последние сообщения
            int newlinePos = logBuffer.indexOf('\n');
            if (newlinePos != -1) {
                logBuffer = logBuffer.substring(newlinePos + 1) + logEntry + "\n";
            }
        }
    }
    
    /**
     * @brief Включение/выключение логирования
     * @param state true - включить, false - выключить
     */
    void enable(bool state) { 
        enabled = state; 
        log(LOG_INFO, "Логгер", state ? "Логирование включено" : "Логирование выключено");
    }
    
    /**
     * @brief Получение содержимого буфера логов
     * @return Строка с всеми сохраненными логами
     */
    String getLogBuffer() const { 
        return logBuffer; 
    }
    
    /**
     * @brief Очистка буфера логов
     */
    void clearBuffer() { 
        logBuffer = ""; 
        log(LOG_INFO, "Логгер", "Буфер логов очищен");
    }
    
    /**
     * @brief Проверка активности логирования
     * @return true если логирование активно
     */
    bool isEnabled() const { 
        return enabled; 
    }
};

// Создание глобального экземпляра логгера
extern RussianLogger Logger;

// Макросы для удобного логирования
#define LOG_ERROR(module, message) Logger.log(RussianLogger::LOG_ERROR, module, message)
#define LOG_WARNING(module, message) Logger.log(RussianLogger::LOG_WARNING, module, message) 
#define LOG_INFO(module, message) Logger.log(RussianLogger::LOG_INFO, module, message)
#define LOG_DEBUG(module, message) Logger.log(RussianLogger::LOG_DEBUG, module, message)
#define LOG_MOTION(module, message) Logger.log(RussianLogger::LOG_MOTION, module, message)

#endif // RUSSIAN_LOGGER_H