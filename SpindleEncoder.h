#ifndef SPINDLE_ENCODER_H
#define SPINDLE_ENCODER_H

#include <Arduino.h>
#include <driver/pcnt.h>
#include "Config.h"
#include "RussianLogger.h"

/**
 * @class SpindleEncoder
 * @brief Управление энкодером шпинделя и расчет скорости вращения
 * 
 * Использует аппаратный счетчик импульсов ESP32 для точного подсчета импульсов энкодера.
 * Вычисляет скорость вращения шпинделя (RPM) и предоставляет позицию для синхронизации осей.
 * Компенсирует механический люфт энкодера при смене направления вращения.
 */
class SpindleEncoder {
private:
    // Текущее состояние энкодера
    long position;              ///< Текущая позиция энкодера в счетных импульсах [0, ENCODER_STEPS_INT-1]
    long positionAvg;           ///< Усредненная позиция с компенсацией люфта энкодера
    long positionGlobal;        ///< Глобальная позиция (не обнуляется при установке нуля)
    int16_t counterValue;       ///< Текущее значение аппаратного счетчика импульсов
    unsigned long lastUpdateUs; ///< Время последнего обновления позиции в микросекундах
    
    // Для расчета скорости вращения (RPM)
    unsigned long bulkStartTimeUs; ///< Время начала измерения для усреднения RPM
    int bulkPulseCount;           ///< Счетчик импульсов для усреднения RPM
    int currentRpm;               ///< Текущие вычисленные обороты в минуту
    unsigned long lastRpmUpdateUs; ///< Время последнего обновления значения RPM
    
    // Синхронизация с осями
    int syncOffset;             ///< Смещение для синхронизации со шпинделем при выходе из упора

public:
    /**
     * @brief Конструктор энкодера шпинделя
     */
    SpindleEncoder() : position(0), positionAvg(0), positionGlobal(0), counterValue(0),
                      lastUpdateUs(micros()), bulkStartTimeUs(micros()), bulkPulseCount(0),
                      currentRpm(0), lastRpmUpdateUs(micros()), syncOffset(0) {}
    
    /**
     * @brief Инициализация аппаратного счетчика импульсов ESP32
     * 
     * Настраивает PCNT (Pulse Counter) для подсчета импульсов энкодера с фильтрацией
     * и компенсацией дребезга контактов.
     */
    void begin() {
        // Конфигурация канала 0 счетчика импульсов
        pcnt_config_t pcntConfig;
        pcntConfig.pulse_gpio_num = ENC_A;          // Импульсы на этом пине
        pcntConfig.ctrl_gpio_num = ENC_B;           // Управление направлением на этом пине
        pcntConfig.channel = PCNT_CHANNEL_0;        // Используем канал 0
        pcntConfig.unit = PCNT_UNIT_0;              // Используем блок 0
        pcntConfig.pos_mode = PCNT_COUNT_INC;       // Счет вперед при положительном фронте
        pcntConfig.neg_mode = PCNT_COUNT_DEC;       // Счет назад при отрицательном фронте
        pcntConfig.lctrl_mode = PCNT_MODE_REVERSE;  // Реверс при низком уровне на ctrl
        pcntConfig.hctrl_mode = PCNT_MODE_KEEP;     // Не менять при высоком уровне на ctrl
        pcntConfig.counter_h_lim = PCNT_LIM;        // Верхний предел счетчика
        pcntConfig.counter_l_lim = -PCNT_LIM;       // Нижний предел счетчика
        
        // Применение конфигурации
        pcnt_unit_config(&pcntConfig);
        
        // Настройка фильтра для подавления дребезга
        pcnt_set_filter_value(PCNT_UNIT_0, ENCODER_FILTER);
        pcnt_filter_enable(PCNT_UNIT_0);
        
        // Запуск счетчика
        pcnt_counter_pause(PCNT_UNIT_0);
        pcnt_counter_clear(PCNT_UNIT_0);
        pcnt_counter_resume(PCNT_UNIT_0);
        
        LOG_INFO("Энкодер", "Инициализирован. PPR: " + String(ENCODER_PPR) + 
                ", Фильтр: " + String(ENCODER_FILTER) + ", Полных импульсов: " + String(ENCODER_STEPS_INT));
    }
    
    /**
     * @brief Обновление состояния энкодера
     * 
     * Должен вызываться в основном цикле системы. Считывает новые импульсы из аппаратного
     * счетчика, обновляет позицию и вычисляет скорость вращения шпинделя.
     */
    void update() {
        int16_t count;
        // Получение текущего значения счетчика
        pcnt_get_counter_value(PCNT_UNIT_0, &count);
        int delta = count - counterValue;
        
        // Если изменений нет - выходим
        if (delta == 0) return;
        
        // Проверка переполнения счетчика
        if (count >= PCNT_CLEAR || count <= -PCNT_CLEAR) {
            // Сброс счетчика для избежания переполнения
            pcnt_counter_clear(PCNT_UNIT_0);
            counterValue = 0;
            LOG_DEBUG("Энкодер", "Счетчик сброшен из-за приближения к пределу");
        } else {
            counterValue = count;
        }
        
        // Обработка новых импульсов
        processPulses(delta);
    }
    
    /**
     * @brief Получение текущей позиции энкодера
     * @return Позиция в счетных импульсах [0, ENCODER_STEPS_INT-1]
     */
    long getPosition() const { 
        return position; 
    }
    
    /**
     * @brief Получение усредненной позиции с компенсацией люфта
     * @return Усредненная позиция в счетных импульсах
     */
    long getAveragePosition() const { 
        return positionAvg; 
    }
    
    /**
     * @brief Получение глобальной позиции (без обнуления)
     * @return Глобальная позиция в счетных импульсах
     */
    long getGlobalPosition() const { 
        return positionGlobal; 
    }
    
    /**
     * @brief Получение текущей скорости вращения шпинделя
     * @return Скорость в RPM (обороты в минуту)
     */
    int getRpm() const { 
        return currentRpm; 
    }
    
    /**
     * @brief Сброс позиции энкодера в ноль
     * 
     * Используется при установке новой нулевой точки системы.
     */
    void resetPosition() {
        position = 0;
        positionAvg = 0;
        syncOffset = 0;
        LOG_INFO("Энкодер", "Позиция сброшена в ноль");
    }
    
    /**
     * @brief Установка смещения для синхронизации
     * @param offset Смещение в счетных импульсах
     * 
     * Используется когда ось стоит на упоре и шпиндель вращается - позволяет
     * синхронизировать начало движения при сходе с упора.
     */
    void setSyncOffset(int offset) { 
        syncOffset = offset;
        LOG_DEBUG("Энкодер", "Установлено смещение синхронизации: " + String(offset));
    }
    
    /**
     * @brief Получение смещения синхронизации
     * @return Смещение в счетных импульсах
     */
    int getSyncOffset() const { 
        return syncOffset; 
    }
    
    /**
     * @brief Проверка активности шпинделя
     * @param timeoutMs Таймаут неактивности в миллисекундах
     * @return true если шпиндель вращается (были импульсы за указанное время)
     */
    bool isSpinning(unsigned long timeoutMs = 100) const {
        return (micros() - lastUpdateUs) < (timeoutMs * 1000);
    }
    
    /**
     * @brief Нормализация позиции в диапазон [0, ENCODER_STEPS_INT-1]
     * @param pos Позиция для нормализации
     * @return Нормализованная позиция
     */
    long normalizePosition(long pos) const {
        pos = pos % ENCODER_STEPS_INT;
        if (pos < 0) {
            pos += ENCODER_STEPS_INT;
        }
        return pos;
    }

private:
    /**
     * @brief Обработка новых импульсов от аппаратного счетчика
     * @param delta Изменение счетчика с момента последнего обновления
     * 
     * Обновляет позиции, вычисляет RPM и применяет компенсацию люфта энкодера.
     */
    void processPulses(int delta) {
        unsigned long microsNow = micros();
        
        // Обновление расчета RPM (усреднение по ENCODER_STEPS_INT импульсам)
        if (bulkPulseCount >= ENCODER_STEPS_INT) {
            unsigned long timeDiff = microsNow - bulkStartTimeUs;
            if (timeDiff > 0) {
                currentRpm = 60000000 / timeDiff; // 60 секунд * 1,000,000 микросекунд
            }
            bulkStartTimeUs = microsNow;
            bulkPulseCount = 0;
        }
        bulkPulseCount += abs(delta);
        
        // Обновление позиции энкодера
        position += delta;
        positionGlobal += delta;
        
        // Нормализация глобальной позиции
        if (positionGlobal > ENCODER_STEPS_INT) {
            positionGlobal -= ENCODER_STEPS_INT;
        } else if (positionGlobal < 0) {
            positionGlobal += ENCODER_STEPS_INT;
        }
        
        // Компенсация люфта энкодера
        // positionAvg отстает от position при обратном движении на величину люфта
        if (position > positionAvg) {
            positionAvg = position;
        } else if (position < positionAvg - ENCODER_BACKLASH) {
            positionAvg = position + ENCODER_BACKLASH;
        }
        
        lastUpdateUs = microsNow;
        
        // Логирование только при значительных изменениях
        if (abs(delta) > 10) {
            LOG_MOTION("Энкодер", "Импульсов: " + String(delta) + ", Позиция: " + String(position) + 
                      ", RPM: " + String(currentRpm));
        }
    }
};

#endif // SPINDLE_ENCODER_H