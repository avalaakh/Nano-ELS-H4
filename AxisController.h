#ifndef AXIS_CONTROLLER_H
#define AXIS_CONTROLLER_H

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include "Config.h"
#include "RussianLogger.h"

/**
 * @class AxisController
 * @brief Управление одной осью станка - шаговым двигателем с обратной связью
 * 
 * Класс инкапсулирует всю логику управления одной осью: движение, ускорение,
 * ограничения, компенсацию люфта и синхронизацию с энкодером шпинделя.
 * Поддерживает как линейные (Z, X), так и вращательные (A1) оси.
 */
class AxisController {
private:
    // Структура конфигурации оси (все параметры из Config.h)
    struct AxisConfig {
        char name;              // Обозначение оси: 'Z', 'X', 'C'
        bool active;            // Активна ли ось в системе
        bool rotational;        // true для вращательной оси (A1), false для линейной
        float motorSteps;       // Число шагов двигателя на один полный оборот (с микрошагом)
        float screwPitch;       // Шаг ходового винта в деци-микронах (0.0001 мм)
        long speedStart;        // Начальная скорость движения в шагах/секунду
        long speedManualMove;   // Максимальная скорость при ручном управлении в шагах/секунду
        long acceleration;      // Ускорение двигателя в шагах/секунду²
        bool invertStepper;     // Инвертировать направление вращения двигателя
        bool needsRest;         // Требуется отключение драйвера при простое (open-loop)
        long maxTravelMm;       // Максимальное перемещение в мм (механическое ограничение)
        long backlashDu;        // Люфт механической передачи в деци-микронах (0.0001 мм)
        int enaPin;             // Номер пина Enable драйвера
        int dirPin;             // Номер пина Direction драйвера  
        int stepPin;            // Номер пина Step драйвера
    };
    
    const AxisConfig config;    // Конфигурация оси (только для чтения)
    SemaphoreHandle_t mutex;    // Семафор для синхронизации доступа к данным оси
    
    // Позиции и состояния оси
    long pos;                   // Относительная позиция инструмента в шагах (относительно нуля)
    long originPos;             // Смещение нулевой позиции в шагах (абсолютный ноль)
    long posGlobal;             // Глобальная позиция двигателя в шагах (никогда не сбрасывается)
    long motorPos;              // Позиция двигателя с учетом люфта (физическое положение)
    float fractionalPos;        // Дробная часть позиции для точного перемещения малыми шагами
    int pendingPos;             // Оставшиеся шаги для выполнения (целевая позиция - текущая)
    
    // Ограничения перемещения
    long leftStop;              // Левый предел в шагах (LONG_MAX если не установлен)
    long rightStop;             // Правый предел в шагах (LONG_MIN если не установлен)
    
    // Параметры движения
    long speed;                 // Текущая скорость в шагах/секунду
    long speedMax;              // Ограничение максимальной скорости
    long acceleration;          // Ускорение/замедление в шагах/секунду²
    long decelerateSteps;       // Число шагов до конечной позиции когда начинать замедление
    
    // Состояния управления
    bool direction;             // Текущее направление движения (true - вперед, false - назад)
    bool directionInitialized;  // Флаг инициализации направления (для начального сброса скорости)
    unsigned long stepStartUs;  // Время последнего шага в микросекундах
    int stepperEnableCounter;   // Счетчик включений драйвера (для нескольких источников)
    bool disabled;              // Ось отключена пользователем
    bool movingManually;        // Ручное движение (кнопками или маховиком)
    bool continuous;            // Непрерывное движение (для синхронных режимов)
    
    // Механические параметры
    long estopSteps;           // Предел перемещения в шагах для аварийной остановки
    long backlashSteps;        // Люфт в шагах для компенсации (движение в обратную сторону)
    
    // Для G-кода
    long gcodeRelativePos;     // Базовая позиция для относительных перемещений G-кода

public:
    /**
     * @brief Конструктор контроллера оси
     * @param name Обозначение оси ('Z', 'X', 'C')
     * @param active Активна ли ось в системе
     * @param rotational Является ли ось вращательной
     * @param motorSteps Число шагов двигателя на оборот
     * @param screwPitch Шаг винта в деци-микронах
     * @param speedStart Начальная скорость в шагах/секунду
     * @param speedManualMove Максимальная скорость при ручном управлении
     * @param acceleration Ускорение в шагах/секунду²
     * @param invertStepper Инвертировать направление
     * @param needsRest Требуется отключение драйвера при простое
     * @param maxTravelMm Максимальное перемещение в мм
     * @param backlashDu Люфт в деци-микронах
     * @param enaPin Пин Enable драйвера
     * @param dirPin Пин Direction драйвера
     * @param stepPin Пин Step драйвера
     */
    AxisController(char name, bool active, bool rotational, float motorSteps, float screwPitch,
                  long speedStart, long speedManualMove, long acceleration, bool invertStepper,
                  bool needsRest, long maxTravelMm, long backlashDu, int enaPin, int dirPin, int stepPin)
        : config{name, active, rotational, motorSteps, screwPitch, speedStart, speedManualMove,
                acceleration, invertStepper, needsRest, maxTravelMm, backlashDu, enaPin, dirPin, stepPin} {
        
        // Создание мьютекса для защиты общих данных
        mutex = xSemaphoreCreateMutex();
        
        // Инициализация позиций
        pos = 0;
        originPos = 0;
        posGlobal = 0;
        motorPos = 0;
        fractionalPos = 0.0;
        pendingPos = 0;
        
        // Инициализация ограничений (нет ограничений по умолчанию)
        leftStop = LONG_MAX;
        rightStop = LONG_MIN;
        
        // Инициализация параметров движения
        speed = speedStart;
        speedMax = LONG_MAX; // По умолчанию нет ограничения скорости
        this->acceleration = acceleration;
        
        // Расчет шагов замедления (когда начинать тормозить до полной остановки)
        decelerateSteps = 0;
        long s = speedManualMove;
        while (s > speedStart) {
            decelerateSteps++;
            s -= acceleration / float(s);
        }
        
        // Инициализация состояний
        direction = true;
        directionInitialized = false;
        stepStartUs = 0;
        stepperEnableCounter = 0;
        disabled = false;
        movingManually = false;
        continuous = false;
        
        // Расчет механических параметров
        estopSteps = maxTravelMm * 10000 / screwPitch * motorSteps;
        backlashSteps = backlashDu * motorSteps / screwPitch;
        gcodeRelativePos = 0;
        
        LOG_DEBUG("Ось " + String(name), "Создан контроллер. Люфт: " + String(backlashSteps) + 
                 " шагов, Макс.перемещение: " + String(estopSteps) + " шагов");
    }
    
    /**
     * @brief Инициализация пинов и состояния оси
     * 
     * Настраивает пины драйвера шагового двигателя на выход и устанавливает
     * начальные состояния. Включает драйвер если он не требует отключения при простое.
     */
    void begin() {
        // Настройка пинов драйвера
        pinMode(config.dirPin, OUTPUT);
        pinMode(config.stepPin, OUTPUT);
        pinMode(config.enaPin, OUTPUT);
        
        // Установка начального состояния пина STEP (активен высокий уровень)
        digitalWrite(config.stepPin, HIGH);
        
        // Включение драйвера если он не требует отключения при простое и ось не отключена
        if (!config.needsRest && !disabled) {
            digitalWrite(config.enaPin, HIGH);
        }
        
        LOG_INFO("Ось " + String(config.name), 
                "Инициализирована. Шагов на оборот: " + String(config.motorSteps) + 
                ", Шаг винта: " + String(config.screwPitch) + " du, " +
                "Люфт: " + String(backlashSteps) + " шагов");
    }
    
    /**
     * @brief Перемещение оси в указанную позицию
     * @param newPos Целевая позиция в шагах
     * @param continuousMode Флаг непрерывного движения (true для синхронных режимов)
     * @return true если команда принята, false при ошибке или занятости мьютекса
     * 
     * Для непрерывного движения (continuous=true) ось будет постоянно пытаться
     * достичь целевой позиции, которая может меняться в реальном времени.
     * Для финального позиционирования (continuous=false) ось остановится при достижении цели.
     */
    bool moveTo(long newPos, bool continuousMode = false) {
        // Попытка захватить мьютекс с таймаутом 10ms
        if (xSemaphoreTake(mutex, 10) != pdTRUE) {
            LOG_ERROR("Ось " + String(config.name), "Не удалось захватить мьютекс для движения к позиции " + String(newPos));
            return false;
        }
        
        continuous = continuousMode;
        
        // Если уже находимся в целевой позиции - сбрасываем ожидающие шаги
        if (newPos == pos) {
            pendingPos = 0;
        } else {
            // Расчет необходимых шагов с учетом люфта
            // При движении вперед люфт уже выбран, при движении назад - нужно его компенсировать
            pendingPos = newPos - motorPos - (newPos > pos ? 0 : backlashSteps);
            
            // Проверка пределов безопасности
            long travel = abs(newPos - pos);
            if (travel > estopSteps) {
                LOG_ERROR("Ось " + String(config.name), 
                         "Превышен предел перемещения: " + String(travel) + 
                         " шагов (максимум: " + String(estopSteps) + ")");
                xSemaphoreGive(mutex);
                return false;
            }
            
            LOG_DEBUG("Ось " + String(config.name), 
                     "Движение к позиции: " + String(newPos) + 
                     ", Ожидающих шагов: " + String(pendingPos));
        }
        
        xSemaphoreGive(mutex);
        return true;
    }
    
    /**
     * @brief Основной цикл управления движением оси
     * 
     * Должен вызываться как можно чаще из основного цикла. Выполняет шаги двигателя,
     * управляет ускорением/замедлением и обновляет позиции.
     */
    void update() {
        // Если нет ожидающих шагов - постепенно снижаем скорость до начальной
        if (pendingPos == 0) {
            if (speed > config.speedStart) {
                speed--;
            }
            return;
        }
        
        // Проверка времени для следующего шага
        unsigned long nowUs = micros();
        float delayUs = 1000000.0 / speed; // Время между шагами в микросекундах
        
        // Если не прошло достаточно времени - ждем
        if (nowUs - stepStartUs < delayUs - 5) {
            return;
        }
        
        // Захват мьютекса для изменения состояния
        if (xSemaphoreTake(mutex, 1) == pdTRUE) {
            // Повторная проверка на случай изменения состояния
            if (pendingPos != 0) {
                // Определение направления движения
                bool dir = pendingPos > 0;
                setDirection(dir);
                
                // Формирование шагового импульса
                digitalWrite(config.stepPin, LOW);
                
                // Обновление позиций
                int delta = dir ? 1 : -1;
                pendingPos -= delta;
                
                // Обновление позиции инструмента с учетом люфта
                // При движении вперед: инструмент движется когда двигатель догоняет позицию
                // При движении назад: инструмент движется когда двигатель отстает на величину люфта
                if (dir && motorPos >= pos) {
                    pos++;
                } else if (!dir && motorPos <= (pos - backlashSteps)) {
                    pos--;
                }
                
                // Обновление позиции двигателя и глобальной позиции
                motorPos += delta;
                posGlobal += delta;
                
                // Управление ускорением/замедлением
                // Ускоряемся при непрерывном движении или далеко от цели
                // Замедляемся при приближении к конечной позиции
                bool accelerate = continuous || 
                                 pendingPos >= decelerateSteps || 
                                 pendingPos <= -decelerateSteps;
                
                speed += (accelerate ? 1 : -1) * acceleration * delayUs / 1000000.0;
                
                // Ограничение скорости
                if (speed > speedMax) {
                    speed = speedMax;
                } else if (speed < config.speedStart) {
                    speed = config.speedStart;
                }
                
                // Запоминаем время шага
                stepStartUs = nowUs;
                
                // Завершение импульса
                digitalWrite(config.stepPin, HIGH);
                
                // Логирование движения (только при значительных изменениях)
                if (abs(pendingPos) % 100 == 0) {
                    LOG_MOTION("Ось " + String(config.name), 
                              "Позиция: " + String(pos) + 
                              ", Цель: " + String(pendingPos) + 
                              ", Скорость: " + String(speed) + " шаг/сек");
                }
            }
            xSemaphoreGive(mutex);
        }
    }
    /**
     * @brief Включение/выключение драйвера оси
     * @param enable true - включить, false - выключить
     * 
     * Использует счетчик включений чтобы несколько модулей могли независимо
     * управлять включением драйвера. Драйвер включен когда счетчик > 0.
     */
    void setEnabled(bool enable) {
        if (!config.needsRest || !config.active) {
            return; // Драйвер не требует управления включением или ось не активна
        }
        
        if (enable) {
            stepperEnableCounter++;
            if (stepperEnableCounter == 1) {
                // Первое включение - активируем драйвер
                updateEnablePin();
                // Задержка для инициализации драйвера перед подачей импульсов
                delay(STEPPED_ENABLE_DELAY_MS);
            }
        } else if (stepperEnableCounter > 0) {
            stepperEnableCounter--;
            if (stepperEnableCounter == 0) {
                // Последнее выключение - деактивируем драйвер
                updateEnablePin();
            }
        }
        
        LOG_DEBUG("Ось " + String(config.name), 
                 enable ? "Включена (счетчик: " + String(stepperEnableCounter) + ")" :
                         "Выключена (счетчик: " + String(stepperEnableCounter) + ")");
    }
    
    /**
     * @brief Установка левого предела перемещения
     * @param stopPos Позиция предела в шагах (LONG_MAX для сброса предела)
     */
    void setLeftStop(long stopPos) {
        long oldStop = leftStop;
        leftStop = stopPos;
        if (oldStop != stopPos) {
            if (stopPos == LONG_MAX) {
                LOG_INFO("Ось " + String(config.name), "Левый упор сброшен");
            } else {
                LOG_INFO("Ось " + String(config.name), "Левый упор установлен: " + String(stopPos));
            }
        }
    }
    
    /**
     * @brief Установка правого предела перемещения
     * @param stopPos Позиция предела в шагах (LONG_MIN для сброса предела)
     */
    void setRightStop(long stopPos) {
        long oldStop = rightStop;
        rightStop = stopPos;
        if (oldStop != stopPos) {
            if (stopPos == LONG_MIN) {
                LOG_INFO("Ось " + String(config.name), "Правый упор сброшен");
            } else {
                LOG_INFO("Ось " + String(config.name), "Правый упор установлен: " + String(stopPos));
            }
        }
    }
    
    /**
     * @brief Установка нулевой позиции в текущее положение
     * 
     * Сдвигает систему координат так, что текущая позиция становится нулем.
     * Соответственно сдвигаются и установленные пределы перемещения.
     */
    void setOrigin() {
        if (xSemaphoreTake(mutex, 10) != pdTRUE) {
            LOG_ERROR("Ось " + String(config.name), "Не удалось установить ноль - мьютекс занят");
            return;
        }
        
        // Сдвиг пределов относительно новой нулевой позиции
        if (leftStop != LONG_MAX) {
            leftStop -= pos;
        }
        if (rightStop != LONG_MIN) {
            rightStop -= pos;
        }
        
        // Корректировка позиций двигателя
        motorPos -= pos;
        originPos += pos;
        
        // Сброс текущей позиции в ноль
        pos = 0;
        fractionalPos = 0;
        pendingPos = 0;
        
        xSemaphoreGive(mutex);
        LOG_INFO("Ось " + String(config.name), "Нулевая позиция установлена в текущее положение");
    }
    
    /**
     * @brief Сброс нулевой позиции (абсолютный ноль)
     * 
     * Устанавливает ноль в текущую позицию без сдвига системы координат.
     */
    void resetOrigin() {
        originPos = -pos;
        LOG_INFO("Ось " + String(config.name), "Абсолютный ноль установлен");
    }
    
    /**
     * @brief Получение текущей позиции в деци-микронах
     * @return Позиция в деци-микронах (0.0001 мм)
     */
    long getPositionDu() const {
        return round(pos * config.screwPitch / config.motorSteps);
    }
    
    /**
     * @brief Получение текущей позиции в шагах
     * @return Позиция в шагах
     */
    long getPositionSteps() const { 
        return pos; 
    }
    
    /**
     * @brief Проверка движения оси
     * @return true если ось движется (есть ожидающие шаги или недавно был шаг)
     */
    bool isMoving() const { 
        return pendingPos != 0 || (micros() - stepStartUs < 50000); 
    }
    
    /**
     * @brief Проверка достижения целевой позиции
     * @param tolerance Допуск в шагах
     * @return true если позиция достигнута в пределах допуска
     */
    bool isTargetReached(int tolerance = 0) const { 
        return abs(pendingPos) <= tolerance; 
    }
    
    /**
     * @brief Установка максимальной скорости
     * @param maxSpeed Максимальная скорость в шагах/секунду
     */
    void setMaxSpeed(long maxSpeed) { 
        speedMax = maxSpeed; 
    }
    
    /**
     * @brief Сброс максимальной скорости до значения по умолчанию
     */
    void resetMaxSpeed() { 
        speedMax = config.speedManualMove; 
    }
    
    // Геттеры для конфигурации и состояния
    char getName() const { return config.name; }
    bool isActive() const { return config.active; }
    bool isRotational() const { return config.rotational; }
    bool isDisabled() const { return disabled; }
    long getLeftStop() const { return leftStop; }
    long getRightStop() const { return rightStop; }
    long getMotorPos() const { return motorPos; }
    long getOriginPos() const { return originPos; }
    long getPosGlobal() const { return posGlobal; }

private:
    /**
     * @brief Установка направления движения
     * @param dir Направление (true - прямое, false - обратное)
     * 
     * При смене направления сбрасывает скорость до начальной для безопасного
     * разгона. Учитывает настройку инвертирования направления.
     */
    void setDirection(bool dir) {
        if (direction != dir || !directionInitialized) {
            // Сброс скорости при смене направления
            speed = config.speedStart;
            direction = dir;
            directionInitialized = true;
            
            // Установка направления с учетом инвертирования
            digitalWrite(config.dirPin, dir ^ config.invertStepper);
            
            // Задержка для стабилизации сигнала направления в драйвере
            delayMicroseconds(DIRECTION_SETUP_DELAY_US);
            
            LOG_DEBUG("Ось " + String(config.name), 
                     "Направление: " + String(dir ? "Вперед" : "Назад") +
                     (config.invertStepper ? " (инвертировано)" : ""));
        }
    }
    
    /**
     * @brief Обновление состояния пина Enable драйвера
     * 
     * Включает драйвер если ось не отключена и есть запросы на включение.
     * Выключает драйвер если ось отключена или нет запросов на включение.
     */
    void updateEnablePin() {
        if (!disabled && (!config.needsRest || stepperEnableCounter > 0)) {
            digitalWrite(config.enaPin, HIGH);
        } else {
            digitalWrite(config.enaPin, LOW);
        }
    }
};

#endif // AXIS_CONTROLLER_H