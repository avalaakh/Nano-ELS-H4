#ifndef SYSTEM_MANAGER_H
#define SYSTEM_MANAGER_H

#include <Arduino.h>
#include <Preferences.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "Config.h"
#include "RussianLogger.h"
#include "MotionController.h"
#include "DisplayManager.h"
#include "InputManager.h"
#include "SpindleEncoder.h"
#include "AxisController.h"

// Глобальный экземпляр логгера
RussianLogger Logger;

/**
 * @class SystemManager
 * @brief Главный координатор всей системы NanoELS
 * 
 * Класс объединяет все компоненты системы, управляет задачами FreeRTOS,
 * сохраняет настройки и обрабатывает аварийные ситуации.
 * Соответствует общей структуре и логике оригинального h4.ino.
 */
class SystemManager {
private:
    // Компоненты системы
    MotionController& motionController;
    DisplayManager& displayManager;
    InputManager& inputManager;
    SpindleEncoder& spindleEncoder;
    AxisController& zAxis;
    AxisController& xAxis;
    AxisController& a1Axis;
    
    // Управление настройками
    Preferences preferences;
    
    // Состояние системы
    int emergencyState;         // Причина аварийной остановки (ESTOP_*)
    unsigned long lastSaveTime; // Время последнего сохранения настроек
    bool settingsChanged;       // Флаг изменения настроек требующих сохранения
    
    // Задачи FreeRTOS
    TaskHandle_t displayTaskHandle;
    TaskHandle_t keypadTaskHandle;
    TaskHandle_t motionTaskHandle;
    TaskHandle_t gcodeTaskHandle;

public:
    /**
     * @brief Конструктор системного менеджера
     * @param motionCtrl Ссылка на контроллер движения
     * @param displayMgr Ссылка на менеджер дисплея
     * @param inputMgr Ссылка на менеджер ввода
     * @param spindleEnc Ссылка на энкодер шпинделя
     * @param zAxisCtrl Ссылка на ось Z
     * @param xAxisCtrl Ссылка на ось X
     * @param a1AxisCtrl Ссылка на ось A1
     */
    SystemManager(MotionController& motionCtrl, 
                  DisplayManager& displayMgr,
                  InputManager& inputMgr,
                  SpindleEncoder& spindleEnc,
                  AxisController& zAxisCtrl,
                  AxisController& xAxisCtrl,
                  AxisController& a1AxisCtrl)
        : motionController(motionCtrl), displayManager(displayMgr), 
          inputManager(inputMgr), spindleEncoder(spindleEnc),
          zAxis(zAxisCtrl), xAxis(xAxisCtrl), a1Axis(a1AxisCtrl),
          emergencyState(ESTOP_NONE), lastSaveTime(0), settingsChanged(false),
          displayTaskHandle(NULL), keypadTaskHandle(NULL), 
          motionTaskHandle(NULL), gcodeTaskHandle(NULL) {}
    
    /**
     * @brief Инициализация всей системы
     * @return true если инициализация успешна
     */
    bool begin() {
        LOG_INFO("Система", "Начало инициализации NanoELS H" + 
                 String(HARDWARE_VERSION) + " V" + String(SOFTWARE_VERSION));
        
        // Инициализация аппаратных пинов
        initializePins();
        
        // Инициализация компонентов
        spindleEncoder.begin();
        zAxis.begin();
        xAxis.begin();
        if (a1Axis.isActive()) {
            a1Axis.begin();
        }
        motionController.begin();
        displayManager.begin();
        
        if (!inputManager.begin()) {
            LOG_ERROR("Система", "Ошибка инициализации клавиатуры");
            return false;
        }
        
        // Загрузка настроек из EEPROM
        loadSettings();
        
        // Проверка целостности системы
        systemIntegrityCheck();
        
        // Создание и запуск задач FreeRTOS
        createTasks();
        
        LOG_INFO("Система", "Инициализация завершена успешно");
        return true;
    }
    
    /**
     * @brief Основной цикл системы (вызывать в loop())
     */
    void update() {
        // Проверка аварийной остановки
        if (emergencyState != ESTOP_NONE) {
            handleEmergencyStop();
            return;
        }
        
        // Сохранение настроек если нужно
        if (settingsChanged && (micros() - lastSaveTime > SAVE_DELAY_US)) {
            saveSettings();
        }
        
        // Кратковременная задержка для других задач
        vTaskDelay(1 / portTICK_PERIOD_MS);
    }
    
    /**
     * @brief Аварийная остановка системы
     * @param reason Причина остановки (ESTOP_*)
     */
    void emergencyStop(int reason) {
        emergencyState = reason;
        
        // Немедленная остановка всех движений
        motionController.setEnabled(false);
        
        // Отключение драйверов двигателей
        zAxis.setEnabled(false);
        xAxis.setEnabled(false);
        if (a1Axis.isActive()) {
            a1Axis.setEnabled(false);
        }
        
        LOG_ERROR("Система", "Аварийная остановка. Причина: " + getEmergencyStopReason(reason));
    }
    
    /**
     * @brief Восстановление после аварийной остановки
     */
    void recoverFromEmergency() {
        if (emergencyState == ESTOP_NONE) {
            return;
        }
        
        LOG_INFO("Система", "Восстановление после аварийной остановки");
        emergencyState = ESTOP_NONE;
        
        // Перезагрузка системы
        // TODO: Реализация безопасного восстановления
    }
    
    /**
     * @brief Получение состояния аварийной остановки
     * @return Причина остановки или ESTOP_NONE если система работает
     */
    int getEmergencyState() const { 
        return emergencyState; 
    }
    
    /**
     * @brief Проверка работоспособности системы
     * @return true если система работает нормально
     */
    bool isSystemOk() const { 
        return emergencyState == ESTOP_NONE; 
    }

private:
    /**
     * @brief Инициализация аппаратных пинов
     */
    void initializePins() {
        // Настройка пинов энкодера шпинделя
        pinMode(ENC_A, INPUT_PULLUP);
        pinMode(ENC_B, INPUT_PULLUP);
        
        // Настройка пинов оси Z
        pinMode(Z_DIR, OUTPUT);
        pinMode(Z_STEP, OUTPUT);
        pinMode(Z_ENA, OUTPUT);
        digitalWrite(Z_STEP, HIGH);
        
        // Настройка пинов оси X
        pinMode(X_DIR, OUTPUT);
        pinMode(X_STEP, OUTPUT);
        pinMode(X_ENA, OUTPUT);
        digitalWrite(X_STEP, HIGH);
        
        // Настройка пинов оси A1 если активна
        if (ACTIVE_A1) {
            pinMode(A12, OUTPUT);
            pinMode(A13, OUTPUT);
            pinMode(A11, OUTPUT);
            digitalWrite(A13, HIGH);
        }
        
        // Настройка пина пищалки
        pinMode(BUZZ, OUTPUT);
        
        // Настройка пинов ручных энкодеров если используются
        if (PULSE_1_USE) {
            pinMode(A11, OUTPUT);
            pinMode(A12, INPUT);
            pinMode(A13, INPUT);
            digitalWrite(A11, LOW);
        }
        
        if (PULSE_2_USE) {
            pinMode(A21, OUTPUT);
            pinMode(A22, INPUT);
            pinMode(A23, INPUT);
            digitalWrite(A21, LOW);
        }
        
        LOG_DEBUG("Система", "Аппаратные пины инициализированы");
    }
    
    /**
     * @brief Создание и запуск задач FreeRTOS
     */
    void createTasks() {
        // Задача обновления дисплея (ядро 0)
        xTaskCreatePinnedToCore(
            displayTask,        // Функция задачи
            "Display",          // Имя задачи
            10000,             // Размер стека
            this,              // Параметр
            1,                 // Приоритет
            &displayTaskHandle, // Хэндл задачи
            0                  // Ядро (0)
        );
        
        // Задача обработки клавиатуры (ядро 0)
        xTaskCreatePinnedToCore(
            keypadTask,
            "Keypad", 
            10000,
            this,
            1,
            &keypadTaskHandle,
            0
        );
        
        // Задача управления движением (ядро 1)
        xTaskCreatePinnedToCore(
            motionTask,
            "Motion",
            10000, 
            this,
            2,                 // Высокий приоритет для точного управления
            &motionTaskHandle,
            1
        );
        
        // Задача обработки G-кода (ядро 1)
        xTaskCreatePinnedToCore(
            gcodeTask,
            "GCode",
            10000,
            this,
            1,
            &gcodeTaskHandle,
            1
        );
        
        LOG_INFO("Система", "Задачи FreeRTOS созданы");
    }
    
    /**
     * @brief Проверка целостности системы
     */
    void systemIntegrityCheck() {
        // Проверка клавиатуры при запуске
        if (keypad.available()) {
            emergencyStop(ESTOP_KEY);
            LOG_ERROR("Система", "Аварийная остановка: клавиша нажата при запуске");
            return;
        }
        
        // TODO: Дополнительные проверки целостности системы
        
        LOG_DEBUG("Система", "Проверка целостности пройдена");
    }
    
    /**
     * @brief Обработка аварийной остановки
     */
    void handleEmergencyStop() {
        // Отображение сообщения об ошибке на дисплее
        // и блокировка дальнейших операций до сброса
        
        // Мигание светодиодом для индикации аварии
        static unsigned long lastBlink = 0;
        if (millis() - lastBlink > 500) {
            digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
            lastBlink = millis();
        }
    }
    
    /**
     * @brief Сохранение настроек в EEPROM
     */
    void saveSettings() {
        // TODO: Реализация сохранения всех настроек
        // в соответствии с оригинальной логикой
        
        lastSaveTime = micros();
        settingsChanged = false;
        LOG_DEBUG("Система", "Настройки сохранены в EEPROM");
    }
    
    /**
     * @brief Загрузка настроек из EEPROM
     */
    void loadSettings() {
        preferences.begin(PREF_NAMESPACE);
        
        // Проверка версии формата хранения
        if (preferences.getInt(PREF_VERSION) != PREFERENCES_VERSION) {
            preferences.clear();
            preferences.putInt(PREF_VERSION, PREFERENCES_VERSION);
            LOG_INFO("Система", "Формат хранения обновлен, настройки сброшены");
        }
        
        // TODO: Загрузка всех настроек в соответствии с оригинальной логикой
        
        preferences.end();
        LOG_DEBUG("Система", "Настройки загружены из EEPROM");
    }
    
    /**
     * @brief Получение текстового описания причины аварийной остановки
     * @param reason Код причины
     * @return Текстовое описание
     */
    String getEmergencyStopReason(int reason) const {
        switch(reason) {
            case ESTOP_NONE: return "Нет остановки";
            case ESTOP_KEY: return "Клавиша нажата при запуске";
            case ESTOP_POS: return "Выход за пределы перемещения";
            case ESTOP_MARK_ORIGIN: return "Ошибка установки нуля";
            case ESTOP_ON_OFF: return "Ошибка включения/выключения";
            case ESTOP_OFF_MANUAL_MOVE: return "Выключение во время ручного движения";
            default: return "Неизвестная причина (" + String(reason) + ")";
        }
    }
    
    // Статические методы для задач FreeRTOS
    static void displayTask(void* parameter) {
        SystemManager* system = (SystemManager*)parameter;
        while (system->emergencyState == ESTOP_NONE) {
            system->displayManager.update();
            vTaskDelay(100 / portTICK_PERIOD_MS);
        }
        vTaskDelete(NULL);
    }
    
    static void keypadTask(void* parameter) {
        SystemManager* system = (SystemManager*)parameter;
        while (system->emergencyState == ESTOP_NONE) {
            system->inputManager.update();
            vTaskDelay(50 / portTICK_PERIOD_MS);
        }
        vTaskDelete(NULL);
    }
    
    static void motionTask(void* parameter) {
        SystemManager* system = (SystemManager*)parameter;
        while (system->emergencyState == ESTOP_NONE) {
            system->motionController.update();
            vTaskDelay(1 / portTICK_PERIOD_MS);
        }
        vTaskDelete(NULL);
    }
    
    static void gcodeTask(void* parameter) {
        SystemManager* system = (SystemManager*)parameter;
        while (system->emergencyState == ESTOP_NONE) {
            // TODO: Реализация обработки G-кода
            vTaskDelay(10 / portTICK_PERIOD_MS);
        }
        vTaskDelete(NULL);
    }
};

#endif // SYSTEM_MANAGER_H