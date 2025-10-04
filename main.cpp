#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <LiquidCrystal.h>
#include <Preferences.h>
#include <Adafruit_TCA8418.h>
#include <driver/pcnt.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>

#include "Config.h"
#include "RussianLogger.h"
#include "SpindleEncoder.h"
#include "AxisController.h"
#include "MotionController.h"
#include "DisplayManager.h"
#include "InputManager.h"
#include "SystemManager.h"

// =============================================================================
// СОЗДАНИЕ ГЛОБАЛЬНЫХ ОБЪЕКТОВ СИСТЕМЫ
// =============================================================================

// Аппаратные объекты
LiquidCrystal lcd(21, 48, 47, 38, 39, 40, 41, 42, 2, 1);
Adafruit_TCA8418 keypad;

// Компоненты системы
SpindleEncoder spindleEncoder;
AxisController zAxis(NAME_Z, true, false, MOTOR_STEPS_Z, SCREW_Z_DU, SPEED_START_Z, 
                    SPEED_MANUAL_MOVE_Z, ACCELERATION_Z, INVERT_Z, NEEDS_REST_Z,
                    MAX_TRAVEL_MM_Z, BACKLASH_DU_Z, Z_ENA, Z_DIR, Z_STEP);

AxisController xAxis(NAME_X, true, false, MOTOR_STEPS_X, SCREW_X_DU, SPEED_START_X,
                    SPEED_MANUAL_MOVE_X, ACCELERATION_X, INVERT_X, NEEDS_REST_X,
                    MAX_TRAVEL_MM_X, BACKLASH_DU_X, X_ENA, X_DIR, X_STEP);

AxisController a1Axis(NAME_A1, ACTIVE_A1, ROTARY_A1, MOTOR_STEPS_A1, SCREW_A1_DU,
                     SPEED_START_A1, SPEED_MANUAL_MOVE_A1, ACCELERATION_A1, INVERT_A1,
                     NEEDS_REST_A1, MAX_TRAVEL_MM_A1, BACKLASH_DU_A1, A11, A12, A13);

MotionController motionController(spindleEncoder, zAxis, xAxis, a1Axis);
DisplayManager displayManager(lcd, motionController);
InputManager inputManager(keypad, motionController);
SystemManager systemManager(motionController, displayManager, inputManager, 
                           spindleEncoder, zAxis, xAxis, a1Axis);

// =============================================================================
// ФУНКЦИИ ARDUINO
// =============================================================================

void setup() {
    // Инициализация последовательного порта для отладки
    Serial.begin(115200);
    Serial.println("NanoELS H4 - Запуск системы...");
    
    // Инициализация системы
    if (!systemManager.begin()) {
        Serial.println("ОШИБКА: Инициализация системы не удалась!");
        
        // Бесконечный цикл с миганием светодиода при ошибке
        while (true) {
            digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
            delay(500);
        }
    }
    
    Serial.println("Система NanoELS успешно запущена и готова к работе");
}

void loop() {
    // Основной цикл системы - управление осуществляется в задачах FreeRTOS
    // Здесь только минимальная логика для мониторинга
    systemManager.update();
    
    // Служебные задачи Arduino
    yield();
}