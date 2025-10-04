#ifndef INPUT_MANAGER_H
#define INPUT_MANAGER_H

#include <Arduino.h>
#include <Adafruit_TCA8418.h>
#include "Config.h"
#include "RussianLogger.h"
#include "MotionController.h"
#include "AxisController.h"

/**
 * @class InputManager
 * @brief Управление клавиатурой, обработка ввода и навигация по меню
 * 
 * Класс обрабатывает все события от кнопок, реализует числовой ввод,
 * навигацию по меню и преобразует аппаратные события в логические команды.
 * Соответствует оригинальной логике обработки ввода из h4.ino.
 */
class InputManager {
private:
    Adafruit_TCA8418& keypad;          // Ссылка на объект клавиатуры TCA8418
    MotionController& motionController; // Ссылка на контроллер движения
    
    // Состояние числового ввода
    int numpadDigits[8];                // Буфер для введенных цифр
    int numpadIndex;                    // Текущий индекс в буфере
    bool inNumpadMode;                  // Активен ли режим числового ввода
    
    // Флаги нажатия кнопок (для длительного нажатия)
    bool leftPressed;                   // Кнопка ВЛЕВО нажата
    bool rightPressed;                  // Кнопка ВПРАВО нажата  
    bool upPressed;                     // Кнопка ВВЕРХ нажата
    bool downPressed;                   // Кнопка ВНИЗ нажата
    bool offPressed;                    // Кнопка ВЫКЛ нажата
    bool gearsPressed;                  // Кнопка MODE_GEARS нажата
    bool turnPressed;                   // Кнопка MODE_TURN нажата
    
    // Тайминги
    unsigned long lastKeypadTime;       // Время последней обработки клавиатуры
    unsigned long resetPressTime;       // Время нажатия кнопки ВЫКЛ для сброса
    
    // Текущее состояние меню и настройки
    int setupWizardIndex;               // Текущий шаг мастера настройки
    bool auxDirectionForward;           // Направление вспомогательной оси
    int gcodeProgramIndex;              // Индекс текущей программы G-кода
    int gcodeProgramCount;              // Общее число программ G-кода

public:
    /**
     * @brief Конструктор менеджера ввода
     * @param keypadRef Ссылка на объект клавиатуры TCA8418
     * @param motionCtrlRef Ссылка на контроллер движения
     */
    InputManager(Adafruit_TCA8418& keypadRef, MotionController& motionCtrlRef)
        : keypad(keypadRef), motionController(motionCtrlRef), numpadIndex(0), 
          inNumpadMode(false), leftPressed(false), rightPressed(false),
          upPressed(false), downPressed(false), offPressed(false),
          gearsPressed(false), turnPressed(false), lastKeypadTime(0),
          resetPressTime(0), setupWizardIndex(0), auxDirectionForward(true),
          gcodeProgramIndex(0), gcodeProgramCount(0) {
        
        // Инициализация буфера числового ввода
        for (int i = 0; i < 8; i++) {
            numpadDigits[i] = 0;
        }
    }
    
    /**
     * @brief Инициализация клавиатуры
     * @return true если клавиатура обнаружена и инициализирована
     */
    bool begin() {
        if (!keypad.begin(TCA8418_DEFAULT_ADDR, &Wire)) {
            LOG_ERROR("Клавиатура", "Контроллер TCA8418 не обнаружен!");
            return false;
        }
        
        keypad.matrix(7, 7);
        keypad.flush();
        
        LOG_INFO("Клавиатура", "Инициализирована успешно");
        return true;
    }
    
    /**
     * @brief Обновление состояния ввода (вызывать периодически)
     * 
     * Обрабатывает события клавиатуры, обновляет состояние кнопок
     * и выполняет соответствующие действия.
     */
    void update() {
        // Обработка событий клавиатуры
        int event = 0;
        if (keypad.available() > 0) {
            event = keypad.getEvent();
        }
        
        if (event == 0) return;
        
        // Извлечение кода кнопки и типа события
        int keyCode = event;
        bitWrite(keyCode, 7, 0); // Убираем бит события
        bool isPress = bitRead(event, 7) == 1; // 1 - нажатие, 0 - отпускание
        
        lastKeypadTime = micros();
        
        // Обработка события кнопки
        handleButtonEvent(keyCode, isPress);
    }
    
    /**
     * @brief Получение результата числового ввода
     * @return Введенное число
     */
    long getNumpadResult() const {
        long result = 0;
        for (int i = 0; i < numpadIndex; i++) {
            result += numpadDigits[i] * pow(10, numpadIndex - 1 - i);
        }
        return result;
    }
    
    /**
     * @brief Сброс числового ввода
     */
    void resetNumpad() {
        numpadIndex = 0;
        inNumpadMode = false;
        for (int i = 0; i < 8; i++) {
            numpadDigits[i] = 0;
        }
    }
    
    /**
     * @brief Проверка активности числового ввода
     * @return true если активен режим числового ввода
     */
    bool isNumpadActive() const { 
        return inNumpadMode; 
    }
    
    // Геттеры для состояния кнопок (для задач движения)
    bool isLeftPressed() const { return leftPressed; }
    bool isRightPressed() const { return rightPressed; }
    bool isUpPressed() const { return upPressed; }
    bool isDownPressed() const { return downPressed; }
    bool isGearsPressed() const { return gearsPressed; }
    bool isTurnPressed() const { return turnPressed; }

private:
    /**
     * @brief Обработка события нажатия кнопки
     * @param keyCode Код кнопки из Config.h
     * @param isPress true - нажатие, false - отпускание
     */
    void handleButtonEvent(int keyCode, bool isPress) {
        // Кнопка ВЫКЛ всегда обрабатывается отдельно
        if (keyCode == B_OFF) {
            offPressed = isPress;
            if (isPress) {
                handleOnOff(false); // Нажатие - выключение
                resetPressTime = millis(); // Запоминаем время нажатия для сброса
            } else {
                handleOffRelease(); // Отпускание - проверка сброса
            }
        }
        
        // В режиме G-кода блокируем другие кнопки кроме ВЫКЛ
        if (motionController.getOperationMode() == MODE_GCODE && 
            motionController.isEnabled() && keyCode != B_OFF) {
            if (isPress) {
                // Подаем звуковой сигнал о невозможности действия
                // tone(BUZZ, 1000, 100);
                LOG_WARNING("Клавиатура", "Кнопка заблокирована в режиме G-кода");
            }
            return;
        }
        
        // Обработка числового ввода (имеет приоритет)
        if (isPress && processNumpadInput(keyCode)) {
            return; // Событие обработано как числовой ввод
        }
        
        // Обработка навигации в мастере настройки
        if (isPress && handleWizardNavigation(keyCode)) {
            return; // Событие обработано как навигация
        }
        
        // Обновление флагов кнопок движения
        if (keyCode == B_LEFT) {
            leftPressed = isPress;
        } else if (keyCode == B_RIGHT) {
            rightPressed = isPress;
        } else if (keyCode == B_UP) {
            upPressed = isPress;
        } else if (keyCode == B_DOWN) {
            downPressed = isPress;
        } else if (keyCode == B_MODE_GEARS) {
            gearsPressed = isPress;
        } else if (keyCode == B_MODE_TURN) {
            turnPressed = isPress;
        }
        
        // Для остальных кнопок обрабатываем только нажатия
        if (!isPress) {
            return;
        }
        
        // Обработка функциональных кнопок
        switch(keyCode) {
            case B_PLUS:
                handlePlusMinus(true);
                break;
            case B_MINUS:
                handlePlusMinus(false);
                break;
            case B_ON:
                handleOnOff(true);
                break;
            case B_STOPL:
                handleLimitButton(zAxis, true);
                break;
            case B_STOPR:
                handleLimitButton(zAxis, false);
                break;
            case B_STOPU:
                handleLimitButton(xAxis, true);
                break;
            case B_STOPD:
                handleLimitButton(xAxis, false);
                break;
            case B_MODE_OTHER:
                handleModeChange();
                break;
            case B_DISPL:
                handleDisplayToggle();
                break;
            case B_X:
                // Установка нуля оси X
                xAxis.setOrigin();
                break;
            case B_Z:
                // Установка нуля оси Z
                zAxis.setOrigin();
                break;
            case B_A:
                // Включение/выключение оси X
                // xAxis.setDisabled(!xAxis.isDisabled());
                break;
            case B_B:
                // Включение/выключение оси Z
                // zAxis.setDisabled(!zAxis.isDisabled());
                break;
            case B_STEP:
                handleStepChange();
                break;
            case B_REVERSE:
                handleReverse();
                break;
            case B_MEASURE:
                handleMeasureChange();
                break;
            case B_MODE_GEARS:
                if (motionController.getOperationMode() != MODE_A1) {
                    motionController.setOperationMode(MODE_NORMAL);
                }
                break;
            case B_MODE_TURN:
                if (motionController.getOperationMode() != MODE_A1) {
                    motionController.setOperationMode(MODE_TURN);
                }
                break;
            case B_MODE_FACE:
                if (motionController.getOperationMode() == MODE_A1) {
                    handleLimitButton(a1Axis, false);
                } else {
                    motionController.setOperationMode(MODE_FACE);
                }
                break;
            case B_MODE_CONE:
                if (motionController.getOperationMode() == MODE_A1) {
                    handleLimitButton(a1Axis, true);
                } else {
                    motionController.setOperationMode(MODE_CONE);
                }
                break;
            case B_MODE_CUT:
                if (motionController.getOperationMode() == MODE_A1) {
                    // Включение/выключение оси A1
                    // a1Axis.setDisabled(!a1Axis.isDisabled());
                } else {
                    motionController.setOperationMode(MODE_CUT);
                }
                break;
            case B_MODE_THREAD:
                if (motionController.getOperationMode() == MODE_A1 || 
                    (motionController.getOperationMode() == MODE_GCODE && ACTIVE_A1)) {
                    a1Axis.setOrigin();
                } else {
                    motionController.setOperationMode(MODE_THREAD);
                }
                break;
        }
        
        LOG_DEBUG("Клавиатура", "Обработана кнопка: " + getButtonName(keyCode));
    }
    
    /**
     * @brief Обработка числового ввода
     * @param keyCode Код цифровой кнопки
     * @return true если событие обработано как числовой ввод
     */
    bool processNumpadInput(int keyCode) {
        // Обработка цифровых кнопок
        if (keyCode >= B_0 && keyCode <= B_9) {
            int digit = -1;
            switch(keyCode) {
                case B_0: digit = 0; break;
                case B_1: digit = 1; break;
                case B_2: digit = 2; break;
                case B_3: digit = 3; break;
                case B_4: digit = 4; break;
                case B_5: digit = 5; break;
                case B_6: digit = 6; break;
                case B_7: digit = 7; break;
                case B_8: digit = 8; break;
                case B_9: digit = 9; break;
            }
            
            if (digit != -1) {
                numpadPress(digit);
                inNumpadMode = true;
                return true;
            }
        }
        
        // Обработка BACKSPACE
        if (keyCode == B_BACKSPACE) {
            numpadBackspace();
            inNumpadMode = true;
            return true;
        }
        
        // Обработка +/- в режиме числового ввода
        if (inNumpadMode && (keyCode == B_PLUS || keyCode == B_MINUS)) {
            numpadPlusMinus(keyCode == B_PLUS);
            return true;
        }
        
        // Если был активен числовой ввод и нажата другая кнопка - обрабатываем результат
        if (inNumpadMode) {
            inNumpadMode = false;
            return processNumpadResult(keyCode);
        }
        
        return false;
    }
    
    /**
     * @brief Обработка результата числового ввода
     * @param keyCode Код кнопки подтверждения
     * @return true если ввод обработан
     */
    bool processNumpadResult(int keyCode) {
        long newDu = numpadToDeciMicrons();
        float newConeRatio = numpadToConeRatio();
        long numpadResult = getNumpadResult();
        
        resetNumpad();
        
        // Подтверждение ввода кнопкой ВКЛ
        if (keyCode == B_ON) {
            if (isPassMode() && setupWizardIndex == 1) {
                motionController.setTurnPasses(min(PASSES_MAX, (int)numpadResult));
                setupWizardIndex++;
            } else if (motionController.getOperationMode() == MODE_CONE && setupWizardIndex == 1) {
                motionController.setConeRatio(newConeRatio);
                setupWizardIndex++;
            } else {
                if (abs(newDu) <= DUPR_MAX) {
                    motionController.setPitch(newDu);
                }
            }
            return true; // Не использовать это нажатие ВКЛ для запуска движения
        }
        
        // TODO: Реализация обработки числового ввода для установки пределов,
        // перемещения осей и других функций в соответствии с оригиналом
        
        return false;
    }
    
    /**
     * @brief Добавление цифры в буфер числового ввода
     * @param digit Цифра от 0 до 9
     */
    void numpadPress(int digit) {
        if (!inNumpadMode) {
            numpadIndex = 0;
        }
        
        if (numpadIndex < 8) {
            numpadDigits[numpadIndex] = digit;
            numpadIndex++;
        } else {
            // Буфер переполнен - сдвигаем
            for (int i = 0; i < 7; i++) {
                numpadDigits[i] = numpadDigits[i + 1];
            }
            numpadDigits[7] = digit;
        }
        
        LOG_DEBUG("Клавиатура", "Введена цифра: " + String(digit) + 
                 ", Буфер: " + String(getNumpadResult()));
    }
    
    /**
     * @brief Удаление последней введенной цифры
     */
    void numpadBackspace() {
        if (inNumpadMode && numpadIndex > 0) {
            numpadIndex--;
            LOG_DEBUG("Клавиатура", "Удалена цифра, Буфер: " + String(getNumpadResult()));
        }
    }
    
    /**
     * @brief Изменение последней введенной цифры
     * @param plus true - увеличить, false - уменьшить
     */
    void numpadPlusMinus(bool plus) {
        if (numpadIndex > 0) {
            if (numpadDigits[numpadIndex - 1] < 9 && plus) {
                numpadDigits[numpadIndex - 1]++;
            } else if (numpadDigits[numpadIndex - 1] > 1 && !plus) {
                numpadDigits[numpadIndex - 1]--;
            }
            // TODO: Реализация перехода через 9 и ниже 1
            LOG_DEBUG("Клавиатура", "Изменена цифра, Буфер: " + String(getNumpadResult()));
        }
    }
    
    /**
     * @brief Преобразование числового ввода в деци-микроны
     * @return Значение в деци-микронах
     */
    long numpadToDeciMicrons() const {
        long result = getNumpadResult();
        if (result == 0) {
            return 0;
        }
        
        // TODO: Реализация преобразования в зависимости от системы измерений
        // в соответствии с оригинальной логикой
        
        return result * 10; // Временная реализация для метрической системы
    }
    
    /**
     * @brief Преобразование числового ввода в коэффициент конуса
     * @return Коэффициент соотношения осей
     */
    float numpadToConeRatio() const {
        return getNumpadResult() / 100000.0;
    }
    
    /**
     * @brief Обработка кнопок +/- (изменение шага/заходов)
     * @param isPlus true - увеличение, false - уменьшение
     */
    void handlePlusMinus(bool isPlus) {
        // TODO: Реализация обработки +/- в соответствии с оригинальной логикой
        // - Изменение шага резьбы
        // - Изменение числа заходов в режиме резьбы
        // - Изменение числа проходов в режимах точения
        // - Регулировка коэффициента конуса
        
        LOG_DEBUG("Клавиатура", "Обработано " + String(isPlus ? "ПЛЮС" : "МИНУС"));
    }
    
    /**
     * @brief Обработка кнопки ВКЛ/ВЫКЛ
     * @param isOn true - включение, false - выключение
     */
    void handleOnOff(bool isOn) {
        resetPressTime = millis();
        
        // Проверка условий для включения
        if (!motionController.isEnabled() && isOn) {
            bool missingZStops = needZStops() && 
                                (zAxis.getLeftStop() == LONG_MAX || 
                                 zAxis.getRightStop() == LONG_MIN);
            
            if (isPassMode() && (missingZStops || 
                                xAxis.getLeftStop() == LONG_MAX || 
                                xAxis.getRightStop() == LONG_MIN)) {
                // tone(BUZZ, 1000, 500);
                LOG_WARNING("Клавиатура", "Нельзя включить - не установлены упоры");
                return;
            }
            
            if (!motionController.isEnabled() && isOn && setupWizardIndex < getLastSetupIndex()) {
                // Переход к следующему шагу мастера настройки
                setupWizardIndex++;
                LOG_DEBUG("Клавиатура", "Переход к шагу мастера: " + String(setupWizardIndex));
                return;
            }
        }
        
        // Включение/выключение системы
        motionController.setEnabled(isOn);
    }
    
    /**
     * @brief Обработка отпускания кнопки ВЫКЛ (сброс системы)
     */
    void handleOffRelease() {
        if (millis() - resetPressTime > 3000) {
            // Сброс системы при длительном нажатии
            resetSystem();
            LOG_INFO("Клавиатура", "Выполнен сброс системы");
        }
    }
    
    /**
     * @brief Обработка кнопок установки пределов
     * @param axis Ось для установки предела
     * @param isLeftLimit true - левый предел, false - правый предел
     */
    void handleLimitButton(AxisController& axis, bool isLeftLimit) {
        if (isLeftLimit) {
            axis.setLeftStop(axis.getLeftStop() == LONG_MAX ? axis.getPositionSteps() : LONG_MAX);
        } else {
            axis.setRightStop(axis.getRightStop() == LONG_MIN ? axis.getPositionSteps() : LONG_MIN);
        }
    }
    
    /**
     * @brief Обработка смены режима отображения
     */
    void handleDisplayToggle() {
        // Переключение между отображением угла, оборотов и другой информации
        // Реализация в DisplayManager
        LOG_DEBUG("Клавиатура", "Смена режима отображения");
    }
    
    /**
     * @brief Обработка смены шага перемещения
     */
    void handleStepChange() {
        // Циклическое переключение между шагами перемещения
        // в метрической или дюймовой системе
        LOG_DEBUG("Клавиатура", "Смена шага перемещения");
    }
    
    /**
     * @brief Обработка смены системы измерений
     */
    void handleMeasureChange() {
        // Переключение между метрической, дюймовой и TPI системами
        LOG_DEBUG("Клавиатура", "Смена системы измерений");
    }
    
    /**
     * @brief Обработка реверса направления
     */
    void handleReverse() {
        motionController.setPitch(-motionController.getPitch());
        LOG_DEBUG("Клавиатура", "Реверс направления шага");
    }
    
    /**
     * @brief Обработка смены режима работы
     */
    void handleModeChange() {
        // Циклическое переключение между режимами работы
        int currentMode = motionController.getOperationMode();
        int newMode;
        
        if (currentMode == MODE_NORMAL) {
            newMode = ACTIVE_A1 ? MODE_A1 : MODE_ELLIPSE;
        } else if (currentMode == MODE_A1) {
            newMode = MODE_ELLIPSE;
        } else if (currentMode == MODE_ELLIPSE) {
            newMode = MODE_GCODE;
        } else if (currentMode == MODE_GCODE) {
            newMode = MODE_ASYNC;
        } else {
            newMode = MODE_NORMAL;
        }
        
        motionController.setOperationMode(newMode);
    }
    
    /**
     * @brief Обработка навигации в мастере настройки
     * @param keyCode Код кнопки навигации
     * @return true если навигация обработана
     */
    bool handleWizardNavigation(int keyCode) {
        if (setupWizardIndex == 2 && (keyCode == B_LEFT || keyCode == B_RIGHT)) {
            auxDirectionForward = !auxDirectionForward;
            motionController.setAuxDirection(auxDirectionForward);
            return true;
        }
        
        // TODO: Дополнительная логика навигации для других шагов мастера
        
        return false;
    }
    
    /**
     * @brief Проверка необходимости установки упоров для Z
     * @return true если для текущего режима требуются упоры Z
     */
    bool needZStops() {
        int mode = motionController.getOperationMode();
        return mode == MODE_TURN || mode == MODE_FACE || mode == MODE_THREAD || mode == MODE_ELLIPSE;
    }
    
    /**
     * @brief Проверка режима с проходами
     * @return true если текущий режим использует проходы
     */
    bool isPassMode() {
        int mode = motionController.getOperationMode();
        return mode == MODE_TURN || mode == MODE_FACE || mode == MODE_CUT || 
               mode == MODE_THREAD || mode == MODE_ELLIPSE;
    }
    
    /**
     * @brief Получение последнего индекса мастера настройки
     * @return Максимальный индекс шага для текущего режима
     */
    int getLastSetupIndex() {
        int mode = motionController.getOperationMode();
        if (mode == MODE_CONE || mode == MODE_GCODE) return 2;
        if (isPassMode()) return 3;
        return 0;
    }
    
    /**
     * @brief Сброс системы к настройкам по умолчанию
     */
    void resetSystem() {
        // Сброс всех настроек и состояний
        motionController.setEnabled(false);
        motionController.setOperationMode(MODE_NORMAL);
        motionController.setPitch(0);
        motionController.setStarts(1);
        setupWizardIndex = 0;
        auxDirectionForward = true;
        resetNumpad();
        
        // TODO: Сброс осей и других параметров
        
        LOG_INFO("Клавиатура", "Система сброшена к настройкам по умолчанию");
    }
    
    /**
     * @brief Перевод кода кнопки в читаемое название
     * @param keyCode Код кнопки
     * @return Строковое название кнопки
     */
    String getButtonName(int keyCode) const {
        switch(keyCode) {
            case B_LEFT: return "ВЛЕВО";
            case B_RIGHT: return "ВПРАВО";
            case B_UP: return "ВВЕРХ";
            case B_DOWN: return "ВНИЗ";
            case B_PLUS: return "ПЛЮС";
            case B_MINUS: return "МИНУС";
            case B_ON: return "ВКЛ";
            case B_OFF: return "ВЫКЛ";
            case B_STOPL: return "СТОП_ЛЕВО";
            case B_STOPR: return "СТОП_ПРАВО";
            case B_STOPU: return "СТОП_ВЕРХ";
            case B_STOPD: return "СТОП_НИЗ";
            case B_DISPL: return "ДИСПЛЕЙ";
            case B_STEP: return "ШАГ";
            case B_SETTINGS: return "НАСТРОЙКИ";
            case B_MEASURE: return "ИЗМЕРЕНИЯ";
            case B_REVERSE: return "РЕВЕРС";
            case B_MODE_GEARS: return "РЕЖИМ_ШЕСТЕРНИ";
            case B_MODE_TURN: return "РЕЖИМ_ТОЧЕНИЕ";
            case B_MODE_FACE: return "РЕЖИМ_ТОРЕЦ";
            case B_MODE_CONE: return "РЕЖИМ_КОНУС";
            case B_MODE_CUT: return "РЕЖИМ_ПРОРЕЗ";
            case B_MODE_THREAD: return "РЕЖИМ_РЕЗЬБА";
            case B_MODE_OTHER: return "РЕЖИМ_ДРУГОЙ";
            case B_X: return "ОСЬ_X";
            case B_Z: return "ОСЬ_Z";
            case B_A: return "ОСЬ_A";
            case B_B: return "ОСЬ_B";
            default: return "НЕИЗВЕСТНАЯ(" + String(keyCode) + ")";
        }
    }
};

#endif // INPUT_MANAGER_H