#ifndef MOTION_CONTROLLER_H
#define MOTION_CONTROLLER_H

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include "Config.h"
#include "RussianLogger.h"
#include "SpindleEncoder.h"
#include "AxisController.h"

/**
 * @class MotionController
 * @brief Главный координатор движения, реализующий все режимы работы NanoELS
 * 
 * Класс управляет взаимодействием между осями и энкодером шпинделя,
 * реализует различные режимы работы: резьбонарезание, точение, G-код и др.
 * Обеспечивает синхронизацию движения осей с вращением шпинделя.
 */
class MotionController {
private:
    // Компоненты системы
    SpindleEncoder& spindle;    // Энкодер шпинделя для отслеживания положения и скорости
    AxisController& zAxis;      // Основная ось Z (продольное движение)
    AxisController& xAxis;      // Ось X (поперечное движение)  
    AxisController& a1Axis;     // Дополнительная ось A1 (делительная головка)
    
    // Синхронизация доступа к общим данным
    SemaphoreHandle_t motionMutex;
    
    // Текущее состояние системы
    int currentMode;            // Текущий режим работы из Config.h
    bool systemEnabled;         // Включена ли система (обработка команд)
    long currentPitch;          // Текущий шаг резьбы в деци-микронах
    int currentStarts;          // Текущее число заходов резьбы [1, STARTS_MAX]
    
    // Переменные для автоматических режимов обработки
    int operationIndex;         // Индекс текущей операции (прохода)
    int operationSubIndex;      // Под-индекс внутри операции (этап)
    bool operationAdvanceFlag;  // Флаг запроса перехода к следующему проходу
    long operationStartPitch;   // Шаг резьбы при начале операции (для проверки изменений)
    int operationPitchSign;     // Знак шага при начале операции (1 или -1)
    
    // Настройки режимов работы
    float coneRatio;            // Коэффициент соотношения осей в режиме конуса
    int turnPasses;             // Число проходов в режимах точения
    bool auxDirectionForward;   // Направление вспомогательной оси (внешняя/внутренняя обработка)

public:
    /**
     * @brief Конструктор контроллера движения
     * @param spindleEnc Ссылка на энкодер шпинделя
     * @param zAxisCtrl Ссылка на контроллер оси Z
     * @param xAxisCtrl Ссылка на контроллер оси X
     * @param a1AxisCtrl Ссылка на контроллер оси A1
     */
    MotionController(SpindleEncoder& spindleEnc,
                    AxisController& zAxisCtrl, 
                    AxisController& xAxisCtrl,
                    AxisController& a1AxisCtrl)
        : spindle(spindleEnc), zAxis(zAxisCtrl), xAxis(xAxisCtrl), a1Axis(a1AxisCtrl),
          currentMode(0), systemEnabled(false), currentPitch(0), currentStarts(1),
          operationIndex(0), operationSubIndex(0), operationAdvanceFlag(false),
          operationStartPitch(0), operationPitchSign(1), coneRatio(1.0), turnPasses(3),
          auxDirectionForward(true) {
        
        // Создание мьютекса для синхронизации доступа к общим данным
        motionMutex = xSemaphoreCreateMutex();
        
        LOG_INFO("Контроллер", "Создан контроллер движения");
    }
    
    /**
     * @brief Инициализация контроллера движения
     * 
     * Выполняет начальную настройку системы. Должна вызываться после создания объектов осей.
     */
    void begin() {
        LOG_INFO("Контроллер", "Система управления движением инициализирована");
    }
    
    /**
     * @brief Основной цикл управления движением
     * 
     * Должен вызываться в основном цикле системы. Управляет всеми режимами работы,
     * обновляет состояние энкодера и осей, выполняет алгоритмы синхронизации.
     */
    void update() {
        // Попытка захватить мьютекс с коротким таймаутом
        if (xSemaphoreTake(motionMutex, 1) != pdTRUE) {
            return; // Мьютекс занят - пропускаем цикл
        }
        
        // Обновление состояния энкодера шпинделя
        spindle.update();
        
        // Если система выключена или шаг нулевой или есть расссинхронизация - пропускаем обработку режимов
        if (!systemEnabled || currentPitch == 0 || spindle.getSyncOffset() != 0) {
            // Режим не активен - только обновляем оси для завершения текущих движений
        } else {
            // Выбор и выполнение текущего режима работы
            switch(currentMode) {
                case MODE_NORMAL: updateNormalMode(); break;    // Обычный ELS режим (резьба)
                case MODE_ASYNC: updateAsyncMode(); break;      // Асинхронный режим
                case MODE_CONE: updateConeMode(); break;        // Коническое точение
                case MODE_TURN: updateTurnMode(); break;        // Продольное точение
                case MODE_FACE: updateFaceMode(); break;        // Подрезка торца
                case MODE_CUT: updateCutMode(); break;          // Прорезка
                case MODE_THREAD: updateThreadMode(); break;    // Нарезание резьбы
                case MODE_ELLIPSE: updateEllipseMode(); break;  // Эллиптическое точение
                case MODE_GCODE: updateGCodeMode(); break;      // Управление по G-коду
                case MODE_A1: updateA1Mode(); break;            // Управление осью A1
                default:
                    LOG_WARNING("Контроллер", "Неизвестный режим работы: " + String(currentMode));
                    break;
            }
        }
        
        // Обновление всех осей (выполнение шагов)
        zAxis.update();
        xAxis.update();
        if (a1Axis.isActive()) {
            a1Axis.update();
        }
        
        xSemaphoreGive(motionMutex);
    }
    
    /**
     * @brief Включение/выключение системы
     * @param enable true - включить, false - выключить
     * 
     * При включении выполняет инициализацию операции, при выключении - безопасную остановку.
     */
    void setEnabled(bool enable) {
        if (systemEnabled && enable) {
            return; // Уже включена
        }
        
        if (!enable) {
            // Выключение системы
            systemEnabled = false;
            operationIndex = 0;
            LOG_INFO("Контроллер", "Система выключена");
        } else {
            // Включение системы
            zAxis.setEnabled(true);
            xAxis.setEnabled(true);
            if (a1Axis.isActive()) {
                a1Axis.setEnabled(true);
            }
            
            // Установка новой точки отсчета для синхронизации
            setNewOrigin();
            
            // Инициализация переменных операции
            systemEnabled = true;
            operationPitchSign = currentPitch >= 0 ? 1 : -1;
            operationStartPitch = currentPitch;
            operationIndex = 0;
            operationAdvanceFlag = false;
            operationSubIndex = 0;
            
            LOG_INFO("Контроллер", "Система включена. Режим: " + String(currentMode) + 
                    ", Шаг: " + String(currentPitch) + " du, Заходов: " + String(currentStarts));
        }
    }
    
    /**
     * @brief Установка режима работы
     * @param mode Режим работы из Config.h (MODE_NORMAL, MODE_TURN, и т.д.)
     * 
     * При смене режима система автоматически выключается для безопасности.
     */
    void setOperationMode(int mode) {
        if (currentMode == mode) {
            return; // Режим не изменился
        }
        
        // Выключение системы при смене режима
        if (systemEnabled) {
            setEnabled(false);
        }
        
        currentMode = mode;
        operationIndex = 0;
        
        LOG_INFO("Контроллер", "Установлен режим: " + String(mode));
    }
    
    /**
     * @brief Установка шага резьбы/подачи
     * @param pitch Шаг в деци-микронах
     * 
     * При изменении шага автоматически устанавливается новая точка отсчета
     * для избежания резкого движения осей.
     */
    void setPitch(long pitch) {
        // Проверка допустимости шага
        if (pitch < -DUPR_MAX || pitch > DUPR_MAX) {
            LOG_ERROR("Контроллер", "Недопустимый шаг: " + String(pitch) + 
                     " (допустимо от " + String(-DUPR_MAX) + " до " + String(DUPR_MAX) + ")");
            return;
        }
        
        currentPitch = pitch;
        
        // Установка новой точки отсчета для синхронизации
        setNewOrigin();
        
        LOG_INFO("Контроллер", "Установлен шаг: " + String(pitch) + " du");
    }
    
    /**
     * @brief Установка числа заходов резьбы
     * @param starts Число заходов [1, STARTS_MAX]
     */
    void setStarts(int starts) {
        // Проверка допустимости числа заходов
        if (starts < 1 || starts > STARTS_MAX) {
            LOG_ERROR("Контроллер", "Недопустимое число заходов: " + String(starts) + 
                     " (допустимо от 1 до " + String(STARTS_MAX) + ")");
            return;
        }
        
        currentStarts = starts;
        
        // Установка новой точки отсчета для синхронизации
        setNewOrigin();
        
        LOG_INFO("Контроллер", "Установлено заходов: " + String(starts));
    }
    
    // Геттеры для состояния системы
    int getOperationMode() const { return currentMode; }
    bool isEnabled() const { return systemEnabled; }
    long getPitch() const { return currentPitch; }
    int getStarts() const { return currentStarts; }
    float getConeRatio() const { return coneRatio; }
    int getTurnPasses() const { return turnPasses; }
    bool getAuxDirection() const { return auxDirectionForward; }
    
    /**
     * @brief Установка коэффициента конуса
     * @param ratio Коэффициент соотношения осей
     */
    void setConeRatio(float ratio) {
        coneRatio = ratio;
        LOG_INFO("Контроллер", "Установлен коэффициент конуса: " + String(ratio, 5));
    }
    
    /**
     * @brief Установка числа проходов точения
     * @param passes Число проходов [1, PASSES_MAX]
     */
    void setTurnPasses(int passes) {
        if (passes < 1 || passes > PASSES_MAX) {
            LOG_ERROR("Контроллер", "Недопустимое число проходов: " + String(passes));
            return;
        }
        turnPasses = passes;
        LOG_INFO("Контроллер", "Установлено проходов точения: " + String(passes));
    }
    
    /**
     * @brief Установка направления вспомогательной оси
     * @param forward true - внешняя обработка, false - внутренняя обработка
     */
    void setAuxDirection(bool forward) {
        auxDirectionForward = forward;
        LOG_INFO("Контроллер", "Направление вспомогательной оси: " + 
                 String(forward ? "внешняя" : "внутренняя"));
    }
    
    /**
     * @brief Запрос перехода к следующему проходу (в автоматических режимах)
     * 
     * Позволяет оператору вручную перейти к следующему проходу без ожидания
     * завершения текущего в режимах точения, подрезки и резьбы.
     */
    void advanceOperation() {
        operationAdvanceFlag = true;
        LOG_DEBUG("Контроллер", "Запрос перехода к следующему проходу");
    }

private:
    /**
     * @brief Режим нормальной работы (резьбонарезание)
     * 
     * Синхронизирует движение оси Z с вращением шпинделя согласно установленному шагу.
     * Ось Z следует за шпинделем, создавая резьбу с заданным шагом и числом заходов.
     */
    void updateNormalMode() {
        // Если ось Z движется вручную - не вмешиваемся
        if (zAxis.isMoving()) {
            return;
        }
        
        // Расчет целевой позиции оси Z на основе позиции шпинделя
        long targetPos = calculateAxisPosition(zAxis, spindle.getAveragePosition(), true);
        
        // Если позиция изменилась - двигаем ось
        if (targetPos != zAxis.getPositionSteps()) {
            zAxis.moveTo(targetPos, true); // Непрерывное движение для плавного слежения
        }
    }
    
    /**
     * @brief Режим асинхронного движения
     * 
     * Ось движется независимо от шпинделя с постоянной скоростью.
     * Используется для быстрых перемещений без привязки к вращению.
     */
    void updateAsyncMode() {
        // В этом режиме ось Z движется самостоятельно
        // Реализация зависит от конкретных требований
        LOG_DEBUG("Контроллер", "Асинхронный режим активен");
        // Дополнительная логика будет добавлена по мере необходимости
    }
    
    /**
     * @brief Режим конического точения
     * 
     * Одновременное движение осей Z и X по заданному соотношению.
     * Используется для обработки конических поверхностей.
     */
    void updateConeMode() {
        // Проверка возможности работы
        if (zAxis.isMoving() || xAxis.isMoving() || coneRatio == 0) {
            return;
        }
        
        // Расчет соотношения движения осей Z и X
        float zToXRatio = -coneRatio / 2 / zAxis.getPositionSteps() * 
                         xAxis.getPositionSteps() / xAxis.getPositionSteps() * 
                         zAxis.getPositionSteps() / zAxis.getPositionSteps() * 
                         (auxDirectionForward ? 1 : -1);
        
        if (zToXRatio == 0) {
            return;
        }
        
        // Снятие ограничений скорости для синхронизации
        xAxis.setMaxSpeed(LONG_MAX);
        zAxis.setMaxSpeed(LONG_MAX);
        
        // TODO: Реализация алгоритма конического точения
        // с учетом ограничений обеих осей
        
        LOG_DEBUG("Контроллер", "Режим конического точения активен. Коэффициент: " + String(coneRatio));
    }
    
    /**
     * @brief Режим продольного точения
     * 
     * Автоматические проходы с возвратом в начало после каждого прохода.
     * Используется для черновой и чистовой обработки валов.
     */
    void updateTurnMode() {
        // Проверка готовности к работе
        if (zAxis.isMoving() || xAxis.isMoving() || turnPasses <= 0 ||
            zAxis.getLeftStop() == LONG_MAX || zAxis.getRightStop() == LONG_MIN ||
            xAxis.getLeftStop() == LONG_MAX || xAxis.getRightStop() == LONG_MIN ||
            currentPitch == 0 || (currentPitch * operationPitchSign < 0) || currentStarts < 1) {
            
            // Условия не выполнены - выключаем систему
            setIsOnFromLoop(false);
            return;
        }
        
        // TODO: Реализация алгоритма продольного точения
        // с автоматическими проходами и возвратом
        
        LOG_DEBUG("Контроллер", "Режим продольного точения активен. Проход " + 
                 String(operationIndex) + " из " + String(turnPasses));
    }
    
    // Остальные режимы будут реализованы аналогично
    void updateFaceMode() {
        LOG_DEBUG("Контроллер", "Режим подрезки торца активен");
        // Реализация подрезки торца
    }
    
    void updateCutMode() {
        LOG_DEBUG("Контроллер", "Режим прорезки активен");
        // Реализация прорезки канавок
    }
    
    void updateThreadMode() {
        LOG_DEBUG("Контроллер", "Режим нарезания резьбы активен");
        // Реализация нарезания резьбы
    }
    
    void updateEllipseMode() {
        LOG_DEBUG("Контроллер", "Режим эллиптического точения активен");
        // Реализация эллиптического точения
    }
    
    void updateGCodeMode() {
        LOG_DEBUG("Контроллер", "Режим G-кода активен");
        // Реализация управления по G-коду
    }
    
    void updateA1Mode() {
        LOG_DEBUG("Контроллер", "Режим оси A1 активен");
        // Реализация управления осью A1
    }
    
    /**
     * @brief Установка новой точки отсчета (синхронизация)
     * 
     * Устанавливает текущие позиции шпинделя и осей как новую нулевую точку.
     * Используется при изменении шага или включении системы для избежания
     * резкого движения осей к новой позиции.
     */
    void setNewOrigin() {
        zAxis.setOrigin();
        xAxis.setOrigin();
        if (a1Axis.isActive()) {
            a1Axis.setOrigin();
        }
        spindle.resetPosition();
        
        LOG_DEBUG("Контроллер", "Установлена новая точка отсчета для синхронизации");
    }
    
    /**
     * @brief Расчет позиции оси на основе позиции шпинделя
     * @param axis Контроллер оси для которой рассчитывается позиция
     * @param spindlePos Позиция шпинделя в счетных импульсах
     * @param respectStops Учитывать ли ограничения перемещения
     * @return Целевая позиция оси в шагах
     */
    long calculateAxisPosition(AxisController& axis, long spindlePos, bool respectStops = true) {
        // Расчет новой позиции оси на основе позиции шпинделя и шага резьбы
        long newPos = spindlePos * axis.getPositionSteps() / axis.getPositionSteps() / 
                     ENCODER_STEPS_INT * currentPitch * currentStarts;
        
        // Учет ограничений перемещения если требуется
        if (respectStops) {
            if (newPos < axis.getRightStop()) {
                newPos = axis.getRightStop();
            } else if (newPos > axis.getLeftStop()) {
                newPos = axis.getLeftStop();
            }
        }
        
        return newPos;
    }
    
    /**
     * @brief Расчет позиции шпинделя на основе позиции оси
     * @param axis Контроллер оси на основе которой рассчитывается позиция шпинделя
     * @param axisPos Позиция оси в шагах
     * @return Позиция шпинделя в счетных импульсах
     */
    long calculateSpindlePosition(AxisController& axis, long axisPos) {
        return axisPos * axis.getPositionSteps() / axis.getPositionSteps() * 
               ENCODER_STEPS_INT / (currentPitch * currentStarts);
    }
    
    // Ссылка на оригинальный метод для совместимости
    void setIsOnFromLoop(bool on) {
        setEnabled(on);
    }
};

#endif // MOTION_CONTROLLER_H