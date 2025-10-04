#ifndef DISPLAY_MANAGER_H
#define DISPLAY_MANAGER_H

#include <Arduino.h>
#include <LiquidCrystal.h>
#include "Config.h"
#include "RussianLogger.h"
#include "MotionController.h"
#include "AxisController.h"

/**
 * @class DisplayManager
 * @brief Управление ЖК-дисплеем и отображение информации о состоянии системы
 * 
 * Класс оптимизирует обновление дисплея, перерисовывая только измененные области
 * и реализует всю логику форматирования и отображения данных на русском языке.
 * Использует хэширование для определения необходимости обновления каждой строки.
 */
class DisplayManager {
private:
    LiquidCrystal& lcd;                 // Ссылка на объект дисплея
    MotionController& motionController; // Ссылка на контроллер движения
    
    // Хэши для отслеживания изменений (оптимизация перерисовки)
    long lineHashes[4];                 // Хэши для каждой из 4 строк дисплея
    static constexpr long INITIAL_HASH = -3845709; // Начальное значение хэша
    
    // Пользовательские символы для дисплея
    uint8_t customChars[7][8];          // Массив для хранения пользовательских символов
    
    // Состояние отображения
    bool showAngle;                     // Показывать угол шпинделя
    bool showTacho;                     // Показывать обороты шпинделя
    bool splashScreen;                  // Показывать заставку
    unsigned long splashStartTime;      // Время начала показа заставки
    
    // Кэшированные значения для редких обновлений
    int cachedRpm;                      // Кэшированные обороты шпинделя
    unsigned long lastRpmUpdate;        // Время последнего обновления RPM

public:
    /**
     * @brief Конструктор менеджера дисплея
     * @param lcdRef Ссылка на объект дисплея LiquidCrystal
     * @param motionCtrlRef Ссылка на контроллер движения
     */
    DisplayManager(LiquidCrystal& lcdRef, MotionController& motionCtrlRef)
        : lcd(lcdRef), motionController(motionCtrlRef), showAngle(false), 
          showTacho(false), splashScreen(true), splashStartTime(millis()),
          cachedRpm(0), lastRpmUpdate(0) {
        
        // Инициализация хэшей
        for (int i = 0; i < 4; i++) {
            lineHashes[i] = INITIAL_HASH;
        }
    }
    
    /**
     * @brief Инициализация дисплея и создание пользовательских символов
     * 
     * Настраивает дисплей, создает пользовательские символы (значки ограничений, мм и т.д.)
     * и выводит начальную заставку.
     */
    void begin() {
        // Инициализация дисплея 20x4
        lcd.begin(20, 4);
        
        // Создание пользовательских символов
        createCustomCharacters();
        
        // Показ заставки
        showSplashScreen();
        
        LOG_INFO("Дисплей", "Инициализирован дисплей 20x4");
    }
    
    /**
     * @brief Обновление отображения (должен вызываться периодически)
     * 
     * Проверяет изменения в состоянии системы и обновляет соответствующие
     * строки дисплея. Использует хэширование для оптимизации.
     */
    void update() {
        // Показ заставки если активна
        if (splashScreen) {
            if (millis() - splashStartTime > 2000) { // Показывать 2 секунды
                splashScreen = false;
                lcd.clear();
                // Сброс хэшей для принудительного обновления
                for (int i = 0; i < 4; i++) {
                    lineHashes[i] = INITIAL_HASH;
                }
            }
            return;
        }
        
        // Обновление строк если данные изменились
        updateStatusLine();     // Строка 0: Режим и состояние
        updatePitchLine();      // Строка 1: Шаг и заходы
        updatePositionLine();   // Строка 2: Позиции осей
        updateInfoLine();       // Строка 3: Информация и подсказки
    }
    
    /**
     * @brief Отображение экрана заставки
     */
    void showSplashScreen() {
        lcd.clear();
        lcd.setCursor(6, 1);
        lcd.print("NanoELS");
        lcd.setCursor(6, 2);
        lcd.print("H" + String(HARDWARE_VERSION) + " V" + String(SOFTWARE_VERSION));
        
        LOG_INFO("Дисплей", "Показана заставка");
    }
    
    /**
     * @brief Переключение отображаемой информации на нижней строке
     * 
     * Циклически переключает между показом угла шпинделя, оборотов и другой информации.
     */
    void toggleDisplayMode() {
        if (!showAngle && !showTacho) {
            showAngle = true;
        } else if (showAngle) {
            showAngle = false;
            showTacho = true;
        } else {
            showTacho = false;
        }
        
        // Сброс хэша информационной строки для принудительного обновления
        lineHashes[3] = INITIAL_HASH;
        
        LOG_DEBUG("Дисплей", "Режим отображения: " + 
                 String(showAngle ? "Угол" : showTacho ? "Обороты" : "Информация"));
    }
    
    /**
     * @brief Установка режима отображения угла/тахометра
     * @param showAng Показывать угол шпинделя
     * @param showTach Показывать обороты шпинделя
     */
    void setDisplayMode(bool showAng, bool showTach) {
        showAngle = showAng;
        showTacho = showTach;
        lineHashes[3] = INITIAL_HASH; // Принудительное обновление
    }

private:
    /**
     * @brief Обновление строки дисплея если данные изменились
     * @param lineIndex Индекс строки (0-3)
     * @param newHash Хэш новых данных для строки
     */
    void updateLineIfChanged(int lineIndex, long newHash) {
        if (lineHashes[lineIndex] != newHash) {
            lineHashes[lineIndex] = newHash;
            // Строка будет обновлена в специализированном методе
        }
    }
    
    /**
     * @brief Обновление верхней строки (режим и состояние)
     * 
     * Отображает текущий режим работы, состояние системы (ВКЛ/ВЫКЛ),
     * установленные ограничения и другую служебную информацию.
     */
    void updateStatusLine() {
        long newHash = motionController.getOperationMode() + 
                      (motionController.isEnabled() ? 1 : 0) +
                      turnPasses * 10;
        
        if (lineHashes[0] != newHash) {
            lineHashes[0] = newHash;
            lcd.setCursor(0, 0);
            
            // Отображение режима работы
            printMode();
            
            // Отображение состояния системы
            lcd.print(motionController.isEnabled() ? "ВКЛ " : "выкл ");
            
            // TODO: Отображение ограничений и другой информации
            // в соответствии с оригинальной логикой
            
            fillRemainingSpaces(8); // Заполнение оставшегося места
        }
    }
    
    /**
     * @brief Обновление строки с шагом резьбы
     * 
     * Отображает текущий шаг резьбы в выбранной системе измерений
     * и число заходов для многозаходной резьбы.
     */
    void updatePitchLine() {
        long newHash = motionController.getPitch() + 
                      motionController.getStarts() * 1000000L;
        
        if (lineHashes[1] != newHash) {
            lineHashes[1] = newHash;
            lcd.setCursor(0, 1);
            lcd.print("Шаг ");
            
            // TODO: Реализация форматированного вывода шага
            // в метрической, дюймовой или TPI системе
            
            // Отображение числа заходов если больше 1
            if (motionController.getStarts() != 1) {
                lcd.print(" x");
                lcd.print(motionController.getStarts());
            }
            
            fillRemainingSpaces(10);
        }
    }
    
    /**
     * @brief Обновление строки с позициями осей
     * 
     * Отображает текущие позиции осей Z и X в выбранной системе измерений.
     * Для вращательной оси A1 отображает угол в градусах.
     */
    void updatePositionLine() {
        long newHash = 0; // TODO: Расчет хэша на основе позиций осей
        
        if (lineHashes[2] != newHash) {
            lineHashes[2] = newHash;
            lcd.setCursor(0, 2);
            
            // TODO: Отображение позиций осей Z и X
            // в соответствии с оригинальной логикой
            
            fillRemainingSpaces(10);
        }
    }
    
    /**
     * @brief Обновление информационной строки
     * 
     * Отображает различную информацию в зависимости от режима работы:
     * - Угол шпинделя или обороты
     * - Подсказки в мастере настройки
     * - Текущий проход в автоматических режимах
     * - Сообщения G-кода
     */
    void updateInfoLine() {
        long newHash = 0; // TODO: Расчет хэша на основе отображаемой информации
        
        if (lineHashes[3] != newHash) {
            lineHashes[3] = newHash;
            lcd.setCursor(0, 3);
            
            // TODO: Реализация логики отображения информации
            // в соответствии с оригинальным кодом
            
            fillRemainingSpaces(0);
        }
    }
    
    /**
     * @brief Форматирование и вывод значения в деци-микронах
     * @param deciMicrons Значение в деци-микронах (0.0001 мм)
     * @param maxPrecision Максимальное число знаков после запятой
     * @return Число выведенных символов
     */
    int printDeciMicrons(long deciMicrons, int maxPrecision) {
        // TODO: Реализация форматированного вывода
        // в соответствии с оригинальной логикой
        return lcd.print(deciMicrons / 10000.0, maxPrecision);
    }
    
    /**
     * @brief Форматирование и вывод угла в градусах
     * @param degrees10000 Угол в градусах * 10000
     * @return Число выведенных символов
     */
    int printDegrees(long degrees10000) {
        // TODO: Реализация форматированного вывода угла
        return lcd.print(degrees10000 / 10000.0, 2);
    }
    
    /**
     * @brief Форматирование и вывод шага резьбы
     * @param pitch Шаг в деци-микронах
     * @return Число выведенных символов
     */
    int printPitch(long pitch) {
        // TODO: Реализация в зависимости от системы измерений
        return lcd.print(pitch);
    }
    
    /**
     * @brief Вывод названия режима работы
     * @return Число выведенных символов
     */
    int printMode() {
        switch(motionController.getOperationMode()) {
            case MODE_NORMAL: return lcd.print("РЕЗЬБА ");
            case MODE_ASYNC: return lcd.print("АСИНХР ");
            case MODE_CONE: return lcd.print("КОНУС ");
            case MODE_TURN: return lcd.print("ПРОДОЛ ");
            case MODE_FACE: return lcd.print("ТОРЕЦ ");
            case MODE_CUT: return lcd.print("ПРОРЕЗ ");
            case MODE_THREAD: return lcd.print("РЕЗЬБА ");
            case MODE_ELLIPSE: return lcd.print("ЭЛЛИПС ");
            case MODE_GCODE: return lcd.print("G-КОД ");
            case MODE_A1: return lcd.print("ОСЬ A1 ");
            default: return lcd.print("НЕИЗВ ");
        }
    }
    
    /**
     * @brief Дозаполнение строки пробелами
     * @param charsPrinted Число уже выведенных символов
     */
    void fillRemainingSpaces(int charsPrinted) {
        for (int i = charsPrinted; i < 20; i++) {
            lcd.print(" ");
        }
    }
    
    /**
     * @brief Создание пользовательских символов для дисплея
     * 
     * Создает значки для отображения ограничений, единиц измерения и другой
     * служебной информации в соответствии с оригинальным кодом.
     */
    void createCustomCharacters() {
        // Символ мм (две буквы m со смещением)
        byte customCharMm[] = {
            B11010,
            B10101,
            B10101,
            B00000,
            B11010,
            B10101,
            B10101,
            B00000
        };
        
        // Символ верхнего ограничения
        byte customCharLimUp[] = {
            B11111,
            B00100,
            B01110,
            B10101,
            B00100,
            B00100,
            B00000,
            B00000
        };
        
        // TODO: Создание остальных пользовательских символов
        // в соответствии с оригинальным кодом
        
        lcd.createChar(0, customCharMm);
        lcd.createChar(1, customCharLimUp);
        
        LOG_DEBUG("Дисплей", "Созданы пользовательские символы");
    }
};

#endif // DISPLAY_MANAGER_H