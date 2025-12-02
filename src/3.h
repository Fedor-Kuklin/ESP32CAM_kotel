#include <Arduino.h>

// Функция для проверки PSRAM
void checkPSRAM() {
  Serial.println("\n========== PSRAM TEST ==========");
  
  // Проверка наличия PSRAM
  if (psramFound()) {
    Serial.println("✓ PSRAM обнаружена!");
    
    // Получение размера PSRAM
    size_t psramSize = ESP.getPsramSize();
    Serial.printf("  Размер PSRAM: %d байт (%.2f MB)\n", 
                  psramSize, psramSize / (1024.0 * 1024.0));
    
    // Получение свободной PSRAM
    size_t freePsram = ESP.getFreePsram();
    Serial.printf("  Свободно PSRAM: %d байт (%.2f MB)\n", 
                  freePsram, freePsram / (1024.0 * 1024.0));
    
    // Получение минимальной свободной PSRAM
    size_t minFreePsram = ESP.getMinFreePsram();
    Serial.printf("  Минимальная свободная PSRAM: %d байт (%.2f MB)\n", 
                  minFreePsram, minFreePsram / (1024.0 * 1024.0));
    
  } else {
    Serial.println("✗ PSRAM НЕ обнаружена!");
    Serial.println("  Возможно, у вас ESP32 без PSRAM или PSRAM отключена");
  }
}

// Функция для проверки общей памяти
void checkMemory() {
  Serial.println("\n========== MEMORY INFO ==========");
  
  // Полная информация о памяти
  size_t heapSize = ESP.getHeapSize();
  size_t freeHeap = ESP.getFreeHeap();
  size_t minFreeHeap = ESP.getMinFreeHeap();
  size_t maxAllocHeap = ESP.getMaxAllocHeap();
  
  Serial.printf("Размер Heap: %d байт (%.2f KB)\n", 
                heapSize, heapSize / 1024.0);
  Serial.printf("Свободно Heap: %d байт (%.2f KB)\n", 
                freeHeap, freeHeap / 1024.0);
  Serial.printf("Минимальная свободная Heap: %d байт (%.2f KB)\n", 
                minFreeHeap, minFreeHeap / 1024.0);
  Serial.printf("Максимальный блок Heap: %d байт (%.2f KB)\n", 
                maxAllocHeap, maxAllocHeap / 1024.0);
}

// Функция для тестирования выделения памяти в PSRAM
void testPsramAllocation() {
  Serial.println("\n========== PSRAM ALLOCATION TEST ==========");
  
  if (!psramFound()) {
    Serial.println("PSRAM не найдена, тест пропущен");
    return;
  }
  
  // Тест 1: Выделение небольшого блока
  Serial.println("Тест 1: Выделение 1 KB в PSRAM");
  size_t beforeFree = ESP.getFreePsram();
  uint8_t* smallBuffer = (uint8_t*)ps_malloc(1024);
  
  if (smallBuffer) {
    size_t afterFree = ESP.getFreePsram();
    Serial.printf("  Успешно! Выделено: %d байт\n", beforeFree - afterFree);
    Serial.printf("  Адрес: 0x%p\n", smallBuffer);
    
    // Заполняем и проверяем
    for (int i = 0; i < 1024; i++) {
      smallBuffer[i] = i % 256;
    }
    
    // Проверяем запись
    bool ok = true;
    for (int i = 0; i < 1024; i++) {
      if (smallBuffer[i] != (i % 256)) {
        ok = false;
        break;
      }
    }
    Serial.printf("  Проверка записи: %s\n", ok ? "ПРОЙДЕНА" : "ОШИБКА");
    
    free(smallBuffer);
    Serial.println("  Память освобождена");
  } else {
    Serial.println("  ОШИБКА выделения памяти!");
  }
  
  // Тест 2: Выделение большого блока (1 MB)
  Serial.println("\nТест 2: Выделение 1 MB в PSRAM");
  beforeFree = ESP.getFreePsram();
  uint8_t* largeBuffer = (uint8_t*)ps_malloc(1024 * 1024);
  
  if (largeBuffer) {
    size_t afterFree = ESP.getFreePsram();
    Serial.printf("  Успешно! Выделено: %d байт (%.2f MB)\n", 
                  beforeFree - afterFree, (beforeFree - afterFree) / (1024.0 * 1024.0));
    Serial.printf("  Адрес: 0x%p\n", largeBuffer);
    
    // Заполняем паттерном
    Serial.println("  Заполнение памяти паттерном...");
    for (int i = 0; i < 1024 * 1024; i += 1024) {
      largeBuffer[i] = (i / 1024) % 256;
    }
    
    // Проверяем паттерн
    bool ok = true;
    for (int i = 0; i < 1024 * 1024; i += 1024) {
      if (largeBuffer[i] != ((i / 1024) % 256)) {
        ok = false;
        Serial.printf("  Ошибка по адресу %d\n", i);
        break;
      }
    }
    Serial.printf("  Проверка паттерна: %s\n", ok ? "ПРОЙДЕНА" : "ОШИБКА");
    
    free(largeBuffer);
    Serial.println("  Память освобождена");
  } else {
    Serial.println("  ОШИБКА выделения памяти!");
  }
  
  // Тест 3: calloc в PSRAM
  Serial.println("\nТест 3: Выделение и обнуление 64 KB в PSRAM (ps_calloc)");
  uint8_t* zeroBuffer = (uint8_t*)ps_calloc(64 * 1024, 1);
  
  if (zeroBuffer) {
    Serial.println("  Успешно! Буфер выделен и обнулен");
    
    // Проверяем, что память обнулена
    bool allZero = true;
    for (int i = 0; i < 100; i++) { // Проверяем только первые 100 байт
      if (zeroBuffer[i] != 0) {
        allZero = false;
        break;
      }
    }
    Serial.printf("  Проверка обнуления: %s\n", allZero ? "ПРОЙДЕНА" : "ОШИБКА");
    
    free(zeroBuffer);
    Serial.println("  Память освобождена");
  } else {
    Serial.println("  ОШИБКА выделения памяти!");
  }
}

// Функция для тестирования Heap vs PSRAM
void testHeapVsPsram() {
  Serial.println("\n========== HEAP vs PSRAM COMPARISON ==========");
  
  size_t heapBefore = ESP.getFreeHeap();
  size_t psramBefore = psramFound() ? ESP.getFreePsram() : 0;
  
  // Выделяем 256KB в Heap
  Serial.println("Выделение 256 KB в Heap:");
  uint8_t* heapBuffer = (uint8_t*)malloc(256 * 1024);
  
  if (heapBuffer) {
    size_t heapAfter = ESP.getFreeHeap();
    Serial.printf("  Успешно! Использовано Heap: %d байт\n", heapBefore - heapAfter);
    free(heapBuffer);
  } else {
    Serial.println("  ОШИБКА: Не удалось выделить в Heap!");
  }
  
  if (psramFound()) {
    // Выделяем 256KB в PSRAM
    Serial.println("Выделение 256 KB в PSRAM:");
    uint8_t* psramBuffer = (uint8_t*)ps_malloc(256 * 1024);
    
    if (psramBuffer) {
      size_t psramAfter = ESP.getFreePsram();
      Serial.printf("  Успешно! Использовано PSRAM: %d байт\n", psramBefore - psramAfter);
      free(psramBuffer);
    } else {
      Serial.println("  ОШИБКА: Не удалось выделить в PSRAM!");
    }
  }
}

// Функция для проверки возможности включения PSRAM в коде
void checkPsramConfiguration() {
  Serial.println("\n========== PSRAM CONFIGURATION ==========");
  
  // Проверка флагов компиляции
  Serial.println("Флаги компиляции (если доступны):");
  
  #ifdef BOARD_HAS_PSRAM
    Serial.println("  BOARD_HAS_PSRAM: определен");
  #else
    Serial.println("  BOARD_HAS_PSRAM: НЕ определен");
  #endif
  
  #ifdef CONFIG_SPIRAM_SUPPORT
    Serial.println("  CONFIG_SPIRAM_SUPPORT: определен");
  #else
    Serial.println("  CONFIG_SPIRAM_SUPPORT: НЕ определен");
  #endif
  
  #ifdef CONFIG_SPIRAM
    Serial.println("  CONFIG_SPIRAM: определен");
  #else
    Serial.println("  CONFIG_SPIRAM: НЕ определен");
  #endif
  
  // Проверка режима PSRAM
  #ifdef CONFIG_SPIRAM_MODE_QUAD
    Serial.println("  Режим PSRAM: QUAD");
  #elif defined(CONFIG_SPIRAM_MODE_OCT)
    Serial.println("  Режим PSRAM: OCT");
  #else
    Serial.println("  Режим PSRAM: неизвестен или не используется");
  #endif
}

// Функция для проверки камеры с PSRAM
void testCameraWithPsram() {
  Serial.println("\n========== CAMERA PSRAM TEST ==========");
  
  if (!psramFound()) {
    Serial.println("PSRAM не найдена. Камера может не работать правильно!");
    Serial.println("Рекомендуется использовать ESP32-CAM с PSRAM.");
    return;
  }
  
  Serial.println("Для теста камеры требуется:");
  Serial.println("1. ESP32-CAM с PSRAM (4MB обычно)");
  Serial.println("2. Правильно подключенный модуль камеры");
  Serial.println("3. Достаточное питание (не от USB компьютера)");
  Serial.println("4. Включенная PSRAM в настройках проекта");
  
  Serial.println("\nПроверка наличия PSRAM для камеры:");
  
  // Минимальные требования для камеры
  size_t minPsramForCamera = 1024 * 1024; // 1MB минимум
  size_t freePsram = ESP.getFreePsram();
  
  if (freePsram > minPsramForCamera) {
    Serial.printf("✓ Достаточно PSRAM для камеры: %.2f MB свободно\n", 
                  freePsram / (1024.0 * 1024.0));
  } else {
    Serial.printf("✗ МАЛО PSRAM для камеры: %.2f MB свободно (нужно минимум 1 MB)\n", 
                  freePsram / (1024.0 * 1024.0));
  }
}

void setup() {
  Serial.begin(115200);
  delay(2000); // Даем время для подключения Serial
  
  Serial.println("\n========================================");
  Serial.println("       ESP32 PSRAM TEST PROGRAM");
  Serial.println("========================================");
  
  // Информация о чипе
  Serial.printf("Chip: %s\n", ESP.getChipModel());
  Serial.printf("Chip Revision: %d\n", ESP.getChipRevision());
  Serial.printf("CPU Frequency: %d MHz\n", ESP.getCpuFreqMHz());
  Serial.printf("SDK Version: %s\n", ESP.getSdkVersion());
  Serial.printf("Flash Size: %d MB\n", ESP.getFlashChipSize() / (1024 * 1024));
  Serial.printf("Flash Speed: %d MHz\n", ESP.getFlashChipSpeed() / 1000000);
  
  // Запуск тестов
  checkPSRAM();
  checkMemory();
  checkPsramConfiguration();
  testPsramAllocation();
  testHeapVsPsram();
  testCameraWithPsram();
  
  Serial.println("\n========================================");
  Serial.println("          TEST COMPLETE");
  Serial.println("========================================");
}

void loop() {
  // Периодически показываем свободную память
  static unsigned long lastUpdate = 0;
  if (millis() - lastUpdate > 10000) { // Каждые 10 секунд
    Serial.printf("\n[%lu] Heap free: %.2f KB", 
                  millis() / 1000, ESP.getFreeHeap() / 1024.0);
    if (psramFound()) {
      Serial.printf(", PSRAM free: %.2f MB", 
                    ESP.getFreePsram() / (1024.0 * 1024.0));
    }
    lastUpdate = millis();
  }
  
  delay(1000);
}