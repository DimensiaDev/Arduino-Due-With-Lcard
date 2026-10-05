/*
  Тест датчика TCRT5000 на Arduino Due (аналоговый вход).

  Подключение (питание ТОЛЬКО от 3.3 В — аналоговые входы Due
  не толерантны к 5 В!):
    ИК-светодиод:    A (вывод 4) -> резистор 100–150 Ом -> 3V3
                     K (вывод 3) -> GND
    Фототранзистор:  C (вывод 1) -> резистор 10 кОм -> 3V3
                     E (вывод 2) -> GND
    Выход сигнала: точка соединения C и резистора 10 кОм -> A0
    GND датчика -> GND Due.

  Логика: чем ближе и светлее поверхность, тем сильнее отражённый
  ИК-поток, тем сильнее открыт фототранзистор и тем НИЖЕ напряжение
  на аналоговом входе.

  Монитор порта: 115200 бод.
*/

const uint8_t SENSOR_PIN = A0;   // любой аналоговый вход Due: A0..A11
const float   VREF       = 3.3;  // опорное напряжение АЦП Due
const int     ADC_MAX    = 4095; // разрешение АЦП Due — 12 бит

void setup() {
  Serial.begin(115200);
  analogReadResolution(12);      // 0..4095
  Serial.println("TCRT5000 test");
}

// Усреднение нескольких отсчётов — АЦП Due шумноват
static uint16_t readAveraged(uint8_t pin, uint8_t samples) {
  uint32_t sum = 0;
  for (uint8_t i = 0; i < samples; ++i) {
    sum += analogRead(pin);
    delay(2);
  }
  return (uint16_t)(sum / samples);
}

void loop() {
  uint16_t raw   = readAveraged(SENSOR_PIN, 8);
  float    volts = raw * VREF / ADC_MAX;

  Serial.print("raw = ");
  Serial.print(raw);
  Serial.print("   U = ");
  Serial.print(volts, 2);
  Serial.print(" V   ");

  // текстовая шкала для наглядности
  uint8_t bars = map(raw, 0, ADC_MAX, 0, 20);
  for (uint8_t i = 0; i < bars; ++i) Serial.print('#');

  if (raw < 1500)      Serial.print("   <- есть отражение (близко/светлое)");
  else if (raw > 3500) Serial.print("   <- нет отражения (далеко/тёмное)");

  Serial.println();
  delay(200);
}
