#include "ICM_20948.h"
#define SERIAL_PORT Serial
#define WIRE_PORT Wire
#define AD0_VAL 0

ICM_20948_I2C myICM;

// Храним средние значения для 6 позиций (в м/с²)
float calib_values[6] = {0};  // 0:X+, 1:X-, 2:Y+, 3:Y-, 4:Z+, 5:Z-
uint8_t state = 0;
bool calibrating = false;

void setup() {
  SERIAL_PORT.begin(115200);
  while (!SERIAL_PORT) {};
  WIRE_PORT.begin();
  WIRE_PORT.setClock(400000);
  
  bool initialized = false;
  while (!initialized) {
    myICM.begin(WIRE_PORT, AD0_VAL);
    SERIAL_PORT.print(F("Initialization: "));
    SERIAL_PORT.println(myICM.statusString());
    if (myICM.status != ICM_20948_Stat_Ok) {
      SERIAL_PORT.println("Retrying...");
      delay(500);
    } else {
      initialized = true;
    }
  }
  printHelp();
}

void printHelp() {
  SERIAL_PORT.println(F("\n=== ICM-20948 Calibration ==="));
  SERIAL_PORT.println(F("Commands:"));
  SERIAL_PORT.println(F("  t - take measurement for current position"));
  SERIAL_PORT.println(F("  r - reset calibration"));
  SERIAL_PORT.println(F("  c - calculate and print bias/scale"));
  SERIAL_PORT.println(F("  s - stream live data"));
  SERIAL_PORT.println(F("\nPositions (state):"));
  SERIAL_PORT.println(F("  0:X+  1:X-  2:Y+  3:Y-  4:Z+  5:Z-"));
  SERIAL_PORT.println(F("  Change state manually in code or add serial command"));
}

void loop() {
  if (SERIAL_PORT.available() > 0) {
    char cmd = SERIAL_PORT.read();
    
    if (cmd == 't' && !calibrating) {
      calibratePosition(state);
    }
    else if (cmd == 'r') {
      resetCalibration();
    }
    else if (cmd == 'c') {
      calculateAndPrintCoefficients();
    }
    else if (cmd == 's') {
      streamData();
    }
    else if (cmd == 'n') {
      state = (state + 1) % 6;
      SERIAL_PORT.print(F("State changed to: "));
      SERIAL_PORT.println(state);
    }
  }
}

// Сбор 1000 замеров для текущей позиции
void calibratePosition(uint8_t pos) {
  SERIAL_PORT.print(F("Calibrating position "));
  SERIAL_PORT.print(pos);
  SERIAL_PORT.println(F("... hold still!"));
  
  float sum = 0;
  const uint16_t samples = 1000;
  
  for (uint16_t i = 0; i < samples; i++) {
    while (!myICM.dataReady()) {}
    myICM.getAGMT();
    
    // Выбираем нужную ось в зависимости от позиции
    switch (pos) {
      case 0: case 1: sum += myICM.accX() / 1000.0 * 9.80665; break;  // X
      case 2: case 3: sum += myICM.accY() / 1000.0 * 9.80665; break;  // Y
      case 4: case 5: sum += myICM.accZ() / 1000.0 * 9.80665; break;  // Z
    }
    delay(2);  // ~500 Гц
  }
  
  calib_values[pos] = sum / samples;
  
  SERIAL_PORT.print(F("Done! Avg for pos "));
  SERIAL_PORT.print(pos);
  SERIAL_PORT.print(F(": "));
  SERIAL_PORT.print(calib_values[pos], 4);
  SERIAL_PORT.println(F(" m/s²"));
  SERIAL_PORT.println(F("Send 'n' for next position, or 'c' to calculate coefficients"));
}

void resetCalibration() {
  for (uint8_t i = 0; i < 6; i++) calib_values[i] = 0;
  state = 0;
  SERIAL_PORT.println(F("Calibration reset"));
}

// Расчёт и вывод bias/scale
void calculateAndPrintCoefficients() {
  SERIAL_PORT.println(F("\n=== Calibration Results ==="));
  
  const float G = 9.80665;
  
  // Проверка: все 6 позиций заполнены?
  bool ready = true;
  for (uint8_t i = 0; i < 6; i++) {
    if (calib_values[i] == 0) {
      SERIAL_PORT.print(F("Position "));
      SERIAL_PORT.print(i);
      SERIAL_PORT.println(F(" not calibrated yet!"));
      ready = false;
    }
  }
  if (!ready) return;
  
  // Расчёт для каждой оси
  float bias_x = (calib_values[0] + calib_values[1]) / 2.0;
  float scale_x = (calib_values[1] - calib_values[0]) / (2.0 * G);
  
  float bias_y = (calib_values[2] + calib_values[3]) / 2.0;
  float scale_y = (calib_values[3] - calib_values[2]) / (2.0 * G);
  
  float bias_z = (calib_values[4] + calib_values[5]) / 2.0;
  float scale_z = (calib_values[5] - calib_values[4]) / (2.0 * G);
  
  // Вывод в удобном формате для копирования в основной код
  SERIAL_PORT.println(F("\n// Copy these into your main code:"));
  SERIAL_PORT.print(F("const float ACCEL_BIAS[] = {"));
  SERIAL_PORT.print(bias_x, 6); SERIAL_PORT.print(F(", "));
  SERIAL_PORT.print(bias_y, 6); SERIAL_PORT.print(F(", "));
  SERIAL_PORT.print(bias_z, 6); SERIAL_PORT.println(F("}; // m/s²"));
  
  SERIAL_PORT.print(F("const float ACCEL_SCALE[] = {"));
  SERIAL_PORT.print(scale_x, 6); SERIAL_PORT.print(F(", "));
  SERIAL_PORT.print(scale_y, 6); SERIAL_PORT.print(F(", "));
  SERIAL_PORT.print(scale_z, 6); SERIAL_PORT.println(F("}; // unitless (~1.0)"));
  
  // Проверка качества
  SERIAL_PORT.println(F("\n// Quality check:"));
  SERIAL_PORT.print(F("Scale X: ")); SERIAL_PORT.println(scale_x < 0.95 || scale_x > 1.05 ? "⚠️ CHECK" : "✓ OK");
  SERIAL_PORT.print(F("Scale Y: ")); SERIAL_PORT.println(scale_y < 0.95 || scale_y > 1.05 ? "⚠️ CHECK" : "✓ OK");
  SERIAL_PORT.print(F("Scale Z: ")); SERIAL_PORT.println(scale_z < 0.95 || scale_z > 1.05 ? "⚠️ CHECK" : "✓ OK");
}

// Поток данных для проверки
void streamData() {
  SERIAL_PORT.println(F("Streaming... send any key to stop"));
  while (!SERIAL_PORT.available()) {
    if (myICM.dataReady()) {
      myICM.getAGMT();
      
      // Чтение с применением калибровки (если коэффициенты заданы)
      float ax = myICM.accX() / 1000.0 * 9.80665;
      float ay = myICM.accY() / 1000.0 * 9.80665;
      float az = myICM.accZ() / 1000.0 * 9.80665;
      
      // Пример применения (раскомментируйте и вставьте свои коэффициенты):
      // ax = (ax - ACCEL_BIAS[0]) / ACCEL_SCALE[0];
      // ay = (ay - ACCEL_BIAS[1]) / ACCEL_SCALE[1];
      // az = (az - ACCEL_BIAS[2]) / ACCEL_SCALE[2];
      
      float norm = sqrt(ax*ax + ay*ay + az*az);
      
      SERIAL_PORT.print(F("ACC: ["));
      SERIAL_PORT.print(ax, 3); SERIAL_PORT.print(F(", "));
      SERIAL_PORT.print(ay, 3); SERIAL_PORT.print(F(", "));
      SERIAL_PORT.print(az, 3); SERIAL_PORT.print(F("] ||a||="));
      SERIAL_PORT.print(norm, 3);
      SERIAL_PORT.print(F(" m/s² | GYR: ["));
      SERIAL_PORT.print(myICM.gyrX() * PI / 180.0, 4); SERIAL_PORT.print(F(", "));
      SERIAL_PORT.print(myICM.gyrY() * PI / 180.0, 4); SERIAL_PORT.print(F(", "));
      SERIAL_PORT.print(myICM.gyrZ() * PI / 180.0, 4);
      SERIAL_PORT.println(F("] rad/s"));
    }
    delay(10);
  }
  SERIAL_PORT.read(); // очистить буфер
  SERIAL_PORT.println(F("Streaming stopped"));
}

// Исправленная функция вывода
void printScaledAGMT(ICM_20948_I2C *sensor) {
  SERIAL_PORT.print("ACC X=");
  SERIAL_PORT.print(sensor->accX() / 1000.0 * 9.80665, 3);
  SERIAL_PORT.print(" Y=");
  SERIAL_PORT.print(sensor->accY() / 1000.0 * 9.80665, 3);
  SERIAL_PORT.print(" Z=");
  SERIAL_PORT.print(sensor->accZ() / 1000.0 * 9.80665, 3);
  
  SERIAL_PORT.print(" GYR X=");
  SERIAL_PORT.print(sensor->gyrX() * PI / 180.0, 4);
  SERIAL_PORT.print(" Y=");
  SERIAL_PORT.print(sensor->gyrY() * PI / 180.0, 4);
  SERIAL_PORT.print(" Z=");
  SERIAL_PORT.print(sensor->gyrZ() * PI / 180.0, 4);
  
  SERIAL_PORT.print(" MAG X=");
  SERIAL_PORT.print(sensor->magX() * 1e-6, 6);  // ✅ Исправлено!
  SERIAL_PORT.print(" Y=");
  SERIAL_PORT.print(sensor->magY() * 1e-6, 6);
  SERIAL_PORT.print(" Z=");
  SERIAL_PORT.print(sensor->magZ() * 1e-6, 6);
  
  SERIAL_PORT.println();
}
