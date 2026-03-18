#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <QMC5883LCompass.h>
float AxRaw,AyRaw,AzRaw,GxRaw,GyRaw,GzRaw,MxRaw,MyRaw,MzRaw;
Adafruit_MPU6050 mpu;
QMC5883LCompass compass;

void setup() {
  // put your setup code here, to run once:
Serial.begin(115200);
mpu.begin();
mpu.setI2CBypass(true);
compass.init();
mpu.setGyroRange(MPU6050_RANGE_1000_DEG);
delay(100);
}

void loop() {
  // put your main code here, to run repeatedly:
compass.read();
sensors_event_t a, g, temp;
mpu.getEvent(&a, &g, &temp);

AxRaw = a.acceleration.x;
AyRaw = a.acceleration.y;
AzRaw = a.acceleration.z;

GxRaw = g.gyro.x;
GyRaw = g.gyro.y;
GzRaw = g.gyro.z;

MxRaw= compass.getX();
MyRaw= compass.getY();
MzRaw= compass.getZ();

Serial.print(AxRaw);
Serial.print(',');
Serial.print(AyRaw);
Serial.print(',');
Serial.print(AzRaw);
Serial.print(',');
Serial.print(GxRaw);
Serial.print(',');
Serial.print(GyRaw);
Serial.print(',');
Serial.print(GzRaw);
Serial.print(',');
Serial.print(MxRaw);
Serial.print(',');
Serial.print(MyRaw);
Serial.print(',');
Serial.println(MzRaw);
delay(100);
}
