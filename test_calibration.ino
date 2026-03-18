#include "ICM_20948.h"
#define SERIAL_PORT Serial
#define WIRE_PORT Wire
#define AD0_VAL 0

ICM_20948_I2C myICM;
float AxCal, AyCal, AzCal, GxCal, GyCal, GzCal, MxCal, MyCal, MzCal;
float Ax, Ay, Az, Gx, Gy, Gz, Mx, My, Mz;

void setup() {
  SERIAL_PORT.begin(115200);
  while (!SERIAL_PORT) {};
  WIRE_PORT.begin();
  WIRE_PORT.setClock(400000);
  
  while (myICM.begin(WIRE_PORT, AD0_VAL) != ICM_20948_Stat_Ok) {
    SERIAL_PORT.println("ICM init failed, retrying...");
    delay(500);
  }
  SERIAL_PORT.println("ICM-20948 initialized with calibration ✓");
}

void loop() {
  if (myICM.dataReady()) {
    myICM.getAGMT();
    Ax = myICM.accX() / 1000.0 * 9.80665;
    Ay = myICM.accY() / 1000.0 * 9.80665;
    Az = myICM.accZ() / 1000.0 * 9.80665;

    Gx = myICM.gyrX();
    Gy = myICM.gyrY();
    Gz = myICM.gyrZ();

    Mx = myICM.magX();
    My = myICM.magY();
    Mz = myICM.magZ();

    calibrateSensors();

    SERIAL_PORT.print(AxCal);
    SERIAL_PORT.print(",");
    SERIAL_PORT.print(AyCal);
    SERIAL_PORT.print(",");
    SERIAL_PORT.print(AzCal);
    SERIAL_PORT.print(",");

    SERIAL_PORT.print(GxCal);
    SERIAL_PORT.print(",");
    SERIAL_PORT.print(GyCal);
    SERIAL_PORT.print(",");
    SERIAL_PORT.print(GzCal);
    SERIAL_PORT.print(",");

    SERIAL_PORT.print(MxCal);
    SERIAL_PORT.print(",");
    SERIAL_PORT.print(MyCal);
    SERIAL_PORT.print(",");
    SERIAL_PORT.print(MzCal);
    SERIAL_PORT.println();
    
    delay(100);
  }
}
void calibrateSensors() {
    const float axOffset =  -0.10971184577473814;
    const float ayOffset =  -0.14777395410727312;
    const float azOffset =  0.11404282076598804;
    const float axScale =  0.0998897165559659;
    const float ayScale =  0.0998897165559659;
    const float azScale =  0.0998897165559659;
    const float gxOffset =  19.775638298940365;
    const float gyOffset =  17.417916971977025;
    const float gzOffset =  2.139997364813627;
    const float mxOffset =  4.855801616866724;
    const float myOffset =  11.253693174681061;
    const float mzOffset =  0.44040179139344815;
    const float mxScale =  0.018953630933590166;
    const float myScale =  0.01827127352285214;
    const float mzScale =  0.02034846742648391;

    AxCal = (Ax - axOffset) * axScale;
    AyCal = (Ay - ayOffset) * ayScale;
    AzCal = (Az - azOffset) * azScale;
    GxCal = Gx - gxOffset;
    GyCal = Gy - gyOffset;
    GzCal = Gz - gzOffset;
    MxCal = (Mx - mxOffset) * mxScale;
    MyCal = (My - myOffset) * myScale;
    MzCal = (Mz - mzOffset) * mzScale;
}
