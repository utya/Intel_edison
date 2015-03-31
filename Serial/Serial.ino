

#include <Wire.h>
#include <LSM303.h>
#include <L3G4200D.h>

L3G4200D gyro;
LSM303 compass;

void setup()
{
  Serial.begin(115200);
  Wire.begin();
  compass.init();
  compass.enableDefault();
  gyro.enableDefault();
}

void loop()
{
  compass.read();

  Serial.print(compass.a.x);
  Serial.print(",");
  Serial.print(compass.a.y);
  Serial.print(",");
  Serial.print(compass.a.z);
  Serial.print(",");
  Serial.print(compass.m.x);
  Serial.print(",");
  Serial.print(compass.m.y);
  Serial.print(",");
  Serial.print(compass.m.z);
  Serial.print(",");
  Serial.print(gyro.g.x);
  Serial.print(",");
  Serial.print(gyro.g.y);
  Serial.print(",");
  Serial.println(gyro.g.z);
  
    delay(1);
}
