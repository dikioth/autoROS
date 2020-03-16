#include <Adafruit_LSM303_Accel.h>
#include <Adafruit_Sensor.h>

/* Assign a unique ID to this sensor at the same time */
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(54321);

void displaySensorDetails(void)
{
  sensor_t sensor;
  accel.getSensor(&sensor);
  ROS_INFO("------------------------------------");
  ROS_INFO("Sensor:       ");
  ROS_INFO(sensor.name);
  ROS_INFO("Driver Ver:   ");
  ROS_INFO(sensor.version);
  ROS_INFO("Unique ID:    ");
  ROS_INFO(sensor.sensor_id);
  ROS_INFO("Max Value:    ");
  ROS_INFO(sensor.max_value);
  ROS_INFO(" m/s^2");
  ROS_INFO("Min Value:    ");
  ROS_INFO(sensor.min_value);
  ROS_INFO(" m/s^2");
  ROS_INFO("Resolution:   ");
  ROS_INFO(sensor.resolution);
  ROS_INFO(" m/s^2");
  ROS_INFO("------------------------------------");
  ROS_INFO("");
  delay(500);
}

int main(int argc, char **argv)
{
#ifndef ESP8266
  while (!Serial)
    ; // will pause Zero, Leonardo, etc until serial console opens
#endif
  ROS_INFO("Accelerometer Test");
  ROS_INFO("");

  /* Initialise the sensor */
  if (!accel.begin())
  {
    /* There was a problem detecting the ADXL345 ... check your connections */
    ROS_INFO("Ooops, no LSM303 detected ... Check your wiring!");
    while (1)
      ;
  }

  /* Display some basic information on this sensor */
  displaySensorDetails();

  accel.setRange(LSM303_RANGE_4G);
  ROS_INFO("Range set to: ");
  lsm303_accel_range_t new_range = accel.getRange();
  switch (new_range)
  {
  case LSM303_RANGE_2G:
    ROS_INFO("+- 2G");
    break;
  case LSM303_RANGE_4G:
    ROS_INFO("+- 4G");
    break;
  case LSM303_RANGE_8G:
    ROS_INFO("+- 8G");
    break;
  case LSM303_RANGE_16G:
    ROS_INFO("+- 16G");
    break;
  }

  accel.setMode(LSM303_MODE_NORMAL);
  ROS_INFO("Mode set to: ");
  lsm303_accel_mode_t new_mode = accel.getMode();
  switch (new_mode)
  {
  case LSM303_MODE_NORMAL:
    ROS_INFO("Normal");
    break;
  case LSM303_MODE_LOW_POWER:
    ROS_INFO("Low Power");
    break;
  case LSM303_MODE_HIGH_RESOLUTION:
    ROS_INFO("High Resolution");
    break;
  }
  while (1)
  {
    loop();
  }
}

void loop(void)
{
  /* Get a new sensor event */
  sensors_event_t event;
  accel.getEvent(&event);

  /* Display the results (acceleration is measured in m/s^2) */
  ROS_INFO("X: ");
  ROS_INFO(event.acceleration.x);
  ROS_INFO("  ");
  ROS_INFO("Y: ");
  ROS_INFO(event.acceleration.y);
  ROS_INFO("  ");
  ROS_INFO("Z: ");
  ROS_INFO(event.acceleration.z);
  ROS_INFO("  ");
  ROS_INFO("m/s^2");

  /* Delay before the next sample */
  delay(500);
}
