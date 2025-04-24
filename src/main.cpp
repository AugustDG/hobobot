#include <Arduino.h>

#include "ir_sensor.h"
#include "line_sensor.h"
#include "utils.hpp"

const u_int16_t IR_PINS[] = {0, 1, 2, 3, 4, 5, 6};
const uint16_t LINE_PINS[] = {14, 15, 16, 17, 20, 21};

ir_sensor *ir_sensors[ARRAY_SIZE(IR_PINS)];
line_sensor *line_sensors[ARRAY_SIZE(LINE_PINS)];

void setup()
{
  // initialize IR sensors
  for (size_t i = 0; i < ARRAY_SIZE(ir_sensors); i++)
  {
    const ir_sensor_config config = {IR_PINS[i], true, nullptr};
    ir_sensors[i] = ir_sensor_create(&config);
  }

  // initialize line sensors
  for (size_t i = 0; i < ARRAY_SIZE(line_sensors); i++)
  {
    const line_sensor_config config = {LINE_PINS[i], nullptr};
    line_sensors[i] = line_sensor_create(&config);
  }
}

void loop()
{
  constexpr size_t NUM_IR_SENSORS = ARRAY_SIZE(ir_sensors);
  constexpr size_t NUM_LINE_SENSORS = ARRAY_SIZE(line_sensors);

  int ir_values[NUM_IR_SENSORS];
  int line_values[NUM_LINE_SENSORS];

  // read IR sensors
  for (size_t i = 0; i < NUM_IR_SENSORS; i++)
    ir_values[i] = ir_sensor_read(ir_sensors[i]);

  // read line sensors
  for (size_t i = 0; i < NUM_LINE_SENSORS; i++)
    line_values[i] = line_sensor_read(line_sensors[i]);

  printf("Sensor Values: ");
  for (size_t i = 0; i < NUM_IR_SENSORS; i++)
    printf("%d ", ir_values[i]);

  printf("| ");
  for (size_t i = 0; i < NUM_LINE_SENSORS; i++)
    printf("%d ", line_values[i]);

  printf("\n");

  delay(10);
}