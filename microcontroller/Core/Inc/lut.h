#ifndef __LUT_H
#define __LUT_H

#include <stdint.h>
#include "calc.h"

// In: Steering angle, throttle %
// Out: Left, Right (sum: 100%)

typedef struct {
  uint16_t i; // Steering Angle
  uint16_t j; // Throttle Input
} LUT_index;

typedef struct {
  float i; // Steering Angle
  float j; // Throttle Input
} LUT_index_float;

typedef struct {
  LUT_index top_left;
  LUT_index top_right;
  LUT_index bottom_left;
  LUT_index bottom_right;
  LUT_index_float ref;
} LUT_index_quad;

#define STEERING_ANGLE_RESOLUTION 13
#define THROTTLE_PERCENT_RESOLUTION 13

extern throttle_percents LUT[STEERING_ANGLE_RESOLUTION][THROTTLE_PERCENT_RESOLUTION];

throttle_percents LUT_lookup(LUT_index in);

LUT_index nearest_index(float steering_angle, float throttle_percent);
LUT_index_float index_float(float steering_angle, float throttle_percent);
LUT_index_quad interp_indices(float steering_angle, float throttle_percent);

throttle_percents lookup_nearest(float steering_angle, float throttle_percent);
throttle_percents lookup_linear(float steering_angle, float throttle_percent);

#endif
