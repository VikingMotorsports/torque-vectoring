#include "lut.h"

#define MIN_STEERING_ANGLE (-0.587965415313701)   // constant: -230deg => steering angle
#define MAX_STEERING_ANGLE  0.527998108411473   // constant:  230deg => steering angle

throttle_percents LUT[STEERING_ANGLE_RESOLUTION][THROTTLE_PERCENT_RESOLUTION];

throttle_percents LUT_lookup(LUT_index in)
{
  return LUT[in.i][in.j];
}

LUT_index nearest_index(float steering_angle, float throttle_percent)
{
  LUT_index_float ref, shifted;

  ref = index_float(steering_angle, throttle_percent);
  shifted = (LUT_index_float){
    .i = ref.i + 0.5,
    .j = ref.j + 0.5,
  };

  return (LUT_index){
    .i = (uint16_t)shifted.i,
    .j = (uint16_t)shifted.j,
  };
}

LUT_index_float index_float(float steering_angle, float throttle_percent)
{
  float i, j;

  i = steering_angle; // [min, max)
  i -= MIN_STEERING_ANGLE; // [0, max - min)
  i /= MAX_STEERING_ANGLE - MIN_STEERING_ANGLE; // [0, 1)
  i *= STEERING_ANGLE_RESOLUTION; // [0, res)

  j = throttle_percent; // [0, 100]
  j /= 100.0; // [0, 1]
  j *= THROTTLE_PERCENT_RESOLUTION; // [0, res]

  return (LUT_index_float){
    .i = i,
    .j = j,
  };
}

LUT_index_quad interp_indices(float steering_angle, float throttle_percent)
{
  LUT_index_float ref;
  LUT_index top_left;
  int dx, dy;

  dx = dy = 1;

  ref = index_float(steering_angle, throttle_percent);
  top_left = (LUT_index){
    .i = (uint16_t)ref.i,
    .j = (uint16_t)ref.j,
  };

  if (top_left.i == STEERING_ANGLE_RESOLUTION)
    dx = 0;

  if (top_left.j == THROTTLE_PERCENT_RESOLUTION)
    dy = 0;

  return (LUT_index_quad){
    .top_left     = top_left,
    .top_right    = { top_left.i + dx, top_left.j,      },
    .bottom_left  = { top_left.i,      top_left.j + dy, },
    .bottom_right = { top_left.i + dx, top_left.j + dy, },
    .ref = ref,
  };
}

throttle_percents lookup_nearest(float steering_angle, float throttle_percent)
{
  LUT_index in = nearest_index(steering_angle, throttle_percent);
  return LUT_lookup(in);
}

// Private
float lerp(float a, float b, float t)
{
  return a*(1-t) + b*t;
}

throttle_percents lookup_linear(float steering_angle, float throttle_percent)
{
  LUT_index_quad in;
  LUT_index_float ref;
  throttle_percents top_left, top_right, bottom_left, bottom_right,
                    top, bottom;
  float s, t;
  in = interp_indices(steering_angle, throttle_percent);
  s = in.ref.i - in.top_left.i;
  t = in.ref.j - in.top_left.j;

  top_left      = LUT_lookup(in.top_left);
  top_right     = LUT_lookup(in.top_right);
  bottom_left   = LUT_lookup(in.bottom_left);
  bottom_right  = LUT_lookup(in.bottom_right);

  top = (throttle_percents){
    .left  = lerp(top_left.left,  top_right.left,  s),
    .right = lerp(top_left.right, top_right.right, s),
  };

  bottom = (throttle_percents){
    .left  = lerp(bottom_left.left,  bottom_right.left,  s),
    .right = lerp(bottom_left.right, bottom_right.right, s),
  };

  return (throttle_percents){
    .left  = lerp(top.left,  bottom.left,  t),
    .right = lerp(top.right, bottom.right, t),
  };
}

