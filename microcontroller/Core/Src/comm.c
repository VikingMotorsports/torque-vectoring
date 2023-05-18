#include "comm.h"

adc_buffer  adc_buf1 = { .top=0, .count=0 },
            adc_buf2 = { .top=0, .count=0 },
            adc_buf3 = { .top=0, .count=0 };
throttle_in_buffer throttle_buf = { .top=0, .count=0 };
steering_angle_buffer steering_angle_buf = { .top=0, .count=0 };

uint16_t push_adc_buf(adc_buffer *buf, uint16_t val)
{
  buf->buf[buf->top] = val;
  buf->top = (1 + buf->top) % ADC_BUF_LEN;
  if (buf->count < ADC_BUF_LEN)
    ++buf->count;

  return val;
}

float push_throttle_in_buf(throttle_in_buffer *buf, float val)
{
  buf->buf[buf->top] = val;
  buf->top = (1 + buf->top) % ADC_BUF_LEN;
  if (buf->count < ADC_BUF_LEN)
    ++buf->count;

  return val;
}

float push_steering_angle_buf(steering_angle_buffer *buf, float val)
{
  buf->buf[buf->top] = val;
  buf->top = (1 + buf->top) % ADC_BUF_LEN;
  if (buf->count < ADC_BUF_LEN)
    ++buf->count;

  return val;
}

throttle_percents write_throttle_out(throttle_percents val)
{
  float sum;
  uint16_t left, right;

  sum = val.left + val.right;
  if (sum > MAX_THROTTLE_PERCENT)
  {
    val.left  = val.left  / sum * MAX_THROTTLE_PERCENT;
    val.right = val.right / sum * MAX_THROTTLE_PERCENT;
  }

  left  = (uint16_t)(4095.0 * val.left  / 100.0);
  right = (uint16_t)(4095.0 * val.right / 100.0);


  if (HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, left) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, right) != HAL_OK)
  {
    Error_Handler();
  }

  return val;
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
  float f;
  uint16_t val;

  val = HAL_ADC_GetValue(hadc);
  if (hadc == &hadc1)
  { // Throttle Input
    push_adc_buf(&adc_buf1, val);
    f = convert_throttle_input(val);
    push_throttle_in_buf(&throttle_buf, f);
  }
  else if (hadc == &hadc2)
  { // Brake pedal sensor
    push_adc_buf(&adc_buf2, val);
  }
  else if (hadc == &hadc3)
  { // Steering angle sensor, extra analog input
    push_adc_buf(&adc_buf3, val);
    f = convert_steering_wheel_angle(val);
    f = steering_wheel_angle_to_steering_angle(f);
    push_steering_angle_buf(&steering_angle_buf, f);
  }
}
