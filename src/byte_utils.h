#pragma once

#include <stdint.h>

/* バイトオーダー変換 */
template<typename T>
void swap32(T *val) {
  uint8_t *u8 = (uint8_t *)val;
  uint8_t tmp;
  // swap the bytes into a temporary buffer
  tmp = u8[0];
  u8[0] = u8[3];
  u8[3] = tmp;
  tmp = u8[1];
  u8[1] = u8[2];
  u8[2] = tmp;
}

template<typename T>
void swap64(T *val) {
  uint8_t *u8 = (uint8_t *)val;
  uint8_t tmp;
  // swap the bytes into a temporary buffer
  tmp = u8[0];
  u8[0] = u8[7];
  u8[7] = tmp;

  tmp = u8[1];
  u8[1] = u8[6];
  u8[6] = tmp;

  tmp = u8[2];
  u8[2] = u8[5];
  u8[5] = tmp;

  tmp = u8[3];
  u8[3] = u8[4];
  u8[4] = tmp;
}
