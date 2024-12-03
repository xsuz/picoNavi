#pragma once

#include<stdint.h>

/// @brief GPSのDeviceData
/// @note 先頭はid
struct GPSData {
  /// @brief デバイス識別子
  uint8_t id;
  /// @brief 時刻
  uint32_t timestamp;
  /// @brief 緯度
  double latitude;
  /// @brief 経度
  double longitude;
  /// @brief GPS高度
  float alt;
  /// @brief 速度(East)
  float ve;
  /// @brief 速度(North)
  float vn;
};

/// @brief PitotのDeviceData
/// @note 先頭はid
struct IMUData {
  /// @brief デバイス識別子
  uint8_t id;
  /// @brief 時刻
  uint32_t timestamp;
  /// @brief 加速度 x
  float a_x;
  /// @brief 加速度 y
  float a_y;
  /// @brief 加速度 z
  float a_z;
  /// @brief 角速度 x
  float w_x;
  /// @brief 角速度 y
  float w_y;
  /// @brief 角速度 z
  float w_z;
};
