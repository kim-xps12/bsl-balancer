// imu_backend.h — M5Unified IMU ラッパ (設計書 §5)
// 鮮度は update() の sensor_mask のみを根拠とする (usec はサンプル時刻ではない)。
// M5Unified の NVS 隠れオフセットは begin 後に破棄し、校正は自前管理 (制御タスク内)。
#pragma once

#include <cstdint>

#include "../core/attitude_estimator.h"

namespace hw {

class ImuBackend {
 public:
  // M5.begin() 後に呼ぶ。隠れオフセット破棄 + 自動校正凍結 + BMI270 ODR 引き上げ。
  bool init();

  // 毎周期 1 回。M5.Imu.update() → 鮮度付きサンプル (軸マップ・符号適用済み)
  core::ImuSample sample();

  // 直近の gyro 連続 stale 数 (FAULT 判定用)
  int gyroStaleCount() const { return gyro_stale_count_; }

 private:
  int gyro_stale_count_ = 0;
};

}  // namespace hw
