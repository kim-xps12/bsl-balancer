#include "imu_backend.h"

#include <M5Unified.h>
#include <cmath>

#include "../app_config.h"
#include "../core/units.h"

namespace hw {
namespace {

// BMI270 レジスタ (データシート): ACC_CONF=0x40, GYR_CONF=0x42
// M5Unified はこれらを書かずチップデフォルト (gyro 200Hz / accel 100Hz) のまま
// (確定事実 4)。200Hz 制御とのビートを避けるため引き上げる。
constexpr uint8_t kBmi270AccConf = 0x40;
constexpr uint8_t kBmi270GyrConf = 0x42;
constexpr uint8_t kBmi270AccConf200HzPerf = 0xA9;  // odr=200Hz, bwp=norm, perf
constexpr uint8_t kBmi270GyrConf400HzPerf = 0xAA;  // odr=400Hz, bwp=norm, perf
constexpr uint8_t kBmi270ChipIdReg = 0x00;
constexpr uint8_t kBmi270ChipId = 0x24;
constexpr uint32_t kI2cFreq = 400000;

bool raiseBmi270Odr() {
  // BMI270 の I2C アドレスを実測 (0x68 / 0x69)
  uint8_t addr = 0;
  for (uint8_t cand : {uint8_t(0x68), uint8_t(0x69)}) {
    if (M5.In_I2C.readRegister8(cand, kBmi270ChipIdReg, kI2cFreq) == kBmi270ChipId) {
      addr = cand;
      break;
    }
  }
  if (addr == 0) return false;
  bool ok = M5.In_I2C.writeRegister8(addr, kBmi270AccConf, kBmi270AccConf200HzPerf, kI2cFreq);
  ok &= M5.In_I2C.writeRegister8(addr, kBmi270GyrConf, kBmi270GyrConf400HzPerf, kI2cFreq);
  return ok;
}

// 軸マップ: 縦置き (画面鉛直)。傾斜面 = YZ、傾斜レート = X 軸まわり (§5.1)。
// v1.1(BMI270) の実装向き差異は §7 の符号試験で確認し、ここを config で吸収する。
void mapAxes(const m5::imu_data_t& d, core::ImuSample* s) {
  s->gyro_rate = d.gyro.x * (units::kPi / 180.0f);  // M5Unified は dps
  s->acc_tilt = d.accel.y * cfg::kGravity;          // M5Unified は g 単位
  s->acc_vert = d.accel.z * cfg::kGravity;
  const float ax = d.accel.x, ay = d.accel.y, az = d.accel.z;
  s->acc_norm = std::sqrt(ax * ax + ay * ay + az * az) * cfg::kGravity;
}

}  // namespace

bool ImuBackend::init() {
  if (!M5.Imu.isEnabled()) return false;

  // 隠れ状態の破棄と自動校正の凍結 (確定事実 5)
  M5.Imu.clearOffsetData();
  M5.Imu.setCalibration(0, 0, 0);

  if (M5.Imu.getType() == m5::imu_t::imu_bmi270) {
    raiseBmi270Odr();  // 失敗してもデフォルト ODR で動作は可能 (gyro 200Hz)
  }
  return true;
}

core::ImuSample ImuBackend::sample() {
  core::ImuSample s;
  const auto mask = M5.Imu.update();  // 鮮度の唯一の根拠 (§5.2)
  m5::imu_data_t d;
  M5.Imu.getImuData(&d);
  mapAxes(d, &s);
  s.gyro_fresh = (mask & m5::IMU_Class::sensor_mask_gyro) != 0;
  s.accel_fresh = (mask & m5::IMU_Class::sensor_mask_accel) != 0;
  gyro_stale_count_ = s.gyro_fresh ? 0 : (gyro_stale_count_ + 1);
  return s;
}

}  // namespace hw
