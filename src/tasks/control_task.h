// control_task.h — 200Hz 制御タスク (設計書 §3/§4/§6)
// IMU と DXL バスの単一所有者。core0 / 高優先度 / vTaskDelayUntil 5ms。
#pragma once

#include "../core/attitude_estimator.h"
#include "../core/balance_core.h"
#include "../core/param_validation.h"
#include "../core/safety_fsm.h"
#include "../hw/dxl_backend.h"
#include "../hw/imu_backend.h"
#include "../shared/shared_state.h"

namespace tasks {

struct ControlContext {
  hw::ImuBackend* imu;
  hw::DxlBackend* dxl;
  shared::SharedState* shared;
  core::TuningParams params;   // 起動時ロード済み (検証通過 or 既定値)
  bool init_ok;                // main での HW 初期化結果 (false → 即 FAULT)
};

// FreeRTOS タスクエントリ (pvParameters = ControlContext*)
void controlTaskEntry(void* pvParameters);

}  // namespace tasks
