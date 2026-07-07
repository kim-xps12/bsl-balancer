// BSL-Balancer (ﾀｲﾘﾝﾁｬﾝ) — XL330-M077 電流制御モード + 200Hz カスケード PID
// アーキテクチャ: docs/plans/2026-07-02-current-mode-freertos-redesign.md
// main.cpp は配線とタスク生成のみ (§8)。
#include <Arduino.h>

#include <M5Unified.h>
#include <Avatar.h>

#include "TairinEye.h"
#include "TairinMouth.h"

#include "app_config.h"
#include "core/param_validation.h"
#include "hw/dxl_backend.h"
#include "hw/imu_backend.h"
#include "hw/param_store.h"
#include "shared/shared_state.h"
#include "tasks/control_task.h"
#include "tasks/telemetry_task.h"
#include "tasks/ui_task.h"

using namespace m5avatar;

namespace {

HardwareSerial& DXL_SERIAL = Serial1;

Avatar avatar;
Face* tairinFace = nullptr;

hw::ImuBackend imu_backend;
hw::DxlBackend dxl_backend(DXL_SERIAL);
shared::SharedState shared_state;
tasks::ControlContext control_ctx;
tasks::UiContext ui_ctx;

Face* createTairinFace() {
  return new Face(new tairinMouth(50, 90, 4, 60), new tairinEye(8, false),
                  new tairinEye(8, true), new Eyeblow(32, 0, false),
                  new Eyeblow(32, 0, true));
}

}  // namespace

void setup() {
  Serial.begin(115200);

  M5.begin();
  M5.Display.setTextSize(2);
  M5.BtnC.setHoldThresh(cfg::kBtnLongPressMs);

  // avatar (製品アイデンティティ維持 §9)
  tairinFace = createTairinFace();
  avatar.setFace(tairinFace);
  avatar.init();

  // NVS ロード (Torque ON 前に完結 §9.1)。無効レコードは既定値へ fail-closed。
  core::TuningParams params;  // 既定値で初期化済み
  hw::ParamStore::loadParams(&params);

  // HW 初期化 (IMU / DXL §4.1)。互いに独立して実行する — IMU が死んでいても
  // DXL 初期化 (前回稼働の残留トルクの解除経路) は必ず走らせる。
  // 失敗時もタスクは起動し FSM が FAULT を表示する。
  const bool imu_ok = imu_backend.init();
  DXL_SERIAL.begin(cfg::kDxlBaud, SERIAL_8N1, cfg::kPinRxServo, cfg::kPinTxServo);
  // バースト的なバス不調 (実測) に備えて初期化シーケンス全体も再試行する
  bool dxl_ok = false;
  for (int attempt = 0; attempt < 3 && !dxl_ok; ++attempt) {
    if (attempt > 0) delay(50);
    dxl_ok = dxl_backend.init();
  }
  const bool init_ok = imu_ok && dxl_ok;
  Serial.printf("[BOOT] imu_ok=%d dxl_ok=%d\n", imu_ok, dxl_ok);

  control_ctx.imu = &imu_backend;
  control_ctx.dxl = &dxl_backend;
  control_ctx.shared = &shared_state;
  control_ctx.params = params;
  control_ctx.init_ok = init_ok;

  ui_ctx.shared = &shared_state;
  ui_ctx.avatar = &avatar;

  // タスク生成 (§3.1): 制御 = core0 / 20、UI = core1 / 3 (avatar タスクより上)
  xTaskCreatePinnedToCore(tasks::controlTaskEntry, "ControlTask", 8192,
                          &control_ctx, 20, nullptr, 0);
  xTaskCreatePinnedToCore(tasks::uiTaskEntry, "UiTask", 8192, &ui_ctx, 3,
                          nullptr, 1);
  // UDP telemetry (計画書 §3.1): secrets 不在時は内部で no-op になるため、
  // 呼び出し側 (ここ) に #ifdef は置かない。
  tasks::startTelemetryTask(shared_state);
}

void loop() {
  vTaskDelete(nullptr);  // loopTask は不要 (§3.1)
}
