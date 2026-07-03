#include "ui_task.h"

#include <M5Unified.h>
#include <cmath>

#include "../app_config.h"
#include "../core/param_validation.h"
#include "../core/safety_fsm.h"
#include "../core/units.h"
#include "../hw/param_store.h"

namespace tasks {
namespace {

using core::FsmState;

constexpr float kRadToDeg = 180.0f / units::kPi;

const char* stateName(uint8_t s) {
  switch (static_cast<FsmState>(s)) {
    case FsmState::Initializing: return "INIT";
    case FsmState::Idle: return "IDLE";
    case FsmState::Balancing: return "BALANCE";
    case FsmState::Fallen: return "FALLEN";
    case FsmState::Disarmed: return "DISARMED";
    case FsmState::Fault: return "FAULT";
  }
  return "?";
}

// 旧 UI と同じ行レイアウト: [-] label: value [+]
void drawRow(const char* label, int y, float value, int decimals) {
  M5.Display.drawRect(20, y, 50, 30, WHITE);
  M5.Display.drawString("-", 40, y, 2);
  M5.Display.drawRect(220, y, 50, 30, WHITE);
  M5.Display.drawString("+", 240, y, 2);
  M5.Display.setCursor(90, y + 5);
  M5.Display.printf("%s: %.*f   ", label, decimals, value);
}

struct PanelState {
  bool visible = false;
  shared::SaveKind pending_save = shared::SaveKind::None;
};

void drawPanel(const shared::Snapshot& sn) {
  M5.Display.setCursor(10, 8);
  M5.Display.printf("%s %s prof:%s dt:%.1fms ",
                    stateName(sn.fsm_state),
                    sn.fault_reason ? "FLT" : "   ",
                    sn.profile == 1 ? "NRM" : "BUP",
                    sn.dt_max * 1000.0f);
  drawRow("Eq[deg]", 30, sn.params.pitch_eq * kRadToDeg, 1);
  drawRow("Kp", 70, sn.params.kp, 2);
  drawRow("Ki", 110, sn.params.ki, 2);
  drawRow("Kd", 150, sn.params.kd, 3);
  M5.Display.setCursor(10, 190);
  M5.Display.printf("th:%+5.1f V:%4.1f Bat:%d%% ", sn.theta * kRadToDeg,
                    sn.voltage, M5.Power.getBatteryLevel());
  // 保存/コミッショニング (torque OFF 状態でのみ有効 §9.1)
  M5.Display.drawRect(20, 210, 110, 28, WHITE);
  M5.Display.drawString("SAVE", 50, 216, 2);
  M5.Display.drawRect(180, 210, 110, 28, WHITE);
  M5.Display.drawString("COMISN", 200, 216, 2);
}

void handleTouch(const shared::Snapshot& sn, shared::SharedState& sh,
                 PanelState& ps) {
  const auto t = M5.Touch.getDetail();
  if (!t.wasPressed()) return;
  const int x = t.x, y = t.y;

  struct Row {
    shared::ParamField field;
    int y;
    float step;
    float value;
  };
  const Row rows[] = {
      {shared::ParamField::PitchEq, 30, 0.2f / kRadToDeg, sn.params.pitch_eq},
      {shared::ParamField::Kp, 70, 0.1f, sn.params.kp},
      {shared::ParamField::Ki, 110, 0.05f, sn.params.ki},
      {shared::ParamField::Kd, 150, 0.01f, sn.params.kd},
  };
  for (const Row& r : rows) {
    if (y < r.y || y >= r.y + 30) continue;
    if (x >= 15 && x < 120) {
      sh.pushParam({r.field, r.value - r.step});
    } else if (x >= 220 && x < 270) {
      sh.pushParam({r.field, r.value + r.step});
    }
    return;
  }

  const bool torque_off =
      sn.fsm_state == static_cast<uint8_t>(FsmState::Idle) ||
      sn.fsm_state == static_cast<uint8_t>(FsmState::Disarmed) ||
      sn.fsm_state == static_cast<uint8_t>(FsmState::Fallen);
  if (y >= 210 && y < 238) {
    if (x >= 20 && x < 130 && torque_off) {
      ps.pending_save = shared::SaveKind::Params;
      sh.requestSave(shared::SaveKind::Params);
    } else if (x >= 180 && x < 290 && torque_off) {
      // 符号試験 (§7) 合格のユーザ明示確認 = コミッショニング (次回起動から有効)
      ps.pending_save = shared::SaveKind::Commission;
      sh.requestSave(shared::SaveKind::Commission);
    } else if (!torque_off) {
      M5.Display.setCursor(10, 190);
      M5.Display.print("STOP first (BtnC hold)   ");
    }
  }
}

// 保存ゲート下の NVS 書込 (§9.1: 制御側が safe-off 検証済みのときのみ来る)
void performSave(const shared::Snapshot& sn, shared::SharedState& sh,
                 PanelState& ps) {
  if (ps.pending_save == shared::SaveKind::Params) {
    hw::ParamStore::saveParams(sn.params);
  } else if (ps.pending_save == shared::SaveKind::Commission) {
    core::CommissioningRecord rec;
    rec.schema_version = cfg::kCommissionSchemaVersion;
    rec.profile = static_cast<uint8_t>(cfg::Profile::Normal);
    rec.sign_axis_checksum = core::currentSignAxisChecksum();
    rec.calib_version = 1;
    rec.user_confirmed = true;
    hw::ParamStore::saveCommissioning(rec);
  }
  ps.pending_save = shared::SaveKind::None;
  sh.setSaveDone();
}

void setExpressionFor(m5avatar::Avatar& avatar, uint8_t state) {
  using m5avatar::Expression;
  switch (static_cast<FsmState>(state)) {
    case FsmState::Balancing: avatar.setExpression(Expression::Neutral); break;
    case FsmState::Idle: avatar.setExpression(Expression::Sleepy); break;
    case FsmState::Fallen: avatar.setExpression(Expression::Sad); break;
    case FsmState::Disarmed: avatar.setExpression(Expression::Doubt); break;
    case FsmState::Fault: avatar.setExpression(Expression::Angry); break;
    default: break;
  }
}

}  // namespace

void uiTaskEntry(void* pvParameters) {
  UiContext& ctx = *static_cast<UiContext*>(pvParameters);
  PanelState ps;
  uint8_t last_state = 0xFF;
  shared::Snapshot sn;          // 最後に読めた正常スナップショットを保持
  bool have_snapshot = false;   // 一度も読めていなければ保存等に使わない
  TickType_t wake = xTaskGetTickCount();

  for (;;) {
    M5.update();

    shared::Snapshot tmp;
    if (ctx.shared->read(&tmp)) {
      sn = tmp;
      have_snapshot = true;
    }

    // BtnC 長押し = STOP/ARM トグル (利便停止 §6。ハード停止は電源スイッチ)
    if (M5.BtnC.wasHold()) ctx.shared->requestStopToggle();

    // BtnA: avatar 再開 (現行 UX 維持)
    if (M5.BtnA.wasPressed() && !ps.visible) ctx.avatar->resume();

    // BtnB: パネル切替
    if (M5.BtnB.wasPressed()) {
      ps.visible = !ps.visible;
      if (ps.visible) {
        ctx.avatar->suspend();
        M5.Display.clear();
      } else {
        M5.Display.clear();
        ctx.avatar->resume();
      }
    }

    if (ps.visible) {
      drawPanel(sn);
      if (M5.Touch.getCount() > 0) handleTouch(sn, *ctx.shared, ps);
    } else if (sn.fsm_state != last_state) {
      setExpressionFor(*ctx.avatar, sn.fsm_state);
    }
    last_state = sn.fsm_state;

    // 保存ゲートが開いたら NVS 書込を実行して完了を報告 (§9.1)。
    // 正常スナップショット未取得のまま既定値/破損値を保存しない
    if (ctx.shared->saveInProgress() && ps.pending_save != shared::SaveKind::None &&
        have_snapshot) {
      performSave(sn, *ctx.shared, ps);
    }

    vTaskDelayUntil(&wake, pdMS_TO_TICKS(cfg::kUiPeriodMs));
  }
}

}  // namespace tasks
