#include "control_task.h"

#include <Arduino.h>
#include <cmath>

#include "../core/units.h"

namespace tasks {
namespace {

using core::FaultReason;
using core::FsmAction;
using core::FsmState;

float nowSeconds() { return static_cast<float>(esp_timer_get_time()) * 1e-6f; }

// 電流妥当性監視 (§6): dwell 付きの単純な監視器
class PlausibilityMonitor {
 public:
  // torque_on 中: |I_pres−I_cmd| 乖離、torque_off: 残留電流を監視
  bool update(bool torque_on, bool fb_valid, float i_cmd, float i_pres, float dt) {
    if (!fb_valid) return false;  // stale 帰還では判定しない
    const float err = std::fabs(i_pres - i_cmd);
    const bool bad = torque_on ? (err > cfg::kCurrentMismatchA)
                               : (std::fabs(i_pres) > cfg::kCurrentResidualA);
    dwell_ = bad ? (dwell_ + dt) : 0.0f;
    return dwell_ > cfg::kCurrentPlausDwellS;
  }
  void reset() { dwell_ = 0.0f; }

 private:
  float dwell_ = 0.0f;
};

struct LoopState {
  core::AttitudeEstimator estimator;
  core::BalanceCore balance;
  core::SafetyFsm fsm;
  PlausibilityMonitor plaus_l, plaus_r;
  core::TuningParams params;
  cfg::Profile profile = cfg::Profile::Bringup;

  int64_t prev_us = 0;
  float dt_max = 0.0f;
  int overrun_count = 0;
  int read_stale_count = 0;
  // 未検証トルク窓 (壁時計 §4.2)
  bool unverified_active = false;
  int64_t unverified_since_us = 0;
  // 保存ゲート
  bool save_gate_active = false;
  int64_t save_gate_since_us = 0;
  uint32_t loop_count = 0;
  hw::HealthInfo health;
  float last_cmd_l = 0.0f, last_cmd_r = 0.0f;
};

void applyBalanceParams(LoopState& ls) {
  core::BalanceCore::Params bp;
  bp.pitch.kp = ls.params.kp;
  bp.pitch.ki = ls.params.ki;
  bp.pitch.kd = ls.params.kd;
  bp.pitch.d_lpf_hz = cfg::kDTermLpfHz;
  bp.pitch.i_limit = cfg::kPitchILimitA;
  bp.pitch.out_limit = cfg::kCurrentPeakA;
  bp.kv = ls.params.kv;
  bp.kvi = ls.params.kvi;
  bp.theta_ref_limit = ls.params.theta_ref_limit;
  bp.vel_loop_enabled = ls.params.vel_loop_enabled;
  bp.wheel_speed_soft = cfg::kWheelSpeedSoftRadS;
  bp.wheel_speed_hard = cfg::kWheelSpeedHardRadS;
  bp.slew_a_per_s = cfg::kCurrentSlewAPerS;
  // ソフト上限はプロファイルの EEPROM Current Limit を超えない (超過指令は
  // XL330 が範囲外エラーを返し verified_write が失敗するため §2.3)
  const float eeprom_limit = cfg::currentLimitFor(ls.profile);
  bp.i2t.i_peak = std::fmin(cfg::kCurrentPeakA, eeprom_limit);
  bp.i2t.i_cont = std::fmin(cfg::kCurrentContA, bp.i2t.i_peak);
  bp.i2t.peak_duration_s = cfg::kPeakDurationS;
  bp.pitch.out_limit = bp.i2t.i_peak;
  ls.balance.setParams(bp);
}

// 後段で検出したフォールトの FSM 反映 + SafeStop 実行を一体化する。
// (update() の戻り action を捨てると Fault 遷移時の Torque OFF が失われる)
void raiseFault(LoopState& ls, ControlContext& ctx, FaultReason reason,
                float now_s) {
  core::SafetyFsm::Input fi;
  fi.fault = reason;
  fi.now_s = now_s;
  const core::SafetyFsm::Result r = ls.fsm.update(fi);
  if (r.action == FsmAction::SafeStop) {
    ctx.dxl->safeStop();  // 未検証なら backend が検疫を発動する (§4.2)
    ls.fsm.notifySafeStopDone();
    ls.last_cmd_l = ls.last_cmd_r = 0.0f;
  }
}

void drainParamQueue(LoopState& ls, shared::SharedState& sh) {
  shared::ParamCommand c;
  bool changed = false;
  while (sh.popParam(&c)) {
    switch (c.field) {
      case shared::ParamField::Kp: ls.params.kp = c.value; break;
      case shared::ParamField::Ki: ls.params.ki = c.value; break;
      case shared::ParamField::Kd: ls.params.kd = c.value; break;
      case shared::ParamField::PitchEq: ls.params.pitch_eq = c.value; break;
      case shared::ParamField::VelLoopEnabled:
        ls.params.vel_loop_enabled = (c.value != 0.0f);
        break;
    }
    changed = true;
  }
  if (changed) {
    // 範囲は §9.1 と同じ表でクランプ (実行時変更も逸脱させない)
    ls.params.kp = units::clampf(ls.params.kp, cfg::kRangeKp.min, cfg::kRangeKp.max);
    ls.params.ki = units::clampf(ls.params.ki, cfg::kRangeKi.min, cfg::kRangeKi.max);
    ls.params.kd = units::clampf(ls.params.kd, cfg::kRangeKd.min, cfg::kRangeKd.max);
    ls.params.pitch_eq =
        units::clampf(ls.params.pitch_eq, cfg::kRangePitchEq.min, cfg::kRangePitchEq.max);
    applyBalanceParams(ls);
  }
}

}  // namespace

void controlTaskEntry(void* pvParameters) {
  ControlContext& ctx = *static_cast<ControlContext*>(pvParameters);
  LoopState ls;
  ls.params = ctx.params;
  ls.profile = ctx.profile;

  // 推定器・制御器・FSM の初期化
  core::AttitudeEstimator::Params ep;
  ep.tau_s = cfg::kEstimatorTauS;
  ep.accel_gate_g = cfg::kAccelGateG;
  ep.gravity = cfg::kGravity;
  ls.estimator.setParams(ep);
  applyBalanceParams(ls);

  core::SafetyFsm::Params fp;
  fp.start_window_rad = cfg::kStartWindowRad;
  fp.start_rate_max = cfg::kStartRateMaxRadS;
  fp.start_wheel_max = cfg::kStartWheelMaxRadS;
  fp.upright_hold_s = cfg::kUprightHoldS;
  fp.fall_threshold_rad = cfg::kFallThresholdRad;
  fp.fall_escalation_count = cfg::kFallEscalationCount;
  fp.fall_escalation_window_s = cfg::kFallEscalationWindowS;
  fp.commissioned = ctx.commissioned;
  fp.auto_arm = cfg::kAutoArmOnBootDefault;
  ls.fsm.setParams(fp);

  if (ctx.init_ok) {
    // 静止校正 (INITIALIZING、心拍は init 内で開始済みの零指令が Watchdog を養う
    // 前提はないため、校正ループ内でも周期心拍を打つ)
    const int calib_cycles =
        static_cast<int>(cfg::kGyroCalibDurationS / cfg::kControlPeriodS);
    double gyro_sum = 0.0, tilt_sum = 0.0;
    int gyro_n = 0, accel_n = 0;
    TickType_t wake = xTaskGetTickCount();
    for (int i = 0; i < calib_cycles; ++i) {
      const core::ImuSample s = ctx.imu->sample();
      if (s.gyro_fresh) { gyro_sum += s.gyro_rate; ++gyro_n; }
      if (s.accel_fresh) { tilt_sum += std::atan2(s.acc_tilt, s.acc_vert); ++accel_n; }
      ctx.dxl->writeZeroHeartbeat();
      vTaskDelayUntil(&wake, pdMS_TO_TICKS(cfg::kControlPeriodMs));
    }
    // gyro/accel それぞれの実サンプル数で平均し、どちらか不足なら InitFailed
    // (accel 欠落を 0 傾斜として飲み込むと重力基準なしでアームしうる)
    if (gyro_n > calib_cycles / 2 && accel_n > calib_cycles / 4) {
      ls.estimator.reset(static_cast<float>(tilt_sum / accel_n),
                         static_cast<float>(gyro_sum / gyro_n));
      ls.fsm.notifyInitDone();
    } else {
      raiseFault(ls, ctx, FaultReason::InitFailed, nowSeconds());  // IMU 不動
    }
  } else {
    raiseFault(ls, ctx, FaultReason::InitFailed, nowSeconds());
  }

  ls.prev_us = esp_timer_get_time();
  TickType_t wake = xTaskGetTickCount();

  for (;;) {
    vTaskDelayUntil(&wake, pdMS_TO_TICKS(cfg::kControlPeriodMs));
    const int64_t now_us = esp_timer_get_time();
    const float dt = static_cast<float>(now_us - ls.prev_us) * 1e-6f;
    ls.prev_us = now_us;
    const float now_s = static_cast<float>(now_us) * 1e-6f;
    ++ls.loop_count;
    if (dt > ls.dt_max) ls.dt_max = dt;

    FaultReason fault = FaultReason::None;

    // デッドライン監視 (§3.3): 連続 N 回で FAULT
    if (dt > cfg::kControlPeriodS * cfg::kDtFaultFactor) {
      if (++ls.overrun_count >= cfg::kDtFaultConsecutive) fault = FaultReason::LoopOverrun;
    } else {
      ls.overrun_count = 0;
    }

    drainParamQueue(ls, *ctx.shared);
    const bool stop_edge = ctx.shared->consumeStopToggle();

    // IMU (§5.2): stale 連続で FAULT
    const core::ImuSample imu = ctx.imu->sample();
    ls.estimator.update(imu, dt);
    if (ctx.imu->gyroStaleCount() >= cfg::kImuStaleFaultCycles) {
      fault = FaultReason::ImuStale;
    }

    // 単一変換点 (§2.1): 以降は 0 中心 θ のみ
    const float theta = ls.estimator.pitchAbs() - ls.params.pitch_eq;
    const float theta_rate = ls.estimator.rate();

    const FsmState state_before = ls.fsm.state();
    const bool torque_on = (state_before == FsmState::Balancing);

    // dt > Watchdog 予算 → Torque Enable 状態に関わらず raw98 検査 (§4.2)
    if (dt > cfg::kBusWatchdogWindowS && !ctx.dxl->quarantineActive() &&
        !ls.save_gate_active) {
      const hw::WatchdogCheck wc = ctx.dxl->checkWatchdog(torque_on, now_s);
      if (wc == hw::WatchdogCheck::Fault) {
        fault = torque_on ? FaultReason::WatchdogTripTorqueOn
                          : FaultReason::WatchdogRecoverRepeated;
      }
    }

    // 周期ブロック読取 (検疫/保存ゲート中は自動的に invalid)
    const hw::WheelFeedback fb = ctx.dxl->readFeedback();
    if (fb.valid) {
      ls.read_stale_count = 0;
    } else if (!ctx.dxl->quarantineActive() && !ls.save_gate_active) {
      if (++ls.read_stale_count >= cfg::kDxlReadStaleFaultCycles) {
        fault = FaultReason::DxlReadStale;
      }
    }
    const float v = cfg::kWheelRadiusM * 0.5f * (fb.omega_left + fb.omega_right);

    // 低頻度ヘルス (64 周期毎 1 項目。劣化サイクルではスキップ §4.2)
    if ((ls.loop_count & 63u) == 0 && dt < cfg::kControlPeriodS * 1.2f &&
        !ctx.dxl->quarantineActive() && !ls.save_gate_active) {
      ctx.dxl->pollHealth(&ls.health);
      if (ls.health.hw_error & 0x80) fault = FaultReason::HwErrorStatus;
      if (ls.health.temperature > cfg::kTempMaxC) fault = FaultReason::OverTemp;
      if (ls.health.voltage > 0.5f && ls.health.voltage < cfg::kVoltageMinV) {
        fault = FaultReason::UnderVoltage;
      }
      if (ls.health.watchdog_tripped && torque_on) {
        fault = FaultReason::WatchdogTripTorqueOn;
      }
    }

    // FSM 更新
    core::SafetyFsm::Input fi;
    fi.theta = theta;
    fi.theta_rate = theta_rate;
    fi.wheel_speed_max = std::fmax(std::fabs(fb.omega_left), std::fabs(fb.omega_right));
    fi.wheel_valid = fb.valid;
    fi.stop_toggle = stop_edge;
    // 保存「要求中」もアーム経路をブロックする (§9.1)。ゲートが開く前の同一
    // 周期で起立検出が先に通ると SAVE タップとトルク ON がレースするため。
    fi.save_in_progress =
        ls.save_gate_active || (ctx.shared->saveRequest() != shared::SaveKind::None);
    fi.fault = fault;
    fi.now_s = now_s;
    fi.dt = dt;
    const core::SafetyFsm::Result fr = ls.fsm.update(fi);

    // FSM アクション実行
    if (fr.action == FsmAction::EnterBalancing) {
      if (ctx.dxl->enterBalancing(now_s)) {
        ls.balance.reset();
        ls.plaus_l.reset();
        ls.plaus_r.reset();
        ls.unverified_active = false;
        ls.fsm.notifyBalancingEntered();
      } else {
        ls.fsm.notifyEntryFailed();
        ctx.dxl->safeStop();
      }
    } else if (fr.action == FsmAction::SafeStop) {
      // 零書込検証→Torque OFF。未検証なら backend が検疫を発動する (§4.2)
      if (!ctx.dxl->safeStop()) {
        raiseFault(ls, ctx, FaultReason::DxlWriteUnverified, now_s);
      } else {
        ls.fsm.notifySafeStopDone();
      }
      ls.last_cmd_l = ls.last_cmd_r = 0.0f;
    }

    // 出力
    core::BalanceCore::Output out;
    const FsmState state_now = ls.fsm.state();
    if (state_now == FsmState::Balancing) {
      core::BalanceCore::Input bi;
      bi.theta = theta;
      bi.theta_rate = theta_rate;
      bi.v = v;
      bi.omega_left = fb.omega_left;
      bi.omega_right = fb.omega_right;
      bi.wheel_valid = fb.valid;
      bi.i_yaw = 0.0f;  // v1: 構造のみ (§2.1)
      bi.dt = dt;
      out = ls.balance.update(bi);

      float cmd_l = out.i_left, cmd_r = out.i_right;
      if (!fb.valid) { cmd_l = 0.0f; cmd_r = 0.0f; }  // 読取失敗周期はコースト (§4.2)
      if (out.hard_overspeed) { cmd_l = cmd_r = 0.0f; }  // 今周期は零指令

      if (cmd_l != out.i_left || cmd_r != out.i_right) {
        // 計算値と送信値が異なる周期はスルーレート状態を送信実績へ整合させる
        ls.balance.overrideOutput(cmd_l, cmd_r);
      }

      // 未検証窓の判定は**次の非零書込より前**に行う (§4.2)。判定を書込後に
      // 置くと、失敗が続くモードで窓超過後にもう 1 回非零を発行してしまう
      const bool window_exceeded =
          ls.unverified_active &&
          static_cast<float>(now_us - ls.unverified_since_us) * 1e-6f >
              cfg::kUnverifiedTorqueMaxS;
      if (window_exceeded) {
        ctx.dxl->writeZeroVerified();  // ベストエフォートの零指令
        raiseFault(ls, ctx, FaultReason::DxlWriteUnverified, now_s);
        ls.balance.overrideOutput(0.0f, 0.0f);
        ls.last_cmd_l = ls.last_cmd_r = 0.0f;
      } else if (ctx.dxl->writeGoalCurrentsVerified(cmd_l, cmd_r)) {
        ls.unverified_active = false;
        ls.last_cmd_l = cmd_l;
        ls.last_cmd_r = cmd_r;
      } else {
        // 配達未検証 → 直ちに検証付き零書込 (§4.2)
        if (!ctx.dxl->writeZeroVerified()) {
          ctx.dxl->engageQuarantine();
          raiseFault(ls, ctx, FaultReason::DxlWriteUnverified, now_s);
        } else {
          if (!ls.unverified_active) {
            ls.unverified_active = true;
            ls.unverified_since_us = now_us;
          }
          // 窓超過の FAULT 判定は次周期先頭 (window_exceeded) で行う
          // 実際に送れたのは零 → スルーレート状態も零へ整合
          ls.balance.overrideOutput(0.0f, 0.0f);
          ls.last_cmd_l = ls.last_cmd_r = 0.0f;
        }
      }

      if (out.hard_overspeed) {
        raiseFault(ls, ctx, FaultReason::HardOverspeed, now_s);
      }
    } else {
      // 心拍不変条件: 全状態で毎周期零書込 (検疫/保存ゲート中は backend が拒否)
      ctx.dxl->writeZeroHeartbeat();
      ls.last_cmd_l = ls.last_cmd_r = 0.0f;
    }

    // 電流妥当性監視 (§6)
    {
      const bool ton = (ls.fsm.state() == FsmState::Balancing);
      const bool pl = ls.plaus_l.update(ton, fb.valid, ls.last_cmd_l, fb.i_left, dt);
      const bool pr = ls.plaus_r.update(ton, fb.valid, ls.last_cmd_r, fb.i_right, dt);
      if (pl || pr) {
        raiseFault(ls, ctx, FaultReason::CurrentPlausibility, now_s);
      }
    }

    // NVS 保存ゲート (§9.1): 要求 → safe-off 検証 → ゲート ON → UI 書込 → 復旧
    if (!ls.save_gate_active) {
      const shared::SaveKind req = ctx.shared->saveRequest();
      const FsmState st = ls.fsm.state();
      const bool torque_off_state =
          (st == FsmState::Idle || st == FsmState::Disarmed || st == FsmState::Fallen);
      if (req != shared::SaveKind::None && torque_off_state &&
          !ctx.dxl->quarantineActive() && ctx.dxl->verifySafeOff()) {
        ls.save_gate_active = true;
        ls.save_gate_since_us = now_us;
        ctx.dxl->setSaveGate(true);
        ctx.shared->setSaveInProgress(true);  // UI が見て NVS を書く
      } else if (req != shared::SaveKind::None && !torque_off_state) {
        ctx.shared->clearSaveRequest();  // BALANCING 中は拒否 (§9.1)
      }
    } else {
      const bool done = ctx.shared->consumeSaveDone();
      const bool timeout =
          static_cast<float>(now_us - ls.save_gate_since_us) * 1e-6f > 3.0f;
      if (done || timeout) {
        ctx.dxl->setSaveGate(false);
        // 保存 = 予期された心拍停止 → raw98 点検/復旧 (§9.1)。復旧失敗や
        // 復旧回数超過は握り潰さずラッチ FAULT へ
        const hw::WatchdogCheck wc =
            ctx.dxl->checkWatchdog(/*torque_may_be_on=*/false, now_s);
        ls.save_gate_active = false;
        ctx.shared->setSaveInProgress(false);
        ctx.shared->clearSaveRequest();
        if (wc == hw::WatchdogCheck::Fault) {
          raiseFault(ls, ctx, FaultReason::WatchdogRecoverRepeated, now_s);
        }
      }
    }

    // スナップショット発行 (seqlock)
    shared::Snapshot sn;
    sn.fsm_state = static_cast<uint8_t>(ls.fsm.state());
    sn.fault_reason = static_cast<uint8_t>(ls.fsm.faultReason());
    sn.commissioned = ctx.commissioned;
    sn.profile = static_cast<uint8_t>(ctx.profile);
    sn.theta = theta;
    sn.theta_rate = theta_rate;
    sn.theta_ref = out.theta_ref;
    sn.v = v;
    sn.omega_left = fb.omega_left;
    sn.omega_right = fb.omega_right;
    sn.i_cmd_left = ls.last_cmd_l;
    sn.i_cmd_right = ls.last_cmd_r;
    sn.i_present_left = fb.i_left;
    sn.i_present_right = fb.i_right;
    sn.dt_last = dt;
    sn.dt_max = ls.dt_max;
    sn.loop_count = ls.loop_count;
    sn.voltage = ls.health.voltage;
    sn.temperature = ls.health.temperature;
    sn.saturated = out.saturated;
    sn.i2t_limited = out.i2t_limited;
    sn.params = ls.params;
    ctx.shared->publish(sn);
  }
}

}  // namespace tasks
