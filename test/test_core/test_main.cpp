// test_main.cpp — 純ロジック層のユニットテスト (設計書 §8 受入テスト)
// 実行: ~/.platformio/penv/bin/pio test -e native
// 注: Unity は math ヘッダ未取込時に isnan/isinf をマクロ定義し libc++ の
// <cmath> を壊すため、C++ ヘッダを unity.h より先に include する。
#include <cmath>

#include "../../src/core/attitude_estimator.h"
#include "../../src/core/balance_core.h"
#include "../../src/core/dt_stats.h"
#include "../../src/core/dxl_verify.h"
#include "../../src/core/param_validation.h"
#include "../../src/core/pid.h"
#include "../../src/core/plausibility_monitor.h"
#include "../../src/core/safety_fsm.h"
#include "../../src/core/units.h"
#include "../../src/core/watchdog_policy.h"

#include <unity.h>

using namespace core;

void setUp() {}
void tearDown() {}

// ---------------- units (§19.1 単位変換試験) ----------------

static void test_units_current() {
  TEST_ASSERT_EQUAL_INT16(1000, units::currentAToRaw(1.0f));
  TEST_ASSERT_EQUAL_INT16(-1000, units::currentAToRaw(-1.0f));
  TEST_ASSERT_EQUAL_INT16(451, units::currentAToRaw(0.4514f));
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, -0.3f, units::rawToCurrentA(-300));
}

static void test_units_velocity() {
  // raw 1 = 0.229 rpm = 0.0239808 rad/s
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0239808f, units::rawToVelRadS(1));
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, -2.39808f, units::rawToVelRadS(-100));
}

static void test_units_position() {
  TEST_ASSERT_FLOAT_WITHIN(1e-5f, 2.0f * units::kPi, units::rawToPosRad(4096));
  TEST_ASSERT_FLOAT_WITHIN(1e-5f, -units::kPi, units::rawToPosRad(-2048));
}

static void test_units_twos_complement() {
  const uint8_t neg1_16[] = {0xFF, 0xFF};
  const uint8_t neg1_32[] = {0xFF, 0xFF, 0xFF, 0xFF};
  const uint8_t pos300[] = {0x2C, 0x01};
  TEST_ASSERT_EQUAL_INT16(-1, units::le16(neg1_16));
  TEST_ASSERT_EQUAL_INT32(-1, units::le32(neg1_32));
  TEST_ASSERT_EQUAL_INT16(300, units::le16(pos300));
}

// ---------------- PID ----------------

static void test_pid_p_and_d() {
  Pid pid;
  Pid::Params p;
  p.kp = 2.0f;
  p.kd = 0.5f;
  p.d_lpf_hz = 0.0f;  // フィルタ無効で厳密比較
  p.i_limit = 0.0f;
  p.out_limit = 0.0f;
  pid.setParams(p);
  pid.reset();
  // e = 0.1, rate = 0.2 → u = 2*0.1 - 0.5*0.2 = 0.1
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.1f, pid.update(0.1f, 0.0f, 0.2f, 0.005f));
}

static void test_pid_anti_windup() {
  Pid pid;
  Pid::Params p;
  p.kp = 1.0f;
  p.ki = 10.0f;
  p.i_limit = 0.2f;
  p.out_limit = 0.3f;
  p.d_lpf_hz = 0.0f;
  pid.setParams(p);
  pid.reset();

  // (1) P 単独で飽和する大誤差では積分がそもそも蓄積しない (条件付き積分)
  for (int i = 0; i < 200; ++i) pid.update(1.0f, 0.0f, 0.0f, 0.005f);
  TEST_ASSERT_FLOAT_WITHIN(1e-9f, 0.0f, pid.iTerm());

  // (2) 線形領域の小誤差では積分が蓄積し、i_limit でクランプされる
  //     e=0.05 → u = 0.05 + i ≤ 0.25 < out_limit (非飽和を維持)
  for (int i = 0; i < 200; ++i) pid.update(0.05f, 0.0f, 0.0f, 0.005f);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.2f, pid.iTerm());

  // (3) i=+0.2 のまま大正誤差 → 飽和を深める向きで凍結
  pid.update(1.0f, 0.0f, 0.0f, 0.005f);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.2f, pid.iTerm());

  // (4) 大負誤差 → 出力は負側に飽和するが、|i| を減らす巻き戻しは許可される
  pid.update(-1.0f, 0.0f, 0.0f, 0.005f);
  TEST_ASSERT_TRUE(pid.iTerm() < 0.2f - 1e-6f);
}

static void test_pid_d_lpf() {
  Pid pid;
  Pid::Params p;
  p.kd = 1.0f;
  p.d_lpf_hz = 25.0f;
  pid.setParams(p);
  pid.reset();
  pid.update(0.0f, 0.0f, 0.0f, 0.005f);   // フィルタ初期化 (rate=0)
  pid.update(0.0f, 0.0f, 1.0f, 0.005f);   // ステップ入力
  // PT1: 1 ステップでは追従しきらない
  TEST_ASSERT_TRUE(pid.rateFilt() > 0.0f);
  TEST_ASSERT_TRUE(pid.rateFilt() < 1.0f);
}

// ---------------- 推定器 (§5.2) ----------------

static AttitudeEstimator makeEstimator(float tau = 0.5f) {
  AttitudeEstimator est;
  AttitudeEstimator::Params p;
  p.tau_s = tau;
  p.accel_gate_g = 0.3f;
  est.setParams(p);
  est.reset(0.0f, 0.0f);
  return est;
}

static ImuSample tiltSample(float tilt_rad, float g = 9.80665f) {
  ImuSample s;
  s.gyro_rate = 0.0f;
  s.gyro_fresh = true;
  s.acc_tilt = g * std::sin(tilt_rad);
  s.acc_vert = g * std::cos(tilt_rad);
  s.acc_norm = g;
  s.accel_fresh = true;
  return s;
}

static void test_estimator_convergence() {
  AttitudeEstimator est = makeEstimator(0.5f);
  const ImuSample s = tiltSample(0.1f);
  for (int i = 0; i < 1000; ++i) est.update(s, 0.005f);  // 5 s
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.1f, est.pitchAbs());
}

static void test_estimator_accel_gate() {
  AttitudeEstimator est = makeEstimator(0.5f);
  ImuSample s = tiltSample(0.5f);
  s.acc_norm = 2.0f * 9.80665f;  // ||a|-1g| = 1g > 0.3g → 補正停止
  for (int i = 0; i < 1000; ++i) est.update(s, 0.005f);
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.0f, est.pitchAbs());  // gyro=0 なので不動
}

static void test_estimator_stale_hold() {
  AttitudeEstimator est = makeEstimator(0.5f);
  ImuSample s;
  s.gyro_rate = 0.2f;
  s.gyro_fresh = true;
  s.accel_fresh = false;
  est.update(s, 0.005f);
  const float p1 = est.pitchAbs();
  s.gyro_fresh = false;  // stale → 前回レートで予測ホールド
  est.update(s, 0.005f);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, p1 + 0.2f * 0.005f, est.pitchAbs());
}

static void test_estimator_bias() {
  AttitudeEstimator est = makeEstimator(0.5f);
  ImuSample s = tiltSample(0.0f);
  s.gyro_rate = 0.05f;  // 一定バイアス
  for (int i = 0; i < 20000; ++i) est.update(s, 0.005f);  // 100 s
  TEST_ASSERT_FLOAT_WITHIN(0.005f, 0.05f, est.bias());
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, est.pitchAbs());
}

// ---------------- balance_core ----------------

static BalanceCore::Params testBalanceParams() {
  BalanceCore::Params p;
  p.pitch.kp = 2.0f;
  p.pitch.ki = 0.0f;
  p.pitch.kd = 0.0f;
  p.pitch.d_lpf_hz = 0.0f;
  p.pitch.i_limit = 0.1f;
  p.pitch.out_limit = 0.45f;
  p.kv = 0.1f;
  p.kvi = 0.05f;
  p.theta_ref_limit = 0.0524f;
  p.vel_loop_enabled = true;
  p.wheel_speed_soft = 25.0f;
  p.wheel_speed_hard = 35.0f;
  p.slew_a_per_s = 10000.0f;  // テストではスルー制限を実質無効化
  p.i2t.i_peak = 0.45f;
  p.i2t.i_cont = 0.30f;
  p.i2t.peak_duration_s = 1000.0f;  // テストでは I²t を実質無効化
  return p;
}

static void test_restoring_direction() {
  // 復元則 (gate2 P1 回帰): 前傾 θ>0 → 前進電流 (正)。レートも同方向に寄与。
  BalanceCore bc;
  bc.setParams(testBalanceParams());
  bc.reset();
  BalanceCore::Input in;
  in.theta = 0.1f;  // 前傾
  in.wheel_valid = true;
  in.dt = 0.005f;
  const BalanceCore::Output out = bc.update(in);
  TEST_ASSERT_TRUE(out.i_left > 0.0f);   // kp=2 → +0.2 A (前進)
  TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.2f, out.i_left);
  // 前進中 (v>0) は減速のため後傾参照 (θ_ref<0)
  bc.reset();
  in.theta = 0.0f;
  in.v = 0.5f;
  const BalanceCore::Output out2 = bc.update(in);
  TEST_ASSERT_TRUE(out2.theta_ref < 0.0f);
}

static void test_mixer_inversion_priority() {
  BalanceCore bc;
  bc.setParams(testBalanceParams());
  bc.reset();
  BalanceCore::Input in;
  in.theta = 1.0f;   // 巨大前傾 → i_common = +2.0 → clamp +0.45 (飽和)
  in.wheel_valid = true;
  in.i_yaw = 0.2f;   // ヘッドルーム 0 → yaw は完全に諦める
  in.dt = 0.005f;
  const BalanceCore::Output out = bc.update(in);
  TEST_ASSERT_TRUE(out.saturated);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, out.i_left, out.i_right);  // 倒立優先: 差動 0
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.45f, out.i_left);
}

static void test_mixer_yaw_headroom() {
  BalanceCore bc;
  bc.setParams(testBalanceParams());
  bc.reset();
  BalanceCore::Input in;
  in.theta = 0.1f;   // i_common = +0.2
  in.wheel_valid = true;
  in.i_yaw = 0.5f;   // headroom = 0.25 → clamp
  in.dt = 0.005f;
  const BalanceCore::Output out = bc.update(in);
  TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.2f - 0.25f, out.i_left);
  TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.2f + 0.25f, out.i_right);
}

static void test_slew_override_after_coast() {
  // stale コースト時に送信実績 (0) へ整合させると、次周期は 0 起点で
  // スルーレート制限される (gate2 P2 回帰)
  BalanceCore::Params p = testBalanceParams();
  p.slew_a_per_s = 10.0f;  // 1 周期 (5ms) あたり 0.05 A
  BalanceCore bc;
  bc.setParams(p);
  bc.reset();
  BalanceCore::Input in;
  in.theta = 1.0f;  // 大電流要求
  in.wheel_valid = true;
  in.dt = 0.005f;
  bc.update(in);                 // 計算は進む (0 → 0.05)
  bc.overrideOutput(0.0f, 0.0f); // だが実際は 0 を送った (コースト)
  const BalanceCore::Output out = bc.update(in);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.05f, out.i_left);  // 0 起点の 1 ステップ分
}

static void test_speed_guard() {
  bool hard = false;
  // 加速方向 (i*ω>0) かつ soft 超 → 線形縮小: ω=30, soft25, hard35 → scale 0.5
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.05f,
                           applySpeedGuard(0.1f, 30.0f, 25.0f, 35.0f, &hard));
  TEST_ASSERT_FALSE(hard);
  // 減速方向は素通し
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, -0.1f,
                           applySpeedGuard(-0.1f, 30.0f, 25.0f, 35.0f, &hard));
  TEST_ASSERT_FALSE(hard);
  // ハード超過 → 0 + フォールト
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f,
                           applySpeedGuard(0.1f, 36.0f, 25.0f, 35.0f, &hard));
  TEST_ASSERT_TRUE(hard);
}

static void test_i2t_symmetric_convergence() {
  I2tLimiter lim;
  I2tLimiter::Params p;
  p.i_peak = 0.45f;
  p.i_cont = 0.30f;
  p.peak_duration_s = 0.5f;
  lim.setParams(p);
  lim.reset();
  // +ピーク持続 → peak_duration 後に連続上限へ収束
  float out = 0.0f;
  for (int i = 0; i < 200; ++i) out = lim.apply(0.45f, 0.005f);  // 1.0 s
  TEST_ASSERT_TRUE(lim.limited());
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.30f, out);
  // 零指令で回復
  for (int i = 0; i < 400; ++i) out = lim.apply(0.0f, 0.005f);
  TEST_ASSERT_FALSE(lim.limited());
  // −ピーク持続でも対称に収束
  lim.reset();
  for (int i = 0; i < 200; ++i) out = lim.apply(-0.45f, 0.005f);
  TEST_ASSERT_TRUE(lim.limited());
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, -0.30f, out);
}

static void test_balance_stale_freezes_outer_loop() {
  BalanceCore bc;
  bc.setParams(testBalanceParams());
  bc.reset();
  BalanceCore::Input in;
  in.theta = 0.0f;
  in.v = 0.5f;  // 前進中 → 積分が動く
  in.wheel_valid = true;
  in.dt = 0.005f;
  bc.update(in);
  const float i1 = bc.velIntegrator();
  TEST_ASSERT_TRUE(std::fabs(i1) > 0.0f);
  in.wheel_valid = false;  // stale → 凍結 (§4.2)
  bc.update(in);
  TEST_ASSERT_FLOAT_WITHIN(1e-9f, i1, bc.velIntegrator());
}

// ---------------- FSM (§6) ----------------

static SafetyFsm::Params fsmParams(bool commissioned = true, bool auto_arm = true) {
  SafetyFsm::Params p;
  p.start_window_rad = 0.0873f;
  p.start_rate_max = 0.35f;
  p.start_wheel_max = 1.0f;
  p.upright_hold_s = 1.0f;
  p.fall_threshold_rad = 0.611f;
  p.fall_escalation_count = 3;
  p.fall_escalation_window_s = 30.0f;
  p.auto_arm = auto_arm;
  p.commissioned = commissioned;
  return p;
}

static SafetyFsm::Input uprightInput(float now_s) {
  SafetyFsm::Input in;
  in.theta = 0.0f;
  in.theta_rate = 0.0f;
  in.wheel_speed_max = 0.0f;
  in.wheel_valid = true;
  in.now_s = now_s;
  in.dt = 0.005f;
  return in;
}

// now_s から hold+margin まで直立入力を流し、最初の EnterBalancing を返す
static bool driveUntilArm(SafetyFsm& fsm, float* now_s, float duration_s) {
  bool requested = false;
  const int steps = static_cast<int>(duration_s / 0.005f);
  for (int i = 0; i < steps; ++i) {
    *now_s += 0.005f;
    const SafetyFsm::Result r = fsm.update(uprightInput(*now_s));
    if (r.action == FsmAction::EnterBalancing) requested = true;
  }
  return requested;
}

static void test_fsm_boot_gate() {
  SafetyFsm fsm;
  fsm.setParams(fsmParams(true, true));
  fsm.notifyInitDone();
  TEST_ASSERT_EQUAL(static_cast<int>(FsmState::Idle), static_cast<int>(fsm.state()));

  SafetyFsm fsm2;
  fsm2.setParams(fsmParams(false, true));  // 未コミッショニング → 明示アーム必須
  fsm2.notifyInitDone();
  TEST_ASSERT_EQUAL(static_cast<int>(FsmState::Disarmed), static_cast<int>(fsm2.state()));
}

static void test_fsm_arm_sequence() {
  SafetyFsm fsm;
  fsm.setParams(fsmParams());
  fsm.notifyInitDone();
  float now = 0.0f;
  TEST_ASSERT_TRUE(driveUntilArm(fsm, &now, 1.2f));
  fsm.notifyBalancingEntered();
  TEST_ASSERT_EQUAL(static_cast<int>(FsmState::Balancing), static_cast<int>(fsm.state()));
}

static void test_fsm_zero_centered_theta_contract() {
  // pitch_eq≠0 でも θ=pitch_abs−pitch_eq が窓内なら起立扱い (単一変換点契約)
  SafetyFsm fsm;
  fsm.setParams(fsmParams());
  fsm.notifyInitDone();
  const float pitch_abs = 0.70f;
  const float pitch_eq = 0.65f;  // 実機で平衡が 0.65rad の取り付けを想定
  float now = 0.0f;
  bool requested = false;
  for (int i = 0; i < 260; ++i) {
    now += 0.005f;
    SafetyFsm::Input in = uprightInput(now);
    in.theta = pitch_abs - pitch_eq;  // = 0.05 < 窓 0.0873
    const SafetyFsm::Result r = fsm.update(in);
    if (r.action == FsmAction::EnterBalancing) requested = true;
  }
  TEST_ASSERT_TRUE(requested);
}

static void test_fsm_no_stale_upright_timer_after_fall() {
  // アーム前の保持タイマが Fallen の静置検出に持ち越されないこと (gate2 P2 回帰)
  SafetyFsm fsm;
  fsm.setParams(fsmParams());
  fsm.notifyInitDone();
  float now = 0.0f;
  driveUntilArm(fsm, &now, 1.2f);
  fsm.notifyBalancingEntered();
  // 長時間バランス後に転倒
  now += 10.0f;
  SafetyFsm::Input in = uprightInput(now += 0.005f);
  in.theta = 0.7f;
  fsm.update(in);
  fsm.notifySafeStopDone();
  // 最初の直立サンプル 1 回では Idle に戻らない (新規に 1.0s を要求)
  const SafetyFsm::Result r = fsm.update(uprightInput(now += 0.005f));
  TEST_ASSERT_EQUAL(static_cast<int>(FsmState::Fallen), static_cast<int>(r.state));
}

static void test_fsm_fall_and_recover() {
  SafetyFsm fsm;
  fsm.setParams(fsmParams());
  fsm.notifyInitDone();
  float now = 0.0f;
  driveUntilArm(fsm, &now, 1.2f);
  fsm.notifyBalancingEntered();

  SafetyFsm::Input in = uprightInput(now += 0.005f);
  in.theta = 0.7f;  // 転倒
  SafetyFsm::Result r = fsm.update(in);
  TEST_ASSERT_EQUAL(static_cast<int>(FsmState::Fallen), static_cast<int>(r.state));
  TEST_ASSERT_EQUAL(static_cast<int>(FsmAction::SafeStop), static_cast<int>(r.action));
  fsm.notifySafeStopDone();

  // 静置 1.0s → Idle → さらに 1.0s → 再アーム要求 (現行 UX)
  bool rearmed = false;
  for (int i = 0; i < 600; ++i) {
    now += 0.005f;
    const SafetyFsm::Result rr = fsm.update(uprightInput(now));
    if (rr.action == FsmAction::EnterBalancing) { rearmed = true; break; }
  }
  TEST_ASSERT_TRUE(rearmed);
}

static void test_fsm_fall_escalation() {
  SafetyFsm fsm;
  fsm.setParams(fsmParams());
  fsm.notifyInitDone();
  float now = 0.0f;
  for (int fall = 0; fall < 3; ++fall) {
    // 転倒後は Fallen静置1.0s + Idle起立1.0s の二段ゲートを通過して再アーム
    driveUntilArm(fsm, &now, 2.4f);
    fsm.notifyBalancingEntered();
    SafetyFsm::Input in = uprightInput(now += 0.005f);
    in.theta = 0.7f;
    fsm.update(in);
    fsm.notifySafeStopDone();
  }
  TEST_ASSERT_EQUAL(static_cast<int>(FsmState::Fault), static_cast<int>(fsm.state()));
  TEST_ASSERT_EQUAL(static_cast<int>(FaultReason::FallEscalation),
                    static_cast<int>(fsm.faultReason()));
}

static void test_fsm_fall_escalation_sliding_window() {
  // 先頭基準リセットでは脱落するパターン (gate2 P2 回帰):
  // 転倒 t≈2.4, 12.4, 33, 35.4 → 後ろ3件が30s窓に収まり4回目でFAULT
  SafetyFsm fsm;
  fsm.setParams(fsmParams());
  fsm.notifyInitDone();
  float now = 0.0f;
  auto doFall = [&fsm, &now]() {
    driveUntilArm(fsm, &now, 2.4f);
    fsm.notifyBalancingEntered();
    SafetyFsm::Input in = uprightInput(now += 0.005f);
    in.theta = 0.7f;
    fsm.update(in);
    fsm.notifySafeStopDone();
  };
  doFall();          // t≈2.4
  now += 7.6f;
  doFall();          // t≈12.4
  now += 18.2f;
  doFall();          // t≈33 (先頭2.4は窓外→旧実装はここでカウント1にリセット)
  TEST_ASSERT_EQUAL(static_cast<int>(FsmState::Fallen), static_cast<int>(fsm.state()));
  doFall();          // t≈35.4 → (12.4, 33, 35.4) の3件が30s窓内 → FAULT
  TEST_ASSERT_EQUAL(static_cast<int>(FsmState::Fault), static_cast<int>(fsm.state()));
  TEST_ASSERT_EQUAL(static_cast<int>(FaultReason::FallEscalation),
                    static_cast<int>(fsm.faultReason()));
}

static void test_fsm_stop_toggle() {
  SafetyFsm fsm;
  fsm.setParams(fsmParams());
  fsm.notifyInitDone();
  float now = 0.0f;
  driveUntilArm(fsm, &now, 1.2f);
  fsm.notifyBalancingEntered();

  SafetyFsm::Input in = uprightInput(now += 0.005f);
  in.stop_toggle = true;  // BALANCING 中の STOP
  SafetyFsm::Result r = fsm.update(in);
  TEST_ASSERT_EQUAL(static_cast<int>(FsmState::Disarmed), static_cast<int>(r.state));
  TEST_ASSERT_EQUAL(static_cast<int>(FsmAction::SafeStop), static_cast<int>(r.action));
  fsm.notifySafeStopDone();

  // 姿勢では復帰しない
  for (int i = 0; i < 400; ++i) {
    now += 0.005f;
    const SafetyFsm::Result rr = fsm.update(uprightInput(now));
    TEST_ASSERT_EQUAL(static_cast<int>(FsmState::Disarmed), static_cast<int>(rr.state));
  }
  // 再トグルで Idle へ (明示アーム)
  in = uprightInput(now += 0.005f);
  in.stop_toggle = true;
  r = fsm.update(in);
  TEST_ASSERT_EQUAL(static_cast<int>(FsmState::Idle), static_cast<int>(r.state));
}

static void test_fsm_save_blocks_autoarm() {
  SafetyFsm fsm;
  fsm.setParams(fsmParams());
  fsm.notifyInitDone();
  float now = 0.0f;
  // 起立窓成立済みでも save_in_progress 中はアームしない (§9.1)
  for (int i = 0; i < 600; ++i) {
    now += 0.005f;
    SafetyFsm::Input in = uprightInput(now);
    in.save_in_progress = true;
    const SafetyFsm::Result r = fsm.update(in);
    TEST_ASSERT_EQUAL(static_cast<int>(FsmAction::None), static_cast<int>(r.action));
    TEST_ASSERT_EQUAL(static_cast<int>(FsmState::Idle), static_cast<int>(r.state));
  }
  // 保存完了後、改めて 1.0s の保持を要求される
  bool requested_early = false;
  for (int i = 0; i < 100; ++i) {  // 0.5 s
    now += 0.005f;
    if (fsm.update(uprightInput(now)).action == FsmAction::EnterBalancing) {
      requested_early = true;
    }
  }
  TEST_ASSERT_FALSE(requested_early);
  TEST_ASSERT_TRUE(driveUntilArm(fsm, &now, 0.7f));
}

static void test_fsm_fault_latch() {
  SafetyFsm fsm;
  fsm.setParams(fsmParams());
  fsm.notifyInitDone();
  float now = 0.0f;
  SafetyFsm::Input in = uprightInput(now += 0.005f);
  in.fault = FaultReason::DxlReadStale;
  SafetyFsm::Result r = fsm.update(in);
  TEST_ASSERT_EQUAL(static_cast<int>(FsmState::Fault), static_cast<int>(r.state));
  TEST_ASSERT_EQUAL(static_cast<int>(FsmAction::SafeStop), static_cast<int>(r.action));
  // ラッチ: 直立でも STOP トグルでも復帰しない
  for (int i = 0; i < 400; ++i) {
    now += 0.005f;
    SafetyFsm::Input in2 = uprightInput(now);
    in2.stop_toggle = (i % 100 == 0);
    TEST_ASSERT_EQUAL(static_cast<int>(FsmState::Fault),
                      static_cast<int>(fsm.update(in2).state));
  }
}

static void test_fsm_entry_failed() {
  SafetyFsm fsm;
  fsm.setParams(fsmParams());
  fsm.notifyInitDone();
  float now = 0.0f;
  driveUntilArm(fsm, &now, 1.2f);
  fsm.notifyEntryFailed();  // enter_balancing() の検証失敗
  TEST_ASSERT_EQUAL(static_cast<int>(FsmState::Fault), static_cast<int>(fsm.state()));
  TEST_ASSERT_EQUAL(static_cast<int>(FaultReason::EntryVerifyFailed),
                    static_cast<int>(fsm.faultReason()));
}

// ---------------- param_validation (§9.1) ----------------

static void test_params_defaults_valid() {
  TuningParams rec;  // 既定値
  TEST_ASSERT_TRUE(validateParams(rec));
}

static void test_params_reject_invalid() {
  TuningParams rec;
  rec.schema_version = 999;  // 版不一致
  TEST_ASSERT_FALSE(validateParams(rec));

  rec = TuningParams{};
  rec.kp = 100.0f;  // 範囲外
  TEST_ASSERT_FALSE(validateParams(rec));

  rec = TuningParams{};
  rec.pitch_eq = NAN;  // NaN → 破棄
  TEST_ASSERT_FALSE(validateParams(rec));
}

static void test_commissioning_fail_closed() {
  const uint32_t csum = currentSignAxisChecksum();
  CommissioningRecord rec;
  rec.schema_version = cfg::kCommissionSchemaVersion;
  rec.profile = static_cast<uint8_t>(cfg::Profile::Normal);
  rec.sign_axis_checksum = csum;
  rec.calib_version = 3;
  rec.user_confirmed = true;
  TEST_ASSERT_TRUE(validateCommissioning(rec, csum, 3));

  // 車輪符号が変わった (チェックサム不一致) → 未コミッショニング扱い
  const uint32_t csum_wheel = signAxisChecksum(
      -cfg::kSignLeft, cfg::kSignRight, cfg::kImuAccTiltSign,
      cfg::kImuAccVertSign, cfg::kImuGyroSign);
  TEST_ASSERT_FALSE(validateCommissioning(rec, csum_wheel, 3));
  // IMU 軸符号が変わっても無効化される (gate2 P2 回帰)
  const uint32_t csum_imu = signAxisChecksum(
      cfg::kSignLeft, cfg::kSignRight, -cfg::kImuAccTiltSign,
      cfg::kImuAccVertSign, cfg::kImuGyroSign);
  TEST_ASSERT_FALSE(validateCommissioning(rec, csum_imu, 3));
  // 校正版が進んだ
  TEST_ASSERT_FALSE(validateCommissioning(rec, csum, 4));
  // 未確認フラグ
  rec.user_confirmed = false;
  TEST_ASSERT_FALSE(validateCommissioning(rec, csum, 3));
  // 旧スキーマ
  rec.user_confirmed = true;
  rec.schema_version = 0;
  TEST_ASSERT_FALSE(validateCommissioning(rec, csum, 3));
  // Bringup プロファイルの記録では auto-arm 不可
  rec.schema_version = cfg::kCommissionSchemaVersion;
  rec.profile = static_cast<uint8_t>(cfg::Profile::Bringup);
  TEST_ASSERT_FALSE(validateCommissioning(rec, csum, 3));
}

// ---------------- dxl_verify (§4.2 配達証明。段階1a抽出) ----------------

static DxlStatusView writeOkView(uint8_t id) {
  DxlStatusView st;
  st.ok = true;
  st.id = id;
  st.err_idx = 0;
  st.recv_param_len = 0;
  return st;
}

static void test_dxl_verify_write_accept_and_reject() {
  // 受理: ID 一致・err=0・param_len=0
  TEST_ASSERT_TRUE(isWriteStatusVerified(writeOkView(3), 3));

  // 拒否1: st.ok=false (= rx nullptr の view。他フィールドは既定値のまま)
  DxlStatusView st_no_rx;
  TEST_ASSERT_FALSE(isWriteStatusVerified(st_no_rx, 3));

  // 拒否2: ID 不一致
  {
    DxlStatusView st = writeOkView(3);
    st.id = 4;
    TEST_ASSERT_FALSE(isWriteStatusVerified(st, 3));
  }
  // 拒否3: err_idx=0x80 (ALERT も含め非零は拒否)
  {
    DxlStatusView st = writeOkView(3);
    st.err_idx = 0x80;
    TEST_ASSERT_FALSE(isWriteStatusVerified(st, 3));
  }
  // 拒否4: err_idx=0x01 (Result Fail 等)
  {
    DxlStatusView st = writeOkView(3);
    st.err_idx = 0x01;
    TEST_ASSERT_FALSE(isWriteStatusVerified(st, 3));
  }
  // 拒否5: recv_param_len != 0 (前回 READ の遅延応答の誤受理拒否)
  {
    DxlStatusView st = writeOkView(3);
    st.recv_param_len = 2;
    TEST_ASSERT_FALSE(isWriteStatusVerified(st, 3));
  }

  // 述語の限界の明示 (§8 残余リスク): 同一 ID・err=0・param_len=0 の遅延
  // WRITE Status は述語単体では判別不能で受理される (現行実装と同一の挙動。
  // drainRx は tx 前の残留バイトのみ除去し、tx 後に到着する遅延応答は防げ
  // ない)。段階5 の transport タイムラインテストでクローズするまでの残余
  // リスクとしてここに明示する — writeOkView(3) は「本来の Status」と
  // 「同一 ID の遅延応答」を型として区別できないため、両者とも受理される。
  TEST_ASSERT_TRUE(isWriteStatusVerified(writeOkView(3), 3));
}

static DxlStatusView readOkView(uint8_t id, uint16_t len, uint8_t err_idx = 0) {
  DxlStatusView st;
  st.ok = true;
  st.id = id;
  st.err_idx = err_idx;
  st.recv_param_len = len;
  return st;
}

static void test_dxl_verify_read_accept_and_reject() {
  // 受理: err=0
  TEST_ASSERT_TRUE(isReadStatusVerified(readOkView(5, 2), 5, 2));
  // 受理: ALERT のみ (0x80) は現行仕様どおりデータ有効
  TEST_ASSERT_TRUE(isReadStatusVerified(readOkView(5, 2, 0x80), 5, 2));

  // 拒否1: rx 失敗 (st.ok=false)
  {
    DxlStatusView st;
    TEST_ASSERT_FALSE(isReadStatusVerified(st, 5, 2));
  }
  // 拒否2: ID 不一致
  {
    DxlStatusView st = readOkView(5, 2);
    st.id = 6;
    TEST_ASSERT_FALSE(isReadStatusVerified(st, 5, 2));
  }
  // 拒否3: 長さ不一致
  TEST_ASSERT_FALSE(isReadStatusVerified(readOkView(5, 1), 5, 2));
  // 拒否4: Result Fail (err & 0x7F != 0)。ALERT と重畳した 0x81 も拒否
  TEST_ASSERT_FALSE(isReadStatusVerified(readOkView(5, 2, 0x01), 5, 2));
  TEST_ASSERT_FALSE(isReadStatusVerified(readOkView(5, 2, 0x81), 5, 2));
}

// ---------------- watchdog_policy (§4.3 遷移表。段階1b抽出) ----------------

static constexpr uint8_t kTestTripped = 0xFF;

static void test_watchdog_raw_read_failure_confirms_immediately() {
  // rawL 読取失敗: 1 回目の feedRaw で即確定 (rawR を feed しない)
  {
    WatchdogDecision d(/*torque_may_be_on=*/true, kTestTripped);
    TEST_ASSERT_EQUAL(static_cast<int>(WatchdogVerdict::Fault),
                       static_cast<int>(d.feedRaw(false, 0)));
  }
  {
    WatchdogDecision d(/*torque_may_be_on=*/false, kTestTripped);
    TEST_ASSERT_EQUAL(static_cast<int>(WatchdogVerdict::Ok),
                       static_cast<int>(d.feedRaw(false, 0)));
  }
  // rawR 読取失敗: 2 回目で即確定 (1 回目は非トリップの正常読取)
  {
    WatchdogDecision d(/*torque_may_be_on=*/true, kTestTripped);
    TEST_ASSERT_EQUAL(static_cast<int>(WatchdogVerdict::Pending),
                       static_cast<int>(d.feedRaw(true, 0x00)));
    TEST_ASSERT_EQUAL(static_cast<int>(WatchdogVerdict::Fault),
                       static_cast<int>(d.feedRaw(false, 0)));
  }
  {
    WatchdogDecision d(/*torque_may_be_on=*/false, kTestTripped);
    TEST_ASSERT_EQUAL(static_cast<int>(WatchdogVerdict::Pending),
                       static_cast<int>(d.feedRaw(true, 0x00)));
    TEST_ASSERT_EQUAL(static_cast<int>(WatchdogVerdict::Ok),
                       static_cast<int>(d.feedRaw(false, 0)));
  }
}

static void test_watchdog_raw_trip_combinations() {
  // 両輪非トリップ → 2 回目の feedRaw で Ok
  {
    WatchdogDecision d(true, kTestTripped);
    TEST_ASSERT_EQUAL(static_cast<int>(WatchdogVerdict::Pending),
                       static_cast<int>(d.feedRaw(true, 0x00)));
    TEST_ASSERT_EQUAL(static_cast<int>(WatchdogVerdict::Ok),
                       static_cast<int>(d.feedRaw(true, 0x00)));
  }
  // トリップ L のみ → 2 回目の feedRaw で Pending (TE 段階へ)
  {
    WatchdogDecision d(true, kTestTripped);
    TEST_ASSERT_EQUAL(static_cast<int>(WatchdogVerdict::Pending),
                       static_cast<int>(d.feedRaw(true, kTestTripped)));
    TEST_ASSERT_EQUAL(static_cast<int>(WatchdogVerdict::Pending),
                       static_cast<int>(d.feedRaw(true, 0x00)));
  }
  // トリップ R のみ
  {
    WatchdogDecision d(true, kTestTripped);
    TEST_ASSERT_EQUAL(static_cast<int>(WatchdogVerdict::Pending),
                       static_cast<int>(d.feedRaw(true, 0x00)));
    TEST_ASSERT_EQUAL(static_cast<int>(WatchdogVerdict::Pending),
                       static_cast<int>(d.feedRaw(true, kTestTripped)));
  }
  // 両輪トリップ
  {
    WatchdogDecision d(true, kTestTripped);
    TEST_ASSERT_EQUAL(static_cast<int>(WatchdogVerdict::Pending),
                       static_cast<int>(d.feedRaw(true, kTestTripped)));
    TEST_ASSERT_EQUAL(static_cast<int>(WatchdogVerdict::Pending),
                       static_cast<int>(d.feedRaw(true, kTestTripped)));
  }
}

static void test_watchdog_te_read_failure_confirms_immediately() {
  // teL 読取失敗: 1 回目の feedTe で即 Fault (teR を feed しない)
  {
    WatchdogDecision d(true, kTestTripped);
    d.feedRaw(true, kTestTripped);
    d.feedRaw(true, 0x00);  // トリップ確定 → Pending (TE 段階へ)
    TEST_ASSERT_EQUAL(static_cast<int>(WatchdogVerdict::Fault),
                       static_cast<int>(d.feedTe(false, 0)));
  }
  // teR 読取失敗: 2 回目で Fault (1 回目は正常読取)
  {
    WatchdogDecision d(true, kTestTripped);
    d.feedRaw(true, kTestTripped);
    d.feedRaw(true, 0x00);
    TEST_ASSERT_EQUAL(static_cast<int>(WatchdogVerdict::Pending),
                       static_cast<int>(d.feedTe(true, 0)));
    TEST_ASSERT_EQUAL(static_cast<int>(WatchdogVerdict::Fault),
                       static_cast<int>(d.feedTe(false, 0)));
  }
}

static void test_watchdog_te_value_combinations() {
  // (teL,teR) 全組合せ (ゲート1第3回指摘対応: 現行 te[0]!=0||te[1]!=0 の OR
  // 意味論が && へ写し間違えられても検出できるよう全組合せを固定)。
  // 2 回目の feedTe 完了時に確定 (1 回目は teL 非零でも Pending = teL 非零
  // でも teR 読取まで行う現行 I/O 順序契約を同じテストで固定)。
  struct Case { uint8_t te_l, te_r; WatchdogVerdict expect; };
  const Case cases[] = {
      {1, 0, WatchdogVerdict::Fault},
      {0, 1, WatchdogVerdict::Fault},
      {1, 1, WatchdogVerdict::Fault},
      {0, 0, WatchdogVerdict::ProceedRecover},
  };
  for (const auto& c : cases) {
    WatchdogDecision d(true, kTestTripped);
    d.feedRaw(true, kTestTripped);
    d.feedRaw(true, 0x00);  // トリップ確定 → Pending
    const WatchdogVerdict v1 = d.feedTe(true, c.te_l);
    TEST_ASSERT_EQUAL(static_cast<int>(WatchdogVerdict::Pending),
                       static_cast<int>(v1));  // 1 回目は常に Pending
    const WatchdogVerdict v2 = d.feedTe(true, c.te_r);
    TEST_ASSERT_EQUAL(static_cast<int>(c.expect), static_cast<int>(v2));
  }
}

static void test_watchdog_characterization_trip_then_raw_read_failure() {
  // 現行挙動の特性化 (§3.2 潜在エッジ・§8 残余リスク): rawL=0xFF (トリップ)
  // を観測した直後に rawR の読取が失敗すると、先行トリップ証拠は結果に
  // 影響せず torque_may_be_on の値のみで確定する (false→Ok / true→Fault)。
  // これは現行 dxl_backend.cpp:374-378 と同一の残余リスクであり、
  // fail-closed 化 (トリップ証拠観測後の読取失敗を Fault 化) は別課題として
  // ユーザへエスカレーション済み (安全挙動変更のためユーザ判断が必要)。
  {
    WatchdogDecision d(/*torque_may_be_on=*/false, kTestTripped);
    TEST_ASSERT_EQUAL(static_cast<int>(WatchdogVerdict::Pending),
                       static_cast<int>(d.feedRaw(true, kTestTripped)));
    TEST_ASSERT_EQUAL(static_cast<int>(WatchdogVerdict::Ok),
                       static_cast<int>(d.feedRaw(false, 0)));
  }
  {
    WatchdogDecision d(/*torque_may_be_on=*/true, kTestTripped);
    TEST_ASSERT_EQUAL(static_cast<int>(WatchdogVerdict::Pending),
                       static_cast<int>(d.feedRaw(true, kTestTripped)));
    TEST_ASSERT_EQUAL(static_cast<int>(WatchdogVerdict::Fault),
                       static_cast<int>(d.feedRaw(false, 0)));
  }
}

static void test_watchdog_recover_limiter_window_and_deny() {
  const float window_s = 60.0f;
  const int max_count = 3;

  // 窓内 2 回目まで allow・3 回目 deny・deny 後も窓内は deny 継続
  {
    WatchdogRecoverLimiter lim;
    TEST_ASSERT_TRUE(lim.allow(0.0f, window_s, max_count));
    TEST_ASSERT_TRUE(lim.allow(10.0f, window_s, max_count));
    TEST_ASSERT_FALSE(lim.allow(20.0f, window_s, max_count));
    TEST_ASSERT_FALSE(lim.allow(30.0f, window_s, max_count));  // deny 継続
  }

  // 境界: ちょうど window_s は同一窓 (> 比較なので == はリセットしない)
  {
    WatchdogRecoverLimiter lim;
    TEST_ASSERT_TRUE(lim.allow(0.0f, window_s, max_count));       // count=1
    TEST_ASSERT_TRUE(lim.allow(window_s, window_s, max_count));   // count=2 (同一窓)
    TEST_ASSERT_FALSE(lim.allow(window_s, window_s, max_count));  // count=3 → deny
  }

  // 境界: window_s+ε で新窓 (リセット)
  {
    WatchdogRecoverLimiter lim;
    TEST_ASSERT_TRUE(lim.allow(0.0f, window_s, max_count));               // count=1
    TEST_ASSERT_TRUE(lim.allow(window_s * 0.5f, window_s, max_count));    // count=2
    TEST_ASSERT_FALSE(lim.allow(window_s * 0.9f, window_s, max_count));   // count=3 → deny (同一窓)
    TEST_ASSERT_TRUE(lim.allow(window_s + 0.001f, window_s, max_count));  // 新窓 → count=1 → allow
  }
}

// ---------------- plausibility_monitor (§6。段階2逐語移動) ----------------

static void test_plausibility_fb_invalid_no_judge() {
  PlausibilityMonitor pm;
  // fb_valid=false: 判定しない・dwell 非蓄積
  for (int i = 0; i < 100; ++i) {
    TEST_ASSERT_FALSE(pm.update(/*torque_on=*/true, /*fb_valid=*/false, 0.0f, 1.0f, 0.005f));
  }
}

static void test_plausibility_torque_on_dwell_threshold() {
  PlausibilityMonitor pm;
  const float dt = cfg::kCurrentPlausDwellS;  // 1 周期でちょうど閾値相当の dt
  // err = |1.0-0.5| = 0.5 > kCurrentMismatchA(0.3) なので bad
  // 1 回目: dwell = 0+dt == 閾値 (ちょうど、ビット同一) → > 比較で false (境界)
  TEST_ASSERT_FALSE(pm.update(true, true, 0.5f, 1.0f, dt));
  // 2 回目: dwell = 2*閾値 > 閾値 → true
  TEST_ASSERT_TRUE(pm.update(true, true, 0.5f, 1.0f, dt));
}

static void test_plausibility_recovery_resets_dwell() {
  PlausibilityMonitor pm;
  const float dt = cfg::kCurrentPlausDwellS;
  TEST_ASSERT_FALSE(pm.update(true, true, 0.5f, 1.0f, dt));  // dwell=閾値 (bad)
  // 正常に戻る (err=0) → dwell リセット
  TEST_ASSERT_FALSE(pm.update(true, true, 1.0f, 1.0f, dt));
  // 良好状態直後にもう一度悪化させても、まだ 1 周期分の dwell しか無い
  TEST_ASSERT_FALSE(pm.update(true, true, 0.5f, 1.0f, dt));
}

static void test_plausibility_torque_off_residual_and_reset() {
  PlausibilityMonitor pm;
  const float dt = cfg::kCurrentPlausDwellS;
  // torque_off: |i_pres| > kCurrentResidualA(0.1) で bad
  TEST_ASSERT_FALSE(pm.update(false, true, 0.0f, 0.2f, dt));  // dwell=閾値ちょうど
  TEST_ASSERT_TRUE(pm.update(false, true, 0.0f, 0.2f, dt));   // 超過
  pm.reset();
  // reset 後は dwell 0 から再スタート。残留電流が閾値内なら bad にならない
  TEST_ASSERT_FALSE(pm.update(false, true, 0.0f, 0.05f, dt));
}

// ---------------- dt_stats (§6 soak gate 計算コア。段階3新設) ----------------

static void test_dtstats_p95_n0_fails_closed() {
  DtP95Window w;
  float v = 12.34f;  // 番兵値
  TEST_ASSERT_FALSE(w.p95(&v));
  TEST_ASSERT_FLOAT_WITHIN(1e-9f, 12.34f, v);  // *out 不変
}

static void test_dtstats_n1() {
  DtP95Window w;
  w.add(0.005f);
  float v = -1.0f;
  TEST_ASSERT_TRUE(w.p95(&v));
  TEST_ASSERT_FLOAT_WITHIN(1e-9f, 0.005f, v);
}

static void test_dtstats_n20_known_distribution() {
  DtP95Window w;
  // 値 1..20 を投入。nearest-rank: idx = ceil(0.95*20)-1 = 18 (0-indexed)
  // = 昇順 19 番目の値 = 19
  for (int i = 1; i <= 20; ++i) w.add(static_cast<float>(i));
  float v = 0.0f;
  TEST_ASSERT_TRUE(w.p95(&v));
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 19.0f, v);
}

static void test_dtstats_unsorted_and_duplicate_input() {
  DtP95Window w;
  // {1..20} の多重集合だが 17 を欠落させ 19 を重複させた (合計 20 個)。
  // ソート後: 1,2,...,16,18,19,19,20 → idx=18 (0-indexed) = 19
  const float vals[] = {5, 1, 20, 3, 19, 19, 2, 18, 4, 6,
                        7, 8, 9, 10, 11, 12, 13, 14, 15, 16};
  for (float x : vals) w.add(x);
  float v = 0.0f;
  TEST_ASSERT_TRUE(w.p95(&v));
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 19.0f, v);
}

static void test_dtstats_p95_preserves_input_order() {
  DtP95Window w;
  w.add(3.0f);
  w.add(1.0f);
  w.add(2.0f);
  float v1 = 0.0f, v2 = 0.0f;
  TEST_ASSERT_TRUE(w.p95(&v1));
  // 同一呼出しを繰り返しても同じ値 (内部作業配列で選択・buf_ は非破壊)
  TEST_ASSERT_TRUE(w.p95(&v2));
  TEST_ASSERT_FLOAT_WITHIN(1e-9f, v1, v2);
  // 呼出し後も add を継続でき、新規サンプルを含めた計算が正しく行われる
  w.add(0.5f);
  TEST_ASSERT_EQUAL(4, static_cast<int>(w.size()));
  float v3 = 0.0f;
  TEST_ASSERT_TRUE(w.p95(&v3));
  // n=4: idx = ceil(0.95*4)-1 = ceil(3.8)-1 = 4-1 = 3 → 最大値 (3.0)
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 3.0f, v3);
}

static void test_dtstats_n400_soak_window() {
  DtP95Window w;
  for (int i = 0; i < 400; ++i) w.add(0.005f);  // 200Hz×2s の均一 dt
  TEST_ASSERT_EQUAL(400, static_cast<int>(w.size()));
  TEST_ASSERT_FALSE(w.overflowed());
  TEST_ASSERT_TRUE(w.ready(400));
  float v = 0.0f;
  TEST_ASSERT_TRUE(w.p95(&v));
  TEST_ASSERT_FLOAT_WITHIN(1e-9f, 0.005f, v);
}

static void test_dtstats_ready_boundary() {
  DtP95Window w399;
  for (int i = 0; i < 399; ++i) w399.add(0.001f);
  TEST_ASSERT_FALSE(w399.ready(400));

  DtP95Window w400;
  for (int i = 0; i < 400; ++i) w400.add(0.001f);
  TEST_ASSERT_TRUE(w400.ready(400));
}

static void test_dtstats_capacity_overflow_keeps_existing_samples() {
  DtP95Window w;
  for (size_t i = 0; i < DtP95Window::kCapacity; ++i) w.add(0.005f);  // ちょうど満杯
  TEST_ASSERT_EQUAL(static_cast<int>(DtP95Window::kCapacity), static_cast<int>(w.size()));
  TEST_ASSERT_FALSE(w.overflowed());

  // 満杯後の追加は記録されず overflowed が立つ (既存サンプルは変化しない)
  w.add(0.999f);
  TEST_ASSERT_EQUAL(static_cast<int>(DtP95Window::kCapacity), static_cast<int>(w.size()));
  TEST_ASSERT_TRUE(w.overflowed());
  float v = 0.0f;
  TEST_ASSERT_TRUE(w.p95(&v));
  TEST_ASSERT_FLOAT_WITHIN(1e-9f, 0.005f, v);  // 0.999 は含まれない
  TEST_ASSERT_FALSE(w.ready(1));  // overflow → ready は常に false
}

static void test_dtstats_reset() {
  DtP95Window w;
  w.add(1.0f);
  w.add(2.0f);
  w.reset();
  TEST_ASSERT_EQUAL(0, static_cast<int>(w.size()));
  TEST_ASSERT_FALSE(w.overflowed());
  float v = 0.0f;
  TEST_ASSERT_FALSE(w.p95(&v));  // n==0 → false・*out 不変
  TEST_ASSERT_FLOAT_WITHIN(1e-9f, 0.0f, v);
  w.add(3.0f);
  TEST_ASSERT_EQUAL(1, static_cast<int>(w.size()));
}

static void test_dtstats_fail_closed_gate_table() {
  // 「ready && p95 && v<=budget」合成ゲートが空窓・不足窓・溢れ窓で合格し
  // 得ないことのテーブルテスト。
  const float budget = 0.010f;

  // 空窓
  {
    DtP95Window w;
    float v = 0.0f;
    const bool gate = w.ready(400) && w.p95(&v) && v <= budget;
    TEST_ASSERT_FALSE(gate);
  }
  // 不足窓 (n=399 < min_samples=400)
  {
    DtP95Window w;
    for (int i = 0; i < 399; ++i) w.add(0.001f);
    float v = 0.0f;
    const bool gate = w.ready(400) && w.p95(&v) && v <= budget;
    TEST_ASSERT_FALSE(gate);
  }
  // ちょうど 400 (予算内) → 合格
  {
    DtP95Window w;
    for (int i = 0; i < 400; ++i) w.add(0.001f);
    float v = 0.0f;
    const bool gate = w.ready(400) && w.p95(&v) && v <= budget;
    TEST_ASSERT_TRUE(gate);
  }
  // 溢れ窓 (overflow) → ready は常に false
  {
    DtP95Window w;
    for (size_t i = 0; i < DtP95Window::kCapacity; ++i) w.add(0.001f);
    w.add(0.001f);  // 溢れ
    float v = 0.0f;
    const bool gate = w.ready(1) && w.p95(&v) && v <= budget;
    TEST_ASSERT_FALSE(gate);
  }
}

// ---------------- runner ----------------

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_units_current);
  RUN_TEST(test_units_velocity);
  RUN_TEST(test_units_position);
  RUN_TEST(test_units_twos_complement);
  RUN_TEST(test_pid_p_and_d);
  RUN_TEST(test_pid_anti_windup);
  RUN_TEST(test_pid_d_lpf);
  RUN_TEST(test_estimator_convergence);
  RUN_TEST(test_estimator_accel_gate);
  RUN_TEST(test_estimator_stale_hold);
  RUN_TEST(test_estimator_bias);
  RUN_TEST(test_restoring_direction);
  RUN_TEST(test_mixer_inversion_priority);
  RUN_TEST(test_mixer_yaw_headroom);
  RUN_TEST(test_slew_override_after_coast);
  RUN_TEST(test_speed_guard);
  RUN_TEST(test_i2t_symmetric_convergence);
  RUN_TEST(test_balance_stale_freezes_outer_loop);
  RUN_TEST(test_fsm_boot_gate);
  RUN_TEST(test_fsm_arm_sequence);
  RUN_TEST(test_fsm_zero_centered_theta_contract);
  RUN_TEST(test_fsm_no_stale_upright_timer_after_fall);
  RUN_TEST(test_fsm_fall_and_recover);
  RUN_TEST(test_fsm_fall_escalation);
  RUN_TEST(test_fsm_fall_escalation_sliding_window);
  RUN_TEST(test_fsm_stop_toggle);
  RUN_TEST(test_fsm_save_blocks_autoarm);
  RUN_TEST(test_fsm_fault_latch);
  RUN_TEST(test_params_reject_invalid);
  RUN_TEST(test_params_defaults_valid);
  RUN_TEST(test_commissioning_fail_closed);
  RUN_TEST(test_fsm_entry_failed);
  RUN_TEST(test_dxl_verify_write_accept_and_reject);
  RUN_TEST(test_dxl_verify_read_accept_and_reject);
  RUN_TEST(test_watchdog_raw_read_failure_confirms_immediately);
  RUN_TEST(test_watchdog_raw_trip_combinations);
  RUN_TEST(test_watchdog_te_read_failure_confirms_immediately);
  RUN_TEST(test_watchdog_te_value_combinations);
  RUN_TEST(test_watchdog_characterization_trip_then_raw_read_failure);
  RUN_TEST(test_watchdog_recover_limiter_window_and_deny);
  RUN_TEST(test_plausibility_fb_invalid_no_judge);
  RUN_TEST(test_plausibility_torque_on_dwell_threshold);
  RUN_TEST(test_plausibility_recovery_resets_dwell);
  RUN_TEST(test_plausibility_torque_off_residual_and_reset);
  RUN_TEST(test_dtstats_p95_n0_fails_closed);
  RUN_TEST(test_dtstats_n1);
  RUN_TEST(test_dtstats_n20_known_distribution);
  RUN_TEST(test_dtstats_unsorted_and_duplicate_input);
  RUN_TEST(test_dtstats_p95_preserves_input_order);
  RUN_TEST(test_dtstats_n400_soak_window);
  RUN_TEST(test_dtstats_ready_boundary);
  RUN_TEST(test_dtstats_capacity_overflow_keeps_existing_samples);
  RUN_TEST(test_dtstats_reset);
  RUN_TEST(test_dtstats_fail_closed_gate_table);
  return UNITY_END();
}
