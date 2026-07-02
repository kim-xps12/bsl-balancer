// test_main.cpp — 純ロジック層のユニットテスト (設計書 §8 受入テスト)
// 実行: ~/.platformio/penv/bin/pio test -e native
#include <unity.h>

#include <cmath>

#include "../../src/core/attitude_estimator.h"
#include "../../src/core/balance_core.h"
#include "../../src/core/param_validation.h"
#include "../../src/core/pid.h"
#include "../../src/core/safety_fsm.h"
#include "../../src/core/units.h"

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
  RUN_TEST(test_fsm_stop_toggle);
  RUN_TEST(test_fsm_save_blocks_autoarm);
  RUN_TEST(test_fsm_fault_latch);
  RUN_TEST(test_params_reject_invalid);
  RUN_TEST(test_params_defaults_valid);
  RUN_TEST(test_commissioning_fail_closed);
  RUN_TEST(test_fsm_entry_failed);
  return UNITY_END();
}
