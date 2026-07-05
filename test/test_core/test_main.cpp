// test_main.cpp — 純ロジック層のユニットテスト (設計書 §8 受入テスト)
// 実行: ~/.platformio/penv/bin/pio test -e native
// 注: Unity は math ヘッダ未取込時に isnan/isinf をマクロ定義し libc++ の
// <cmath> を壊すため、C++ ヘッダを unity.h より先に include する。
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstring>
#include <thread>

#include "../../src/core/attitude_estimator.h"
#include "../../src/core/balance_core.h"
#include "../../src/core/dt_histogram.h"
#include "../../src/core/param_validation.h"
#include "../../src/core/pid.h"
#include "../../src/core/safety_fsm.h"
#include "../../src/core/telemetry_format.h"
#include "../../src/core/units.h"
#include "../../src/core/wifi_guard.h"
#include "../../src/shared/shared_state.h"

#include <unity.h>

using namespace core;

void setUp() {}
void tearDown() {}

// ---------------- UDP telemetry Phase1: WifiGuard 用フェイク Ops ----------------
// (計画書 §3.1 「インターフェース注入で Arduino API を抽象化」に対応するテストダブル)

namespace {

struct FakeWifiState {
  int begin_calls = 0;
  int disconnect_calls = 0;
  bool disconnect_return = true;
  int set_radio_off_calls = 0;
  bool radio_off_status = false;
  int begin_packet_calls = 0;
  int begin_packet_return = 1;  // 1 = 成功
  int write_calls = 0;
  int write_return = -1;  // -1 の場合は呼び出し時の len をそのまま返す (成功扱い)
  int end_packet_calls = 0;
  int end_packet_return = 1;  // 1 = 成功
};

void FakeBegin(void* ctx) { ++static_cast<FakeWifiState*>(ctx)->begin_calls; }
bool FakeDisconnect(void* ctx) {
  auto* s = static_cast<FakeWifiState*>(ctx);
  ++s->disconnect_calls;
  return s->disconnect_return;
}
void FakeSetRadioOff(void* ctx) { ++static_cast<FakeWifiState*>(ctx)->set_radio_off_calls; }
bool FakeIsRadioOff(void* ctx) { return static_cast<FakeWifiState*>(ctx)->radio_off_status; }
int FakeBeginPacket(void* ctx) {
  auto* s = static_cast<FakeWifiState*>(ctx);
  ++s->begin_packet_calls;
  return s->begin_packet_return;
}
int FakeWritePacket(void* ctx, const uint8_t*, size_t len) {
  auto* s = static_cast<FakeWifiState*>(ctx);
  ++s->write_calls;
  return s->write_return < 0 ? static_cast<int>(len) : s->write_return;
}
int FakeEndPacket(void* ctx) {
  auto* s = static_cast<FakeWifiState*>(ctx);
  ++s->end_packet_calls;
  return s->end_packet_return;
}

WifiOps makeFakeOps(FakeWifiState* s) {
  WifiOps o;
  o.ctx = s;
  o.begin = &FakeBegin;
  o.disconnect = &FakeDisconnect;
  o.setRadioOff = &FakeSetRadioOff;
  o.isRadioOff = &FakeIsRadioOff;
  o.beginPacket = &FakeBeginPacket;
  o.writePacket = &FakeWritePacket;
  o.endPacket = &FakeEndPacket;
  return o;
}

}  // namespace

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

// ---------------- SafetyFsm::armPending() (UDP telemetry Phase1 計画書 §3.1) ----------------

static void test_fsm_arm_pending_auto_arm() {
  SafetyFsm fsm;
  fsm.setParams(fsmParams(true, true));  // commissioned + auto_arm → Idle
  fsm.notifyInitDone();
  TEST_ASSERT_EQUAL(static_cast<int>(FsmState::Idle), static_cast<int>(fsm.state()));
  TEST_ASSERT_FALSE(fsm.armPending());  // 起立確認前はまだ保留していない

  float now = 0.0f;
  bool became_pending = false;
  bool requested = false;
  for (int i = 0; i < 260; ++i) {
    now += 0.005f;
    const SafetyFsm::Result r = fsm.update(uprightInput(now));
    if (fsm.armPending()) became_pending = true;
    if (r.action == FsmAction::EnterBalancing) { requested = true; break; }
  }
  TEST_ASSERT_TRUE(became_pending);
  TEST_ASSERT_TRUE(requested);
  TEST_ASSERT_TRUE(fsm.armPending());  // EnterBalancing 発行直後もまだ Idle のまま

  fsm.notifyBalancingEntered();
  TEST_ASSERT_FALSE(fsm.armPending());  // Balancing 遷移後は解除
}

static void test_fsm_arm_pending_manual_arm() {
  SafetyFsm fsm;
  fsm.setParams(fsmParams(false, true));  // 未コミッショニング → Disarmed 起動
  fsm.notifyInitDone();
  TEST_ASSERT_EQUAL(static_cast<int>(FsmState::Disarmed), static_cast<int>(fsm.state()));
  TEST_ASSERT_FALSE(fsm.armPending());

  float now = 0.0f;
  SafetyFsm::Input in = uprightInput(now += 0.005f);
  in.stop_toggle = true;  // BtnC 手動アーム → Idle
  fsm.update(in);
  TEST_ASSERT_EQUAL(static_cast<int>(FsmState::Idle), static_cast<int>(fsm.state()));

  bool became_pending = false;
  bool requested = false;
  for (int i = 0; i < 260; ++i) {
    now += 0.005f;
    const SafetyFsm::Result r = fsm.update(uprightInput(now));
    if (fsm.armPending()) became_pending = true;
    if (r.action == FsmAction::EnterBalancing) { requested = true; break; }
  }
  TEST_ASSERT_TRUE(became_pending);
  TEST_ASSERT_TRUE(requested);
  fsm.notifyBalancingEntered();
  TEST_ASSERT_FALSE(fsm.armPending());
}

// ---------------- dt_histogram (計画書 §3.1) ----------------

static void test_dt_histogram_boundaries() {
  const float period = 0.005f;
  TEST_ASSERT_EQUAL(0, classifyDtBin(period * 1.000f, period));
  TEST_ASSERT_EQUAL(0, classifyDtBin(period * 1.019f, period));
  TEST_ASSERT_EQUAL(1, classifyDtBin(period * 1.020f, period));
  TEST_ASSERT_EQUAL(1, classifyDtBin(period * 1.049f, period));
  TEST_ASSERT_EQUAL(2, classifyDtBin(period * 1.050f, period));
  TEST_ASSERT_EQUAL(2, classifyDtBin(period * 1.099f, period));
  TEST_ASSERT_EQUAL(3, classifyDtBin(period * 1.100f, period));
  TEST_ASSERT_EQUAL(3, classifyDtBin(period * 1.199f, period));
  TEST_ASSERT_EQUAL(4, classifyDtBin(period * 1.200f, period));
  TEST_ASSERT_EQUAL(4, classifyDtBin(period * 1.299f, period));
  TEST_ASSERT_EQUAL(5, classifyDtBin(period * 1.300f, period));
  TEST_ASSERT_EQUAL(5, classifyDtBin(period * 1.499f, period));
  TEST_ASSERT_EQUAL(6, classifyDtBin(period * 1.500f, period));
  TEST_ASSERT_EQUAL(6, classifyDtBin(period * 1.999f, period));
  TEST_ASSERT_EQUAL(7, classifyDtBin(period * 2.000f, period));
  TEST_ASSERT_EQUAL(7, classifyDtBin(period * 5.000f, period));

  TEST_ASSERT_FALSE(isOverrunDt(period * 1.5f, period));
  TEST_ASSERT_TRUE(isOverrunDt(period * 1.5f + 1e-6f, period));
}

static void test_dt_monotonic_counter_survives_consecutive_reset() {
  // 既存 LoopState.overrun_count (連続回数、正常サイクルでリセット) を模した挙動と
  // 新設 overrun_total (monotonic、リセット経路なし) を対比する (ゲート1第2回指摘)。
  const float period = 0.005f;
  int consecutive = 0;
  uint32_t total = 0;
  const float dts[] = {period * 2.0f, period * 1.0f, period * 2.0f,
                       period * 1.0f, period * 2.0f, period * 1.0f};
  for (float dt : dts) {
    if (isOverrunDt(dt, period)) {
      ++consecutive;
      ++total;
    } else {
      consecutive = 0;  // 連続カウンタは正常サイクルでリセット (既存 LoopState と同じ)
    }
  }
  TEST_ASSERT_EQUAL_UINT32(3, total);  // monotonic 側は 3 回の overrun を漏れなく検出
  TEST_ASSERT_EQUAL(0, consecutive);   // 連続カウンタは最後の正常サイクルでリセット済み
}

// ---------------- telemetry_format (計画書 §3.1) ----------------

static void test_telemetry_format_full_and_diag_success() {
  char buf[kTelemetryBufferBytes];

  TelemetryDiagFields df;
  df.seq = 5;
  df.tick = 6;
  df.t_us = 123;
  df.dev = "core2-aaaa";
  df.fw = "abc123";
  df.reason = DiagReason::ReadFail;
  df.read_fail = 2;
  df.trunc = 0;
  const size_t n = formatDiagPacket(buf, sizeof(buf), df);
  TEST_ASSERT_TRUE(n > 0);
  TEST_ASSERT_TRUE(n < kTelemetryBufferBytes);
  TEST_ASSERT_NOT_NULL(strstr(buf, "\"snap_valid\":false"));
  TEST_ASSERT_NOT_NULL(strstr(buf, "\"reason\":\"read_fail\""));

  TelemetryFullFields ff;
  ff.seq = 1;
  ff.tick = 1;
  ff.t_us = 1;
  ff.dev = "core2-aaaa";
  ff.fw = "abc123";
  const size_t n2 = formatFullPacket(buf, sizeof(buf), ff);
  TEST_ASSERT_TRUE(n2 > 0);
  TEST_ASSERT_TRUE(n2 < kTelemetryBufferBytes);
  TEST_ASSERT_NOT_NULL(strstr(buf, "\"snap_valid\":true"));
  TEST_ASSERT_NOT_NULL(strstr(buf, "\"v\":1"));
}

static void test_telemetry_format_truncation_forced() {
  // バッファを意図的に極小にして truncation を強制する (計画書 §3.1)。
  TelemetryFullFields f;
  f.dev = "core2-aaaa";
  f.fw = "deadbeef";
  char buf[16];
  const size_t n = formatFullPacket(buf, sizeof(buf), f);
  TEST_ASSERT_EQUAL_UINT32(0u, static_cast<uint32_t>(n));

  TelemetryDiagFields df;
  df.dev = "core2-aaaa";
  df.fw = "deadbeef";
  const size_t n2 = formatDiagPacket(buf, sizeof(buf), df);
  TEST_ASSERT_EQUAL_UINT32(0u, static_cast<uint32_t>(n2));
}

// ---------------- WifiGuard: WIFI_QUIET / freshness 5 ケース (計画書 §3.1) ----------------

static void test_wifi_quiet_five_cases() {
  // (1) no-publish: loop_count が常に 0 のまま (未 publish の既定値)
  {
    FreshnessTracker ft;
    const bool f1 = ft.observe(true, 0);
    const bool f2 = ft.observe(true, 0);
    TEST_ASSERT_FALSE(f1);
    TEST_ASSERT_FALSE(f2);
    TEST_ASSERT_TRUE(wifiQuiet(f2, false, false));
  }
  // (2) stale-loop: 前進後に停滞
  {
    FreshnessTracker ft;
    ft.observe(true, 10);
    const bool f1 = ft.observe(true, 11);
    TEST_ASSERT_TRUE(f1);
    const bool f2 = ft.observe(true, 11);
    TEST_ASSERT_FALSE(f2);
    TEST_ASSERT_TRUE(wifiQuiet(f2, false, false));
  }
  // (3) read-fail
  {
    FreshnessTracker ft;
    ft.observe(true, 5);
    const bool f = ft.observe(false, 999);
    TEST_ASSERT_FALSE(f);
    TEST_ASSERT_TRUE(wifiQuiet(f, false, false));
  }
  // (4) fresh だが Balancing
  {
    FreshnessTracker ft;
    ft.observe(true, 1);
    const bool f = ft.observe(true, 2);
    TEST_ASSERT_TRUE(f);
    TEST_ASSERT_TRUE(wifiQuiet(f, /*balancing=*/true, false));
  }
  // (5) fresh かつ非 Balancing かつ非 arm_pending → quiet ではない
  {
    FreshnessTracker ft;
    ft.observe(true, 1);
    const bool f = ft.observe(true, 2);
    TEST_ASSERT_TRUE(f);
    TEST_ASSERT_FALSE(wifiQuiet(f, false, false));
  }
}

// ---------------- WifiGuard: begin/abort 状態機械 (計画書 §3.1) ----------------

static void test_wifi_guard_begin_gated_by_quiet() {
  FakeWifiState st;
  WifiGuard::Params p;
  p.reconnect_backoff_ticks = 0;
  WifiGuard g(makeFakeOps(&st), p);

  g.tick(true, 1, false, false);  // 初回観測: fresh 未確定 → quiet
  TEST_ASSERT_EQUAL(0, st.begin_calls);

  g.tick(true, 2, /*balancing=*/true, false);  // fresh だが Balancing → quiet
  TEST_ASSERT_EQUAL(0, st.begin_calls);

  g.tick(true, 3, false, false);  // fresh ∧ 非Balancing ∧ 非arm_pending → begin
  TEST_ASSERT_EQUAL(1, st.begin_calls);
  TEST_ASSERT_TRUE(g.connecting());
}

static void test_wifi_guard_connecting_abort_exactly_once() {
  FakeWifiState st;
  WifiGuard::Params p;
  p.reconnect_backoff_ticks = 0;
  p.abort_confirm_ticks = 2;
  WifiGuard g(makeFakeOps(&st), p);

  g.tick(true, 1, false, false);
  g.tick(true, 2, false, false);  // begin 発行 → connecting_
  TEST_ASSERT_EQUAL(1, st.begin_calls);
  TEST_ASSERT_TRUE(g.connecting());

  // CONNECTING 中に Balancing へ遷移 (WIFI_QUIET) → ちょうど1回の disconnect
  g.tick(true, 3, /*balancing=*/true, false);
  TEST_ASSERT_EQUAL(1, st.disconnect_calls);
  TEST_ASSERT_TRUE(g.aborting());

  // 確認イベント (自発的切断 = ASSOC_LEAVE) 到着 → abort 完了
  g.pushEvent(WifiGuard::EventKind::StaDisconnected, WifiGuard::kReasonAssocLeave);
  g.tick(true, 4, true, false);
  TEST_ASSERT_FALSE(g.aborting());
  TEST_ASSERT_FALSE(g.connecting());
  TEST_ASSERT_EQUAL(1, st.disconnect_calls);  // 追加の disconnect は発行されない

  // Balancing が続く間、begin の retry は発行されない
  g.tick(true, 5, true, false);
  TEST_ASSERT_EQUAL(1, st.begin_calls);

  // Balancing 終了・fresh 観測 → 新規 begin
  g.tick(true, 6, false, false);
  TEST_ASSERT_EQUAL(2, st.begin_calls);
}

static void test_wifi_guard_abort_ignores_time_without_confirm_event() {
  // 「status が非接続でも one-shot begin が継続中」を模す: 確認イベントが来ない限り
  // 何 tick 待っても abort は解除されない (status 変化だけでは完了扱いしない)。
  FakeWifiState st;
  WifiGuard::Params p;
  p.reconnect_backoff_ticks = 0;
  p.abort_confirm_ticks = 1000;  // 十分長く待たせる (disconnect() 自体は成功するため retry も起きない)
  WifiGuard g(makeFakeOps(&st), p);

  g.tick(true, 1, false, false);
  g.tick(true, 2, false, false);
  g.tick(true, 3, true, false);  // Balancing へ → abort 開始
  TEST_ASSERT_TRUE(g.aborting());
  TEST_ASSERT_EQUAL(1, st.disconnect_calls);

  for (uint32_t i = 0; i < 20; ++i) {
    g.tick(true, 4 + i, true, false);
    TEST_ASSERT_TRUE(g.aborting());
  }
  TEST_ASSERT_EQUAL(1, st.disconnect_calls);  // 時間経過だけでは retry も完了もしない

  g.pushEvent(WifiGuard::EventKind::StaDisconnected, WifiGuard::kReasonAssocLeave);
  g.tick(true, 100, true, false);
  TEST_ASSERT_FALSE(g.aborting());
}

static void test_wifi_guard_abort_retry_then_radio_off_latch() {
  FakeWifiState st;
  st.disconnect_return = false;  // disconnect() が常に失敗を返す
  WifiGuard::Params p;
  p.reconnect_backoff_ticks = 0;
  p.abort_confirm_ticks = 2;
  p.abort_max_retries = 2;
  WifiGuard g(makeFakeOps(&st), p);

  g.tick(true, 1, false, false);
  g.tick(true, 2, false, false);  // begin → connecting_
  TEST_ASSERT_TRUE(g.connecting());

  g.tick(true, 3, true, false);  // Balancing へ → abort 開始 (1回目、失敗)
  TEST_ASSERT_TRUE(g.aborting());
  TEST_ASSERT_EQUAL(1, st.disconnect_calls);

  g.tick(true, 4, true, false);  // disconnect() 失敗 → 即座にリトライ
  TEST_ASSERT_EQUAL(2, st.disconnect_calls);
  TEST_ASSERT_TRUE(g.aborting());

  g.tick(true, 5, true, false);  // 2回目のリトライ (上限到達)
  TEST_ASSERT_EQUAL(3, st.disconnect_calls);
  TEST_ASSERT_TRUE(g.aborting());

  g.tick(true, 6, true, false);  // リトライ上限到達 → radio off へ
  TEST_ASSERT_FALSE(g.aborting());
  TEST_ASSERT_TRUE(g.radioOffPending());
  TEST_ASSERT_EQUAL(1, st.set_radio_off_calls);
  TEST_ASSERT_FALSE(g.wifiAbortFailed());

  st.radio_off_status = true;  // radio 停止の確認
  g.tick(true, 7, true, false);
  TEST_ASSERT_TRUE(g.wifiAbortFailed());
  TEST_ASSERT_EQUAL_UINT32(1u, g.wifiAbortFailedTotal());

  // 以降 Wi-Fi 活動は全停止 (Balancing 終了後も begin は再発行されない)
  g.tick(true, 8, false, false);
  TEST_ASSERT_EQUAL(1, st.begin_calls);
}

// ---------------- WifiGuard: lib_reconnect_pending ライフサイクル (計画書 §3.1) ----------------

static void test_wifi_guard_lib_reconnect_idle_resolves_then_arm_no_abort() {
  FakeWifiState st;
  // reconnect_backoff_ticks は既定値のまま (>0) にする: 0 にすると AP 喪失の同一
  // tick で guard 自身の begin() が即座に再発行され (§3.1 ライフサイクル条件 (d))、
  // lib_reconnect_pending が観測できないまま同時にクリアされてしまうため。
  WifiGuard::Params p;
  WifiGuard g(makeFakeOps(&st), p);

  g.tick(true, 1, false, false);
  g.tick(true, 2, false, false);  // begin
  g.pushEvent(WifiGuard::EventKind::GotIp);
  g.tick(true, 3, false, false);
  TEST_ASSERT_TRUE(g.connected());

  // Idle 中 (非 Balancing・非 arm_pending) に AP 一時喪失 → ライブラリ one-shot 発火
  g.pushEvent(WifiGuard::EventKind::StaDisconnected, /*reason=*/1);
  g.tick(true, 4, false, false);
  TEST_ASSERT_TRUE(g.libReconnectPending());
  TEST_ASSERT_EQUAL(0, st.disconnect_calls);  // Idle 中は abort しない

  // one-shot 成功 (GOT_IP) → pending クリア
  g.pushEvent(WifiGuard::EventKind::GotIp);
  g.tick(true, 5, false, false);
  TEST_ASSERT_FALSE(g.libReconnectPending());

  // その後 arm (Balancing) → disconnect は発行されない
  g.tick(true, 6, /*balancing=*/true, false);
  TEST_ASSERT_EQUAL(0, st.disconnect_calls);
}

static void test_wifi_guard_lib_reconnect_one_shot_failure_clears_pending() {
  FakeWifiState st;
  // 既定の reconnect_backoff_ticks (>0) を使う (理由は上のテストと同じ)。
  WifiGuard::Params p;
  WifiGuard g(makeFakeOps(&st), p);

  g.tick(true, 1, false, false);
  g.tick(true, 2, false, false);
  g.pushEvent(WifiGuard::EventKind::GotIp);
  g.tick(true, 3, false, false);

  g.pushEvent(WifiGuard::EventKind::StaDisconnected, /*reason=*/1);
  g.tick(true, 4, false, false);
  TEST_ASSERT_TRUE(g.libReconnectPending());

  // one-shot の begin() も失敗 → 再度の非自発切断 (b): in-flight 終了とみなし clear
  g.pushEvent(WifiGuard::EventKind::StaDisconnected, /*reason=*/1);
  g.tick(true, 5, false, false);
  TEST_ASSERT_FALSE(g.libReconnectPending());
  TEST_ASSERT_EQUAL(0, st.disconnect_calls);  // Idle 中なので abort は発行されない
}

static void test_wifi_guard_lib_reconnect_during_arm_pending_aborts_once() {
  FakeWifiState st;
  WifiGuard::Params p;
  p.reconnect_backoff_ticks = 0;
  p.abort_confirm_ticks = 2;
  WifiGuard g(makeFakeOps(&st), p);

  g.tick(true, 1, false, false);
  g.tick(true, 2, false, false);
  g.pushEvent(WifiGuard::EventKind::GotIp);
  g.tick(true, 3, false, false);
  TEST_ASSERT_TRUE(g.connected());

  // 直立ホールド (arm_pending) 中に AP 喪失 → ライブラリ one-shot 発火
  g.pushEvent(WifiGuard::EventKind::StaDisconnected, /*reason=*/1);
  g.tick(true, 4, /*balancing=*/false, /*arm_pending=*/true);
  TEST_ASSERT_TRUE(g.aborting());
  TEST_ASSERT_EQUAL(1, st.disconnect_calls);

  g.pushEvent(WifiGuard::EventKind::StaDisconnected, WifiGuard::kReasonAssocLeave);
  g.tick(true, 5, false, true);
  TEST_ASSERT_FALSE(g.aborting());
  TEST_ASSERT_FALSE(g.libReconnectPending());
  TEST_ASSERT_EQUAL(1, st.disconnect_calls);  // ちょうど1回
}

// ---------------- WifiGuard: 同一 tick 内で pending→GotIp が完結するケース
// (ゲート2レビュー(2回目)指摘1) ----------------
// arm_pending/Balancing 中の AP 喪失で、次の tick までの間に
// STA_DISCONNECTED(非自発的) と後続の GOT_IP の両方がキューされると、両方が
// 同一 tick の drainEvents() 内で drain される。GotIp ハンドラが
// lib_reconnect_pending_ をクリアしてしまうため、ラッチがなければ直後の
// quiet 判定は connecting_/lib_reconnect_pending_ のいずれも false と観測し
// startAbort() がスキップされてしまう (udp_ready_ 済みなら quiet 窓中に
// 成立した接続経由で送信が再開されるバグ)。per-tick ラッチにより、この
// ケースでも quiet であれば有界キャンセルの意味論どおり ちょうど1回
// disconnect() が発行されることを検証する。

static void test_wifi_guard_lib_reconnect_completes_same_tick_during_arm_pending_aborts_once() {
  FakeWifiState st;
  WifiGuard::Params p;
  p.abort_confirm_ticks = 2;
  WifiGuard g(makeFakeOps(&st), p);

  g.tick(true, 1, false, false);
  g.tick(true, 2, false, false);  // begin -> connecting_
  g.pushEvent(WifiGuard::EventKind::GotIp);
  g.tick(true, 3, false, false);
  TEST_ASSERT_TRUE(g.connected());

  // 直立ホールド (arm_pending) 中に AP 喪失 → ライブラリ one-shot 発火 →
  // 同一 tick 内で one-shot 再接続の成功 (GOT_IP) までキューされる (次の 50ms
  // tick までの間に両方のイベントが立て続けに発生したケースを模擬)。
  g.pushEvent(WifiGuard::EventKind::StaDisconnected, /*reason=*/1);
  g.pushEvent(WifiGuard::EventKind::GotIp);
  g.tick(true, 4, /*balancing=*/false, /*arm_pending=*/true);

  // drain 直後は lib_reconnect_pending_/connecting_ ともに false に見えるが、
  // ラッチにより quiet 窓中に成立したこの接続は有界キャンセルされなければ
  // ならない。
  TEST_ASSERT_TRUE(g.aborting());
  TEST_ASSERT_EQUAL(1, st.disconnect_calls);  // ちょうど1回
  TEST_ASSERT_FALSE(g.libReconnectPending());
}

static void test_wifi_guard_lib_reconnect_completes_same_tick_not_quiet_keeps_connection() {
  FakeWifiState st;
  WifiGuard::Params p;
  WifiGuard g(makeFakeOps(&st), p);

  g.tick(true, 1, false, false);
  g.tick(true, 2, false, false);
  g.pushEvent(WifiGuard::EventKind::GotIp);
  g.tick(true, 3, false, false);
  TEST_ASSERT_TRUE(g.connected());

  // 同じ「pending→GotIp が同一 tick で完結する」シーケンスでも、その tick が
  // !WIFI_QUIET (非 Balancing・非 arm_pending・fresh) なら abort してはならず、
  // ライブラリが自律的に再確立した接続をそのまま容認する。
  g.pushEvent(WifiGuard::EventKind::StaDisconnected, /*reason=*/1);
  g.pushEvent(WifiGuard::EventKind::GotIp);
  g.tick(true, 4, /*balancing=*/false, /*arm_pending=*/false);

  TEST_ASSERT_FALSE(g.aborting());
  TEST_ASSERT_TRUE(g.connected());
  TEST_ASSERT_EQUAL(0, st.disconnect_calls);
  TEST_ASSERT_FALSE(g.libReconnectPending());

  // ラッチは !quiet の tick で消費済み (リセット済み) であり、後続で
  // arm_pending に入っても、この時点で新たな pending/イベントがない限り
  // 誤って持ち越されて abort を起こしてはならない。
  g.tick(true, 5, /*balancing=*/false, /*arm_pending=*/true);
  TEST_ASSERT_FALSE(g.aborting());
  TEST_ASSERT_EQUAL(0, st.disconnect_calls);
}

// ---------------- WifiGuard: abort/radio-off 中の遅延 GOT_IP (ゲート2レビュー指摘4) ----------------
// abort/radio-off 進行中にライブラリの遅延 GOT_IP イベントが drain されると
// connected_ が true に戻り得るが、この窓では abort-class 操作のみ許可されるため
// readyToAttempt()/trySend() は false/NotConnected を維持しなければならない。

static void test_wifi_guard_delayed_got_ip_during_abort_blocks_send() {
  FakeWifiState st;
  WifiGuard::Params p;
  p.reconnect_backoff_ticks = 0;
  p.abort_confirm_ticks = 3;
  WifiGuard g(makeFakeOps(&st), p);
  const uint8_t payload[4] = {1, 2, 3, 4};

  // 通常接続 + prewarm 成功 (udp_ready_ = true) にしておく。
  g.tick(true, 1, false, false);
  g.tick(true, 2, false, false);  // begin -> connecting_
  g.pushEvent(WifiGuard::EventKind::GotIp);
  g.tick(true, 3, false, false);
  TEST_ASSERT_TRUE(g.connected());
  TEST_ASSERT_EQUAL(static_cast<int>(WifiGuard::SendOutcome::Sent),
                    static_cast<int>(g.trySend(payload, sizeof(payload))));
  TEST_ASSERT_TRUE(g.udpReady());

  // Idle 中に AP を一時喪失 (非自発的切断) -> ライブラリ one-shot 再接続が
  // in-flight になり (lib_reconnect_pending_)、かつ同一 tick で Balancing へ
  // 遷移 (WIFI_QUIET 成立) させて abort を開始させる。reconnect_backoff_ticks=0
  // のまま quiet が成立しない tick を挟むと、guard 自身の begin() 再発行 (ライフ
  // サイクル条件 (d)) が同一 tick 内で lib_reconnect_pending_ を消してしまう
  // (test_wifi_guard_lib_reconnect_idle_resolves_then_arm_no_abort のコメント
  // 参照) ため、one-shot 発火と WIFI_QUIET 成立を同一 tick に揃える。
  g.pushEvent(WifiGuard::EventKind::StaDisconnected, /*reason=*/1);
  g.tick(true, 4, /*balancing=*/true, false);
  TEST_ASSERT_TRUE(g.aborting());
  TEST_ASSERT_EQUAL(1, st.disconnect_calls);
  TEST_ASSERT_FALSE(g.connected());

  // abort 完了確認 (ASSOC_LEAVE / STA_STOP) が届く前に、ライブラリ側の遅延
  // GOT_IP が drain される (in-flight one-shot が実は成功していたレース)。
  g.pushEvent(WifiGuard::EventKind::GotIp);
  g.tick(true, 5, true, false);
  TEST_ASSERT_TRUE(g.connected());  // バグの温床: connected_ は true に戻る
  TEST_ASSERT_TRUE(g.aborting());   // abort はまだ進行中 (確認イベント未到着)
  TEST_ASSERT_EQUAL(1, st.disconnect_calls);  // 追加の disconnect は発行されない

  // それでも送信は一切許可されない (abort-class 操作のみが許される窓)。
  TEST_ASSERT_FALSE(g.readyToAttempt());
  const WifiGuard::SendOutcome o = g.trySend(payload, sizeof(payload));
  TEST_ASSERT_EQUAL(static_cast<int>(WifiGuard::SendOutcome::NotConnected), static_cast<int>(o));
  TEST_ASSERT_EQUAL(1, st.begin_packet_calls);  // 直前の成功送信の1回のみ (再試行なし)
}

// ---------------- WifiGuard: abort 完了と同一 drain 内のラッチ残留
// (2026-07-06 ゲート2レビュー(8回目)指摘2) ----------------
// abort 進行中に、遅延 GOT_IP (one-shot 再接続の完了) と post-abort の
// STA_DISCONNECTED(ASSOC_LEAVE) (abort 自身の完了確認) が同一 tick の
// drainEvents() 内で両方消化されると: GotIp ハンドラが
// connection_established_in_drain_ ラッチを立てた直後に、StaDisconnected
// ハンドラが completeAbort() を呼んで abort 自体は完了する
// (aborting_==false, connected_==false)。ラッチだけを見て abort する旧実装
// では、この完了済み (かつ既に切断済みの) 接続へ二度目の startAbort() が
// 発行されてしまい、対応する ASSOC_LEAVE は二度と来ないため確認 tick を
// 消費してリトライへ進む。修正後はラッチ条件に connected_ (drain 終了時点
// で接続が生存しているか) を追加し、この場合は取り消す対象がないとして
// 二度目の abort を発行しない。
static void test_wifi_guard_abort_completes_same_drain_as_delayed_got_ip_no_second_abort() {
  FakeWifiState st;
  WifiGuard::Params p;
  p.reconnect_backoff_ticks = 0;
  p.abort_confirm_ticks = 3;
  p.abort_max_retries = 3;
  WifiGuard g(makeFakeOps(&st), p);

  // 通常接続を確立する。
  g.tick(true, 1, false, false);
  g.tick(true, 2, false, false);  // begin -> connecting_
  g.pushEvent(WifiGuard::EventKind::GotIp);
  g.tick(true, 3, false, false);
  TEST_ASSERT_TRUE(g.connected());

  // AP を一時喪失 (非自発的切断) -> ライブラリ one-shot 再接続が in-flight
  // になり (lib_reconnect_pending_)、同一 tick で Balancing (quiet) へ遷移
  // させて abort を開始させる (test_wifi_guard_delayed_got_ip_during_abort_
  // blocks_send と同じ土台)。
  g.pushEvent(WifiGuard::EventKind::StaDisconnected, /*reason=*/1);
  g.tick(true, 4, /*balancing=*/true, false);
  TEST_ASSERT_TRUE(g.aborting());
  TEST_ASSERT_EQUAL(1, st.disconnect_calls);
  TEST_ASSERT_FALSE(g.connected());

  // 確認 (ASSOC_LEAVE) 未到着のうちに、in-flight だった one-shot の遅延
  // GOT_IP と、abort 自身の完了確認 (ASSOC_LEAVE) が同一 tick の drain 内
  // に両方キューされる (レース: one-shot が実は成功していた直後に、abort
  // で発行した disconnect() の確認も同じ 50ms 窓に届いた場合)。
  g.pushEvent(WifiGuard::EventKind::GotIp);
  g.pushEvent(WifiGuard::EventKind::StaDisconnected, WifiGuard::kReasonAssocLeave);
  g.tick(true, 5, /*balancing=*/true, false);

  // abort は (StaDisconnected の completeAbort() により) この1 tick で完了
  // していなければならない。ラッチ (connection_established_in_drain) だけ
  // を見る旧実装では、GotIp が立てたラッチが completeAbort() 後も残り、
  // connected_==false なのに quiet && ラッチ で二度目の startAbort() が
  // 発行されて aborting() が再び true に戻ってしまう (このテストは修正
  // なしでは失敗する: 逆検証済み)。
  TEST_ASSERT_FALSE(g.aborting());
  TEST_ASSERT_FALSE(g.connected());
  TEST_ASSERT_EQUAL(1, st.disconnect_calls);  // 追加の disconnect は発行されない
  TEST_ASSERT_FALSE(g.radioOffPending());
  TEST_ASSERT_FALSE(g.wifiAbortFailed());

  // 後続の tick でも (新しいイベントがない限り) 状態は安定したまま:
  // 二度目の abort が発行されていれば、確認イベントが二度と来ないため
  // リトライを重ねて radio-off/latch まで進んでしまうはずだが、修正後は
  // 何も起きない。
  for (uint32_t t = 6; t < 6 + (p.abort_confirm_ticks * (p.abort_max_retries + 1)); ++t) {
    g.tick(true, t, /*balancing=*/true, false);
  }
  TEST_ASSERT_FALSE(g.aborting());
  TEST_ASSERT_FALSE(g.radioOffPending());
  TEST_ASSERT_FALSE(g.wifiAbortFailed());
  TEST_ASSERT_EQUAL(1, st.disconnect_calls);
}

static void test_wifi_guard_delayed_got_ip_during_radio_off_pending_blocks_send() {
  FakeWifiState st;
  st.disconnect_return = false;  // disconnect() を常に失敗させ、retry 上限到達を強制する
  WifiGuard::Params p;
  p.reconnect_backoff_ticks = 0;
  p.abort_confirm_ticks = 1;
  p.abort_max_retries = 1;
  WifiGuard g(makeFakeOps(&st), p);
  const uint8_t payload[4] = {1, 2, 3, 4};

  g.tick(true, 1, false, false);
  g.tick(true, 2, false, false);  // begin -> connecting_
  g.tick(true, 3, true, false);   // Balancing -> abort 開始 (1回目、失敗)
  TEST_ASSERT_TRUE(g.aborting());
  g.tick(true, 4, true, false);   // リトライ (まだ失敗)
  TEST_ASSERT_TRUE(g.aborting());
  g.tick(true, 5, true, false);   // リトライ上限到達 -> radio off pending へ
  TEST_ASSERT_FALSE(g.aborting());
  TEST_ASSERT_TRUE(g.radioOffPending());

  // radio 停止確認前に遅延 GOT_IP が drain される。
  g.pushEvent(WifiGuard::EventKind::GotIp);
  g.tick(true, 6, true, false);
  TEST_ASSERT_TRUE(g.connected());       // バグの温床
  TEST_ASSERT_TRUE(g.radioOffPending());  // 停止確認はまだ

  TEST_ASSERT_FALSE(g.readyToAttempt());
  const WifiGuard::SendOutcome o = g.trySend(payload, sizeof(payload));
  TEST_ASSERT_EQUAL(static_cast<int>(WifiGuard::SendOutcome::NotConnected), static_cast<int>(o));
  TEST_ASSERT_EQUAL(0, st.begin_packet_calls);
}

// ---------------- WifiGuard: prewarm / udp_ready (計画書 §3.1) ----------------

static void test_wifi_guard_prewarm_beginpacket_failure_blocks_write() {
  FakeWifiState st;
  st.begin_packet_return = 0;  // 常に失敗 (socket/malloc 失敗を模す)
  WifiGuard::Params p;
  p.reconnect_backoff_ticks = 0;
  WifiGuard g(makeFakeOps(&st), p);

  g.tick(true, 1, false, false);
  g.tick(true, 2, false, false);
  g.pushEvent(WifiGuard::EventKind::GotIp);
  g.tick(true, 3, false, false);
  TEST_ASSERT_TRUE(g.connected());
  TEST_ASSERT_TRUE(g.readyToAttempt());

  const uint8_t payload[4] = {1, 2, 3, 4};
  const WifiGuard::SendOutcome o = g.trySend(payload, sizeof(payload));
  TEST_ASSERT_EQUAL(static_cast<int>(WifiGuard::SendOutcome::BeginPacketFailed),
                    static_cast<int>(o));
  TEST_ASSERT_EQUAL(0, st.write_calls);  // beginPacket 失敗後は write に進まない
  TEST_ASSERT_EQUAL(0, st.end_packet_calls);
  TEST_ASSERT_FALSE(g.udpReady());
  TEST_ASSERT_EQUAL_UINT32(1u, g.prewarmFailTotal());
}

static void test_wifi_guard_prewarm_withheld_during_arm_pending() {
  FakeWifiState st;
  WifiGuard::Params p;
  p.reconnect_backoff_ticks = 0;
  WifiGuard g(makeFakeOps(&st), p);

  g.tick(true, 1, false, false);
  g.tick(true, 2, false, false);  // begin

  // arm_pending 中に GOT_IP (Connected 遷移) が drain された場合、quiet 窓中に
  // 成立した接続として有界キャンセルされ、prewarm も発生しない
  // (ゲート2レビュー(4回目)指摘1 で「接続維持+prewarm保留」から abort へ仕様変更)
  g.pushEvent(WifiGuard::EventKind::GotIp);
  g.tick(true, 3, false, /*arm_pending=*/true);
  TEST_ASSERT_TRUE(g.aborting());
  TEST_ASSERT_FALSE(g.readyToAttempt());
  TEST_ASSERT_EQUAL(0, st.begin_packet_calls);
  TEST_ASSERT_EQUAL(1, st.disconnect_calls);

  // abort 確認 → arm_pending 解消 → 再接続 → prewarm 許可
  g.pushEvent(WifiGuard::EventKind::StaDisconnected, WifiGuard::kReasonAssocLeave);
  g.tick(true, 4, false, false);  // abort 完了 + begin 再発行
  g.pushEvent(WifiGuard::EventKind::GotIp);
  g.tick(true, 5, false, false);  // !quiet で接続成立 → abort されない
  TEST_ASSERT_FALSE(g.aborting());
  TEST_ASSERT_TRUE(g.readyToAttempt());
}

static void test_wifi_guard_endpacket_failure_keeps_udp_ready_during_balancing() {
  FakeWifiState st;
  WifiGuard::Params p;
  p.reconnect_backoff_ticks = 0;
  WifiGuard g(makeFakeOps(&st), p);

  g.tick(true, 1, false, false);
  g.tick(true, 2, false, false);
  g.pushEvent(WifiGuard::EventKind::GotIp);
  g.tick(true, 3, false, false);

  const uint8_t payload[4] = {1, 2, 3, 4};
  TEST_ASSERT_EQUAL(static_cast<int>(WifiGuard::SendOutcome::Sent),
                    static_cast<int>(g.trySend(payload, sizeof(payload))));
  TEST_ASSERT_TRUE(g.udpReady());

  // Balancing 中の endPacket 失敗: udp_ready を維持したまま送信失敗として計上
  g.tick(true, 4, /*balancing=*/true, false);
  st.end_packet_return = 0;
  const WifiGuard::SendOutcome o2 = g.trySend(payload, sizeof(payload));
  TEST_ASSERT_EQUAL(static_cast<int>(WifiGuard::SendOutcome::EndPacketFailed),
                    static_cast<int>(o2));
  TEST_ASSERT_TRUE(g.udpReady());  // 維持される
  TEST_ASSERT_EQUAL_UINT32(1u, g.sendFailTotal());
}

// 切断で udp_ready がリセットされ、再接続後の初回送信 (prewarm) が WIFI_QUIET 窓で
// 通らないこと (ゲート2第3回指摘対応: stale readiness による prewarm ゲート迂回の防止)
static void test_wifi_guard_disconnect_clears_udp_ready_blocks_quiet_send_after_reconnect() {
  FakeWifiState st;
  WifiGuard::Params p;
  p.reconnect_backoff_ticks = 0;
  WifiGuard g(makeFakeOps(&st), p);

  // 接続 → prewarm 完了 (udp_ready)
  g.tick(true, 1, false, false);
  g.tick(true, 2, false, false);
  g.pushEvent(WifiGuard::EventKind::GotIp);
  g.tick(true, 3, false, false);
  const uint8_t payload[4] = {1, 2, 3, 4};
  TEST_ASSERT_EQUAL(static_cast<int>(WifiGuard::SendOutcome::Sent),
                    static_cast<int>(g.trySend(payload, sizeof(payload))));
  TEST_ASSERT_TRUE(g.udpReady());

  // 接続喪失 (自発的扱いで lib one-shot を絡めない) → readiness も無効化される
  g.pushEvent(WifiGuard::EventKind::StaDisconnected, WifiGuard::kReasonAssocLeave);
  g.tick(true, 4, false, false);  // drain (+ !quiet なので begin 再発行)
  TEST_ASSERT_FALSE(g.udpReady());

  // 再接続完了が WIFI_QUIET (Balancing) 中に drain されるケース:
  // stale な udp_ready_ で readyToAttempt() が true になってはならず (3回目指摘)、
  // さらに quiet 窓中に成立した接続は有界キャンセルされる (4回目指摘)
  g.pushEvent(WifiGuard::EventKind::GotIp);
  g.tick(true, 5, /*balancing=*/true, false);
  TEST_ASSERT_FALSE(g.readyToAttempt());  // quiet 窓では送信不可
  TEST_ASSERT_TRUE(g.aborting());         // quiet 窓中に成立した接続は abort
  TEST_ASSERT_EQUAL(1, st.disconnect_calls);

  // abort 確認 → !WIFI_QUIET に戻ったら再接続 → prewarm が許可される
  g.pushEvent(WifiGuard::EventKind::StaDisconnected, WifiGuard::kReasonAssocLeave);
  g.tick(true, 6, false, false);  // abort 完了 + begin 再発行
  TEST_ASSERT_FALSE(g.aborting());
  g.pushEvent(WifiGuard::EventKind::GotIp);
  g.tick(true, 7, false, false);  // !quiet で接続成立 → abort されない
  TEST_ASSERT_FALSE(g.aborting());
  TEST_ASSERT_TRUE(g.readyToAttempt());
  TEST_ASSERT_FALSE(g.udpReady());  // 切断で reset 済み → 要 prewarm
  TEST_ASSERT_EQUAL(static_cast<int>(WifiGuard::SendOutcome::Sent),
                    static_cast<int>(g.trySend(payload, sizeof(payload))));
  TEST_ASSERT_TRUE(g.udpReady());
}

// !quiet 中に正規発行した begin() が tick 間に完了し、次 tick では既に
// arm_pending/Balancing (WIFI_QUIET) になっているケース: GotIp ハンドラが
// connecting_ をクリアするためラッチなしでは abort がスキップされる
// (ゲート2レビュー(4回目)指摘1・P1 対応)
static void test_wifi_guard_own_begin_completes_during_quiet_aborts_once() {
  FakeWifiState st;
  WifiGuard::Params p;
  p.reconnect_backoff_ticks = 0;
  WifiGuard g(makeFakeOps(&st), p);

  g.tick(true, 1, false, false);
  g.tick(true, 2, false, false);  // !quiet で正規の begin 発行
  TEST_ASSERT_EQUAL(1, st.begin_calls);
  TEST_ASSERT_TRUE(g.connecting());

  // tick 間に接続が完了し、次 tick では既に arm_pending (WIFI_QUIET)
  g.pushEvent(WifiGuard::EventKind::GotIp);
  g.tick(true, 3, false, /*arm_pending=*/true);
  TEST_ASSERT_EQUAL(1, st.disconnect_calls);  // 有界キャンセルが発火
  TEST_ASSERT_TRUE(g.aborting());
  TEST_ASSERT_FALSE(g.readyToAttempt());

  // abort 確認 → 完了。quiet が続く限り begin は再発行されない
  g.pushEvent(WifiGuard::EventKind::StaDisconnected, WifiGuard::kReasonAssocLeave);
  g.tick(true, 4, false, true);
  TEST_ASSERT_FALSE(g.aborting());
  TEST_ASSERT_EQUAL(1, st.begin_calls);
  TEST_ASSERT_EQUAL(1, st.disconnect_calls);
}

// ---------------- WifiGuard: イベントキュー満杯 (2026-07-05 ゲート2レビュー
// (5回目)指摘3, fail-closed) ----------------
// GOT_IP/STA_DISCONNECTED はキュー満杯で破棄されると connected_/connecting_/
// lib_reconnect_pending_/udp_ready_ の唯一の入力源を失うため、破棄が起きた
// tick は「状態不明」として保守的に扱わなければならない: 接続系の派生状態を
// disconnected 基線にリセットし、in-flight 活動があり得る前提で startAbort()
// を1回発行して既存の有界キャンセル経路 (確認→リトライ→WIFI_OFF終端) に乗せる。

static void test_wifi_guard_event_queue_overflow_triggers_abort_and_reset() {
  FakeWifiState st;
  WifiGuard::Params p;
  p.reconnect_backoff_ticks = 0;
  p.abort_confirm_ticks = 2;
  WifiGuard g(makeFakeOps(&st), p);

  // 通常の接続 + prewarm 完了状態を作る。
  g.tick(true, 1, false, false);
  g.tick(true, 2, false, false);  // begin -> connecting_
  g.pushEvent(WifiGuard::EventKind::GotIp);
  g.tick(true, 3, false, false);
  TEST_ASSERT_TRUE(g.connected());
  const uint8_t payload[4] = {1, 2, 3, 4};
  TEST_ASSERT_EQUAL(static_cast<int>(WifiGuard::SendOutcome::Sent),
                    static_cast<int>(g.trySend(payload, sizeof(payload))));
  TEST_ASSERT_TRUE(g.udpReady());
  TEST_ASSERT_EQUAL_UINT32(0u, g.eventOverflowTotal());

  // キューを溢れさせる (容量16に対し20回 push -> 末尾4回は破棄されオーバーフロー
  // フラグが立つ)。
  for (int i = 0; i < 20; ++i) {
    g.pushEvent(WifiGuard::EventKind::GotIp);
  }

  // 次 tick: drainEvents() 後にオーバーフローを検出し、fail-closed 処理が走る。
  g.tick(true, 4, false, false);
  TEST_ASSERT_EQUAL_UINT32(1u, g.eventOverflowTotal());
  TEST_ASSERT_FALSE(g.connected());
  TEST_ASSERT_FALSE(g.udpReady());
  TEST_ASSERT_TRUE(g.aborting());
  TEST_ASSERT_EQUAL(1, st.disconnect_calls);  // startAbort() がちょうど1回発行される

  // abort 完了確認 → 完了。オーバーフローラッチは exchange 済みなので、
  // 新規イベントなしの以降の tick で再発行されない。
  g.pushEvent(WifiGuard::EventKind::StaDisconnected, WifiGuard::kReasonAssocLeave);
  g.tick(true, 5, false, false);
  TEST_ASSERT_FALSE(g.aborting());
  TEST_ASSERT_EQUAL(1, st.disconnect_calls);
  TEST_ASSERT_EQUAL_UINT32(1u, g.eventOverflowTotal());  // 累積カウンタは保持される
}

// ---------------- 統合: armPending() が Wi-Fi 接続を Balancing 前に abort する ----------------
// (計画書 §3.1: commissioned auto-arm と BtnC 手動アームの両経路で同一機構であることの確認)

static void runArmPendingAbortsInFlightConnection(SafetyFsm& fsm, float start_now) {
  TEST_ASSERT_EQUAL(static_cast<int>(FsmState::Idle), static_cast<int>(fsm.state()));

  FakeWifiState st;
  WifiGuard::Params p;
  p.reconnect_backoff_ticks = 0;
  p.abort_confirm_ticks = 2;
  WifiGuard g(makeFakeOps(&st), p);

  float now = start_now;
  uint32_t loop_count = 0;
  for (int i = 0; i < 3; ++i) {
    ++loop_count;
    g.tick(true, loop_count, false, fsm.armPending());
  }
  TEST_ASSERT_TRUE(g.connecting());
  TEST_ASSERT_EQUAL(1, st.begin_calls);

  bool aborted_before_balancing = false;
  bool requested = false;
  for (int i = 0; i < 260 && !requested; ++i) {
    now += 0.005f;
    ++loop_count;
    const SafetyFsm::Result r = fsm.update(uprightInput(now));
    if (r.action == FsmAction::EnterBalancing) requested = true;
    g.tick(true, loop_count, fsm.state() == FsmState::Balancing, fsm.armPending());
    if (fsm.armPending() && g.aborting()) {
      g.pushEvent(WifiGuard::EventKind::StaDisconnected, WifiGuard::kReasonAssocLeave);
    }
    if (st.disconnect_calls > 0 && !g.connecting()) aborted_before_balancing = true;
  }
  TEST_ASSERT_TRUE(requested);
  TEST_ASSERT_TRUE(aborted_before_balancing);
  TEST_ASSERT_EQUAL(1, st.disconnect_calls);  // ちょうど1回の abort

  fsm.notifyBalancingEntered();
  ++loop_count;
  g.tick(true, loop_count, true, fsm.armPending());
  TEST_ASSERT_EQUAL(1, st.begin_calls);  // Balancing 中に begin は再発行されない
}

static void test_integration_wifi_abort_before_balancing_auto_arm() {
  SafetyFsm fsm;
  fsm.setParams(fsmParams(true, true));
  fsm.notifyInitDone();
  runArmPendingAbortsInFlightConnection(fsm, 0.0f);
}

static void test_integration_wifi_abort_before_balancing_manual_arm() {
  SafetyFsm fsm;
  fsm.setParams(fsmParams(false, true));  // 未コミッショニング → Disarmed 起動
  fsm.notifyInitDone();
  float now = 0.0f;
  SafetyFsm::Input in = uprightInput(now += 0.005f);
  in.stop_toggle = true;  // BtnC 手動アーム → Idle
  fsm.update(in);
  runArmPendingAbortsInFlightConnection(fsm, now);
}

// ---------------- SharedState: critical section 置換 (計画書 §3.1 前提修正) ----------------

static void test_shared_state_concurrent_publish_read_no_torn_read() {
  shared::SharedState state;
  std::atomic<bool> stop{false};
  std::atomic<bool> torn_detected{false};
  std::atomic<uint64_t> reads{0};
  std::atomic<uint64_t> writes{0};

  std::thread writer([&]() {
    shared::Snapshot s;
    uint32_t i = 0;
    while (!stop.load(std::memory_order_relaxed)) {
      ++i;
      s.loop_count = i;
      s.theta = static_cast<float>(i);
      s.dt_hist_total[0] = i;
      s.dt_hist_total[7] = i;
      s.overrun_total = i;
      s.imu_stale_total = i;
      state.publish(s);
      writes.fetch_add(1, std::memory_order_relaxed);
    }
  });

  std::thread reader([&]() {
    shared::Snapshot out;
    while (!stop.load(std::memory_order_relaxed)) {
      if (state.read(&out)) {
        reads.fetch_add(1, std::memory_order_relaxed);
        // 全フィールドが同一の loop_count 由来の値で揃っているか (torn なら不一致)
        if (static_cast<uint32_t>(out.theta) != out.loop_count ||
            out.dt_hist_total[0] != out.loop_count ||
            out.dt_hist_total[7] != out.loop_count ||
            out.overrun_total != out.loop_count ||
            out.imu_stale_total != out.loop_count) {
          torn_detected.store(true, std::memory_order_relaxed);
        }
      }
    }
  });

  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  stop.store(true, std::memory_order_relaxed);
  writer.join();
  reader.join();

  TEST_ASSERT_FALSE(torn_detected.load());
  TEST_ASSERT_TRUE(reads.load() > 0);   // 両タスクが実際に進行したこと
  TEST_ASSERT_TRUE(writes.load() > 0);
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

  // UDP telemetry Phase1 (計画書 §3.1)
  RUN_TEST(test_fsm_arm_pending_auto_arm);
  RUN_TEST(test_fsm_arm_pending_manual_arm);
  RUN_TEST(test_dt_histogram_boundaries);
  RUN_TEST(test_dt_monotonic_counter_survives_consecutive_reset);
  RUN_TEST(test_telemetry_format_full_and_diag_success);
  RUN_TEST(test_telemetry_format_truncation_forced);
  RUN_TEST(test_wifi_quiet_five_cases);
  RUN_TEST(test_wifi_guard_begin_gated_by_quiet);
  RUN_TEST(test_wifi_guard_connecting_abort_exactly_once);
  RUN_TEST(test_wifi_guard_abort_ignores_time_without_confirm_event);
  RUN_TEST(test_wifi_guard_abort_retry_then_radio_off_latch);
  RUN_TEST(test_wifi_guard_lib_reconnect_idle_resolves_then_arm_no_abort);
  RUN_TEST(test_wifi_guard_lib_reconnect_one_shot_failure_clears_pending);
  RUN_TEST(test_wifi_guard_lib_reconnect_during_arm_pending_aborts_once);
  RUN_TEST(test_wifi_guard_lib_reconnect_completes_same_tick_during_arm_pending_aborts_once);
  RUN_TEST(test_wifi_guard_lib_reconnect_completes_same_tick_not_quiet_keeps_connection);
  RUN_TEST(test_wifi_guard_delayed_got_ip_during_abort_blocks_send);
  RUN_TEST(test_wifi_guard_abort_completes_same_drain_as_delayed_got_ip_no_second_abort);
  RUN_TEST(test_wifi_guard_delayed_got_ip_during_radio_off_pending_blocks_send);
  RUN_TEST(test_wifi_guard_prewarm_beginpacket_failure_blocks_write);
  RUN_TEST(test_wifi_guard_prewarm_withheld_during_arm_pending);
  RUN_TEST(test_wifi_guard_endpacket_failure_keeps_udp_ready_during_balancing);
  RUN_TEST(test_wifi_guard_disconnect_clears_udp_ready_blocks_quiet_send_after_reconnect);
  RUN_TEST(test_wifi_guard_own_begin_completes_during_quiet_aborts_once);
  RUN_TEST(test_wifi_guard_event_queue_overflow_triggers_abort_and_reset);
  RUN_TEST(test_integration_wifi_abort_before_balancing_auto_arm);
  RUN_TEST(test_integration_wifi_abort_before_balancing_manual_arm);
  RUN_TEST(test_shared_state_concurrent_publish_read_no_torn_read);
  return UNITY_END();
}
