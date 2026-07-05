// wifi_guard.h — UDP telemetry の Wi-Fi 安全ガード (UDP telemetry Phase1 計画書 §3.1)。
// Arduino 非依存・host テスト可能。実際の WiFi/WiFiUDP API 呼び出しは Ops 経由の
// 関数ポインタ注入で抽象化する (telemetry_task.cpp が実装を bind する)。
//
// 単一の禁止述語 WIFI_QUIET (計画書 §3.1。条件の伝播漏れを防ぐため 1 箇所で定義):
//   WIFI_QUIET := snapshot が fresh でない (read 失敗/未 publish/loop_count 停滞)
//                 または snapshot.fsm == Balancing
//                 または snapshot.arm_pending == true
// 操作クラス:
//   start-class (begin/mode変更/socket作成・再作成/初回beginPacket): !WIFI_QUIET でのみ許可
//   abort-class (abort の disconnect・終端の mode(WIFI_OFF)): abort_in_flight でのみ許可
//
// 検証済みバージョン: framework-arduinoespressif32 3.20017.241212+sha.dcc1105b
// (espressif32@6.12.0 で解決。WiFiGeneric.cpp:1034- の _eventCallback 実装を実ソースで
// 確認: 非自発的切断 (reason != WIFI_REASON_ASSOC_LEAVE) の初回で first_connect
// one-shot として autoReconnect 設定に関係なく disconnect();begin(); を発行し、
// 自発的切断 (ASSOC_LEAVE=8) はこの one-shot から除外される。3.20016.0 のソースとも
// diff で同一であることを確認済み)。
#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>

namespace core {

// telemetry FSM が観測した snapshot の fresh 判定 (計画書 §3.1):
// 「read が成功し、かつ loop_count が前回観測値から前進している」。
// 未 publish の既定バッファ (loop_count==0 で不変) や read 失敗はこの述語で
// 自動的に reject される。read 失敗時は基準をリセットし、次回成功時に改めて
// 「前進」を要求する (失敗直後の 1 回だけを fresh 扱いしてしまう抜け穴を防ぐ)。
class FreshnessTracker {
 public:
  bool observe(bool read_ok, uint32_t loop_count) {
    if (!read_ok) {
      have_baseline_ = false;
      return false;
    }
    if (!have_baseline_) {
      have_baseline_ = true;
      last_loop_count_ = loop_count;
      return false;  // 初回観測は「前進」を証明できないため fresh 扱いしない
    }
    const bool advanced = (loop_count != last_loop_count_);
    last_loop_count_ = loop_count;
    return advanced;
  }

 private:
  bool have_baseline_ = false;
  uint32_t last_loop_count_ = 0;
};

// 単一の禁止述語 WIFI_QUIET (計画書 §3.1)。以降のすべての規則はこれを参照する。
inline bool wifiQuiet(bool fresh, bool balancing, bool arm_pending) {
  return !fresh || balancing || arm_pending;
}

// Wi-Fi/UDP 副作用の抽象化 (Arduino 非依存にするための関数ポインタ注入)。
// ctx は telemetry_task.cpp 側の実装オブジェクトへのポインタ (host テストでは
// フェイク実装を bind する)。
struct WifiOps {
  void* ctx = nullptr;
  // WiFi.persistent(false) + WiFi.setAutoReconnect(false) + WiFi.begin() を実行する
  // (start-class。呼び出し前に !WIFI_QUIET が確認済みであることが前提)。
  void (*begin)(void* ctx) = nullptr;
  // WiFi.disconnect() の戻り値 (abort-class)。
  bool (*disconnect)(void* ctx) = nullptr;
  // WiFi.mode(WIFI_OFF) 相当 (abort-class の終端。idempotent に複数回呼ばれ得る)。
  void (*setRadioOff)(void* ctx) = nullptr;
  // WiFi.status()==WL_NO_SHIELD 相当 (radio 停止の補助確認)。
  bool (*isRadioOff)(void* ctx) = nullptr;
  // WiFiUDP::beginPacket() の戻り値 (0 = 失敗)。
  int (*beginPacket)(void* ctx) = nullptr;
  // WiFiUDP::write() の戻り値 (書き込みバイト数)。
  int (*writePacket)(void* ctx, const uint8_t* buf, size_t len) = nullptr;
  // WiFiUDP::endPacket() の戻り値 (0 = 失敗)。
  int (*endPacket)(void* ctx) = nullptr;
};

class WifiGuard {
 public:
  // 自発的切断 (Voluntarily disconnected) の reason コード。
  // esp_wifi_types.h の WIFI_REASON_ASSOC_LEAVE と同値 (Arduino 非依存にするため
  // ヘッダを取り込まず数値を直接定義。telemetry_task.cpp から渡される reason は
  // このヘッダの esp_wifi_types.h 由来の値と比較可能)。
  static constexpr uint8_t kReasonAssocLeave = 8;

  enum class EventKind : uint8_t { GotIp, StaDisconnected, StaStop };

  enum class SendOutcome : uint8_t {
    NotConnected,
    BeginPacketFailed,
    WriteFailed,
    EndPacketFailed,
    Sent,
  };

  struct Params {
    // abort 発行後、完了確認 (post-abort epoch の ASSOC_LEAVE 切断 or STA_STOP) を
    // 待つ tick 数。計画書 §3.1: 目安 3 tick = 150ms (telemetry 50ms 周期前提)。
    uint32_t abort_confirm_ticks = 3;
    // 完了未確認時の有界リトライ回数上限 (計画書 §3.1: 最大3回)。
    int abort_max_retries = 3;
    // lib_reconnect_pending のフォールバック タイムアウト (計画書 §3.1: 目安15s)。
    // 50ms tick 換算で 300 tick。
    uint32_t lib_reconnect_timeout_ticks = 300;
    // 失敗した接続試行の後、次の begin() を許可するまでの猶予 (busy loop 防止)。
    // 計画書は具体的な秒数を規定していないため、保守側 (安全側) の既定値として
    // 3s (=60 tick @50ms) を採用する。TUNE。
    uint32_t reconnect_backoff_ticks = 60;
  };

  // Params にはデフォルト値を明示せず、呼び出し側に常に意図した設定値を渡させる
  // (Params は WifiGuard の入れ子型のため、既定引数値に Params{} を書くと
  // 「入れ子型の既定メンバ初期化子を、外側クラス定義完了前に参照する」ことになり
  // 一部コンパイラでエラーになる。呼び出し側は Params{} を明示的に渡せばよい)。
  WifiGuard(WifiOps ops, Params params) : ops_(ops), p_(params) {}

  // Wi-Fi イベントコールバックから呼ぶ。atomic な SPSC キューへの push のみを行い、
  // Wi-Fi API は一切呼ばない (イベントタスク内での再入を避ける。計画書 §3.1)。
  void pushEvent(EventKind kind, uint8_t reason = 0) {
    const uint32_t h = ev_head_.load(std::memory_order_relaxed);
    const uint32_t t = ev_tail_.load(std::memory_order_acquire);
    if (h - t >= kEventQueueLen) return;  // 満杯 → 破棄 (fail-closed 側の判定が遅れるだけ)
    event_q_[h % kEventQueueLen] = QueuedEvent{kind, reason};
    ev_head_.store(h + 1, std::memory_order_release);
  }

  // telemetry task から 20Hz で呼ぶ。read_ok/loop_count から fresh を再計算し、
  // WIFI_QUIET に基づいて begin/abort/radio-off を単一スレッドで進める。
  void tick(bool read_ok, uint32_t loop_count, bool balancing, bool arm_pending) {
    drainEvents();
    if (abort_failed_latched_) return;  // 全停止ラッチ後は Wi-Fi 活動を再開しない

    const bool fresh = freshness_.observe(read_ok, loop_count);
    const bool quiet = wifiQuiet(fresh, balancing, arm_pending);
    last_wifi_quiet_ = quiet;

    if (lib_reconnect_pending_) {
      if (++lib_pending_ticks_ >= p_.lib_reconnect_timeout_ticks) {
        lib_reconnect_pending_ = false;  // (c) タイムアウト フォールバック
      }
    }

    if (aborting_) {
      handleAbortTick();
      return;
    }
    if (radio_off_pending_) {
      handleRadioOffTick();
      return;
    }

    // CONNECTING 中の fail-closed は abort 方向、かつ lib_reconnect_pending
    // (ライブラリ one-shot 再接続 in-flight) も同一機構で中断する (計画書 §3.1)。
    if (quiet && (connecting_ || lib_reconnect_pending_)) {
      startAbort();
      return;
    }

    if (backoff_ticks_remaining_ > 0) --backoff_ticks_remaining_;

    // start-class: begin は !WIFI_QUIET の場合のみ発行する (初回も再接続も同一ルール)。
    if (!connecting_ && !connected_ && !quiet && backoff_ticks_remaining_ == 0) {
      issueBegin();
    }
  }

  // packet 送信 (prewarm 兼用)。readyToAttempt()==false の間は呼び出し側がスキップして
  // よい。beginPacket が失敗したら write には絶対に進まない (tx バッファ deref のため)。
  SendOutcome trySend(const uint8_t* buf, size_t len) {
    if (!connected_ || abort_failed_latched_) return SendOutcome::NotConnected;

    const int bp = ops_.beginPacket(ops_.ctx);
    if (bp == 0) {
      countSendFailure();
      return SendOutcome::BeginPacketFailed;
    }
    const int w = ops_.writePacket(ops_.ctx, buf, len);
    if (w < 0 || static_cast<size_t>(w) != len) {
      countSendFailure();
      return SendOutcome::WriteFailed;
    }
    const int ep = ops_.endPacket(ops_.ctx);
    if (ep == 0) {
      // Balancing 中の endPacket 失敗は udp_ready を維持したまま送信失敗として計上し、
      // socket 再作成 (再 prewarm) は !WIFI_QUIET まで保留する (計画書 §3.1)。
      countSendFailure();
      return SendOutcome::EndPacketFailed;
    }
    udp_ready_ = true;
    return SendOutcome::Sent;
  }

  // 呼び出し側 (telemetry_task.cpp) が「この tick で送信を試みるべきか」を判定する。
  // 未 ready の prewarm は !WIFI_QUIET の tick でのみ許可、ready 後は WIFI_QUIET でも
  // (Balancing 中でも) 確保済み資源の再利用として送信を継続する (計画書 §3.1)。
  bool readyToAttempt() const {
    if (!connected_) return false;
    return udp_ready_ || !last_wifi_quiet_;
  }

  bool connected() const { return connected_; }
  bool connecting() const { return connecting_; }
  bool udpReady() const { return udp_ready_; }
  bool libReconnectPending() const { return lib_reconnect_pending_; }
  bool aborting() const { return aborting_; }
  bool radioOffPending() const { return radio_off_pending_; }
  bool wifiAbortFailed() const { return abort_failed_latched_; }

  uint32_t prewarmFailTotal() const { return prewarm_fail_total_; }
  uint32_t sendFailTotal() const { return send_fail_total_; }
  uint32_t wifiAbortFailedTotal() const { return wifi_abort_failed_total_; }

 private:
  struct QueuedEvent {
    EventKind kind;
    uint8_t reason;
  };
  static constexpr uint32_t kEventQueueLen = 8;

  bool popEvent(QueuedEvent* out) {
    const uint32_t t = ev_tail_.load(std::memory_order_relaxed);
    const uint32_t h = ev_head_.load(std::memory_order_acquire);
    if (t == h) return false;
    *out = event_q_[t % kEventQueueLen];
    ev_tail_.store(t + 1, std::memory_order_release);
    return true;
  }

  void countSendFailure() {
    if (!udp_ready_) {
      ++prewarm_fail_total_;
    } else {
      ++send_fail_total_;
    }
  }

  // イベント駆動ガード (計画書 §3.1): epoch は drain 順に単調増加させる
  // (SPSC キューは単一消費者 = tick() 呼び出し元のみが drain するため、
  // ここは非 atomic なプレーン uint32_t でよい)。
  void drainEvents() {
    QueuedEvent ev;
    while (popEvent(&ev)) {
      ++epoch_;
      const uint32_t this_epoch = epoch_;
      switch (ev.kind) {
        case EventKind::GotIp:
          connected_ = true;
          connecting_ = false;
          lib_reconnect_pending_ = false;  // (a) one-shot 再接続の成功
          break;
        case EventKind::StaDisconnected: {
          const bool was_active = connecting_ || connected_;
          connected_ = false;
          connecting_ = false;
          if (was_active && !aborting_) {
            backoff_ticks_remaining_ = p_.reconnect_backoff_ticks;
          }
          if (ev.reason != kReasonAssocLeave) {
            if (!lib_first_connect_consumed_) {
              // ライブラリ側 first_connect one-shot が発火したことが確定 (非自発的
              // 切断の初回)。以降 autoReconnect は無効化済みのためこの one-shot は
              // 生涯で一度きり。
              lib_first_connect_consumed_ = true;
              lib_reconnect_pending_ = true;
              lib_pending_ticks_ = 0;
            } else if (lib_reconnect_pending_) {
              // (b) one-shot begin 後の再度の STA_DISCONNECTED = one-shot 失敗。
              // first_connect burnt + autoReconnect off により以降の再接続は
              // 発生しないため in-flight 終了とみなす。
              lib_reconnect_pending_ = false;
            }
          }
          if (aborting_ && ev.reason == kReasonAssocLeave &&
              this_epoch > abort_epoch_) {
            completeAbort();
          }
          break;
        }
        case EventKind::StaStop:
          connected_ = false;
          connecting_ = false;
          if (aborting_ && this_epoch > abort_epoch_) completeAbort();
          if (radio_off_pending_ && this_epoch > radio_off_epoch_) completeRadioOff();
          break;
      }
    }
  }

  void issueBegin() {
    ops_.begin(ops_.ctx);
    connecting_ = true;
    lib_reconnect_pending_ = false;  // (d) 容認された begin 発行時にクリア
  }

  void startAbort() {
    aborting_ = true;
    abort_epoch_ = epoch_;
    abort_ticks_waited_ = 0;
    abort_retry_count_ = 0;
    abort_issue_ok_ = ops_.disconnect(ops_.ctx);
  }

  void handleAbortTick() {
    ++abort_ticks_waited_;
    const bool need_retry =
        !abort_issue_ok_ || abort_ticks_waited_ >= p_.abort_confirm_ticks;
    if (!need_retry) return;
    if (abort_retry_count_ >= p_.abort_max_retries) {
      aborting_ = false;
      startRadioOff();
      return;
    }
    ++abort_retry_count_;
    abort_ticks_waited_ = 0;
    // abort_epoch_ は最初の発行時点を維持する (retry の度に更新すると、retry と
    // 競合したちょうどのタイミングの確認イベントを取りこぼす恐れがあるため)。
    abort_issue_ok_ = ops_.disconnect(ops_.ctx);
  }

  void completeAbort() {
    aborting_ = false;
    connecting_ = false;
    lib_reconnect_pending_ = false;  // (e) cancel-watchdog による abort 完了確認時
    backoff_ticks_remaining_ = p_.reconnect_backoff_ticks;
  }

  void startRadioOff() {
    radio_off_pending_ = true;
    radio_off_epoch_ = epoch_;
    ops_.setRadioOff(ops_.ctx);
  }

  void handleRadioOffTick() {
    if (ops_.isRadioOff(ops_.ctx)) {
      completeRadioOff();
      return;
    }
    ops_.setRadioOff(ops_.ctx);  // 確認まで idempotent に再発行
  }

  void completeRadioOff() {
    radio_off_pending_ = false;
    abort_failed_latched_ = true;
    ++wifi_abort_failed_total_;
    connecting_ = false;
    connected_ = false;
    lib_reconnect_pending_ = false;
    udp_ready_ = false;
  }

  WifiOps ops_;
  Params p_;

  FreshnessTracker freshness_;
  bool last_wifi_quiet_ = true;

  // イベントキュー (producer: Wi-Fi イベントコールバック、consumer: tick())
  QueuedEvent event_q_[kEventQueueLen] = {};
  std::atomic<uint32_t> ev_head_{0};
  std::atomic<uint32_t> ev_tail_{0};
  uint32_t epoch_ = 0;

  // 接続状態
  bool connecting_ = false;
  bool connected_ = false;
  bool udp_ready_ = false;
  uint32_t backoff_ticks_remaining_ = 0;

  // lib_reconnect_pending ライフサイクル
  bool lib_first_connect_consumed_ = false;
  bool lib_reconnect_pending_ = false;
  uint32_t lib_pending_ticks_ = 0;

  // abort 状態機械
  bool aborting_ = false;
  uint32_t abort_epoch_ = 0;
  uint32_t abort_ticks_waited_ = 0;
  int abort_retry_count_ = 0;
  bool abort_issue_ok_ = true;

  // radio-off (終端) 状態機械
  bool radio_off_pending_ = false;
  uint32_t radio_off_epoch_ = 0;
  bool abort_failed_latched_ = false;

  // 観測用カウンタ
  uint32_t prewarm_fail_total_ = 0;
  uint32_t send_fail_total_ = 0;
  uint32_t wifi_abort_failed_total_ = 0;
};

}  // namespace core
