# 安全機構の純関数抽出 + dt p95 計算コア（監査残課題 段階1〜3）

作成日: 2026-07-06
ブランチ: `feat/safety-core-extraction`（`dev/current-mode-xl330-fable5` から独立分岐。PR は同ブランチへ）
出典: `docs/reviews/2026-07-04-best-practice-audit.md`（監査 HIGH「安全機構の mock テスト未実装」・MEDIUM「dt p95 soak gate 未実装」）
ユーザ承認: 2026-07-06 に段階1〜3 の実装を承認済み（段階4〜5 = control_task 周期本体の関数抽出・p95 ゲート結線・transport 抽象化は本 PR の非スコープ）。

## 1. 目的・スコープ

設計書（`docs/plans/2026-07-02-current-mode-freertos-redesign.md` §4.2/§4.3/§6）が安全論拠の
根幹として要求しながら host テスト空白になっている判断ロジックを、**ロジック変更ゼロの
純関数抽出**で `src/core/`（Arduino 非依存・native テスト可能層）へ移し、監査 5 テストの
うち 2 本（配達証明の誤受理・Watchdog 遷移表の左右非対称）の**判断ロジック部分**を
テスト可能化する（I/O 実行列を含む完全クローズは段階5 の transport fake が担う —
下表・§8 参照）。あわせて dt p95 soak gate（設計書 §6）の**計算コア**を新設する
（FSM への結線は段階4）。

| 段階 | 内容 | 対応する監査テスト |
|---|---|---|
| 1a | 配達証明の検証述語を `src/core/dxl_verify.h` へ抽出 | テスト1「配達証明の誤受理」の**述語部分の特性化**（完全クローズは段階5、§8参照） |
| 1b | Bus Watchdog 遷移表の判定 + 復旧頻度制限を `src/core/watchdog_policy.h` へ抽出 | テスト4「遷移表の左右非対称」（判定・順序契約。I/O 実行列の検証は段階5） |
| 2 | `PlausibilityMonitor` を `src/core/plausibility_monitor.h` へ移設（逐語移動） | 既存妥当性監視の恒久テスト化 |
| 3 | dt p95 計算コア `src/core/dt_stats.h` を新設 | dt p95 soak gate の土台（結線は段階4） |

**非スコープ（明示）**: hw 層 I/O の transport 抽象化（段階5）、`checkWatchdog` の I/O
実行部・`watchdogRecoverOne` の I/O 列の検証、未検証トルク窓/検疫沈黙/保存レースの
統合テスト（段階4〜5）、INITIALIZING への p95 ゲート結線・`FaultReason` 追加・予算閾値
定数（段階4）、制御則・安全機構の**挙動変更一切**。

## 2. 設計原則

1. **ロジック変更ゼロ**: 抽出は「判断式の移動 + 呼び出し置換」のみ。判定条件・比較
   演算子・境界値・状態更新順序を一切変えない（§3 に現行コードとの対応を明記）。
2. **リリース挙動同一**: バイナリの byte 同一は主張しない（inline 境界・シンボルが
   動くため）。保証するのは挙動同一性 = 抽出前後で全既存テストパス + 判定式の
   逐語対応（レビューで対査）。
3. **既存パターン踏襲**: `src/core/` の既存ヘッダ（`pid.h`/`safety_fsm.h` 等）と同じ
   header-only・`namespace core`・cfg 定数参照スタイル。
4. **テストは境界値と非対称ケースを網羅**: 特に Watchdog は左右の全非対称組合せを
   遷移表としてテストする（§5）。

## 3. 抽出内容（現行コードとの対応）

### 3.1 段階1a: `src/core/dxl_verify.h`（新規）

現行 `DxlWithRxInfo::writeVerified`（`src/hw/dxl_backend.cpp:40-59`）/
`readVerified`（同 :61-79`）の**受理判定部分**を抽出する。I/O（txInstPacket/
rxStatusPacket）と param 組み立ては hw 層に残す。

```cpp
namespace core {
// Status 応答の観測値ビュー (ゲート1第1回指摘3対応 — rx==nullptr 時に呼び出し側で
// フィールドを deref させない。ライブラリ型 InfoToParseDXLPacket_t は Arduino 依存の
// ため core には持ち込まず、POD ビューに写して渡す):
struct DxlStatusView {
  bool ok = false;          // rxStatusPacket() が nullptr でない
  uint8_t id = 0;
  uint8_t err_idx = 0;
  uint16_t recv_param_len = 0;
};
// WRITE の Status 受理判定 (設計書 §4.2)。現行 dxl_backend.cpp:52-57 と逐語対応:
//   !st.ok → 拒否 / id 不一致 → 拒否 / err_idx != 0 → 拒否 (ALERT 0x80 含む)
//   / recv_param_len != 0 → 拒否 (前回 READ の遅延応答を配達証明として誤受理しない)
inline bool isWriteStatusVerified(const DxlStatusView& st, uint8_t expect_id);
// READ の Status 受理判定。現行 dxl_backend.cpp:71-76 と逐語対応:
//   !st.ok / id 不一致 / recv_param_len != expect_len → 拒否
//   / (err_idx & 0x7F) != 0 → 拒否 (READ は ALERT ビットのみ許容 — HW エラー通知は
//   ヘルス側で扱う。Result Fail 等は拒否)
inline bool isReadStatusVerified(const DxlStatusView& st, uint8_t expect_id,
                                 uint16_t expect_len);
}
```

`dxl_backend.cpp` 側は rx 受信後の if 連鎖を「view 構築 + 述語 1 呼び出し」に置換する。
**view 構築の必須パターン**（nullptr 安全性の call-site 契約。ゲート1第1回指摘3対応 —
引数評価は関数呼び出し前に行われるため、`rx->id` を引数式に書く形は禁止）:

```cpp
const auto* rx = rxStatusPacket(...);
core::DxlStatusView st;
if (rx != nullptr) { st.ok = true; st.id = rx->id; st.err_idx = rx->err_idx;
                     st.recv_param_len = rx->recv_param_len; }
if (!core::isWriteStatusVerified(st, id)) return false;
```

この call-site パターン（nullptr 時にフィールドへ一切触れない）はゲート2レビューでの
対査項目とし、計画書のこの節を対査基準にする。データコピーは READ 成功時のみ実行
（現行と同順序）。

### 3.2 段階1b: `src/core/watchdog_policy.h`（新規）

現行 `DxlBackend::checkWatchdog`（`src/hw/dxl_backend.cpp:371-403`）は I/O と判断が
混在。判断のみを 2 部品として抽出する:

**API 形状は逐次リデューサ**（ゲート1第1回指摘2対応 — 現行 `checkWatchdog` は
「rawL 読取失敗で**即** return（rawR を読まない）」「teL 読取失敗で**即** return
（teR を読まない）」という早期 return を持ち、『両輪分を読んでから一括判定』の
API では I/O 回数・順序の同一性が破れる。劣化経路でのバス往復・タイムアウト遅延の
増加は安全上許容しない）:

```cpp
namespace core {
enum class WatchdogVerdict : uint8_t { Pending, Ok, Fault, ProceedRecover };
// 現行 checkWatchdog の判断列を I/O と同じ逐次順で消費するリデューサ。
// call-site 契約: 読取 I/O は現行と同一順 (raw[L]→raw[R]→(トリップ時のみ)
// te[L]→te[R]) で行い、各 feed が Pending 以外を返した時点で以降の I/O を
// 行わない (早期 return の I/O 回数同一性)。逐語対応は現行 :374-388:
//   feedRaw: 読取失敗 → torque_may_be_on ? Fault : Ok (即時) /
//            両輪 feed 完了かつ非トリップ (いずれも != tripped_value) → Ok /
//            トリップあり → Pending (TE 読取へ)
//   feedTe:  読取失敗 → Fault (即時) / 両輪 feed 完了かつ両輪 0 → ProceedRecover /
//            いずれか非零 → Fault (注: 現行 :388 は両輪読取後に te[0]!=0||te[1]!=0 を
//            判定するため、teL 非零でも teR の読取は行われる。この I/O 回数も保存
//            する — 非零検出は両輪 feed 完了時に行う)
class WatchdogDecision {
 public:
  explicit WatchdogDecision(bool torque_may_be_on, uint8_t tripped_value);
  WatchdogVerdict feedRaw(bool read_ok, uint8_t value);  // L→R の順で最大 2 回
  WatchdogVerdict feedTe(bool read_ok, uint8_t value);   // L→R の順で最大 2 回
};
// 復旧頻度制限 (60s 内 3 回で FAULT)。現行 :390-397 の recover_count_/
// recover_window_start_s_ 状態機械を逐語移動 (リセット条件 count==0 ||
// (now - start) > window、++count >= max で deny):
class WatchdogRecoverLimiter {
 public:
  // 復旧を許可するなら true。deny (= FAULT) 時も現行同様 count は増加済みのまま。
  bool allow(float now_s, float window_s, int max_count);
 private:
  int recover_count_ = 0;
  float recover_window_start_s_ = 0.0f;
};
}
```

`checkWatchdog` 側は「raw 読取ループ内で feedRaw、Pending 以外なら即 return 変換 →
トリップ時のみ TE 読取ループ内で feedTe → ProceedRecover なら `limiter_.allow(...)` →
許可時のみ `watchdogRecoverOne` ×2」に再構成（I/O の位置・回数・順序は現行と同一。
`WatchdogCheck` への写像: Ok→Ok / Fault→Fault / ProceedRecover+allow 拒否→Fault /
復旧実行失敗→Fault / 成功→Recovered — 現行 :397-402 と同一）。
`DxlBackend` の生メンバ `recover_count_`/`recover_window_start_s_` は
`core::WatchdogRecoverLimiter limiter_` に置き換える（状態の意味・遷移は同一）。
**限界の明示**: リデューサの step 意味論（どの feed で確定するか）は native テストで
固定するが、**hw 層 call-site が契約どおり feed するかの機械的検証は段階5
（transport fake）まで残る**（それまでは本節をゲート2対査基準とするレビュー担保）。
**現行挙動の潜在エッジ（ゲート1第2回指摘で発見・本 PR では挙動保存）**: 現行
:374-378 は「トリップ済み raw 値を観測した後に他輪の raw 読取が失敗」しても、
torque_may_be_on==false なら Ok を返す（既知のトリップ証拠が破棄され、復旧も
FAULT も発生しない）。リデューサはこの挙動を**そのまま保存**する（feedRaw の
読取失敗判定は先行 raw 値に依存しない）。これはロジック変更ゼロ原則（§2-1）に
よる意図的判断であり、§5-3 の特性化テストで現行挙動を固定し、§8 に残余リスク
として記録の上、**挙動修正（トリップ証拠観測後の読取失敗を fail-closed 化）は
別課題としてユーザへエスカレーション**する（安全挙動の変更は本 PR のスコープ外）。

### 3.3 段階2: `src/core/plausibility_monitor.h`（新規・逐語移動）

現行 `control_task.cpp:18-33` の `PlausibilityMonitor` は既に完全に純粋
（I/O なし・内部状態は `dwell_` のみ）。置き場所だけが native テストの障害のため、
クラス本体を**逐語移動**（`namespace core`、cfg 定数参照は現行のまま）。
`control_task.cpp` はローカル定義を削除し `core::PlausibilityMonitor` を使用。

### 3.4 段階3: `src/core/dt_stats.h`（新規）

設計書 §6「トルク OFF で全経路を ~2s 回して dt p95 が予算内であること」の計算コア。

```cpp
namespace core {
// 固定容量 (kCapacity=512 > 200Hz×2s=400) の dt サンプル窓と p95 抽出。
// heap 不使用 (float 512 本 = 2048B は使用側が静的に確保する想定。結線は段階4)。
// API は fail-closed (ゲート1第1回指摘4対応 — 空窓/不足窓/溢れ窓が「予算内」に
// 見える構成を型レベルで作らせない):
class DtP95Window {
 public:
  static constexpr size_t kCapacity = 512;
  void add(float dt_s);           // 容量超過時は記録せず overflowed_ を立てる
  size_t size() const;
  bool overflowed() const;
  // 窓がゲート判定に使える状態か (サンプル数下限を満たし、溢れていない)。
  // 段階4 のゲートは ready() && p95(&v) && v <= budget の形でのみ合格し得る。
  bool ready(size_t min_samples) const;   // = !overflowed_ && size_ >= min_samples
  // nearest-rank 法: idx = ceil(0.95*n) - 1。n==0 は false を返し *out 不変
  // (空窓を数値として扱わせない)。作業配列を内部コピーして選択、入力順は非破壊。
  bool p95(float* out) const;
  void reset();
};
}
```

p95 定義（nearest-rank、`idx = ceil(0.95*n) - 1`）を採用する理由: 標本補間なしで
「n サンプル中 95% がこの値以下」を厳密に主張でき、しきい値ゲート（段階4）の
判定文として明確。選択は `std::nth_element`（`<algorithm>`、heap 不使用）を用いる。
代替案（既存 8bin ヒストグラム流用）は分解能が bin 境界に量子化されるため不採用
（起動時 1 回・2KB 一時領域のコストは無視できる。監査調査レポートの比較表参照）。
既存 `dt_histogram.h`（telemetry 用 monotonic カウンタ）とは目的が異なるため
統合しない（コメントで相互参照）。

## 4. 変更ファイル一覧

| ファイル | 変更 | リリース挙動 |
|---|---|---|
| `src/core/dxl_verify.h` | 新規（DxlStatusView + 述語 2 関数） | 同一（式の移動のみ。call-site の nullptr ガードパターンは §3.1 が対査基準） |
| `src/core/watchdog_policy.h` | 新規（WatchdogDecision リデューサ + 頻度制限クラス） | 同一（I/O 位置・回数・順序を保存。§3.2 が対査基準） |
| `src/core/plausibility_monitor.h` | 新規（逐語移動） | 同一 |
| `src/core/dt_stats.h` | 新規（p95 計算コア） | 影響なし（本 PR では未結線・未使用。ヘッダ追加のみ） |
| `src/hw/dxl_backend.cpp` | 述語/判定/制限の呼び出し置換 | 同一 |
| `src/hw/dxl_backend.h` | `recover_count_`/`recover_window_start_s_` → `core::WatchdogRecoverLimiter limiter_` | 同一 |
| `src/tasks/control_task.cpp` | ローカル `PlausibilityMonitor` 削除 → `core::` 版使用 | 同一 |
| `test/test_core/test_main.cpp` | 新規テスト群（§5） | — |

## 5. テスト計画（native、追加分）

1. **配達証明 WRITE**（`isWriteStatusVerified`）: 受理 1 + 拒否 4 軸（st.ok=false
   （= rx nullptr の view、他フィールドは既定値のまま）/ ID 不一致 /
   err_idx=0x80(ALERT) / err_idx=0x01 / recv_param_len≠0 = 遅延 READ 応答の
   誤受理拒否）。**述語の限界の文書化テスト**: 同一 ID・err=0・param_len=0 の
   遅延 WRITE 応答は述語単体では判別不能で受理される（現行実装と同一の挙動）
   ことを明示的に assert し、テストコメントで §8 の残余リスク（段階5 で
   transport タイムラインテストによりクローズ）へ参照を張る。
2. **配達証明 READ**（`isReadStatusVerified`）: 受理（err=0）+ **ALERT のみ (0x80) は
   受理**（現行仕様: READ の ALERT はデータ有効）+ 拒否 4 軸（rx 失敗 / ID 不一致 /
   len 不一致 / Result Fail (err & 0x7F ≠ 0、0x81 も拒否)）。
3. **Watchdog 遷移表**（`WatchdogDecision` リデューサ）: 左右非対称の全ケースを
   **確定タイミング（どの feed で Pending 以外になるか）込み**で検証 —
   rawL 読取失敗 → 1 回目の feedRaw で即確定（torque_may_be_on {true→Fault,
   false→Ok}。rawR を feed しない = I/O 打切り位置の固定）、rawR 読取失敗 →
   2 回目で即確定、両輪非トリップ → 2 回目の feedRaw で Ok、トリップ {L のみ,
   R のみ, 両輪} → 2 回目の feedRaw で Pending（TE 段階へ）、teL 読取失敗 →
   1 回目の feedTe で即 Fault（teR を feed しない）、teR 読取失敗 → 2 回目で
   Fault、**TE 値の全組合せを確定タイミング込みで固定**（ゲート1第3回指摘対応 —
   現行 `te[0] != 0 || te[1] != 0` の OR 意味論が `&&` へ写し間違えられても検出
   できるように）: (teL,teR) = **(1,0) → Fault**・**(0,1) → Fault**・(1,1) → Fault・
   (0,0) → ProceedRecover、いずれも **2 回目の feedTe 完了時**に確定（1 回目の
   feedTe は teL=1 でも Pending を返す = teL 非零でも teR 読取まで行う現行 I/O
   順序契約を同じテストで固定）。**現行挙動の特性化（§3.2 潜在エッジ）**: rawL=0xFF（トリップ）→
   rawR 読取失敗 の系列が torque_may_be_on=false で Ok / true で Fault となる
   （先行トリップ証拠が結果に影響しない）ことを明示 assert し、テストコメントで
   §8 の残余リスク・エスカレーション方針へ参照を張る。
4. **復旧頻度制限**（`WatchdogRecoverLimiter`）: 窓内 2 回目まで allow / 3 回目 deny /
   窓経過（> 比較の境界: ちょうど window_s は同一窓、window_s+ε で新窓）で
   リセット / deny 後も窓内は deny 継続。
5. **PlausibilityMonitor**: fb_valid=false で不判定・dwell 非蓄積 / torque_on 乖離の
   dwell 蓄積→ 閾値超で true（> 比較の境界確認）/ 正常戻りで dwell リセット /
   torque_off 残留電流経路 / reset()。
6. **DtP95Window**: n=1 / n=20 の既知分布（nearest-rank の期待 idx を手計算で固定）/
   未ソート入力 / 重複値 / n=400（2s 相当）/ 容量 512 超過で overflowed かつ既存
   サンプル保持 / reset / p95() が入力順を破壊しない（呼出し前後で add 継続可能）/
   **fail-closed 検証**: n=0 で p95() が false（*out 不変）/ ready(400) が
   n=399 で false・n=400 で true / overflowed 時 ready が常に false /
   「ready && p95 && v<=budget」合成ゲートが空窓・不足窓・溢れ窓で合格し得ない
   ことのテーブルテスト。

既存テスト全パス（回帰なし）。抽出前後の等価性は「既存の関連テスト（FSM・
watchdog 関連の統合的な期待値があれば）が無修正でパスすること」+ 判定式の逐語
レビューで担保する。

## 6. 検証（PR 前に必須）

1. `pio test -e native` 全パス（既存 32 本 + 新規。pio は `~/.platformio/penv/bin/pio`）
2. `pio run -e m5stack-core2` 成功（リリース経路のコンパイル確認）

（注: `scripts/verify_builds.sh`・trace env・wifi_secrets 分岐は PR #27/#28 系列にのみ
存在し、dev 起点の本ブランチには無いことを 2026-07-06 に確認済み。本ブランチの
検証は上記 2 点で完結する。#27 と合流後は verify_builds.sh が両系列の変更を包含して
検証する。）

## 7. Git 運用

- コミットは日本語・1 目的 1 コミット・`Co-Authored-By` トレーラ。
- PR base = `dev/current-mode-xl330-fable5`（マージはユーザ判断）。
- **PR #27 との合流時の競合見込み**: `control_task.cpp` を両系列が編集するが、領域が
  異なる（本 PR は :18-33 のクラス削除、#27 は LoopState/publish 部の追加）ため
  自明に解消可能な見込み。PR 本文に注記する。
- `docs/reviews/`・`prompt_memo.md`・`claude_best_fable.sh`・handover 文書は
  コミットしない。

## 8. リスクと限界

| リスク | 対応 |
|---|---|
| 抽出時の判定式の写し間違い（等価性の破れ） | §3 の逐語対応表 + 境界値テスト（§5）+ ゲート2 レビューで対査。比較演算子（`>` vs `>=`）と評価順序をテストで固定 |
| `WatchdogRecoverLimiter` の状態移動でタイミング意味が変わる | 現行のリセット条件・インクリメント位置・deny 後の状態を逐語移動し、§5-4 で deny 継続まで検証 |
| dt_stats.h が未使用ヘッダとしてリリースに影響 | どの TU からも include されない（テストのみが include）。段階4 で結線するまで dead file であることを PR 本文に明記 |
| 段階1〜3 では監査テスト 2/5 本のみ（残 3 本は未カバー） | 意図的な段階分割（ユーザ承認済み）。残りは段階4〜5 の別 PR |
| **トリップ観測後の他輪 raw 読取失敗が torque-off 経路で Ok になる**（現行 :374-378 の早期 return 意味論。既知トリップ証拠の破棄） | 現行実装と同一の残余リスク（本 PR は挙動を変えない）。§5-3 の特性化テストで固定し、fail-closed 化（例: トリップ証拠観測後の読取失敗 → Fault）は**別課題としてユーザへエスカレーション**（安全挙動変更のためユーザ判断 + 独立レビューが必要） |
| **同一 ID・err=0・param_len=0 の遅延 WRITE Status は述語では判別不能**（drainRx は tx 前の残留バイトのみ除去し、tx 後に到着する遅延応答は防げない） | 現行実装と同一の残余リスク（本 PR は挙動を変えない）。§5-1 で述語の限界を明示テスト化し、段階5 の transport タイムラインテスト（同一 ID 遅延 OK 応答の注入）でクローズする。監査テスト1の「完全クローズ」は段階5 完了まで主張しない（§1 表の注記） |
