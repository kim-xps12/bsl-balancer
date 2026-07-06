// dxl_verify.h — DYNAMIXEL Status 応答の配達証明判定 (設計書 §4.2)。
// Arduino 非依存 (native テスト可能)。生 I/O (txInstPacket/rxStatusPacket) と
// param 組み立ては呼び出し側 (hw/dxl_backend.cpp) に残し、受理判定のみを
// 抽出する (段階1a。docs/plans/2026-07-06-safety-core-extraction.md §3.1)。
#pragma once

#include <cstdint>

namespace core {

// Status 応答の観測値ビュー (ゲート1第1回指摘3対応 — rx==nullptr 時に呼び出し
// 側でフィールドを deref させない。ライブラリ型 InfoToParseDXLPacket_t は
// Arduino 依存のため core には持ち込まず、POD ビューに写して渡す)。
struct DxlStatusView {
  bool ok = false;          // rxStatusPacket() が nullptr でない
  uint8_t id = 0;
  uint8_t err_idx = 0;
  uint16_t recv_param_len = 0;
};

// WRITE の Status 受理判定 (設計書 §4.2)。現行 dxl_backend.cpp:52-57 と逐語
// 対応:
//   !st.ok → 拒否 / id 不一致 → 拒否 / err_idx != 0 → 拒否 (ALERT 0x80 含む)
//   / recv_param_len != 0 → 拒否 (前回 READ の遅延応答を配達証明として誤受理
//   しない)
// 限界 (§8 残余リスク): 同一 ID・err=0・recv_param_len=0 の遅延 WRITE Status
// は本述語単体では判別不能で受理される (drainRx は tx 前の残留バイトのみ除去
// し、tx 後に到着する遅延応答は防げない)。段階5 の transport タイムライン
// テストでクローズする。
inline bool isWriteStatusVerified(const DxlStatusView& st, uint8_t expect_id) {
  if (!st.ok) return false;
  if (st.id != expect_id) return false;
  if (st.err_idx != 0) return false;
  if (st.recv_param_len != 0) return false;
  return true;
}

// READ の Status 受理判定。現行 dxl_backend.cpp:71-76 と逐語対応:
//   !st.ok / id 不一致 / recv_param_len != expect_len → 拒否
//   / (err_idx & 0x7F) != 0 → 拒否 (READ は ALERT ビットのみ許容 — HW エラー
//   通知はヘルス側で扱う。Result Fail 等は拒否)
inline bool isReadStatusVerified(const DxlStatusView& st, uint8_t expect_id,
                                 uint16_t expect_len) {
  if (!st.ok) return false;
  if (st.id != expect_id) return false;
  if (st.recv_param_len != expect_len) return false;
  if ((st.err_idx & 0x7F) != 0) return false;
  return true;
}

}  // namespace core
