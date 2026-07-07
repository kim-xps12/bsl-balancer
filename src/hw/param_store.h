// param_store.h — NVS (Preferences) レコード入出力 (設計書 §9.1)
// 検証は core/param_validation.h の純関数。書込は UI タスクが保存ゲート下で実行。
#pragma once

#include "../core/param_validation.h"

namespace hw {

class ParamStore {
 public:
  // 読込 (INITIALIZING 内・Torque ON 前)。無効レコードは既定値のまま false。
  static bool loadParams(core::TuningParams* out);

  // 書込 (検証済み safe-off 下でのみ呼ぶこと — 呼び出し側の責務)
  static bool saveParams(const core::TuningParams& rec);
};

}  // namespace hw
