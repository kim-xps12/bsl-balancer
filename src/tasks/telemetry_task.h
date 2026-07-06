// telemetry_task.h — UDP telemetry タスク (計画書 §3.1)。core1 / priority1 / stack8192。
// secrets (include/wifi_secrets.h) の有無によらず常に同一の安定 API を提供する。
// __has_include("wifi_secrets.h") の分岐は telemetry_task.cpp 内部に閉じ込め、
// 不在時は startTelemetryTask() が no-op になる (呼び出し側に #ifdef を置かない)。
#pragma once

#include "../shared/shared_state.h"

namespace tasks {

// secrets 有効時: telemetry task を core1 / priority1 / stack8192 で生成する。
// secrets 不在時: no-op (task を生成しない)。
void startTelemetryTask(shared::SharedState& shared_state);

// telemetry task が実際に有効 (secrets が見つかりビルドに組み込まれた) かどうか。
bool telemetryEnabled();

}  // namespace tasks
