// ui_task.h — UI タスク (設計書 §3.1/§9)。core1 / 優先度 3 / ~33ms。
// M5.update()・BtnC 長押し STOP/ARM・チューニングパネル・avatar 表情・NVS 書込実行。
#pragma once

#include <Avatar.h>

#include "../shared/shared_state.h"

namespace tasks {

struct UiContext {
  shared::SharedState* shared;
  m5avatar::Avatar* avatar;
};

// FreeRTOS タスクエントリ (pvParameters = UiContext*)
void uiTaskEntry(void* pvParameters);

}  // namespace tasks
