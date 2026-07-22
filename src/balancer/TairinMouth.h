// Copyright (c) Shinya Ishikawa. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full
// license information.
// 
// Modified for tairin-chan, by B-SKY Lab (Yutaro KIMURA)

#ifndef MYMOUTH_H_
#define MYMOUTH_H_

#include <M5GFX.h>
#include "BoundingRect.h"
#include "DrawContext.h"
#include "Drawable.h"

namespace m5avatar {
class tairinMouth final : public Drawable {
 private:
  uint16_t minWidth;
  uint16_t maxWidth;
  uint16_t minHeight;
  uint16_t maxHeight;

 public:
  // constructor
  tairinMouth() = delete;
  ~tairinMouth() = default;
  tairinMouth(const tairinMouth &other) = default;
  tairinMouth &operator=(const tairinMouth &other) = default;
  tairinMouth(uint16_t minWidth, uint16_t maxWidth, uint16_t minHeight,
        uint16_t maxHeight);
  void draw(M5Canvas *spi, BoundingRect rect,
            DrawContext *drawContext) override;
};

}  // namespace m5avatar

#endif  // MYMOUTH_H_
