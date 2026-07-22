// Copyright (c) Shinya Ishikawa. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full
// license information.
// 
// Modified for tairin-chan, by B-SKY Lab (Yutaro KIMURA)

#ifndef TAIRIN_EYE_H_
#define TAIRIN_EYE_H_

#define LGFX_USE_V1
#include <M5GFX.h>
#include "DrawContext.h"
#include "Drawable.h"

namespace m5avatar {

class tairinEye final : public Drawable {
 private:
  uint16_t r;
  bool isLeft;

 public:
  // constructor
  tairinEye() = delete;
  tairinEye(uint16_t x, uint16_t y, uint16_t r, bool isLeft);  // deprecated
  tairinEye(uint16_t r, bool isLeft);
  ~tairinEye() = default;
  tairinEye(const tairinEye &other) = default;
  tairinEye &operator=(const tairinEye &other) = default;
  void draw(M5Canvas *spi, BoundingRect rect,
            DrawContext *drawContext) override;
};

}  // namespace m5avatar

#endif
