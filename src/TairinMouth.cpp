// Copyright (c) Shinya Ishikawa. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full
// license information.
// 
// Modified for tairin-chan, by B-SKY Lab (Yutaro KIMURA)

#include "TairinMouth.h"

namespace m5avatar {

tairinMouth::tairinMouth(uint16_t minWidth, uint16_t maxWidth, uint16_t minHeight,
             uint16_t maxHeight)
    : minWidth{minWidth},
      maxWidth{maxWidth},
      minHeight{minHeight},
      maxHeight{maxHeight} {}

void tairinMouth::draw(M5Canvas *spi, BoundingRect rect, DrawContext *ctx) {
  uint16_t primaryColor = ctx->getColorDepth() == 1 ? 1 : ctx->getColorPalette()->get(COLOR_PRIMARY);
  float breath = _min(1.0f, ctx->getBreath());
  float openRatio = ctx->getMouthOpenRatio();

  int h_lip = minHeight + (maxHeight - minHeight) * openRatio;
  int w_lip = minWidth + (maxWidth - minWidth) * (1 - openRatio);
  int x_lip = rect.getLeft() - w_lip / 2;
  int y_lip = rect.getTop()  - h_lip / 2 + breath * 2;
  spi->fillRect(x_lip, y_lip, w_lip, h_lip, primaryColor);

  uint32_t x = rect.getCenterX();
  uint32_t y = rect.getCenterY();
  uint32_t r = 25;
  uint32_t offsetX = 100; //w_lip /2;
  uint32_t offsetY = 20;//-(r-1) - breath*2;
  
  spi->drawCircle(x + offsetX, y + offsetY, r, primaryColor);
  spi->drawCircle(x + offsetX+1, y + offsetY, r, primaryColor);

  spi->drawCircle(x - offsetX, y + offsetY, r, primaryColor);
  spi->drawCircle(x - offsetX-1, y + offsetY, r, primaryColor);
}

}  // namespace m5avatar
