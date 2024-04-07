// Copyright (c) Shinya Ishikawa. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full
// license information.
// 
// Modified for tairin-chan, by B-SKY Lab (Yutaro KIMURA)

#include "TairinEye.h"

namespace m5avatar {

bool en_draw_flower = false;  // If drawn only once every two times, the lines will be doubled.

tairinEye::tairinEye(uint16_t x, uint16_t y, uint16_t r, bool isLeft) : tairinEye(r, isLeft) {}

tairinEye::tairinEye(uint16_t r, bool isLeft) : r{r}, isLeft{isLeft} {}

void tairinEye::draw(M5Canvas *spi, BoundingRect rect, DrawContext *ctx) {
  Expression exp = ctx->getExpression();
  uint32_t x = rect.getCenterX();
  uint32_t y = rect.getCenterY();
  Gaze g = ctx->getGaze();
  float openRatio = ctx->getEyeOpenRatio();
  uint32_t offsetX = g.getHorizontal() * 3;
  uint32_t offsetY = g.getVertical() * 3;
  uint16_t primaryColor = ctx->getColorDepth() == 1 ? 1 : ctx->getColorPalette()->get(COLOR_PRIMARY);
  uint16_t backgroundColor = ctx->getColorDepth() == 1 ? 0 : ctx->getColorPalette()->get(COLOR_BACKGROUND);

  // Draw eyes
  if (openRatio > 0) {
    spi->fillCircle(x + offsetX, y + offsetY, r, primaryColor);   // opened
  } else {
    int x1 = x - r + offsetX;
    int y1 = y - 2 + offsetY;
    int w = r * 2;
    int h = 4;
    spi->fillRect(x1, y1, w, h, primaryColor);    // closed
  }

  // Draw flower
  if(en_draw_flower){
    for (double theta=0; theta<=2*PI; theta += 0.01) {
      double n = 5; // The number of petals is twice n.
      double a = 30;
      double b = 6;
      double c = 1;
      double r = abs(a*sin(n*theta) + b*sin(3*n*theta) + c*sin(5*n*theta));

      // Convert to Cartesian coordinate system
      double x_bias = 275;   //275 is value for core2, x_bias is static
      double y_bias = y-55;  //-55 is value for core2, y_bias is dynamic (for animation)
      double x_flower = r * cos(theta) + x_bias;
      double y_flower = r * sin(theta) + y_bias;
      spi->drawPixel(x_flower, y_flower, primaryColor);
    }
  }
  en_draw_flower = !en_draw_flower;

}
}  // namespace m5avatar
