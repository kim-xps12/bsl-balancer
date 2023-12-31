# BSL-Balancer

**front**
![front](./front.png)

**back**
![back](./back.png)


## BOM

|     |     |     |     |
| --- | --- | --- | --- |
| Item | Model | Qty. | Link |
| MCU | M5Stack Core2<br> | 1 pc. | switch-science, sengoku |
| Servo Motor<br> | DYNAMIXEL X330 | 2 pcs. | ROBOTIS e-shop, RT-shop |
| Servo IF Board | B-SKY Lab Original Board | 1 pc. | [GitHub](https://github.com/kim-xps12/m5stack_interface_board_feetech_sts "https://github.com/kim-xps12/m5stack_interface_board_feetech_sts") |
| Cable | M5STACK-CABLE-10 | 1 pc. | [switch-science](https://www.switch-science.com/products/5213 "https://www.switch-science.com/products/5213"), sengoku<br> |
| Tire Unit | TAMIYA Narrow Tire | 1 set | Amazon |
| Battery | AAA Type | 3 pcs. | \-  |
| Battery Box | SBH-441AS | 1 pc. | akitsuki |
| Body (3D-Printed Parts) | B-SKY Lab Original Board<br> | 1 set | this repository |
| M3 Bolt |   | 4 pcs. |    |
| M3 Nut |   | 4 pcs. |    |
  

## 開発環境
- Arduino IDE on Windows 11 (please apply M5Stack setting)

## 手動の修正

### IF基板
- please pull up output pin to 3.3V

### dynamixel2arduino
- please refer [here](https://qiita.com/B-SKY-Lab/items/c52801ce683df49d64a1#%E3%83%A9%E3%82%A4%E3%83%96%E3%83%A9%E3%83%AA%E3%81%AE%E6%9B%B8%E3%81%8D%E6%8F%9B%E3%81%88)
  

## Parts 3D-Print and Assembly
- please refer **fron** and **back** view.
- manual: coming soon !


## Reference

トランジスタ技術2019年7月号　月着陸船アポロに学ぶ確率統計コンピュータ: [https://cc.cqpub.co.jp/lib/system/doclib\_library/detail/81368/](https://cc.cqpub.co.jp/lib/system/doclib_library/detail/81368/)

HomeMadeGarbage SHISEIGYO-1DC Plus Recipe: [https://shop.homemadegarbage.com/product/s-1\_dc-plus\_recipe/](https://shop.homemadegarbage.com/product/s-1_dc-plus_recipe/)

HomeMadeGarbage フルスケールレンジの変更 (MPU6886): [https://homemadegarbage.com/reactionwheel06#%E3%83%95%E3%83%AB%E3%82%B9%E3%82%B1%E3%83%BC%E3%83%AB%E3%83%AC%E3%83%B3%E3%82%B8%E5%A4%89%E6%9B%B4](https://homemadegarbage.com/reactionwheel06#%E3%83%95%E3%83%AB%E3%82%B9%E3%82%B1%E3%83%BC%E3%83%AB%E3%83%AC%E3%83%B3%E3%82%B8%E5%A4%89%E6%9B%B4)

M5Unified入門 その1 概要確認 #IMU: [https://lang-ship.com/blog/work/m5unified-1/#toc16](https://lang-ship.com/blog/work/m5unified-1/#toc16)

M5StickCの6軸入力から姿勢を求める(カルマンフィルタ編): [https://shiker.hatenablog.com/entry/2019/08/24/004637](https://shiker.hatenablog.com/entry/2019/08/24/004637)

M5Stack Core2 内蔵IMU (MPU6886) Datasheet: [https://github.com/m5stack/M5-Schematic/blob/master/datasheet/MPU-6886-000193%2Bv1.1\_GHIC.PDF.pdf](https://github.com/m5stack/M5-Schematic/blob/master/datasheet/MPU-6886-000193%2Bv1.1_GHIC.PDF.pdf)

TKJElectronics KalmanFilter Library: [https://github.com/TKJElectronics/KalmanFilter](https://github.com/TKJElectronics/KalmanFilter)