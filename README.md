# BSL-Balancer

## About
BSL-Balancerは手乗りサイズの学習向け対向二輪型倒立振子です．
M5Stack Core2とDYNAMIXEL XL330を主軸に3Dプリンタ製のボディで構成することで，コストと性能のバランス両立を目指しました．

BSL-Balancer is a Palm-sized, two-wheeled inverted pendulum educational use.
By constructing a 3D printed body with M5Stack Core2 and DYNAMIXEL XL330 as the main components, I aimed to achieve a balance between cost and performance.

**front**
![front](./docs/front.jpg)

**back**
![back](./docs/back.jpg)


## BOM

|     |     |     |     |
| --- | --- | --- | --- |
| **Item** | **Model** | **Qty.** | **Link** |
| MCU | M5Stack Core2<br> | 1 pc. | [SWITCH SCIENCE](https://ssci.to/9349), <br> [Sengoku](https://www.sengoku.co.jp/mod/sgk_cart/detail.php?code=EEHD-68G8) |
| Servo Motor<br> | DYNAMIXEL XL330-M077-T | 2 pcs. | [ROBOTIS e-shop](https://e-shop.robotis.co.jp/product.php?id=416), <br> [RT-shop](https://www.rt-shop.jp/index.php?main_page=product_info&products_id=3902) |
| Servo IF Board | B-SKY Lab Original Board | 1 pc. | [GitHub](https://github.com/kim-xps12/m5stack_board_dynamixel_ttl_rs3485) |
| Cable | M5STACK-CABLE-10 | 1 pc. | [SWITCH SCIENCE](https://www.switch-science.com/products/5213 "https://www.switch-science.com/products/5213"), <br> [Sengoku](https://www.sengoku.co.jp/mod/sgk_cart/detail.php?code=EEHD-5CLV) |
| Tire Unit | TAMIYA Narrow Tire | 1 set | [Amazon](https://amzn.asia/d/4A3hlcZ) |
| Battery | AAA Type | 3 pcs. | \-  |
| Battery Box | SBH-441AS | 1 pc. | [Akitsuki](https://akizukidenshi.com/catalog/g/g100735/) |
| Body (3D-Printed Parts) | B-SKY Lab Original parts  | 1 set | This repository |
| Magnet | D=6mm, t=3mm| 4 pcs. | [DAISO](https://jp.daisonet.com/products/4549131156621) |
| M3 Hex Nut | class 1 in JIS | 10 pcs. |-|
| M3 Bolt | L=10mm (for top cover)  | 4 pcs. |-|
| M3 Bolt | L=12mm (for tire)  | 6 pcs. |-|
  

## Environment
- PlatformIO on Visual Studio Code
- ~~Arduino IDE on Windows 11 (please apply M5Stack setting)~~

**NOTE**

このリポジトリではArduino IDEのサポートのサポートを終了しました．
`for_arduino_ide`に以前までのコードを保存しているので，サンプルとしての利用は可能です．
Arduino IDEで開発される場合は，これをベースにPIO向けのコードの内容を手動で反映させることでお使いいただけます．

This repository no longer supports Arduino IDE.
The previous code is saved in `for_arduino_ide`, so it can be used as a example.
If you are developing with Arduino IDE, you can use this as a base by manually reflecting the contents of the code in PIO code.

## Parts 3D-Print and Assembly
- please refer **fron** and **back** view.
- manual: coming soon !


## Reference

- トランジスタ技術2019年7月号　月着陸船アポロに学ぶ確率統計コンピュータ: [https://cc.cqpub.co.jp/lib/system/doclib\_library/detail/81368/](https://cc.cqpub.co.jp/lib/system/doclib_library/detail/81368/)

- HomeMadeGarbage SHISEIGYO-1DC Plus Recipe: [https://shop.homemadegarbage.com/product/s-1\_dc-plus\_recipe/](https://shop.homemadegarbage.com/product/s-1_dc-plus_recipe/)

- HomeMadeGarbage フルスケールレンジの変更 (MPU6886): [https://homemadegarbage.com/reactionwheel06#%E3%83%95%E3%83%AB%E3%82%B9%E3%82%B1%E3%83%BC%E3%83%AB%E3%83%AC%E3%83%B3%E3%82%B8%E5%A4%89%E6%9B%B4](https://homemadegarbage.com/reactionwheel06#%E3%83%95%E3%83%AB%E3%82%B9%E3%82%B1%E3%83%BC%E3%83%AB%E3%83%AC%E3%83%B3%E3%82%B8%E5%A4%89%E6%9B%B4)

- M5Unified入門 その1 概要確認 #IMU: [https://lang-ship.com/blog/work/m5unified-1/#toc16](https://lang-ship.com/blog/work/m5unified-1/#toc16)

- M5StickCの6軸入力から姿勢を求める(カルマンフィルタ編): [https://shiker.hatenablog.com/entry/2019/08/24/004637](https://shiker.hatenablog.com/entry/2019/08/24/004637)

- M5Stack Core2 内蔵IMU (MPU6886) Datasheet: [https://github.com/m5stack/M5-Schematic/blob/master/datasheet/MPU-6886-000193%2Bv1.1\_GHIC.PDF.pdf](https://github.com/m5stack/M5-Schematic/blob/master/datasheet/MPU-6886-000193%2Bv1.1_GHIC.PDF.pdf)

- Dynamixel2Arduino: [https://github.com/ROBOTIS-GIT/Dynamixel2Arduino](https://github.com/ROBOTIS-GIT/Dynamixel2Arduino)

- M5Unified: [https://github.com/m5stack/M5Unified](https://github.com/m5stack/M5Unified)

- TKJElectronics KalmanFilter Library: [https://github.com/TKJElectronics/KalmanFilter](https://github.com/TKJElectronics/KalmanFilter)
