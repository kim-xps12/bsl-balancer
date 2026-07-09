# BSL-Balancer (ﾀｲﾘﾝﾁｬﾝ)

## About
BSL-Balancerは手乗りサイズの学習向け対向二輪型倒立振子です．
M5Stack Core2とDYNAMIXEL XL330を主軸に3Dプリンタ製のボディで構成することで，コストと性能のバランス両立を目指しました．
ｽﾀｯｸﾁｬﾝ*と同じ[m5stack-avatar](https://github.com/meganetaaan/m5stack-avatar)による顔を設定することで親しみやすいデザインに．対向二輪型倒立振子タイプのｽﾀｯｸﾁｬﾝ，「ﾀｲﾘﾝﾁｬﾝ」とお呼びください．画面右上の髪飾りは大輪の花を表しています．

BSL-Balancer is a Palm-sized, two-wheeled inverted pendulum educational use.
By constructing a 3D printed body with M5Stack Core2 and DYNAMIXEL XL330 as the main components, I aimed to achieve a balance between cost and performance.
It has a friendly design with a face created by [m5stack-avatar](https://github.com/meganetaaan/m5stack-avatar), which is the same as stack-chan*. This is a two-wheeled inverted pendulum type Stack Chan, please call me "Tairin-chan".The hair ornament at the top right of the screen represents a "large flower (in japanese, Tairin-no-hana)."

*ｽﾀｯｸﾁｬﾝは[ししかわ](https://twitter.com/stack_chan)さんが開発，公開している， 手乗りサイズのｽｰﾊﾟｰｶﾜｲｲコミュニケーションロボットです．リポジトリ：https://github.com/meganetaaan/stack-chan

*stack-chan is a hand-sized super cute communication robot developed and published by [Shishikawa](https://twitter.com/stack_chan)-san. Repository: https://github.com/meganetaaan/stack-chan


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
| Battery Box | SBH-431-1AS150 | 1 pc. | [Akizuki](https://akizukidenshi.com/catalog/g/g103196/) |
| XH Connector Housing| XHP-2 | 1 pc. | [Akizuki](https://akizukidenshi.com/catalog/g/g112255/) |
| XH Connector Contact| SXH-001T-P0.6 | 1 pack | [Akizuki](https://akizukidenshi.com/catalog/g/g112264/) |
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

### Battery Box

[基板のREADMEのピンアサイン](https://github.com/kim-xps12/m5stack_board_dynamixel_ttl_rs3485?tab=readme-ov-file#pin-assign)を参考に，電池ボックスにXHコネクタを取り付けてください．

Crimp the XH connector to the battery box, referring to the [pin assignment in the README of the board](https://github.com/kim-xps12/m5stack_board_dynamixel_ttl_rs3485?tab=readme-ov-file#pin-assign).

### Change baudrate of XL330

お好みのDYNAMIXEL開発環境，あるいは[m5core2_dynamixel_wizard](https://github.com/kim-xps12/m5core2_dynamixel_wizard)をM5Stack Core2へ書き込んで利用し，使用するDYNAMIXEL XL330のBaudrateを`1000000`に設定してください．

Use your preferred DYNAMIXEL development environment or [m5core2_dynamixel_wizard](https://github.com/kim-xps12/m5core2_dynamixel_wizard) to write it to the M5Stack Core2 and set the Baudrate of the DYNAMIXEL XL330 you are using to `1000000`.

## Usage

**重要（v2ファームウェア）**: 本ファームウェアはXL330を**電流制御モード（Current Control Mode）**・200Hz制御で駆動します。安全設計の詳細は [`docs/plans/2026-07-02-current-mode-freertos-redesign.md`](./docs/plans/2026-07-02-current-mode-freertos-redesign.md) を参照してください。

**IMPORTANT (v2 firmware)**: This firmware drives the XL330 in **Current Control Mode** with a 200 Hz loop. See the design doc above for the safety architecture.

### Initial bring-up (first time only) / 初回ブリングアップ

工場状態（未コミッショニング）では安全のため **PROFILE_BRINGUP（電流上限150mA・自動アームなし）** で起動します。以下の符号試験に合格してから通常運転へ移行してください。

Out of the box the firmware boots in **PROFILE_BRINGUP** (150 mA current limit, no auto-arm). Complete the sign test below before normal operation.

1. 車体を浮かせた状態で電源を入れ、**BtnC（右）長押し1秒**でアーム（DISARMED→IDLE）
2. BtnB（中央）でパネルを開き、`th`（傾斜角）表示を確認: **前傾で正方向に変化**すること（IMU軸/符号の確認）
3. 手で立てて倒立開始後、前進時に両輪の正規化速度が正であること（車輪符号の確認）
4. 問題があれば `src/app_config.h` の `kSignLeft/kSignRight`（車輪）または `imu_backend.cpp` の軸マップを修正
5. 合格したらパネルの **[COMISN]** をタップ（トルクOFF状態で）→ 次回起動から **PROFILE_NORMAL（900mA・自動アーム有効）**

### How to stand up

※コミッショニング完了後の手順です（自動アーム有効）。初回は上のブリングアップを先に実施してください。
(After commissioning, auto-arm is enabled. Run the bring-up above first.)

1. 電池ボックスの電源スイッチをONにします
    
    Turn on the power switch on the battery box.
   
1. M5Stack Core2の画面が鉛直になるように手で支えて保持します

    Manually hold and support the M5Stack Core2 so that its screen is vertical.

1. M5Stack Core2の電源をONにします

    Turn on the power of the M5Stack Core2.

1. ゆっくりと手を離すと倒立します

    Release your hand and it will stand up.


もし難しい場合は，次の手順を試してみてください．

If you find this difficult, please try the following steps:

1. 先にM5Stack Core2の電源をONにします

    First, turn on the power of the M5Stack Core2.

1. 電池ボックスの電源スイッチをONにします

    Turn on the power switch on the battery box.

1. M5Stack Core2の画面が鉛直になるように手で支えて保持します

    Manually hold and support the M5Stack Core2 so that its screen is vertical.
   
1. M5Stack Core2のリセットボタン（本体下部側のボタン）を短く押して離します

    Press shortly and release the reset button located at the bottom side of the device.

1. ゆっくりと手を離すと倒立します

    Release your hand and it will stand up.



### How to tune parameters

1. ボタンB（中央）をタップしてコントロールパネルを表示します

    Tap the B button (middle) to display the control panel.

1. [+]/[-]で `Eq`（平衡点トリム, deg）・`Kp`・`Ki`・`Kd` を変更できます（内部はSI単位: A/rad系）

    Use [+]/[-] to adjust `Eq` (equilibrium trim, deg), `Kp`, `Ki`, `Kd` (internally SI: A/rad).

1. **[SAVE]** をタップすると設定をNVSへ保存します（**トルクOFF状態でのみ有効**。倒立中はBtnC長押しで停止してから）

    Tap **[SAVE]** to persist settings to NVS (only while torque is OFF — stop with a BtnC long-press first).

### Stop / Safety

- **BtnC（右）長押し1秒 = 停止/アーム トグル**（DISARMED⇄IDLE）。倒立中でも即座にトルクを切ります
- **確実な停止は電池ボックスの電源スイッチ**です（タッチボタンは利便機能であり安全装置ではありません）
- 転倒すると自動でトルクOFFになり、立て直して約2秒静止すると再開します（30秒に3回転倒すると安全のためFAULTでラッチ→本体リセットで復帰）
- FAULT時はavatarが怒り顔になります。復帰は本体リセット。サーボ側Shutdown（過熱等）が原因の場合はサーボ電源の再投入も必要です

- **BtnC (right) long-press = STOP/ARM toggle.** The hard stop is the battery box power switch (touch buttons are convenience, not safety devices).
- After a fall, torque turns off automatically; stand it still for ~2 s to resume. Three falls within 30 s latch a FAULT (reset to recover; servo-side Shutdown also needs a servo power cycle).



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

