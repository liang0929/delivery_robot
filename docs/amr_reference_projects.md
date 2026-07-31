# 開源 AMR 專案參考彙整

> 整理日期：2026-07-28
> 用途：為 base_dev 平台（24V BLDC 差速底盤 / Jetson Orin NX / Nav2）挑選可借鏡的成熟專案
> 資料來源：各專案官方文件與手冊，連結見各節

---

## 1. 速查表

| 專案 | 類型 | 成熟度 | 授權 | 對本專案的價值 |
|------|------|--------|------|----------------|
| [Nav2 `opennav_docking`](#21-nav2-opennav_docking) | 軟體套件 | ⭐⭐⭐⭐⭐ | Apache 2.0 | **直接可用**，自動對接充電座 |
| [Husarion ROSbot](#22-husarion-rosbot) | 完整機器人 | ⭐⭐⭐⭐ | Apache 2.0 | 套件架構範本、電池狀態上報 |
| [TurtleBot 4 / Create 3](#23-turtlebot-4--irobot-create-3) | 完整機器人 | ⭐⭐⭐⭐⭐ | 開源標竿 | 智慧電池 + 自動回充座的完整實作 |
| [linorobot2](#24-linorobot2) | 軟韌體框架 | ⭐⭐⭐ | MIT | Nav2/SLAM 整合範本、韌體電池監測 |
| [AgileX LIMO](#25-agilex-limo) | 完整機器人 | ⭐⭐⭐ | 部分開源 | 教育用途，參考價值較低 |
| [Nova Carter](#26-nova-carter-segway--nvidia) | 工業平台 | ⭐⭐⭐⭐⭐ | 硬體商用 | **電源架構與充電座設計的標竿** |
| [ST 工業 AMR 參考設計](#27-st--nvidia-工業-amr-reference-design) | 參考設計 | ⭐⭐⭐⭐ | 商用 | 24V→48V 電源架構 |
| [Libre Solar BMS C1](#31-libre-solar-bms-c1) | 開源 BMS | ⭐⭐⭐⭐ | CERN-OHL-W v2 | 自組電池組時的 BMS 方案 |
| [openAMRobot](#4-對照組openamrobot) | 完整機器人 | ⭐ | 開源 | **不建議參考實作**，但對接研究文件有價值 |

---

## 2. 機器人平台

### 2.1 Nav2 `opennav_docking`

自動對接的官方解法，**優先於任何自製方案**。

- Repo：<https://github.com/open-navigation/opennav_docking>（上游已併入 `ros-navigation/navigation2` 的 `nav2_docking`）
- 官方文件：[Using Docking Server](https://docs.nav2.org/tutorials/docs/using_docking.html)、[參數設定](https://docs.nav2.org/configuration/packages/configuring-docking-server.html)
- 實務經驗：[ROSCon 2024 — On Use of Nav2 Docking](https://roscon.ros.org/2024/talks/On_Use_of_Nav2_Docking.pdf)

**關鍵特性**

- `SimpleChargingDock` plugin 是 **detector-agnostic** 的，吃任何 `detected_dock_pose`
  → 相機（AprilTag）或 LiDAR 偵測都可以接
- `use_battery_status` 參數：訂閱 `sensor_msgs/BatteryState` 判斷「真的在充電了沒」
- 也支援用 docked 狀態當備案（初期測試或還沒有電池遙測時）
- 支援 odometry「dock blind」收尾

**對本專案**：`go_charging` 的「最後一公尺」直接用這個，不要自己寫。前置條件是要能發出 `BatteryState`。

---

### 2.2 Husarion ROSbot

商業產品但驅動完全開源，**ROS 2 套件架構的最佳範本**。

- 主 repo：<https://github.com/husarion/rosbot_ros>（ROSbot XL / 3 / 3 PRO / 2 / 2R / 2 PRO）
- ROSbot XL 專用：<https://github.com/husarion/rosbot_xl_ros>
- 手冊：<https://husarion.com/manuals/rosbot-xl/>
- 授權：Apache 2.0

**電池規格**

| 型號 | 電池 | 電壓 | 容量 | 充電 |
|------|------|------|------|------|
| ROSbot XL | 內建 3S Li-ion pack（含保護電路） | 11.1V（9V=0%，12.6V=100%） | 7800mAh / **86Wh** | **USB-C PD 65W（12–19V），可邊充邊跑**，5–6h 充滿 |
| ROSbot 3 | 3× 18650 protected | 11.1V（3S） | 3500mAh/顆 | 座充 |
| ROSbot 2R | 3× 18650 protected | 11.1V（3S） | 1800–3500mAh/顆 | 座充 |

**值得抄**

- 套件切分方式：`driver` / `description` / `bringup` / `gazebo` 分開，各自可獨立測試
- micro-ROS + STM32F4 韌體結構（對應本專案的 Pico 2 W 下位機）
- 電池狀態上報的實作方式
- [選擇機器人電池的技術文](https://husarion.com/blog/batteries-for-mobile-robots/)

---

### 2.3 TurtleBot 4 / iRobot Create 3

開源機器人標竿。**唯一一個「自動回充 + 智慧電池」有量產驗證的開源平台。**

- Repo：<https://github.com/turtlebot/turtlebot4>
- 使用手冊：<https://turtlebot.github.io/turtlebot4-user-manual/>
- Create 3 電氣文件：<https://iroboteducation.github.io/create3_docs/hw/electrical/>
- ROS 2 Jazzy 支援：[Clearpath 公告](https://clearpathrobotics.com/blog/2024/10/turtlebot-4-now-supports-ros-2-jazzy/)

**電池規格（Create 3 底盤）**

| 項目 | 規格 |
|------|------|
| 電池 | Roomba e/i 系列原廠 **4S Li-ion 智慧電池** |
| 電壓 | 14.4V 標稱（12V min / 16.8V max） |
| 容量 | **26Wh** |
| 續航 / 充電 | 2.5–4h（依負載）/ 2.5h 充滿 |
| 充電 | iRobot Home Base **自動回充座** |
| 供電輸出 | UI board 提供 VBAT / 12V / 5V / 3.3V 給 payload |

**智慧電池的保護邏輯（值得完整照抄的部分）**

- 總電壓到 **12.0V 時回報 0%**（不是等到真的沒電）
- **10.8V 以下自我保護，直接與負載斷開**
- **連續充電 4 小時未達 100% 會自動禁止充電**（防止充不飽的異常電池持續受熱）
- 保護觸發後需**取出靜置 15 分鐘**才能 reset
- 軟體版本 G.5.4 / H.2.4 起，電量低於 2% 自動關機，避免機器人卡在半路

**對本專案**：這套「多層保護 + 分級門檻」的設計思路，正好對應 `docs/power_design.md` §2 的電壓分級表。

---

### 2.4 linorobot2

openAMRobot 的**上游**。與其看下游未完成的分支，不如直接看這裡。

- 軟體：<https://github.com/linorobot/linorobot2>
- 韌體：<https://github.com/linorobot/linorobot2_hardware>
- 活躍分支：`jazzy`
- 社群 fork（電池監測較完整）：<https://github.com/hippo5329/linorobot2_hardware/wiki>

**特點**

- Nav2 + SLAM Toolbox + robot_localization **已經接好**的完整基礎
- 支援 2WD / 4WD / Mecanum
- Gazebo 模擬與實機共用同一套設定
- 電池：未指定，建議 3S LiPo（11.1V / 12V）

**韌體的電池監測 hook**（`lino_base_config.h`）

```c
// #define BATTERY_PIN 33
// 3.3V ref, 12 bits ADC, 33k + 10k voltage divider
#define BATTERY_ADJUST(v) ((v) * (3.3 / 4096 * (33 + 10) / 10))
// #define USE_INA219
#define BATTERY_DIP 0.98   // 電壓下垂警告
// #define BATTERY_CAP 2.0  // 電池容量 Ah
```

上游預設全部註解掉，`hippo5329` 的 fork 完成度較高。**本專案要做庫倫計時可以參考這裡的分壓與校正公式。**

---

### 2.5 AgileX LIMO

教育／研究用小型多模態平台，參考價值主要在文件完整度。

- 文件 repo：<https://github.com/agilexrobotics/limo-doc>
- 規格：<https://docs.trossenrobotics.com/agilex_limo_docs/specifications.html>

**電池**：12V 內建鋰電，標準版 5200mAh（續航 40 分鐘）／ Pro 版 10Ah（續航 2.5 小時）
**充電**：12.6V 5A 充電器，DC barrel jack 5.5×2.1mm

電壓等級（12V）與本專案（24V）不同，硬體不可直接參考。

---

### 2.6 Nova Carter（Segway + NVIDIA）

工業級 AMR 開發平台。**電源架構與充電座設計的標竿。**

- 產品頁：<https://robotics.segway.com/nova-carter/>
- 產品手冊（規格出處）：[Nova Carter Product Manual v1.0](https://robotics.segway.com/wp-content/uploads/2023/11/Nova-Carter-Product-Manual-v1.0.pdf)
- 底盤：Segway RMP Lite 220；運算：NVIDIA Jetson AGX Orin

**電源規格**

| 項目 | 規格 |
|------|------|
| 電池容量 | **1033Wh** |
| 充電器 | 輸入 100–240V 50/60Hz 2.5A，**輸出 42V 5.0A**（推測 10S 鋰電） |
| 充電時間 | 5h |
| 續航 | ≥8h |
| 電池更換 | 支援（可熱抽換） |
| 電池防護 | **IPX7** |
| 認證 | **UL2271、UN38.3、MSDS、EMC、GOST-R、RoHS、EU Battery Directive** |
| 承載 | 50kg |
| 外部介面 | 10GbE、DisplayPort、USB-C 3.2 Gen 2 |

**充電座設計（本專案第二階段的直接參考）**

- **Safe-charge 系統**：只有機器人接觸時才通電 —— 外露接點平時不帶電
- **彈簧探針（spring-loaded posts）**：維持整個充電週期的接觸壓力
- **對位導引結構**：機械上把機器人導進正確位置
- **視覺狀態指示**：顯示充電狀態
- 隨附**開源的自動對接軟體模組**

**核心啟示**：成熟平台的電池是「買認證過的整包」，不是自己疊電芯。BMS、認證、防水都由電池供應商負責。

---

### 2.7 ST + NVIDIA 工業 AMR Reference Design

- 報導：[Industrial AMR Reference Design](https://www.electronicsforu.com/electronics-projects/industrial-amr-reference-design-to-accelerate-robot-development)
- 組成：完整 STMicroelectronics BOM + NVIDIA Jetson Orin Nano + ROS 2 生態
- **24V 電池架構，並具備往 48V 遷移的驗證路徑**

與本專案（24V + Jetson Orin NX）電壓等級與運算平台最接近，適合當電源架構的對照組。

---

## 3. BMS / 電池管理

> ⚠️ **重要原則：不要在已有內建 BMS 的電池組上再串一顆 BMS。**
> 兩塊保護板串聯會造成閾值誤判、MOSFET 壓降與發熱疊加、均衡功能互相打架、
> 故障排除困難。保護與量測是兩件事，應該分開。

### 3.1 Libre Solar BMS C1

自組電池組時的首選開源方案。

- 產品頁：<https://libre.solar/hardware/bms-c1.html>
- Repo：<https://github.com/LibreSolar/bms-c1>
- 手冊：<https://libre.solar/bms-c1/manual/>

| 項目 | 規格 |
|------|------|
| 主控 | TI **bq76952** + Espressif **ESP32-C3** |
| 支援化學 | LiFePO4、Li-ion NMC |
| 串數 / 電流 | 最高 **16S / 100A**（涵蓋 12V / 24V / 48V） |
| 通訊 | **CAN、RS-485**、USB、USART、I2C、BLE、WiFi |
| 保護 | 過流／短路等時間關鍵保護**做在 ASIC 硬體層**，韌體只負責校準 |
| 硬體授權 | CERN-OHL-W v2（弱互惠） |

**設計亮點**：把時間關鍵的保護放在硬體、韌體只做參數校正——這是安全關鍵設計的正確做法，值得在任何電源安全設計中借鏡。

### 3.2 diyBMS

社群 DIY 路線，成本更低但工程嚴謹度不如 Libre Solar。
GitHub topic：<https://github.com/topics/bms>

### 3.3 量測（非保護）方案

若電池組已有 BMS，缺的是 **SOC 量測**而非保護：

| 方案 | 說明 |
|------|------|
| **INA226 / INA228 + 分流電阻** | I2C 介面，可掛在既有的 MCU 下位機。INA226 共模上限 36V（適用 24V 系統），INA228 上限 85V |
| 商用庫倫計 | 多數帶 UART 輸出，免自製 |
| 更換為 smart BMS | JBD / Daly 等帶 UART+藍牙或 CAN，但需拆解電池組 |

---

## 4. 對照組：openAMRobot

<https://github.com/openAMRobot>

**結論：不建議參考其實作，但研究文件有價值。**

| 面向 | 狀態 |
|------|------|
| 版本 | v0.0.1（2026-07-13 首次公開發布） |
| 作者自承 | 「still early」，functional safety / docking reliability / industrial readiness 皆未到位 |
| 電池 | 4× DM12-7S 鉛酸 AGM（12V 7Ah），2 串 = 24V 7Ah ≈ 168Wh（實際可用約 84Wh） |
| BMS | **無** |
| 電源安全 | **無保險絲、無電池側斷路、電池與市電並聯有回灌風險**（自己列出但未修） |
| 充電 | 手動外接鉛酸充電器，充電器型號未記錄 |
| 自動對接 | Phase 6（實際接電）標明為 target architecture，**未實作** |
| 待確認項 | DC-DC 型號、AC/DC 型號、LiDAR 型號皆未抄錄 |

**唯一值得讀的**：[`14_docking_research.md`](https://github.com/openAMRobot/openamr-platform-sw/blob/main/ros2/src/openamrobot_docking/docs/14_docking_research.md)

一份 vendor-agnostic 的無線充電對接研究，包含：

- 四家商用無線充電器（TZBOT、WiBotic、Wiferion、Xnergy）的公開規格比較
  —— 橫向容差 ±30~50mm、氣隙 10~30mm、host interface 為 binary open-collector 或 CAN/Modbus
- 感測方法完整分類與比較（AprilTag 單標／多標、ChArUco、LiDAR、RGB-D）
- 接收線圈安裝幾何的比較分析
- 失效模式分析與校正／驗收流程

此文件品質遠高於該專案的實作，可單獨作為對接設計的參考。

> **注意**：該研究針對**無線充電**（線圈對位，需 ±10mm 級精度）。
> 若採**接觸式充電**（彈簧探針 + 機械導引），容差可放寬到 ±10~20mm，難度低一個量級。

---

## 5. 電池規格橫向對照

| 專案 | 化學／組態 | 電壓 | 容量 | 充電方式 | BMS |
|------|-----------|------|------|---------|-----|
| **base_dev（本專案）** | 7S2P+ 18650 Li-ion | 25.9V（充電上限 28.0V） | 30Ah / **777Wh** | 手動 XT60，28.0V CC/CV | ✅ 電池內建 |
| Nova Carter | 鋰電 10S | 36V（42V 滿充） | **1033Wh** | 42V/5A，選配自動充電座 | ✅ 含 UL2271 認證 |
| openAMRobot | 鉛酸 AGM | 24V | 7Ah / 168Wh | 手動 | ❌ 無 |
| ROSbot XL | Li-ion 3S | 11.1V | 7800mAh / 86Wh | USB-C PD 65W | ✅ 內建保護電路 |
| ROSbot 3 | Li-ion 3S（18650） | 11.1V | 3500mAh/顆 | 座充 | ✅ 電芯保護板 |
| TurtleBot 4 / Create 3 | Li-ion 4S 智慧電池 | 14.4V | 26Wh | 自動回充座 | ✅ 智慧電池內建 |
| AgileX LIMO | 鋰電 | 12V | 5200mAh（Pro 10Ah） | 12.6V/5A barrel jack | 電池內建 |
| linorobot2 | 建議 3S LiPo | 11.1V / 12V | 自選 | 平衡充 | LiPo 保護板 |

**觀察**

1. **電壓分兩派**：教育／小型機在 11.1–14.4V，工業／大型 AMR 在 24–36V。本專案屬後者，故 TurtleBot 4 / ROSbot 的電池方案不可直接移植，只能借鏡其邏輯。
2. **成熟專案不自製 BMS**，一律採購帶 BMS 與安全認證的成品電池組。
3. **充電介面演進**：DC barrel jack → USB-C PD（可邊充邊用）→ 自動回充座。
4. 本專案的 777Wh 在此列表中僅次於 Nova Carter，能量規模不是瓶頸。

---

## 6. 對 base_dev 的採用建議

| 需求 | 採用方案 | 優先度 |
|------|---------|--------|
| 自動對接充電座 | **Nav2 `opennav_docking`**（§2.1），detector 可用 LiDAR 或相機 | 高 |
| 充電座硬體設計 | **Nova Carter 模式**（§2.6）：safe-charge + 彈簧探針 + 機械導引 | 高 |
| 電池 SOC 量測 | **INA226 + 分流電阻**掛既有 Pico 2 W（§3.3），輸出 `sensor_msgs/BatteryState` | 高 |
| 電池保護分級門檻 | 參考 **Create 3 智慧電池邏輯**（§2.3），對應 `power_design.md` §2 | 中 |
| ROS 2 套件架構重整 | 參考 **`rosbot_ros`** 的分層（§2.2） | 中 |
| 韌體電池監測分壓設計 | 參考 **linorobot2 `BATTERY_ADJUST`**（§2.4） | 中 |
| 未來自組電池組 | **Libre Solar BMS C1**（§3.1） | 低 |

**不建議**：參考 openAMRobot 的任何實作程式碼；在既有電池組上外加第二顆 BMS。

---

## 相關文件

- [`docs/power_design.md`](power_design.md) — 本機配電設計
- [`ARCHITECTURE.md`](../ARCHITECTURE.md) §8 — 未來規劃（`[ ] 自動充電對接`）
