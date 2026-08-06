# pico_sensor_hub

AMR 感測器集線板韌體。跑在 **Raspberry Pi Pico 2 WH（RP2350）** 上，
負責 8 顆 HC-SR04 超音波的微秒級脈寬量測與 INA226 電流電壓監測，
整理後以 USB CDC 序列埠回報給 Jetson。

## 為什麼需要這顆 Pico

- HC-SR04 需要微秒級的脈寬量測，Jetson 跑 Linux 沒有即時性保證。
- INA226 的 I2C 不適合跟 Jetson 40-pin 搶（那條已經給 BNO055 用了）。

LiDAR 與 IMU 直接接 Jetson，不經過這裡。

## 可靠性目標

超音波在本設計中的定位是**近距離補盲**，主要避障仍由 LiDAR 承擔
（`docs/power_design.md` 第 409 行）。因此本韌體的要求是
**「不可以卡死、不可以回報假的近距離」**，而不是「距離要多準」：

| 風險 | 後果 | 防線 |
|---|---|---|
| 回報假的近距離 | 機器人無故急停 | 距離哨兵值**絕不使用 0**；ECHO 腳下拉避免浮接亂跳；讀取失敗時送哨兵值而非上一次的舊值 |
| 卡死 | 補盲整個消失 | 硬體看門狗 2 秒；主迴圈全非阻塞；USB 未連線時直接丟棄輸出 |

---

## 1. 環境建置

在乾淨的 Ubuntu 22.04（Jetson Orin NX，aarch64）上從零開始。

### 1.1 交叉工具鏈與建置工具

```bash
sudo apt-get update
sudo apt-get install -y \
    gcc-arm-none-eabi \
    libnewlib-arm-none-eabi \
    libstdc++-arm-none-eabi-newlib \
    ninja-build \
    cmake \
    git
```

> **不要用 `pip` 裝任何東西。** 本機的 Python 環境很脆弱，`pip --user`
> 曾經破壞既有套件。需要 Python 套件一律用 `apt`。
> 本專案的 `tools/picoterm.py` 只用標準庫，所以連 `python3-serial` 都不是必要的。

### 1.2 pico-sdk

放在家目錄，**不要放進專案 repo**：

```bash
git clone --recurse-submodules --depth 1 --shallow-submodules \
    https://github.com/raspberrypi/pico-sdk.git ~/pico-sdk
```

驗證版本與 RP2350 支援：

```bash
grep -E "set\(PICO_SDK_VERSION_(MAJOR|MINOR|REVISION)" ~/pico-sdk/pico_sdk_version.cmake
ls ~/pico-sdk/src/boards/include/boards/pico2_w.h
```

本韌體開發時使用 **SDK 2.3.0**。RP2350 需要 SDK 2.0 以上。

### 1.3 環境變數

```bash
export PICO_SDK_PATH=~/pico-sdk
```

要長期生效就加進 `~/.bashrc`。

---

## 2. 編譯

```bash
cd firmware/pico_sensor_hub
rm -rf build
cmake -S . -B build -G Ninja -DPICO_SDK_PATH=~/pico-sdk
cmake --build build
```

產出：`build/pico_sensor_hub.uf2`

目標板由 `CMakeLists.txt` 的 `PICO_BOARD pico2_w` 決定。
若用的是不帶無線的 Pico 2，改成 `-DPICO_BOARD=pico2` 重新 configure。

---

## 3. 燒錄

### 3.1 韌體已經在跑的時候（免手按 BOOTSEL）

第二次以後的燒錄用這個，不必碰硬體：

```bash
stty -F /dev/ttyACM0 1200      # 1200 baud 是 pico-sdk 的 magic value
sleep 3                        # 等它重新列舉成 BOOTSEL 裝置
cp build/pico_sensor_hub.uf2 /media/$USER/RP2350/
sync
```

原理是 pico-sdk 的 `PICO_ENABLE_USB_RESET_VIA_BAUD_RATE`（預設開啟）：
主機把 CDC 埠的 baud rate 設成 1200 就會重開進 BOOTSEL。

> `picotool load` 也能做到同一件事，但 SDK 自動建置的 picotool 是
> **不含 USB 支援**的版本（缺 `libusb-1.0-dev`），只能處理檔案不能對裝置操作。
> 要用 `picotool` 直接燒錄的話得先 `sudo apt-get install libusb-1.0-0-dev`
> 再重新建置 picotool。上面的 `stty` 方法不需要這些。

> ⚠️ **重開之後裝置節點可能會變**（`/dev/ttyACM0` → `/dev/ttyACM1`），
> 因為舊的節點還沒被核心釋放。裝好 3.3 的 udev rule 之後就不用管這件事——
> 一律用 `/dev/pico_sensor_hub`，它永遠指向當下正確的 ACM 節點。
> 上面 `stty` 那行的裝置路徑同理，可以直接寫 `/dev/pico_sensor_hub`。

### 3.2 第一次燒錄（或韌體掛掉救不回來時）

1. 按住 Pico 的 **BOOTSEL** 鈕再插 USB（或按住 BOOTSEL 按一下 RESET）。
2. 確認進入 bootloader：

   ```bash
   lsusb | grep 2e8a
   # Bus 001 Device 006: ID 2e8a:000f Raspberry Pi RP2350 Boot
   ```

3. 掛載並複製：

   ```bash
   # 大多數桌面環境會自動掛載成 /media/$USER/RP2350
   ls /media/$USER/
   cp build/pico_sensor_hub.uf2 /media/$USER/RP2350/
   sync
   ```

   沒有自動掛載時手動掛：

   ```bash
   lsblk -o NAME,SIZE,LABEL | grep -i rp2350
   sudo mkdir -p /mnt/pico
   sudo mount /dev/sdX1 /mnt/pico
   sudo cp build/pico_sensor_hub.uf2 /mnt/pico/
   sudo umount /mnt/pico
   ```

4. Pico 會自動重開並以 CDC 裝置重新列舉：

   ```bash
   ls -l /dev/ttyACM*
   # crw-rw---- 1 root dialout 166, 0 ... /dev/ttyACM0
   ```

存取 `/dev/ttyACM0` 需要在 `dialout` 群組：

```bash
id -nG | tr ' ' '\n' | grep -x dialout || sudo usermod -aG dialout $USER
# 加完要重新登入才生效
```

### 3.3 固定裝置節點（udev rule）

Pico 每次重新列舉都可能在 `/dev/ttyACM0` 與 `/dev/ttyACM1` 之間跳，
寫死路徑的消費者（ROS 節點、picoterm）會隨機斷線。裝這條 rule 之後，
一律改用穩定的 `/dev/pico_sensor_hub`：

```bash
cd firmware/pico_sensor_hub
sudo cp tools/99-pico-sensor-hub.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger --subsystem-match=tty
```

驗證（拔插一次再看）：

```bash
ls -l /dev/pico_sensor_hub
# lrwxrwxrwx 1 root root 7 ... /dev/pico_sensor_hub -> ttyACM0
```

規則以 USB 的 `2e8a:0009`（Raspberry Pi CDC）匹配，權限維持 `dialout` 群組，
所以仍需 3.2 的群組設定。車上若之後掛第二顆 Pico，VID/PID 會撞號，
要在規則裡加 `ATTRS{serial}` 區分——註解裡有寫法。

> BOOTSEL 模式（`2e8a:000f`）是大量儲存裝置不是 tty，不會命中這條規則，
> 所以燒錄時 `/dev/pico_sensor_hub` 會暫時消失，燒完自動回來。

---

## 4. 驗證

### 4.1 最快的健康檢查

```bash
cat /dev/pico_sensor_hub
```

協定是 ASCII 行式，可以直接讀。應該看到 `$US` / `$PW` 每秒各 10 行、
`$ST` 每秒 1 行。

### 4.2 用 picoterm.py

```bash
cd firmware/pico_sensor_hub

# 即時表格（Ctrl-C 結束）
./tools/picoterm.py

# 驗收用：跑 5 分鐘後印統計摘要
./tools/picoterm.py --duration 300

# 只看原始行
./tools/picoterm.py --raw

# 送指令（自動補上 XOR 校驗）
./tools/picoterm.py --send 'CMD,PING' --duration 5
./tools/picoterm.py --send 'CMD,I2CSCAN' --duration 3
./tools/picoterm.py --send 'CMD,JITTER' --duration 3

# 原樣送出，不補校驗 —— 驗證韌體面對畸形輸入會回 $NAK 而非當機
./tools/picoterm.py --duration 5 \
    --send-raw 'this is not a protocol line' \
    --send-raw '$US,1*ZZ' \
    --send-raw '$CMD,PING*00'

# 指定裝置節點（預設 /dev/ttyACM0；裝了 3.3 的 udev rule 後建議用固定節點）
./tools/picoterm.py -d /dev/pico_sensor_hub --duration 300
```

`--duration` 模式結束時會直接對照三項驗收門檻印出 PASS/FAIL：
校驗錯誤 0 筆、掉幀率 < 0.1%、更新率 10 Hz ±10%。
未達標時行程回傳碼為 1，可直接串進自動化流程。

腳本只用 Python 標準庫（`termios` + `os`）。系統上若剛好裝了 pyserial
（`sudo apt-get install python3-serial`），可用 `--backend pyserial` 切換。

### 4.3 跳線自測（不需要真的感測器）

用一條杜邦線把某通道的 **TRIG 直接短接到 ECHO**，
該通道就會回報約 **1–2 mm** 而不是「逾時」——
這證明觸發、中斷、計時、換算整條量測路徑真的在工作。

原理：TRIG 的 10µs 脈衝直接回灌 ECHO，量到 10µs，
換算成 `10 × 0.1715 ≈ 1.7 mm`。

| 通道 | 車體位置 | TRIG | ECHO |
|---|---|---|---|
| 0 | 前左 | GPIO0 | GPIO1 |
| 1 | 前右 | GPIO2 | GPIO3 |
| 2 | 右前 | GPIO4 | GPIO5 |
| 3 | 右後 | GPIO6 | GPIO7 |
| 4 | 後右 | GPIO8 | GPIO9 |
| 5 | 後左 | GPIO10 | GPIO11 |
| 6 | 左後 | GPIO12 | GPIO13 |
| 7 | 左前 | GPIO14 | GPIO15 |

### 4.4 看門狗驗證

```bash
./tools/picoterm.py --send 'CMD,TESTHANG' --duration 10
```

`$CMD,TESTHANG` 會進入一個不餵狗的無限迴圈，2 秒後 RP2350 自動重開，
重新送出 `$ID`，且 `$ST` 的 flags 會出現 `bit1`（看門狗重開）。

這條指令**刻意保留**在正式韌體裡：它是唯一能在不拆機的情況下
證明看門狗還活著的手段，日後改動主迴圈結構時應該重跑一次。

---

## 5. 序列協定

ASCII 行式 + XOR 校驗。選 ASCII 是因為本專案的除錯成本比頻寬重要
（10 Hz × 幾十位元組完全不是問題），可以直接 `cat` 出來看。

### 5.1 Pico → 主機

```
$US,<seq>,<d1>,<d2>,<d3>,<d4>,<d5>,<d6>,<d7>,<d8>*<XX>
$PW,<seq>,<ok>,<bus_mV>,<current_mA>,<power_mW>*<XX>
$ST,<uptime_ms>,<flags_hex>,<us_timeout_cnt>,<i2c_err_cnt>*<XX>
$ID,<fw_ver>,<build_date>,<board>*<XX>
```

| 訊息 | 週期 |
|---|---|
| `$US` | 100 ms |
| `$PW` | 100 ms |
| `$ST` | 1000 ms |
| `$ID` | 開機時一次，以及每次 USB 從斷線變成連線時 |

- **`seq`**：0–65535 循環，每個量測循環 +1。`$US` 與 `$PW` 共用同一個 seq。
- **`dN`**：距離 **mm 整數**，順序為 `d1..d8` = 通道 0..7（前左、前右、右前、右後、後右、後左、左後、左前）。
  - `-1` = 逾時無回波（**含感測器未接線**）
  - `-2` = 通道故障（觸發前 ECHO 就已是高電位，代表線卡住或感測器壞了）
  - **絕不會是 0。** 0 會被下游當成「貼著障礙物」而誤急停。
- **`current_mA`**：有號整數，**放電為正、充電為負**。
- **`ok`**：`1` = 本次 INA226 讀取有效；`0` = 無效（INA226 不在或 I2C 逾時），
  此時後三欄一律為 `-1`，且 `$ST` flags bit0 會置位。無效時**不會**送出上一次的舊值。

  > **v0.2.0 的格式變更。** v0.1.x 用「三欄同時為 `-1`」表示無效，
  > 但 `current_mA = -1` 本身是合法量測值（充電 1 mA），下游無法可靠區分。
  > 改成獨立 `ok` 欄後，**下游只該看 `ok`**，不要再靠哨兵值推測。
  > 升級注意：欄位數由 5 變 6，舊解析器會讀錯（會把 `ok` 當成 `bus_mV`），
  > 必須連同 `tools/picoterm.py` 與 ROS 端一起更新。

- **`*<XX>`**：`$` 之後、`*` 之前所有字元的 XOR，兩位大寫十六進位。

### 5.2 `$ST` 的 flags 位元

| bit | 意義 |
|---|---|
| 0 | INA226 不存在或 I2C 讀取失敗 |
| 1 | 上一次重開是看門狗造成的 |
| 2 | 本次循環有通道逾時（含未接線） |
| 3 | 曾因 USB 未連線而丟棄輸出 |
| 4 | INA226 初始化／校正寫入失敗 |
| 5 | 曾收到無法辨識或校驗錯誤的指令 |
| 6 | 有通道處於故障狀態 |

### 5.3 主機 → Pico

| 指令 | 回應 | 用途 |
|---|---|---|
| `$CMD,PING*XX` | `$ACK,PING*XX` | 存活確認 |
| `$CMD,ID*XX` | `$ID,...*XX` | 查韌體版本 |
| `$CMD,RESET*XX` | `$ACK,RESET*XX` 後重開 | 用看門狗觸發重開 |
| `$CMD,I2CSCAN*XX` | `$I2C,<n>,0xNN,...*XX` | 掃描 I2C 匯流排 |
| `$CMD,JITTER*XX` | 8 行 `$JT,<ch>,<n>,<min_us>,<max_us>,<last_us>*XX` | 脈寬量測抖動統計 |
| `$CMD,JITRESET*XX` | `$ACK,JITRESET*XX` | 清空抖動統計 |
| `$CMD,TESTHANG*XX` | `$ACK,TESTHANG*XX` 後卡死 | 驗證看門狗（見 4.4） |

無法辨識的指令 → `$NAK,<原文>*XX`，**不會當機**。
校驗錯誤的行同樣回 `$NAK`。

手算校驗很麻煩，用 `picoterm.py --send 'CMD,PING'` 就好，它會自動補上。

---

## 6. 硬體對應

### 6.1 腳位（唯一權威：`hardware/kicad/kicad circuit.net`）

| Pico GPIO | 網路名 | 對應 | 車體位置 |
|---|---|---|---|
| GPIO0 / GPIO1 | `US1_TRIG` / `US1_ECHO` | `U11` | 前左 |
| GPIO2 / GPIO3 | `US2_TRIG` / `US2_ECHO` | `U12` | 前右 |
| GPIO4 / GPIO5 | `US3_TRIG` / `US3_ECHO` | `U13` | 右前 |
| GPIO6 / GPIO7 | `US4_TRIG` / `US4_ECHO` | `U14` | 右後 |
| GPIO8 / GPIO9 | `US5_TRIG` / `US5_ECHO` | `U15` | 後右 |
| GPIO10 / GPIO11 | `US6_TRIG` / `US6_ECHO` | `U16` | 後左 |
| GPIO12 / GPIO13 | `US7_TRIG` / `US7_ECHO` | `U17` | 左後 |
| GPIO14 / GPIO15 | `US8_TRIG` / `US8_ECHO` | `U18` | 左前 |
| GPIO16 / GPIO17 | `I2C_SDA` / `I2C_SCL` | `U6` INA226 | — |
| GPIO18 | `INA226_ALERT` | `U6` ALE | — |
| GPIO19–22, 26–28 | 未接 | — | 保留 |
| VBUS (pin40) | `USB_PICO` | 由 Jetson USB 供電 | — |
| VSYS (pin39) | 未接 | — | — |

**實作與這張表不符時要停手回報，不要自行更動腳位。**

### 6.2 串音對策

HC-SR04 最遠約 4 m，來回約 23 ms。八顆同時發射會互相聽到對方的回波，
所以採 **對向分組同時發射**，指向相反的兩顆一組：

| 組 | 同時發射 | 量測窗 |
|---|---|---|
| A | 前左 + 後右 | 25 ms |
| B | 前右 + 後左 | 25 ms |
| C | 右前 + 左後 | 25 ms |
| D | 右後 + 左前 | 25 ms |

四組輪完 100 ms → **更新率 10 Hz**。
逾時採非阻塞判定，任一通道逾時都不會擋住其他通道或主迴圈。

### 6.3 INA226 量測鏈

```
電池 → F1(25A) → R1 分流器 2.5mΩ (Kelvin 四端) → 正極匯流排
                  └→ sense 端接 U6 的 IN+/IN-
                                U6 的 VBUS 腳接匯流排量電壓
```

校正參數（由 planner 指定，韌體不自行換算）：

| 參數 | 值 |
|---|---|
| `R_shunt` | 0.0025 Ω |
| `Current_LSB` | 1 mA |
| `CAL` | `0.00512 / (1mA × 0.0025Ω)` = **2048** |
| `Power_LSB` | 25 × 1 mA = **25 mW** |
| 量測上限 | ≈ ±32.768 A（81.92 mV ÷ 2.5 mΩ） |

> ⚠️ **INA226 模組板載的 0.1Ω 電阻（絲印 R100）必須已拆除**，改用外部 2.5 mΩ。
> 若沒拆，電流讀值會差 40 倍。校驗方式：拿三用電表量電池電壓跟 `$PW` 的
> `bus_mV` 比對，誤差超過 5% 就要回頭檢查 R100。

`CONFIG = 0x4527`：16 次平均、VBUSCT/VSHCT 各 1.1 ms、Shunt+Bus 連續模式，
轉換時間 16 × (1.1 + 1.1) = 35.2 ms，短於 100 ms 的回報週期。

`INA226_ALERT`（GPIO18）本版設為 **Conversion Ready**（`MASK_EN` bit10），
讓讀取時序確定、不漏樣本。

---

## 7. 目錄結構

```
firmware/pico_sensor_hub/
  CMakeLists.txt
  pico_sdk_import.cmake     ← 從 SDK 複製，用來定位 PICO_SDK_PATH
  README.md
  src/
    main.c                  ← 排程、指令處理、看門狗
    ultrasonic.c/.h         ← 8 通道量測、對向分組、抖動統計
    ina226.c/.h             ← I2C 驅動、位址掃描、校正
    proto.c/.h              ← XOR 校驗編解碼、非阻塞輸出
  tools/
    picoterm.py             ← 主機端驗證腳本（純標準庫）
    99-pico-sensor-hub.rules ← udev 規則，固定成 /dev/pico_sensor_hub（見 3.3）
```

ROS 2 端的消費者是 `src/pico_sensor_hub/`（另一個 package，見該目錄的 README）。

`build/` 為產出目錄，不進版控。

---

## 8. 疑難排解

| 症狀 | 可能原因 |
|---|---|
| `/dev/ttyACM0` 沒出現 | 韌體沒跑起來，或 Pico 還在 BOOTSEL（`lsusb` 看到 `2e8a:000f` 就是還在 bootloader；正常執行是 `2e8a:0009`） |
| 開啟 `/dev/ttyACM0` 被拒 | 不在 `dialout` 群組，或加了群組後沒重新登入 |
| 八個通道全部 `-1` | 正常——感測器沒接就是這樣。用 4.3 的跳線自測確認量測路徑本身沒問題 |
| 某通道固定 `-2` | 該通道 ECHO 卡在高電位：接線短路到 VCC、感測器故障，或 ECHO 接錯腳 |
| `$PW` 的 `ok` 欄一直是 0 | INA226 沒接、位址不對或 I2C 沒上拉。跑 `$CMD,I2CSCAN` 看匯流排上有什麼 |
| `/dev/pico_sensor_hub` 不存在 | udev rule 沒裝或沒 reload（見 3.3），或 Pico 還在 BOOTSEL |
| 電流值差很多（約 40 倍） | INA226 模組的 R100（0.1Ω）沒拆 |
| 更新率明顯低於 10 Hz | 主機端沒在讀導致 USB 寫入卡住。韌體端已有 20 ms 逾時保護，若仍發生請回報 |
| `$ST` flags 出現 bit1 | 上次是看門狗重開的。若非執行 `TESTHANG` 所致，代表主迴圈真的卡過，要查 |
