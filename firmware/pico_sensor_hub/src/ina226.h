/*
 * ina226.h — INA226 電流／電壓監測驅動
 *
 * 接線（來源同 ultrasonic.h 的權威腳位表）：
 *   GPIO16 = I2C_SDA, GPIO17 = I2C_SCL  (i2c0, 400 kHz)
 *   GPIO18 = INA226_ALERT (U6 的 ALE 腳)
 *
 * 量測鏈：電池 → F1(25A) → R1 分流器 2.5mΩ (Kelvin 四端) → 正極匯流排
 *   R1 的 sense 端 → U6 的 IN+/IN-
 *   U6 的 VBUS 腳 → 匯流排（量電壓）
 *
 * 校正參數由 planner 指定，不自行換算：
 *   R_shunt     = 0.0025 Ω
 *   Current_LSB = 1 mA
 *   CAL         = 0.00512 / (Current_LSB × R_shunt) = 2048
 *   Power_LSB   = 25 × Current_LSB = 25 mW
 *   量測上限    ≈ ±32.768 A（分流器 81.92mV ÷ 2.5mΩ）
 *
 * 前提：INA226 模組板載的 0.1Ω 電阻（絲印 R100）必須已拆除，改用外部 2.5mΩ。
 *       若未拆，讀值會完全錯誤（電流會小 40 倍）。
 *
 * 健壯性契約：INA226 不存在或 I2C 逾時時，本模組所有函式都必須立即回傳失敗，
 *            絕不阻塞。呼叫端據此標示故障旗標並繼續運轉。
 */
#ifndef INA226_H
#define INA226_H

#include <stdint.h>
#include <stdbool.h>

#define INA226_I2C_SDA_PIN   16
#define INA226_I2C_SCL_PIN   17
#define INA226_ALERT_PIN     18
#define INA226_I2C_BAUD      400000u

/* planner 指定的校正常數 */
#define INA226_CAL_VALUE     2048u   /* 寫進 CALIBRATION 暫存器 */
#define INA226_CURRENT_LSB_UA 1000u  /* 1 mA */
#define INA226_POWER_LSB_MW  25u     /* 25 mW */

/* 暫存器位址 */
#define INA226_REG_CONFIG      0x00
#define INA226_REG_SHUNT_V     0x01
#define INA226_REG_BUS_V       0x02
#define INA226_REG_POWER       0x03
#define INA226_REG_CURRENT     0x04
#define INA226_REG_CALIBRATION 0x05
#define INA226_REG_MASK_EN     0x06
#define INA226_REG_ALERT_LIM   0x07
#define INA226_REG_MFG_ID      0xFE
#define INA226_REG_DIE_ID      0xFF

#define INA226_MFG_ID_TI       0x5449  /* "TI" */
#define INA226_DIE_ID_226      0x2260

/*
 * 初始化 I2C、掃描匯流排、設定 CONFIG / CALIBRATION / MASK_EN。
 * 回傳 true 表示找到並成功設定 INA226；false 表示未找到或設定失敗
 * —— 兩種情況下韌體都必須繼續跑。
 */
bool ina226_init(void);

/* 目前是否處於可用狀態 */
bool ina226_present(void);

/* 偵測到的 I2C 位址（未偵測到時回 0） */
uint8_t ina226_addr(void);

/*
 * 讀取三個量測值。任一暫存器讀取失敗即回 false，且不寫入輸出參數
 * —— 呼叫端必須輸出哨兵值而非上一次的舊值（回報假數值比不回報更危險）。
 */
bool ina226_read(int32_t *bus_mV, int32_t *current_mA, int32_t *power_mW);

/* 累計 I2C 錯誤次數 */
uint32_t ina226_err_count(void);

/*
 * 掃描整條 I2C 匯流排（0x08..0x77），把找到的位址寫入 found[]。
 * 回傳找到的裝置數。供 $CMD,I2CSCAN 與開機診斷使用。
 */
int ina226_bus_scan(uint8_t *found, int max);

#endif /* INA226_H */
