/*
 * ultrasonic.h — 8 通道 HC-SR04 量測
 *
 * 腳位對應以 .aidev/tasks/exec1.md「呼叫鏈與資料流」那張表為唯一權威，
 * 該表取自 hardware/kicad/kicad circuit.net。此處不得自行更動。
 *
 *   ch  TRIG    ECHO    網路名            車體位置
 *   0   GPIO0   GPIO1   US1_TRIG/ECHO     前左
 *   1   GPIO2   GPIO3   US2_TRIG/ECHO     前右
 *   2   GPIO4   GPIO5   US3_TRIG/ECHO     右前
 *   3   GPIO6   GPIO7   US4_TRIG/ECHO     右後
 *   4   GPIO8   GPIO9   US5_TRIG/ECHO     後右
 *   5   GPIO10  GPIO11  US6_TRIG/ECHO     後左
 *   6   GPIO12  GPIO13  US7_TRIG/ECHO     左後
 *   7   GPIO14  GPIO15  US8_TRIG/ECHO     左前
 *
 * 串音對策：四組對向分組同時發射，指向相反的兩顆聽不到彼此。
 *   組 A = ch0(前左) + ch4(後右)
 *   組 B = ch1(前右) + ch5(後左)
 *   組 C = ch2(右前) + ch6(左後)
 *   組 D = ch3(右後) + ch7(左前)
 * 每組 25ms 量測窗，四組輪完 100ms → 更新率 10 Hz。
 */
#ifndef ULTRASONIC_H
#define ULTRASONIC_H

#include <stdint.h>
#include <stdbool.h>

#define US_NUM_CH        8
#define US_NUM_GROUPS    4
#define US_TIMEOUT_US    25000u   /* 回波逾時，對應約 4.2m */
#define US_TRIG_PULSE_US 10u      /* 觸發脈衝寬度 */

/* 距離回報的哨兵值。絕不可用 0 表示「沒量到」—— 0 會被下游當成貼著障礙物而誤急停。 */
#define US_DIST_TIMEOUT  (-1)     /* 逾時無回波（含未接線） */
#define US_DIST_FAULT    (-2)     /* 通道故障：ECHO 卡在高電位 */

void us_init(void);

/* 觸發第 group 組（0..3）的兩個通道 */
void us_start_group(int group);

/* 主迴圈呼叫：處理逾時判定。不阻塞。 */
void us_poll(void);

/* 把第 group 組當前的量測結果鎖存進對外的結果陣列 */
void us_latch_group(int group);

/* 取得通道最後一次鎖存的距離（mm），或 US_DIST_TIMEOUT / US_DIST_FAULT */
int32_t us_distance_mm(int ch);

/* 車體位置名稱，例如 "前左" */
const char *us_channel_name(int ch);

/* 累計逾時次數（所有通道合計） */
uint32_t us_timeout_count(void);

/* 目前是否有任何通道處於故障狀態 */
bool us_any_channel_fault(void);

/*
 * 脈寬量測抖動統計 —— 供日後評估是否要改用 PIO 的依據。
 * n 為樣本數，min/max/last 單位為 µs。無樣本時 n = 0。
 */
void us_jitter_stats(int ch, uint32_t *n, uint32_t *min_us,
                     uint32_t *max_us, uint32_t *last_us);
void us_jitter_reset(void);

#endif /* ULTRASONIC_H */
