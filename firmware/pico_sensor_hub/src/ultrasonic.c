#include "ultrasonic.h"

#include <string.h>

#include "pico/stdlib.h"
#include "hardware/gpio.h"

/* 腳位表 —— 對應 kicad circuit.net，不得更動 */
static const uint8_t TRIG_PIN[US_NUM_CH] = { 0, 2, 4, 6, 8, 10, 12, 14 };
static const uint8_t ECHO_PIN[US_NUM_CH] = { 1, 3, 5, 7, 9, 11, 13, 15 };

static const char *CH_NAME[US_NUM_CH] = {
    "前左", "前右", "右前", "右後", "後右", "後左", "左後", "左前"
};

/* 對向分組：指向相反的兩顆同時發射，彼此聽不到對方的回波 */
static const uint8_t GROUP[US_NUM_GROUPS][2] = {
    { 0, 4 },  /* A: 前左 + 後右 */
    { 1, 5 },  /* B: 前右 + 後左 */
    { 2, 6 },  /* C: 右前 + 左後 */
    { 3, 7 },  /* D: 右後 + 左前 */
};

typedef enum {
    ST_IDLE = 0,
    ST_WAIT_RISE,
    ST_WAIT_FALL,
    ST_DONE,
    ST_TIMEOUT,
    ST_FAULT,
} ch_state_t;

typedef struct {
    volatile ch_state_t state;
    volatile uint64_t   t_armed;    /* 觸發時刻，用於逾時判定 */
    volatile uint64_t   t_rise;
    volatile uint32_t   pulse_us;   /* 最後一次成功量到的脈寬 */
    /* jitter 統計 */
    uint32_t jn;
    uint32_t jmin;
    uint32_t jmax;
    uint32_t jlast;
} ch_t;

static ch_t     s_ch[US_NUM_CH];
static int32_t  s_result[US_NUM_CH];
static int8_t   s_pin2ch[32];        /* ECHO GPIO → channel 反查表 */
static uint32_t s_timeout_cnt = 0;

/*
 * ECHO 邊緣中斷。
 * 同一時刻最多只有 2 個通道處於 armed 狀態，中斷密度很低。
 * ISR 內只做時間戳與狀態轉移，距離換算留給主迴圈。
 */
static void echo_irq(uint gpio, uint32_t events) {
    if (gpio >= 32) {
        return;
    }
    int8_t ch = s_pin2ch[gpio];
    if (ch < 0) {
        return;
    }

    uint64_t now = time_us_64();

    if (events & GPIO_IRQ_EDGE_RISE) {
        if (s_ch[ch].state == ST_WAIT_RISE) {
            s_ch[ch].t_rise = now;
            s_ch[ch].state  = ST_WAIT_FALL;
        }
    }
    if (events & GPIO_IRQ_EDGE_FALL) {
        if (s_ch[ch].state == ST_WAIT_FALL) {
            uint64_t d = now - s_ch[ch].t_rise;
            s_ch[ch].pulse_us = (d > 0xFFFFFFFFull) ? 0xFFFFFFFFu : (uint32_t)d;
            s_ch[ch].state    = ST_DONE;
        }
    }
}

void us_init(void) {
    memset(s_ch, 0, sizeof s_ch);
    memset(s_pin2ch, -1, sizeof s_pin2ch);

    for (int ch = 0; ch < US_NUM_CH; ++ch) {
        gpio_init(TRIG_PIN[ch]);
        gpio_set_dir(TRIG_PIN[ch], GPIO_OUT);
        gpio_put(TRIG_PIN[ch], 0);

        gpio_init(ECHO_PIN[ch]);
        gpio_set_dir(ECHO_PIN[ch], GPIO_IN);
        /*
         * ECHO 下拉：感測器未接線時腳位才會穩定在低電位，
         * 否則浮接會亂跳並產生假的回波邊緣（= 假的近距離，正是本韌體最該避免的事）。
         */
        gpio_pull_down(ECHO_PIN[ch]);

        s_pin2ch[ECHO_PIN[ch]] = (int8_t)ch;
        s_ch[ch].state = ST_IDLE;
        s_ch[ch].jmin  = UINT32_MAX;
        s_result[ch]   = US_DIST_TIMEOUT;
    }

    gpio_set_irq_callback(&echo_irq);
    for (int ch = 0; ch < US_NUM_CH; ++ch) {
        gpio_set_irq_enabled(ECHO_PIN[ch],
                             GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);
    }
    irq_set_enabled(IO_IRQ_BANK0, true);
}

void us_start_group(int group) {
    if (group < 0 || group >= US_NUM_GROUPS) {
        return;
    }

    for (int i = 0; i < 2; ++i) {
        int ch = GROUP[group][i];

        /*
         * 觸發前 ECHO 就已經是高電位 → 這條線卡住了（短路到 VCC、感測器故障，
         * 或上一次的回波沒有結束）。標為故障而不是逾時：故障要能跟「單純沒接線」
         * 區分開來，否則問題會被埋在一片 -1 裡看不見。
         */
        if (gpio_get(ECHO_PIN[ch])) {
            s_ch[ch].state = ST_FAULT;
            continue;
        }

        s_ch[ch].state   = ST_WAIT_RISE;
        s_ch[ch].t_armed = time_us_64();
    }

    /* 兩顆同時發射：先一起拉高，等 10µs，再一起拉低 */
    for (int i = 0; i < 2; ++i) {
        int ch = GROUP[group][i];
        if (s_ch[ch].state == ST_WAIT_RISE) {
            gpio_put(TRIG_PIN[ch], 1);
        }
    }
    busy_wait_us(US_TRIG_PULSE_US);
    for (int i = 0; i < 2; ++i) {
        gpio_put(TRIG_PIN[GROUP[group][i]], 0);
    }
}

void us_poll(void) {
    uint64_t now = time_us_64();
    for (int ch = 0; ch < US_NUM_CH; ++ch) {
        ch_state_t st = s_ch[ch].state;
        if (st != ST_WAIT_RISE && st != ST_WAIT_FALL) {
            continue;
        }
        if (now - s_ch[ch].t_armed >= US_TIMEOUT_US) {
            s_ch[ch].state = ST_TIMEOUT;
            /*
             * 計數不放在這裡。逾時門檻是 t_armed + 25ms，而 t_armed 設定於
             * us_start_group()，比鎖存的節拍基準晚一點點，所以鎖存幾乎總是
             * 搶先把狀態清成 IDLE，這條路徑事實上很少走到。
             * 計數統一在 us_latch_group() 依最終結果認定，每通道每循環一次。
             */
        }
    }
}

void us_latch_group(int group) {
    if (group < 0 || group >= US_NUM_GROUPS) {
        return;
    }

    for (int i = 0; i < 2; ++i) {
        int ch = GROUP[group][i];

        switch (s_ch[ch].state) {
        case ST_DONE: {
            uint32_t p = s_ch[ch].pulse_us;

            /* 距離 = 脈寬 × 343 m/s ÷ 2 = 脈寬 × 0.1715 mm/µs */
            int32_t mm = (int32_t)(((uint64_t)p * 1715u) / 10000u);
            s_result[ch] = mm;

            /* jitter 統計 */
            s_ch[ch].jlast = p;
            if (p < s_ch[ch].jmin) s_ch[ch].jmin = p;
            if (p > s_ch[ch].jmax) s_ch[ch].jmax = p;
            s_ch[ch].jn++;
            break;
        }
        case ST_FAULT:
            s_result[ch] = US_DIST_FAULT;
            break;
        case ST_TIMEOUT:
        case ST_WAIT_RISE:   /* 還沒回來就算逾時，不讓它跨到下一輪 */
        case ST_WAIT_FALL:
        default:
            s_result[ch] = US_DIST_TIMEOUT;
            s_timeout_cnt++;   /* 逾時計數的唯一來源：每通道每循環最多一次 */
            break;
        }

        s_ch[ch].state = ST_IDLE;
    }
}

int32_t us_distance_mm(int ch) {
    if (ch < 0 || ch >= US_NUM_CH) {
        return US_DIST_FAULT;
    }
    return s_result[ch];
}

const char *us_channel_name(int ch) {
    if (ch < 0 || ch >= US_NUM_CH) {
        return "??";
    }
    return CH_NAME[ch];
}

uint32_t us_timeout_count(void) {
    return s_timeout_cnt;
}

bool us_any_channel_fault(void) {
    for (int ch = 0; ch < US_NUM_CH; ++ch) {
        if (s_result[ch] == US_DIST_FAULT) {
            return true;
        }
    }
    return false;
}

void us_jitter_stats(int ch, uint32_t *n, uint32_t *min_us,
                     uint32_t *max_us, uint32_t *last_us) {
    if (ch < 0 || ch >= US_NUM_CH) {
        if (n) *n = 0;
        return;
    }
    if (n)       *n       = s_ch[ch].jn;
    if (min_us)  *min_us  = (s_ch[ch].jn == 0) ? 0 : s_ch[ch].jmin;
    if (max_us)  *max_us  = s_ch[ch].jmax;
    if (last_us) *last_us = s_ch[ch].jlast;
}

void us_jitter_reset(void) {
    for (int ch = 0; ch < US_NUM_CH; ++ch) {
        s_ch[ch].jn   = 0;
        s_ch[ch].jmin = UINT32_MAX;
        s_ch[ch].jmax = 0;
        s_ch[ch].jlast = 0;
    }
}
