/*
 * main.c — Pico 感測器集線板韌體
 *
 * 目標板：Raspberry Pi Pico 2 W (RP2350)
 *
 * 這顆 Pico 負責硬即時的部分：8 顆 HC-SR04 的微秒級脈寬量測，以及 INA226 的
 * I2C 輪詢，整理好之後以 USB CDC 回報給 Jetson。
 *
 * 超音波在本設計中的定位是「近距離補盲」，主要避障由 LiDAR 承擔，因此本韌體的
 * 可靠性目標是「不可以卡死、不可以回報假的近距離」，而不是「距離要多準」：
 *   - 回報假的近距離 → 機器人無故急停
 *   - 卡死            → 補盲整個消失
 * 這兩件事的防線分別是：距離哨兵值絕不使用 0、以及硬體看門狗 + 全非阻塞主迴圈。
 *
 * 時序（更新率 10 Hz）：
 *   t=0ms   觸發組 A (前左+後右)
 *   t=25ms  鎖存 A、觸發組 B (前右+後左)
 *   t=50ms  鎖存 B、觸發組 C (右前+左後)
 *   t=75ms  鎖存 C、觸發組 D (右後+左前)
 *   t=100ms 鎖存 D、送出 $US 與 $PW、seq++、回到組 A
 */

#include <stdio.h>
#include <string.h>
#include <ctype.h>

#include "pico/stdlib.h"
#include "pico/stdio_usb.h"
#include "hardware/watchdog.h"

#include "proto.h"
#include "ultrasonic.h"
#include "ina226.h"

/* ------------------------------------------------------------------ */
/* 版本與組建資訊                                                       */
/* ------------------------------------------------------------------ */
#ifndef FW_VERSION
#define FW_VERSION "0.1.0"
#endif
#ifndef FW_BUILD_DATE
#define FW_BUILD_DATE "unknown"
#endif
#ifndef FW_BOARD
#define FW_BOARD "pico2_w"
#endif

/* ------------------------------------------------------------------ */
/* $ST 的狀態旗標                                                      */
/* ------------------------------------------------------------------ */
#define FLAG_INA226_FAULT   (1u << 0)  /* INA226 不存在或 I2C 讀取失敗 */
#define FLAG_WDT_REBOOT     (1u << 1)  /* 上一次重開是看門狗造成的 */
#define FLAG_US_TIMEOUT     (1u << 2)  /* 本次循環有通道逾時（含未接線） */
#define FLAG_USB_DROP       (1u << 3)  /* 曾因 USB 未連線而丟棄輸出 */
#define FLAG_INA226_CAL_ERR (1u << 4)  /* INA226 初始化／校正寫入失敗 */
#define FLAG_BAD_CMD        (1u << 5)  /* 曾收到無法辨識或校驗錯誤的指令 */
#define FLAG_US_CH_FAULT    (1u << 6)  /* 有通道處於故障狀態（ECHO 卡高電位） */

/* ------------------------------------------------------------------ */
/* 排程週期                                                            */
/* ------------------------------------------------------------------ */
#define GROUP_PERIOD_MS   25
#define REPORT_PERIOD_MS  100
#define STATUS_PERIOD_MS  1000
#define WATCHDOG_TIMEOUT_MS 2000

#define CMD_BUF_SZ 128

static uint32_t s_flags = 0;
static uint16_t s_seq   = 0;

/* ------------------------------------------------------------------ */
/* 輸出                                                                */
/* ------------------------------------------------------------------ */

static void send_id(void) {
    proto_send("ID,%s,%s,%s", FW_VERSION, FW_BUILD_DATE, FW_BOARD);
}

static void send_us(void) {
    /*
     * 距離一律是整數 mm；-1 = 逾時無回波，-2 = 通道故障。
     * 絕不使用 0 —— 0 會被下游解讀成「貼著障礙物」而觸發誤急停。
     */
    proto_send("US,%u,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld",
               (unsigned)s_seq,
               (long)us_distance_mm(0), (long)us_distance_mm(1),
               (long)us_distance_mm(2), (long)us_distance_mm(3),
               (long)us_distance_mm(4), (long)us_distance_mm(5),
               (long)us_distance_mm(6), (long)us_distance_mm(7));
}

static void send_pw(void) {
    int32_t bus_mV = 0, cur_mA = 0, pwr_mW = 0;
    unsigned ok;

    if (ina226_read(&bus_mV, &cur_mA, &pwr_mW)) {
        s_flags &= ~FLAG_INA226_FAULT;
        ok = 1;
    } else {
        /*
         * 讀不到就送哨兵值，不送上一次的舊值。
         * 回報陳舊的電池電壓比不回報更危險。
         *
         * v0.2.0 起有效性由獨立的 ok 欄位表達，不再靠「三欄同時 -1」推測：
         * current_mA = -1 本身是合法量測值（充電 1 mA），舊格式無法區分。
         * 後三欄仍送 -1 以保持哨兵語意，但下游只該看 ok。
         */
        s_flags |= FLAG_INA226_FAULT;
        ok = 0;
        bus_mV = -1;
        cur_mA = -1;
        pwr_mW = -1;
    }

    proto_send("PW,%u,%u,%ld,%ld,%ld",
               (unsigned)s_seq, ok, (long)bus_mV, (long)cur_mA, (long)pwr_mW);
}

static void send_st(void) {
    /*
     * 只反映「最近這個 $ST 週期內有沒有新的丟棄」，不做成黏性旗標。
     * 開機時 USB 還沒連上，第一次的 $ID 必然被丟棄；做成黏性的話這個位元
     * 會永遠亮著，正常狀態長得跟故障一樣，旗標就失去意義了。
     */
    static uint32_t last_dropped = 0;
    uint32_t dropped_now = proto_dropped_lines();
    if (dropped_now > last_dropped) {
        s_flags |= FLAG_USB_DROP;
    } else {
        s_flags &= ~FLAG_USB_DROP;
    }
    last_dropped = dropped_now;

    if (us_any_channel_fault()) {
        s_flags |= FLAG_US_CH_FAULT;
    } else {
        s_flags &= ~FLAG_US_CH_FAULT;
    }

    proto_send("ST,%lu,%04lX,%lu,%lu",
               (unsigned long)to_ms_since_boot(get_absolute_time()),
               (unsigned long)s_flags,
               (unsigned long)us_timeout_count(),
               (unsigned long)ina226_err_count());
}

/* ------------------------------------------------------------------ */
/* 指令處理                                                            */
/* ------------------------------------------------------------------ */

/*
 * 判準 8 的看門狗驗證用鉤子。
 *
 * 用途：驗證硬體看門狗確實在運作 —— 進入這個迴圈後就不再餵狗，
 *       2 秒後 RP2350 會自動重開並重新送出 $ID。
 * 保留原因：這是唯一能在不拆機的情況下證明看門狗還活著的手段，
 *          日後改動主迴圈結構時應該重跑一次。
 * 風險：這是刻意的當機路徑，正常運轉不會走到。
 */
static void cmd_testhang(void) {
    proto_send("ACK,TESTHANG");
    for (;;) {
        tight_loop_contents();   /* 不餵狗，等看門狗把我們咬醒 */
    }
}

static void cmd_jitter(void) {
    /*
     * 脈寬量測抖動統計，作為日後評估是否改用 PIO 的依據。
     * 最乾淨的量法是把 TRIG 短接到 ECHO：理論脈寬固定 10µs，
     * 量到的離散度就是「中斷 + 硬體計時器」這條路徑的抖動。
     */
    for (int ch = 0; ch < US_NUM_CH; ++ch) {
        uint32_t n = 0, mn = 0, mx = 0, last = 0;
        us_jitter_stats(ch, &n, &mn, &mx, &last);
        proto_send("JT,%d,%lu,%lu,%lu,%lu",
                   ch, (unsigned long)n, (unsigned long)mn,
                   (unsigned long)mx, (unsigned long)last);
    }
}

static void cmd_i2cscan(void) {
    uint8_t found[16];
    int n = ina226_bus_scan(found, (int)(sizeof found));

    char list[PROTO_PAYLOAD_MAX];
    int off = snprintf(list, sizeof list, "I2C,%d", n);
    for (int i = 0; i < n && i < (int)(sizeof found); ++i) {
        int rem = (int)sizeof list - off;
        if (rem <= 6) {
            break;
        }
        off += snprintf(list + off, (size_t)rem, ",0x%02X", found[i]);
    }
    proto_send("%s", list);
}

static void handle_command(const char *payload) {
    /* payload 形如 "CMD,PING" */
    if (strncmp(payload, "CMD,", 4) != 0) {
        s_flags |= FLAG_BAD_CMD;
        proto_send("NAK,%s", payload);
        return;
    }

    const char *arg = payload + 4;

    if (strcmp(arg, "PING") == 0) {
        proto_send("ACK,PING");
    } else if (strcmp(arg, "ID") == 0) {
        send_id();
    } else if (strcmp(arg, "RESET") == 0) {
        proto_send("ACK,RESET");
        sleep_ms(10);                 /* 讓 ACK 有機會送出去 */
        watchdog_reboot(0, 0, 0);     /* 用看門狗觸發重開 */
        for (;;) { tight_loop_contents(); }
    } else if (strcmp(arg, "JITTER") == 0) {
        cmd_jitter();
    } else if (strcmp(arg, "JITRESET") == 0) {
        us_jitter_reset();
        proto_send("ACK,JITRESET");
    } else if (strcmp(arg, "I2CSCAN") == 0) {
        cmd_i2cscan();
    } else if (strcmp(arg, "TESTHANG") == 0) {
        cmd_testhang();               /* 不返回 */
    } else {
        s_flags |= FLAG_BAD_CMD;
        proto_send("NAK,%s", payload);
    }
}

/* 非阻塞讀取主機指令。任何格式錯誤都回 $NAK，絕不當機。 */
static void poll_input(void) {
    static char buf[CMD_BUF_SZ];
    static size_t len = 0;
    static bool overflow = false;

    for (;;) {
        int c = getchar_timeout_us(0);
        if (c == PICO_ERROR_TIMEOUT || c < 0) {
            return;
        }

        if (c == '\n' || c == '\r') {
            if (len == 0) {
                overflow = false;
                continue;
            }
            buf[len] = '\0';

            if (overflow) {
                s_flags |= FLAG_BAD_CMD;
                proto_send("NAK,OVERFLOW");
            } else {
                char payload[CMD_BUF_SZ];
                if (proto_parse_line(buf, payload, sizeof payload)) {
                    handle_command(payload);
                } else {
                    /* 校驗錯或格式錯 —— 回報原文讓對方能對照，但不當機 */
                    s_flags |= FLAG_BAD_CMD;
                    proto_send("NAK,%s", buf);
                }
            }
            len = 0;
            overflow = false;
            continue;
        }

        if (len + 1 >= sizeof buf) {
            overflow = true;   /* 超長就標記，等收到換行再一次回 NAK */
            continue;
        }
        buf[len++] = (char)c;
    }
}

/* ------------------------------------------------------------------ */
/* main                                                               */
/* ------------------------------------------------------------------ */

int main(void) {
    stdio_init_all();

    /*
     * 開機流程不依賴任何前次狀態：USB 由 Jetson 供電，Jetson 重開機時
     * 這顆 Pico 會直接斷電重啟，所以每次都必須從乾淨狀態起來。
     */
    if (watchdog_caused_reboot()) {
        s_flags |= FLAG_WDT_REBOOT;
    }

    us_init();

    if (!ina226_init()) {
        /*
         * INA226 不在也要繼續跑。標旗標、繼續量超音波 ——
         * 少了電流監測不該讓整台車失去近距離感知。
         */
        s_flags |= FLAG_INA226_FAULT | FLAG_INA226_CAL_ERR;
    }

    watchdog_enable(WATCHDOG_TIMEOUT_MS, 1);

    send_id();

    absolute_time_t next_group  = make_timeout_time_ms(GROUP_PERIOD_MS);
    absolute_time_t next_status = make_timeout_time_ms(STATUS_PERIOD_MS);
    int  cur_group = 0;
    bool usb_was_connected = false;

    us_start_group(cur_group);

    for (;;) {
        watchdog_update();

        /*
         * USB 斷線→連線時補送 $ID。
         * 開機時不等待主機（等待會阻塞），所以第一次的 $ID 主機通常收不到；
         * 這裡在對方真的連上來時再送一次，讓人知道跑的是哪一版。
         */
        bool usb_now = stdio_usb_connected();
        if (usb_now && !usb_was_connected) {
            send_id();
        }
        usb_was_connected = usb_now;

        us_poll();
        poll_input();

        if (time_reached(next_group)) {
            us_latch_group(cur_group);

            bool completed_cycle = (cur_group == US_NUM_GROUPS - 1);
            cur_group = (cur_group + 1) % US_NUM_GROUPS;

            if (completed_cycle) {
                /* 四組都量完了，這一輪的八個距離才是完整的 */
                bool any_timeout = false;
                for (int ch = 0; ch < US_NUM_CH; ++ch) {
                    if (us_distance_mm(ch) == US_DIST_TIMEOUT) {
                        any_timeout = true;
                        break;
                    }
                }
                if (any_timeout) {
                    s_flags |= FLAG_US_TIMEOUT;
                } else {
                    s_flags &= ~FLAG_US_TIMEOUT;
                }

                send_us();
                send_pw();
                s_seq++;
            }

            us_start_group(cur_group);

            /*
             * 用固定節拍推進而不是 now + 25ms，避免處理時間累積成漂移，
             * 更新率才守得住 10 Hz。
             */
            next_group = delayed_by_ms(next_group, GROUP_PERIOD_MS);
        }

        if (time_reached(next_status)) {
            send_st();
            next_status = delayed_by_ms(next_status, STATUS_PERIOD_MS);
        }
    }

    return 0;   /* 到不了 */
}
