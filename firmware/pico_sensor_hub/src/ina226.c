#include "ina226.h"

#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/gpio.h"

#define I2C_PORT        i2c0
#define I2C_TIMEOUT_US  2000   /* 400kHz 下傳 3 bytes 約 60µs，2ms 已是極寬鬆的上限 */

/*
 * CONFIG = 0x4527
 *   [15]    RST    = 0
 *   [14:12] 固定    = 100
 *   [11:9]  AVG    = 010  → 16 次平均
 *   [8:6]   VBUSCT = 100  → 1.1 ms
 *   [5:3]   VSHCT  = 100  → 1.1 ms
 *   [2:0]   MODE   = 111  → Shunt and Bus, Continuous
 * 轉換時間 = 16 × (1.1 + 1.1) = 35.2 ms，遠短於 100ms 的回報週期。
 */
#define INA226_CONFIG_VALUE  0x4527u

/* MASK/ENABLE bit10 = CNVR：ALERT 腳在每次轉換完成時觸發。 */
#define INA226_MASK_CNVR     0x0400u

static uint8_t  s_addr    = 0;
static bool     s_present = false;
static uint32_t s_err     = 0;

static bool wr16(uint8_t reg, uint16_t val) {
    uint8_t buf[3] = { reg, (uint8_t)(val >> 8), (uint8_t)(val & 0xFF) };
    int n = i2c_write_timeout_us(I2C_PORT, s_addr, buf, 3, false, I2C_TIMEOUT_US);
    if (n != 3) {
        s_err++;
        return false;
    }
    return true;
}

static bool rd16_at(uint8_t addr, uint8_t reg, uint16_t *val) {
    int n = i2c_write_timeout_us(I2C_PORT, addr, &reg, 1, true, I2C_TIMEOUT_US);
    if (n != 1) {
        s_err++;
        return false;
    }
    uint8_t b[2];
    n = i2c_read_timeout_us(I2C_PORT, addr, b, 2, false, I2C_TIMEOUT_US);
    if (n != 2) {
        s_err++;
        return false;
    }
    *val = (uint16_t)((b[0] << 8) | b[1]);
    return true;
}

static bool rd16(uint8_t reg, uint16_t *val) {
    return rd16_at(s_addr, reg, val);
}

int ina226_bus_scan(uint8_t *found, int max) {
    int cnt = 0;
    for (uint8_t a = 0x08; a <= 0x77; ++a) {
        uint8_t dummy;
        /* 1 byte 讀取探測；有 ACK 就代表位址上有裝置 */
        int n = i2c_read_timeout_us(I2C_PORT, a, &dummy, 1, false, I2C_TIMEOUT_US);
        if (n == 1) {
            if (found && cnt < max) {
                found[cnt] = a;
            }
            cnt++;
        }
    }
    return cnt;
}

bool ina226_init(void) {
    s_present = false;
    s_addr    = 0;

    i2c_init(I2C_PORT, INA226_I2C_BAUD);
    gpio_set_function(INA226_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(INA226_I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(INA226_I2C_SDA_PIN);
    gpio_pull_up(INA226_I2C_SCL_PIN);

    /* ALERT 腳設為輸入。本版用途是 Conversion Ready（讀取時序確定、不漏樣本）。 */
    gpio_init(INA226_ALERT_PIN);
    gpio_set_dir(INA226_ALERT_PIN, GPIO_IN);
    gpio_pull_up(INA226_ALERT_PIN);   /* ALERT 為 open-drain active-low */

    /*
     * 掃描 INA226 可能的位址範圍（A0/A1 兩腳共 16 種組合 = 0x40..0x4F），
     * 靠 Manufacturer ID 確認真的是 INA226，避免誤把別的裝置當成它。
     */
    for (uint8_t a = 0x40; a <= 0x4F; ++a) {
        uint16_t mfg = 0;
        if (!rd16_at(a, INA226_REG_MFG_ID, &mfg)) {
            continue;
        }
        if (mfg == INA226_MFG_ID_TI) {
            s_addr = a;
            break;
        }
    }

    if (s_addr == 0) {
        return false;
    }

    if (!wr16(INA226_REG_CONFIG, INA226_CONFIG_VALUE)) {
        return false;
    }
    if (!wr16(INA226_REG_CALIBRATION, INA226_CAL_VALUE)) {
        return false;
    }
    if (!wr16(INA226_REG_MASK_EN, INA226_MASK_CNVR)) {
        return false;
    }

    /* 回讀 CALIBRATION 確認寫進去了 —— 校正值錯的話電流讀數會整個歪掉 */
    uint16_t cal_back = 0;
    if (!rd16(INA226_REG_CALIBRATION, &cal_back) || cal_back != INA226_CAL_VALUE) {
        return false;
    }

    s_present = true;
    return true;
}

bool ina226_present(void) {
    return s_present;
}

uint8_t ina226_addr(void) {
    return s_addr;
}

bool ina226_read(int32_t *bus_mV, int32_t *current_mA, int32_t *power_mW) {
    if (!s_present) {
        return false;
    }

    uint16_t vbus_raw = 0, cur_raw = 0, pwr_raw = 0;

    /*
     * 讀取失敗只回 false，不清除 s_present。
     * 一次匯流排雜訊造成的暫時性逾時不該讓電流監測永久失效 —— 讓它下個週期
     * 自己重試就會恢復。真的掉線的話，錯誤會持續累加到 $ST 的 i2c_err_cnt，
     * 加上 flags bit0 一直亮著，照樣看得出來。
     * 每次讀取最多 3 × 2ms 逾時 = 6ms，仍遠短於 100ms 的回報週期。
     */
    if (!rd16(INA226_REG_BUS_V, &vbus_raw))   { return false; }
    if (!rd16(INA226_REG_CURRENT, &cur_raw))  { return false; }
    if (!rd16(INA226_REG_POWER, &pwr_raw))    { return false; }

    /* Bus voltage LSB = 1.25 mV */
    if (bus_mV) {
        *bus_mV = (int32_t)(((uint32_t)vbus_raw * 125u) / 100u);
    }

    /* Current LSB = 1 mA，暫存器為二補數有號 */
    if (current_mA) {
        *current_mA = (int32_t)(int16_t)cur_raw;
    }

    /* Power LSB = 25 mW，暫存器為無號 */
    if (power_mW) {
        *power_mW = (int32_t)((uint32_t)pwr_raw * INA226_POWER_LSB_MW);
    }

    return true;
}

uint32_t ina226_err_count(void) {
    return s_err;
}
