/*
 * proto.h — 序列協定編碼／解碼
 *
 * 協定為 ASCII 行式 + XOR 校驗，選 ASCII 是為了能直接 `cat /dev/ttyACM0` 除錯。
 * 格式：$<payload>*<XX>\n
 *   <XX> = payload 每個字元的 XOR，兩位大寫十六進位
 *   校驗範圍不含 '$' 與 '*'
 */
#ifndef PROTO_H
#define PROTO_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/* 一行 payload 的上限（不含 '$'、'*XX'、'\n'） */
#define PROTO_PAYLOAD_MAX 192

/* 計算 payload 的 XOR 校驗 */
uint8_t proto_xor(const char *payload);

/*
 * 組裝並送出一行。
 *
 * 非阻塞保證：USB CDC 未連線時直接丟棄並累加計數器，不進入寫入路徑；
 * 已連線時走 pico-sdk 的 stdio_usb，其寫入有 PICO_STDIO_USB_STDOUT_TIMEOUT_US
 * 逾時保護（本專案在 CMakeLists.txt 收斂為 20ms，遠小於 100ms 的主迴圈週期）。
 */
void proto_send(const char *payload_fmt, ...) __attribute__((format(printf, 1, 2)));

/* 因 USB 未連線而被丟棄的行數 */
uint32_t proto_dropped_lines(void);

/*
 * 驗證並拆解一行輸入。
 * line 形如 "$CMD,PING*3F"（尾端的 \r\n 應已由呼叫端去除）。
 * 成功時把 payload（不含 '$' 與 '*XX'）複製到 out，回傳 true。
 * 校驗錯誤、格式錯誤一律回 false —— 但呼叫端仍應回 $NAK 而非當機。
 */
bool proto_parse_line(const char *line, char *out, size_t out_sz);

#endif /* PROTO_H */
