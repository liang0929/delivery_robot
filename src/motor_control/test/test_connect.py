import serial  # 引用pySerial模組
import json
COM_PORT = '/dev/ttyTHS1'    # 指定通訊埠名稱
BAUD_RATES = 115200    # 設定傳輸速率
ser = serial.Serial(COM_PORT, BAUD_RATES)   # 初始化序列通訊埠

try:
    while True:
        while ser.in_waiting:          # 若收到序列資料…
            data_raw = ser.readline()  # 讀取一行
            data = data_raw.decode()   # 用預設的UTF-8解碼
            start_idx = data.find('::')
            end_idx = data.find('::', start_idx + 2)
            payload = data[start_idx + len('::'):end_idx]
            print('接收到的原始資料：', data_raw)
            json_data = json.loads(payload)
            print('接收到的資料：', json_data.get('type'))

except KeyboardInterrupt:
    ser.close()    # 清除序列通訊物件
    print('再見！')
