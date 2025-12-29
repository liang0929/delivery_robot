#!/usr/bin/env python3
"""
Modbus 馬達驅動器連接測試腳本
測試 USB-RS232 與 AGV-BLD-2S 驅動器的通訊
"""

import sys
import time
from pymodbus.client import ModbusSerialClient
from pymodbus.exceptions import ModbusException

# 配置參數
SERIAL_PORT = '/dev/ttyUSB0'
BAUDRATE = 115200
SLAVE_ID = 1

# 寄存器地址
ADDR_MOTOR_A_STATE = 2002
ADDR_MOTOR_B_STATE = 2003
ADDR_MOTOR_A_SPEED_PV = 1002
ADDR_MOTOR_B_SPEED_PV = 1003
ADDR_FAULT_CODE = 1005


def test_connection():
    print("=" * 50)
    print("Modbus 馬達驅動器連接測試")
    print("=" * 50)
    print(f"串口: {SERIAL_PORT}")
    print(f"波特率: {BAUDRATE}")
    print(f"Slave ID: {SLAVE_ID}")
    print("=" * 50)

    # 建立連接
    print("\n[1/5] 建立 Modbus 連接...")
    client = ModbusSerialClient(
        port=SERIAL_PORT,
        baudrate=BAUDRATE,
        parity='N',
        stopbits=1,
        bytesize=8,
        timeout=2.0
    )

    if not client.connect():
        print(f"❌ 無法開啟串口 {SERIAL_PORT}")
        print("\n可能原因:")
        print("  - USB-RS232 轉接線未插入")
        print("  - 串口被其他程式佔用")
        print("  - 權限不足 (嘗試: sudo chmod 666 /dev/ttyUSB0)")
        return False

    print(f"✓ 串口 {SERIAL_PORT} 開啟成功")

    # 測試讀取故障碼
    print("\n[2/5] 讀取故障碼...")
    try:
        result = client.read_input_registers(ADDR_FAULT_CODE, 1, slave=SLAVE_ID)
        if result.isError():
            print(f"❌ 讀取失敗: {result}")
            print("\n可能原因:")
            print("  - TX/RX 接線錯誤 (需交叉連接)")
            print("  - Slave ID 不正確")
            print("  - 驅動器未設定為 Modbus 模式")
            client.close()
            return False
        fault_code = result.registers[0]
        print(f"✓ 故障碼: {fault_code} {'(正常)' if fault_code == 0 else '(有故障!)'}")
    except ModbusException as e:
        print(f"❌ Modbus 異常: {e}")
        client.close()
        return False

    # 測試讀取馬達轉速
    print("\n[3/5] 讀取馬達轉速...")
    try:
        result = client.read_input_registers(ADDR_MOTOR_A_SPEED_PV, 2, slave=SLAVE_ID)
        if result.isError():
            print(f"❌ 讀取失敗: {result}")
            client.close()
            return False
        rpm_a = result.registers[0]
        rpm_b = result.registers[1]
        print(f"✓ A馬達轉速: {rpm_a} RPM")
        print(f"✓ B馬達轉速: {rpm_b} RPM")
    except ModbusException as e:
        print(f"❌ Modbus 異常: {e}")
        client.close()
        return False

    # 測試寫入 (啟用馬達)
    print("\n[4/5] 測試寫入 (啟用馬達)...")
    try:
        result = client.write_register(ADDR_MOTOR_A_STATE, 1, slave=SLAVE_ID)
        if result.isError():
            print(f"❌ 寫入失敗: {result}")
            client.close()
            return False
        print("✓ A馬達啟用成功")

        result = client.write_register(ADDR_MOTOR_B_STATE, 1, slave=SLAVE_ID)
        if result.isError():
            print(f"❌ 寫入失敗: {result}")
            client.close()
            return False
        print("✓ B馬達啟用成功")
    except ModbusException as e:
        print(f"❌ Modbus 異常: {e}")
        client.close()
        return False

    # 馬達轉動測試 (可選)
    print("\n[5/5] 馬達轉動測試")
    user_input = input("是否要測試馬達轉動? (y/N): ").strip().lower()

    if user_input == 'y':
        print("\n⚠️  警告: 馬達即將轉動，請確保安全!")
        print("按 Enter 繼續，或 Ctrl+C 取消...")
        try:
            input()
        except KeyboardInterrupt:
            print("\n已取消")
            client.close()
            return True

        try:
            # 設定方向 (正轉)
            client.write_register(2004, 0, slave=SLAVE_ID)  # A馬達方向
            client.write_register(2005, 0, slave=SLAVE_ID)  # B馬達方向

            # 設定轉速 200 RPM
            print("設定轉速 200 RPM...")
            client.write_register(2006, 200, slave=SLAVE_ID)  # A馬達轉速
            client.write_register(2007, 200, slave=SLAVE_ID)  # B馬達轉速

            # 等待 2 秒
            for i in range(4):
                time.sleep(0.5)
                result = client.read_input_registers(ADDR_MOTOR_A_SPEED_PV, 2, slave=SLAVE_ID)
                if not result.isError():
                    print(f"  實際轉速: A={result.registers[0]} RPM, B={result.registers[1]} RPM")

            # 停止馬達
            print("停止馬達...")
            client.write_register(2006, 0, slave=SLAVE_ID)
            client.write_register(2007, 0, slave=SLAVE_ID)
            print("✓ 馬達轉動測試完成")

        except ModbusException as e:
            print(f"❌ 馬達測試異常: {e}")
        except KeyboardInterrupt:
            # 緊急停止
            client.write_register(2006, 0, slave=SLAVE_ID)
            client.write_register(2007, 0, slave=SLAVE_ID)
            print("\n已緊急停止")

    # 關閉連接
    client.close()
    print("\n" + "=" * 50)
    print("✓ 連接測試完成!")
    print("=" * 50)
    return True


if __name__ == '__main__':
    try:
        success = test_connection()
        sys.exit(0 if success else 1)
    except KeyboardInterrupt:
        print("\n已取消")
        sys.exit(1)
