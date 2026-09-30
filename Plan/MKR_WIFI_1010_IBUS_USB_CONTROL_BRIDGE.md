# FlySky FS-i6 → MKR WiFi 1010 → P4 Control Station (RC-only)

## 1. Phạm vi đã chốt

MKR WiFi 1010 chỉ đọc 10 kênh iBUS của FS-iA6B và chuyển chúng qua USB cho
P4 Control Station. Telemetry trả về tay cầm đã được bỏ vì firmware FS-i6
10CH không hiển thị ổn định các loại sensor iBUS nâng cao.

Luồng duy nhất còn lại:

```text
FS-i6 → FS-iA6B iBUS-SERVO → MKR D13 → USB → bridge_mkr_usb.py
      → Control Station → MAVLink RC override → ArduPilot SITL
```

GUI/Mission Planner vẫn hiển thị đầy đủ mode, độ cao và groundspeed từ MAVLink.
Tay cầm chỉ dùng điều khiển.

## 2. Đấu dây

| FS-iA6B | Arduino MKR WiFi 1010 | Ghi chú |
|---|---|---|
| iBUS-SERVO signal | `D13` | Serial1 RX, dữ liệu RC 10 kênh |
| GND | `GND` | Bắt buộc dùng chung mass |
| USB | PC Windows | Cấp nguồn và giao tiếp bridge |

Không nối cổng **SENS**. D0, D1 và D14 để trống.

> Cần xác nhận tín hiệu iBUS là logic 3.3 V trước khi nối vào MKR.

## 3. Firmware MKR

Nạp sketch [Controller.ino](../MCU/Controller/Controller.ino) bằng Arduino IDE:

- Board: **Arduino MKR WiFi 1010**
- Protocol USB: `P4RC/1`, 100 Hz
- Không có `P4TEL`, SENS, SERCOM phụ hay telemetry reply.
- Timeout iBUS 150 ms: xuất CH3 = 1000, các kênh khác = 1500, đặt cờ failsafe.

Mỗi frame có dạng:

```text
P4RC,1,sequence,millis,ch1,ch2,ch3,ch4,ch5,ch6,ch7,ch8,ch9,ch10,flags,CRC16
```

## 4. Gán kênh FlySky

| Kênh | Chức năng trong mô phỏng |
|---|---|
| CH1 | Roll |
| CH2 | Pitch |
| CH3 | Throttle |
| CH4 | Yaw |
| CH5–CH6 | Aux, chỉ hiển thị |
| CH7 / SWA | ARM: LOW → HIGH, chỉ khi CH3 thấp |
| CH8 / SWB | QRTL khi HIGH |
| CH9 / SWC | LOW QLOITER; MID transition; HIGH FBWB |
| CH10 | Spare, chỉ hiển thị |

Control Station kiểm tra trạng thái trước khi gửi các lệnh quan trọng. CH7–CH9
không bypass state machine chuyển tiếp của P4 Transwing.

## 5. Chạy trên Windows

1. Đóng Arduino Serial Monitor để nhả COM port.
2. Khởi động P4 Control Station như bình thường.
3. Mở PowerShell tại thư mục project và chạy:

```powershell
py -3 Simulation\tools\bridge_mkr_usb.py --port COM26
```

Đổi `COM26` thành cổng MKR thực tế. Log ổn định sẽ có:

```text
P4RC USB ingest rate: ~100 Hz
P4RC forward rate: ~10 Hz
```

Forward rate thấp hơn ingest rate là bình thường: bridge chỉ gửi trạng thái RC
mới nhất để HTTP chậm không làm mất frame iBUS. Control Station nhận được frame
còn mới trong giới hạn 750 ms.

### Máy bị chặn cài Python

Nếu Windows báo `exit code: 1625` hoặc `Access is denied` với Python Store,
dùng bridge PowerShell tích hợp sẵn, không cần cài thêm Python:

```powershell
PowerShell -ExecutionPolicy Bypass -File Simulation\tools\bridge_mkr_usb.ps1 -Port COM26
```

Bridge này vẫn đọc P4RC ở 115200 baud và gửi trạng thái mới nhất lên Control
Station khoảng 10 Hz.

## 6. Trình tự bay an toàn

1. TX bật, throttle thấp; SWA/SWB/SWC ở LOW.
2. Chạy bridge, chờ GUI hiện **MKR ready** và CH1–CH10 thay đổi.
3. Bấm **TRAO QUYỀN RADIO**.
4. Dùng SWA LOW → HIGH để ARM.
5. Dùng SWC cho QLOITER / transition / FBWB; dùng SWB HIGH khi cần QRTL.
6. Sau khi hạ cánh, đưa throttle thấp và SWA về LOW để DISARM.

Nếu USB/iBUS mất quá 750 ms, Control Station giữ RC an toàn và yêu cầu QLOITER.

## 7. Kiểm tra trước demo

- Không còn dây SENS nối vào MKR.
- Arduino Serial Monitor đã đóng trước khi chạy bridge.
- GUI hiển thị đúng CH1–CH10 và trạng thái switch CH7–CH9.
- `P4RC USB ingest rate` xấp xỉ 100 Hz.
- Thử ARM/DISARM khi máy bay ở mặt đất trước khi bay.

Sketch [Telemetry.ino](../MCU/Controller/Telemetry/Telemetry.ino) được giữ lại
nguyên vẹn làm thử nghiệm phần cứng độc lập, nhưng không dùng trong cấu hình
bay/demo RC-only này.
