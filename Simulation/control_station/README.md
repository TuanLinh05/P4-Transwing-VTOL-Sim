# P4 Transwing Control Station

Dashboard cục bộ này được chuyển thể từ MiniHawk để điều phối một stack P4:

- Gazebo Harmonic server và GUI với model `p4_transwing_ardupilot`;
- ArduPlane QuadPlane SITL và MAVProxy;
- telemetry, thao tác bay và mission waypoint qua MAVLink;
- Mission Planner tùy chọn.

Xem [hướng dẫn cài đặt đầy đủ](../SETUP.md) trước khi chạy trên máy mới.

## Khởi động

Từ Windows, nhấp đúp `P4-Transwing-Control-Station.cmd` ở thư mục gốc. Launcher:

1. tự chọn Ubuntu WSL 22.04/24.04;
2. tự xác định repository, kể cả khi đường dẫn chứa khoảng trắng;
3. chạy preflight dependency/SDF/cổng;
4. mở backend tại `http://127.0.0.1:8765`.

Chọn cấu hình `baseline`, nhấn **START SIMULATION** rồi đợi Gazebo, SITL và
MAVLink chuyển sang sẵn sàng. Máy bay luôn bắt đầu **DISARMED**. Live weather
và payload sweep vẫn bị khóa. Chuyển tiếp cánh bằng có một luồng thử giới hạn
đã được nhận dạng cho baseline; đây vẫn là mô phỏng surrogate, không phải tham
số dùng cho máy bay thật.

## Luồng vận hành an toàn

1. Kiểm tra GPS, mode, attitude và cảnh báo PreArm.
2. Ở trạng thái DISARMED, xác nhận đúng chiều rotor, cánh gập, aileron và
   V-tail.
3. Cất cánh QLOITER tới ít nhất 30 m và chờ thân cân bằng.
4. Nhấn **CHUYỂN CÁNH BẰNG**. Dashboard chỉ dùng FBWB/RC3 1750 để lấy tốc độ
   chuyển tiếp; khi ArduPilot báo hoàn tất nó tự hạ về RC3 1470 (cruise ~34 m/s),
   đồng thời giám sát độ cao và tư thế rồi tự quay về QLOITER nếu vượt giới hạn.
5. Khi đã báo hoàn tất, dùng FBWB hoặc **TỰ ĐỘNG LEO/HẠ**. Nhấn
   **ABORT → QLOITER** để quay lại VTOL.
6. Dùng **QLAND** để hạ thẳng đứng; chỉ dùng RTL khi mission/home đã đúng.
7. Kết thúc bằng **STOP STACK** để chỉ đóng các tiến trình do dashboard tạo.

Dashboard chỉ bind `127.0.0.1`, không mở quyền điều khiển ra LAN. Các thao tác
ARM, takeoff và AUTO đều yêu cầu xác nhận.

## Log và phục hồi

Log launcher:

```text
%LOCALAPPDATA%\P4 Transwing\control-station.log
%LOCALAPPDATA%\P4 Transwing\control-station-error.log
```

Đóng tab trình duyệt không dừng mô phỏng. Mở lại
`http://127.0.0.1:8765` để kết nối và nhấn **STOP STACK**. Nếu một lần khởi
động thất bại, preflight tích hợp sẽ ghi rõ dependency, SDF hoặc cổng nào cần
xử lý trước khi Gazebo/SITL được spawn.
