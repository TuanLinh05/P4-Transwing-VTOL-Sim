# Thông số tham chiếu và kế hoạch mô hình P4 Transwing

Tài liệu này tách riêng ba loại dữ liệu:

- **Published:** công bố trực tiếp trong X-P4 UAFM hoặc tài liệu PteroDynamics;
- **Measured:** đo từ animation/geometry trong `Ref/P4_clone`;
- **Estimated:** lựa chọn kỹ thuật của mô hình Gazebo, chưa phải thông số P4 thật.

Nguồn máy đọc tương ứng nằm tại
`Simulation/config/p4_reference.json`.

## 1. Nguồn tham chiếu

1. [X-P4 Unmanned Aircraft Flight Manual, Revision D](https://downloads.regulations.gov/FAA-2024-2404-0001/attachment_10.pdf)
2. [PteroDynamics – Game Changing Autonomous VTOL Systems, 2025](https://pterodynamics.com/media/PD_TranswingSpecs_2025.pdf)
3. [Transwing by Tim Whitehand – ArduPilot, mốc 00:09](https://www.youtube.com/watch?v=-QAy2u33BEc&t=9s)
4. Animation/mesh cục bộ trong `Ref/P4_clone`.

Video năm 2021 là bài trình bày về cơ học Transwing. Slide ở 00:09 không ghi
rõ đây là envelope của X-P4 Rev. D, vì vậy các con số chuyển tiếp trong slide
không được dùng thay cho bảng tốc độ chính thức của X-P4.

## 2. Thông số X-P4 được công bố

### Hình học, khối lượng và hiệu năng

| Thông số | UAFM Rev. D | SI | Trạng thái |
|---|---:|---:|---|
| MTOW | 89 lb | 40.370 kg | Published |
| Sải cánh | 13.1 ft | 3.99288 m | Published |
| Diện tích cánh | 9 ft² | 0.836127 m² | Published |
| Aspect ratio | 19 | 19 | Published |
| Wing incidence | +3.2° root → +0.9° tip | — | Published |
| Wing washout | 0° | — | Published |
| Airfoil đuôi | NACA0012 | — | Published |
| Chiều dài thân | 6.4 ft | 1.95072 m | Published |
| Bề rộng khi gập | 4.2 ft | 1.28016 m | Published |
| BEW, gồm pin | 43.6 lb | 19.777 kg | Published |
| Payload | 15 lb | 6.804 kg | Published |
| Khối lượng pin | 28.7 lb | 13.018 kg | Published |
| V_stall | 52 kt | 26.751 m/s | Published |
| V_loiter | 55 kt | 28.294 m/s | Published |
| V_cruise | 60–65 kt | 30.867–33.439 m/s | Published |
| V_dash | 100 kt | 51.444 m/s | Published |
| Range | 60 NM | 111.12 km | Published |
| Endurance | 1 giờ | 60 phút | Published |
| Ceiling | 15,000+ ft | 4,572+ m | Published |

Brochure PteroDynamics năm 2025 cập nhật P4 thành MTOW khoảng **41 kg**, payload
**6.8 kg**, footprint **2.1 × 1.3 m**, cruise **31 m/s**, dash **51 m/s** và
endurance **70 phút**. Sai khác nhỏ với UAFM được giữ nguyên theo từng revision,
không trộn thành một con số “chính xác giả”.

### Propulsion và control surface

- 4 × Hacker Q80-7M-V2, 210 KV, 5.5 kW, 1.035 kg/motor, motor limit 9,000 rpm;
- 4 × Currawong Velocity HS ESC, 60 V, 150 A liên tục / 300 A peak;
- propeller folding GM 22 × 15 inch, 7,000+ rpm liên tục;
- Motor 1: Right Inboard; Motor 2: Right Outboard;
  Motor 3: Left Outboard; Motor 4: Left Inboard;
- bốn aileron độc lập và hai ruddervator;
- VTOL dùng cả bốn propulsor;
- fixed-wing cruise tắt hai motor để tăng hiệu suất; có thể dùng cặp inboard
  hoặc outboard.

Khối lượng `0.2 kg` của mỗi rotor link trong SDF là ước lượng cho phần quay
propeller/hub. Không được hiểu là khối lượng toàn bộ motor; stator, ESC, pin và
nacelle đang được gộp vào wing assembly.

## 3. Dữ kiện chuyển tiếp từ video tại 00:09

Slide **“TRANSITION MECHANICS – 50%”** cho biết:

- ETL và wing lift sau khoảng 30% hành trình giúp giảm đáng kể công suất rotor;
- tại 50% transition: airspeed khoảng **10–20 m/s**;
- AoA phải **dưới 20°**;
- propulsor có cross-coupling, pitch authority yếu và cần nhiều roll damping;
- control surface đã hoạt động;
- elevator là điều khiển pitch chính;
- aileron quan trọng để dập roll oscillation.

### Ánh xạ vào surrogate hiện tại

| Dữ kiện video | Mô hình hiện tại | Đánh giá |
|---|---|---|
| 50% transition | `-1.029716127 rad` | Khớp midpoint animation |
| Giữ ở 50% khi tăng tốc | `Q_TILT_MAX 45` | Khớp normalized tilt của ArduPilot |
| 10–20 m/s | Mốc giữa chuyển tiếp | Không dùng làm tốc độ bỏ lực nâng VTOL |
| AoA < 20° | `alpha_stall 0.28 rad` ≈ 16.0° | Bảo thủ hơn slide |
| Surfaces active | 4 aileron + 2 ruddervator | Đã mô hình |

`AIRSPEED_MIN 27` của SITL hiện bám sát V_stall 26.751 m/s công bố trong UAFM;
TECS dùng mục tiêu 31 m/s. Đây vẫn là lịch chuyển tiếp của mô hình ước lượng,
không phải tham số an toàn cho phần cứng thật. Mốc 10–20 m/s trong video chỉ
được giữ làm dữ kiện giữa chuyển tiếp.

## 4. Cơ cấu gập và mô hình hình học

UAFM mô tả một linear actuator có motor/gearbox, lead screw, shuttle, spreader
arm và hai control rod. Controller thật có position feedback, homing và fault
thresholds. Mô hình hiện tại rút gọn cơ cấu đó thành hai revolute joint đồng bộ:

- hover: `0 rad`;
- transition 50%: `-1.029716127 rad`;
- cruise: `-2.059432253 rad`;
- tổng hành trình đo từ animation: `2.059432253 rad` ≈ `117.997°`.

Rút gọn này giữ đúng pose cánh/rotor để phát triển controller, nhưng chưa mô
phỏng backlash, lead-screw dynamics, linkage load, homing hoặc lệch cánh.

## 5. Những gì chưa nên thay đổi từ video

Không thay trực tiếp các giá trị sau chỉ dựa trên slide:

- mass distribution và inertia;
- `motorConstant`/reaction torque;
- center of gravity;
- lift/drag coefficients;
- transition rate;
- fixed-wing PID và hover PID.

Các giá trị này cần dữ liệu thrust stand, CAD mass properties, wind tunnel hoặc
flight log. Việc gắn nhãn nguồn quan trọng hơn việc đưa thêm số có vẻ chính xác.

## 6. Khoảng trống cần triển khai

1. Mô phỏng airspeed/pitot SITL và xác minh trước khi bật `ARSPD_TYPE=100`.
2. Xây dựng scheduler transition có ràng buộc AoA, airspeed và timeout.
3. Thêm logic two-motor cruise; không dùng đồng thời bốn motor như cấu hình
   tilt-quad chuẩn sau khi chuyển hoàn toàn sang fixed wing.
4. Thêm actuator position feedback, wing synchronization và fault injection.
5. Nhận dạng rotor thrust/torque, CG, inertia và aero coefficient.
6. Tune roll damping ở transition, sau đó mới mở automatic transition.

## 7. Trạng thái quyết định

Video mới **xác nhận hướng thiết kế hiện tại**, đặc biệt ở midpoint transition,
airspeed mô phỏng, AoA bảo thủ và việc kích hoạt control surface. Chưa có đủ
căn cứ để đổi hệ số vật lý. Các thay đổi thực hiện ở vòng này là chuẩn hóa nguồn
dữ liệu, bổ sung thông số propulsion/envelope thật và ghi rõ những sai khác cần
được triển khai sau.
