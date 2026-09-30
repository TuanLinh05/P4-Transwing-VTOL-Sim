# P4 reference plan

Tài liệu chính: [Plan.md](Plan.md).

Nội dung được tổ chức theo bốn mức:

- thông số PteroDynamics/X-P4 đã công bố;
- dữ kiện transition từ video ArduPilot năm 2021;
- hình học đo từ `Ref/P4_clone`;
- giả định kỹ thuật đang dùng trong Gazebo.

File máy đọc: `Simulation/config/p4_reference.json`.

Nguyên tắc: không chuyển một con số từ presentation thành tham số bay X-P4 nếu
chưa xác nhận đúng airframe/revision và chưa kiểm tra trong mô phỏng.
