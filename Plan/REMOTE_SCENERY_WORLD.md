# World cảnh quan tùy chọn cho máy remote

## Bản XL 1 km (remote mạnh)

Để có bản đồ rộng khoảng 1 km với nhiều nhà và cao ốc hơn, dùng launcher riêng:

```text
P4-Transwing-Scenery-XL-Remote.cmd -RenderMode Gpu
```

Bản XL dùng world `Simulation/gazebo/worlds/p4_ardupilot_scenery_xl.sdf` và model
`Simulation/gazebo/models/p4_scenery_xl`. Texture bản đồ nền đã được bỏ khỏi world;
Gazebo chỉ dựng các hình học 3D gồm mạng đường, hơn 100 khối nhà, cụm cao ốc
48--96 m, công viên, kênh nước, cây và đèn đường (705 visual). Tất cả cảnh quan vẫn visual-only,
không có collision, nên không làm thay đổi động lực học hay thuật toán bay.

Nếu máy remote không đủ FPS, dùng lại `P4-Transwing-Scenery-Remote.cmd` (bản 400 m)
hoặc launcher mặc định để chạy world nhẹ.

World mặc định `p4_ardupilot.sdf` vẫn là bản nhẹ để chạy trên máy local. Bản
cảnh quan dùng world riêng `p4_ardupilot_scenery.sdf` và model
`p4_scenery`; các tòa nhà, đường và cây chỉ có visual, không có collision nên
không làm thay đổi động lực học bay.

Bản hiện tại có hơn 300 visual low-poly: một khu đô thị mở rộng, mạng đường,
heliport, công viên, tháp cao, đèn đường và các dải rừng. Một texture bản đồ
đô thị gốc cũng được đặt dưới cảnh 3D để dễ quan sát đường bay từ trên cao.
Đây là bản nặng hơn để chạy trên máy remote nhưng vẫn nhẹ hơn nhiều so với mesh
thành phố thật.

## Chạy trên Windows remote

Đồng bộ project sang máy remote, dừng stack đang chạy rồi mở:

```text
P4-Transwing-Scenery-Remote.cmd
```

Nếu remote có GPU/WSLg tốt, có thể chọn:

```text
P4-Transwing-Scenery-Remote.cmd -RenderMode Gpu
```

Muốn quay lại world nhẹ, dùng `P4-Transwing-Control-Station.cmd` như trước.

## Chạy trực tiếp trong WSL

```bash
bash Simulation/control_station/start_control_station_scenery_wsl.sh
```

Hoặc chỉ chạy Gazebo server cảnh quan để ghép với các tiến trình riêng:

```bash
bash Simulation/gazebo/run_p4_ardupilot_scenery_server_wsl.sh
```

GUI Gazebo vẫn dùng `Simulation/gazebo/run_p4_gui_wsl.sh` và phải chạy cùng
`GZ_PARTITION` với server.

## Điều chỉnh cảnh

Các primitive low-poly nằm trong:

```text
Simulation/gazebo/models/p4_scenery/model.sdf
```

Texture nền ở `Simulation/gazebo/models/p4_scenery/materials/textures/urban_aerial_diffuse.png`.

Có thể đổi pose/kích thước/màu của building và tree. Vì model không có
`<collision>`, drone sẽ bay xuyên qua cảnh; đây là chủ ý để quan sát đường bay
và giữ mô phỏng ổn định. Nếu cần kiểm tra tránh vật cản, nên tạo một world
collision riêng sau khi đã chốt cấu hình bay.
