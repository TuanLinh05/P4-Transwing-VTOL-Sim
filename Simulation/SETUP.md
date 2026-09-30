# Cài đặt và chạy mô phỏng P4 Transwing

Project gồm Gazebo Harmonic, plugin JSON của `ardupilot_gazebo`, ArduPlane
QuadPlane SITL và Control Station chạy cục bộ. Đây là mô hình surrogate để phát
triển logic điều khiển và chuyển tiếp; chưa phải digital twin đã được nhận dạng
khí động học từ P4 thật.

## Nền tảng được hỗ trợ

- Windows 11 với WSL2 và WSLg;
- Ubuntu WSL 22.04 hoặc 24.04;
- Gazebo Harmonic / Gazebo Sim 8.x;
- GPU hoặc chế độ render software mặc định, phù hợp cả máy không có driver
  OpenGL ổn định trong WSL.

Môi trường đã chạy kiểm chứng đầy đủ:

| Thành phần | Phiên bản |
|---|---|
| Ubuntu | 22.04 |
| Gazebo Sim | 8.14.0 |
| ArduPilot | commit `4f50f1997fec263e648119402991beda1071bc7c` |
| ardupilot_gazebo | commit `082a0fe231f6e63bc8d1598f1cba461d9e2ea7f5` |
| Python / pymavlink / MAVProxy | 3.10.12 / 2.4.49 / 1.8.74 |

Nguồn phiên bản chuẩn nằm trong
`Simulation/config/compatibility.json`. Gazebo sai major version là lỗi; commit
ArduPilot khác bản đã thử sẽ phát cảnh báo để vẫn cho phép phát triển có chủ đích.

Tài liệu upstream:

- [Cài Gazebo Harmonic trên Ubuntu](https://gazebosim.org/docs/harmonic/install_ubuntu/)
- [ardupilot_gazebo chính thức](https://github.com/ArduPilot/ardupilot_gazebo)
- [Thiết lập môi trường build ArduPilot](https://ardupilot.org/dev/docs/building-setup-linux.html)
- [Thiết lập SITL trên Linux](https://ardupilot.ardupilot.org/dev/docs/setting-up-sitl-on-linux.html)

## Cài trên máy mới

### 1. Cài WSL2/Ubuntu

Cài Ubuntu 22.04 hoặc 24.04 từ Windows, mở distro một lần và tạo tài khoản
Linux. Bảo đảm lệnh sau thấy distro:

```powershell
wsl.exe --list --verbose
```

### 2. Cài dependency tái lập

Trong terminal Ubuntu WSL, đi đến repository (vị trí có thể tùy ý, kể cả đường
dẫn chứa khoảng trắng), xem script rồi chạy:

```bash
cd "/mnt/d/duong-dan/P4 Transwing Clone Drone"
less Simulation/tools/install_wsl_dependencies.sh
bash Simulation/tools/install_wsl_dependencies.sh --yes
```

Script dùng `sudo`, `apt` và mạng; nó cài Gazebo Harmonic, clone đúng hai commit
đã kiểm chứng, build ArduPlane SITL cùng `libArduPilotPlugin.so`, rồi tự chạy
preflight sâu. Để bảo vệ source đang có, script sẽ dừng nếu
`$HOME/ardupilot` hoặc `$HOME/ardupilot_gazebo` đã tồn tại.

Nếu đã cài ArduPilot ở nơi khác, không chạy installer. Trong WSL, export:

```bash
export P4_ARDUPILOT_HOME="/duong/dan/ardupilot"
export P4_ARDUPILOT_GAZEBO_HOME="/duong/dan/ardupilot_gazebo"
```

Sau đó chạy preflight trong cùng terminal.

### 3. Kiểm tra trước khi chạy

Từ Windows, nhấp đúp:

```text
P4-Transwing-Preflight.cmd
```

Hoặc chạy bản sâu:

```powershell
.\P4-Transwing-Preflight.cmd -Deep
```

Preflight tự tìm Ubuntu phù hợp và tự chuyển đường dẫn repository sang WSL. Nó
kiểm tra dependency, phiên bản, shared library, model/texture, SDF, quyền ghi
state, cổng mạng và unit test. Không cần sửa đường dẫn `/mnt/d/...` trong source.

Nếu có nhiều distro Ubuntu:

```powershell
$env:P4_TRANSWING_WSL_DISTRO = "Ubuntu-24.04"
.\P4-Transwing-Preflight.cmd -Deep
```

## Đồng bộ từ máy phát triển qua Remote Desktop

Khi làm việc qua Remote Desktop và đã bật drive redirection, nhấp đúp
`P4-Transwing-Sync.cmd` trên máy remote. Cửa sổ này giữ một đồng bộ một chiều
từ `\\tsclient\D\Intern_Data\P4 Transwing Clone Drone` sang thư mục project
trên máy remote, kiểm tra mỗi 5 giây. Nhấn `Ctrl+C` để dừng.

Script dùng Robocopy incremental, **không** dùng `/MIR`: file thay đổi và file
mới từ máy phát triển sẽ được chép sang, còn DataFlash/SITL state, log và các
file chỉ có ở máy remote không bị xóa. Sau một lần sync, chạy lại preflight
rồi restart Control Station để nạp thay đổi code/configuration.

Khi đã sửa file trực tiếp trên máy remote và muốn đưa thay đổi về máy phát
triển, nhấp đúp `P4-Transwing-Push-To-Host.cmd` **trên máy remote**. Push là
một lần, chỉ chép file remote mới hơn (`/XO` bảo vệ file mới hơn ở máy phát
triển), không xóa file và không mang SITL state/DataFlash log về source.
Không chạy hai chiều liên tục: nếu hai máy cùng sửa một file, hãy chọn bản cần
giữ hoặc dùng Git để giải quyết conflict một cách tường minh.

Nếu source nằm ở drive/thư mục khác, mở PowerShell trong thư mục project trên
máy remote và thay giá trị `-Source`, ví dụ:

```powershell
.\Simulation\tools\sync_from_tsclient.ps1 -Watch -Source "\\tsclient\E\Projects\P4 Transwing Clone Drone"
```

## Khởi động một nhấp

Nhấp đúp:

```text
P4-Transwing-Control-Station.cmd
```

Launcher luôn chạy preflight trước khi mở backend, ghi log vào
`%LOCALAPPDATA%\P4 Transwing`, rồi mở
`http://127.0.0.1:8765`. Trong dashboard chọn `baseline`, nhấn **START
SIMULATION** và đợi Gazebo, SITL, MAVLink sẵn sàng. Máy bay luôn bắt đầu ở
trạng thái **DISARMED**.

Mission Planner là tùy chọn. Nếu cài bằng installer tiêu chuẩn, launcher tự
nhận diện `C:\Program Files (x86)\Mission Planner\MissionPlanner.exe` và mở nó
cùng stack. Launcher cũng đọc vị trí MSI từ Windows registry khi cài ở thư mục
khác. Với bản portable không đăng ký trong Windows, đặt đường dẫn
**Windows** trước khi chạy:

```powershell
$env:P4_TRANSWING_MISSION_PLANNER = "E:\Mission Planner\MissionPlanner.exe"
.\P4-Transwing-Control-Station.cmd
```

Launcher tự chuyển đường dẫn này sang WSL. Khi không cấu hình, dashboard bỏ
chọn Mission Planner và Gazebo/SITL vẫn hoạt động bình thường.

### Khi dùng qua Remote Desktop

Dashboard trong trình duyệt chính là Control Station; không có thêm một GUI
điều khiển riêng. Sau khi nhấn **START ALL**, hãy chờ tối đa 45 giây vì Gazebo
server và ArduPlane SITL cần khởi tạo trước khi Mission Planner được gọi.
Nếu chỉ thấy biểu tượng Gazebo mà không có cửa sổ, chạy trong Ubuntu:

```bash
bash Simulation/tools/diagnose_remote_wsl.sh
```

Script in ra trạng thái WSLg, tiến trình và log dashboard; gửi toàn bộ output
đó để chẩn đoán. Dashboard sẽ đánh dấu stack `degraded` và ghi lỗi cụ thể nếu
GUI thoát sớm do WSLg/Qt/OpenGL.

Nếu diagnostics cho thấy `gz sim -g` vẫn chạy nhưng chỉ có biểu tượng Gazebo
trên taskbar, yêu cầu WSLg đưa cửa sổ đó lên trước:

```bash
bash Simulation/tools/focus_gazebo_window_wsl.sh
```

Lần đầu nếu thiếu helper, cài `xdotool` và `wmctrl` theo đúng lệnh script in
ra rồi chạy lại lệnh focus.

Các tùy chọn hữu ích:

```powershell
.\P4-Transwing-Control-Station.cmd -NoBrowser
.\P4-Transwing-Control-Station.cmd -Distro Ubuntu-24.04
```

### Gazebo chậm hoặc đứng khung hình qua Remote Desktop

Gazebo server (physics) và Gazebo GUI là hai tiến trình riêng. Nếu Mission
Planner/dashboard vẫn cập nhật nhưng cửa sổ Gazebo chậm hoặc đứng, mô phỏng
vẫn có thể đang chạy; thường là WSLg đang render bằng CPU. Launcher mặc định
dùng software rendering để tương thích tối đa. Trên máy remote có GPU, sau khi
đảm bảo máy bay đã **DISARMED**, nhấn **STOP STACK**, rồi mở Windows PowerShell
và chạy lệnh sau để dừng backend WSL cũ (lệnh này dừng mọi tiến trình Linux
trong WSL):

```powershell
wsl --shutdown
```

Sau đó thử launcher GPU riêng:

```text
P4-Transwing-Control-Station-GPU.cmd
```

Khởi động lại stack trong dashboard rồi so sánh độ mượt. Nếu cửa sổ Gazebo bị
đen, crash hoặc chậm hơn, làm lại hai bước dừng ở trên rồi quay lại
`P4-Transwing-Control-Station.cmd`; lúc đó mô phỏng trở lại software rendering
an toàn.

Chỉ dùng `-SkipPreflight` khi vừa chạy preflight thành công và đang chẩn đoán
launcher.

## Chạy thủ công trong WSL

Mở ba terminal tại repository:

```bash
bash Simulation/gazebo/run_p4_ardupilot_server_wsl.sh
```

```bash
bash Simulation/gazebo/run_p4_gui_wsl.sh
```

```bash
bash Simulation/ardupilot/run_p4_sitl_wsl.sh --wipe
```

Chỉ dùng `--wipe` khi muốn xóa EEPROM mô phỏng P4 và nạp lại toàn bộ
`p4_transwing_sitl.param`. State và DataFlash log được cô lập tại
`Simulation/ardupilot/sitl_state`.

## Kiểm thử

Smoke test tích hợp sau load world/model, kiểm tra IMU, JSON FDM, MAVLink và
tham số điều khiển. Test không ARM:

```bash
bash Simulation/gazebo/smoke_test_p4_ardupilot_wsl.sh
```

Sau khi test DISARMED đạt, có thể chạy bài hồi quy cất cánh thật từ mặt đất
trong Windows PowerShell. Test tự ARM, lên 5 m, kiểm tra hover, QLAND và dừng
stack:

```powershell
.\Simulation\tools\smoke_test_p4_takeoff.ps1 -TargetAltitudeM 5
```

Các test riêng:

```bash
python3 -m unittest Simulation.control_station.test_control_station -v
bash Simulation/control_station/smoke_test_wsl.sh
```

Trước lần ARM đầu tiên, phải kiểm tra trong trạng thái DISARMED: chiều Motor
1–4, chiều gập đồng bộ hai cánh, bốn aileron và hai ruddervator. Không dùng
model hiện tại để kết luận range, endurance, tải kết cấu hay transition
corridor của P4 thật.

## Cổng và xử lý sự cố

| Cổng | Mục đích |
|---|---|
| TCP `127.0.0.1:8765` | Dashboard/API |
| UDP `9002` | JSON FDM ArduPlane ↔ Gazebo |
| TCP `5760` | MAVProxy ↔ ArduPlane |
| UDP `14550` | Mission Planner |
| UDP `14551` | Control Station MAVLink |

- Nếu preflight báo cổng bận, dừng stack cũ bằng nút **STOP STACK** rồi chạy
  lại; không mở đồng thời hai stack.
- Nếu Gazebo GUI đen hoặc crash do OpenGL, giữ mặc định software rendering.
  Có thể thử GPU bằng `P4_TRANSWING_SOFTWARE_RENDERING=0` trong terminal WSL.
- Nếu GUI không tự đưa cửa sổ lên trước, cài `xdotool` và `wmctrl`; mô phỏng
  vẫn chạy khi thiếu hai helper này.
- Log Windows:
  `%LOCALAPPDATA%\P4 Transwing\control-station.log` và
  `control-station-error.log`.
- Log từng tiến trình và lỗi preflight cũng hiển thị trong tab log của
  dashboard.

## Đổi vị trí Home

Cập nhật đồng bộ:

- `Simulation/config/site.json`;
- `<spherical_coordinates>` trong
  `Simulation/gazebo/worlds/p4_ardupilot.sdf`;
- `P4_TRANSWING_SITL_HOME` hoặc mặc định trong
  `Simulation/ardupilot/run_p4_sitl_wsl.sh`;
- các mission tuyệt đối trong `Simulation/ardupilot/missions`.
