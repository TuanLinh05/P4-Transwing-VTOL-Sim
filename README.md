# ✈️ P4 Transwing VTOL Simulation

![ArduPilot](https://img.shields.io/badge/ArduPilot-QuadPlane%20SITL-0B3D91)
![Gazebo](https://img.shields.io/badge/Gazebo-Harmonic%20(Sim%208)-F58113)
![Platform](https://img.shields.io/badge/Platform-Windows%2011%20%2B%20WSL2-0078D6?logo=windows&logoColor=white)
![Python](https://img.shields.io/badge/Python-3.10-3776AB?logo=python&logoColor=white)
![MCU](https://img.shields.io/badge/RC%20bridge-Arduino%20MKR%20WiFi%201010-00979D?logo=arduino&logoColor=white)

<a id="english"></a>**🇬🇧 English** · [🇻🇳 Tiếng Việt](#tieng-viet)

A **Gazebo + ArduPilot SITL** simulation of a **Transwing VTOL** aircraft, inspired by the PteroDynamics P4.
Instead of tilting rotors, the two outer wing panels **fold** about oblique hinges: folded for hover, unfolded for fixed-wing cruise, so the wings, rotors and control surfaces all move as one assembly during the VTOL ↔ fixed-wing transition.
A local **Control Station** (web dashboard) starts the whole stack with one click, flies the aircraft over MAVLink and supervises the transition.

> [!IMPORTANT]
> This is a **surrogate / engineering-estimate model**, not a validated digital twin. The P4-clone geometry is used for visuals and first estimates; the aerodynamic, rotor and transition parameters do **not** represent the real PteroDynamics P4.
> This project is not affiliated with PteroDynamics.

---

## ✨ Features

- **Transwing kinematics:** two mirrored oblique fold joints. One 120° motion turns the wings from spanwise (cruise) to vertical along the fuselage (hover), and rotates the four rotor thrust axes from forward to upward.
- **Asset-based P4 model:** textured fuselage, moving wing and propeller meshes, 4 rotor force models that move with the wings, 4 ailerons, 2 ruddervators (V-tail), estimated lift/drag, MTOW mass distribution and inertia, IMU / NavSat / air-pressure sensors.
- **ArduPlane QuadPlane integration** via the official `ardupilot_gazebo` JSON plugin: Quad-X hover motors, one MotorTilt output driving both fold joints, mirrored ailerons and V-tail mixing, isolated EEPROM and DataFlash state.
- **One-click Control Station:** Windows launchers start Gazebo, SITL, MAVProxy and a local dashboard (`http://127.0.0.1:8765`) with telemetry, flight actions, waypoint missions and a supervised transition to fixed-wing flight.
- **Safety by design:** the aircraft always starts **DISARMED**, the dashboard binds only to `127.0.0.1`, ARM / takeoff / AUTO need confirmation, and **ABORT** always returns to QLOITER.
- **Preflight checks:** dependencies, versions, shared libraries, models/textures, SDF, write permissions, network ports and unit tests are checked before anything is spawned.
- **Optional scenery worlds:** visual-only low-poly city (400 m) and XL city (1 km) for stronger remote machines. They have no collisions, so flight dynamics are unchanged.
- **Real RC transmitter input:** FlySky FS-i6 → FS-iA6B iBUS → **Arduino MKR WiFi 1010** → USB → Control Station → MAVLink RC override.

## 🏗️ Architecture

```text
 FlySky FS-i6 ─iBUS─▶ MKR WiFi 1010 ─USB─▶ bridge_mkr_usb.py ─┐        (optional RC input)
                                                                ▼
┌───────────────────────────┐  HTTP 127.0.0.1:8765  ┌──────────────────────────────┐
│ Browser dashboard         │ ◀───────────────────▶ │ Control Station (server.py)  │
│ telemetry · map · mission │                       │ process manager · MAVLink    │
└───────────────────────────┘                       └──────┬───────────────┬───────┘
                                                           │ UDP 14551     │ starts / stops
                                                           ▼               ▼
┌──────────────────────────┐  JSON FDM (UDP 9002) ┌────────────────────────────────┐
│ Gazebo Harmonic          │ ◀──────────────────▶ │ ArduPlane QuadPlane SITL       │
│ p4_transwing_ardupilot   │                      │ + MAVProxy (TCP 5760)          │
└──────────────────────────┘                      └───────────────┬────────────────┘
                                                                  │ UDP 14550
                                                                  ▼
                                                     Mission Planner (optional)
```

## 🚀 Quick start (Windows 11 + WSL2)

**Requirements:** Windows 11 with WSL2/WSLg, Ubuntu 22.04 or 24.04, Gazebo Harmonic (Sim 8.x). Verified with Ubuntu 22.04, Gazebo Sim 8.14.0, and the ArduPilot / `ardupilot_gazebo` commits pinned in `Simulation/config/compatibility.json`.

1. **Install dependencies** from an Ubuntu WSL terminal at the repository root (installs Gazebo Harmonic, clones the pinned ArduPilot commits, builds ArduPlane SITL and `libArduPilotPlugin.so`):
   ```bash
   bash Simulation/tools/install_wsl_dependencies.sh --yes
   ```
2. **Run the preflight check:** double-click `P4-Transwing-Preflight.cmd` (or `.\P4-Transwing-Preflight.cmd -Deep`).
3. **Start the simulation:** double-click `P4-Transwing-Control-Station.cmd` (or `P4-Transwing-Control-Station-GPU.cmd` for GPU rendering).
4. In the dashboard, choose **`baseline`** and press **START SIMULATION**.

📖 Full setup guide (Vietnamese): [`Simulation/SETUP.md`](Simulation/SETUP.md).

### Safe flight workflow

1. While **DISARMED**, check the direction of the 4 motors, both wing folds, the ailerons and the V-tail.
2. Take off in **QLOITER** to at least **30 m** and wait until roll/pitch are within ±10°.
3. Press **CHUYỂN CÁNH BẰNG** (*transition to fixed-wing*). The dashboard uses a supervised FBWB flow, monitors altitude and attitude, and falls back to QLOITER if limits are exceeded.
4. Press **ABORT → QLOITER** to return to VTOL, then **QLAND** to land vertically.
5. Finish with **STOP STACK**.

### Manual run in WSL

```bash
bash Simulation/gazebo/run_p4_ardupilot_server_wsl.sh   # terminal 1: Gazebo server
bash Simulation/gazebo/run_p4_gui_wsl.sh                # terminal 2: Gazebo GUI
bash Simulation/ardupilot/run_p4_sitl_wsl.sh --wipe     # terminal 3: ArduPlane SITL
```

`--wipe` resets the simulated EEPROM and reloads `p4_transwing_sitl.param`.

## 🖥️ Launchers

| Launcher | Purpose |
|---|---|
| `P4-Transwing-Preflight.cmd` | Check dependencies, SDF, ports and tests |
| `P4-Transwing-Control-Station.cmd` | Start the full stack with the light default world |
| `P4-Transwing-Control-Station-GPU.cmd` | Same, with GPU rendering |
| `P4-Transwing-Scenery-Remote.cmd` | 400 m low-poly city world (remote machine) |
| `P4-Transwing-Scenery-XL-Remote.cmd` | 1 km XL city world (`-RenderMode Gpu`) |
| `P4-Transwing-Sync.cmd` / `P4-Transwing-Push-To-Host.cmd` | One-way sync between the host and a Remote Desktop machine via `\\tsclient` |

## 📂 Repository structure

```text
├── Simulation/
│   ├── gazebo/
│   │   ├── models/p4_transwing_ardupilot/   # P4 model integrated with ArduPilot
│   │   ├── models/p4_transwing_estimated/   # Asset-based estimated P4 model (meshes, joints)
│   │   ├── models/transwing_surrogate*/     # Primitive-geometry Transwing surrogate
│   │   ├── models/p4_scenery*/              # Optional visual-only city scenery
│   │   ├── worlds/                          # p4_ardupilot.sdf (main) + scenery worlds
│   │   └── tools/prepare_p4_meshes.py       # Builds Gazebo meshes from Ref/P4_clone
│   ├── ardupilot/                           # QuadPlane/Transwing params, SITL launcher, missions
│   ├── control_station/                     # Dashboard backend (server.py) + static web UI
│   ├── experiments/                         # Mission runner, metrics, energy, faults, wind (tests)
│   ├── config/                              # site.json (Home), p4_reference.json, compatibility.json
│   ├── tools/                               # WSL installer, preflight, RC bridge, sync scripts
│   └── SETUP.md
├── MCU/Controller/Controller.ino            # MKR WiFi 1010 iBUS → USB bridge (RC-only)
├── Plan/                                    # Published P4 data, estimates, RC bridge and scenery notes
├── Ref/P4_clone/                            # Source P4-clone meshes (fbx / glb / gltf / obj)
└── P4-Transwing-*.cmd                       # Windows one-click launchers
```

## 🔌 Ports

| Port | Purpose |
|---|---|
| TCP `127.0.0.1:8765` | Dashboard / API |
| UDP `9002` | JSON FDM ArduPlane ↔ Gazebo |
| TCP `5760` | MAVProxy ↔ ArduPlane |
| UDP `14550` | Mission Planner |
| UDP `14551` | Control Station MAVLink |

## 🧪 Tests

```bash
python3 -m unittest discover -s Simulation -p "test_*.py" -t .   # unit tests
bash Simulation/gazebo/smoke_test_p4_ardupilot_wsl.sh             # end-to-end, DISARMED
```

```powershell
.\Simulation\tools\smoke_test_p4_takeoff.ps1 -TargetAltitudeM 5   # ARM, 5 m hover, QLAND
```

## 📍 Home location

Home is set to **HCM University of Technology, 268 Lý Thường Kiệt, District 10, Ho Chi Minh City** (`10.772100, 106.657900`, 10 m).
To change it, update `Simulation/config/site.json`, `<spherical_coordinates>` in the world files, `P4_TRANSWING_SITL_HOME` in `Simulation/ardupilot/run_p4_sitl_wsl.sh`, and the absolute missions in `Simulation/ardupilot/missions`.

## 📚 References & credits

- [X-P4 Unmanned Aircraft Flight Manual, Revision D](https://downloads.regulations.gov/FAA-2024-2404-0001/attachment_10.pdf)
- [PteroDynamics – Transwing specifications (2025)](https://pterodynamics.com/media/PD_TranswingSpecs_2025.pdf)
- [Transwing by Tim Whitehand – ArduPilot](https://www.youtube.com/watch?v=-QAy2u33BEc&t=9s)
- [ArduPilot](https://github.com/ArduPilot/ardupilot) and [ardupilot_gazebo](https://github.com/ArduPilot/ardupilot_gazebo)
- The Control Station is adapted from the MiniHawk VTOL simulation dashboard.
- `Ref/P4_clone` contains third-party P4-clone 3D assets, used only for visualisation and geometry estimates.

Published data, measured geometry and engineering estimates are kept separate in [`Plan/Plan.md`](Plan/Plan.md) and `Simulation/config/p4_reference.json`.

---

<a id="tieng-viet"></a>

## 🇻🇳 Tiếng Việt

[🇬🇧 English](#english) · **🇻🇳 Tiếng Việt**

Mô phỏng **Gazebo + ArduPilot SITL** cho máy bay **VTOL Transwing**, lấy cảm hứng từ PteroDynamics P4.
Thay vì nghiêng rotor, hai nửa cánh ngoài **gập** quanh bản lề xiên: gập lại khi bay treo, mở ra khi bay bằng. Vì vậy cánh, rotor và các mặt điều khiển chuyển động cùng nhau trong quá trình chuyển tiếp VTOL ↔ cánh bằng.
**Control Station** (dashboard web chạy cục bộ) khởi động toàn bộ hệ thống bằng một cú nhấp, điều khiển máy bay qua MAVLink và giám sát quá trình chuyển tiếp.

> [!IMPORTANT]
> Đây là **mô hình surrogate / ước lượng kỹ thuật**, chưa phải digital twin đã kiểm chứng. Geometry P4 clone chỉ dùng để hiển thị và ước lượng ban đầu; các hệ số khí động, rotor và chuyển tiếp **không** đại diện cho máy bay PteroDynamics P4 thật.
> Dự án không liên kết với PteroDynamics.

### ✨ Tính năng

- **Động học Transwing:** hai khớp gập xiên đối xứng. Một chuyển động 120° đưa cánh từ nằm ngang (cruise) sang dựng dọc thân (hover), đồng thời xoay trục lực đẩy của 4 rotor từ hướng về trước sang hướng lên.
- **Mô hình P4 từ asset:** thân có texture, mesh cánh và cánh quạt chuyển động, 4 mô hình lực rotor di chuyển theo cánh, 4 aileron, 2 ruddervator (V-tail), lực nâng/cản ước lượng, phân bố khối lượng theo MTOW và mô-men quán tính, cảm biến IMU / NavSat / áp suất.
- **Tích hợp ArduPlane QuadPlane** qua plugin JSON chính thức `ardupilot_gazebo`: motor hover Quad-X, một kênh MotorTilt điều khiển cả hai khớp gập, aileron đối xứng và trộn V-tail, EEPROM và DataFlash tách riêng.
- **Control Station một cú nhấp:** launcher Windows khởi động Gazebo, SITL, MAVProxy và dashboard (`http://127.0.0.1:8765`) với telemetry, thao tác bay, mission waypoint và luồng chuyển sang bay bằng có giám sát.
- **An toàn:** máy bay luôn khởi động **DISARMED**, dashboard chỉ bind `127.0.0.1`, ARM / cất cánh / AUTO đều cần xác nhận, **ABORT** luôn quay về QLOITER.
- **Preflight:** kiểm tra dependency, phiên bản, thư viện, model/texture, SDF, quyền ghi, cổng mạng và unit test trước khi khởi chạy.
- **World cảnh quan tùy chọn:** thành phố low-poly 400 m và bản XL 1 km cho máy remote mạnh. Chỉ có visual, không có collision nên không ảnh hưởng động lực học bay.
- **Tay cầm RC thật:** FlySky FS-i6 → FS-iA6B iBUS → **Arduino MKR WiFi 1010** → USB → Control Station → MAVLink RC override.

Sơ đồ kiến trúc, bảng launcher, cấu trúc thư mục và bảng cổng: xem phần tiếng Anh ở trên.

### 🚀 Bắt đầu nhanh trên Windows

1. Đọc [hướng dẫn cài trên máy mới](Simulation/SETUP.md), rồi cài dependency trong Ubuntu WSL: `bash Simulation/tools/install_wsl_dependencies.sh --yes`.
2. Nhấp đúp `P4-Transwing-Preflight.cmd`.
3. Khi mọi kiểm tra đạt, nhấp đúp `P4-Transwing-Control-Station.cmd`.
4. Trong dashboard chọn `baseline` và nhấn **START SIMULATION**.

Để thử bay cánh bằng: cất cánh QLOITER tới 30 m, chờ roll/pitch nằm trong ±10°, rồi nhấn **CHUYỂN CÁNH BẰNG**. Nút này dùng luồng FBWB có giám sát thay vì đổi mode trực tiếp. **ABORT** luôn yêu cầu quay về QLOITER; sau khi cánh gập về VTOL, dùng **QLAND** để hạ thẳng đứng.

Máy bay luôn khởi động DISARMED. Không ARM trước khi kiểm tra đúng chiều bốn motor, hai cánh gập, aileron và V-tail.

### 🖥️ Máy remote và cảnh quan

- Khi chạy ở máy remote qua RDP, nhấp đúp `P4-Transwing-Sync.cmd` để đồng bộ incremental một chiều từ drive được redirect (`\\tsclient`) mà không xóa state và log trên máy remote. Nhấp đúp `P4-Transwing-Push-To-Host.cmd` trên remote để đẩy các thay đổi mới hơn về máy chính.
- Nếu máy remote đủ mạnh, dùng `P4-Transwing-Scenery-Remote.cmd` để có thêm nhà, đường và dải cây low-poly (chỉ để quan sát). Máy local vẫn dùng world nhẹ `P4-Transwing-Control-Station.cmd`.
- Bản đồ rộng 1 km với nhiều nhà và cao ốc hơn: `P4-Transwing-Scenery-XL-Remote.cmd -RenderMode Gpu`. Xem chi tiết tại [`Plan/REMOTE_SCENERY_WORLD.md`](Plan/REMOTE_SCENERY_WORLD.md).

### 📍 Vị trí Home

Home đặt tại **Trường Đại học Bách khoa – ĐHQG TP.HCM, 268 Lý Thường Kiệt, Quận 10, TP.HCM** (`10.772100, 106.657900`, cao 10 m).
Muốn đổi, cập nhật đồng bộ `Simulation/config/site.json`, `<spherical_coordinates>` trong các file world, `P4_TRANSWING_SITL_HOME` trong `Simulation/ardupilot/run_p4_sitl_wsl.sh` và các mission tuyệt đối trong `Simulation/ardupilot/missions`.

### 📚 Tài liệu

- [`Simulation/SETUP.md`](Simulation/SETUP.md): cài đặt, chạy thủ công, kiểm thử, cổng và xử lý sự cố.
- [`Plan/Plan.md`](Plan/Plan.md): thông số công bố, dữ kiện video và phạm vi áp dụng.
- [`Plan/MKR_WIFI_1010_IBUS_USB_CONTROL_BRIDGE.md`](Plan/MKR_WIFI_1010_IBUS_USB_CONTROL_BRIDGE.md): đấu dây và giao thức cầu nối tay cầm RC.
- `Simulation/config/p4_reference.json`: nguồn dữ liệu máy đọc.
- `Ref/P4_clone`: asset 3D P4 clone của bên thứ ba, chỉ dùng để hiển thị và ước lượng hình học.

---

<p align="center">Made by <a href="https://github.com/TuanLinh05">Vu Tuan Linh</a> · HCMUT</p>
