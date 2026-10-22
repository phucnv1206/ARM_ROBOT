# 🤖 Robot Arm 4-DOF + Gripper Pick & Place (Teach & Playback)

Dự án tay máy robot 4 bậc tự do (**4-DOF**) kèm khâu chấp hành tay gắp (**Gripper**) ứng dụng gắp và đặt vật thể (**Pick and Place**) với cơ chế **Dạy học trước (Teach/Lưu điểm) - Tự động tính toán và phát lại chuỗi hành động (Playback)**.

Toàn bộ quá trình tính toán động học, nội suy quỹ đạo mượt mà và lưu trữ dữ liệu chuỗi hành động được thực hiện **trực tiếp trên vi điều khiển ESP32** (lưu vào bộ nhớ Flash NVS, không mất dữ liệu khi mất điện). Giao diện Web HTML5 kết nối trực tiếp qua **Web Serial API** với mô hình đồ họa 3D trực quan và hệ thống phím tắt điều khiển thời gian thực.

---

## 🌟 Tính Năng Nổi Bật

1. **Học hành động & Lưu Flash (Teach & Flash Persistence):**
   - Người dùng điều khiển tay máy đến vị trí mong muốn và lưu điểm.
   - ESP32 lưu trữ tối đa **8 điểm thao tác (Waypoints)** trực tiếp vào bộ nhớ Flash (`Preferences` NVS). Mỗi điểm bao gồm tọa độ Descartes $(X, Y, Z)$ cùng góc quay cổ tay (**Wrist**) và góc kẹp (**Gripper**).
   - Khôi phục nguyên vẹn chuỗi điểm ngay khi khởi động lại nguồn.
2. **Tiền tính toán quỹ đạo (Pre-baked Trajectory Caching):**
   - Khi bấm **"Chạy chuỗi"**, ESP32 sẽ tự động phân tích và "nướng" trước (Bake) toàn bộ $N-1$ đoạn quỹ đạo nối giữa các điểm vào mảng bộ nhớ RAM (`bakedSegments`).
   - Khi phát lại, robot chạy mượt mà ở tần số cao **200 Hz (chu kỳ 5 ms)**, loại bỏ hoàn toàn hiện tượng khựng/giật giữa các điểm chuyển tiếp.
3. **Quy trình Pick & Place thực tế tại điểm dừng:**
   - Khi cánh tay di chuyển từ điểm $A$ đến điểm $B$:
     - **Cổ tay (Wrist):** Nếu góc tại $B$ khác $A$, cổ tay sẽ xoay **từ từ** (sử dụng hàm nội suy mượt *Smoothstep* $s = 3t^2 - 2t^3$) trong `WRIST_SETTLE_MS` (500 ms) giúp hạn chế rung lắc cơ khí.
     - **Tay gắp (Gripper):** Sau khi cổ tay quay xong, nếu góc gắp tại $B$ khác $A$, tay gắp sẽ kẹp hoặc nhả dứt khoát trong `GRIPPER_SETTLE_MS` (500 ms).
     - **Tối ưu tốc độ:** Nếu tại điểm $B$ cả Wrist và Gripper đều không đổi góc so với $A$, robot sẽ **tiếp tục lướt mượt sang đoạn kế tiếp mà không dừng lại**.
4. **Tự động tiếp cận vị trí đầu (Auto-Seeking):**
   - Trước khi phát chuỗi, nếu tay máy đang ở vị trí bất kỳ khác với điểm xuất phát (Điểm 1), ESP32 sẽ tự động tạo quỹ đạo S-Curve đưa tay máy về điểm xuất phát một cách an toàn rồi mới bắt đầu chuỗi.
5. **Hệ thống vòng kín trục xoay (Closed-Loop Stepper):**
   - Trục đế (Base) sử dụng động cơ bước kết hợp cảm biến từ tính **AS5600 12-bit (I2C)** gắn trực tiếp tại trục ra.
   - Tự động bù sai lệch bước khi đang chạy (`moving trim`) và khi dừng (`settled trim`), đồng thời tự động phát hiện kẹt động cơ (**Stall Detection**) để ngắt chuyển động khẩn cấp bảo vệ phần cứng.
6. **Giao diện Web Serial 3D thời gian thực:**
   - Chạy trực tiếp trên trình duyệt (Chrome/Edge), không cần cài đặt phần mềm trung gian hay driver server.
   - Canvas 3D trực quan có thể xoay, phóng to/thu nhỏ góc nhìn và hiển thị trực quan toàn bộ các điểm quỹ đạo đã lưu.

---

## 🛠 Sơ Đồ Phần Cứng & Kết Nối Chân (Pinout)

### 1. Danh sách linh kiện
- **Bộ điều khiển trung tâm:** ESP32 Dev Module (Dual-core 240MHz).
- **DOF 1 (Trục xoay đế - Base):** Động cơ bước (Stepper motor, góc bước 0.9°), driver TMC2225 (chế độ 1/4 microstep, tỉ số truyền dây đai/bánh răng 2:1).
- **Cảm biến phản hồi trục đế:** Cảm biến góc từ tính AS5600 (Giao tiếp I2C 400kHz).
- **DOF 2 (Vai - Shoulder / L1):** Servo RC (LEDC PWM Channel 0).
- **DOF 3 (Khuỷu - Elbow / L2):** Servo RC (LEDC PWM Channel 1).
- **DOF 4 (Cổ tay - Wrist):** Servo RC xoay cổ tay 0°–180° (LEDC PWM Channel 2).
- **Khâu tác động cuối / Chấp hành (Tay gắp - Gripper End-Effector):** Servo RC kẹp 0°–180° (LEDC PWM Channel 3).

### 2. Sơ đồ nối dây với ESP32

| Thiết bị | Chân thiết bị | Chân ESP32 (GPIO) | Ghi chú |
| :--- | :--- | :--- | :--- |
| **TMC2225 (Stepper Driver)** | `STEP` | `GPIO 5` | Xung điều khiển bước |
| | `DIR` | `GPIO 18` | Chiều quay |
| | `EN` / `ENA` | `GPIO 25` | Kích hoạt driver |
| **Cảm biến AS5600** | `SCL` | `GPIO 16` | I2C Clock (Tần số 400 Hz) |
| | `SDA` | `GPIO 17` | I2C Data |
| | `VCC` / `GND` | `3.3V` / `GND` | Nguồn cảm biến |
| **Servo Khớp 1 (L1)** | `PWM (Signal)` | `GPIO 23` | LEDC Ch 0, tần số 50Hz |
| **Servo Khớp 2 (L2)** | `PWM (Signal)` | `GPIO 22` | LEDC Ch 1, tần số 50Hz |
| **Servo Cổ tay (Wrist)** | `PWM (Signal)` | `GPIO 21` | LEDC Ch 2, tần số 50Hz |
| **Servo Tay gắp (Gripper)**| `PWM (Signal)` | `GPIO 19` | LEDC Ch 3, tần số 50Hz |
| **Nguồn Servo rời** | `+5V` / `GND` | Nguồn ngoài 5V–6V (>= 3A) | **Chung GND với ESP32** |

---

## 📐 Thông Số Hình Học & Giới Hạn Cơ Khí

Các thông số kích thước cấu hình trong file [`include/config.h`](file:///c:/arm_robot/include/config.h):

- Khâu 1 ($L_1$): `130 mm`
- Khâu 2 ($L_2$): `130 mm`
- Độ lệch tâm trục xoay ($D$): `20 mm`
- Chiều cao trục đế ($H$): `88 mm`
- Vùng cấm hình trụ (Deadzone): Bán kính `70 mm`, chiều cao `90 mm`.

### Giới hạn góc các khớp:
- **Trục xoay (Stepper):** `60°` đến `300°` (tốc độ tối đa ~72°/s).
- **Servo 1:** `0°` đến `90°`.
- **Servo 2:** `90°` đến `180°` (phụ thuộc góc Servo 1 theo điều kiện tránh va chạm cơ khí).
- **Cổ tay (Wrist):** `0°` đến `180°`.
- **Tay gắp (Gripper):** `0°` đến `180°`.

---

## 🎮 Hướng Dẫn Sử Dụng Giao Diện Web

Giao diện web nằm tại file [`web/index.html`](file:///c:/arm_robot/web/index.html).

### 1. Kết nối với Robot
1. Cắm cáp USB nối ESP32 với máy tính.
2. Mở file `web/index.html` bằng trình duyệt Google Chrome hoặc Microsoft Edge.
3. Bấm nút **"Kết nối UART"**, chọn đúng cổng COM của ESP32 (Baudrate 115200).

### 2. Phím tắt điều khiển (Bàn phím)
Bạn có thể dùng thanh trượt hoặc các phím tắt nhanh được bố trí trực quan:
- **Servo 1 (Khớp 1):** `Q` (Tăng) / `W` (Giảm) - Bước 1°.
- **Servo 2 (Khớp 2):** `A` (Tăng) / `S` (Giảm) - Bước 1°.
- **Trục xoay (Base Stepper):** `Z` (Tăng) / `X` (Giảm) - Bước 0.9°.
- **Cổ tay (Wrist):** `R` (Tăng) / `E` (Giảm) - Bước 1°.
- **Tay gắp (Gripper):** `F` (Mở/Tăng) / `D` (Kẹp/Giảm) - Bước 1°.

### 3. Quy trình dạy học Pick & Place (Teach & Playback)
1. **Dạy điểm tiếp cận vật:** Điều khiển cánh tay đến vị trí phía trên vật thể, mở kẹp (`Gripper = 0°`), bấm **"Lưu điểm hiện tại"** $\rightarrow$ Lưu điểm 1.
2. **Dạy điểm gắp:** Hạ cánh tay xuống sát vật thể, đóng kẹp kẹp chặt vật (`Gripper = 90°` hoặc góc phù hợp), bấm **"Lưu điểm hiện tại"** $\rightarrow$ Lưu điểm 2.
3. **Dạy điểm nâng:** Nâng cánh tay lên vị trí trung gian an toàn (giữ kẹp đóng), bấm **"Lưu điểm hiện tại"** $\rightarrow$ Lưu điểm 3.
4. **Dạy điểm chuyển & xoay hướng:** Xoay đế sang vị trí thả, xoay góc cổ tay (`Wrist`) phù hợp, bấm **"Lưu điểm hiện tại"** $\rightarrow$ Lưu điểm 4.
5. **Dạy điểm đặt:** Hạ tay máy xuống vị trí đặt, mở kẹp nhả vật (`Gripper = 0°`), bấm **"Lưu điểm hiện tại"** $\rightarrow$ Lưu điểm 5.
6. **Thực thi:** Bấm **"Chạy chuỗi"**:
   - ESP32 sẽ tự động tiền tính toán (Bake) toàn bộ quỹ đạo nối giữa các điểm.
   - Robot sẽ tự động lướt qua các điểm, xoay cổ tay mượt mà và kẹp/nhả chính xác theo đúng kịch bản đã học.
7. **Xóa kịch bản:** Bấm **"Xóa chuỗi điểm"** để xóa toàn bộ dữ liệu trong Flash khi muốn dạy kịch bản mới.

---

## 💻 Cấu Trúc Dự Án

```plaintext
arm_robot/
├── include/
│   ├── config.h          # Cấu hình phần cứng, chân I/O, kích thước khâu, thông số thời gian
│   ├── kinematics.h      # Khai báo hàm động học thuận (FK) và nghịch (IK)
│   ├── motion.h          # Bộ tạo quỹ đạo S-Curve, mẫu thời gian và vận tốc
│   ├── as5600.h          # Driver đọc cảm biến góc từ tính I2C AS5600
│   ├── stepper.h         # Điều khiển xung Stepper, theo dõi sai số và bù bước
│   ├── PWM.h             # Cấu hình 4 kênh LEDC PWM cho các Servo
│   └── robot_comms.h     # Giao thức truyền thông nhị phân UART, máy trạng thái chuyển động
├── src/
│   ├── main.cpp          # Vòng lặp điều khiển chính (Control Loop 200Hz, Feedback 400Hz)
│   ├── kinematics.cpp    # Cài đặt giải thuật FK / IK giải tích và kiểm tra va chạm
│   ├── motion.cpp        # Thuật toán tính toán profile quỹ đạo mượt
│   ├── as5600.cpp        # Giao tiếp I2C và xử lý lọc góc từ tính
│   ├── stepper.cpp       # Phát xung step, quản lý vị trí trục xoay vòng kín
│   ├── PWM.cpp           # Khởi tạo và cập nhật xung PWM (Servo 1, 2, Wrist, Gripper)
│   └── robot_comms.cpp   # Xử lý lệnh nhị phân, bộ nhớ Flash Preferences, Pre-baking quỹ đạo
├── web/
│   └── index.html        # Giao diện điều khiển Web Serial, Canvas 3D Viewer
├── platformio.ini         # Cấu hình môi trường biên dịch PlatformIO cho ESP32
└── README.md             # Tài liệu hướng dẫn dự án
```

---

## 🔒 Cơ Chế An Toàn Phần Cứng

- **Giới hạn góc vật lý & Điều kiện va chạm:** Hàm `checkCondition()` và `limitsOk()` liên tục kiểm tra tránh va chạm giữa khâu 1 và khâu 2.
- **Bảo vệ kẹt động cơ (Stall Detection):** Nếu xung bước đã phát đủ nhưng cảm biến AS5600 đo được góc trục ra không thay đổi trong `500 ms`, hệ thống sẽ lập tức dừng toàn bộ cánh tay và gửi mã lỗi `errorStepperStall` lên web.
