# DPSHIP_FUSION — 1 ESP32 đọc 2 cảm biến: la bàn GY-271 + RPLIDAR A1M8

Firmware cho **một board ESP32 duy nhất** đọc đồng thời hai cảm biến của mô hình tàu DP
(Dynamic Positioning) dùng cho triển lãm khoa học:

- **La bàn GY-271** (chip HMC5883L) → hướng mũi tàu (heading).
- **RPLIDAR A1M8** (quét laser 360°) → khoảng cách tới vách xung quanh → suy ra **vị trí x/y** của tàu trong bể.

Dữ liệu được gửi qua **Wi-Fi → MQTT (TLS)** mỗi giây:

| Luồng | Topic | Nội dung |
|---|---|---|
| Tàu | `bas/DP_SHIP_001/telemetry` | `{"x","y","hdg","rot","pos","cal","src":"esp32-fusion","uptime_s"}` |
| Lidar | `bas/BAS_LIDAR_001/telemetry` | mảng `ranges` 360 ô (1 ô = 1°, đơn vị mm, 0 = không có vật) |
| Lệnh xuống | `bas/DP_SHIP_001/cmd` | `{"motor": -100..100}` — điều khiển động cơ DC (mục 1.3); telemetry tàu cũng báo lại trường `"motor"` đang chạy |

Nơi nhận dữ liệu:

- **Web tàu 3D** (hiển thị tàu theo dữ liệu thật): https://ngshini.github.io/tau-happy-hunter-blue/
- **Database** trên máy Linux ở nhà (dịch vụ `dp-ship-logger` ghi mọi gói vào SQLite `dp_ship.db`, 2 bảng `ship_telemetry` + `lidar_scans`).

Trạng thái: **đã chạy thật** ngày 10/11/2026 — cả 2 luồng phát liên tục ~1 gói/giây,
lidar quét ~6,5 vòng/giây, mỗi vòng ~180 điểm.

---

## 1. Sơ đồ nối chân (bản đã chạy thật)

### 1.1 La bàn GY-271 (giao tiếp I2C)

| Chân GY-271 | Nối vào ESP32 | Ghi chú |
|---|---|---|
| VCC | **3V3** | |
| GND | **GND** | mass chung với toàn hệ |
| SDA | **GPIO32** | |
| SCL | **GPIO33** | |
| DRDY | *(không nối)* | để hở |

Cảm biến phải đặt **nằm ngang** (code không bù nghiêng) và nên hiệu chuẩn tại vị trí lắp cuối cùng (xem mục 4).

### 1.2 RPLIDAR A1M8 (giao tiếp UART + điều khiển motor)

Lidar có giắc **7 chân**, trên mạch in sẵn nhãn `VMOTO, MOTOCTL, GND, V5.0, TX, RX, GND`.
**Nối theo nhãn in cạnh từng chân, không theo màu dây** (dây chuyển đi kèm không đáng tin về màu).

| Chân lidar (theo nhãn) | Nối vào | Ghi chú |
|---|---|---|
| 1 — GND | Mass chung | |
| 2 — RX | **GPIO17** (TX2 của ESP32) | nối **chéo**: RX của lidar ← TX của ESP32 |
| 3 — TX | **GPIO16** (RX2 của ESP32) | nối **chéo**: TX của lidar → RX của ESP32 |
| 4 — V5.0 | **+5V nguồn ngoài** | nuôi mạch lidar |
| 5 — GND | Mass chung | 2 chân GND thông nhau bên trong lidar, nối cả 2 càng chắc |
| 6 — MOTOCTL | **GPIO26** | mức CAO = motor quay |
| 7 — VMOTO | **+5V nguồn ngoài** | nuôi motor (chịu dòng lớn) |

Sơ đồ tổng:

```
                        ESP32
 GY-271 (I2C)           ┌──────────────┐            RPLIDAR A1M8
   VCC ─────────────────┤ 3V3          │
   GND ─────────────────┤ GND          │      ┌─── 1 GND ────── mass chung
   SDA ─────────────────┤ GPIO32       │      ├─── 2 RX  ◄───── GPIO17 (TX2)
   SCL ─────────────────┤ GPIO33       │      ├─── 3 TX  ─────► GPIO16 (RX2)
   DRDY ─ ─ ─ (để hở)   │              │      ├─── 4 V5.0 ◄──── +5V ngoài
                        │ GPIO26 ──────┼──────┼─── 6 MOTOCTL
                        │ USB ◄── nguồn riêng │─── 7 VMOTO ◄─── +5V ngoài
                        └──────────────┘      └─── 5 GND ────── mass chung
```

### 1.3 Động cơ DC (điều khiển tốc độ + chiều qua web)

Board xuất tín hiệu cho **module driver động cơ** loại PWM/DIR (ví dụ Cytron MD10C):

| Tín hiệu | Chân ESP32 | Nối vào driver | Ghi chú |
|---|---|---|---|
| Xung tốc độ (PWM 20 kHz) | **GPIO23** | chân **PWM** của driver | độ rộng xung theo % tốc độ |
| Chiều quay | **GPIO22** | chân **DIR** của driver | mức CAO = tiến, mức THẤP = lùi (ngược thực tế thì đảo 2 dây động cơ) |

- Thang tốc độ **−100% … +100%**: dương = tiến, âm = lùi, 0 = dừng; vùng chết ±3% quanh 0.
- Điều khiển từ **web tàu 3D** (popup 🎛 ĐIỀU KHIỂN → mục ĐỘNG CƠ DC, thanh trượt −100…+100) hoặc lệnh MQTT `{"motor": <số>}` tới topic `bas/DP_SHIP_001/cmd`; gửi `{"stop":1}` để dừng.
- An toàn: mở nguồn lên động cơ **đứng yên** chờ lệnh; đang chạy mà mất kết nối MQTT quá 5 giây → **tự dừng**.
- Nguồn động cơ cấp riêng qua driver theo thông số của driver/động cơ — **không** lấy nguồn động cơ từ ESP32; mass của driver vẫn phải **chung mass** với ESP32.

### 1.4 Quy tắc nguồn — đọc trước khi cắm

- **ESP32 cấp nguồn riêng bằng USB** (máy tính hoặc củ sạc 5V). **Không** nối nguồn 5V ngoài vào chân 5V/VIN của ESP32.
- **Lidar dùng nguồn 5V ngoài riêng** (chân V5.0 + VMOTO). Motor lidar ăn dòng lớn; nguồn yếu → lidar không quay hoặc ESP32 reset vòng lặp.
- **BẮT BUỘC chung mass (GND)** giữa ESP32 và lidar/nguồn 5V — không chung mass thì dữ liệu UART sai hoặc không có, dù lidar vẫn quay.
- Mức logic UART của lidar là 3,3V → nối thẳng vào GPIO16/17, không cần mạch chuyển mức.
- Không dùng GPIO1/GPIO3 cho lidar (đó là chân nạp/debug serial).

---

## 2. Định vị x/y từ lidar

Mỗi vòng quét, firmware chiếu 360 khoảng cách lên 4 hướng (Đông/Tây/Bắc/Nam theo la bàn),
lấy phân vị 85% của các tia trong cửa sổ ±40° quanh mỗi hướng làm khoảng cách tới vách,
rồi suy ra tọa độ tâm bể: `x = (Đông − Tây)/2`, `y = (Bắc − Nam)/2`.

- Thuật toán cần **vách phẳng bao quanh** (trong bể thật, hoặc hộp thử chữ nhật). Để giữa bàn đầy đồ đạc thì tọa độ chỉ tương đối.
- Lệch trục lắp đặt giữa lidar và la bàn được hiệu chuẩn bằng lệnh `p` (mục 4) và lưu vào bộ nhớ board.
- Gói telemetry có trường `"pos": 1` khi vị trí của giây đó hợp lệ, `0` khi không tìm đủ vách (khi đó web giữ vị trí hợp lệ gần nhất).

## 3. Wi-Fi kép

Board cấu hình **2 mạng Wi-Fi**: mạng chính (ở nhà) và mạng dự phòng (hotspot điện thoại).
Mất mạng đang dùng → board tự thử lại luân phiên, mỗi mạng tối đa ~12 giây, và ưu tiên mạng vừa dùng tốt gần nhất.
Nhờ vậy mang ra triển lãm chỉ cần bật hotspot điện thoại là board tự bắt sóng.

## 4. Lệnh qua Serial (115200)

| Lệnh | Tác dụng |
|---|---|
| `c` | Hiệu chuẩn la bàn: xoay cảm biến chậm theo hình số 8 trong 20 giây (giữ tương đối ngang), kết quả lưu vào board |
| `r` | Xoá hiệu chuẩn la bàn |
| `h` | In hướng dẫn |
| `p` | Hiệu chuẩn trục lidar ↔ la bàn (giữ rig đứng yên, xung quanh là vách chữ nhật) |
| `t 3.60 2.40` | Đặt kích thước lòng bể: rộng × sâu (mét) — dùng để kiểm tra tính hợp lệ của vị trí |
| `d` | In trạng thái định vị hiện tại (trục, kích thước bể, khoảng cách 4 vách, x/y) |
| `m 50` | Chạy động cơ DC tại bàn: `m <số>` với số −100..100 (âm = lùi, `m 0` = dừng) |

## 5. Cài đặt và nạp code

Yêu cầu: [PlatformIO](https://platformio.org/).

```bash
cp src/config.example.h src/config.h   # rồi điền Wi-Fi + MQTT thật vào config.h
pio run -t upload                      # nạp qua USB
pio device monitor                     # xem log serial 115200
```

- `src/config.h` chứa mật khẩu Wi-Fi và đã được `.gitignore` loại khỏi repo — **không commit file này**.
- `src/ca_cert.h` là chứng thư gốc công khai của broker EMQX (không phải bí mật).

## 6. Lưu ý vận hành

- Khi đang **nạp code**, board ở chế độ bootloader nên motor lidar tạm dừng ~1 phút — nạp xong tự quay lại.
- Chỉ nên có **một** board phát lên topic của tàu tại một thời điểm (trùng topic + client ID sẽ tranh nhau).
- La bàn chưa hiệu chuẩn (`"cal": 0`) thì hướng vẫn dùng được nhưng có sai số; hiệu chuẩn bằng lệnh `c`.
