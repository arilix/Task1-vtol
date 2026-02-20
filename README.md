# TurtleBot3 Autonomous Square Mission

Misi navigasi otonom TurtleBot3 membentuk pola **persegi sempurna (2×2 meter)** menggunakan ROS. Robot bergerak secara autonomous melalui 4 waypoint dan kembali ke titik awal, lalu berhenti otomatis.

Tersedia implementasi untuk **ROS 1 (Noetic)** dan **ROS 2 (Foxy/Humble)**, masing-masing dalam versi **Python** dan **C++**.

---

## Daftar Isi

- [TurtleBot3 Autonomous Square Mission](#turtlebot3-autonomous-square-mission)
  - [Daftar Isi](#daftar-isi)
  - [Fitur](#fitur)
  - [Arsitektur](#arsitektur)
  - [Struktur Direktori](#struktur-direktori)
  - [Prasyarat](#prasyarat)
  - [Instalasi \& Build](#instalasi--build)
    - [ROS 1 Noetic](#ros-1-noetic)
    - [ROS 2 Foxy / Humble](#ros-2-foxy--humble)
  - [Cara Menjalankan](#cara-menjalankan)
    - [Menjalankan ROS 1](#menjalankan-ros-1)
    - [Menjalankan ROS 2](#menjalankan-ros-2)
  - [Parameter](#parameter)
  - [Topik ROS](#topik-ros)
  - [Cara Kerja](#cara-kerja)
    - [Perbedaan ROS 1 vs ROS 2](#perbedaan-ros-1-vs-ros-2)
    - [Perbedaan Python vs C++](#perbedaan-python-vs-c)
  - [Emergency Stop](#emergency-stop)
  - [Lisensi](#lisensi)

---

## Fitur

- **Dual Language** — tersedia dalam Python dan C++ untuk ROS 1 maupun ROS 2
- **Distance-Based Waypoint Navigation** — menggunakan Euclidean distance, bukan `time.sleep()`
- **Proportional Control** — kontrol rotasi dan linear yang halus
- **State Machine** (ROS 2) / **Sequential** (ROS 1) — arsitektur navigasi yang terstruktur
- **Emergency Stop** — robot langsung berhenti saat menerima sinyal emergency
- **Configurable Parameters** — semua parameter misi bisa diubah via launch file
- **Odometry-Based Positioning** — posisi dihitung dari data odometry robot
- **Auto-Stop** — robot berhenti otomatis setelah kembali ke titik awal

---

## Arsitektur

```
Odometry (/odom)          Emergency Stop (/emergency_stop)
       │                            │
       ▼                            ▼
┌─────────────────────────────────────────┐
│         Square Mission Node             │
│                                         │
│  ┌─────────┐   ┌──────────┐             │
│  │ Rotate   │──▶│  Move    │──▶ Next WP │
│  │ to target│   │ to target│            │
│  └─────────┘   └──────────┘             │
│                                         │
│  Waypoints: (2,0)→(2,2)→(0,2)→(0,0)     │
└──────────────────┬──────────────────────┘
                   │
                   ▼
           Velocity (/cmd_vel)
```

---

## Struktur Direktori

```
.
├── README.md
├── ros1_ws/                              # Workspace ROS 1 Noetic
│   └── src/
│       └── turtlebot3_square_mission/
│           ├── CMakeLists.txt
│           ├── package.xml
│           ├── launch/
│           │   ├── square_mission.launch        # Launch Python node
│           │   └── square_mission_cpp.launch     # Launch C++ node
│           ├── scripts/
│           │   └── square_mission.py             # Python node
│           └── src/
│               └── square_mission.cpp            # C++ node
│
└── ros2_ws/                              # Workspace ROS 2 (Foxy/Humble)
    └── src/
        ├── turtlebot3_square_mission/            # Python package
        │   ├── package.xml
        │   ├── setup.cfg
        │   ├── setup.py
        │   ├── launch/
        │   │   └── square_mission.launch.py
        │   ├── resource/
        │   │   └── turtlebot3_square_mission
        │   └── turtlebot3_square_mission/
        │       ├── __init__.py
        │       └── square_mission.py
        │
        └── turtlebot3_square_mission_cpp/        # C++ package
            ├── CMakeLists.txt
            ├── package.xml
            ├── launch/
            │   └── square_mission.launch.py
            └── src/
                └── square_mission.cpp
```

---

## Prasyarat

| Komponen | ROS 1 | ROS 2 |
|---|---|---|
| **Distro ROS** | Noetic (Ubuntu 20.04) | Foxy / Humble (Ubuntu 20.04 / 22.04) |
| **Python** | 3.8+ | 3.8+ |
| **C++ Compiler** | g++ (C++11) | g++ (C++17) |
| **TurtleBot3 Packages** | `turtlebot3`, `turtlebot3_gazebo` | `turtlebot3`, `turtlebot3_gazebo` |
| **Gazebo** | Gazebo 11 | Gazebo 11 |

Pastikan environment variable TurtleBot3 sudah diatur:

```bash
export TURTLEBOT3_MODEL=burger
```

---

## Instalasi & Build

### ROS 1 Noetic

```bash
# 1. Masuk ke workspace
cd ros1_ws

# 2. Build dengan catkin (Python + C++ sekaligus)
catkin_make

# 3. Source workspace
source devel/setup.bash

# 4. Pastikan script Python executable
chmod +x src/turtlebot3_square_mission/scripts/square_mission.py
```

### ROS 2 Foxy / Humble

```bash
# 1. Masuk ke workspace
cd ros2_ws

# 2. Build semua package (Python + C++)
colcon build --packages-select turtlebot3_square_mission turtlebot3_square_mission_cpp

# 3. Source workspace
source install/setup.bash
```

> **Catatan:** Untuk build hanya versi tertentu, gunakan `--packages-select` dengan nama package yang diinginkan.

---

## Cara Menjalankan

### Menjalankan ROS 1

Launch file akan menjalankan **Gazebo** dan **Square Mission node** secara bersamaan:

**Python:**
```bash
# Default (persegi 2x2 meter, model burger)
roslaunch turtlebot3_square_mission square_mission.launch
```

**C++:**
```bash
roslaunch turtlebot3_square_mission square_mission_cpp.launch
```

Dengan parameter custom:

```bash
roslaunch turtlebot3_square_mission square_mission_cpp.launch \
    side_length:=3.0 \
    linear_speed:=0.15 \
    model:=waffle
```

### Menjalankan ROS 2

**Python:**
```bash
# Default (persegi 2x2 meter, model burger)
ros2 launch turtlebot3_square_mission square_mission.launch.py
```

**C++:**
```bash
ros2 launch turtlebot3_square_mission_cpp square_mission.launch.py
```

Dengan parameter custom:

```bash
ros2 launch turtlebot3_square_mission_cpp square_mission.launch.py \
    side_length:=3.0 \
    threshold:=0.25 \
    model:=waffle
```

---

## Parameter

| Parameter | Default | Deskripsi |
|---|---|---|
| `side_length` | `2.0` | Panjang sisi persegi (meter) |
| `threshold` | `0.3` | Jarak threshold waypoint tercapai (meter) |
| `linear_speed` | `0.2` | Kecepatan linear maju (m/s) |
| `angular_speed` | `0.5` | Kecepatan angular rotasi (rad/s) |
| `angle_threshold` | `0.05` | Threshold sudut rotasi selesai (rad) |
| `rate_hz` | `20` | Frekuensi control loop (Hz) |

---

## Topik ROS

| Topik | Tipe | Arah | Deskripsi |
|---|---|---|---|
| `/odom` | `nav_msgs/Odometry` | Subscribe | Data posisi dan orientasi robot |
| `/cmd_vel` | `geometry_msgs/Twist` | Publish | Perintah kecepatan ke robot |
| `/emergency_stop` | `std_msgs/Bool` | Subscribe | Sinyal emergency stop |

---

## Rumus & Logika Navigasi

Berikut penjelasan lengkap rumus matematika dan logika kontrol yang digunakan dalam navigasi robot.

### 1. Konversi Quaternion ke Euler (Yaw)

Data orientasi dari odometry berupa **quaternion** $(q_x, q_y, q_z, q_w)$. Untuk navigasi 2D, kita hanya membutuhkan sudut **yaw** ($\psi$) — rotasi terhadap sumbu Z.

$$\psi = \text{atan2}\left(2(q_w q_z + q_x q_y),\; 1 - 2(q_y^2 + q_z^2)\right)$$

> **Catatan:** Pada ROS 1, digunakan `tf.transformations` (Python) atau `tf::Matrix3x3::getRPY()` (C++). Pada ROS 2, digunakan custom function tanpa dependency `tf`.

**Implementasi Python (ROS 2):**
```python
t3 = 2.0 * (w * z + x * y)
t4 = 1.0 - 2.0 * (y * y + z * z)
yaw = math.atan2(t3, t4)
```

### 2. Euclidean Distance

Digunakan untuk mengukur jarak antara posisi robot saat ini $(x_c, y_c)$ dengan posisi target $(x_t, y_t)$:

$$d = \sqrt{(x_t - x_c)^2 + (y_t - y_c)^2}$$

Robot dianggap **sudah mencapai waypoint** jika:

$$d < \text{threshold} \quad (\text{default: } 0.3 \text{ meter})$$

**Implementasi Python:**
```python
def euclidean_distance(self, target_x, target_y):
    return math.sqrt(
        (target_x - self.current_x) ** 2 +
        (target_y - self.current_y) ** 2
    )
```

### 3. Sudut ke Target (Angle to Target)

Menghitung sudut yang harus ditempuh robot dari posisinya saat ini untuk menghadap target:

$$\theta_{\text{target}} = \text{atan2}(y_t - y_c,\; x_t - x_c)$$

Hasil $\theta_{\text{target}}$ berada di range $[-\pi, \pi]$ radian.

**Implementasi Python:**
```python
def angle_to_target(self, target_x, target_y):
    return math.atan2(
        target_y - self.current_y,
        target_x - self.current_x
    )
```

### 4. Normalisasi Sudut

Selisih sudut bisa melebihi range $[-\pi, \pi]$, sehingga perlu dinormalisasi agar robot selalu berputar ke arah terdekat:

$$\theta_{\text{norm}} = \begin{cases} \theta - 2\pi & \text{jika } \theta > \pi \\ \theta + 2\pi & \text{jika } \theta < -\pi \end{cases}$$

**Implementasi Python:**
```python
def normalize_angle(self, angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle
```

### 5. Kontrol Rotasi (Rotate to Target)

Sebelum bergerak ke waypoint, robot berputar di tempat hingga menghadap target. Menggunakan **proportional control** pada kecepatan angular:

$$\Delta\theta = \text{normalize}(\theta_{\text{target}} - \psi_{\text{current}})$$

$$\omega = \omega_{\text{max}} \times \text{sign}(\Delta\theta) \times \begin{cases} 0.5 & \text{jika } |\Delta\theta| < 0.3 \text{ rad} \\ 1.0 & \text{lainnya} \end{cases}$$

Rotasi dianggap selesai jika:

$$|\Delta\theta| < \text{angle\_threshold} \quad (\text{default: } 0.05 \text{ rad} \approx 2.86°)$$

**Diagram alur:**
```
        ┌──────────────┐
        │ Hitung Δθ    │
        └──────┬───────┘
               │
        ┌──────▼───────┐     Ya      ┌──────────────┐
        │ |Δθ| < 0.05? │────────────▶│ ROTASI DONE  │
        └──────┬───────┘              └──────────────┘
               │ Tidak
        ┌──────▼───────┐
        │ |Δθ| < 0.3?  │
        └──┬───────┬───┘
       Ya  │       │ Tidak
     ┌─────▼──┐ ┌──▼─────┐
     │ω × 0.5 │ │ω × 1.0 │
     └─────┬──┘ └──┬─────┘
           │       │
        ┌──▼───────▼──┐
        │ Publish ω    │
        │ ke /cmd_vel  │
        └──────────────┘
```

### 6. Kontrol Gerak Linear (Move to Target)

Setelah menghadap target, robot bergerak maju dengan **koreksi heading** secara simultan:

**Kecepatan linear** ($v$) — dengan perlambatan saat mendekati target:

$$v = \begin{cases} v_{\text{max}} \times 0.5 & \text{jika } d < 0.5 \text{ m} \\ v_{\text{max}} & \text{lainnya} \end{cases}$$

**Koreksi angular** ($\omega$) — proportional control untuk menjaga heading:

$$\omega = K_p \times \Delta\theta \quad \text{dimana } K_p = 1.5$$

Ini memastikan robot tetap mengarah ke target meski ada drift atau gangguan.

**Implementasi Python:**
```python
twist = Twist()

# Linear speed - perlambatan mendekati target
if distance < 0.5:
    twist.linear.x = self.linear_speed * 0.5
else:
    twist.linear.x = self.linear_speed

# Angular correction (Kp = 1.5)
twist.angular.z = 1.5 * angle_diff
```

### 7. Waypoint Generation

Waypoint dihitung **relatif terhadap posisi awal** robot, bukan koordinat absolut. Ini membuat misi bisa dimulai dari posisi manapun.

Dengan posisi awal $(x_0, y_0)$ dan panjang sisi $L$:

| Waypoint | Koordinat | Default ($L=2$) |
|---|---|---|
| WP1 | $(x_0 + L,\; y_0)$ | $(2, 0)$ |
| WP2 | $(x_0 + L,\; y_0 + L)$ | $(2, 2)$ |
| WP3 | $(x_0,\; y_0 + L)$ | $(0, 2)$ |
| WP4 | $(x_0,\; y_0)$ | $(0, 0)$ — kembali |

**Visualisasi lintasan persegi:**
```
    (0,L) WP3 ◄──────────── WP2 (L,L)
          │                    ▲
          │                    │
          │      Persegi       │
          │      L × L m       │
          ▼                    │
  (0,0) WP4/Start ──────────▶ WP1 (L,0)
```

### 8. State Machine (ROS 2)

Pada versi ROS 2, navigasi menggunakan **state machine** berbasis timer callback yang dijalankan pada frekuensi `rate_hz` (default 20Hz):

```
┌────────────┐    Odom diterima    ┌────────────┐
│ WAIT_ODOM  │────────────────────▶│   ROTATE   │◄─────────┐
└────────────┘                     └─────┬──────┘          │
                                         │                 │
                                   |Δθ| < threshold        │
                                         │                 │
                                   ┌─────▼──────┐    Next  │
                                   │    MOVE     │────WP───▶│
                                   └─────┬──────┘          │
                                         │
                                   Semua WP tercapai
                                         │
                                   ┌─────▼──────┐
                                   │    DONE     │
                                   └────────────┘

                            ┌────────────┐
         (dari state mana   │  STOPPED   │  ◄── Emergency Stop
          saja)             └────────────┘
```

Setiap iterasi timer (50ms pada 20Hz), state machine mengevaluasi kondisi saat ini dan mengirim perintah kecepatan yang sesuai. Ini berbeda dengan arsitektur ROS 1 yang menggunakan blocking `while` loop.

---

## Cara Kerja

1. **Inisialisasi** — Node dimulai dan menunggu data odometry pertama dari `/odom`
2. **Hitung Waypoints** — 4 waypoint persegi dihitung relatif terhadap posisi awal robot:
   - WP1: `(start_x + side_length, start_y)`
   - WP2: `(start_x + side_length, start_y + side_length)`
   - WP3: `(start_x, start_y + side_length)`
   - WP4: `(start_x, start_y)` — kembali ke awal
3. **Untuk setiap waypoint:**
   - **Rotate** — robot berputar menghadap target menggunakan proportional control
   - **Move** — robot bergerak maju menuju target dengan koreksi heading
   - **Check** — jika Euclidean distance < threshold (0.3m), waypoint dianggap tercapai
4. **Selesai** — setelah WP4 tercapai, robot berhenti dan misi berakhir

### Perbedaan ROS 1 vs ROS 2

| Aspek | ROS 1 | ROS 2 |
|---|---|---|
| Arsitektur | Sequential (`while` loop) | State Machine via Timer Callback |
| Quaternion-to-Euler | `tf.transformations` / `tf::Matrix3x3` | Custom function (tanpa dependency `tf`) |
| Parameter | `rospy.get_param()` / `nh_.param()` | `declare_parameter()` / `get_parameter()` |
| QoS | Tidak ada | `ReliabilityPolicy.RELIABLE` |

### Perbedaan Python vs C++

| Aspek | Python | C++ |
|---|---|---|
| ROS 1 Arsitektur | Sequential `while` loop | Sequential `while` loop + `ros::spinOnce()` |
| ROS 2 Arsitektur | Timer callback + state machine | Timer callback + state machine |
| ROS 1 Quaternion | `tf.transformations.euler_from_quaternion()` | `tf::Matrix3x3::getRPY()` |
| ROS 2 Quaternion | Custom `euler_from_quaternion()` | Custom `eulerFromQuaternion()` |
| Build System (ROS 1) | `catkin_install_python` | `add_executable` + `target_link_libraries` |
| Build System (ROS 2) | `ament_python` (setup.py) | `ament_cmake` (CMakeLists.txt) |
| Performa | Interpreted | Compiled (lebih cepat) |

---

## Emergency Stop

Untuk menghentikan robot secara darurat saat misi berjalan:

**ROS 1:**
```bash
rostopic pub /emergency_stop std_msgs/Bool "data: true" --once
```

**ROS 2:**
```bash
ros2 topic pub /emergency_stop std_msgs/msg/Bool "{data: true}" --once
```

Robot akan langsung berhenti dan misi dihentikan.

---

## Lisensi

Project ini dilisensikan di bawah [MIT License](https://opensource.org/licenses/MIT).
