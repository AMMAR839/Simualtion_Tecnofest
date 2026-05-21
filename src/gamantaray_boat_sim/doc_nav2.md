# Dokumentasi Kode Nav2 ASV Gamantaray

Dokumen ini menjelaskan bagian kode yang dipakai untuk mode navigasi
`navigation_mode:=nav2` pada simulasi ASV Gamantaray. Fokusnya adalah alur
data dari Gazebo, ROS 2 bridge, Nav2, LiDAR, costmap, recovery, waypoint
mission, sampai command thrust propeller.

## Cara Menjalankan Mode Nav2

Command utama:

```bash
cd /home/ammar/Documents/ws_tecnofest
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch gamantaray_boat_sim simple_ocean_gamantaray.launch.py navigation_mode:=nav2
```

Dengan target kamikaze Course 3:

```bash
ros2 launch gamantaray_boat_sim simple_ocean_gamantaray.launch.py navigation_mode:=nav2 target_color:=red
```

Pilihan `target_color` yang tersedia mengikuti world saat ini:

- `red`
- `green`
- `black`

## Ringkasan Arsitektur Nav2

Alur utama command kapal adalah:

```text
Gazebo sensors
  -> ros_gz_bridge
  -> /asv/odom, /asv/lidar/scan_raw, /asv/camera/front/image
  -> lidar_scan_filter
  -> /asv/lidar/scan
  -> Nav2 local costmap, STVL, collision monitor, waypoint mission
  -> /cmd_vel_nav
  -> velocity_smoother
  -> /cmd_vel_nav2_smoothed
  -> collision_monitor
  -> /cmd_vel_nav2_collision_checked
  -> asv_nav2_safety_recovery
  -> /cmd_vel
  -> cmd_vel_to_thrusters
  -> Gazebo left/right thruster command topics
```

Topic thrust final ke Gazebo:

```text
/model/gamantaray_boat/joint/left_propeller_joint/cmd_thrust
/model/gamantaray_boat/joint/right_propeller_joint/cmd_thrust
```

Nav2 tidak langsung menggerakkan propeller. Nav2 hanya membuat command
kecepatan. Command tersebut melewati beberapa lapisan pengaman sebelum
diterjemahkan menjadi gaya dorong kiri dan kanan.

## File Utama Nav2

| File | Fungsi |
|---|---|
| `launch/simple_ocean_gamantaray.launch.py` | Launch utama Gazebo, bridge, sensor, Nav2, RViz, recovery, waypoint mission, dan kamikaze. |
| `launch/nav2_asv.launch.py` | Launch khusus node Nav2: controller, planner, smoother, behavior server, collision monitor, BT navigator, waypoint follower, lifecycle manager. |
| `config/nav2_params.yaml` | Parameter utama Nav2, DWB local planner, costmap, STVL, collision monitor, velocity smoother, dan behavior server. |
| `config/tecnofest_waypoints.yaml` | Daftar startpoint dan waypoint GN1 sampai GN5. |
| `config/tecnofest_nav2.rviz` | Tampilan RViz untuk path, local costmap, local window, marker kapal, kamera, dan deteksi buoy. |
| `gamantaray_boat_sim/nav2_waypoint_mission.py` | Node misi waypoint GN1 sampai GN5 menggunakan `BasicNavigator`. |
| `gamantaray_boat_sim/asv_nav2_safety_recovery.py` | Safety layer setelah Nav2 untuk stop, mundur, belok, lalu lanjut saat obstacle terlalu dekat. |
| `gamantaray_boat_sim/lidar_scan_filter.py` | Filter LiDAR raw agar body kapal, noise dekat, dan return tidak valid tidak masuk costmap. |
| `gamantaray_boat_sim/lidar_scan_to_pointcloud.py` | Konversi LaserScan ke PointCloud2 untuk STVL local costmap. |
| `gamantaray_boat_sim/cmd_vel_to_thrusters.py` | Mengubah `/cmd_vel` menjadi gaya dorong propeller kiri dan kanan. |
| `gamantaray_boat_sim/target_buoy_detector.py` | Deteksi warna target buoy dari kamera untuk Course 3. |
| `gamantaray_boat_sim/kamikaze_engagement.py` | Mode terminal setelah WP5 untuk mengincar dan menabrak target buoy. |
| `gamantaray_boat_sim/odom_tf_broadcaster.py` | Publish TF `odom -> base_link` dari `/asv/odom`. |
| `gamantaray_boat_sim/rviz_boat_marker.py` | Marker kapal ringan di RViz. |
| `gamantaray_boat_sim/lidar_obstacle_marker.py` | Marker visual hasil clustering buoy dari LiDAR. |
| `gamantaray_boat_sim/local_window_marker.py` | Marker lingkaran local window sesuai radius LiDAR. |
| `gamantaray_nav2_bt_plugins/behavior_trees/asv_navigate_to_pose.xml` | Behavior Tree custom untuk navigasi ASV. |
| `gamantaray_nav2_bt_plugins/src/asv_obstacle_window_condition.cpp` | Plugin BT custom `AsvObstacleWindow`. |

## Launch Utama

File:

```text
src/gamantaray_boat_sim/launch/simple_ocean_gamantaray.launch.py
```

Launch ini adalah entrypoint utama simulasi. Argumen penting:

| Argument | Fungsi |
|---|---|
| `navigation_mode:=nav2` | Mengaktifkan mode Nav2. |
| `use_nav2:=true` | Mengaktifkan stack Nav2. |
| `use_rviz:=true` | Membuka RViz. |
| `use_gazebo_gui:=true` | Membuka GUI Gazebo. |
| `target_color:=red|green|black` | Warna target Course 3. |
| `enable_kamikaze:=true|false` | Mengaktifkan mode kamikaze setelah WP5. |
| `enable_nav2_recovery:=true|false` | Mengaktifkan safety recovery setelah Nav2. |
| `show_lidar_rays:=true|false` | Menampilkan ray LiDAR di RViz jika diperlukan debug. Default dibuat false agar ringan. |

Launch ini juga mengatur environment Gazebo:

```text
GZ_SIM_RESOURCE_PATH
GZ_SIM_SYSTEM_PLUGIN_PATH
GZ_RENDERING_PLUGIN_PATH
LD_LIBRARY_PATH
```

Tujuannya agar Gazebo bisa menemukan model, world, plugin, dan resource lokal
package.

## Bridge Gazebo ke ROS 2

Bridge dibuat di `simple_ocean_gamantaray.launch.py` melalui
`ros_gz_bridge/parameter_bridge`.

Topic sensor utama:

| ROS topic | Pesan | Fungsi |
|---|---|---|
| `/clock` | `rosgraph_msgs/msg/Clock` | Waktu simulasi. |
| `/asv/odom` | `nav_msgs/msg/Odometry` | Pose dan twist kapal dari Gazebo. |
| `/asv/lidar/scan_raw` | `sensor_msgs/msg/LaserScan` | LiDAR mentah dari Gazebo. |
| `/asv/camera/front/image` | `sensor_msgs/msg/Image` | Kamera depan untuk Course 3. |
| `/asv/camera/front/camera_info` | `sensor_msgs/msg/CameraInfo` | Informasi kamera depan. |
| `/asv/imu/data` | `sensor_msgs/msg/Imu` | IMU simulasi. |
| `/asv/gps/fix` | `sensor_msgs/msg/NavSatFix` | GPS simulasi. |

Untuk Nav2 saat ini, pose utama berasal dari `/asv/odom` dan TF
`odom -> base_link`. GPS dan IMU tersedia, tetapi belum menjadi sumber fusion
utama Nav2.

## TF dan Frame

File:

```text
src/gamantaray_boat_sim/gamantaray_boat_sim/odom_tf_broadcaster.py
```

Node ini membaca `/asv/odom`, lalu publish transform:

```text
odom -> base_link
```

Transform ini wajib stabil karena Nav2, RViz, local costmap, dan marker kapal
mengandalkannya. Jika TF ini putus, gejala umumnya:

- RViz menampilkan `No transform to fixed frame [odom]`.
- Boat marker berkedip atau hilang.
- Costmap tidak sinkron dengan posisi kapal.
- Nav2 gagal menghitung pose robot.

Static TF sensor dibuat di launch:

```text
base_link -> lidar_link
base_link -> gamantaray_boat/base_link/lidar_sensor
base_link -> front_camera_link
base_link -> gamantaray_boat/base_link/front_camera_sensor
map -> odom
```

`map -> odom` dibuat static karena konfigurasi default sekarang adalah mapless
berbasis odom, bukan SLAM atau GPS localization penuh.

## Launch Nav2

File:

```text
src/gamantaray_boat_sim/launch/nav2_asv.launch.py
```

Launch ini menjalankan node Nav2 secara eksplisit, bukan memakai bringup
default. Node yang dijalankan:

| Node | Package | Fungsi |
|---|---|---|
| `controller_server` | `nav2_controller` | Local planner DWB, menghasilkan `/cmd_vel_nav`. |
| `planner_server` | `nav2_planner` | Global planner, membuat path global `/plan`. |
| `smoother_server` | `nav2_smoother` | Smoothing path. |
| `behavior_server` | `nav2_behaviors` | Behavior recovery seperti backup, wait, drive on heading. |
| `velocity_smoother` | `nav2_velocity_smoother` | Menghaluskan command dari controller. |
| `collision_monitor` | `nav2_collision_monitor` | Stop/slowdown jika obstacle masuk zona bahaya. |
| `bt_navigator` | `nav2_bt_navigator` | Menjalankan Behavior Tree navigasi. |
| `waypoint_follower` | `nav2_waypoint_follower` | Support Nav2 waypoint API. |
| `lifecycle_manager_navigation` | `nav2_lifecycle_manager` | Mengaktifkan lifecycle semua node Nav2. |

Lifecycle nodes:

```text
controller_server
smoother_server
planner_server
behavior_server
velocity_smoother
collision_monitor
bt_navigator
waypoint_follower
```

## Pipeline Command Nav2

Pipeline command dibuat supaya Nav2 tidak bisa langsung memaksa kapal maju
ketika obstacle terlalu dekat.

```text
controller_server
  -> /cmd_vel_nav
velocity_smoother
  -> /cmd_vel_nav2_smoothed
collision_monitor
  -> /cmd_vel_nav2_collision_checked
asv_nav2_safety_recovery
  -> /cmd_vel
cmd_vel_to_thrusters
  -> thrust kiri/kanan Gazebo
```

Alasan dibuat berlapis:

1. `controller_server` fokus mengikuti path.
2. `velocity_smoother` membuat command lebih halus.
3. `collision_monitor` menghentikan atau memperlambat command jika ada obstacle
   pada polygon kritis.
4. `asv_nav2_safety_recovery` memberi logika ASV: stop, mundur, belok, lalu
   lanjut.
5. `cmd_vel_to_thrusters` menerjemahkan velocity command menjadi differential
   thrust kiri-kanan.

## Parameter Nav2

File:

```text
src/gamantaray_boat_sim/config/nav2_params.yaml
```

### BT Navigator

Bagian `bt_navigator` memakai:

```yaml
global_frame: odom
robot_base_frame: base_link
odom_topic: /asv/odom
```

Plugin custom yang dimuat:

```yaml
plugin_lib_names:
  - gamantaray_asv_obstacle_window_condition_bt_node
```

Artinya Behavior Tree Nav2 bisa memakai node custom:

```xml
<AsvObstacleWindow ... />
```

### Controller Server

Controller yang dipakai adalah:

```yaml
FollowPath:
  plugin: "dwb_core::DWBLocalPlanner"
```

DWB local planner memilih kombinasi kecepatan maju dan yaw terbaik berdasarkan
path, goal, obstacle, dan costmap.

Parameter penting:

| Parameter | Fungsi |
|---|---|
| `max_vel_x` | Batas kecepatan maju ASV. |
| `max_vel_theta` | Batas yaw rate untuk belok kanan/kiri. |
| `acc_lim_x` | Batas akselerasi linear. |
| `acc_lim_theta` | Batas akselerasi yaw. |
| `vtheta_samples` | Jumlah sampling yaw. Makin besar, pilihan belok makin banyak. |
| `sim_time` | Horizon simulasi trajectory DWB. |
| `BaseObstacle.scale` | Bobot menghindari obstacle. |
| `PathAlign.scale` | Bobot agar orientasi mengikuti path. |
| `PathDist.scale` | Bobot agar kapal dekat dengan path global. |
| `GoalDist.scale` | Bobot mengejar goal. |

Untuk ASV, `yaw_goal_tolerance` dibuat longgar karena kapal tidak perlu berhenti
tepat menghadap orientasi waypoint seperti robot darat.

### Local Costmap

Local costmap adalah costmap utama untuk obstacle avoidance.

Konfigurasi penting:

```yaml
global_frame: odom
robot_base_frame: base_link
rolling_window: true
width: 20
height: 20
resolution: 0.10
plugins: ["obstacle_layer", "stvl_layer", "inflation_layer"]
```

Maknanya:

- Costmap selalu mengikuti kapal.
- Area lokal sekitar kapal berukuran 20 m x 20 m.
- Resolusi grid 0.10 m.
- Obstacle dari LiDAR masuk ke `obstacle_layer` dan `stvl_layer`.
- `inflation_layer` memberi margin aman di sekitar buoy.

Local costmap inilah yang tampil di RViz sebagai:

```text
/local_costmap/costmap
/local_costmap/costmap_updates
```

### Obstacle Layer

Obstacle layer memakai LaserScan terfilter:

```yaml
topic: /asv/lidar/scan
data_type: "LaserScan"
clearing: true
marking: true
obstacle_min_range: 0.75
obstacle_max_range: 9.4
raytrace_max_range: 10.0
observation_persistence: 0.35
```

`observation_persistence` memberi memori singkat agar obstacle yang sempat
terdeteksi tidak langsung hilang pada frame berikutnya. Ini penting karena buoy
bisa masuk blindspot ketika terlalu dekat atau tertutup geometri kapal.

### STVL Layer

STVL memakai PointCloud2 hasil konversi dari LaserScan:

```yaml
plugin: "spatio_temporal_voxel_layer/SpatioTemporalVoxelLayer"
topic: /asv/lidar/points_filtered
voxel_decay: 2.5
clear_after_reading: true
publish_voxel_map: true
```

Fungsinya:

- Menyimpan obstacle lokal secara sementara.
- Menghapus obstacle palsu setelah beberapa detik melalui `voxel_decay`.
- Membantu mengurangi efek noise ombak atau pantulan kecil.

Pada konfigurasi ini, STVL dipakai untuk local costmap saja. Global costmap
tetap dibuat ringan agar path global tidak terlalu kacau oleh noise lokal.

### Global Costmap

Global costmap memakai `odom` sebagai frame dan bersifat rolling:

```yaml
global_frame: odom
rolling_window: true
width: 130
height: 28
resolution: 0.35
plugins: ["inflation_layer"]
```

Obstacle layer pada global costmap disiapkan di file, tetapi tidak masuk daftar
plugin aktif. Tujuannya agar global path tetap fokus ke waypoint utama, sedangkan
avoidance dilakukan oleh local costmap dan controller.

### Planner Server

Planner global:

```yaml
GridBased:
  plugin: "nav2_navfn_planner::NavfnPlanner"
  use_astar: true
  allow_unknown: true
```

Planner ini membuat path global dari pose kapal ke waypoint aktif.

### Velocity Smoother

Velocity smoother membatasi command agar tidak terlalu kasar:

```yaml
max_velocity: [0.88, 0.0, 2.30]
min_velocity: [-0.12, 0.0, -2.30]
max_accel: [0.48, 0.0, 4.20]
max_decel: [-0.70, 0.0, -4.20]
```

Karena kapal adalah ASV, `y` selalu nol. Kapal hanya bergerak maju/mundur dan
berputar yaw.

### Collision Monitor

Collision monitor menerima command dari:

```yaml
cmd_vel_in_topic: "cmd_vel_nav2_smoothed"
```

Lalu mengeluarkan command aman ke:

```yaml
cmd_vel_out_topic: "cmd_vel_nav2_collision_checked"
```

Zona yang dipakai:

| Polygon | Fungsi |
|---|---|
| `PolygonStop` | Stop jika obstacle sangat dekat di depan kapal. |
| `PolygonSlow` | Perlambat kapal jika obstacle masuk zona lebih jauh. |

`PolygonStop`:

```yaml
points: "[[1.45, 0.62], [1.45, -0.62], [0.38, -0.62], [0.38, 0.62]]"
action_type: "stop"
```

`PolygonSlow`:

```yaml
points: "[[2.85, 1.05], [2.85, -1.05], [0.32, -1.05], [0.32, 1.05]]"
action_type: "slowdown"
slowdown_ratio: 0.32
```

Collision monitor adalah pengaman reaktif. Jika local planner terlambat
menghindar, collision monitor tetap bisa memperlambat atau menghentikan command.

## Behavior Tree Custom

File XML:

```text
src/gamantaray_nav2_bt_plugins/behavior_trees/asv_navigate_to_pose.xml
```

File plugin C++:

```text
src/gamantaray_nav2_bt_plugins/src/asv_obstacle_window_condition.cpp
```

Package:

```text
src/gamantaray_nav2_bt_plugins
```

BT ini melakukan:

1. Compute path ke goal.
2. Replan berkala pada `0.7 Hz`.
3. Cek local scan menggunakan `AsvObstacleWindow`.
4. Jika scan terlalu padat atau stale, clear local costmap sebentar.
5. Follow path dengan DWB.
6. Jika gagal, lakukan recovery seperti clear costmap, wait, dan backup.

Bagian penting:

```xml
<AsvObstacleWindow
  scan_topic="/asv/lidar/scan"
  max_scan_age="0.9"
  min_range="0.75"
  forward_distance="4.0"
  half_width="1.2"
  max_points_in_window="220"/>
```

Plugin `AsvObstacleWindow` menghitung jumlah titik LiDAR pada jendela depan.
Jika terlalu padat, BT menganggap ada clutter/noise lokal dan memicu clear local
costmap. Ini dibuat untuk mengurangi efek costmap yang penuh oleh noise ombak.

Recovery backup pada BT:

```xml
<BackUp backup_dist="0.80" backup_speed="0.20"/>
```

Backup ini berbeda dari safety recovery Python. Backup BT aktif saat Nav2
mengalami failure, sedangkan safety recovery Python aktif sebagai guard command
real-time setelah Nav2.

## Waypoint Mission

File:

```text
src/gamantaray_boat_sim/gamantaray_boat_sim/nav2_waypoint_mission.py
```

Waypoint:

```text
src/gamantaray_boat_sim/config/tecnofest_waypoints.yaml
```

Isi waypoint saat ini:

| Nama | X | Y |
|---|---:|---:|
| Startpoint | -49.0 | -8.0 |
| GN1 | -40.0 | -4.0 |
| GN2 | -32.0 | 4.5 |
| GN3 | -24.0 | -4.0 |
| GN4 | -16.0 | 4.5 |
| GN5 | 36.5 | -1.2 |

Startpoint adalah posisi awal kapal. Misi Nav2 tetap dimulai dari GN1 sampai GN5.

Node `nav2_waypoint_mission.py` melakukan langkah berikut:

1. Membaca file YAML waypoint.
2. Menunggu runtime siap:
   - `/clock`
   - `/asv/odom`
   - `/asv/lidar/scan`
   - TF `odom -> base_link`
3. Menunggu Nav2 aktif dengan `BasicNavigator`.
4. Mengirim goal GN1.
5. Jika waypoint tercapai atau sudah terlewati secara valid, lanjut ke waypoint
   berikutnya.
6. Setelah GN5 tercapai, publish:

```text
/asv/navigation/status = nav2_waypoints_succeeded
```

Parameter penting dari launch:

| Parameter | Nilai | Fungsi |
|---|---:|---|
| `waypoint_acceptance_radius_m` | 5.50 | Radius waypoint dianggap tercapai. |
| `waypoint_passed_margin_m` | 2.00 | Margin setelah melewati waypoint. |
| `waypoint_passed_cross_track_m` | 6.50 | Batas cross-track agar waypoint yang terlewati masih dianggap valid. |
| `max_goal_retries` | 1 | Jumlah retry jika goal gagal. |

Log status utama:

```text
waiting_for_runtime:...
runtime_ready
nav2_waypoints_started:5
nav2_waypoint_goal:1:gn1_start
nav2_waypoint_reached:1:gn1_start
...
nav2_waypoints_succeeded
```

## LiDAR Filtering

File:

```text
src/gamantaray_boat_sim/gamantaray_boat_sim/lidar_scan_filter.py
```

Input:

```text
/asv/lidar/scan_raw
```

Output:

```text
/asv/lidar/scan
```

Fungsi utama:

- Memaksa frame ke `lidar_link` agar TF konsisten.
- Restamp scan agar sinkron dengan sim time.
- Menghapus return terlalu dekat dengan body kapal.
- Menghapus max-range return yang tidak berguna.
- Membatasi angle LiDAR yang dipakai.
- Menghapus cluster kecil atau terlalu lebar sesuai parameter.

Parameter penting dari launch:

```text
min_valid_range = 0.75
keep_angle_min = -2.05
keep_angle_max = 2.05
cluster_gap_m = 0.45
min_cluster_points = 1
max_cluster_width_m = 1.40
sensor_x = 0.95
```

LiDAR terfilter inilah yang dipakai oleh:

- local costmap
- STVL converter
- collision monitor
- safety recovery
- marker obstacle RViz
- kamikaze contact detection

## LaserScan ke PointCloud2

File:

```text
src/gamantaray_boat_sim/gamantaray_boat_sim/lidar_scan_to_pointcloud.py
```

Input:

```text
/asv/lidar/scan
```

Output:

```text
/asv/lidar/points_filtered
```

Node ini memakai `laser_geometry.LaserProjection` untuk mengubah LaserScan
menjadi PointCloud2. Topic PointCloud2 ini masuk ke STVL local costmap.

## Safety Recovery Setelah Nav2

File:

```text
src/gamantaray_boat_sim/gamantaray_boat_sim/asv_nav2_safety_recovery.py
```

Node ini adalah pengaman command terakhir untuk mode Nav2.

Input:

```text
/cmd_vel_nav2_collision_checked
/cmd_vel_nav2_smoothed
/asv/kamikaze/cmd_vel
/asv/lidar/scan
/asv/navigation/status
```

Output:

```text
/cmd_vel
/asv/recovery/status
```

State yang dipakai:

| State | Fungsi |
|---|---|
| `normal` | Teruskan command Nav2. |
| `stop` | Stop sebentar jika obstacle terlalu dekat di depan. |
| `backup` | Mundur untuk keluar dari blindspot atau kontak dekat. |
| `turn` | Belok ke sisi LiDAR yang lebih kosong. |
| `recover` | Cooldown lalu kembali normal. |
| `kamikaze` | Setelah WP5, teruskan command dari node kamikaze. |

Parameter penting dari launch:

| Parameter | Nilai | Fungsi |
|---|---:|---|
| `danger_distance_m` | 1.85 | Jarak depan yang dianggap bahaya. |
| `front_half_angle_deg` | 28.0 | Sektor depan untuk trigger recovery. |
| `min_valid_range_m` | 0.75 | Abaikan pembacaan terlalu dekat/self. |
| `min_cluster_points` | 3 | Jumlah titik minimal agar obstacle valid. |
| `front_obstacle_memory_s` | 1.50 | Memori obstacle depan saat LiDAR blindspot. |
| `backup_speed_mps` | -0.22 | Kecepatan mundur saat recovery. |
| `turn_speed_radps` | 1.05 | Kecepatan yaw saat recovery turn. |

Node ini penting untuk Parkur 2, karena buoy yang terlalu dekat bisa hilang dari
scan akibat blindspot. Memori `front_obstacle_memory_s` membuat obstacle tetap
dianggap berbahaya beberapa saat.

## Mapping Command ke Thruster

File:

```text
src/gamantaray_boat_sim/gamantaray_boat_sim/cmd_vel_to_thrusters.py
```

Input:

```text
/cmd_vel
```

Output:

```text
/model/gamantaray_boat/joint/left_propeller_joint/cmd_thrust
/model/gamantaray_boat/joint/right_propeller_joint/cmd_thrust
```

Rumus dasar:

```text
throttle = linear.x -> thrust maju/mundur
turn = angular.z -> beda thrust kiri/kanan

left  = throttle - turn
right = throttle + turn
```

Parameter penting dari launch:

| Parameter | Nilai | Fungsi |
|---|---:|---|
| `max_forward_thrust_n` | 58.0 | Batas dorong maju. |
| `max_reverse_thrust_n` | 28.0 | Batas dorong mundur. |
| `max_speed_cmd_mps` | 0.92 | Skala command linear dari Nav2. |
| `yaw_to_thrust_n_per_radps` | 152.0 | Kekuatan differential thrust saat belok. |
| `max_yaw_rate_cmd_radps` | 2.35 | Batas command yaw. |
| `thrust_slew_rate_nps` | 125.0 | Batas perubahan thrust agar tidak kasar. |
| `turn_throttle_reduction` | 0.28 | Mengurangi throttle saat belok tajam. |

Jika kapal sulit belok kanan/kiri, parameter utama yang biasanya dituning adalah:

- `yaw_to_thrust_n_per_radps`
- `max_yaw_rate_cmd_radps`
- `turn_throttle_reduction`
- `min_turn_throttle_fraction`
- DWB `max_vel_theta`
- DWB `vtheta_samples`

## RViz untuk Nav2

File:

```text
src/gamantaray_boat_sim/config/tecnofest_nav2.rviz
```

Display utama:

| Display | Topic | Fungsi |
|---|---|---|
| `Grid` | `odom` | Grid referensi. |
| `Boat Model` | `/asv/visualization/boat_model` | Marker kapal sederhana. |
| `Local Window` | `/asv/visualization/local_window` | Lingkaran radius LiDAR/local window. |
| `LiDAR Buoy Detections` | `/asv/perception/lidar_obstacles` | Cluster buoy hasil LiDAR. |
| `Local Obstacles` | `/local_costmap/costmap` | Local costmap utama. |
| `Local Footprint` | `/local_costmap/published_footprint` | Footprint kapal. |
| `Front Camera` | `/asv/camera/front/image` | Kamera untuk Course 3. |
| `Global Path` | `/plan` | Path global Nav2. |
| `Smoothed Path` | `/plan_smoothed` | Path hasil smoother. |

Fixed frame:

```text
odom
```

View target frame:

```text
base_link
```

LiDAR ray tidak ditampilkan default agar RViz lebih ringan. Jika perlu debug:

```bash
ros2 launch gamantaray_boat_sim simple_ocean_gamantaray.launch.py navigation_mode:=nav2 show_lidar_rays:=true
```

## Marker Kapal

File:

```text
src/gamantaray_boat_sim/gamantaray_boat_sim/rviz_boat_marker.py
```

Marker kapal dibuat ringan. Default launch:

```text
use_heavy_mesh = False
detail_level = simple
publish_in_fixed_frame = False
frame_locked = True
```

Artinya RViz cukup menampilkan representasi sederhana berbentuk box pada frame
`base_link`. Ini mengurangi flicker dan beban RViz dibanding mesh kapal penuh.

## Local Window

File:

```text
src/gamantaray_boat_sim/gamantaray_boat_sim/local_window_marker.py
```

Node ini hanya visualisasi lingkaran local window:

```text
/asv/visualization/local_window
```

Pada launch, radius:

```text
radius_m = 10.0
```

Diameter local window = 20 m, sama dengan diameter area LiDAR/local costmap yang
dipakai untuk monitoring sekitar kapal. Marker ini hanya border lingkaran, bukan
obstacle. Obstacle sebenarnya tetap berasal dari local costmap dan LiDAR cluster.

## Marker Deteksi Buoy LiDAR

File:

```text
src/gamantaray_boat_sim/gamantaray_boat_sim/lidar_obstacle_marker.py
```

Input:

```text
/asv/lidar/scan
```

Output:

```text
/asv/perception/lidar_obstacles
```

Node ini melakukan clustering titik LiDAR, lalu menampilkan buoy sebagai marker
silinder di RViz. Marker hanya dibuat jika cluster berada di dalam local window:

```text
enforce_local_window = True
local_window_radius_m = 10.0
```

Ini hanya visualisasi. Penghindaran sebenarnya tetap memakai Nav2 local costmap,
collision monitor, dan safety recovery.

## Course 3 Kamikaze Engagement

Task 3 aktif setelah misi waypoint selesai:

```text
/asv/navigation/status = nav2_waypoints_succeeded
```

### Target Buoy Detector

File:

```text
src/gamantaray_boat_sim/gamantaray_boat_sim/target_buoy_detector.py
```

Input:

```text
/asv/camera/front/image
```

Output:

```text
/asv/perception/target_selection
```

Logika:

- Ambil frame kamera.
- Ambil warna target dari parameter `target_color`.
- Sampling pixel setiap 4 pixel.
- Cocokkan threshold warna sederhana.
- Hitung centroid warna target.
- Publish offset target dari tengah gambar.

Format output:

```text
target=green visible=true pixels=... offset=...
target=green visible=false
unsupported_color:...
```

Warna yang didukung:

```text
red, green, black
```

### Kamikaze Engagement

File:

```text
src/gamantaray_boat_sim/gamantaray_boat_sim/kamikaze_engagement.py
```

Input:

```text
/asv/navigation/status
/asv/perception/target_selection
/asv/lidar/scan
/asv/odom
```

Output:

```text
/asv/kamikaze/cmd_vel
/asv/kamikaze/status
```

Logika:

1. Tunggu sampai Nav2 publish `nav2_waypoints_succeeded`.
2. Aktifkan mode kamikaze.
3. Jika target terlihat kamera:
   - `offset > 0` berarti target di kanan gambar.
   - command yaw dibuat negatif agar kapal mengarah ke kanan.
   - jika target sudah center, kapal maju lebih cepat.
4. Jika target tidak terlihat:
   - gunakan coarse homing ke posisi target berdasarkan warna.
   - jika tidak bisa, sweep/search.
5. Kontak dianggap terjadi jika LiDAR depan membaca target dalam jarak
   `contact_distance_m`.
6. Setelah kontak, kapal tetap push beberapa saat lalu publish:

```text
kamikaze_succeeded
```

Parameter target Course 3:

| Warna | X | Y |
|---|---:|---:|
| Red | 46.0 | 6.6 |
| Green | 46.0 | 1.8 |
| Black | 46.0 | -3.0 |

## Topic Status Penting

| Topic | Isi |
|---|---|
| `/asv/navigation/status` | Status waypoint Nav2. |
| `/asv/recovery/status` | Status safety recovery. |
| `/asv/kamikaze/status` | Status Task 3 kamikaze. |
| `/asv/control/thruster_status` | Status thrust kiri/kanan. |
| `/asv/perception/target_selection` | Status target kamera. |

Contoh cek:

```bash
ros2 topic echo /asv/navigation/status
ros2 topic echo /asv/recovery/status
ros2 topic echo /asv/kamikaze/status
ros2 topic echo /asv/control/thruster_status
```

## Cara Debug Nav2

### Cek Sensor dan TF

```bash
ros2 topic hz /asv/odom
ros2 topic hz /asv/lidar/scan_raw
ros2 topic hz /asv/lidar/scan
ros2 run tf2_ros tf2_echo odom base_link
```

Jika `/asv/odom` atau TF putus, Nav2 dan RViz akan bermasalah.

### Cek Nav2 Lifecycle

```bash
ros2 lifecycle get /controller_server
ros2 lifecycle get /planner_server
ros2 lifecycle get /bt_navigator
ros2 lifecycle get /collision_monitor
```

Semua node penting harus active.

### Cek Command Chain

```bash
ros2 topic echo /cmd_vel_nav
ros2 topic echo /cmd_vel_nav2_smoothed
ros2 topic echo /cmd_vel_nav2_collision_checked
ros2 topic echo /cmd_vel
```

Jika `/cmd_vel_nav` ada tetapi `/cmd_vel` kosong, masalah kemungkinan ada pada
collision monitor atau safety recovery.

### Cek Costmap

```bash
ros2 topic echo /local_costmap/costmap --once
ros2 topic hz /local_costmap/costmap
ros2 topic hz /local_costmap/costmap_updates
```

Jika local costmap tidak muncul di RViz:

1. Pastikan `/asv/lidar/scan` publish.
2. Pastikan TF `odom -> base_link -> lidar_link` ada.
3. Pastikan `collision_monitor` dan `local_costmap` active.
4. Pastikan RViz fixed frame adalah `odom`.

## Parameter yang Paling Sering Dituning

### Agar Kapal Lebih Cepat ke Waypoint

Edit:

```text
config/nav2_params.yaml
launch/simple_ocean_gamantaray.launch.py
```

Parameter:

- `FollowPath.max_vel_x`
- `velocity_smoother.max_velocity[0]`
- `cmd_vel_to_thrusters.max_forward_thrust_n`
- `cmd_vel_to_thrusters.max_speed_cmd_mps`
- `waypoint_acceptance_radius_m`

### Agar Kapal Lebih Berani Belok

Parameter:

- `FollowPath.max_vel_theta`
- `FollowPath.vtheta_samples`
- `FollowPath.acc_lim_theta`
- `cmd_vel_to_thrusters.yaw_to_thrust_n_per_radps`
- `cmd_vel_to_thrusters.turn_throttle_reduction`

### Agar Tidak Menabrak Buoy

Parameter:

- `local_costmap.obstacle_layer.scan.observation_persistence`
- `local_costmap.inflation_layer.inflation_radius`
- `collision_monitor.PolygonStop.points`
- `collision_monitor.PolygonSlow.points`
- `asv_nav2_safety_recovery.danger_distance_m`
- `asv_nav2_safety_recovery.front_obstacle_memory_s`
- `asv_nav2_safety_recovery.backup_duration_s`
- `asv_nav2_safety_recovery.turn_duration_s`

### Agar Waypoint Tidak Membuat Kapal Putar Balik

Parameter:

- `waypoint_acceptance_radius_m`
- `waypoint_passed_margin_m`
- `waypoint_passed_cross_track_m`

Radius waypoint dibuat cukup besar karena kapal punya inertia dan tidak bisa
berhenti presisi seperti robot darat. Jika radius terlalu kecil, kapal bisa
melewati waypoint lalu berputar balik untuk mengejar titik yang sudah lewat.

## Batasan Implementasi Saat Ini

1. Nav2 memakai odometry Gazebo sebagai pose utama, bukan GPS fusion penuh.
2. GPS dan IMU sudah tersedia sebagai topic, tetapi belum dipakai sebagai
   `robot_localization` + `navsat_transform_node` default.
3. Kamera Course 3 memakai threshold warna sederhana, bukan model ML.
4. STVL dipakai dari PointCloud2 hasil konversi 2D LiDAR, bukan dari 3D LiDAR.
5. Global costmap sengaja ringan; obstacle avoidance utama ada di local costmap,
   collision monitor, dan safety recovery.
6. Mode ArduPilot adalah opsi terpisah. Dokumen ini fokus pada
   `navigation_mode:=nav2`.

## Ringkasan Praktis

Untuk mode Nav2, file paling penting adalah:

```text
launch/simple_ocean_gamantaray.launch.py
launch/nav2_asv.launch.py
config/nav2_params.yaml
config/tecnofest_waypoints.yaml
gamantaray_boat_sim/nav2_waypoint_mission.py
gamantaray_boat_sim/asv_nav2_safety_recovery.py
gamantaray_boat_sim/cmd_vel_to_thrusters.py
gamantaray_nav2_bt_plugins/behavior_trees/asv_navigate_to_pose.xml
gamantaray_nav2_bt_plugins/src/asv_obstacle_window_condition.cpp
```

Jika kapal tidak bergerak, cek urutan ini:

1. `/clock`, `/asv/odom`, `/asv/lidar/scan`.
2. TF `odom -> base_link`.
3. Lifecycle Nav2 active.
4. `/asv/navigation/status`.
5. `/cmd_vel_nav`, `/cmd_vel_nav2_smoothed`,
   `/cmd_vel_nav2_collision_checked`, `/cmd_vel`.
6. `/asv/control/thruster_status`.
7. Topic thrust Gazebo kiri dan kanan.

Jika kapal menabrak obstacle, cek urutan ini:

1. Apakah obstacle muncul di `/asv/lidar/scan`.
2. Apakah obstacle muncul di `/local_costmap/costmap`.
3. Apakah collision monitor mengurangi command.
4. Apakah `/asv/recovery/status` masuk `stop`, `backup`, atau `turn`.
5. Apakah `cmd_vel_to_thrusters` menghasilkan beda thrust kiri-kanan cukup besar.
