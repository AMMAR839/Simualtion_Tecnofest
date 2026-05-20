# Optimasi Navigasi ASV

Dokumen ini menjelaskan bagian yang paling penting untuk mengatur navigasi,
costmap, LiDAR, dan waypoint pada simulasi ASV TEKNOFEST.

## Konsep Navigasi

Simulasi ini memakai Nav2 mapless berbasis frame `odom`. Alur dasarnya:

```text
waypoint YAML -> nav2_waypoint_mission -> Nav2 planner/controller -> /cmd_vel
/cmd_vel -> cmd_vel_to_thrusters -> Gazebo thruster
/asv/lidar/scan_raw -> lidar_scan_filter -> /asv/lidar/scan -> local costmap
```

Global path dipakai sebagai rute referensi menuju waypoint. Obstacle avoidance
dilakukan oleh local costmap dan local planner. Jadi garis global tidak harus
selalu menghindari semua buoy dari awal; kapal akan membelok saat obstacle masuk
ke jangkauan LiDAR dan local costmap.

Sistem sekarang punya dua opsi navigasi:

```bash
ros2 launch gamantaray_boat_sim simple_ocean_gamantaray.launch.py navigation_mode:=nav2
ros2 launch gamantaray_boat_sim simple_ocean_gamantaray.launch.py navigation_mode:=ardupilot
```

`nav2` tetap menjadi mode default dan memakai waypoint node lama. `ardupilot`
adalah opsi tambahan. Mode ArduPilot menjalankan ArduRover SITL, MAVROS,
bridge LiDAR proximity, dan node upload mission ArduPilot. Nav2 tidak dihapus
dan tidak dipaksa diganti.

## Local Window Lingkaran

`Local Window` di RViz adalah visualisasi area lokal yang sedang diamati kapal.
Area ini dibuat lingkaran agar sesuai dengan pola jangkauan LiDAR.

Nilai saat ini:

```text
LiDAR radius efektif: 10 m
Local window diameter: 20 m
Local costmap width/height: 20 m x 20 m
```

File yang mengatur:

```text
models/gamantaray_boat/model.sdf
  -> <range><max>10.0</max>

launch/simple_ocean_gamantaray.launch.py
  -> local_window_marker radius_m
  -> lidar_scan_to_pointcloud range_cutoff

config/nav2_params.yaml
  -> local_costmap width / height
  -> obstacle_max_range / obstacle_range

config/tecnofest_nav2.rviz
  -> display Local Window dan Local Obstacles
```

Jika radius LiDAR diubah, ubah semua nilai terkait secara konsisten. Contoh:
kalau radius LiDAR dibuat `12 m`, maka diameter local window dan ukuran local
costmap harus menjadi `24 m x 24 m`.

Border local window hanya dipakai sebagai batas jangkauan LiDAR. Border ini
tidak memakai warna deteksi obstacle. Warna aqua dan ungu hanya dipakai untuk
marker hasil deteksi buoy dari `/asv/perception/lidar_obstacles`.

Marker debug obstacle juga dibatasi oleh lingkaran ini. Titik/cluster di luar
radius `10 m` dari `base_link` tidak dipublish sebagai marker, sehingga RViz
hanya menampilkan hasil deteksi buoy yang berada di dalam local window.

Pembacaan LiDAR yang tepat berada di batas maksimum range juga difilter:

```text
drop_max_range_returns: true
max_range_return_margin_m: 0.60
```

Tujuannya agar `/local_costmap/costmap` tidak mewarnai border lingkaran LiDAR.
Warna aqua/ungu pada RViz hanya boleh muncul dari obstacle/buoy yang benar-benar
terdeteksi di dalam local window. Karena itu local costmap tetap aktif, tetapi
range marking obstacle dibatasi ke `9.4 m`, sementara radius border LiDAR tetap
`10 m`. Selisih `0.6 m` ini adalah buffer agar return di tepi sensor tidak
dibaca sebagai obstacle.

## Mengoptimalkan Costmap

Parameter utama ada di:

```text
src/gamantaray_boat_sim/config/nav2_params.yaml
```

Bagian yang paling sering dituning:

```yaml
local_costmap:
  local_costmap:
    ros__parameters:
      width: 20
      height: 20
      resolution: 0.10
      plugins: ["obstacle_layer", "stvl_layer", "inflation_layer"]
```

Panduan tuning:

- `resolution` lebih kecil membuat costmap lebih detail, tetapi lebih berat.
- `inflation_radius` lebih besar membuat kapal lebih jauh dari buoy, tetapi bisa
  membuat jalur terasa sempit.
- `cost_scaling_factor` lebih besar membuat zona bahaya turun lebih cepat dari
  obstacle.
- `voxel_decay` STVL mengatur berapa lama titik obstacle sementara disimpan.
  Nilai pendek membantu menghilangkan pantulan palsu dari ombak.

Untuk simulasi ini, obstacle utama berasal dari:

```text
/asv/lidar/scan
/asv/lidar/points_filtered
```

## Mengoptimalkan Gerak Kapal

Gerak kapal dikontrol oleh dua bagian:

```text
config/nav2_params.yaml
  -> DWB local planner

launch/simple_ocean_gamantaray.launch.py
  -> cmd_vel_to_thrusters
```

Jika kapal kurang berani belok, cek:

```yaml
max_vel_theta
vtheta_samples
BaseObstacle.scale
PathAlign.scale
PathDist.scale
GoalDist.scale
```

Jika kapal menerima perintah belok tetapi fisiknya lambat belok, cek:

```text
yaw_to_thrust_n_per_radps
max_yaw_rate_cmd_radps
turn_throttle_reduction
min_turn_throttle_fraction
```

## Profil Fast Mission ke WP5

Profil saat ini dibuat lebih agresif untuk memangkas waktu tempuh ke WP5:

```yaml
config/nav2_params.yaml
  controller_frequency: 12.0
  max_vel_x: 0.88
  max_speed_xy: 0.88
  max_vel_theta: 1.95
  acc_lim_x: 0.48
  acc_lim_theta: 3.00
  xy_goal_tolerance: 2.60
```

```text
launch/simple_ocean_gamantaray.launch.py
  max_forward_thrust_n: 58.0
  max_reverse_thrust_n: 20.0
  max_speed_cmd_mps: 0.92
  yaw_to_thrust_n_per_radps: 112.0
  thrust_slew_rate_nps: 82.0
  waypoint_acceptance_radius_m: 5.50
  waypoint_advance_pause_s: 0.15
```

Behavior Tree juga dipangkas agar tidak terlalu banyak menunggu:

```text
gamantaray_nav2_bt_plugins/behavior_trees/asv_navigate_to_pose.xml
  NavigateRecovery retries: 3
  replan rate: 0.7 Hz
  local clear wait: 0.1 s
  recovery wait: 0.3 s
```

Logikanya: global planner tidak perlu terus-menerus menghitung ulang rute karena
jalur utama hanya waypoint. Obstacle avoidance tetap dikerjakan oleh local
costmap dan DWB. Dengan begitu CPU/RViz lebih ringan dan kapal tidak sering
berhenti karena recovery kecil.

Kalau kapal mulai menabrak buoy setelah profil cepat ini, turunkan bertahap:

```text
max_forward_thrust_n -> 50.0
max_vel_x -> 0.75
waypoint_acceptance_radius_m -> 5.00
```

## Pengaturan Waypoint

Koordinat waypoint ada di:

```text
src/gamantaray_boat_sim/config/tecnofest_waypoints.yaml
```

Pose marker di Gazebo ada di:

```text
src/gamantaray_boat_sim/worlds/tecnofest_asv_course.sdf
```

Jika waypoint GN di YAML diubah, marker GN di world sebaiknya ikut disamakan
agar tampilan Gazebo dan goal Nav2 tidak berbeda.

Waypoint YAML yang sama juga dipakai oleh ArduPilot:

```text
src/gamantaray_boat_sim/gamantaray_boat_sim/ardupilot_waypoint_mission.py
```

Node tersebut mengubah koordinat `odom`/ENU meter menjadi koordinat WGS84 untuk
MAVROS mission. Origin geodetik diatur di:

```text
src/gamantaray_boat_sim/config/ardupilot_nav.yaml
```

Nilai origin harus sama dengan `spherical_coordinates` di world Gazebo.

## Jarak Minimal Waypoint

Kapal ASV tidak berhenti sepresisi robot darat. Karena itu waypoint tidak perlu
disentuh tepat di titik koordinatnya. Simulasi ini memakai radius penerimaan
waypoint:

```text
waypoint_acceptance_radius_m: 5.50
```

Parameter ini ada di:

```text
src/gamantaray_boat_sim/launch/simple_ocean_gamantaray.launch.py
```

Selain radius, ada logika passed waypoint:

```text
waypoint_passed_margin_m: 2.00
waypoint_passed_cross_track_m: 6.50
```

Artinya waypoint dianggap sudah dilewati jika kapal sudah melampaui garis
waypoint pada arah segmen lintasan, selama jarak melintangnya masih masuk batas
aman. Ini mencegah kapal memutar balik hanya karena tidak menyentuh titik
waypoint secara presisi.

Nav2 goal checker juga dibuat lebih longgar:

```yaml
xy_goal_tolerance: 2.60
yaw_goal_tolerance: 3.14
```

Parameter ini ada di:

```text
src/gamantaray_boat_sim/config/nav2_params.yaml
```

Alasannya: kalau radius terlalu kecil, kapal bisa sudah melewati waypoint tetapi
Nav2 masih menganggap belum sampai. Akibatnya kapal berputar balik dan geraknya
tidak natural. Nilai `5.50 m` masih dijaga di bawah setengah jarak antar
waypoint Parkur 1 yang sekitar `11.67 m`, sehingga kapal lebih cepat lanjut ke
goal berikutnya tetapi tidak langsung melewati seluruh titik penting.

Jangan menaikkan radius waypoint terlalu besar. Jika radius terlalu besar, kapal
bisa menganggap waypoint tercapai terlalu awal dan memotong bagian penting dari
lintasan.

## Mode ArduPilot

File utama ArduPilot:

```text
src/gamantaray_boat_sim/config/ardupilot_asv.parm
src/gamantaray_boat_sim/config/ardupilot_nav.yaml
src/gamantaray_boat_sim/gamantaray_boat_sim/ardupilot_lidar_bridge.py
src/gamantaray_boat_sim/gamantaray_boat_sim/ardupilot_waypoint_mission.py
```

`ardupilot_asv.parm` berisi parameter kendaraan dan obstacle avoidance:

```text
ARMING_CHECK 0
BRD_SAFETYENABLE 0
OA_TYPE 3
OA_BR_TYPE 1
PRX1_TYPE 2
AVOID_ENABLE 7
WP_RADIUS 5.5
WP_OVERSHOOT 4.0
```

`OA_TYPE=3` berarti ArduPilot memakai Dijkstra + BendyRuler. Dijkstra dipakai
untuk rute utama ketika database obstacle tersedia. BendyRuler dipakai sebagai
local planner yang memilih arah belok paling aman saat ada obstacle di depan.
`OA_BR_TYPE=1` memilih BendyRuler horizontal, cocok untuk rover/boat.

`ARMING_CHECK=0` dan `BRD_SAFETYENABLE=0` hanya untuk SITL/Gazebo agar mode
ArduPilot tidak gagal jalan karena safety interlock simulasi. Nilai ini tidak
boleh langsung disalin ke hardware asli tanpa safety review.

`PRX1_TYPE=2` berarti proximity obstacle datang dari MAVLink/MAVROS. Pipeline
LiDAR ArduPilot adalah:

```text
/asv/lidar/scan_raw
  -> lidar_scan_filter
  -> /asv/lidar/scan
  -> ardupilot_lidar_bridge
  -> /mavros/obstacle/send
  -> ArduPilot proximity / object avoidance
```

Jangkauan proximity disamakan dengan radius LiDAR/local window `10 m`.
Jangan set `PRX1_MAX` atau `ardupilot_lidar_bridge.max_range` lebih besar dari
range LiDAR, karena obstacle di luar lingkaran lokal tidak valid.

`ardupilot_nav.yaml` mengatur upload mission dan mode:

```text
mode_before_arm: GUIDED
mode_after_upload: AUTO
arm_vehicle: true
rtl_on_finish: false
```

SITL dijalankan langsung dari binary ArduRover dan MAVROS connect ke:

```text
fcu_url: tcp://127.0.0.1:5760
```

Jalur ini lebih stabil untuk mode ini daripada bergantung ke MAVProxy sebagai
penghubung utama. Mission Planner tetap bisa dipantau lewat output GCS MAVROS
yang disediakan launch.

Mode yang disiapkan:

- `AUTO`: menjalankan mission waypoint yang diupload dari YAML.
- `GUIDED`: mode siap untuk kontrol/goal MAVLink langsung.
- `RTL`: bisa dipakai untuk kembali ke home, misalnya dengan mengubah
  `rtl_on_finish: true` atau memanggil `/mavros/set_mode`.

Status ArduPilot dapat dicek dari:

```bash
ros2 topic echo /asv/ardupilot/navigation/status
ros2 topic echo /mavros/state
```

Jika kapal masih memutar balik di ArduPilot, naikkan sedikit:

```text
config/ardupilot_asv.parm
  WP_RADIUS
  WP_OVERSHOOT

config/ardupilot_nav.yaml
  waypoint_radius_m
  waypoint_passed_margin_m
```

Jangan menaikkannya terlalu besar. Untuk Parkur 1, jarak antar waypoint sekitar
`11.67 m`, sehingga radius `5.5 m` sudah dekat batas atas yang masih masuk akal.

## Urutan Uji

Build:

```bash
cd /home/ammar/Documents/ws_tecnofest
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
```

Launch:

```bash
source install/setup.bash
ros2 launch gamantaray_boat_sim simple_ocean_gamantaray.launch.py navigation_mode:=nav2
```

Launch ArduPilot:

```bash
source install/setup.bash
ros2 launch gamantaray_boat_sim simple_ocean_gamantaray.launch.py navigation_mode:=ardupilot
```

Cek topic penting:

```bash
ros2 topic hz /asv/lidar/scan
ros2 topic hz /local_costmap/costmap
ros2 topic echo /asv/navigation/status
```

Jika local costmap tidak terlihat di RViz, cek apakah topic
`/local_costmap/costmap` publish. Jika topic ada tetapi obstacle tidak muncul,
cek `/asv/lidar/scan` dan posisi LiDAR di model kapal.
