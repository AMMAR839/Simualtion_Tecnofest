# Gamantaray TEKNOFEST ASV Simulation

Package ini menjalankan simulasi ASV Gamantaray untuk course TEKNOFEST di ROS 2
Jazzy dan Gazebo Harmonic. World utama memakai lintasan air 100 m x 20 m dengan
model wave lokal dari `tecnofest_ocean_waves`, buoy course, LiDAR, kamera, GPS,
IMU, dan dua thruster Gazebo yang menerima command thrust.

Simulasi ini dibuat untuk stabilitas misi waypoint dan obstacle avoidance.
Thruster memakai `gz::sim::systems::Thruster` sehingga kapal benar-benar
bergerak dari gaya dorong di Gazebo. Wave visual dan damping hidrodinamika
dipakai sebagai pendekatan simulasi misi yang stabil, bukan klaim model
hidrodinamika penuh.
Kapal juga memakai buoyancy volume yang diproses oleh
`gz-waves1-hydrodynamics-system`, ditambah anti-sink guard ringan sebagai
pengaman jika solver fisika sempat membuat kapal masuk di bawah mesh ombak.

World utama hanya menampilkan elemen lintasan misi: air, ASV, buoy batas,
obstacle buoy, dan target buoy. Garis bantu start/finish dan frame area buatan
dihapus agar tampilan mengikuti lintasan course. Layout buoy mengikuti gambar
referensi: kapal spawn di marker hitam `Startpoint` sekitar 9.8 m sebelum GN1,
lalu Parkur 1 memakai dua sisi batas zig-zag pada centerline GN1-GN2-GN3-GN4
dengan lebar koridor sekitar 11.6 m. Parkur 2 berupa koridor panjang dengan
obstacle kuning di tengah, dan
Parkur 3 target vertikal merah, hijau, hitam. Marker GN1-GN5 berwarna biru
hanya visual dan tidak memiliki collision. Jarak 8-12 m adalah jarak buoy
berpasangan pada dua sisi berlawanan, bukan jarak antar buoy berurutan di satu
sisi lintasan.

Warna air berasal dari shader lokal
`models/tecnofest_ocean_waves/materials/waves_fs.glsl`. Refleksi skybox pada
shader dikurangi dan fallback material biru ditambahkan di `model.sdf` supaya
water surface tidak tampil putih ketika dilihat di Gazebo. Model wave memakai
`tile_size` `104 26`, mengikuti area course yang dibuat mepet dengan objek
lintasan. Parameter wave sengaja dibuat halus (`cell_count` 96, update 18 Hz)
agar physics kapal, costmap, dan RViz tidak terlalu berat.

## Build

```bash
cd /home/ammar/Documents/ws_tecnofest
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## Launch Utama

Jalankan simulasi lengkap dengan Nav2 dan misi 5 waypoint:

```bash
ros2 launch gamantaray_boat_sim simple_ocean_gamantaray.launch.py
```

Jalankan hanya Gazebo, bridge, sensor, dan kontrol thruster untuk tes manual:

```bash
ros2 launch gamantaray_boat_sim simple_ocean_gamantaray.launch.py navigation_mode:=manual
```

Pilih mode navigasi dari launch utama:

```bash
ros2 launch gamantaray_boat_sim simple_ocean_gamantaray.launch.py navigation_mode:=nav2
ros2 launch gamantaray_boat_sim simple_ocean_gamantaray.launch.py navigation_mode:=manual
ros2 launch gamantaray_boat_sim simple_ocean_gamantaray.launch.py navigation_mode:=ardupilot
```

`nav2` memakai Nav2 dan `/cmd_vel_to_thrusters`. `manual` hanya menjalankan
Gazebo, bridge, sensor, dan mapper `/cmd_vel`. `ardupilot` menjalankan
ArduRover SITL + MAVROS + bridge LiDAR ke MAVLink proximity; Nav2 tidak
dijalankan pada mode ini.
Pada mode `ardupilot`, launch membuka terminal GUI terpisah untuk proses
ArduRover SITL. Jika terminal GUI tidak tersedia, output SITL akan fallback ke
terminal launch utama.

RViz akan terbuka otomatis untuk menampilkan `/plan`, local costmap, TF, odometry,
LiDAR, marker obstacle LiDAR, dan marker model kapal. Marker kapal di RViz
memakai proxy ringan agar RViz tetap responsif; mesh `.obj` asli kapal tetap
ditampilkan di Gazebo. View RViz memakai target frame `base_link`, jadi kamera
RViz mengikuti pergerakan kapal. Launch juga memakai GUI config Gazebo khusus
agar panel/tab Gazebo disembunyikan dan jendela Gazebo/RViz dibuka dengan
layout seperti referensi: Gazebo di kiri mulai setelah dock Ubuntu, RViz di
kanan. Pada Wayland, window manager masih bisa mengabaikan posisi absolut
window, tetapi ukuran dan layout RViz/Gazebo sudah disiapkan dari config
package.

Jika jendela Gazebo ditutup, launch akan ikut mematikan bridge, Nav2, dan node
ROS lain. Ini sengaja dibuat supaya Nav2 tidak berjalan tanpa `/clock`,
`/asv/odom`, dan `/asv/lidar/scan`. Node misi waypoint juga menunggu `/clock`,
odom, LiDAR, dan TF `odom -> base_link` aktif sebelum mengirim goal Nav2.

Untuk mengejar RTF lebih ringan tanpa visualisasi path, jalankan:

```bash
ros2 launch gamantaray_boat_sim simple_ocean_gamantaray.launch.py use_rviz:=false
```

RViz bisa dipakai sebagai tampilan utama navigasi. Gazebo tetap diperlukan
sebagai simulator fisika kapal, thruster, wave, buoy, LiDAR, kamera, dan odom;
RViz hanya viewer untuk path dan data ROS. Jadi mode kerja yang disarankan
untuk debugging navigasi adalah membuka RViz dan memakai Gazebo hanya sebagai
simulator di belakang. Jika ingin paling ringan, tutup/minimize tampilan Gazebo
dan fokus pada RViz. Opsi benar-benar headless dapat ditambahkan kemudian agar
Gazebo server berjalan tanpa GUI.

Waypoint default ada di:

```text
src/gamantaray_boat_sim/config/tecnofest_waypoints.yaml
```

Panduan lengkap untuk mengatur waypoint, Nav2, costmap, LiDAR, path RViz, dan
thruster ada di:

```text
src/gamantaray_boat_sim/NAVIGATION.md
```

## Kontrol Manual

Kirim command velocity ke `/cmd_vel`. Node `cmd_vel_to_thrusters` akan
mengubahnya menjadi command thrust kiri/kanan.

Maju:

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 1.0}, angular: {z: 0.0}}" -r 10
```

Belok kiri:

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}, angular: {z: 0.8}}" -r 10
```

Jika hasil tes manual menunjukkan `angular.z` positif justru membuat kapal
belok ke arah berlawanan, ubah parameter `yaw_sign` pada node
`cmd_vel_to_thrusters` dari `1.0` ke `-1.0` di launch file.

Stop:

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.0}}" -1
```

## Topic Utama

Kontrol:

- `/cmd_vel`
- `/model/gamantaray_boat/joint/left_propeller_joint/cmd_thrust`
- `/model/gamantaray_boat/joint/right_propeller_joint/cmd_thrust`
- `/asv/control/thruster_status`

Sensor dan navigasi:

- `/asv/odom`
- `/asv/imu/data`
- `/asv/gps/fix`
- `/asv/lidar/scan_raw`
- `/asv/lidar/scan`
- `/asv/visualization/lidar_rays`
- `/asv/camera/front/image`
- `/asv/camera/front/camera_info`
- `/asv/perception/target_selection`
- `/asv/navigation/status`
- `/plan`
- `/global_costmap/costmap`
- `/local_costmap/costmap`
- `/asv/visualization/boat_model`
- `/asv/perception/lidar_obstacles`

Frame sensor dari Gazebo memakai nama aktual:

- LiDAR: `gamantaray_boat/base_link/lidar_sensor`
- Kamera: `gamantaray_boat/base_link/front_camera_sensor`

Launch juga menerbitkan alias TF `lidar_link` dan `front_camera_link` dari
`base_link` untuk kompatibilitas.

`/asv/lidar/scan_raw` adalah output langsung Gazebo. `/asv/lidar/scan` adalah
hasil filter footprint kapal sendiri dan dipakai oleh Nav2, RViz obstacle
marker, serta bridge ArduPilot. Path biru `/plan` adalah referensi global
menuju waypoint; obstacle avoidance buoy dilakukan oleh local costmap dan DWB
controller dari data LiDAR 2D.

## Mode ArduPilot

Mode ArduPilot memakai ArduRover SITL dari:

```text
/home/ammar/ardu_ws/src/ardupilot/Tools/autotest/sim_vehicle.py
```

Plugin Gazebo ArduPilot diambil dari:

```text
/home/ammar/ardupilot_gazebo/build/libArduPilotPlugin.so
```

Parameter awal ASV ada di:

```text
src/gamantaray_boat_sim/config/ardupilot_asv.parm
```

Mission Planner adalah ground control station. Setelah launch
`navigation_mode:=ardupilot`, hubungkan Mission Planner ke MAVLink UDP SITL
yang keluar dari ArduPilot. Konfigurasi obstacle avoidance memakai
`PRX1_TYPE=2`, `AVOID_ENABLE=7`, dan `OA_TYPE=3` untuk proximity MAVLink dan
Dijkstra+BendyRuler. Ini adalah jalur awal integrasi; tuning waypoint AUTO /
GUIDED dan validasi proximity di Mission Planner tetap perlu dilakukan saat
SITL sudah connect.

## Validasi Cepat

Tes sensor:

```bash
ros2 topic echo --once /asv/odom nav_msgs/msg/Odometry
ros2 topic echo --once /asv/lidar/scan sensor_msgs/msg/LaserScan
ros2 topic echo --once /asv/camera/front/camera_info sensor_msgs/msg/CameraInfo
```

Tes gerak:

```bash
timeout 4s ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 1.0}, angular: {z: 0.0}}"
ros2 topic echo --once /asv/odom nav_msgs/msg/Odometry
```

Tes Nav2:

```bash
ros2 topic echo /asv/navigation/status std_msgs/msg/String
```

Status yang diharapkan saat misi mulai:

```text
nav2_waypoints_started:5
```

Kapal mulai dari `Startpoint` dan goal Nav2 pertama tetap GN1, sehingga urutan
misi adalah Startpoint -> GN1 -> GN2 -> GN3 -> GN4 -> GN5.

## Setting Parkur 1

Atur posisi GN dan goal Nav2 di `src/gamantaray_boat_sim/config/tecnofest_waypoints.yaml`.
Atur marker visual GN, Startpoint, spawn kapal, dan buoy Parkur 1 di
`src/gamantaray_boat_sim/worlds/tecnofest_asv_course.sdf`. Untuk Parkur 1,
pasangan yang dicek jaraknya adalah nama dengan indeks sama, misalnya
`p1_s1_upper_01` dengan `p1_s1_lower_01`.
