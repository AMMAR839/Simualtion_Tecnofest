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
`tile_size` `100 20`, sehingga dynamic wave mesh dari plugin Gazebo juga
dibatasi ke lebar lintasan 20 m, bukan tile default 100 m x 100 m.

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
ros2 launch gamantaray_boat_sim simple_ocean_gamantaray.launch.py use_nav2:=false
```

Waypoint default ada di:

```text
src/gamantaray_boat_sim/config/tecnofest_waypoints.yaml
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
- `/asv/lidar/scan`
- `/asv/camera/front/image`
- `/asv/camera/front/camera_info`
- `/asv/perception/target_selection`
- `/asv/navigation/status`

Frame sensor dari Gazebo memakai nama aktual:

- LiDAR: `gamantaray_boat/base_link/lidar_sensor`
- Kamera: `gamantaray_boat/base_link/front_camera_sensor`

Launch juga menerbitkan alias TF `lidar_link` dan `front_camera_link` dari
`base_link` untuk kompatibilitas.

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
