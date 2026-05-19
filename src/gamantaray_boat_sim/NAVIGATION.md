# Navigasi ASV TEKNOFEST

Dokumen ini menjelaskan alur navigasi di package `gamantaray_boat_sim` dan
bagian kode yang dipakai untuk mengatur waypoint, obstacle avoidance, path
Nav2, sensor, dan gerak thruster.

## Alur Sistem

Alur utama navigasi:

```text
tecnofest_waypoints.yaml
        -> nav2_waypoint_mission.py
        -> Nav2 goal GN1-GN5
        -> planner membuat /plan
        -> controller membuat /cmd_vel
        -> cmd_vel_to_thrusters.py
        -> Gazebo Thruster plugin
        -> kapal bergerak
```

Data posisi kapal:

```text
Gazebo /asv/odom
        -> odom_tf_broadcaster.py
        -> TF odom -> base_link
        -> Nav2 tahu posisi kapal
```

Obstacle avoidance:

```text
/asv/lidar/scan
        -> local_costmap dan global_costmap
        -> obstacle layer menandai buoy/halangan
        -> planner/controller memilih jalur yang tidak menabrak
```

Kamera:

```text
/asv/camera/front/image
        -> target_buoy_detector.py
        -> /asv/perception/target_selection
```

Kamera dipakai untuk pemilihan target Course 3. Kamera tidak langsung dipakai
Nav2 standar untuk obstacle avoidance. Obstacle avoidance utama tetap LiDAR.

GPS dan IMU:

```text
/asv/gps/fix
/asv/imu/data
```

GPS dan IMU tersedia sebagai topic sensor, tetapi pose utama Nav2 saat ini
tetap memakai `/asv/odom` dari Gazebo. Ini cukup untuk simulasi karena odom
Gazebo stabil dan sudah sesuai world.

## File Yang Sering Diatur

Waypoint misi:

```text
src/gamantaray_boat_sim/config/tecnofest_waypoints.yaml
```

Atur `startpoint` dan urutan goal GN1-GN5 di sini. `startpoint` adalah posisi
awal/spawn kapal, sedangkan daftar `waypoints` adalah goal yang dikirim ke
Nav2.

World, buoy, marker, dan spawn kapal:

```text
src/gamantaray_boat_sim/worlds/tecnofest_asv_course.sdf
```

Atur marker `Startpoint`, marker GN, buoy Parkur 1/2/3, obstacle buoy, target
buoy, dan pose awal `gamantaray_boat` di sini. Jika posisi waypoint GN di YAML
diubah, marker GN di world sebaiknya ikut disamakan.

Parameter Nav2:

```text
src/gamantaray_boat_sim/config/nav2_params.yaml
```

Atur planner, controller, costmap, inflation, kecepatan, dan collision monitor
di file ini.

Sensor kapal:

```text
src/gamantaray_boat_sim/models/gamantaray_boat/model.sdf
```

Atur update rate, resolusi, dan parameter LiDAR/kamera/GPS/IMU di sini.

Visualisasi RViz:

```text
src/gamantaray_boat_sim/config/tecnofest_nav2.rviz
```

RViz menampilkan `/plan`, `/plan_smoothed`, costmap, LiDAR, odom, dan TF.
Model kapal di RViz berasal dari topic `/asv/visualization/boat_model` yang
dipublish oleh `rviz_boat_marker.py`. Secara default marker ini memakai proxy
ringan, bukan mesh `.obj` asli, supaya RViz tidak freeze.

Objek hasil pembacaan LiDAR divisualisasikan oleh `lidar_obstacle_marker.py`
di topic `/asv/perception/lidar_obstacles`. Node ini mengelompokkan titik
LaserScan menjadi marker silinder kuning agar buoy/obstacle yang terbaca LiDAR
mudah dilihat di RViz.

Launch utama:

```text
src/gamantaray_boat_sim/launch/simple_ocean_gamantaray.launch.py
```

Launch ini menjalankan Gazebo, bridge ROS-Gazebo, TF sensor, Nav2, waypoint
mission, target detector, thruster mapper, dan RViz.

## Cara Mengatur Waypoint

Edit:

```yaml
startpoint: {name: Startpoint, x: -49.0, y: -8.0, yaw: 0.42}
waypoints:
  - {name: gn1_start, x: -40.0, y: -4.0}
  - {name: gn2_peak, x: -32.0, y: 4.5}
  - {name: gn3_valley, x: -24.0, y: -4.0}
  - {name: gn4_parkur2_entry, x: -16.0, y: 4.5}
  - {name: gn5_parkur2_exit, x: 36.5, y: -1.2}
```

Aturan praktis:

- `startpoint` bukan goal Nav2. Kapal spawn di titik ini.
- Goal pertama Nav2 adalah waypoint pertama, saat ini `gn1_start`.
- Gunakan satuan meter.
- Kalau mengubah GN1-GN4 di YAML, samakan juga marker visual di
  `tecnofest_asv_course.sdf`.

## Cara Mengatur Buoy Parkur 1

Di `tecnofest_asv_course.sdf`, buoy Parkur 1 memakai pola:

```text
p1_s1_upper_01 <-> p1_s1_lower_01
p1_s1_upper_02 <-> p1_s1_lower_02
p1_s1_upper_03 <-> p1_s1_lower_03
```

`s1`, `s2`, `s3` adalah segmen zig-zag:

- `s1`: GN1 -> GN2
- `s2`: GN2 -> GN3
- `s3`: GN3 -> GN4

Jarak 8-12 m yang dicek adalah jarak pasangan `upper/lower` dengan nomor sama,
bukan jarak antar buoy berurutan di satu sisi. Contoh yang benar:

```text
p1_s1_upper_01 ke p1_s1_lower_01
```

Jika ingin lintasan lebih lebar, geser pasangan `upper/lower` menjauh dari
centerline. Jika ingin lebih sempit, dekatkan pasangan itu. Tetap jaga jarak
pasangan di rentang 8-12 m.

## Cara Mengatur Obstacle Avoidance

Obstacle avoidance diatur terutama di `nav2_params.yaml`.

Bagian LiDAR masuk costmap:

```yaml
obstacle_layer:
  observation_sources: scan
  scan:
    topic: /asv/lidar/scan
    obstacle_max_range: 12.0
    raytrace_max_range: 20.0
```

Parameter penting:

- `obstacle_max_range`: jarak maksimum obstacle dari LiDAR yang dimasukkan ke
  costmap.
- `raytrace_max_range`: jarak clearing ray untuk membersihkan obstacle lama.
- `inflation_radius`: jarak aman tambahan di sekitar obstacle.
- `cost_scaling_factor`: seberapa cepat cost turun saat menjauh dari obstacle.
- `robot_radius`: radius aman kapal untuk costmap.

Jika kapal terlalu dekat dengan buoy:

- Naikkan `inflation_radius`.
- Naikkan `robot_radius` sedikit.
- Turunkan `desired_linear_vel`.

Jika kapal terlalu takut dan jalur terlalu jauh:

- Turunkan `inflation_radius`.
- Turunkan `robot_radius` sedikit.
- Naikkan `obstacle_max_range` hanya jika LiDAR perlu melihat obstacle lebih
  jauh.

## Cara Mengatur Gerak Kapal

Kecepatan yang diminta Nav2 diatur di `nav2_params.yaml`.

Bagian controller:

```yaml
FollowPath:
  desired_linear_vel: 0.55
  lookahead_dist: 1.4
```

Bagian velocity smoother:

```yaml
max_velocity: [0.60, 0.0, 0.85]
max_accel: [0.45, 0.0, 1.00]
```

Mapping `/cmd_vel` ke thruster ada di `cmd_vel_to_thrusters.py`.

Parameter default penting:

```text
max_forward_thrust_n = 100.0
max_reverse_thrust_n = 55.0
max_speed_cmd_mps = 0.65
yaw_to_thrust_n_per_radps = 100.0
max_yaw_rate_cmd_radps = 0.80
```

Jika kapal terlalu lambat:

- Naikkan `desired_linear_vel` sedikit.
- Naikkan `max_velocity[0]`.
- Jika masih kurang, naikkan `max_forward_thrust_n`.

Jika kapal zig-zag atau belok terlalu agresif:

- Turunkan `max_velocity[2]`.
- Turunkan `yaw_to_thrust_n_per_radps`.
- Naikkan `lookahead_dist` supaya tracking path lebih halus.

## Visualisasi Path

RViz dibuka otomatis oleh launch utama. Jalankan:

```bash
ros2 launch gamantaray_boat_sim simple_ocean_gamantaray.launch.py
```

Yang perlu dilihat di RViz:

- `/plan`: path global Nav2.
- `/plan_smoothed`: path smoothing jika topic tersedia.
- `/global_costmap/costmap`: costmap global.
- `/local_costmap/costmap`: costmap lokal di sekitar kapal.
- `/asv/lidar/scan`: titik LiDAR untuk obstacle.
- `/asv/perception/lidar_obstacles`: marker kuning obstacle hasil clustering LiDAR.
- `/asv/odom`: jejak odometry kapal.
- `/asv/visualization/boat_model`: proxy visual kapal yang ringan.

Catatan penting: mesh asli kapal `assembly_2_0.obj` berukuran sangat besar
karena berasal dari file `assembly_2_0.obj.gz`. Gazebo tetap memakai mesh asli
itu melalui `models/gamantaray_boat/model.sdf`, tetapi RViz tidak memakai mesh
tersebut secara default. Jika mesh asli dipaksa masuk RViz, RViz bisa lama
loading atau terlihat `not responding`.

Jika ingin Gazebo tetap jalan tetapi RViz dimatikan:

```bash
ros2 launch gamantaray_boat_sim simple_ocean_gamantaray.launch.py use_rviz:=false
```

## Urutan Debug Navigasi

1. Cek apakah sumber pose dan sensor hidup:

```bash
ros2 topic hz /asv/odom
ros2 topic hz /asv/lidar/scan
ros2 run tf2_ros tf2_echo odom base_link
```

Jika TF `odom -> base_link` belum muncul, RViz akan menampilkan error `No
transform to fixed frame [odom]` dan Nav2 tidak bisa menghitung arah kapal
dengan benar.

2. Cek pose kapal:

```bash
ros2 topic echo --once /asv/odom
```

3. Cek LiDAR:

```bash
ros2 topic echo --once /asv/lidar/scan
```

4. Cek status waypoint:

```bash
ros2 topic echo /asv/navigation/status
```

5. Cek path Nav2:

```bash
ros2 topic echo --once /plan
```

6. Cek output gerak:

```bash
ros2 topic echo /cmd_vel
```

7. Cek output thruster:

```bash
ros2 topic echo /asv/control/thruster_status
```

8. Cek arah belok thruster:

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}, angular: {z: 0.5}}" -r 10
```

Untuk konvensi ROS, `angular.z` positif harus membuat heading kapal berputar
positif di frame `odom`. Jika arah visual di Gazebo/RViz terbalik, ubah
parameter `yaw_sign` pada `cmd_vel_to_thrusters` menjadi `-1.0`.

## Catatan Desain

Untuk simulasi ini, Nav2 berjalan mapless di frame `odom`. Tidak ada AMCL dan
tidak ada map statis. Ini sengaja dipilih karena arena berupa lintasan air
dengan buoy, dan obstacle avoidance utama berasal dari LiDAR/costmap.

GPS dan IMU tetap tersedia sebagai sensor, tetapi belum difusion sebagai pose
utama. Untuk simulasi Gazebo, `/asv/odom` sudah cukup stabil. Fusion
GPS+IMU+odom bisa ditambahkan nanti jika targetnya simulasi yang lebih mirip
robot asli.
