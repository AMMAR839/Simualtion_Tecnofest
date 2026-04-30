# Gamantaray Boat Simulation

Workspace ini membuat `Assembly 2.0.obj` menjadi model kapal Gazebo di world laut sederhana.

## setup

```bash
git clone https://github.com/gamantaray-2026/kapal_agung_ws.git gamataray_boat_ws
cd kapal_agung_ws
```

File mesh kapal besar disimpan sebagai `assembly_2_0.obj.gz` supaya bisa masuk GitHub.
Saat `colcon build`, file `assembly_2_0.obj` akan dibuat otomatis kalau belum ada.

## Build

```bash
colcon build --symlink-install
source install/setup.bash
```

## Upload ke GitHub

```bash
cd /home/ammar/gamantaray_boat_ws
git status
git push -u origin main
```

Kalau GitHub meminta login, gunakan Personal Access Token sebagai password.

## Launch

```bash
ros2 launch gamantaray_boat_sim simple_ocean_gamantaray.launch.py
```

## Jalankan Kapal

Kirim perintah maju dan belok lewat `/cmd_vel`:

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 1.0}, angular: {z: 0.0}}" -r 10
```

Belok kiri:

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}, angular: {z: 1.0}}" -r 10
```

Stop:

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.0}}" -1
```

Sensor ROS:

- Kamera: `/world/simple_ocean/model/gamantaray_boat/link/base_link/sensor/front_camera/image`
- Camera info: `/world/simple_ocean/model/gamantaray_boat/link/base_link/sensor/front_camera/camera_info`
- Lidar: `/world/simple_ocean/model/gamantaray_boat/link/base_link/sensor/front_lidar/scan`
