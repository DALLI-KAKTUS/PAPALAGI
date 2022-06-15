# **Proje için ROS2 Workspace hazırlanması**
## ROS2 kurulumu
### fastdds kurulumu
foonathan_memory_vendor'u indir ve kaynaktan derle.
```bash
git clone https://github.com/eProsima/foonathan_memory_vendor.git
cd foonathan_memory_vendor
mkdir build && cd build
cmake ..
cmake --build . --target install
```
fastdds'i indir ve derle.
```bash
#fastDDS sürümü 2.0.0 olmalı                                                                                                  +++++++++
git clone --recursive https://github.com/eProsima/Fast-DDS.git ~/FastDDS-2.0.0
cd ~/FastDDS-2.0.0
mkdir build && cd build
cmake -DTHIRDPARTY=ON -DSECURITY=ON ..
make -j$(nproc --all)
sudo make install
```
#### sdk ve *gradle 6.0.0* kur
# **Proje için ROS2 Workspace hazırlanması**
## ROS2 kurulumu
### fastdds kurulumu
foonathan_memory_vendor'u indir ve kaynaktan derle.
```bash
git clone https://github.com/eProsima/foonathan_memory_vendor.git
cd foonathan_memory_vendor
mkdir build && cd build
cmake ..
cmake --build . --target install
```
fastdds'i indir ve derle.
```bash
#fastDDS sürümü 2.0.0 olmalı                                                                                                  +++++++++
git clone --recursive https://github.com/eProsima/Fast-DDS.git ~/FastDDS-2.0.0
cd ~/FastDDS-2.0.0
mkdir build && cd build
cmake -DTHIRDPARTY=ON -DSECURITY=ON ..
make -j$(nproc --all)
sudo make install
```
#### jdk8 ve *gradle 6.0.0* kurduktan sonra fastrtps-gen kurmak için
```bash
git clone --recursive https://github.com/eProsima/Fast-DDS-Gen.git -b v1.0.4 ~/Fast-RTPS-Gen \
cd ~/Fast-RTPS-Gen \
./gradlew assemble \
sudo ./gradlew install
```
## darknet_ros kurulumu
```bash
source /opt/ros/foxy/setup.bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone --recursive  https://github.com/Ar-Ray-code/darknet_ros_yolov4.git
darknet_ros_yolov4/darknet_ros/rm_darknet_CMakeLists.sh
cd ~/ros2_ws
colcon build --symlink-install
```
### driver sürümü değiştrme
 `/home/jetson/kkts_ws/src/darknet_ros_yolov4/darknet_ros/darknet_ros/CMakeLists.txt` 
 
 içindeki 
 ```
    -gencode arch=compute_50,code=[sm_50,compute_50]
    -gencode arch=compute_52,code=[sm_52,compute_52]
    -gencode arch=compute_61,code=[sm_61,compute_61]
    -gencode arch=compute_72,code=[sm_72,compute_72] # Jetson AGX Xavier or TITAN V
    -gencode arch=compute_75,code=[sm_75,compute_75]
    -gencode arch=compute_80,code=[sm_80,compute_80]
    -gencode arch=compute_86,code=[sm_86,compute_86] # GeForce RTX 3000 Series
    -gencode arch=compute_87,code=[sm_87,compute_87]
```
satırlarını
   `-gencode arch=compute_53,code=[sm_53,compute_53]` ile değiştir.
   
    `apt-get install ros-foxy-v4l2-camera`
ile v4l2 ros driverını kur.

### kendi datasetimizi kullanma
`/home/jetson/kkts_ws/src/darknet_ros_yolov4/darknet_ros/darknet_ros/yolo_network_config` yolundaki weights ve cfg dosyalarını değiştir ve tüm workspace'deki data cart curt dosyalarını kendi isteğine göre ayarla.
## px4_ros_com paketini kendi workspace'imize uyarlama
```bash
cd <workspace yolu>/src
git clone https://github.com/PX4/px4_ros_com.git 
git clone https://github.com/PX4/px4_msgs.git
source ~/kkts_ws/src/px4_ros_com/scripts/build_ros2_workspace.bash
```
### setpoint kodunun eklenmesi
[setpoint.cpp](https://github.com/DALLI-KAKTUS/PAPALAGI/blob/main/Software/setpoint.cpp "setpoint kodu") dosyasını indirip `/home/jetson/kkts_ws/src/px4_ros_com/src/examples/offboard` klasörüne at. ardından `/home/jetson/kkts_ws/src/px4_ros_com/CMakeLists.txt` dosyasındaki ` custom_executable(examples/offboard offboard_control)`  satırının altına ` custom_executable(examples/offboard setpoint)`  satırını ekle

###spiral çizme fonksiyonu
[spiral.cpp ](hhttps://github.com/DALLI-KAKTUS/PAPALAGI/blob/main/Software/spiral.cppttp:// "spiral.cpp ") dosyası spiral çizdirmek için gerekli koordinatları cpp vector sözdizimine uygun bi şekilde consola yazdırır, bu çıktı [setpoint.cpp](https://github.com/DALLI-KAKTUS/PAPALAGI/blob/main/Software/setpoint.cpp "setpoint kodu") içindeki waypoints değişkenine atanmalıdır.
#### spiralin tune edilmesi
#####Tune için gerekli değişkenler ve açıklamaları
**A**: yataydaki başlangıç noktası
**B**: dikeydeki başlangıç noktası
**angle**: iki veri arasında ilerlenen açı
**l**: vektörün uzunluğu, toplam nokta sayısı
### launch dosyasını uyarlama
`/home/jetson/kkts_ws/src/darknet_ros_yolov4/darknet_ros/darknet_ros/launch/darknet_ros.launch.py` dosyasındaki
