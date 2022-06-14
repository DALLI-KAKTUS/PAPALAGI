# **Proje için ROS2 Workspace hazırlanması**
## ROS2 kurulumu
### fastdds kurulumu
foonathan_memory_vendor'u indir ve kaynaktan derle
```bash
git clone https://github.com/eProsima/foonathan_memory_vendor.git
cd foonathan_memory_vendor
mkdir build && cd build
cmake ..
cmake --build . --target install
```
fastdds'i indir ve derle
```bash
#fastDDS sürümü 2.0.0 olmalı                                                                                                  +++++++++
git clone --recursive https://github.com/eProsima/Fast-DDS.git ~/FastDDS-2.0.0
cd ~/FastDDS-2.0.0
mkdir build && cd build
cmake -DTHIRDPARTY=ON -DSECURITY=ON ..
make -j$(nproc --all)
sudo make install
```
