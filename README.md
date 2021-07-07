## 0 新电脑

### dependencies

```
git clone https://github.com/leethomason/tinyxml2.git
cd tinyxml2 && sudo make install
```

* 安装雷达相关驱动
```
git clone https://github.com/Livox-SDK/Livox-SDK.git
cd Livox-SDK
sudo apt install libapr1-dev
cd build && cmake ..
make
sudo make install
```
* 编译
cd logistics_ws/
rosdep install --from-paths src --ignore-src -r -y
sudo apt-get install libpcap-dev
sudo ln -s /usr/include/eigen3/Eigen /usr/include/Eigen
catkin_make
```
