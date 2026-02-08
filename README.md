# the_automatic_navigation_system_based_on_fast_livo2_and_cmu_automatic
基于fast_livo2与cmu_automatic的自动导航系统，使用了mid360与d435i作为传感器，在nx上完成了测试


不使用gazebo的分支，理论上可以直接部署在机器上，使用ros2版本为humbel<br>
部署流程 （ht_ws文件夹）<br>
1. LIVOX-SDK(不要放在包下面哦ovo)<br>
    git clone https://github.com/Livox-SDK/Livox-SDK2.git
    
    cd ./Livox-SDK2/
    
    mkdir build
    
    cd build
    
    cmake .. && make -j
    
    sudo make install
2. livox_ros_driver2（这个东东记得编译的时候拿出来不然会报错，单独编译一次即可）<br>
  （去ws_livox下面启动哦，git的时候路径不要在这个下面）<br>
  git clone https://github.com/Livox-SDK/livox_ros_driver2.git ws_livox/src/livox_ros_driver2
  
  cd ws_livox/src/livox_ros_driver2
  
  ./build.sh humble
  
  (ps:我里面有了，如果用我里面的可以看下面的教程，不用的可以脱出来删掉)
  
  cd ./src/livox_ros_driver2
  
  ./build.sh humble
  
   2.1 修改/src/livox_ros_driver2/config/mid360配置文件（ps:我用的是mid360,其他的可能要做适配）<br>   
       可以看看参考配置：<br>
         {
      "lidar_summary_info" : {
        "lidar_type": 8
      },
      "MID360": {
        "lidar_net_info" : {
          "cmd_data_port": 56100,
          "push_msg_port": 56200,
          "point_data_port": 56300,
          "imu_data_port": 56400,
          "log_data_port": 56500
        },
        "host_net_info" : {
          "cmd_data_ip" : "192.168.1.51",
          "cmd_data_port": 56101,
          "push_msg_ip": "192.168.1.51",
          "push_msg_port": 56201,
          "point_data_ip": "192.168.1.51",
          "point_data_port": 56301,
          "imu_data_ip" : "192.168.1.51",
          "imu_data_port": 56401,
          "log_data_ip" : "",
          "log_data_port": 56501
        }
      },
          "lidar_configs" :[ 
          {
              "ip" : "192.168.1.192",
              "pcl_data_type" : 1,
              "pattern_mode" : 0,
              "extrinsic_parameter" : {
                "roll": 0.0,
                "pitch": 0.0,
                "yaw": 0.0,
                "x": 0,
                "y": 0,
                "z": 0
              }
            }
          ]
        }
      2.2 修改动态链接库路径<br>
        错误信息： [livox_ros_driver2_node-1] 抛出 "class_loader::LibraryLoadException" 异常 [livox_ros_driver2_node-1] 具体错误：无法加载库 dlopen 错误：liblivox_lidar_sdk_shared.so - 无法打开共享对象文件（文件不存在），位于 ./src/shared_library.c:99<br>
        # 找到 Livox SDK 安装位置
        
        find ~ -name "liblivox_lidar_sdk_shared.so" 2>/dev/null
        
        请确保将 SDK 文件 liblivox_lidar_sdk_shared.so 手动复制到 ~/install/livox_ros_driver2/lib 目录下，否则运行时会报错

3. D435i安装<br>
      3.1 安装RealSense SDK<br>
         
         sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE
         
         sudo add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" -u
         
         sudo apt-get install librealsense2-dkms librealsense2-utils librealsense2-dev librealsense2-dbg
      3.2 测试一下<br>
         
         realsense-viewer
      
      3.3 安装一下依赖<br>
         
         rosdep install -i --from-path src --rosdistro humble -y
      
      3.4 单独编译一下<br>
         
          colcon build --packages-select realsense2_camera  --symlink-instal
         
      
4. 编译安装第三方库 Sophus<br>
  
  git clone https://github.com/strasdat/Sophus.git
  
  cd Sophus
  
  git checkout a621ff
   4.1 修改源码错误（好像是兼容性问题）<br>
      需要修改的代码文件是sophus/so2.cpp<br>
      将32和33行的代码，修改为如下所示<br>
      unit_complex_.real(1.);<br>
      unit_complex_.imag(0.);<br>
  
  mkdir build && cd build
  
  cmake ..
  
  make 
  
  sudo make install

4.到工作空间下编译一下就好了<br>
  
  colcon build
  
  source install/setup.bash
  
5.启动，启动，全部启动！！！<br>
  ros2 launch livox_ros_driver2 msg_MID360_launch.py //mid360启动 
  
  ros2 launch realsense2_camera rs_launch.py //d435i启动
  
  ros2 launch fast_livo mapping_avia.launch.py //fast_livo2启动
  
  （ps：使用rviz2可以看到点云输出了）<br>
6.Autonomous Exploration导航算法(ps:在Autonomous Exploration工作空间下)<br>
  
  sudo apt update
  
  sudo apt install libusb-dev ros-humble-joy
  
  colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
  
  source install/setup.sh
  
7. 开始启动<br>
  
  source install/setup.bash
  
  ros2 launch vehicle_simulator system_real_robot.launch
    


  
  
