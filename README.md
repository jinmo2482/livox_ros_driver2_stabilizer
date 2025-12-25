# Livox ROS Driver 2

Livox ROS Driver 2 是用于连接 Livox 旗下激光雷达产品的第二代驱动包，适用于 ROS（推荐 Noetic）和 ROS2（推荐 Foxy 或 Humble）。

**注意：**

作为调试工具，Livox ROS Driver 不建议用于量产场景，仅用于测试用途。你应基于原始代码进行优化以满足实际需求。

## 1. 准备工作

### 1.1 操作系统要求

- Ubuntu 18.04（ROS Melodic）
- Ubuntu 20.04（ROS Noetic / ROS2 Foxy）
- Ubuntu 22.04（ROS2 Humble）

**提示：**

Colcon 是 ROS2 的构建工具。

安装 colcon：<https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Colcon-Tutorial.html>

### 1.2 安装 ROS & ROS2

- ROS Melodic 安装：<https://wiki.ros.org/melodic/Installation>
- ROS Noetic 安装：<https://wiki.ros.org/noetic/Installation>
- ROS2 Foxy 安装：<https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html>
- ROS2 Humble 安装：<https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html>

建议选择 Desktop-Full 安装方式。

## 2. 编译与运行 Livox ROS Driver 2

### 2.1 克隆源码

```shell
git clone https://github.com/Livox-SDK/livox_ros_driver2.git ws_livox/src/livox_ros_driver2
```

**注意：**

请务必将源码克隆到 `[work_space]/src/` 目录下（如上所示），否则会因为构建工具的限制导致编译错误。

### 2.2 编译并安装 Livox-SDK2

**注意：**

请参照 <https://github.com/Livox-SDK/Livox-SDK2/blob/master/README.md> 进行安装。

### 2.3 编译 Livox ROS Driver 2

#### ROS（以 Noetic 为例）

```shell
source /opt/ros/noetic/setup.sh
./build.sh ROS1
```

#### ROS2 Foxy

```shell
source /opt/ros/foxy/setup.sh
./build.sh ROS2
```

#### ROS2 Humble

```shell
source /opt/ros/humble/setup.sh
./build.sh humble
```

### 2.4 运行 Livox ROS Driver 2

#### ROS

```shell
source ../../devel/setup.sh
roslaunch livox_ros_driver2 [launch file]
```

其中：

- **livox_ros_driver2**：ROS 包名
- **[launch file]**：启动文件；`launch_ROS1` 目录下提供了多个示例

HAP 雷达的 rviz 启动示例：

```shell
roslaunch livox_ros_driver2 rviz_HAP.launch
```

#### ROS2

```shell
source ../../install/setup.sh
ros2 launch livox_ros_driver2 [launch file]
```

其中：

- **[launch file]**：ROS2 启动文件；`launch_ROS2` 目录下提供了多个示例

HAP 雷达的 rviz 启动示例：

```shell
ros2 launch livox_ros_driver2 rviz_HAP_launch.py
```

## 3. 启动文件与内部参数说明

### 3.1 启动文件说明

ROS 启动文件位于 `ws_livox/src/livox_ros_driver2/launch_ROS1`，ROS2 启动文件位于 `ws_livox/src/livox_ros_driver2/launch_ROS2`。不同启动文件对应不同场景：

| 启动文件名 | 说明 |
| --- | --- |
| rviz_HAP.launch | 连接 HAP 雷达<br>发布 pointcloud2 数据<br>自动启动 rviz |
| msg_HAP.launch | 连接 HAP 雷达<br>发布 Livox 自定义点云 |
| rviz_MID360.launch | 连接 MID360 雷达<br>发布 pointcloud2 数据<br>自动启动 rviz |
| msg_MID360.launch | 连接 MID360 雷达<br>发布 Livox 自定义点云 |
| rviz_mixed.launch | 连接 HAP 与 MID360<br>发布 pointcloud2 数据<br>自动启动 rviz |
| msg_mixed.launch | 连接 HAP 与 MID360<br>发布 Livox 自定义点云 |

### 3.2 Livox ROS Driver 2 内部主要参数

Livox_ros_driver2 的内部参数全部在启动文件中。以下是常用参数说明：

| 参数 | 说明 | 默认值 |
| --- | --- | --- |
| publish_freq | 点云发布频率，浮点型，推荐 5.0/10.0/20.0/50.0 等，最大 100.0 Hz | 10.0 |
| multi_topic | 是否每台雷达使用独立 topic 发布点云<br>0：所有雷达共享一个 topic<br>1：每台雷达一个 topic | 0 |
| xfer_format | 点云格式<br>0：Livox pointcloud2 (PointXYZRTLT)<br>1：Livox 自定义格式<br>2：PCL PointXYZI 标准格式（仅 ROS） | 0 |

**注意：**

表中未列出的参数不建议修改，除非已充分理解其含义。

**Livox_ros_driver2 点云数据格式说明：**

1. Livox pointcloud2 (PointXYZRTLT)：

```c
float32 x               # X axis, unit:m
float32 y               # Y axis, unit:m
float32 z               # Z axis, unit:m
float32 intensity       # the value is reflectivity, 0.0~255.0
uint8   tag             # livox tag
uint8   line            # laser number in lidar
float64 timestamp       # Timestamp of point
```

**注意：**

每帧点数可能不同，但每个点都带有时间戳。

2. Livox 自定义数据包格式：

```c
std_msgs/Header header     # ROS standard message header
uint64          timebase   # The time of first point
uint32          point_num  # Total number of pointclouds
uint8           lidar_id   # Lidar device id number
uint8[3]        rsvd       # Reserved use
CustomPoint[]   points     # Pointcloud data
```

自定义点云（CustomPoint）格式：

```c
uint32  offset_time     # offset time relative to the base time
float32 x               # X axis, unit:m
float32 y               # Y axis, unit:m
float32 z               # Z axis, unit:m
uint8   reflectivity    # reflectivity, 0~255
uint8   tag             # livox tag
uint8   line            # laser number in lidar
```

3. PCL 标准 pointcloud2 (pcl::PointXYZI)：

请参考 PCL 库中的 point_types.hpp 文件。

## 4. LiDAR 配置

LiDAR 配置（如 ip、端口、数据类型等）通过 json 格式配置文件设置。HAP、MID360 及混合雷达配置文件位于 `config` 目录。启动文件中的 `user_config_path` 指向配置文件。

1. HAP 配置示例（config/HAP_config.json）：

```json
{
  "lidar_summary_info" : {
    "lidar_type": 8  # protocol type index, please don't revise this value
  },
  "HAP": {
    "device_type" : "HAP",
    "lidar_ipaddr": "",
    "lidar_net_info" : {
      "cmd_data_port": 56000,  # command port
      "push_msg_port": 0,
      "point_data_port": 57000,
      "imu_data_port": 58000,
      "log_data_port": 59000
    },
    "host_net_info" : {
      "cmd_data_ip" : "192.168.1.5",  # host ip (it can be revised)
      "cmd_data_port": 56000,
      "push_msg_ip": "",
      "push_msg_port": 0,
      "point_data_ip": "192.168.1.5",  # host ip
      "point_data_port": 57000,
      "imu_data_ip" : "192.168.1.5",  # host ip
      "imu_data_port": 58000,
      "log_data_ip" : "",
      "log_data_port": 59000
    }
  },
  "lidar_configs" : [
    {
      "ip" : "192.168.1.100",  # ip of the LiDAR you want to config
      "pcl_data_type" : 1,
      "pattern_mode" : 0,
      "blind_spot_set" : 50,
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
```

参数说明如下：

**LiDAR 配置参数**

| 参数 | 类型 | 说明 | 默认值 |
| --- | --- | --- | --- |
| ip | String | 需要配置的 LiDAR IP | 192.168.1.100 |
| pcl_data_type | Int | 点云分辨率<br>1：笛卡尔坐标（32 位）<br>2：笛卡尔坐标（16 位）<br>3：球坐标 | 1 |
| pattern_mode | Int | 扫描模式<br>0：非重复扫描<br>1：重复扫描<br>2：低速重复扫描 | 0 |
| blind_spot_set（仅 HAP） | Int | 盲区设置，范围 50~200 cm | 50 |
| extrinsic_parameter |  | 外参设置，roll/pitch/yaw 为 float，x/y/z 为 int |  |

更多 HAP 配置说明：<https://github.com/Livox-SDK/Livox-SDK2/wiki/hap-config-file-description>

2. 多雷达配置时，在 `lidar_configs` 中追加不同雷达配置。混合雷达示例：

```json
{
  "lidar_summary_info" : {
    "lidar_type": 8  # protocol type index, please don't revise this value
  },
  "HAP": {
    "lidar_net_info" : {  # HAP ports, please don't revise these values
      "cmd_data_port": 56000,  # HAP command port
      "push_msg_port": 0,
      "point_data_port": 57000,
      "imu_data_port": 58000,
      "log_data_port": 59000
    },
    "host_net_info" : {
      "cmd_data_ip" : "192.168.1.5",  # host ip
      "cmd_data_port": 56000,
      "push_msg_ip": "",
      "push_msg_port": 0,
      "point_data_ip": "192.168.1.5",  # host ip
      "point_data_port": 57000,
      "imu_data_ip" : "192.168.1.5",  # host ip
      "imu_data_port": 58000,
      "log_data_ip" : "",
      "log_data_port": 59000
    }
  },
  "MID360": {
    "lidar_net_info" : {  # Mid360 ports, please don't revise these values
      "cmd_data_port": 56100,  # Mid360 command port
      "push_msg_port": 56200,
      "point_data_port": 56300,
      "imu_data_port": 56400,
      "log_data_port": 56500
    },
    "host_net_info" : {
      "cmd_data_ip" : "192.168.1.5",  # host ip
      "cmd_data_port": 56101,
      "push_msg_ip": "192.168.1.5",  # host ip
      "push_msg_port": 56201,
      "point_data_ip": "192.168.1.5",  # host ip
      "point_data_port": 56301,
      "imu_data_ip" : "192.168.1.5",  # host ip
      "imu_data_port": 56401,
      "log_data_ip" : "",
      "log_data_port": 56501
    }
  },
  "lidar_configs" : [
    {
      "ip" : "192.168.1.100",  # ip of the HAP you want to config
      "pcl_data_type" : 1,
      "pattern_mode" : 0,
      "blind_spot_set" : 50,
      "extrinsic_parameter" : {
        "roll": 0.0,
        "pitch": 0.0,
        "yaw": 0.0,
        "x": 0,
        "y": 0,
        "z": 0
      }
    },
    {
      "ip" : "192.168.1.12",  # ip of the Mid360 you want to config
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
```

3. 主机多网口连接多台雷达时，需要在 `lidar_configs` 中追加对应配置，并分别运行不同启动文件。示例：

**MID360_config1：**

```json
{
  "lidar_summary_info" : {
    "lidar_type": 8  # protocol type index，please don't revise this value
  },
    "MID360": {
        "lidar_net_info": {
            "cmd_data_port": 56100, # command port
            "push_msg_port": 56200, 
            "point_data_port": 56300,
            "imu_data_port": 56400,
            "log_data_port": 56500
        },
        "host_net_info": [
            {
                "lidar_ip": ["192.168.1.100"], # Lidar ip
                "host_ip": "192.168.1.5", # host ip
                "cmd_data_port": 56101,
                "push_msg_port": 56201,
                "point_data_port": 56301,
                "imu_data_port": 56401,
                "log_data_port": 56501
            }
        ]
    },
    "lidar_configs": [
        {
            "ip": "192.168.1.100", # ip of the LiDAR you want to config
            "pcl_data_type": 1,
            "pattern_mode": 0,
            "extrinsic_parameter": {
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
```

**MID360_config2：**

```json
{
  "lidar_summary_info" : {
    "lidar_type": 8  # protocol type index，please don't revise this value
  },
    "MID360": {
        "lidar_net_info": {
            "cmd_data_port": 56100, # command port
            "push_msg_port": 56200, 
            "point_data_port": 56300,
            "imu_data_port": 56400,
            "log_data_port": 56500
        },
        "host_net_info": [
            {
                "lidar_ip": ["192.168.2.100"], # Lidar ip
                "host_ip": "192.168.2.5", # host ip
                "cmd_data_port": 56101,
                "push_msg_port": 56201,
                "point_data_port": 56301,
                "imu_data_port": 56401,
                "log_data_port": 56501
            }
        ]
    },
    "lidar_configs": [
        {
            "ip": "192.168.2.100", # ip of the LiDAR you want to config
            "pcl_data_type": 1,
            "pattern_mode": 0,
            "extrinsic_parameter": {
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
```

**Launch1：**

```
<launch>
    <!--user configure parameters for ros start-->
    <arg name="lvx_file_path" default="livox_test.lvx"/>
    <arg name="bd_list" default="100000000000000"/>
    <arg name="xfer_format" default="0"/>
    <arg name="multi_topic" default="1"/>
    <arg name="data_src" default="0"/>
    <arg name="publish_freq" default="10.0"/>
    <arg name="output_type" default="0"/>
    <arg name="rviz_enable" default="true"/>
    <arg name="rosbag_enable" default="false"/>
    <arg name="cmdline_arg" default="$(arg bd_list)"/>
    <arg name="msg_frame_id" default="livox_frame"/>
    <arg name="lidar_bag" default="true"/>
    <arg name="imu_bag" default="true"/>
    <!--user configure parameters for ros end--> 

    <param name="xfer_format" value="$(arg xfer_format)"/>
    <param name="multi_topic" value="$(arg multi_topic)"/>
    <param name="data_src" value="$(arg data_src)"/>
    <param name="publish_freq" type="double" value="$(arg publish_freq)"/>
    <param name="output_data_type" value="$(arg output_type)"/>
    <param name="cmdline_str" type="string" value="$(arg bd_list)"/>
    <param name="cmdline_file_path" type="string" value="$(arg lvx_file_path)"/>
    <param name="user_config_path" type="string" value="$(find livox_ros_driver2)/config/MID360_config1.json"/> # Mid360 MID360_config1 name
    <param name="frame_id" type="string" value="$(arg msg_frame_id)"/>
    <param name="enable_lidar_bag" type="bool" value="$(arg lidar_bag)"/>
    <param name="enable_imu_bag" type="bool" value="$(arg imu_bag)"/>

    <node name="livox_lidar_publisher1" pkg="livox_ros_driver2"
          type="livox_ros_driver2_node" required="true"
          output="screen" args="$(arg cmdline_arg)"/>

    <group if="$(arg rviz_enable)">
        <node name="livox_rviz" pkg="rviz" type="rviz" respawn="true"
                args="-d $(find livox_ros_driver2)/config/display_point_cloud_ROS1.rviz"/>
    </group>

    <group if="$(arg rosbag_enable)">
        <node pkg="rosbag" type="record" name="record" output="screen"
                args="-a"/>
    </group>

</launch>
```

**Launch2：**

```
<launch>
    <!--user configure parameters for ros start-->
    <arg name="lvx_file_path" default="livox_test.lvx"/>
    <arg name="bd_list" default="100000000000000"/>
    <arg name="xfer_format" default="0"/>
    <arg name="multi_topic" default="1"/>
    <arg name="data_src" default="0"/>
    <arg name="publish_freq" default="10.0"/>
    <arg name="output_type" default="0"/>
    <arg name="rviz_enable" default="true"/>
    <arg name="rosbag_enable" default="false"/>
    <arg name="cmdline_arg" default="$(arg bd_list)"/>
    <arg name="msg_frame_id" default="livox_frame"/>
    <arg name="lidar_bag" default="true"/>
    <arg name="imu_bag" default="true"/>
    <!--user configure parameters for ros end--> 

    <param name="xfer_format" value="$(arg xfer_format)"/>
    <param name="multi_topic" value="$(arg multi_topic)"/>
    <param name="data_src" value="$(arg data_src)"/>
    <param name="publish_freq" type="double" value="$(arg publish_freq)"/>
    <param name="output_data_type" value="$(arg output_type)"/>
    <param name="cmdline_str" type="string" value="$(arg bd_list)"/>
    <param name="cmdline_file_path" type="string" value="$(arg lvx_file_path)"/>
    <param name="user_config_path" type="string" value="$(find livox_ros_driver2)/config/MID360_config2.json"/> # Mid360 MID360_config2 name
    <param name="frame_id" type="string" value="$(arg msg_frame_id)"/>
    <param name="enable_lidar_bag" type="bool" value="$(arg lidar_bag)"/>
    <param name="enable_imu_bag" type="bool" value="$(arg imu_bag)"/>

    <node name="livox_lidar_publisher2" pkg="livox_ros_driver2"
          type="livox_ros_driver2_node" required="true"
          output="screen" args="$(arg cmdline_arg)"/>

    <group if="$(arg rviz_enable)">
        <node name="livox_rviz" pkg="rviz" type="rviz" respawn="true"
                args="-d $(find livox_ros_driver2)/config/display_point_cloud_ROS1.rviz"/>
    </group>

    <group if="$(arg rosbag_enable)">
        <node pkg="rosbag" type="record" name="record" output="screen"
                args="-a"/>
    </group>

</launch>
```

## 5. 支持的 LiDAR 列表

- HAP
- Mid360
- （更多型号持续更新）

## 6. 常见问题

### 6.1 使用 "livox_lidar_rviz_HAP.launch" 启动后 rviz 无点云显示？

请检查 RViz 的 “Global Options - Fixed Frame”，设置为 `livox_frame`，并勾选 “PointCloud2”。

### 6.2 使用 "ros2 launch livox_lidar_rviz_HAP_launch.py" 启动后提示找不到 "liblivox_sdk_shared.so"？

请将 `/usr/local/lib` 加入环境变量 `LD_LIBRARY_PATH`。

- 当前终端生效：

  ```shell
  export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:/usr/local/lib
  ```

- 当前用户永久生效：

  ```shell
  vim ~/.bashrc
  export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:/usr/local/lib
  source ~/.bashrc
  ```
