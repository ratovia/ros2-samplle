# ROS2 Action Serverの作成
Nodeとinterfaceの作成をする
Node・・・Action Serverを実行待機
interface・・・actionの定義

# Node
## ディレクトリ生成
Node
- ros2 pkg create apple --build-type ament_python

## Nodeの作成
apple/appleにnode.pyを作成する

## setup.pyを編集
```
from setuptools import setup

package_name = 'py_pubsub'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'simple_node = py_pubsub.simple_node:main',
        ],
    },
)
```

## package.xmlを編集
```
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>my_action_server</name>
  <version>0.0.0</version>
  <description>TODO: Package description</description>
  <maintainer email="root@todo.todo">root</maintainer>
  <license>TODO: License declaration</license>

  ### 追加
  <depend>rclpy</depend>

  <exec_depend>std_msgs</exec_depend>
  <exec_depend>std_srvs</exec_depend>
  <exec_depend>my_action_server_interface</exec_depend>
  ### ここまで

  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```



# interface

## ディレクトリ生成
interface
ros2 pkg create apple_interface --build-type ament_cmake --dependencies action_msgs rclcpp

## actionを作る
action/Notice.actionを作成する
actionは3つの---で区切られるフォーマット
```
# Goal
int32 order_id

---
# Result
bool success
string result_message

---
# feedback
int32 progress
```

# CMakeLists.txtの編集
```
cmake_minimum_required(VERSION 3.8)
project(apple_interface)

### 追加
# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
### ここまで


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(action_msgs REQUIRED)
### 削除
- find_package(rclcpp REQUIRED)
### ここまで

### 追加
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/Notice.action"
)

ament_export_dependencies(rosidl_default_runtime)
### ここまで

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
```

# 外から動作確認
ros2 action send_goal Notice my_action_server_interface/action/Notice "{order_id: 5}"
