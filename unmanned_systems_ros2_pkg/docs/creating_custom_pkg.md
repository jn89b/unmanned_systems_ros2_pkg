# How to build your custom package 


## Creating the CMake Package for cpp code
Please refer to this repo when making this package as an example 
https://github.com/jn89b/foxy-nvidia/tree/foxy/src/unmanned_systems

In the terminal cd into your ros_ws/src directory:
```
cd ros2_ws/src 
```

From there ros2_ws/src create a custom package from this command, where <package_name> is the name you want your custom package to be

```
ros2 pkg create --build-type ament_cmake <package_name>

#example
ros2 pkg create --build-type ament_cmake unmanned_systems
```

This makes a directory consisting of:
```
package_name/
├── include
│   └── 
├── src
│   └── 
├── CMakeLists.txt
├── package.xml
```

### Add Cpp code 
- In **package_name/src** add a cpp file called **cpp_node.cpp**
- In **package_name/include** add a cpp header file called **cpp_header.hpp**

## Add Python implementation to custom package
- In **package_name** add a file called **__init__.py**
- In **package_name** add a directory called **scripts**
- In **package_name** add a directory called **package_name**, yes it should be the same name as your top level directory 
- In **package_name/package_name** add a fille called **__init__.py**
- In **package_name/package_name** add a fille called **some_python_module.py**
- In **package_name/scripts** add a file called **py_node.py**

In **py_node.py** copy paste the following code:
```python
import rclpy
from rclpy.node import Node

from std_msgs.msg import String

"""IMPORT OUR PYTHON MODULE"""
from test_pkg.some_python_module import print_hello

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```


## Compile your custom package to implement Python and Cpp 
Delete your CMakLists and copy paste this one, replace **project(<test_pkg>)** with the name of your custom package, this allows you to include both cpp and python files in your package

```C
cmake_minimum_required(VERSION 3.5)
project(test_pkg)
# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)

# Include Cpp "include" directory
include_directories(include)
# Create Cpp executable
add_executable(cpp_node src/cpp_node.cpp)
ament_target_dependencies(cpp_node rclcpp)
# Install Cpp executables
install(TARGETS
  cpp_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})
# Install Python executables
install(PROGRAMS
  scripts/python_node.py
  DESTINATION lib/${PROJECT_NAME}
)
ament_package()
```

## Update package.xml
In **package_name/package.xml** copy paste the following
```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>test_pkg</name>
  <version>0.0.0</version>
  <description>TODO: Package description</description>
  <maintainer email="jnguyenblue2804@gmail.com">ros</maintainer>
  <license>TODO: License declaration</license>

  <buildtool_depend>ament_cmake</buildtool_depend>
  <buildtool_depend>ament_cmake_python</buildtool_depend>
  
  <depend>rclcpp</depend>
  <depend>rclpy</depend>

  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
``` 

Change <name>test_pkg</name> to   <name>"your_pkg_name"</name>

After **colcon** build your package with the following command:
```colcon build --packages-select test_pkg```
Build this at the top level directory 