#include <iostream>
#include <fstream>
#include <ctime>
#include <iomanip>
#include <filesystem>
#include <casadi/casadi.hpp>
#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

int main(int argc, char** argv) {
    // Initialize ROS2
    rclcpp::init(argc, argv);

    // file name
    std::string file_name = "gen";
    // code prefix
    std::string package_name = "px4_ros_com";
    std::string prefix_code = ament_index_cpp::get_package_share_directory(package_name) + "/include/";
    // shared library prefix
    std::string prefix_lib = ament_index_cpp::get_package_share_directory(package_name) + "/include/";

    // compile C++ code to a shared library
    std::string compile_command = "gcc -fPIC -shared -O3 " + 
        prefix_code + file_name + ".cpp -o " +
        prefix_lib + file_name + ".so";

    std::cout << compile_command << std::endl;
    int compile_flag = std::system(compile_command.c_str());
    if (compile_flag != 0) {
        std::cerr << "Compilation failed" << std::endl;
        return 1;
    }
    std::cout << "Compilation succeeded!" << std::endl;

    // Shutdown ROS2
    rclcpp::shutdown();

    return 0;
}

