#ifndef GANTRY_LOGGER_HPP
#define GANTRY_LOGGER_HPP

#include <fstream>
#include <string>
#include <chrono>
#include <iomanip>
#include <sstream>
#include <array>
#include <stdexcept>
#include <cstdlib>
#include <sys/stat.h>

#include <Eigen/Dense>
#include "rclcpp/rclcpp.hpp"
#include "crane_rrt/msg/gantry_crane_state.hpp"
#include "crane_state.hpp"
#include "crane_input.hpp"

namespace gantry {

class Logger {
public:
    explicit Logger(const std::string& file_name = "", rclcpp::Logger logger = rclcpp::get_logger("gantry_logger"))
    {
        std::string final_path;

        if (file_name.empty()) {
            final_path = getDefaultLogDirectory() + "/" + generateTimestampFilename();
        } else {
            if (file_name.find('/') == std::string::npos && file_name.find('\\') == std::string::npos) {
                final_path = getDefaultLogDirectory() + "/" + file_name;
            } else {
                final_path = file_name;
            }
        }

        mFile.open(final_path);
        if (!mFile.is_open()) {
            throw std::runtime_error("Unable to open log file: " + final_path);
        }

        RCLCPP_INFO(logger, "Logging to file: %s", final_path.c_str());

        mFile << "ros_time,sim_time,iteration,"
              << "pos_x,pos_y,alpha,beta,"
              << "vel_x,vel_y,dalpha,dbeta,"
              << "input_x,input_y\n";
        mFile.flush();
    }

    ~Logger() {
        if (mFile.is_open()) {
            mFile.close();
        }
    }

    void log(const rclcpp::Time& ros_time, double sim_time, uint64_t iteration,
             const std::array<double, 4>& pos, const std::array<double, 4>& vel,
             const std::array<double, 2>& input)
    {
        mFile << ros_time.seconds() << ","
              << sim_time << ","
              << iteration << ","
              << pos[0] << "," << pos[1] << "," << pos[2] << "," << pos[3] << ","
              << vel[0] << "," << vel[1] << "," << vel[2] << "," << vel[3] << ","
              << input[0] << "," << input[1] << "\n";
        mFile.flush();
    }

    void log(double sim_time, uint64_t iteration,
             const std::array<double, 4>& pos, const std::array<double, 4>& vel,
             const std::array<double, 2>& input)
    {
        log(rclcpp::Time(0, 0, RCL_ROS_TIME), sim_time, iteration, pos, vel, input);
    }

    void log(const rclcpp::Time& ros_time, double sim_time, uint64_t iteration,
             const Eigen::Matrix<double, 8, 1>& state,
             const Eigen::Matrix<double, 2, 1>& input)
    {
        mFile << ros_time.seconds() << ","
              << sim_time << ","
              << iteration << ","
              << state(0) << "," << state(1) << "," << state(2) << "," << state(3) << ","
              << state(4) << "," << state(5) << "," << state(6) << "," << state(7) << ","
              << input(0) << "," << input(1) << "\n";
        mFile.flush();
    }

    void log(double sim_time, uint64_t iteration,
             const Eigen::Matrix<double, 8, 1>& state,
             const Eigen::Matrix<double, 2, 1>& input)
    {
        log(rclcpp::Time(0, 0, RCL_ROS_TIME), sim_time, iteration, state, input);
    }

    void log(const crane_rrt::msg::GantryCraneState& msg)
    {
        mFile << rclcpp::Time(msg.stamp).seconds() << ","
              << msg.sim_time << ","
              << msg.iteration << ","
              << msg.position[0] << "," << msg.position[1] << "," << msg.position[2] << "," << msg.position[3] << ","
              << msg.velocity[0] << "," << msg.velocity[1] << "," << msg.velocity[2] << "," << msg.velocity[3] << ","
              << msg.input[0] << "," << msg.input[1] << "\n";
        mFile.flush();
    }

    template<typename Scalar>
    void log(const rclcpp::Time& ros_time, double sim_time, uint64_t iteration,
       const CraneState<Scalar>& state,
       const CraneInput<Scalar>& input)
    {
        log(ros_time, sim_time, iteration, state.toVector(), input.toVector());
    }

    template<typename Scalar>
    void log(double sim_time, uint64_t iteration,
       const CraneState<Scalar>& state,
       const CraneInput<Scalar>& input)
    {
        log(rclcpp::Time(0, 0, RCL_ROS_TIME), sim_time, iteration, state, input);
    }


private:
    std::ofstream mFile;

    std::string getDefaultLogDirectory() const
    {
        const char* home = std::getenv("HOME");
        if (!home) {
            throw std::runtime_error("Environment variable HOME not set. Cannot determine .ros directory.");
        }

        std::string ros_dir = std::string(home) + "/.ros";

        // Create directory if it does not exist
        struct stat st;
        if (stat(ros_dir.c_str(), &st) != 0) {
            mkdir(ros_dir.c_str(), 0755); // UNIX-style permissions
        }

        return ros_dir;
    }

    std::string generateTimestampFilename() const
    {
        auto now = std::chrono::system_clock::now();
        auto in_time = std::chrono::system_clock::to_time_t(now);
        std::tm buf;
    #ifdef _WIN32
        localtime_s(&buf, &in_time);
    #else
        localtime_r(&in_time, &buf);
    #endif
        std::ostringstream oss;
        oss << "log_" << std::put_time(&buf, "%Y%m%d_%H%M%S") << ".csv";
        return oss.str();
    }
};

} // namespace gantry

#endif // GANTRY_LOGGER_HPP
