/******************************************************************
class to handle modbus

Features:
- modbus
- xxx

Dependencies:
- sudo apt install ros-<ros distro>-serial
- sudo usermod -a -G dialout <user name>, then reboot

Written by Xinjue Zou, xinjue.zou@outlook.com
           Yuhang Shang

Apache License Version 2.0, check LICENSE for more information.
All text above must be included in any redistribution.

Changelog:
2026-04-20: Initial version
2026-xx-xx: xxx
******************************************************************/
#pragma once
#include <whi_interfaces/msg/whi_mod_bus.hpp>
#include <whi_interfaces/srv/whi_srv_mod_bus.hpp>

#include <rclcpp/rclcpp.hpp>
#include <serial/serial.h>

#include <condition_variable>
#include <mutex>
#include <map>
#include <thread>

namespace whi_modbus_server
{
	class Modbus
	{
    public:
        class Data
        {
        public:
            class Pack
            {
            public:
                Pack() = default;
                ~Pack() = default;
            
            public:
                rclcpp::Time write_time_;
                rclcpp::Time read_time_;
                std::mutex mtx_;
                std::condition_variable cv_;
                bool ready_{ false };
                std::vector<uint8_t> data_;
            };
        public:
            Data() = default;
            ~Data() = default;

        public:
            std::map<uint8_t, std::shared_ptr<Pack>> pack_map_;
        };

    public:
        Modbus() = delete;
        Modbus(std::shared_ptr<rclcpp::Node>& NodeHandle);
        ~Modbus();

    protected:
        void init();
        void threadRead();
        void sendRequest(const whi_interfaces::msg::WhiModBus& Msg);
        void onService(const std::shared_ptr<whi_interfaces::srv::WhiSrvModBus::Request> Request,
            std::shared_ptr<whi_interfaces::srv::WhiSrvModBus::Response> Response);
        void callbackSub(const whi_interfaces::msg::WhiModBus::SharedPtr Msg);

    protected:
        std::shared_ptr<rclcpp::Node> node_handle_{ nullptr };
        std::thread th_read_;
        std::atomic_bool terminated_{ false };
        std::chrono::duration<double> duration_{ 0.05 };
        int device_addr_{ 0x01 };
	    std::string serial_port_;
	    int baudrate_{ 9600 };
        std::unique_ptr<serial::Serial> serial_inst_{ nullptr };
        std::map<uint8_t, Data> read_map_;
        rclcpp::Service<whi_interfaces::srv::WhiSrvModBus>::SharedPtr service_{ nullptr};
        rclcpp::Subscription<whi_interfaces::msg::WhiModBus>::SharedPtr subscriber_{ nullptr };
	};
} // namespace whi_modbus_server
