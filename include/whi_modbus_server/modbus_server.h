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
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <bondcpp/bond.hpp>
#include <serial/serial.h>

#include <map>
#include <list>
#include <condition_variable>
#include <mutex>
#include <thread>

namespace whi_modbus_server
{
    using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

	class Modbus : public rclcpp_lifecycle::LifecycleNode
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
        Modbus(const std::string& NodeName = "whi_modbus_server",
            const rclcpp::NodeOptions& Options = rclcpp::NodeOptions());
        ~Modbus();

    public:
        // Create bond connection for nav2 lifecycle manager
        void createBond();
        // Destroy bond connection for nav2 lifecycle manager
        void destroyBond();
        CallbackReturn on_configure(const rclcpp_lifecycle::State&) override;
        CallbackReturn on_activate(const rclcpp_lifecycle::State&) override;
        CallbackReturn on_deactivate(const rclcpp_lifecycle::State&) override;
        CallbackReturn on_cleanup(const rclcpp_lifecycle::State&) override;
        CallbackReturn on_shutdown(const rclcpp_lifecycle::State&) override;

    protected:
        void init();
        void sendRequest(const whi_interfaces::msg::WhiModBus& Msg);
        void readResponse(int TryCount = 3, int DurationMs = 200);
        void onService(const std::shared_ptr<whi_interfaces::srv::WhiSrvModBus::Request> Request,
            std::shared_ptr<whi_interfaces::srv::WhiSrvModBus::Response> Response);
        void onMsg(const whi_interfaces::msg::WhiModBus::SharedPtr Msg);

    protected:
        int device_addr_{ 0x01 };
	    std::string serial_port_;
	    int baudrate_{ 9600 };
        std::unique_ptr<serial::Serial> serial_inst_{ nullptr };
        std::map<uint8_t, Data> read_map_;
        std::mutex mtx_;
        rclcpp::Service<whi_interfaces::srv::WhiSrvModBus>::SharedPtr service_{ nullptr};
        rclcpp::Subscription<whi_interfaces::msg::WhiModBus>::SharedPtr subscriber_{ nullptr };
        bool print_debug_rw_{ false };
        bool print_debug_restored_{ false };

        std::atomic_bool terminated_{ false };
        std::thread queue_thread_;
        std::list<whi_interfaces::msg::WhiModBus> queue_;
        rclcpp::CallbackGroup::SharedPtr async_callback_group_;

        bool with_bond_{ true };
        double heart_beat_period_{ 0.1 };
        double heart_beat_timeout_{ 4.0 };
        // Connection to tell that server is still up
        std::shared_ptr<bond::Bond> bond_{nullptr};
	};
} // namespace whi_modbus_server
