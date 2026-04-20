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

******************************************************************/
#include "whi_modbus_server/modbus_server.h"

#include <yaml-cpp/yaml.h>

#include <thread>

namespace whi_modbus_server
{
    Modbus::Modbus(std::shared_ptr<rclcpp::Node>& NodeHandle)
        : node_handle_(NodeHandle)
    {
        init();
    }

    Modbus::~Modbus()
    {
	    if (serial_inst_)
	    {
		    serial_inst_->close();
	    }

        terminated_.store(true);
        if (th_read_.joinable())
        {
            th_read_.join();
        }
    }

    void Modbus::init()
    {
        // params
        node_handle_->declare_parameter("frequency", 20.0);
        node_handle_->declare_parameter("port", "/dev/ttyUSB0");
        node_handle_->declare_parameter("baudrate", 9600);
        duration_ = std::chrono::duration<double>(1.0 / node_handle_->get_parameter("frequency").as_double());
        serial_port_ = node_handle_->get_parameter("port").as_string();
        baudrate_ = node_handle_->get_parameter("baudrate").as_int();

        // serial
	    try
	    {
		    serial_inst_ = std::make_unique<serial::Serial>(serial_port_, baudrate_, serial::Timeout::simpleTimeout(500));
	    }
	    catch (serial::IOException& e)
	    {
		    RCLCPP_FATAL_STREAM(node_handle_->get_logger(), "\033[1;31" << "failed to open serial " <<
                serial_port_ << "\033[0m");
	    }

        if (serial_inst_)
        {
            service_ = node_handle_->create_service<whi_interfaces::srv::WhiSrvModBus>("modbus_request", 
                std::bind(&Modbus::onService, this, std::placeholders::_1, std::placeholders::_2));
            subscriber_ = node_handle_->create_subscription<whi_interfaces::msg::WhiModBus>(
                "modbus_request", 10, std::bind(&Modbus::callbackSub, this, std::placeholders::_1));

            // spawn the read thread
            th_read_ = std::thread(std::bind(&Modbus::threadRead, this));
        }
    }

    void Modbus::threadRead()
    {
        while (!terminated_.load())
        {
            size_t count = serial_inst_->available();
            if (count > 0)
            {
                unsigned char rbuff[count];
                size_t readNum = serial_inst_->read(rbuff, count);
                if (readNum > 1)
                {
                    if (auto search = read_map_.find(rbuff[0]); search != read_map_.end())
                    {
                        if (auto search_pack = search->second.pack_map_.find(rbuff[1]); search_pack != search->second.pack_map_.end())
                        {
                            std::lock_guard<std::mutex> lock(search_pack->second->mtx_);
                            search_pack->second->data_ = std::vector<uint8_t>(rbuff, rbuff + readNum);
                            search_pack->second->read_time_ = rclcpp::Clock().now();
                            search_pack->second->ready_ = true;
                            search_pack->second->cv_.notify_all();
                        }
                    }
#ifdef DEBUG
    std::cout << "read device addr: " << int(rbuff[0]) << ", func: " << int(rbuff[1]) << ", data: ";
    for (size_t i = 0; i < readNum - 2; ++i)
    {
        std::cout << std::hex << int(rbuff[i + 2]) << ",";
    }
    std::cout << std::endl;
#endif
                }
            }

            std::this_thread::sleep_for(std::chrono::duration_cast<std::chrono::milliseconds>(duration_));
        }
    }

    void Modbus::sendRequest(const whi_interfaces::msg::WhiModBus& Msg)
    {
        read_map_[Msg.device] = Data();
        read_map_.at(Msg.device).pack_map_[Msg.func] = std::make_shared<Data::Pack>();
        read_map_.at(Msg.device).pack_map_.at(Msg.func)->write_time_ = rclcpp::Clock().now();

        try
        {
            std::vector<uint8_t> data;
            data.push_back(Msg.device);
            data.push_back(Msg.func);
            data.insert(data.end(), Msg.data.begin(), Msg.data.end());
            serial_inst_->write(data.data(), data.size());
#ifdef DEBUG
    std::cout << "write device addr: " << int(Msg.device) << ", func: " << int(Msg.func) << ", data: ";
    for (size_t i = 0; i < Msg.data.size(); ++i)
    {
        std::cout << std::dec << int(Msg.data[i]) << ",";
    }
    std::cout << std::endl;
#endif
        }
        catch (const serial::IOException& e) 
        {
		    RCLCPP_FATAL_STREAM(node_handle_->get_logger(), "\033[1;31" << "ModBUS IO Exception: " <<
                e.what() << "\033[0m");
        }
        catch (const serial::SerialException& e) 
        {
		    RCLCPP_FATAL_STREAM(node_handle_->get_logger(), "\033[1;31" << "ModBUS Serial Exception: " <<
                e.what() << "\033[0m");
        }
    }

    void Modbus::onService(const std::shared_ptr<whi_interfaces::srv::WhiSrvModBus::Request> Request,
        std::shared_ptr<whi_interfaces::srv::WhiSrvModBus::Response> Response)
    {
        sendRequest(Request->instance);
        
        {
            std::unique_lock lk(read_map_.at(Request->instance.device).pack_map_.at(Request->instance.func)->mtx_);
            read_map_.at(Request->instance.device).pack_map_.at(Request->instance.func)->cv_.wait_for(
                lk,
                std::chrono::seconds(1),
                [this, Request]{ return read_map_.at(Request->instance.device).pack_map_.at(Request->instance.func)->ready_; }
            );
        }

        if (read_map_.at(Request->instance.device).pack_map_.at(Request->instance.func)->ready_)
        {
            Response->data = read_map_.at(Request->instance.device).pack_map_.at(Request->instance.func)->data_;
            Response->result = true;
        }
        else
        {
            Response->result = false;
        }
    }

    void Modbus::callbackSub(const whi_interfaces::msg::WhiModBus::SharedPtr Msg)
    {
        sendRequest(*Msg);
    }
} // namespace whi_modbus_server
