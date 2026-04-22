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
    static uint16_t crc16(const uint8_t* Data, size_t Length)
    {
        uint16_t crc = 0xffff;
        uint16_t polynomial = 0xa001;

        for (size_t i = 0; i < Length; ++i)
        {
            crc ^= Data[i];
            for (int j = 0; j < 8; ++j)
            {
                if ((crc & 0x0001))
                {
                    crc = (crc >> 1) ^ polynomial;
                }
                else
                {
                    crc >>= 1;
                }
            }
        }

        return crc;
    }

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
        Data::Pack* restore = nullptr;

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
                            if (search_pack->second->data_.size() > 4)
                            {
                                search_pack->second->read_time_ = rclcpp::Clock().now();
                                search_pack->second->ready_ = true;
                                search_pack->second->cv_.notify_all();
                            }
                            else
                            {
                                restore = search_pack->second.get();
                                continue;
                            }
                        }
                    }
                    else
                    {
                        if (restore)
                        {
                            // simple restore mechanism
                            std::lock_guard<std::mutex> lock(restore->mtx_);
                            restore->data_.insert(restore->data_.end(), rbuff, rbuff + readNum);
                            restore->read_time_ = rclcpp::Clock().now();
                            restore->ready_ = true;
                            restore->cv_.notify_all();
#ifdef DEBUG
    std::cout << "restored data:";
    for (const auto& it : restore->data_)
    {
        std::cout << std::hex << int(it) << ",";
    }
    std::cout << std::endl;
#endif
                            restore = nullptr;
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
            if (Msg.crc_size == 0)
            {
                uint16_t crc = crc16(data.data(), data.size());
                data.push_back(crc);
                data.push_back(uint8_t(crc >> 8));
            }
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
                std::chrono::seconds(2),
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
