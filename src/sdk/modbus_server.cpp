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

    Modbus::Modbus(const std::string& NodeName/* = "whi_modbus_server"*/,
        const rclcpp::NodeOptions& Options/* = rclcpp::NodeOptions()*/)
        : rclcpp_lifecycle::LifecycleNode(NodeName, "", Options)
    {
        // params
        declare_parameter("frequency", 20.0);
        declare_parameter("port", "/dev/ttyUSB0");
        declare_parameter("baudrate", 9600);
        declare_parameter("debug.print_debug_rw", false);
        declare_parameter("debug.print_debug_restored", false);
        declare_parameter("with_bond", true);
        declare_parameter("heart_beat_period", 0.1);
        declare_parameter("heart_beat_timeout", 4.0);
    }

    Modbus::~Modbus()
    {
	    if (serial_inst_)
	    {
		    serial_inst_->close();
	    }
    }

    void Modbus::createBond()
    {
        if (with_bond_)
        {
            RCLCPP_INFO(get_logger(), "Creating bond (%s) to lifecycle manager.", get_name());

            bond_ = std::make_shared<bond::Bond>(std::string("bond"), get_name(), shared_from_this());

            bond_->setHeartbeatPeriod(heart_beat_period_);
            bond_->setHeartbeatTimeout(heart_beat_timeout_);
            bond_->start();
        }
    }

    void Modbus::destroyBond()
    {
        if (with_bond_)
        {
            RCLCPP_INFO(get_logger(), "Destroying bond (%s) to lifecycle manager.", get_name());

            if (bond_)
            {
                bond_.reset();
            }
        }
    }

    CallbackReturn Modbus::on_configure(const rclcpp_lifecycle::State&)
    {
        RCLCPP_INFO(get_logger(), "Configuring");

        init();
        
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn Modbus::on_activate(const rclcpp_lifecycle::State&)
    {
        RCLCPP_INFO(get_logger(), "Activating");

        createBond();

        return CallbackReturn::SUCCESS;
    }

    CallbackReturn Modbus::on_deactivate(const rclcpp_lifecycle::State&)
    {
        RCLCPP_INFO(get_logger(), "Deactivating");

        if (serial_inst_)
	    {
		    serial_inst_->close();
	    }

        destroyBond();

        return CallbackReturn::SUCCESS;
    }

    CallbackReturn Modbus::on_cleanup(const rclcpp_lifecycle::State&)
    {
        RCLCPP_INFO(get_logger(), "Cleaning up");

        subscriber_.reset();
        service_.reset();

        return CallbackReturn::SUCCESS;
    }

    CallbackReturn Modbus::on_shutdown(const rclcpp_lifecycle::State&)
    {
        RCLCPP_INFO(get_logger(), "Shutting down");
        return CallbackReturn::SUCCESS;
    }

    void Modbus::init()
    {
        // params
        duration_ = std::chrono::duration<double>(1.0 / get_parameter("frequency").as_double());
        serial_port_ = get_parameter("port").as_string();
        baudrate_ = get_parameter("baudrate").as_int();
        print_debug_rw_ = get_parameter("debug.print_debug_rw").as_bool();
        print_debug_restored_ = get_parameter("debug.print_debug_restored").as_bool();
        with_bond_ = get_parameter("with_bond").as_bool();
        if (with_bond_)
        {
            heart_beat_period_ = get_parameter("heart_beat_period").as_double();
            heart_beat_timeout_ = get_parameter("heart_beat_timeout").as_double();
        }

        // serial
	    try
	    {
		    serial_inst_ = std::make_unique<serial::Serial>(serial_port_, baudrate_, serial::Timeout::simpleTimeout(500));
	    }
	    catch (serial::IOException& e)
	    {
		    RCLCPP_FATAL_STREAM(get_logger(), "\033[1;31" << "failed to open serial " <<
                serial_port_ << "\033[0m");
	    }

        if (serial_inst_)
        {
            service_ = create_service<whi_interfaces::srv::WhiSrvModBus>("modbus_request", 
                std::bind(&Modbus::onService, this, std::placeholders::_1, std::placeholders::_2));
            subscriber_ = create_subscription<whi_interfaces::msg::WhiModBus>(
                "modbus_request", 10, std::bind(&Modbus::callbackSub, this, std::placeholders::_1));
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

            if (print_debug_rw_)
            {
                std::cout << "write device addr: " << int(Msg.device) << ", func: " << int(Msg.func) << ", data: ";
                for (size_t i = 0; i < Msg.data.size(); ++i)
                {
                    std::cout << std::dec << int(Msg.data[i]) << ",";
                }
                std::cout << std::endl;
            }
        }
        catch (const serial::IOException& e) 
        {
		    RCLCPP_FATAL_STREAM(get_logger(), "\033[1;31" << "ModBUS IO Exception: " <<
                e.what() << "\033[0m");
        }
        catch (const serial::SerialException& e) 
        {
		    RCLCPP_FATAL_STREAM(get_logger(), "\033[1;31" << "ModBUS Serial Exception: " <<
                e.what() << "\033[0m");
        }
    }

    void Modbus::readResponse()
    {
        Data::Pack* restore = nullptr;

        do
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

                            if (print_debug_restored_)
                            {
                                std::cout << "restored data:";
                                for (const auto& it : restore->data_)
                                {
                                    std::cout << std::hex << int(it) << ",";
                                }
                                std::cout << std::endl;
                            }
                            restore = nullptr;
                        }
                    }

                    if (print_debug_rw_)
                    {
                        std::cout << "read device addr: " << int(rbuff[0]) << ", func: " << int(rbuff[1]) << ", data: ";
                        for (size_t i = 0; i < readNum - 2; ++i)
                        {
                            std::cout << std::hex << int(rbuff[i + 2]) << ",";
                        }
                        std::cout << std::endl;
                    }
                }
            }

            std::this_thread::sleep_for(std::chrono::duration_cast<std::chrono::milliseconds>(duration_));
        } while (restore);
    }

    void Modbus::onService(const std::shared_ptr<whi_interfaces::srv::WhiSrvModBus::Request> Request,
        std::shared_ptr<whi_interfaces::srv::WhiSrvModBus::Response> Response)
    {
        sendRequest(Request->instance);
        readResponse();
        
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
        readResponse();
    }
} // namespace whi_modbus_server
