# whi_modbus_server
ModBUS server under ROS 2

## Dependency
```
git clone https://github.com/xinjuezou-whi/whi_interfaces.git
```

## Advertised topic
**modbus_request**

```
# client takes the charge of CRC
ros2 topic pub -1 /modbus_request whi_interfaces/msg/WhiModBus "{instance: {device: 2, func: 6, data: [0, 0, 0, 3, 201, 248], crc_size: 2}}"

# leave the CRC to server
ros2 topic pub -1 /modbus_request whi_interfaces/msg/WhiModBus "{instance: {device: 2, func: 6, data: [0, 0, 0, 3], crc_size: 0}}"
```

## Advertised service
**modbus_request**

```
# client takes the charge of CRC
ros2 service call /modbus_request whi_interfaces/srv/WhiSrvModBus "{instance: {device: 2, func: 6, data: [0, 0, 0, 3, 201, 248], crc_size: 2}}"

# leave the CRC to server
ros2 service call /modbus_request whi_interfaces/srv/WhiSrvModBus "{instance: {device: 2, func: 6, data: [0, 0, 0, 3], crc_size: 0}}"
```

## Configuration

```
frequency: 20.0 # hz
port: /dev/ttyUART_485_1
baudrate: 115200
```
