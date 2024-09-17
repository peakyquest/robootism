# Hardware Interfaces Explained

A hardware interface is a software component that can communicate with ROS2_control on one end, and one or more hardware devices at the other end.
For instance, you have a position sensor that sends out position data readings. These raw readings are received by the hardware interface and forwarded to a controller listening to that data. After the controller does its work, it outputs a new target command recieved by the hardware interface. It then is transformed into the specific data format required and understood by the hardware device. Therefore you can think of a hardware interface as an intermediary and interpreter that translates data from the hardware to the controller and vice-versa.

In reality, the hardware interface and ros2_control are coupled by a plugin interface. In technical terms, we say ROS2_control implements an "abstract class," and the hardware interface implements the "derived class." This way, the control algorithm does not have to know the specifics of the hardware. And because a hardware interface is a plugin, it does not create a ROS2 node.

## Code Explained 

### 1. `on_init(const hardware_interface::HardwareInfo & info)`
This function initializes the hardware interface. It performs the following actions:
- Checks if the hardware configuration is valid.
- Reads some example hardware parameters (e.g., start and stop durations, slowdown factor).
- Ensures that each joint has exactly one state interface and one command interface, both of which are for position control.

### 2. `on_configure(const rclcpp_lifecycle::State & previous_state)`
This function resets the state and command values to `0` for all joints when the hardware is being configured.

### 3. `export_state_interfaces()`
This function exports the state interfaces of the robot. It provides the current position of each joint.

### 4. `export_command_interfaces()`
This function exports the command interfaces of the robot, which allow for controlling the position of each joint.

### 5. `on_activate(const rclcpp_lifecycle::State & previous_state)`
This function is called when the hardware is being activated. It simulates some delay for hardware activation and ensures that the command values are initialized to the current state values.

### 6. `on_deactivate(const rclcpp_lifecycle::State & previous_state)`
This function is called when the hardware is being deactivated. It simulates some delay during the deactivation process.

### 7. `read(const rclcpp::Time & time, const rclcpp::Duration & period)`
This function reads the current state of the joints. It simulates the movement of the robot by slowly moving the current state towards the commanded position, based on a slowdown factor.

### 8. `write(const rclcpp::Time & time, const rclcpp::Duration & period)`
This function writes the command to the hardware. It simulates sending the commands to the joints, where the target position for each joint is printed.

### Communication with the Hardware (Arduino)
To communicate with the hardware we can use serial communication and from which we read and write the values. Following code can be used for the serial communication, and the serial function would be add in the *my_hardware_interface.cpp*

```
#include "serial_port.hpp"

// Function to open the serial port
int SerialPort::openSerialPort(const char* portname) {
    int fd = open(portname, O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0) {
        RCLCPP_ERROR(rclcpp::get_logger("SerialPort"), "Error opening %s: %s", portname, strerror(errno));
        return -1;
    }
    RCLCPP_INFO(rclcpp::get_logger("SerialPort"), "Serial port %s opened successfully", portname);
    return fd;
}

// Function to configure the serial port
bool SerialPort::configureSerialPort(int fd, int speed) {
    struct termios tty;
    if (tcgetattr(fd, &tty) != 0) {
        RCLCPP_ERROR(rclcpp::get_logger("SerialPort"), "Error from tcgetattr: %s", strerror(errno));
        return false;
    }

    cfsetospeed(&tty, speed);
    cfsetispeed(&tty, speed);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8; // 8-bit characters
    tty.c_iflag &= ~IGNBRK; // disable break processing
    tty.c_lflag = 0; // no signaling chars, no echo, no canonical processing
    tty.c_oflag = 0; // no remapping, no delays
    tty.c_cc[VMIN] = 0; // read doesn't block
    tty.c_cc[VTIME] = 5; // 0.5 seconds read timeout

    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff control

    tty.c_cflag |= (CLOCAL | CREAD); // ignore modem controls, enable reading
    tty.c_cflag &= ~(PARENB | PARODD); // shut off parity
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        RCLCPP_ERROR(rclcpp::get_logger("SerialPort"), "Error from tcsetattr: %s", strerror(errno));
        return false;
    }

    RCLCPP_INFO(rclcpp::get_logger("SerialPort"), "Serial port configured successfully");
    return true;
}

// Function to read data from the serial port
int SerialPort::readFromSerialPort(int fd, char* buffer, size_t size) {
    int n = read(fd, buffer, size);
    if (n < 0) {
        RCLCPP_ERROR(rclcpp::get_logger("SerialPort"), "Error reading from serial port: %s", strerror(errno));
    }
    return n;
}

// Function to write data to the serial port
int SerialPort::writeToSerialPort(int fd, const char* buffer, size_t size) {
    int n = write(fd, buffer, size);
    if (n < 0) {
        RCLCPP_ERROR(rclcpp::get_logger("SerialPort"), "Error writing to serial port: %s", strerror(errno));
    }
    return n;
}

// Function to close the serial port
void SerialPort::closeSerialPort(int fd) {
    close(fd);
    RCLCPP_INFO(rclcpp::get_logger("SerialPort"), "Serial port closed");
}
```



