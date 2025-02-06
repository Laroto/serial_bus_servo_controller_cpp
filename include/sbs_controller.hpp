#ifndef SBS_CONTROLLER_HPP
#define SBS_CONTROLLER_HPP

#include <iostream>
#include <vector>
#include <stdexcept>
#include <cstring>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <chrono>
#include <optional>

class SBSController {
private:
    int serialPort;
    int baudRate_;
    struct termios tty;

    void writeToSerial(const std::vector<uint8_t>& buf) {
        if (write(serialPort, buf.data(), buf.size()) < 0) {
            throw std::runtime_error("Failed to write to serial port");
        }
    }

    int readWithTimeout(int serialPort, std::vector<char> &buffer, int expectedBytes, int timeout_us) {
        fd_set set;
        struct timeval timeout;
        
        FD_ZERO(&set);
        FD_SET(serialPort, &set);

        // Set timeout in seconds and microseconds with X microseconds of tolerance
        auto X = 5000; // 5ms tolerance 
        timeout.tv_sec = (timeout_us + X) / 1000000;
        timeout.tv_usec = (timeout_us + X) % 1000000;

        int bytesRead = 0;
        while (bytesRead < expectedBytes) {
            int rv = select(serialPort + 1, &set, NULL, NULL, &timeout);
            
            if (rv == -1) {
                throw std::runtime_error("Error: select() failed");
            } else if (rv == 0) {
                // throw std::runtime_error("Timeout: No data received from serial port");
                std::cerr << "Warning: Timeout occurred while reading from serial port" << std::endl;
            }

            // If data is available, read it
            int n = read(serialPort, buffer.data() + bytesRead, expectedBytes - bytesRead);
            if (n < 0) {
                throw std::runtime_error("Error reading from serial port");
            } else if (n == 0) {
                throw std::runtime_error("Serial port closed");
            }

            bytesRead += n;
        }
        return bytesRead;
    }

    std::optional<std::vector<uint8_t>> readFromSerial(size_t expectedBytes) {
        std::vector<uint8_t> buffer(expectedBytes);
        size_t bytesRead = 0;
        fd_set set;
        struct timeval timeout;
        
        FD_ZERO(&set);
        FD_SET(serialPort, &set);

        timeout.tv_sec = (expectedBytes * 10) / baudRate_;
        timeout.tv_usec = ((expectedBytes * 10 * 1000) / baudRate_) % 1000000;

        while (bytesRead < expectedBytes) {
            int rv = select(serialPort + 1, &set, NULL, NULL, &timeout);
            if (rv == -1) {
                throw std::runtime_error("Error: select() failed");
            } else if (rv == 0) {
                return std::nullopt; // Timeout occurred
            }

            int n = read(serialPort, buffer.data() + bytesRead, expectedBytes - bytesRead);
            if (n < 0) {
                throw std::runtime_error("Failed to read from serial port");
            }
            else if (n == 0) {
                std::cerr << "Warning: Serial port closed" << std::endl;
                return std::nullopt;
            }
            bytesRead += n;
        }

        return buffer;
    }

public:
    SBSController(const std::string& dev, int baudRate = 9600) {
        serialPort = open(dev.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
        baudRate_ = baudRate;
        if (serialPort < 0) {
            throw std::runtime_error("Failed to open serial port");
        }

        memset(&tty, 0, sizeof tty);
        if (tcgetattr(serialPort, &tty) != 0) {
            close(serialPort);
            throw std::runtime_error("Failed to get terminal attributes");
        }

        cfsetospeed(&tty, baudRate);
        cfsetispeed(&tty, baudRate);

        tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
        tty.c_iflag &= ~IGNBRK;
        tty.c_lflag = 0;
        tty.c_oflag = 0;
        tty.c_cc[VMIN] = 1;
        tty.c_cc[VTIME] = 1;

        tty.c_iflag &= ~(IXON | IXOFF | IXANY);
        tty.c_cflag |= (CLOCAL | CREAD);
        tty.c_cflag &= ~(PARENB | PARODD);
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~CRTSCTS;

        if (tcsetattr(serialPort, TCSANOW, &tty) != 0) {
            close(serialPort);
            throw std::runtime_error("Failed to set terminal attributes");
        }
    }

    ~SBSController() {
        close(serialPort);
    }

    void cmdServoMove(const std::vector<uint8_t>& servoIds, const std::vector<uint16_t>& positions, uint16_t time) {
        if (servoIds.size() != positions.size()) {
            throw std::invalid_argument("Servo IDs and positions must have the same length");
        }

        std::vector<uint8_t> buf = {0x55, 0x55};
        buf.push_back(static_cast<uint8_t>(servoIds.size() * 3 + 5));
        buf.push_back(0x03);
        buf.push_back(static_cast<uint8_t>(servoIds.size()));

        buf.push_back(static_cast<uint8_t>(time & 0xFF));
        buf.push_back(static_cast<uint8_t>((time >> 8) & 0xFF));

        for (size_t i = 0; i < servoIds.size(); ++i) {
            buf.push_back(servoIds[i]);
            buf.push_back(static_cast<uint8_t>(positions[i] & 0xFF));
            buf.push_back(static_cast<uint8_t>((positions[i] >> 8) & 0xFF));
        }

        writeToSerial(buf);
    }

    std::optional<float> cmdGetBatteryVoltage() {
        std::vector<uint8_t> buf = {0x55, 0x55, 0x02, 0x0F};
        writeToSerial(buf);
 

        auto response = readFromSerial(6);
        if (!response) {
            std::cerr <<"Battery voltage response timeout" << std::endl;
            return std::nullopt;
        }
        
        std::vector<uint8_t> response_data = response.value();

        if (response_data[0] == 0x55 && response_data[1] == 0x55 && response_data[3] == 0x0F) {
            uint16_t voltage = response_data[4] | (response_data[5] << 8);
            return voltage / 1000.0;
        }
        throw std::runtime_error("Invalid response for battery voltage");
    }

    void cmdMultiServoUnload(const std::vector<uint8_t>& servoIds) {
        std::vector<uint8_t> buf = {0x55, 0x55};
        buf.push_back(static_cast<uint8_t>(servoIds.size() + 3));
        buf.push_back(0x14);
        buf.push_back(static_cast<uint8_t>(servoIds.size()));

        for (auto id : servoIds) {
            buf.push_back(id);
        }

        writeToSerial(buf);
    }

    std::optional<std::vector<uint16_t>> cmdMultiServoPosRead(const std::vector<uint8_t>& servoIds) {

        std::vector<uint8_t> buf = {0x55, 0x55};
        buf.push_back(static_cast<uint8_t>(servoIds.size() + 3));
        buf.push_back(0x15);
        buf.push_back(static_cast<uint8_t>(servoIds.size()));

        for (auto id : servoIds) {
            buf.push_back(id);
        }

        writeToSerial(buf);

        size_t expectedBytes = servoIds.size() * 3 + 5;
        auto response = readFromSerial(expectedBytes);
        if (!response) {
            std::cerr << "Warning: Servo response timeout" << std::endl;
            return std::nullopt;
        }

        std::vector<uint8_t> response_data = response.value();
        
        if (response_data[0] == 0x55 && response_data[1] == 0x55 && response_data[3] == 0x15) {
            std::vector<uint16_t> positions(servoIds.size());
            for (size_t i = 0; i < servoIds.size(); ++i) {
                positions[i] = response_data[6 + 3 * i] | (response_data[7 + 3 * i] << 8);
            }
            return positions;
        }
    }
};

#endif // SBS_CONTROLLER_HPP
