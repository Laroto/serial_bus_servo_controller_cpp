#include "sbs_controller.hpp"
#include <chrono>

int main() {
    try {
        // Create controller object
        SBSController controller("/dev/ttyUSB0"); // Replace with the correct port

        // Move servos - in this case servo 5 to position 500 and servo 1 to position 1000 in 0ms
        controller.cmdServoMove({5, 1}, {500, 1000}, 0);

        // Get battery voltage
        float voltage = controller.cmdGetBatteryVoltage();
        std::cout << "Battery Voltage: " << voltage << "V\n";

        // Unload servos - this releases the servos / removes the torque
        auto start = std::chrono::steady_clock::now();
        while (std::chrono::steady_clock::now() - start < std::chrono::seconds(2)) {
            // Active wait so that the servos have time to move to the desired position before unloading
        }
        controller.cmdMultiServoUnload({5, 1});

        // Read servo positions
        auto positions = controller.cmdMultiServoPosRead({5});
        for (size_t i = 0; i < positions.size(); ++i) {
            std::cout << "Servo " << i + 1 << " Position: " << positions[i] << "\n";
        }

    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << "\n";
    }

    return 0;
}