#include "sbs_controller.hpp"
#include <chrono>

int main() {
    try {
        // Create controller object
        SBSController controller("/dev/ttyUSB0"); // Replace with the correct port

        std::vector<unsigned char> servo_ids_ = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18};

        // Read servo positions
        auto positions = controller.cmdMultiServoPosRead({servo_ids_});
        if (!positions) {
            std::cerr << "Failed to read servo positions\n";
            return 1;
        }
        auto pos_values = positions.value();
        for (size_t i = 0; i < pos_values.size(); ++i) {
            float degrees = ((static_cast<float>(pos_values[i]) - 750.0f) / 750.0f) * 120.0f;
            std::cout << "Servo " << i << " Position: " << pos_values[i] << "("<< degrees << ")\n";
        }

    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << "\n";
    }

    return 0;
}