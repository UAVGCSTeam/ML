#include <iostream>
#include <fcntl.h>      // File control definitions
#include <unistd.h>     // UNIX standard function definitions
#include <termios.h>    // POSIX terminal control definitions
#include <errno.h>      // Error number definitions
#include <cstring>      // For memset
#include <iomanip>      // For hex output formatting

int main() {
    const char* device = "/dev/ttyUSB1";
    int fd = open(device, O_RDWR | O_NOCTTY | O_NDELAY);

    if (fd == -1) {
        std::cerr << "Failed to open port " << device << " ("
                  << strerror(errno) << ")" << std::endl;
        return 1;
    }

    // Make sure the file descriptor is blocking
    fcntl(fd, F_SETFL, 0);

    // configure port settings
    termios options;
    if (tcgetattr(fd, &options) != 0) {
        std::cerr << "Error getting port attributes (" 
                  << strerror(errno) << ")" << std::endl;
        close(fd);
        return 1;
    }

    // Set input/output baud rate to 115200
    cfsetispeed(&options, B115200);
    cfsetospeed(&options, B115200);

    // 8N1 mode (8 data bits, No parity, 1 stop bit)
    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;

    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    options.c_oflag &= ~OPOST;

    // Set vmin and vtime for better reading behavior
    options.c_cc[VMIN] = 0;   // Non-blocking read
    options.c_cc[VTIME] = 1;  // 100ms timeout

    if (tcsetattr(fd, TCSANOW, &options) != 0) {
        std::cerr << "Error setting port attributes ("
                  << strerror(errno) << ")" << std::endl;
        close(fd);
        return 1;
    }

    std::cout << "Reading from " << device << " at 115200 baud... (Press Ctrl+C to stop)\n";

    unsigned char frame[9];
    int frame_index = 0;

    while (true) {
        unsigned char byte;
        ssize_t bytes_read = read(fd, &byte, 1);

        if (bytes_read > 0) {
            // Search for frame header (0x59, 0x59)
            if (frame_index == 0 && byte == 0x59) {
                frame[0] = byte;
                frame_index = 1;
            } else if (frame_index == 1 && byte == 0x59) {
                frame[1] = byte;
                frame_index = 2;
            } else if (frame_index >= 2 && frame_index < 9) {
                frame[frame_index] = byte;
                frame_index++;

                // If we have a complete frame
                if (frame_index == 9) {
                    // Validate checksum
                    unsigned char checksum = 0;
                    for (int i = 0; i < 8; i++) {
                        checksum += frame[i];
                    }

                    if (checksum == frame[8]) {
                        // Extract distance (little endian: low byte first)
                        uint16_t distance = frame[2] | (frame[3] << 8);
                        
                        // Extract signal strength
                        uint16_t strength = frame[4] | (frame[5] << 8);
                        
                        // Optional temperature 
                        uint16_t temp_raw = frame[6] | (frame[7] << 8);
                        float temperature = temp_raw / 8.0f - 256.0f;

                        std::cout << "Distance: " << distance << " cm, ";
                        std::cout << "Signal: " << strength << ", ";
                        std::cout << "Temp: " << temperature << "°C" << std::endl;
                    } else {
                        std::cout << "Checksum error" << std::endl;
                    }

                    // Reset for next frame
                    frame_index = 0;
                }
            } else {
                // Invalid data, reset frame index
                frame_index = 0;
            }
        } else if (bytes_read < 0 && errno != EAGAIN) {
            std::cerr << "Error reading from port (" 
                      << strerror(errno) << ")" << std::endl;
            // Handle the error or break as needed
        }

        // Small delay to prevent tight loop
        usleep(1000);
    }

    close(fd);
    return 0;
}