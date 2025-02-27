#include <iostream>
#include <fcntl.h>      // File control definitions
#include <unistd.h>     // UNIX standard function definitions
#include <termios.h>    // POSIX terminal control definitions
#include <errno.h>      // Error number definitions
#include <cstring>      // For memset
#include <iomanip>      // For hex output formatting

int main() {
    const char* device = "/dev/ttyUSB1";

    // creates a file descriptor object based off the port given
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
    unsigned char byte;
    unsigned char checksum;
    uint16_t distance;
    uint16_t strength;

    while (true) {
        ssize_t bytes_read = read(fd, &byte, 1);

        if (bytes_read > 0) {
            // Search for frame header (0x59, 0x59)
            if (frame_index == 0 && byte == 0x59) {
                frame[0] = byte;
                checksum = byte;
                frame_index++;
            } else if (frame_index == 1 && byte == 0x59) {
                frame[1] = byte;
                checksum += byte;
                frame_index++;
            } else if (frame_index == 2) {
                frame[2] = byte;
                checksum += byte;
                frame_index++;
            } else if (frame_index == 3) {
                frame[3] = byte;
                checksum += byte;
                frame_index++;
                // Extract distance (little endian: low byte first)
                distance = frame[2] | (frame[3] << 8);
            } else if (frame_index == 4) {
                frame[4] = byte;
                checksum += byte;
                frame_index++;
            } else if (frame_index == 5) {
                frame[5] = byte;
                checksum += byte;
                frame_index++;
                // Extract signal strength (little endian: low byte first)
                strength = frame[4] | (frame[5] << 8);
            } else if (frame_index == 6) {
                frame[6] = byte;
                checksum += byte;
                frame_index++;
            } else if (frame_index == 7) {
                frame[7] = byte;
                checksum += byte;
                frame_index++;
            } else if (frame_index == 8) {
                frame[8] = byte;

                // If we have a complete frame
                if (frame_index == 9) {
                    // Validate checksum
                    if (checksum == byte) {
                        std::cout << "Distance: " << distance << " cm, ";
                        std::cout << "Signal: " << strength << std::endl;
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