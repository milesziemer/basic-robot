#include <errno.h>
#include <fcntl.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/select.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>

#include "basic_robot_interfaces/msg/polar.hpp"
#include "rclcpp/rclcpp.hpp"

using basic_robot_interfaces::msg::Polar;

const size_t DESCRLEN      = 7;
const size_t HEALTHLEN     = 3;
const size_t SAMPLERATELEN = 4;
const ssize_t SCANLEN      = 5;

const unsigned char STOP[]       = {0xa5, 0x25};
const unsigned char RESET[]      = {0xa5, 0x40};
const unsigned char HEALTH[]     = {0xa5, 0x52};
const unsigned char SAMPLERATE[] = {0xa5, 0x59};
const unsigned char SCAN[]       = {0xa5, 0x20};

// Print out a buffer with a prefix in front
void printbuf(const char *prefix, unsigned char buf[], ssize_t n) {
    printf("%s\n > ", prefix);
    for (int i = 0; i < n; i++) {
        printf("%x ", buf[i]);
    }
    printf("\n");
}

// Read a descriptor and print it
void descriptor(int serial_port, const char *prefix) {
    unsigned char buf[DESCRLEN];
    ssize_t n = read(serial_port, &buf, DESCRLEN);
    printbuf(prefix, buf, n);
}

// Get lidar health
void health(int serial_port) {
    write(serial_port, HEALTH, sizeof(HEALTH));
    descriptor(serial_port, "health desc");
    unsigned char health_buf[HEALTHLEN];
    ssize_t nhealth = read(serial_port, &health_buf, HEALTHLEN);
    int status      = health_buf[0];
    int errorcode   = health_buf[2] << 8 | health_buf[1];
    printf("health\n > status: %i, error code: %i\n", status, errorcode);
}

// Get lidar samplerate
void samplerate(int serial_port) {
    write(serial_port, SAMPLERATE, sizeof(SAMPLERATE));
    descriptor(serial_port, "samplerate desc");
    unsigned char samplerate_buf[SAMPLERATELEN];
    ssize_t nsamplerate = read(serial_port, &samplerate_buf, SAMPLERATELEN);
    printbuf("samplerate", samplerate_buf, nsamplerate);
    int tstandard = samplerate_buf[1] << 8 | samplerate_buf[0];
    int texpress  = samplerate_buf[3] << 8 | samplerate_buf[2];
    printf("samplerate\n > standard: %i, express: %i\n", tstandard, texpress);
}

// Stop the scan
void stop(int serial_port) {
    printf("stopping...");
    write(serial_port, STOP, sizeof(STOP));
    usleep(1000); // Wait for 1 ms to account for lack of response
    printf("done\n");
}

// Reset device
void reset(int serial_port) {
    printf("resetting...");
    write(serial_port, RESET, sizeof(RESET));
    usleep(2000); // Wait for 2 ms to account for lack of response
    printf("done\n");
}

// Serial port to read and write to lidar device
// Declared here to use in sigint handler
int _serial_port;

// Make sure to stop the lidar and close the port before shutting down
void handlesigint(int sig) {
    printf("received signal %i\n", sig);
    stop(_serial_port);
    close(_serial_port);
    rclcpp::shutdown();
    printf("exiting\n");
    exit(1);
}

int main(int argc, char *argv[]) {

    rclcpp::init(argc, argv);

    // Open serial port
    _serial_port = open("/dev/ttyUSB0", O_RDWR);
    if (_serial_port < 0) {
        printf("Error %i from open: %s\n", errno, strerror(errno));
    }

    // Create new termios struct and read in old settings, checking for error
    struct termios tty;

    if (tcgetattr(_serial_port, &tty) != 0) {
        printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
    }

    // Set in/out baud rate for lidar device
    cfsetospeed(&tty, B115200);
    cfsetispeed(&tty, B115200);

    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~PARENB;  // Use no check bit
    tty.c_cflag &= ~CSTOPB;  // Use one stop bit
    tty.c_cflag &= ~CRTSCTS; // Disable flow control

    // Set byte size
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;

    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off software flow control
    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); // Use raw input
    tty.c_oflag &= ~OPOST;                          // Use raw output

    // Wait for any data up to half a second
    tty.c_cc[VTIME] = 5;
    tty.c_cc[VMIN]  = 0;

    // Save tty settings, also checking for error
    if (tcsetattr(_serial_port, TCSANOW, &tty) != 0) {
        printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
        close(_serial_port);
        rclcpp::shutdown();
        return 1;
    }

    tcflush(_serial_port, TCIFLUSH);

    // Configure DTR
    int dtrbit = TIOCM_DTR;
    ioctl(_serial_port, TIOCMBIC, &dtrbit);

    health(_serial_port);     // get health
    samplerate(_serial_port); // get samplerate

    tty.c_cc[VMIN] = 5; // Wait for 5 bytes from lidar

    // Save settings, and close port if necessary
    if (tcsetattr(_serial_port, TCSANOW, &tty) != 0) {
        printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
        close(_serial_port);
        rclcpp::shutdown();
        return 1;
    }

    // Setup node and create publisher
    auto node = rclcpp::Node::make_shared("lidar_node");
    auto pub  = node->create_publisher<Polar>("lidar_topic", 10);
    auto msg_ = std::make_shared<Polar>();

    write(_serial_port, SCAN, sizeof(SCAN)); // Start scan

    signal(SIGINT, handlesigint); // Setup sigint handler

    descriptor(_serial_port, "scan desc"); // Read in descriptor

    // Read from serial port, process and publish data
    while (rclcpp::ok()) {

        unsigned char scan_buf[SCANLEN];
        ssize_t nscan = read(_serial_port, &scan_buf, SCANLEN);

        // If there is a timeout, or no response don't try to process and
        // publish data
        if (nscan != SCANLEN) {
            printf("scan error, length: %lu\n", nscan);
            continue;
        }
        int theta    = ((scan_buf[2] << 7) | (scan_buf[1] >> 1)) / 64;
        int radius   = ((scan_buf[4] << 8) | (scan_buf[3])) / 4;
        msg_->radius = radius;
        msg_->theta  = theta;
        pub->publish(*msg_);
        rclcpp::spin_some(node);
    }

    // Cleanup
    stop(_serial_port);
    close(_serial_port);
    rclcpp::shutdown();

    return 0;
}