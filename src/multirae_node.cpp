/******************************************************************************
 *
 * "Distribution A: Approved for public release; distribution unlimited.
 * OPSEC Number: 4944"
 *
 * PACKAGE         : multirae
 * ORIGINAL AUTHOR : Ty Valascho
 * MODIFIED DATE   : 14Dec2020
 * MODIFIED BY     :
 * REVISION        : 0
 *
 *Copyright (c) 2019-2020 U.S. Federal Government (in countries where recognized)
 *Permission is hereby granted, free of charge, to any person obtaining a copy of
 *this software and associated documentation files (the "Software"), to deal in
 *the Software without restriction, including without limitation the rights to use,
 *copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the
 *Software, and to permit persons to whom the Software is furnished to do so,
 *subject to the following conditions:
 *
 *The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 *THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 *EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 *MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 *IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
 *DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
 *ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 *DEALINGS IN THE SOFTWARE.
 *
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 *  You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 *****************************************************************************/

#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <fcntl.h>
#include <termios.h>

// boilerplate error handling
#define FAIL(REASON)          \
do {                          \
    ROS_ERROR_STREAM(REASON); \
    close(serial_port);       \
    r.sleep();                \
    goto INIT_DEVICE;         \
} while (0)


int serialInit(const char *port_name, int baud);


int main( int argc, char* argv[]) {

  ros::init(argc, argv, "multirae");
  ros::NodeHandle nh;
  // Attempt to use ROS parameters
  std::string port = "/dev/ttyUSB0";
  if (ros::param::get("usb_port", port))
    ROS_INFO("USB Port name read from parameter server: %s", port.c_str());
  else
    ROS_WARN("USB Port name not read from parameter server, using default %s", port.c_str());

  int baud = 9600;
  if (ros::param::get("baud_rate", baud))
    ROS_INFO("Baudrate read from parameter server: %d", baud);
  else
    ROS_WARN("Baudrate not read from parameter server, using default %d", baud);

  int poll_rate = 1;
  if (ros::param::get("poll_rate", poll_rate))
    ROS_INFO("Poll rate read from parameter server: %d", poll_rate);
  else
    ROS_WARN("Poll rate not read from parameter server, using default %d", poll_rate);
  ros::Rate r(poll_rate);

  INIT_DEVICE:                                                                 // Any fatal error will return to this point to re-initialize the device
  int bytes_read = 0;
  int serial_port = -1;
  struct timeval Timeout;
  Timeout.tv_sec = 0;
  fd_set read_fds;
  char read_buf[256];
  unsigned char msg = 'n';
  do{
    ROS_INFO("connection initializing (%s) at %d baud", port.c_str(), baud);
    serial_port = serialInit(port.c_str(), baud);                              // Attempt to open serial port
    if(serial_port < 0)
      FAIL("Unable to initialize serial port");
    FD_ZERO(&read_fds);
    FD_SET(serial_port, &read_fds);                                            // Create file descriptor set so `select` can check for ready data

    write(serial_port, &msg, 1);                                               // Send 'n' to get sensor names
    Timeout.tv_usec = ((1.0/poll_rate)*1000000);                               // Set timeout to match poll rate
    if(select(serial_port+1, &read_fds, NULL, NULL, &Timeout) == 1)            // Wait a max period of 'timeout' for a line to become available
      bytes_read = read(serial_port, &read_buf, sizeof(read_buf));             // Read in a line of sensor names
  }while (bytes_read <= 0);

  ROS_INFO("Successfully initialized");

  read_buf[bytes_read-2] = '\0';                                               // Truncate "\r\n" with null terminator
  std::istringstream iss(read_buf);                                            // Create string stream for parsing
  std::string sensor_name;
  std::vector<ros::Publisher> publishers;
  while (std::getline(iss, sensor_name, '\t'))                                 // Split the line by tab characters and create a publisher for each
    publishers.push_back(nh.advertise<std_msgs::Float32>(sensor_name, 100));

  ROS_INFO("Measuring sensors");
  msg='r';
  while (ros::ok()){
    write(serial_port, &msg, 1);                                               // Send 'r' to refresh data
    Timeout.tv_usec = ((1.0/poll_rate)*1000000);                               // reset timeout
    if(select(serial_port+1, &read_fds, NULL, NULL, &Timeout) == 1){           // Wat a max period of 'timeout' for a line to become available
      read(serial_port, NULL, 1);                                              // The multirae helpfully null-terminates its strings, but in canonical mode we don't want those because \n marks the end of transmission
      bytes_read = read(serial_port, &read_buf, sizeof(read_buf));             // Read in the line of sensor data
    }
    else
      FAIL("Serial device timeout");
    if (bytes_read <= 0)
      FAIL("Error reading from device");

    read_buf[bytes_read-2] = '\0';                                             // Truncate "\r\n" with null terminator
    iss = std::istringstream(read_buf);                                        // Create string stream for parsing
    std::string sensor_reading;
    for(ros::Publisher pub : publishers){                                      // Loop through publishers
      std::getline(iss, sensor_reading, '\t');                                   // Extract data substring separated by tab character
      if(sensor_reading=="N/A"){                                                 // Catch non-ready sensors
        ROS_WARN_STREAM("Sensor for "<<pub.getTopic()<<" is not yet running");
        continue;
      }
      std_msgs::Float32 ros_msg;
      ros_msg.data=std::stod(sensor_reading);                                    // Create ROS message and convert data string to double
      pub.publish(ros_msg);                                                      // Publish
    }
    r.sleep();
  }

  close(serial_port);
  return 0;
};



int serialInit(const char *port_name, int baud)
{
  int serial_port = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY);         // Open device read/write
  if (serial_port < 0)
    return -1;
  struct termios tty;                                                // Holds terminal settings

  if(tcgetattr(serial_port, &tty) != 0)                              // Read in existing settings
    return -2;

  tty.c_cflag = CS8 | CREAD | CLOCAL;                                // no parity, 1 stop bit

  tty.c_lflag = ICANON;                                              // Canonical mode - receive data in lines, buffer becomes available to read when newline received
  tty.c_lflag &= ~ISIG;                                              // Disable INTR, QUIT and SUSP
  tty.c_iflag &= ~(IXON | IXOFF | IXANY);                            // No flow control
  tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL);   // Disable special handling of received bytes

  switch (baud)                                                      // Look up baud rate constant
  {
    case 38400:
      default:
        baud = B38400;
        break;
    case 19200:
        baud  = B19200;
        break;
    case 9600:
        baud  = B9600;
        break;
    case 4800:
        baud  = B4800;
        break;
    case 2400:
        baud  = B2400;
        break;
    case 1800:
        baud  = B1800;
        break;
    case 1200:
        baud  = B1200;
        break;
  }

  if (cfsetispeed(&tty, baud) < 0 || cfsetospeed(&tty, baud) < 0)    // Set port speed
    return -3;

  if (tcsetattr(serial_port, TCSANOW, &tty) != 0)                    // Apply settings
    return -4;

  tcflush(serial_port, TCIOFLUSH);                                   // Clear out any stale data

  return serial_port;
}
