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
#include "std_msgs/String.h"
#include "std_msgs/UInt8MultiArray.h"
#include "std_msgs/Float32.h"
#include <sstream>
#include <pthread.h>
#include <sys/types.h>
#include <sys/time.h>
#include <fcntl.h>
#include <termios.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string>

#define DEFAULT_BAUDRATE 9600
const int NUMBER_OF_SENSORS = 20;

timespec sleep_time_100ms;
timespec sleep_time_1ms;

// Global data
FILE *fpSerial = NULL;

ros::Publisher CI2;
ros::Publisher CO2;
ros::Publisher CO;
ros::Publisher HCN;
ros::Publisher H2S;
ros::Publisher LEL;
ros::Publisher NH3;
ros::Publisher OXY;
ros::Publisher SO2;
ros::Publisher VOC;

int ucIndex;

// Initialize serial port, return file descriptor
FILE *serialInit(const char * port, int baud)
{
  int BAUD = 0;
  int fd = -1;
  struct termios newtio;
  FILE *fp = NULL;

  // Open the serial port as a file descriptor for low level configuration
  // read/write, not controlling terminal for process,
  fd = open(port, O_RDWR | O_NOCTTY | O_NDELAY);
  if (fd < 0)
  {
    ROS_ERROR("serialInit: Could not open serial device %s", port);
    return fp;
  }

  // set up new settings
  memset(&newtio, 0, sizeof(newtio));
  newtio.c_cflag =  CS8 | CLOCAL | CREAD;  // no parity, 1 stop bit
  newtio.c_iflag = IGNCR;    // ignore CR, other options off
  newtio.c_iflag |= IGNBRK;  // ignore break condition
  newtio.c_oflag = 0;        // all options off

  // activate new settings
  tcflush(fd, TCIFLUSH);
  // Look up appropriate baud rate constant
  switch (baud)
  {
    case 38400:
      default:
        BAUD = B38400;
        break;
    case 19200:
        BAUD  = B19200;
        break;
    case 9600:
        BAUD  = B9600;
        break;
    case 4800:
        BAUD  = B4800;
        break;
    case 2400:
        BAUD  = B2400;
        break;
    case 1800:
        BAUD  = B1800;
        break;
    case 1200:
        BAUD  = B1200;
        break;
  }  // end of switch baud_rate

  if (cfsetispeed(&newtio, BAUD) < 0 || cfsetospeed(&newtio, BAUD) < 0)
  {
      ROS_ERROR("serialInit: Failed to set serial baud rate: %d", baud);
      close(fd);
      return NULL;
  }
  tcsetattr(fd, TCSANOW, &newtio);
  tcflush(fd, TCIOFLUSH);

  // Open file as a standard I/O stream
  fp = fdopen(fd, "r+");  // "r+ = read and write access
  if (!fp)
  {
      ROS_ERROR("serialInit: Failed to open serial stream %s", port);
      fp = NULL;
  }
  return fp;
}  // serialInit

unsigned char sensed_distances[NUMBER_OF_SENSORS];

// Receive command responses from robot uController
// and publish as a ROS message
void *rcvThread(void *arg)
{
  const int kRcvBufSize = 1000;  // Buffer some messages
  char rcvBuffer[kRcvBufSize];  // response string from uController
  char *bufPos = NULL;
  std_msgs::UInt8MultiArray msgArray;
  msgArray.layout.dim.push_back(std_msgs::MultiArrayDimension());
  msgArray.layout.dim[0].size = NUMBER_OF_SENSORS;
  msgArray.layout.dim[0].stride = 1;
  msgArray.layout.dim[0].label = "multirae_sensor_data";

  ROS_INFO("rcvThread: receive thread running");

  // Number of times to try character reads on the serial bus
  int max_num_serial_reads = 1000;

  // sensor_names = Array of up to NUMBER_OF_SENSORS sensors, where
  // row 0 = sensor number in serial message
  // row 1 = name of the sensor, max of five characters
  char sensor_names[NUMBER_OF_SENSORS][5];
  memset(sensor_names, 0x00, sizeof(sensor_names));  // Set the whole array to 0x00

  // sensor_values = Array of up to NUMBER_OF_SENSORS values, where
  // row 0 = sensor number in serial message
  // row 1 = sensed value, max of 10 characters, stored as an array of characters
  char sensor_values[NUMBER_OF_SENSORS][10];
  memset(sensor_values, 0x00, sizeof(sensor_names));  // Set the whole array to 0x00

  unsigned char recd_char = 0;
  int rcvBufferCounter = 0;
  int j = 0;

  // Clear buffer before use by filling with invalids (zeros)
  memset(rcvBuffer, 0x00, kRcvBufSize);
  unsigned int number_sensors_present = 0;
  unsigned int char_cntr = 0;

  while (ros::ok())
  {
    // Read which sensors are present and the order
    fprintf(fpSerial, "%s", "N");  // Send the cmd "N" for Sensor Name

    for (int cntr = 0; cntr < max_num_serial_reads; cntr++)
    {
      // Parse the messages
      recd_char = fgetc(fpSerial);

      if (recd_char == '\0')
      {
        // End of the serial message, break from reading the message
        cntr = max_num_serial_reads;
      }
      else if (recd_char == 0x00)
      {
        ROS_INFO("NULL");
      }
      else if ((recd_char == 0xFF) || (recd_char == '\n'))
      {
        // Do nothing - this is blank space in the message
      }
      else if (recd_char == 0x09)
      {
        // Spaces separate the items and delineate the different sensors
        // Increment to the next sensor
        number_sensors_present++;
        char_cntr = 0;
      }
      else
      {
        // This character is part of the sensor name
        sensor_names[number_sensors_present][char_cntr] = recd_char;
        char_cntr++;
      }

      // Slight pause was necessary in between reads
      if (nanosleep(&sleep_time_1ms, &sleep_time_1ms) == -1)
          {
        ROS_ERROR("Woke up from sleep too soon.");
      }
    }

    if (nanosleep(&sleep_time_100ms, &sleep_time_100ms) == -1)
    {
      ROS_ERROR("Woke up from sleep too soon.");
    }


    while (1)
    {
      // Clear buffer before use by filling with invalids (zeros)
      memset(rcvBuffer, 0x00, kRcvBufSize);
      // R = Instant (Sensor) Reading - asks for the sensed values
      fprintf(fpSerial, "%s", "R");
      number_sensors_present = 0;
      char_cntr = 0;
      for (int cntr = 0; cntr < max_num_serial_reads; cntr++)
      {
        recd_char = fgetc(fpSerial);
        if (recd_char == '\0')  // 0x00
        {
          cntr = max_num_serial_reads;
        }
        else if (recd_char == 0x00)
        {
          ROS_INFO("NULL");
        }
        else if (recd_char == 0xFF)
        {
          // Do nothing - this is blank space
        }
        else if (recd_char == 0x09)
        {
          // Spaces separate the items
          number_sensors_present++;
          char_cntr = 0;
        }
        else
        {
          sensor_values[number_sensors_present][char_cntr] = recd_char;
          char_cntr++;
        }

        if (nanosleep(&sleep_time_1ms, &sleep_time_1ms) == -1)
        {
          ROS_ERROR("Woke up from sleep too soon.");
        }
      }

      for (int i = 0; i < NUMBER_OF_SENSORS; i++)
      {
        if (sensor_names[i][0] != 0x00)
        {
          std_msgs::Float32 msg;
          msg.data = strtof(sensor_values[i], NULL);
          // Brute force method of finding the correct sensor by name and publishing it
          if (strcmp(sensor_names[i], "CI2") == 0x00)
          {
            CI2.publish(msg);
          }
          else if (strcmp(sensor_names[i], "CO2") == 0x00)
          {
            CO2.publish(msg);
          }
          else if (strcmp(sensor_names[i], "CO") == 0x00)
          {
            CO.publish(msg);
          }
          else if (strcmp(sensor_names[i], "HCN") == 0x00)
          {
            HCN.publish(msg);
          }
          else if (strcmp(sensor_names[i], "H2S") == 0x00)
          {
            H2S.publish(msg);
          }
          else if (strcmp(sensor_names[i], "LEL") == 0x00)
          {
            LEL.publish(msg);
          }
          else if (strcmp(sensor_names[i], "NH3") == 0x00)
          {
            NH3.publish(msg);
          }
          else if (strcmp(sensor_names[i], "OXY") == 0x00)
          {
            OXY.publish(msg);
          }
          else if (strcmp(sensor_names[i], "SO2") == 0x00)
          {
            SO2.publish(msg);
          }
          else if (strcmp(sensor_names[i], "VOC") == 0x00)
          {
            VOC.publish(msg);
          }
        }
        else
        {
          // There are no more sensors in the serial messages
          i = NUMBER_OF_SENSORS;
        }
      }
      // sleep for a second, this reduces the burden on the robot's processor
      sleep(1);
    }
  }
    return NULL;
}  // rcvThread


int main(int argc, char **argv)
{
  // Init the ROS system
  ros::init(argc, argv, "multirae");

  // Establish this as a ROS node
  ros::NodeHandle rosNode;

  ROS_INFO_STREAM("multirae started");

  // default usb port name
  std::string port = "/dev/ttyUSB1";

  if (ros::param::get("/multirae/usb_port", port) == true)
  {
    ROS_INFO("USB Port name read from Parameter Server: %s", port.c_str());
  }
  else
  {
    ROS_INFO("USB Port name not read from Parameter Server, using default %s", port.c_str());
  }

  // baud rate
  int baud = 9600;
  if (ros::param::get("/multirae/baudrate", baud) == true)
  {
    ROS_INFO("Baudrate read from Parameter Server: %d", baud);
  }
  else
  {
    ROS_INFO("Baudrate not read from Parameter Server, using default %d", baud);
  }

  sleep_time_100ms.tv_nsec = 100000000;
  sleep_time_100ms.tv_sec = 0;
  sleep_time_1ms.tv_nsec = 1000000;
  sleep_time_1ms.tv_sec = 0;

  // receive thread ID
  pthread_t rcvThrID;
  int err = -1;

  ROS_INFO("connection initializing (%s) at %d baud", port.c_str(), baud);
  fpSerial = serialInit(port.c_str(), baud);
  if (!fpSerial )
  {
    ROS_ERROR("unable to create a new serial port");
    return 1;
  }
  ROS_INFO("serial connection successful");

  // Setup to publish ROS messages
  CI2 = rosNode.advertise<std_msgs::Float32>("multirae/CI2", 100);
  CO2 = rosNode.advertise<std_msgs::Float32>("multirae/CO2", 100);
  CO = rosNode.advertise<std_msgs::Float32>("multirae/CO", 100);
  HCN = rosNode.advertise<std_msgs::Float32>("multirae/HCN", 100);
  H2S = rosNode.advertise<std_msgs::Float32>("multirae/H2S", 100);
  LEL = rosNode.advertise<std_msgs::Float32>("multirae/LEL", 100);
  NH3 = rosNode.advertise<std_msgs::Float32>("multirae/NH3", 100);
  OXY = rosNode.advertise<std_msgs::Float32>("multirae/OXY", 100);
  SO2 = rosNode.advertise<std_msgs::Float32>("multirae/SO2", 100);
  VOC = rosNode.advertise<std_msgs::Float32>("multirae/VOC", 100);

  // Create receive thread
  err = pthread_create(&rcvThrID, NULL, rcvThread, NULL);
  if (err != 0)
  {
    ROS_ERROR("unable to create receive thread");
    return 1;
  }

  // Process ROS messages and send serial commands to uController
  ros::spin();

  fclose(fpSerial);
  ROS_INFO("r2Serial stopping");
  ROS_INFO_STREAM("multirae ended");
}
