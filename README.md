Distribution A. Approved for public release; distribution unlimited. OPSEC Number: 4944

# Overview
Multirae is a ROS package supporting MultiRAE<sup>1</sup> gas monitors. The master branch has been tested on ROS Melodic with the MultiRAE Lite Pumped.

## How to Use
Build using catkin tools:

```
catkin build multirae 
```
Launch:
```
roslaunch multirae multirae.launch
```

To view output as a panel in RViz:

1. Install [Rviz Topic Viewer Plugin](https://gitlab.com/InstitutMaupertuis/topics_rviz_plugin).

## Troubleshooting
If the serial port is unable to be opened, it could be that the account you are running this from doesn't have correct dial in permissions, try: 
```
sudo usermod -a -G dialout <user account>
```
Also, the serial port may not have correct privileges. Try the following, after replacing /dev/ttyUSB0 with the correct port (ttyACM0 or ttyACM4, for example):

```
sudo chmod a+rw /dev/ttyUSB0 
```

## ROS API

## Nodes

### multirae 
- Interfaces with multirae sensor.
#### Published Topics

`~/multirae/CI2` ([std\_msgs/Float32](http://docs.ros.org/en/melodic/api/std_msgs/html/msg/Float32.html))

- The current CI2 reading.

`~/multirae/CO` ([std\_msgs/Float32](http://docs.ros.org/en/melodic/api/std_msgs/html/msg/Float32.html))

- The current CO reading.

`~/multirae/CO2` ([std\_msgs/Float32](http://docs.ros.org/en/melodic/api/std_msgs/html/msg/Float32.html))

- The current CO2 reading.

`~/multirae/HCN` ([std\_msgs/Float32](http://docs.ros.org/en/melodic/api/std_msgs/html/msg/Float32.html))

- The current HCN reading.

`~/multirae/H2S` ([std\_msgs/Float32](http://docs.ros.org/en/melodic/api/std_msgs/html/msg/Float32.html))

- The current H2S reading.

`~/multirae/LEL` ([std\_msgs/Float32](http://docs.ros.org/en/melodic/api/std_msgs/html/msg/Float32.html))

- The current LEL reading.

`~/multirae/NH3` ([std\_msgs/Float32](http://docs.ros.org/en/melodic/api/std_msgs/html/msg/Float32.html))

- The current NH3 reading.

`~/multirae/OXY` ([std\_msgs/Float32](http://docs.ros.org/en/melodic/api/std_msgs/html/msg/Float32.html))

- The current OXY reading.

`~/multirae/SO2` ([std\_msgs/Float32](http://docs.ros.org/en/melodic/api/std_msgs/html/msg/Float32.html))

- The current SO2 reading.

`~/multirae/VOC` ([std\_msgs/Float32](http://docs.ros.org/en/melodic/api/std_msgs/html/msg/Float32.html))

- The current VOC reading.
#### Subscribed Topics

- None

#### Services

- None

#### Parameters

`~/multirae/usb_port` (`String`, default: "/dev/ttyUSB1")

- USB port to interface with sensor.

MultiRAE<sup>1</sup> https://sps.honeywell.com/us/en/products/safety/gas-and-flame-detection/portables/multirae

## Hardware Setup
1. Purchase the sensor and travel charger (from either [Premier Safety](www.premiersafety.com) or [Accurate Safety](https://accsafety.com)).

    MAB3-B8C102E-020	MultiRAE Lite: pumped, CO2/LEL/CO/O2, wired w/ Li-ion battery [need it configured for P2P (Point to Point) comms]	$2,322.77

    SAS-0003-000		P2P upgrade (the M01-0309-000 Travel charger piece [$210.00] is included with P2P upgrade)

2. Configure the sensor for P2P (Point 2 Point) communications:


    1.	The MultiRAE needs to be firmware 1.50 or higher.  Make sure to upgrade Sensor firmware from [here](https://www.raesystems.com/customer-care/firmware-updates/multirae-sensor-firmware) first and Application firmware from [here](https://www.raesystems.com/customer-care/firmware-updates/multirae-application-firmware) second using the most recent version of ProRAE Studio II. from [here](https://www.raesystems.com/customer-care/software-updates/prorae-studio-ii).
    2.	Purchase the P2P upgrade Part Number (PN) SAS-0003-000 from Inside Sales order RMA group and they will send the P2P cable PN M01-2066-000 and travel charger PN M01-3021-100.
    3.	Make sure to reference the Sales Order and Serial Number(s) and send both to rae-callcenter@honeywell.com
    4.	We will send a Key to unlock the P2P function for your instrument(s).
    5.	Open ProRAE Studio II as Administrator using the password rae.
    6.	Select Operation.
    7.	Select License and then Next.
    8.	Type in the 20-digit Product key, follow the instruction to complete the process.
    9.	TN-190 found [here](https://honsps.my.salesforce.com/sfc/p/300000000P5P/a/f30000004Xev/i3HA.QDuPieBXRs02VDih7CdqSD4R70mECG1JwqwydY) has the instructions for using the P2P functions on the MultiRAE 
    *	Note: If you currently have a P2P instrument and wish to upgrade the firmware on your instrument, please send the serial number of your instrument to rae-callcenter@honeywell.com so that we can verify the instrument.  We will then send you the product code to be able to follow steps 5-8.

3. Technical Note TN-190 says to connect the travel charger via RS-232, this is incorrect.  Use a USB mini -USB Std A cable to connect the data link to a Windows or Linux computer

4. Power on sensor and enable PC Communications mode: Press the "N/-" button ~10 times to cycle through the menu choices until you get to "Enter Communications Mode?"  Select Yes and then "PC."

5. Then connect up the USB to the PC and start ProRAE Studio II, Admin password is "rae".  Connection is COMx (look in Device Manager to find it.  The manual says to make sure the value of x is 5 or higher.  COM6 worked for me) speed is 115200 by default.  

6. To flash with the P2P functionality, you'll need to purchase the MultiRAE upgrade kit "SAS-0003-000" which includes the M01-0309-000 travel charger needed for RS232 serial communication.  THe "regular" travel charger M01-3021-000 is USB only and cannot be used for P2P functionality.  Follow the procedure in P2P process.docx to flash with P2P serial upgrade




        