/*
 * dxl_control_node.cpp
 *
 *  Created on: Feb 14, 2017
 *      Author: ubuntu-ti
 */

#include "dxl_xh430_control.h"

int getch()
{
#ifdef __linux__
  struct termios oldt, newt;
  int ch;
  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);
  ch = getchar();
  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  return ch;
#elif defined(_WIN32) || defined(_WIN64)
  return _getch();
#endif
}

int kbhit(void)
{
#ifdef __linux__
  struct termios oldt, newt;
  int ch;
  int oldf;

  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);
  oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
  fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

  ch = getchar();

  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  fcntl(STDIN_FILENO, F_SETFL, oldf);

  if (ch != EOF)
  {
    ungetc(ch, stdin);
    return 1;
  }

  return 0;
#elif defined(_WIN32) || defined(_WIN64)
  return _kbhit();
#endif
}

dynamixel::PortHandler   *portHandler;
dynamixel::PacketHandler *packetHandler;

bool DynamixelControl::DynamixelOpenPort()
{
	// Initialize PortHandler instance
	// Set the port path
	// Get methods and members of PortHandlerLinux or PortHandlerWindows
	portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);

	// Initialize PacketHandler instance
	// Set the protocol version
	// Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
	packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

	// Open port
	if (portHandler->openPort())
	{
		ROS_INFO("Succeeded to open the port!\n");
	}
	else
	{
		ROS_INFO("Failed to open the port!\n");
		ROS_INFO("Press any key to terminate...\n");
		getch();
		return false;
	}

	// Set port baudrate
	if (portHandler->setBaudRate(BAUDRATE))
	{
		ROS_INFO("Succeeded to change the baudrate!\n");
	}
	else
	{
		ROS_INFO("Failed to change the baudrate!\n");
		ROS_INFO("Press any key to terminate...\n");
		getch();
		return false;
	}
	return true;
}

uint8_t DynamixelControl::DynamixelTorqueEnable(uint8_t joint_index)
{
	int dxl_comm_result = COMM_TX_FAIL;   // Communication result
	uint8_t dxl_error = 0;                // Dynamixel error

   	// Enable Dynamixel#i Torque
   	dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, joint_index, ADDR_XH_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);

   	if (dxl_comm_result != COMM_SUCCESS)
   	{
   		packetHandler->printTxRxResult(dxl_comm_result);
   		return 0;
   	}
   	else if (dxl_error != 0)
   	{
   		packetHandler->printRxPacketError(dxl_error);
   		return 0;
   	}
   	else
   	{
   		ROS_INFO("All DXL has been successfully connected and TORQUE ON");
   	}
   	return 1;
}

uint8_t DynamixelControl::DynamixelTorqueEnableAll()
{

	DynamixelControl::DynamixelTorqueEnable(ADDR_XH_BROADCAST_ID );

}


uint8_t DynamixelControl::DynamixelTorqueDisable(uint8_t joint_index)
{
	int dxl_comm_result = COMM_TX_FAIL;     // Communication result
	uint8_t dxl_error = 0;                  // Dynamixel error

	// Disable Dynamixel#1 Torque
	dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, joint_index, ADDR_XH_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
	if (dxl_comm_result != COMM_SUCCESS)
	{
		packetHandler->printTxRxResult(dxl_comm_result);
	}
	else if (dxl_error != 0)
	{
		packetHandler->printRxPacketError(dxl_error);
	}else
	{
		ROS_INFO("All DXL has been successfully connected and TORQUE OFF");
	}
}

uint8_t DynamixelTorqueDisableAll(uint8_t joint_index)
{

	DynamixelControl::DynamixelTorqueDisable(ADDR_XH_BROADCAST_ID );

}


uint8_t DynamixelControl::DynamixelPing(uint8_t joint_index)
{
	int dxl_comm_result = COMM_TX_FAIL;      // Communication result
	uint8_t dxl_error = 0;                   // Dynamixel error
	uint16_t dxl_model_number;               // Dynamixel model number

	// Try to ping the Dynamixel
	// Get Dynamixel model number

	dxl_comm_result = packetHandler->ping(portHandler, joint_index, &dxl_model_number, &dxl_error);
	if (dxl_comm_result != COMM_SUCCESS){
		packetHandler->printTxRxResult(dxl_comm_result);
	}
	else if (dxl_error != 0){
		packetHandler->printRxPacketError(dxl_error);
		ROS_INFO("[ID:%03d] ping Failed. Dynamixel model number : %d",joint_index, dxl_model_number);
		return 0;
	}

	ROS_INFO("[ID:%03d] ping Succeeded. Dynamixel model number : %d",joint_index, dxl_model_number);

	// Close port
	// portHandler->closePort();
	return 1;
}



