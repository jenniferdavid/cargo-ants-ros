#ifndef constants_H
#define constants_H

//general purpose constants
#define SDF_MODE 1
#define LIDAR_MODE 2
#define BUFFER_LENGTH 2048 //in Bytes
#define STACKING_PACKETS 0
#define MESSAGE_COMPLETE 1
#define FIRST_PACKET 2

//UDP SDF constants
#define SDF_THIS_PORT 13000
//#define SDF_SENDER_PORT 65533 //port from where sdf (vehicle state + radar) data is being sent
#define SDF_SYNC_WORD 0xF0E1D200
#define SDF_COUNT_INIT 0xC3 //starting value of sdf datagram counter
#define SDF_DATAGRAM_PER_MSG 4
#define SDF_DATAGRAM_LENGTH 1472 //367*4Bytes + 4Bytes (with sync word)
#define SDF_MESSAGE_LENGTH 5872 //1468*4Bytes

//UPD LIDAR constants
#define LIDAR_THIS_PORT 2001
//#define LIDAR_PORT 65530 //port from where lidar data is being sent
#define LIDAR_SYNC_WORD 0xFEDCBA00
#define LIDAR_COUNT_INIT 0x98 //starting value of lidar datagram counter
#define LIDAR_DATAGRAM_PER_MSG 12
#define LIDAR_DATAGRAM_LENGTH 1472 //367*4Bytes + 4Bytes (with sync word)
#define LIDAR_MESSAGE_LENGTH 17616 //367*4*12Bytes

//IBEO CONSTANTS
#define NUM_IBEO_LAYERS 8 
#define IBEO_NUM_POINTS 1000

#endif
