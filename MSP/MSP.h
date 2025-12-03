#pragma once
#include <unistd.h>
#include <stdint.h>

#define MSPV2_PAYLOAD_MAX 65535
#define MSPV2_CRC_EXTEND 5

struct MSPV2
{
    uint8_t header;
    uint8_t version;
    uint8_t type;
    uint8_t flag;
    uint16_t function;
    uint16_t payloadSize;
    uint8_t payload[1];
};

struct MSPV2_CRC
{
    uint8_t header;
    uint8_t version;
    uint8_t type;
    uint8_t data[MSPV2_PAYLOAD_MAX + MSPV2_CRC_EXTEND];
};

#define MSP2_IS_SENSOR_MESSAGE(x) ((x) >= 0x1F00 && (x) <= 0x1FFF)
#define MSP2_SENSOR_RANGEFINDER 0x1F01
#define MSP2_SENSOR_OPTIC_FLOW 0x1F02
#define MSP2_SENSOR_GPS 0x1F03
#define MSP2_SENSOR_COMPASS 0x1F04
#define MSP2_SENSOR_BAROMETER 0x1F05
#define MSP2_SENSOR_AIRSPEED 0x1F06