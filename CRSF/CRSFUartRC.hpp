#pragma once
#ifdef DEBUG
#include <iomanip>
#include <iostream>
#endif
#include <fcntl.h>
#include <string>
#include <sstream>
#include <unistd.h>
#include <sys/ioctl.h>
#include <asm-generic/ioctls.h>
#include <sys/types.h>
#include <stdint.h>
#include <string.h>
#include <linux/i2c-dev.h>
#define termios asmtermios
#include <asm/termbits.h>
#undef termios
#include <termios.h>

#include "CRSFProtocol.hpp"

#define CRSF_MAX_READ_SIZE 500
#define CRSF_DEFAULT_BANDRATE 420000

#define M_PIf 3.14159265358979323846f
#define M_LN2f 0.69314718055994530942f
#define M_Ef 2.71828182845904523536f

#define RAD (M_PIf / 180.0f)

#define CRSF_HEADER_SIZE 4

namespace CRSFTelemetry
{
    /*
    0x08 Battery sensor
    Payload:
    uint16_t    Voltage ( mV * 100 )
    uint16_t    Current ( mA * 100 )
    uint24_t    Capacity ( mAh )
    uint8_t     Battery remaining ( percent )
    */
    inline crsfProtocol::crsfFrameDef_t crsfFrameBatterySensor(uint8_t address,
                                                               uint16_t Voltage,
                                                               uint16_t Current,
                                                               uint32_t Capacity,
                                                               uint8_t BatteryInpercent)
    {
        crsfProtocol::crsfFrameDef_t frame;

        frame.deviceAddress = address;
        frame.frameLength =
            crsfProtocol::CRSF_FRAME_BATTERY_SENSOR_PAYLOAD_SIZE;
        frame.type = crsfProtocol::CRSF_FRAMETYPE_BATTERY_SENSOR;

        frame.payload[0] = (uint8_t)(Voltage >> 8);
        frame.payload[1] = (uint8_t)(Voltage);
        frame.payload[2] = (uint8_t)(Current >> 8);
        frame.payload[3] = (uint8_t)(Current);
        frame.payload[4] = (uint8_t)(Capacity >> 16);
        frame.payload[5] = (uint8_t)(Capacity >> 8);
        frame.payload[6] = (uint8_t)(Capacity);
        frame.payload[7] = (uint8_t)(BatteryInpercent);

        return frame;
    }

    inline static int16_t _decidegrees2Radians10000(int16_t angle_decidegree)
    {
        while (angle_decidegree > 1800)
        {
            angle_decidegree -= 3600;
        }
        while (angle_decidegree < -1800)
        {
            angle_decidegree += 3600;
        }
        return (int16_t)(RAD * 1000.0f * angle_decidegree);
    }

    /*
    0x1E Attitude
    Payload:
    int16_t     Pitch angle * 10
    int16_t     Roll angle * 10
    int16_t     Yaw angle * 10
    */
    inline crsfProtocol::crsfFrameDef_t crsfFrameAttitude(uint8_t address,
                                                          uint16_t pitch,
                                                          uint16_t roll,
                                                          uint16_t yaw)
    {
        crsfProtocol::crsfFrameDef_t frame;

        frame.deviceAddress = address;
        frame.frameLength =
            crsfProtocol::CRSF_FRAME_ATTITUDE_PAYLOAD_SIZE;
        frame.type = crsfProtocol::CRSF_FRAMETYPE_ATTITUDE;

        // int16_t     Pitch angle ( rad / 10000 )
        // int16_t     Roll angle ( rad / 10000 )
        // int16_t     Yaw angle ( rad / 10000 )
        uint16_t pitchRad = _decidegrees2Radians10000(pitch);
        frame.payload[0] = (uint8_t)(pitchRad >> 8);
        frame.payload[1] = (uint8_t)pitchRad;
        uint16_t rollRad = _decidegrees2Radians10000(roll);
        frame.payload[2] = (uint8_t)(rollRad >> 8);
        frame.payload[3] = (uint8_t)rollRad;
        uint16_t yawRad = _decidegrees2Radians10000(yaw);
        frame.payload[4] = (uint8_t)(yawRad >> 8);
        frame.payload[5] = (uint8_t)yawRad;

        return frame;
    }

    /*
    0x21 Flight mode text based
    Payload:
    char[]      Flight mode ( Null­terminated string )
    */
    inline crsfProtocol::crsfFrameDef_t crsfFrameFlightMode(uint8_t address, const char *flightMode)
    {
        crsfProtocol::crsfFrameDef_t frame;

        frame.deviceAddress = address;
        frame.frameLength = strlen(flightMode) + 1;
        frame.type = crsfProtocol::CRSF_FRAMETYPE_FLIGHT_MODE;
        std::copy(flightMode, flightMode + strlen(flightMode) + 1, frame.payload);
        return frame;
    }

    /*
    0x02 GPS
    Payload:
    int32_t     Latitude ( degree / 10`000`000 )
    int32_t     Longitude (degree / 10`000`000 )
    uint16_t    Groundspeed ( km/h / 10 )
    uint16_t    GPS heading ( degree / 100 )
    uint16_t      Altitude ( meter ­1000m offset )
    uint8_t     Satellites in use ( counter )
    FIXME: uint ro int ,but don't use int, will cause header loss
    */
    inline crsfProtocol::crsfFrameDef_t crsfFrameGps(uint8_t address,
                                                     uint32_t Latitude,
                                                     uint32_t Longitude,
                                                     uint16_t Groundspeed,
                                                     uint16_t GPSHeading,
                                                     uint16_t Altitude,
                                                     uint8_t SatellitesCount)
    {
        crsfProtocol::crsfFrameDef_t frame;

        frame.deviceAddress = address;
        frame.frameLength =
            crsfProtocol::CRSF_FRAME_GPS_PAYLOAD_SIZE;
        frame.type = crsfProtocol::CRSF_FRAMETYPE_GPS;

        frame.payload[0] = (uint8_t)(Latitude >> 24);
        frame.payload[1] = (uint8_t)(Latitude >> 16);
        frame.payload[2] = (uint8_t)(Latitude >> 8);
        frame.payload[3] = (uint8_t)(Latitude);

        frame.payload[4] = (uint8_t)(Longitude >> 24);
        frame.payload[5] = (uint8_t)(Longitude >> 16);
        frame.payload[6] = (uint8_t)(Longitude >> 8);
        frame.payload[7] = (uint8_t)(Longitude);

        frame.payload[8] = (uint8_t)(Groundspeed >> 8);
        frame.payload[9] = (uint8_t)(Groundspeed);

        frame.payload[10] = (uint8_t)(GPSHeading >> 8);
        frame.payload[11] = (uint8_t)(GPSHeading);

        frame.payload[12] = (uint8_t)(Altitude >> 8);
        frame.payload[13] = (uint8_t)(Altitude);

        frame.payload[14] = (uint8_t)(SatellitesCount);

        return frame;
    }

    /*
    0x07 Vario sensor
    Payload:
    int16      Vertical speed ( cm/s )
    */
    inline crsfProtocol::crsfFrameDef_t crsfFrameVarioSensor(uint8_t address, uint16_t VerticalSpeed)
    {
        crsfProtocol::crsfFrameDef_t frame;

        frame.deviceAddress = address;
        frame.frameLength =
            crsfProtocol::CRSF_FRAMETYPE_VARIO_SENSOR;
        frame.type = crsfProtocol::CRSF_FRAME_VARIO_SENSOR_PAYLOAD_SIZE;

        frame.payload[0] = (uint8_t)VerticalSpeed >> 8;
        frame.payload[1] = (uint8_t)VerticalSpeed;

        return frame;
    }
};

class CRSF
{
public:
    inline CRSF(const char *UartDevice, int bandrate = CRSF_DEFAULT_BANDRATE)
    {
        lose_frameCount = 0;
        InputBuffer = 0;
        rcChannelsFrame = {1000};
        dataBuffer = new uint8_t[CRSF_MAX_READ_SIZE];
        //
        CRSFUart_fd = open(UartDevice, O_RDWR | O_CLOEXEC | O_NONBLOCK);
        if (CRSFUart_fd == -1)
            throw std::invalid_argument("[UART] CRSF Unable to open device:" + std::string(UartDevice));

        struct termios2 options;

        if (0 != ioctl(CRSFUart_fd, TCGETS2, &options))
        {
            close(CRSFUart_fd);
            CRSFUart_fd = -1;
        }

        // options.c_cflag = B460800 | CS8 | CLOCAL | CREAD;
        // options.c_iflag = 0;
        // options.c_oflag = 0;
        // options.c_lflag = 0;

        // usleep(200 * 1000);

        // seens to must set BBAUD and ~CBAUD, and set i/o/l parament, then kernel can find out baudrate
        options.c_cflag = B460800 | CS8 | CLOCAL | CREAD;
        options.c_cflag &= ~CBAUD;
        options.c_cflag |= BOTHER;
        options.c_iflag = 0;
        options.c_oflag = 0;
        options.c_lflag = 0;
        options.c_ispeed = bandrate;
        options.c_ospeed = bandrate;

        if (0 != ioctl(CRSFUart_fd, TCSETS2, &options))
        {
            close(CRSFUart_fd);
            CRSFUart_fd = -1;
            throw std::invalid_argument("[UART] CRSF init failed");
        }
    };

    inline int CRSFRead(int *channelsData, int timeout = 100000)
    {
        int ret = -1;
        if (CRSFUart_fd == -1)
            return -1;
        //
        FD_ZERO(&fd_Maker);
        FD_SET(CRSFUart_fd, &fd_Maker);
        lose_frameCount = 0;
        //
        timeval timecl;
        timecl.tv_sec = 0;
        timecl.tv_usec = timeout;
        int err = select(CRSFUart_fd + 1, &fd_Maker, NULL, NULL, &timecl);
        //
        InputBuffer = read(CRSFUart_fd, dataBuffer, CRSF_MAX_READ_SIZE - 2);
        if (InputBuffer > 3)
        {
            ret = CRSFParser(dataBuffer, InputBuffer, channelsData);
            //
            // std::cout << "size:" << InputBuffer << "\n";
            // for (size_t i = 0; i < InputBuffer; i++)
            // {
            //     std::cout << std::setw(2) << std::setfill('0')
            //               << std::hex << (int)dataBuffer[i] << std::dec << " ";
            // }
            // std::cout << "\n";
            //
        }
        return ret;
    };

    int CRSFParser(uint8_t *data, int size, int channelsOut[15])
    {
        const crsfProtocol::crsfFrame_t *hdr = (crsfProtocol::crsfFrame_t *)data;
        if (hdr->frame.deviceAddress == crsfProtocol::CRSF_ADDRESS_FLIGHT_CONTROLLER)
        {
            // FIXME: tardiction problem
            if (hdr->frame.frameLength < crsfProtocol::CRSF_FRAME_SIZE_MAX)
            {
                uint8_t crc = gencrc((uint8_t *)(hdr->frame.payload), hdr->frame.frameLength - 2, hdr->frame.type);
                // std::cout << std::dec << "framesize: " << (int)hdr->frame.frameLength
                //           << std::hex << " check crc: 0x" << (int)crc
                //           << " == 0x" << (int)hdr->frame.payload[hdr->frame.frameLength - 2]
                //           << std::dec << "\n";
                // for (size_t i = 0; i < hdr->frame.frameLength; i++)
                // {
                //     std::cout << std::setw(2) << std::setfill('0')
                //               << std::hex << (int)hdr->frame.payload[i] << std::dec << " ";
                // }
                // std::cout << '\n';

                if (crc == hdr->frame.payload[hdr->frame.frameLength - 2])
                {
                    // std::cout << "device addr: " << std::hex << (int)hdr->frame.deviceAddress << std::dec << "\n";
                    //
                    switch (hdr->frame.type)
                    {
                    case crsfProtocol::CRSF_FRAMETYPE_GPS:
                        // packetGps(hdr);
                        return crsfProtocol::CRSF_FRAMETYPE_GPS;
                    case crsfProtocol::CRSF_FRAMETYPE_RC_CHANNELS_PACKED:
                        packetChannelsPacked(hdr, channelsOut);
                        return crsfProtocol::CRSF_FRAMETYPE_RC_CHANNELS_PACKED;
                    case crsfProtocol::CRSF_FRAMETYPE_LINK_STATISTICS:
                        // packetLinkStatistics(hdr);
                        return crsfProtocol::CRSF_FRAMETYPE_LINK_STATISTICS;
                    }
                }
            }
        } //

        return -1;
    };

    inline uint16_t rcToUs(uint16_t rc)
    {
        return (uint16_t)((rc * 0.62477120195241F) + 881);
    };

    inline ~CRSF()
    {
        close(CRSFUart_fd);
        delete dataBuffer;
    };

    inline int CRSFTelemtry(crsfProtocol::crsfFrameDef_t TelemetryData)
    {
        crsfProtocol::crsfFrame_t frameout;

        uint8_t crc = gencrc((uint8_t *)TelemetryData.payload,
                             TelemetryData.frameLength,
                             TelemetryData.type);
        TelemetryData.payload[TelemetryData.frameLength] = crc;

        TelemetryData.frameLength += crsfProtocol::CRSF_FRAME_LENGTH_TYPE_CRC;

        frameout.frame = TelemetryData;

        int ret = write(CRSFUart_fd, frameout.bytes, TelemetryData.frameLength + CRSF_HEADER_SIZE);

        // std::cout << ret << " " << TelemetryData.frameLength + CRSF_HEADER_SIZE << '\n';

        if (ret == TelemetryData.frameLength + CRSF_HEADER_SIZE)
            return 0;
        return 1;

        // for (size_t i = 0; i < TelemetryData.frameLength + 4; i++)
        // {
        //     std::cout << std::setw(2) << std::setfill('0')
        //               << std::hex << (int)frameout.bytes[i] << std::dec << " ";
        // }
        // std::cout << '\n';

        // std::cout << "crsf tel:" << ret << std::hex << " crc: " << (int)crc << std::dec << "\n";
    }

private:
    fd_set fd_Maker;
    //
    int CRSFUart_fd;
    std::string CRSFDevice;
    //
    int InputBuffer;
    int lose_frameCount;
    uint8_t *dataBuffer;
    //
    crsfProtocol::crsfPayloadRcChannelsPacked_s rcChannelsFrame;
    //
    void packetChannelsPacked(const crsfProtocol::crsfFrame_t *p, int _channels[15])
    {
        crsfProtocol::crsfPayloadRcChannelsPacked_s *ch =
            (crsfProtocol::crsfPayloadRcChannelsPacked_s *)&p->frame.payload;
        _channels[0] = ch->chan0;
        _channels[1] = ch->chan1;
        _channels[2] = ch->chan2;
        _channels[3] = ch->chan3;
        _channels[4] = ch->chan4;
        _channels[5] = ch->chan5;
        _channels[6] = ch->chan6;
        _channels[7] = ch->chan7;
        _channels[8] = ch->chan8;
        _channels[9] = ch->chan9;
        _channels[10] = ch->chan10;
        _channels[11] = ch->chan11;
        _channels[12] = ch->chan12;
        _channels[13] = ch->chan13;
        _channels[14] = ch->chan14;
        _channels[15] = ch->chan15;
    }

    uint8_t gencrc(uint8_t *data, size_t len, uint8_t type)
    {
        size_t i, j;
        uint8_t crc = 0x00;
        // must check type at first, and skip
        crc ^= type;
        for (j = 0; j < 8; j++)
        {
            if ((crc & 0x80) != 0)
                crc = (uint8_t)((crc << 1) ^ 0xd5);
            else
                crc <<= 1;
        }
        //
        for (i = 0; i < len; i++)
        {
            crc ^= data[i];
            for (j = 0; j < 8; j++)
            {
                if ((crc & 0x80) != 0)
                    crc = (uint8_t)((crc << 1) ^ 0xd5);
                else
                    crc <<= 1;
            }
        }
        return crc;
    }
};
