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

#define CRSF_DEFAULT_BANDRATE 420000

class CRSF
{
public:
    inline CRSF(const char *UartDevice, int bandrate = CRSF_DEFAULT_BANDRATE)
    {
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
        InputBuffer = read(CRSFUart_fd, &dataBuffer, sizeof(dataBuffer));
        if (InputBuffer > 0)
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
        const crsfProtocol::frame_t *hdr = (crsfProtocol::frame_t *)data;
        if (hdr->frame.deviceAddress == crsfProtocol::CRSF_ADDRESS_FLIGHT_CONTROLLER)
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
        } //

        return -1;
    }

    inline uint16_t rcToUs(uint16_t rc)
    {
        return (uint16_t)((rc * 0.62477120195241F) + 881);
    };

private:
    fd_set fd_Maker;
    //
    int CRSFUart_fd;
    std::string CRSFDevice;
    //
    int InputBuffer;
    int lose_frameCount;
    uint8_t dataBuffer[5000];
    //
    crsfProtocol::frame_t rcChannelsFrame;
    //
    void packetChannelsPacked(const crsfProtocol::frame_t *p, int _channels[15])
    {
        crsfProtocol::rcChannelsPacked_t *ch = (crsfProtocol::rcChannelsPacked_t *)&p->frame.payload;
        _channels[0] = ch->channel0;
        _channels[1] = ch->channel1;
        _channels[2] = ch->channel2;
        _channels[3] = ch->channel3;
        _channels[4] = ch->channel4;
        _channels[5] = ch->channel5;
        _channels[6] = ch->channel6;
        _channels[7] = ch->channel7;
        _channels[8] = ch->channel8;
        _channels[9] = ch->channel9;
        _channels[10] = ch->channel10;
        _channels[11] = ch->channel11;
        _channels[12] = ch->channel12;
        _channels[13] = ch->channel13;
        _channels[14] = ch->channel14;
        _channels[15] = ch->channel15;
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