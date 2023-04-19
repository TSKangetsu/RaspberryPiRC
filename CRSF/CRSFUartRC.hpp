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

class CRSFCRSFUartRC
{
public:
    inline CRSFCRSFUartRC(const char *UartDevice, int bandrate = 9600)
    {
        CRSFUart_fd = open(UartDevice, O_RDWR | O_CLOEXEC);
        if (CRSFUart_fd == -1)
            throw std::invalid_argument("[UART] CRSF Unable to open device:" + std::string(UartDevice));

        struct termios2 options;

        if (0 != ioctl(CRSFUart_fd, TCGETS2, &options))
        {
            close(CRSFUart_fd);
            CRSFUart_fd = -1;
        }

        //
        options.c_cflag = B460800 | CS8 | CLOCAL | CREAD;
        options.c_oflag = 0;
        options.c_lflag = 0;

        if (0 != ioctl(CRSFUart_fd, TCSETS2, &options))
        {
            close(CRSFUart_fd);
            CRSFUart_fd = -1;
            throw std::invalid_argument("[UART] CRSF init failed");
        }
    };

    inline int CRSFRead(int *channelsData, int waitTime, int lose_HoldTime)
    {
        if (CRSFUart_fd == -1)
            return -1;
        //
        // FD_ZERO(&fd_Maker);
        // FD_SET(CRSFUart_fd, &fd_Maker);
        // lose_frameCount = 0;
        //
        InputBuffer = read(CRSFUart_fd, &dataBuffer, sizeof(dataBuffer));
        if (InputBuffer > 0)
        {
            CRSFParser(dataBuffer, InputBuffer, channelsData);
            // std::cout << "size:" << InputBuffer << "\n";
            // for (size_t i = 0; i < InputBuffer; i++)
            // {
            //     std::cout << std::hex << (int)dataBuffer[i] << std::dec << " ";
            // }
            // std::cout << "\n";
        }
    }

    void CRSFParser(uint8_t *data, int size, int channelsOut[15])
    {
        const crsf_header_t *hdr = (crsf_header_t *)data;
        if (hdr->device_addr == CRSF_ADDRESS_FLIGHT_CONTROLLER)
        {
            switch (hdr->type)
            {
            case CRSF_FRAMETYPE_GPS:
                // packetGps(hdr);
                break;
            case CRSF_FRAMETYPE_RC_CHANNELS_PACKED:
                packetChannelsPacked(hdr, channelsOut);
                break;
            case CRSF_FRAMETYPE_LINK_STATISTICS:
                // packetLinkStatistics(hdr);
                break;
            }
        } //
    }

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
    typedef struct crsf_header_s
    {
        uint8_t device_addr;
        uint8_t frame_size;
        uint8_t type;
        uint8_t data[0];
    } crsf_header_t;
    //
    void packetChannelsPacked(const crsf_header_t *p, int _channels[15])
    {
        crsf_channels_t *ch = (crsf_channels_t *)&p->data;
        _channels[0] = ch->ch0;
        _channels[1] = ch->ch1;
        _channels[2] = ch->ch2;
        _channels[3] = ch->ch3;
        _channels[4] = ch->ch4;
        _channels[5] = ch->ch5;
        _channels[6] = ch->ch6;
        _channels[7] = ch->ch7;
        _channels[8] = ch->ch8;
        _channels[9] = ch->ch9;
        _channels[10] = ch->ch10;
        _channels[11] = ch->ch11;
        _channels[12] = ch->ch12;
        _channels[13] = ch->ch13;
        _channels[14] = ch->ch14;
        _channels[15] = ch->ch15;

        // for (unsigned int i = 0; i < CRSF_NUM_CHANNELS; ++i)
        // _channels[i] = std::map(_channels[i], CRSF_CHANNEL_VALUE_1000, CRSF_CHANNEL_VALUE_2000, 1000, 2000);
    }
};