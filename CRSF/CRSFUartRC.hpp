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

class CRSFCRSFUartRC
{
public:
    inline CRSFCRSFUartRC(const char *UartDevice, int bandrate = CRSF_DEFAULT_BANDRATE)
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

        // options.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
        // options.c_iflag |= (INPCK | IGNPAR);
        // options.c_oflag &= ~OPOST;
        // options.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
        // options.c_cflag &= ~(CSIZE | CRTSCTS | PARODD | CBAUD);
        // options.c_cflag |= (CS8 | CSTOPB | CLOCAL | PARENB | BOTHER | CREAD);

        // options.c_cflag |= BOTHER;
        // options.c_cflag &= ~CBAUD;
        // options.c_iflag = 0;
        // options.c_oflag = 0;
        // options.c_lflag = 0;
        // options.c_ispeed = 420000;
        // options.c_ospeed = 420000;

        // options.c_cflag = B115200 | CS8 | CLOCAL | CREAD;
        // options.c_iflag = IGNPAR;
        // options.c_oflag = 0;
        // options.c_lflag = 0;

        options.c_cflag &= ~CBAUD;
        options.c_cflag |= BOTHER;
        options.c_ispeed = bandrate;
        options.c_ospeed = bandrate;

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
        return InputBuffer;
    };

    void CRSFParser(uint8_t *data, int size, int channelsOut[15])
    {
        const crsfProtocol::frame_t *hdr = (crsfProtocol::frame_t *)data;
        if (hdr->frame.deviceAddress == crsfProtocol::CRSF_ADDRESS_FLIGHT_CONTROLLER)
        {
            switch (hdr->frame.type)
            {
            case crsfProtocol::CRSF_FRAMETYPE_GPS:
                // packetGps(hdr);
                break;
            case crsfProtocol::CRSF_FRAMETYPE_RC_CHANNELS_PACKED:
                packetChannelsPacked(hdr, channelsOut);
                break;
            case crsfProtocol::CRSF_FRAMETYPE_LINK_STATISTICS:
                // packetLinkStatistics(hdr);
                break;
            }
        } //
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
};