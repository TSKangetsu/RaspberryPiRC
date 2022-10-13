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
#include <sys/types.h>
#include <stdint.h>
#include <string.h>
#include <linux/i2c-dev.h>
#define termios asmtermios
#include <asm/termbits.h>
#undef termios
#include <termios.h>

class CRSFCRSFUartRC
{
public:
    inline CRSFCRSFUartRC(const char *UartDevice, int bandrate = 9600)
    {
        CRSFUart_fd = open(UartDevice, O_RDWR | O_NONBLOCK | O_CLOEXEC);
        if (CRSFUart_fd == -1)
        {
#ifdef DEBUG
            std::cout << "SbusDeviceError\n";
#endif
            throw std::string("SbusDeviceError");
        }

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
        }
    };

    inline int CRFSRead(int *channelsData, int waitTime, int lose_HoldTime)
    {
        if (CRSFUart_fd == -1)
            return -1;

        FD_ZERO(&fd_Maker);
        FD_SET(CRSFUart_fd, &fd_Maker);
        lose_frameCount = 0;
        while (true)
        {
            InputBuffer = read(CRSFUart_fd, &dataBuffer, sizeof(dataBuffer));
            if (InputBuffer > 0)
            {
                std::cout << "size:" << InputBuffer << "\n";
                for (size_t i = 0; i < InputBuffer; i++)
                {
                    std::cout << std::hex << (int)dataBuffer[i] << std::dec << " ";
                }
                std::cout << "\n";
            }
            usleep(1500);
        }
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
};