#pragma once
#ifdef DEBUG
#include <iostream>
#endif
#include <wiringSerial.h>
#include <fcntl.h>
#include <string>
#include <sstream>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <stdint.h>
#include <string.h>
#include <linux/i2c-dev.h>
#include <wiringPiI2C.h>
#include <wiringSerial.h>
#define termios asmtermios
#include <asm/termbits.h>
#undef termios
#include <termios.h>

class MSPUartFlow
{
public:
    inline MSPUartFlow(const char *UartDevice)
    {
        FlowDevice = UartDevice;
        MSPUart_fd = open(UartDevice, O_RDWR | O_NONBLOCK | O_CLOEXEC);
        if (MSPUart_fd == -1)
        {
#ifdef DEBUG
            std::cout << "GPSDeviceError\n";
#endif
            throw std::string("GPSDeviceError");
        }

        struct termios2 options;

        if (0 != ioctl(MSPUart_fd, TCGETS2, &options))
        {
            close(MSPUart_fd);
            MSPUart_fd = -1;
        }
        options.c_cflag = B115200 | CS8 | CLOCAL | CREAD;
        options.c_iflag = BRKINT;
        options.c_oflag = 0;
        options.c_lflag = 0;
        if (0 != ioctl(MSPUart_fd, TCSETS2, &options))
        {
            close(MSPUart_fd);
            MSPUart_fd = -1;
        }
        serialFlush(MSPUart_fd);
    };

    inline int MSPDataRead(int &XOutput, int &YOutput, int &Altitude)
    {
        bool IsAltitudeFound = false;
        bool IsMovingFound = false;
        int Status = 0;
        if (MSPUart_fd == -1)
            return -1;
        FD_ZERO(&fd_Maker);
        FD_SET(MSPUart_fd, &fd_Maker);
        char TmpData[5000];
        int InputFrame;
        InputFrame = read(MSPUart_fd, &TmpData, sizeof(TmpData));
        if (InputFrame > 0)
        {
            for (size_t i = 0; i < InputFrame; i++)
            {
                if (TmpData[i] == '$')
                {
                    char Header[8];
                    for (size_t s = 0; s < 8; s++)
                    {
                        Header[s] = TmpData[i + s];
                    }
                    if (Header[4] == 1)
                    {
                        int len = (short)((int)Header[7] << 8 | (int)Header[6]);
                        Altitude = (short)((int)TmpData[i + 8 + 2] << 8 | (int)TmpData[i + 8 + 1]);
                        IsAltitudeFound = true;
                    }
                    else if (Header[4] == 2)
                    {
                        int len = (short)((int)Header[7] << 8 | (int)Header[6]);
                        YOutput = (short)((int)TmpData[i + 8 + 2] << 8 | (int)TmpData[i + 8 + 1]);
                        XOutput = (short)((int)TmpData[i + 8 + 6] << 8 | (int)TmpData[i + 8 + 5]);
                        IsMovingFound = true;
                    }
                }
            }
            if (IsMovingFound && IsAltitudeFound)
                Status = 3;
            else if (IsMovingFound && !IsAltitudeFound)
                Status = 2;
            else if (!IsMovingFound && IsAltitudeFound)
                Status = 1;
            else
                Status = -1;
        }
        else
        {
            Status = -1;
        }
        serialFlush(MSPUart_fd);
        return Status;
    };

private:
    fd_set fd_Maker;
    int MSPUart_fd;
    std::string FlowDevice;
};