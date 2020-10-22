#pragma once
#ifdef DEBUG
#include <iostream>
#endif
#include <wiringSerial.h>
#include <fcntl.h>
#include <string>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <stdint.h>
#include <string.h>
#include <asm-generic/termbits.h>

class GPSUart
{
public:
    inline GPSUart(const char *UartDevice)
    {
        GPSUart_fd = open(UartDevice, O_RDWR | O_NONBLOCK | O_CLOEXEC);
        if (GPSUart_fd == -1)
        {
#ifdef DEBUG
            std::cout << "GPSDeviceError\n";
#endif
            throw std::string("GPSDeviceError");
        }

        struct termios2 options;

        if (0 != ioctl(GPSUart_fd, TCGETS2, &options))
        {
            close(GPSUart_fd);
            GPSUart_fd = -1;
        }
        options.c_cflag = B9600 | CS8 | CLOCAL | CREAD;
        options.c_iflag = IGNPAR;
        options.c_oflag = 0;
        options.c_lflag = 0;
        if (0 != ioctl(GPSUart_fd, TCSETS2, &options))
        {
            close(GPSUart_fd);
            GPSUart_fd = -1;
        }

        if (write(GPSUart_fd, GPSDisableGPGSVConfig, sizeof(GPSDisableGPGSVConfig)) == -1)
        {
#ifdef DEBUG
            std::cout << "GPSWriteConfigError\n";
#endif
            throw std::string("GPSWriteConfigError");
        }
        else
        {
            usleep(50000);
            if (write(GPSUart_fd, GPS5HzConfig, sizeof(GPS5HzConfig)) == -1)
            {
#ifdef DEBUG
                std::cout << "GPSWriteConfig5HZError\n";
#endif
                throw std::string("GPSWriteConfig5HZError");
            }
            else
            {
                usleep(50000);
                if (write(GPSUart_fd, Set_to_57kbps, sizeof(Set_to_57kbps)) == -1)
                {
#ifdef DEBUG
                    std::cout << "GPSWriteConfig57Error\n";
#endif
                    throw std::string("GPSWriteConfig57Error");
                }
                else
                {
                    usleep(50000);
                    close(GPSUart_fd);
                    GPSUart_fd = -1;
                    //reopen for 57kbps
                    usleep(50000);
                    GPSUart_fd = open(UartDevice, O_RDWR | O_NONBLOCK | O_CLOEXEC);
                    if (GPSUart_fd == -1)
                    {
#ifdef DEBUG
                        std::cout << "GPS57DeviceError\n";
#endif
                        throw std::string("GPS57DeviceError");
                    }
                    struct termios2 options_57;

                    if (0 != ioctl(GPSUart_fd, TCGETS2, &options_57))
                    {
                        close(GPSUart_fd);
                        GPSUart_fd = -1;
                    }
                    options_57.c_cflag = B57600 | CS8 | CLOCAL | CREAD;
                    options_57.c_iflag = IGNPAR;
                    options_57.c_oflag = 0;
                    options_57.c_lflag = 0;
                    if (0 != ioctl(GPSUart_fd, TCSETS2, &options_57))
                    {
                        close(GPSUart_fd);
                        GPSUart_fd = -1;
                    }
                    ioctl(GPSUart_fd, TCFLSH, 0);
                }
            }
        };
    }

    inline int GPSRead(std::string &GPSData)
    {
        if (GPSUart_fd == -1)
            return -1;
        ioctl(GPSUart_fd, TCIFLUSH, 0);
        usleep(2000);
        FD_ZERO(&fd_Maker);
        FD_SET(GPSUart_fd, &fd_Maker);
        lose_frameCount = 0;
        GPSData = "";
        char Header[5];
        while (true)
        {
            if (read(GPSUart_fd, &GPSSingleData, sizeof(GPSSingleData)) != -1)
            {
                if (GPSSingleData == '$')
                {
                    for (size_t i = 0; i < 99; i++)
                    {
                        if (read(GPSUart_fd, &GPSSingleData, sizeof(GPSSingleData)) != -1)
                        {
                            if (i < 5)
                            {
                                Header[i] = GPSSingleData;
                            }
                            if (strncmp(Header, "GNTXT", 5) == 0)
                            {
                                if (read(GPSUart_fd, &GPSSingleData, sizeof(GPSSingleData)) != -1)
                                {
                                    if (GPSSingleData == '*')
                                    {
                                        GPSData = "";
                                        break;
                                    }
                                }
                                else
                                    usleep(800);
                            }
                            else
                            {
                                GPSData += GPSSingleData;
                                if (GPSSingleData == '*')
                                {
                                    return 1;
                                }
                            }
                        }
                        else
                        {
                            i--;
                            usleep(800);
                        }
                    }
                }
            }
            usleep(1000);
        }
    };

private:
    int GPSUart_fd;
    char GPSSingleData;
    bool GNRMCComfirm = false;
    uint8_t GPSDisableGPGSVConfig[11] = {0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x03, 0x00, 0xFD, 0x15};
    uint8_t GPS5HzConfig[14] = {0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0xC8, 0x00, 0x01, 0x00, 0x01, 0x00, 0xDE, 0x6A};
    uint8_t Set_to_57kbps[28] = {0xB5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x01, 0x00, 0x00, 0x00, 0xD0, 0x08, 0x00, 0x00,
                                 0x00, 0xE1, 0x00, 0x00, 0x07, 0x00, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0xE2, 0xE1};
    int lose_frameCount;
    fd_set fd_Maker;
};
