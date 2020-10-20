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
#include <asm-generic/termbits.h>

class GPSUart
{
public:
    inline GPSUart(const char *UartDevice)
    {
        GPSUart_fd = serialOpen(UartDevice, 9600);
        usleep(50000);
        serialPuts(GPSUart_fd, (char *)GPSDisableGPGSVConfig);
        usleep(50000);
        serialPuts(GPSUart_fd, (char *)GPS5HzConfig);
        usleep(50000);
        serialPuts(GPSUart_fd, (char *)GPS57kbpsConfig);
        usleep(50000);
        serialClose(GPSUart_fd);
        usleep(50000);
        GPSUart_fd = serialOpen(UartDevice, 57600);
    }

    inline void GPSRead(char GPSData[5][100])
    {
        while (true)
        {
            if (serialDataAvail(GPSUart_fd) > -1)
            {
                if (serialGetchar(GPSUart_fd) == '$')
                {
                    int e = 0;
                    for (size_t i = 0; i <= 99; i++)
                    {
                        GPSData[e][i] = serialGetchar(GPSUart_fd);
                        if (GPSData[e][i] == '\n')
                        {
                            e++;
                            if (e == 4)
                            {
                                break;
                            }
                            i = 0;
                        }
                    }
                    break;
                }
            }
        }
    }

private:
    int GPSUart_fd;
    uint8_t GPSDisableGPGSVConfig[11] = {0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x03, 0x00, 0xFD, 0x15};
    uint8_t GPS5HzConfig[14] = {0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0xC8, 0x00, 0x01, 0x00, 0x01, 0x00, 0xDE, 0x6A};
    uint8_t GPS57kbpsConfig[28] = {0xB5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x01, 0x00, 0x00, 0x00, 0xD0, 0x08, 0x00, 0x00,
                                   0x00, 0xE1, 0x00, 0x00, 0x07, 0x00, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0xE2, 0xE1};
    //===================================================================================================//
    // public:
    //     inline GPSUart(const char *UartDevice)
    //     {
    //         GPSUart_fd = open(UartDevice, O_RDWR | O_NONBLOCK | O_CLOEXEC);
    //         if (GPSUart_fd == -1)
    //         {
    // #ifdef DEBUG
    //             std::cout << "GPSDeviceError\n";
    // #endif
    //             throw std::string("GPSDeviceError");
    //         }

    //         struct termios2 options;

    //         if (0 != ioctl(GPSUart_fd, TCGETS2, &options))
    //         {
    //             close(GPSUart_fd);
    //             GPSUart_fd = -1;
    //         }
    //         options.c_cflag = B9600 | CS8 | CLOCAL | CREAD;
    //         options.c_iflag = IGNPAR;
    //         options.c_oflag = 0;
    //         options.c_lflag = 0;
    //         if (0 != ioctl(GPSUart_fd, TCSETS2, &options))
    //         {
    //             close(GPSUart_fd);
    //             GPSUart_fd = -1;
    //         }

    //         if (write(GPSUart_fd, GPSDisableGPGSVConfig, sizeof(GPSDisableGPGSVConfig)) == -1)
    //         {
    // #ifdef DEBUG
    //             std::cout << "GPSWriteConfigError\n";
    // #endif
    //             throw std::string("GPSWriteConfigError");
    //         }
    //         else
    //         {
    //             usleep(50000);
    //             if (write(GPSUart_fd, GPS5HzConfig, sizeof(GPS5HzConfig)) == -1)
    //             {
    // #ifdef DEBUG
    //                 std::cout << "GPSWriteConfig5HZError\n";
    // #endif
    //                 throw std::string("GPSWriteConfig5HZError");
    //             }
    //             else
    //             {
    //                 usleep(50000);
    //                 if (write(GPSUart_fd, Set_to_57kbps, sizeof(Set_to_57kbps)) == -1)
    //                 {
    // #ifdef DEBUG
    //                     std::cout << "GPSWriteConfig57Error\n";
    // #endif
    //                     throw std::string("GPSWriteConfig57Error");
    //                 }
    //                 else
    //                 {
    //                     usleep(50000);
    //                     close(GPSUart_fd);
    //                     GPSUart_fd = -1;
    //                     //reopen for 57kbps
    //                     usleep(50000);
    //                     GPSUart_fd = open(UartDevice, O_RDWR | O_NONBLOCK | O_CLOEXEC);
    //                     if (GPSUart_fd == -1)
    //                     {
    // #ifdef DEBUG
    //                         std::cout << "GPS57DeviceError\n";
    // #endif
    //                         throw std::string("GPS57DeviceError");
    //                     }
    //                     struct termios2 options_57;

    //                     if (0 != ioctl(GPSUart_fd, TCGETS2, &options_57))
    //                     {
    //                         close(GPSUart_fd);
    //                         GPSUart_fd = -1;
    //                     }
    //                     options_57.c_cflag = B57600 | CS8 | CLOCAL | CREAD;
    //                     options_57.c_iflag = IGNPAR;
    //                     options_57.c_oflag = 0;
    //                     options_57.c_lflag = 0;
    //                     if (0 != ioctl(GPSUart_fd, TCSETS2, &options_57))
    //                     {
    //                         close(GPSUart_fd);
    //                         GPSUart_fd = -1;
    //                     }
    //                 }
    //             }
    //         };
    //     }

    //     inline int GPSRead(uint8_t GPSDatas[99], int waitTime, int lose_HoldTime)
    //     {
    //         if (GPSUart_fd == -1)
    //             return -1;
    //         FD_ZERO(&fd_Maker);
    //         FD_SET(GPSUart_fd, &fd_Maker);
    //         lose_frameCount = 0;
    //         while (true)
    //         {
    //             InputBuffer = read(GPSUart_fd, &GPSData, sizeof(GPSData));
    //             if (InputBuffer == 99)
    //             {
    //                 if (GPSData[0] == '$')
    //                     break;
    //             }
    //             lose_frameCount += 1;
    //             if (lose_frameCount == lose_HoldTime)
    //                 return -1;
    //             usleep(waitTime);
    //         }

    //         for (size_t i = 0; i < 99; i++)
    //         {
    //             GPSDatas[i] = GPSData[i];
    //         }
    //     }

    // private:
    //     int GPSUart_fd;
    //     uint8_t GPSSingleData;
    //     uint8_t GPSDisableGPGSVConfig[11] = {0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x03, 0x00, 0xFD, 0x15};
    //     uint8_t GPS5HzConfig[14] = {0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0xC8, 0x00, 0x01, 0x00, 0x01, 0x00, 0xDE, 0x6A};
    //     uint8_t Set_to_57kbps[28] = {0xB5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x01, 0x00, 0x00, 0x00, 0xD0, 0x08, 0x00, 0x00,
    //                                  0x00, 0xE1, 0x00, 0x00, 0x07, 0x00, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0xE2, 0xE1};
    //     int InputBuffer;
    //     int lose_frameCount;
    //     uint8_t GPSData[99];
    //     fd_set fd_Maker;
};