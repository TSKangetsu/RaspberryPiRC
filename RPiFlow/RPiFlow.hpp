#pragma once
#ifdef DEBUG
#include <iostream>
#include <iomanip>
#endif
#include <vector>
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

#define CRC_START_8 0x00

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
    };

    inline int MSPDataRead(int &XOutput, int &YOutput, int &Altitude, int &DataQuality)
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
                    // step 1: get length of data
                    unsigned int len = (short)((int)TmpData[i + 7] << 8 | (int)TmpData[i + 6]);
                    if (i + len < InputFrame)
                    {
                        // step 2: copy dat
                        recvDataBuffer.clear();
                        // step 3: copy header
                        for (size_t s = 0; s < 5; s++)
                        {
                            recvDataBuffer.push_back(TmpData[i + 3 + s]);
                        }
                        // step 4: copy data
                        for (size_t s = 0; s < len; s++)
                        {
                            recvDataBuffer.push_back(TmpData[i + 3 + 5 + s]);
                        }
                        // step 5: get crc from master
                        uint8_t crcFromMaster = TmpData[i + 3 + 5 + len];
                        // step 6: caculate data CRC to compare
                        uint8_t crc8DataParse = gencrc(recvDataBuffer.data(), recvDataBuffer.size());
#ifdef DEBUG
                        std::cout << "framelen: " << InputFrame << "\n";
                        std::cout << "datalen: " << len << "\n";
                        for (size_t s = 0; s < recvDataBuffer.size(); s++)
                        {
                            std::cout << std::setw(2) << std::setfill('0') << std::hex << (int)recvDataBuffer[s] << std::dec << " ";
                        }
                        std::cout << "\ngot crc: " << std::hex << (int)crcFromMaster << std::dec << '\n';
                        std::cout << "CRC from Data: " << std::hex << (int)crc8DataParse << std::dec << "\n-\n";
#endif
                        // step 7: check crc is correct, if correct, parsing to data out
                        if (crc8DataParse == crcFromMaster)
                        {
                            if (recvDataBuffer[1] == 1)
                            {
                                Altitude = (short)((int)recvDataBuffer[5 + 2] << 8 | (int)recvDataBuffer[5 + 1]);
                                IsAltitudeFound = true;
                            }
                            if (recvDataBuffer[1] == 2)
                            {
                                DataQuality = recvDataBuffer[5];
                                YOutput = (short)((int)recvDataBuffer[5 + 2] << 8 | (int)recvDataBuffer[5 + 1]);
                                XOutput = (short)((int)recvDataBuffer[5 + 6] << 8 | (int)recvDataBuffer[5 + 5]);
                                IsMovingFound = true;
                            }
                        }
                        else
                        {
#ifdef DEBUG
                            std::cout << "data CRC mismatch!";
#endif
                        }
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
        return Status;
    };

    inline ~MSPUartFlow()
    {
        close(MSPUart_fd);
    };

private:
    fd_set fd_Maker;
    int MSPUart_fd;
    std::string FlowDevice;
    std::vector<uint8_t> recvDataBuffer;

    uint8_t gencrc(uint8_t *data, size_t len)
    {
        uint8_t crc = 0x00;
        size_t i, j;
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