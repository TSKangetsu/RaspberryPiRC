#pragma once
#ifdef DEBUG
#include <iostream>
#endif
#include <fcntl.h>
#include <string>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <stdint.h>
#define termios asmtermios
#include <asm/termbits.h>
#undef termios

class Sbus
{
public:
    inline Sbus(const char *UartDevice)
    {
        Sbus_fd = open(UartDevice, O_RDWR | O_NONBLOCK | O_CLOEXEC);
        if (Sbus_fd == -1)
        {
#ifdef DEBUG
            std::cout << "SbusDeviceError\n";
#endif
            throw std::string("SbusDeviceError");
        }

        struct termios2 options;

        if (0 != ioctl(Sbus_fd, TCGETS2, &options))
        {
            close(Sbus_fd);
            Sbus_fd = -1;
        }

        options.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
        options.c_iflag |= (INPCK | IGNPAR);
        options.c_oflag &= ~OPOST;
        options.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
        options.c_cflag &= ~(CSIZE | CRTSCTS | PARODD | CBAUD);
        options.c_cflag |= (CS8 | CSTOPB | CLOCAL | PARENB | BOTHER | CREAD);
        options.c_ispeed = 100000;
        options.c_ospeed = 100000;
        options.c_cc[VMIN] = 25;
        options.c_cc[VTIME] = 0;

        if (0 != ioctl(Sbus_fd, TCSETS2, &options))
        {
            close(Sbus_fd);
            Sbus_fd = -1;
        }
    }

    inline int SbusRead(int *channelsData, int waitTime, int lose_HoldTime)
    {
        if (Sbus_fd == -1)
            return -1;
        FD_ZERO(&fd_Maker);
        FD_SET(Sbus_fd, &fd_Maker);
        lose_frameCount = 0;
        while (true)
        {
            InputBuffer = read(Sbus_fd, &sbusData, sizeof(sbusData));
            if (InputBuffer == 25)
            {
                if (sbusData[0] == 0x0f && sbusData[24] == 0x00)
                    break;
            }
            lose_frameCount += 1;
            if (lose_frameCount == lose_HoldTime)
                return -1;
            usleep(waitTime);
        }
        SbusPaser(sbusData, ChannelsData);

        for (size_t i = 0; i < 16; i++)
        {
            channelsData[i] = (int)ChannelsData[i];
        }
        ioctl(Sbus_fd, TCIFLUSH, 0);
        return lose_frameCount;
    }

    inline int SbusQuickRead()
    {
        read(Sbus_fd, &sbusSingleData, sizeof(sbusSingleData));
        return (int)sbusSingleData;
    }

    inline void SbusPaser(uint8_t *sbusRaw, int *Channel)
    {
        Channel[0] = (uint16_t)(((sbusRaw[1] | sbusRaw[2] << 8) & 0x07FF) * sbus_scaler + .5f) + sbus_offset;
        Channel[1] = (uint16_t)(((sbusRaw[2] >> 3 | sbusRaw[3] << 5) & 0x07FF) * sbus_scaler + .5f) + sbus_offset;
        Channel[2] = (uint16_t)(((sbusRaw[3] >> 6 | sbusRaw[4] << 2 | sbusRaw[5] << 10) & 0x07FF) * sbus_scaler + .5f) + sbus_offset;
        Channel[3] = (uint16_t)(((sbusRaw[5] >> 1 | sbusRaw[6] << 7) & 0x07FF) * sbus_scaler + .5f) + sbus_offset;
        Channel[4] = (uint16_t)(((sbusRaw[6] >> 4 | sbusRaw[7] << 4) & 0x07FF) * sbus_scaler + .5f) + sbus_offset;
        Channel[5] = (uint16_t)(((sbusRaw[7] >> 7 | sbusRaw[8] << 1 | sbusRaw[9] << 9) & 0x07FF) * sbus_scaler + .5f) + sbus_offset;
        Channel[6] = (uint16_t)(((sbusRaw[9] >> 2 | sbusRaw[10] << 6) & 0x07FF) * sbus_scaler + .5f) + sbus_offset;
        Channel[7] = (uint16_t)(((sbusRaw[10] >> 5 | sbusRaw[11] << 3) & 0x07FF) * sbus_scaler + .5f) + sbus_offset;
        Channel[8] = (uint16_t)(((sbusRaw[12] | sbusRaw[13] << 8) & 0x07FF) * sbus_scaler + .5f) + sbus_offset;
        Channel[9] = (uint16_t)(((sbusRaw[13] >> 3 | sbusRaw[14] << 5) & 0x07FF) * sbus_scaler + .5f) + sbus_offset;
        Channel[10] = (uint16_t)(((sbusRaw[14] >> 6 | sbusRaw[15] << 2 | sbusRaw[16] << 10) & 0x07FF) * sbus_scaler + .5f) + sbus_offset;
        Channel[11] = (uint16_t)(((sbusRaw[16] >> 1 | sbusRaw[17] << 7) & 0x07FF) * sbus_scaler + .5f) + sbus_offset;
        Channel[12] = (uint16_t)(((sbusRaw[17] >> 4 | sbusRaw[18] << 4) & 0x07FF) * sbus_scaler + .5f) + sbus_offset;
        Channel[13] = (uint16_t)(((sbusRaw[18] >> 7 | sbusRaw[19] << 1 | sbusRaw[20] << 9) & 0x07FF) * sbus_scaler + .5f) + sbus_offset;
        Channel[14] = (uint16_t)(((sbusRaw[20] >> 2 | sbusRaw[21] << 6) & 0x07FF) * sbus_scaler + .5f) + sbus_offset;
        Channel[15] = (uint16_t)(((sbusRaw[21] >> 5 | sbusRaw[22] << 3) & 0x07FF) * sbus_scaler + .5f) + sbus_offset;
    }

private:
    int Sbus_fd;
    int InputBuffer;
    int lose_frameCount;
    fd_set fd_Maker;
    uint8_t sbusData[25];
    uint8_t sbusSingleData;
    int ChannelsData[16];
    const double sbus_scaler = (2000.0f - 1000.0f) / (1800.0f - 200.0f);
    const int sbus_offset = 1000.0f - (sbus_scaler * 200.0f + 0.5f);
};