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
#include <asm-generic/ioctls.h>
#include <sys/types.h>
#include <stdint.h>
#include <string.h>
#include <linux/i2c-dev.h>
#define termios asmtermios
#include <asm/termbits.h>
#undef termios
#include <termios.h>

#include "../MSP/MSP.h"
#include "../MSP/MSPSensor.h"

#define CRC_START_8 0x00

class MSPUartFlow
{
public:
    inline MSPUartFlow(const char *UartDevice)
    {
        FlowDevice = UartDevice;
        MSPUart_fd = open(UartDevice, O_RDWR | O_NONBLOCK | O_CLOEXEC);

        // TODO: better expection expected
        if (MSPUart_fd == -1)
            throw std::invalid_argument("[UART] FLOW Unable to open device:" + std::string(UartDevice));

        struct termios2 options;

        if (0 != ioctl(MSPUart_fd, TCGETS2, &options))
        {
            close(MSPUart_fd);
            MSPUart_fd = -1;
        }
        options.c_cflag = B115200 | CS8 | CLOCAL | CREAD;
        options.c_iflag = 0;
        options.c_oflag = 0;
        options.c_lflag = 0;
        if (0 != ioctl(MSPUart_fd, TCSETS2, &options))
        {
            close(MSPUart_fd);
            MSPUart_fd = -1;
            throw std::invalid_argument("[UART] FLOW init failed");
        }

        inputData.reset(new uint8_t[MSPV2_PAYLOAD_MAX]);
    };

    inline int MSPDataRead(int &XOutput, int &YOutput, int &OPQuality,
                           int &AltitudeMm, int &RFQuality,
                           int timeout = 1000000)
    {
        int ret = -1;
        if (MSPUart_fd == -1)
            return -1;
        //
        FD_ZERO(&fd_Maker);
        FD_SET(MSPUart_fd, &fd_Maker);
        //
        timeval timecl;
        timecl.tv_sec = 0;
        timecl.tv_usec = timeout;
        int err = select(MSPUart_fd + 1, &fd_Maker, NULL, NULL, &timecl);
        //
        int InputFrame = read(MSPUart_fd, inputData.get(), MSPV2_PAYLOAD_MAX);
        if (InputFrame > 8 && inputData) // this size must to length offset
        {
            MSPV2 *mspData = (MSPV2 *)inputData.get();
            MSPV2_CRC *mspDataCRC = (MSPV2_CRC *)inputData.get();
            // length size alway cause problem
            if ((mspData->payloadSize + MSPV2_CRC_EXTEND) < (sizeof(mspDataCRC->data) / sizeof(mspDataCRC->data[0])))
            {
                uint8_t crcget = gencrc(mspDataCRC->data, mspData->payloadSize + MSPV2_CRC_EXTEND);
                // std::cout << "[UART] check msp crc:" << std::hex
                //           << (int)crcget
                //           << " "
                //           << (int)mspData->payload[mspData->payloadSize]
                //           << std::dec << "\n";

                // std::cout << "[UART] check msp data: "
                //           << mspData->header << " "
                //           << mspData->version << " "
                //           << mspData->type << " "
                //           << std::hex
                //           << (int)mspData->flag << " "
                //           << (int)mspData->function << " "
                //           << (int)mspData->payloadSize << " "
                //           << std::dec
                //           << "\n";

                if (crcget == mspData->payload[mspData->payloadSize])
                {
                    // std::cout << "[UART] check MSP raw: " << std::hex;
                    // for (size_t i = 0; i < mspData->payloadSize; i++)
                    // {
                    //     std::cout << (int)mspData->payload[i] << " ";
                    // }
                    // std::cout << std::dec << '\n';

                    if (mspData->header == '$' && mspData->version == 'X' && mspData->type == '<')
                    {
                        if (mspData->function == MSP2_SENSOR_RANGEFINDER)
                        {
                            mspSensorRangefinderDataMessage_t *rfdata =
                                (mspSensorRangefinderDataMessage_t *)mspData->payload;
                            AltitudeMm = rfdata->distanceMm;
                            RFQuality = rfdata->quality;
                            return 1;
                        }
                        //
                        if (mspData->function == MSP2_SENSOR_OPTIC_FLOW)
                        {
                            mspSensorOpflowDataMessage_t *opdata =
                                (mspSensorOpflowDataMessage_t *)mspData->payload;
                            XOutput = opdata->motionX;
                            YOutput = opdata->motionY;
                            OPQuality = opdata->quality;
                            return 2;
                        }
                    }
                }
            }
        }
        return ret;
    };
    
    inline ~MSPUartFlow()
    {
        inputData.reset();
        close(MSPUart_fd);
    };

private:
    fd_set fd_Maker;
    int MSPUart_fd;
    std::string FlowDevice;
    std::unique_ptr<uint8_t> inputData;

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