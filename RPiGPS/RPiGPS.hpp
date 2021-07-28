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
#include <thread>
#include <linux/i2c-dev.h>
#include <wiringPiI2C.h>
#include <wiringSerial.h>
#include <iostream>
#define termios asmtermios
#include <asm/termbits.h>
#undef termios
#include <termios.h>
#define PI 3.1415926
#define QMC5883LADDR 0x0D
#define HMC5883LADDR 0x1E
#define COMPASS_QMC5883L 0
#define COMPASS_HMC5883L 1

#define CompassYScaler 0
#define CompassZScaler 1
#define CompassXOffset 2
#define CompassYOffset 3
#define CompassZOffset 4

struct GPSUartData
{
    int satillitesCount;
    bool lat_North_Mode;
    bool lat_East_Mode;
    double lat = 0;
    double lng = 0;
    double alititude = 0;
    bool DataUnCorrect;
};

class GPSUart
{
public:
    inline GPSUart(const char *UartDevice)
    {
        GPSDevice = UartDevice;
        GPSUart_fd = open(UartDevice, O_RDWR | O_NOCTTY | O_NDELAY);
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
        options.c_iflag = 0;
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
            tcdrain(GPSUart_fd);
            tcflush(GPSUart_fd, TCOFLUSH);
            if (write(GPSUart_fd, GPS5HzConfig, sizeof(GPS5HzConfig)) == -1)
            {
#ifdef DEBUG
                std::cout << "GPSWriteConfig5HZError\n";
#endif
                throw std::string("GPSWriteConfig5HZError");
            }
            else
            {
                tcdrain(GPSUart_fd);
                tcflush(GPSUart_fd, TCOFLUSH);
                if (write(GPSUart_fd, Set_to_115kbps, sizeof(Set_to_115kbps)) == -1)
                {
#ifdef DEBUG
                    std::cout << "GPSWriteConfig115Error\n";
#endif
                    throw std::string("GPSWriteConfig115Error");
                }
                else
                {
                    tcdrain(GPSUart_fd);
                    tcflush(GPSUart_fd, TCOFLUSH);
                    close(GPSUart_fd);
                    GPSUart_fd = -1;
                }
            }
        };
    }

    inline void GPSReOpen()
    {
        if (GPSUart_fd != -1)
        {
            close(GPSUart_fd);
            GPSUart_fd = -1;
        }
        GPSUart_fd = open(GPSDevice.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
        if (GPSUart_fd == -1)
        {
#ifdef DEBUG
            std::cout << "GPS115DeviceError\n";
#endif
            throw std::string("GPS115DeviceError");
        }
        struct termios2 options_57;

        if (0 != ioctl(GPSUart_fd, TCGETS2, &options_57))
        {
            close(GPSUart_fd);
            GPSUart_fd = -1;
        }
        options_57.c_cflag = B115200 | CS8 | CLOCAL | CREAD;
        options_57.c_iflag = IGNPAR;
        options_57.c_oflag = 0;
        options_57.c_lflag = 0;
        if (0 != ioctl(GPSUart_fd, TCSETS2, &options_57))
        {
            close(GPSUart_fd);
            GPSUart_fd = -1;
        }
        tcflush(GPSUart_fd, TCIOFLUSH);
        GPSPeeker = fdopen(GPSUart_fd, "r");
    }

    inline bool GPSCheckDataAvaliable()
    {
        int c[6];
        char GNRMC[6];
        int bytes_avaiable;
        ioctl(GPSUart_fd, FIONREAD, &bytes_avaiable);
        if (bytes_avaiable > 30)
        {
            for (size_t i = 0; i < 6; i++)
            {
                c[i] = fgetc(GPSPeeker);
                GNRMC[i] = (char)c[i];
            }
            for (size_t i = 0; i < 6; i++)
            {
                ungetc(c[i], GPSPeeker);
            }
            if (strncmp(GNRMC, "$GNRMC", 6) == 0 || strncmp(GNRMC, "CMRNG$", 6) == 0)
            {
                return true;
            }
            else
            {
                GPSReOpen();
                return false;
            }
        }
        else
        {
            return false;
        }
    }

    inline int GPSRead(std::string &outputData)
    {
        outputData = "";
        if (GPSUart_fd == -1)
            return -1;
        FD_ZERO(&fd_Maker);
        FD_SET(GPSUart_fd, &fd_Maker);
        //================================================$GNRMC
        if (GPSCheckDataAvaliable())
        {
            int bytes_avaiable;
            ioctl(GPSUart_fd, FIONREAD, &bytes_avaiable);
            //================================================
            char TmpData[bytes_avaiable];
            int InputFrame;
            InputFrame = read(GPSUart_fd, &TmpData, bytes_avaiable);
            if (InputFrame > 0)
            {
                for (size_t i = 0; i < InputFrame; i++)
                {
                    outputData += TmpData[i];
                }
            }
            tcflush(GPSUart_fd, TCIOFLUSH);
            return InputFrame;
        }
        return -1;
    }

    inline GPSUartData GPSParse()
    {
        int DataCount = 0;
        int GGADataCrash = 0;
        GPSUartData myData;
        myData.DataUnCorrect = false;
        std::string GPSDataStr;
        std::string GPSDataStrError;
        std::string GPSData[255];
        std::string GPSDataSub[40];
        std::string GPSTmpData[2];
        std::string GPSDataChecker[5];

        GPSRead(GPSDataStr);
        DataCount = dataParese(GPSDataStr, GPSData, "\r\n", 255);

        int Count = 0;
        while (GPSData[Count] != std::string(";"))
        {
            if (strncmp("$GNGGA", GPSData[Count].c_str(), 5) == 0)
            {
                GGADataCrash++;
                if (GGADataCrash > 1)
                {
                    break;
                }
                dataParese(GPSData[Count], GPSDataChecker, '*', 5);
                if (GPSDataChecker[1] == std::string(""))
                {
                    myData.DataUnCorrect = true;
                }

                dataParese(GPSData[Count], GPSDataSub, ',', 40);
                std::string GPSDataTmpLat = std::to_string(std::atof(GPSDataSub[2].c_str()) / 100.0);
                dataParese(GPSDataTmpLat, GPSTmpData, '.', 2);
                myData.lat = std::atof(GPSTmpData[0].c_str()) * 10000.0;
                myData.lat += std::atof(GPSTmpData[1].c_str()) / 60.0;
                myData.lat = (int)(myData.lat * 100);

                std::string GPSDataTmpLng = std::to_string(std::atof(GPSDataSub[4].c_str()) / 100.0);
                dataParese(GPSDataTmpLng, GPSTmpData, '.', 2);
                myData.lng = std::atof(GPSTmpData[0].c_str()) * 10000.0;
                myData.lng += std::atof(GPSTmpData[1].c_str()) / 60.0;
                myData.lng = (int)(myData.lng * 100);

                if (myData.lat == 0 || myData.lng == 0)
                {
                    myData.DataUnCorrect = true;
                }

                if (strncmp(GPSDataSub[3].c_str(), "N", 1) == 0)
                    myData.lat_North_Mode = true;
                else
                    myData.lat_North_Mode = false;
                if (strncmp(GPSDataSub[5].c_str(), "E", 1) == 0)
                    myData.lat_East_Mode = true;
                else
                    myData.lat_East_Mode = false;
                myData.satillitesCount = std::atof(GPSDataSub[7].c_str());
            }
            else if (strncmp("$GNGLL", GPSData[Count].c_str(), 5) == 0)
            {
                dataParese(GPSData[Count], GPSDataSub, ',', 40);
            }
            Count++;
            GPSLastDebug = GPSDataStr;
        }
        serialFlush(GPSUart_fd);
        return myData;
    };

    inline ~GPSUart()
    {
        close(GPSUart_fd);
    }

private:
    int GPSUart_fd;
    char GPSSingleData;
    bool GNRMCComfirm = false;
    std::string GPSDevice;
    std::string GPSLastDebug;
    uint8_t GPSDisableGPGSVConfig[11] = {0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x03, 0x00, 0xFD, 0x15};
    uint8_t GPS5HzConfig[14] = {0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0xC8, 0x00, 0x01, 0x00, 0x01, 0x00, 0xDE, 0x6A};
    uint8_t Set_to_115kbps[28] = {0xB5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x01, 0x00, 0x00, 0x00, 0xD0, 0x28, 0x00, 0x00,
                                  0x00, 0xC2, 0x01, 0x00, 0x03, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0xDC, 0x3E};
    int lose_frameCount;
    fd_set fd_Maker;
    FILE *GPSPeeker;

    void dataParese(std::string data, std::string *databuff, const char splti, int MaxSize)
    {
        std::istringstream f(data);
        std::string s;
        int count = 0;
        while (getline(f, s, splti))
        {
            if (count > MaxSize)
                break;
            databuff[count] = s;
            count++;
        }
    }

    int dataParese(std::string data, std::string *databuff, std::string splti, int MaxSize)
    {
        int Count = 0;
        size_t pos = 0;
        std::string token;
        while ((pos = data.find(splti)) != std::string::npos)
        {
            if (Count > MaxSize)
                break;
            token = data.substr(0, pos);
            databuff[Count] = token;
            data.erase(0, pos + splti.length());
            Count++;
        }
        databuff[Count] = data;
        databuff[Count + 1] = ";";
        return Count;
    }
};

class GPSI2CCompass
{
public:
    GPSI2CCompass(int Compass_Type)
    {
        CompassType = Compass_Type;
        switch (CompassType)
        {
        case COMPASS_QMC5883L:
        {
            CompassFD = wiringPiI2CSetup(QMC5883LADDR);
            wiringPiI2CWriteReg16(CompassFD, 0x0A, 0x80);
            wiringPiI2CWriteReg16(CompassFD, 0x0B, 0x01);
            wiringPiI2CWriteReg16(CompassFD, 0x09, 0x0D);
        }
        break;
        case COMPASS_HMC5883L:
        {
            CompassFD = wiringPiI2CSetup(HMC5883LADDR);
            wiringPiI2CWriteReg16(CompassFD, 0x00, 0x78);
            wiringPiI2CWriteReg16(CompassFD, 0x01, 0x20);
            wiringPiI2CWriteReg16(CompassFD, 0x02, 0x00);
        }
        break;
        }
    };

    void CompassCalibration(bool Calibrating, double *CalibrationData)
    {
        if (Calibrating)
        {
            int i;
            bool finish = true;
            long compassCal[6] = {0, 0, 0, 0, 0};
            std::thread compass = std::thread(
                [&]
                {
                    long RAWX;
                    long RAWY;
                    long RAWZ;
                    double compassXOffset;
                    double compassYOffset;
                    double compassZOffset;
                    double compassYScaler;
                    double compassZScaler;
                    while (finish)
                    {
                        CompassUpdate();
                        CompassGetRaw(RAWX, RAWY, RAWZ);
                        compassCal[0] = RAWX < compassCal[0] ? RAWX : compassCal[0];
                        compassCal[1] = RAWX > compassCal[1] ? RAWX : compassCal[1];
                        compassCal[2] = RAWY < compassCal[2] ? RAWY : compassCal[2];
                        compassCal[3] = RAWY > compassCal[3] ? RAWY : compassCal[3];
                        compassCal[4] = RAWZ < compassCal[4] ? RAWZ : compassCal[4];
                        compassCal[5] = RAWZ > compassCal[5] ? RAWZ : compassCal[5];
                        usleep(10000);
                    }
                    compassYScaler = ((float)compassCal[1] - compassCal[0]) / (compassCal[3] - compassCal[2]);
                    compassZScaler = ((float)compassCal[1] - compassCal[0]) / (compassCal[5] - compassCal[4]);
                    compassXOffset = (compassCal[1] - compassCal[0]) / 2 - compassCal[1];
                    compassYOffset = (((float)compassCal[3] - compassCal[2]) / 2 - compassCal[3]) * compassYScaler;
                    compassZOffset = (((float)compassCal[5] - compassCal[4]) / 2 - compassCal[5]) * compassZScaler;

                    CalibrationData[CompassYScaler] = compassYScaler;
                    CalibrationData[CompassZScaler] = compassZScaler;
                    CalibrationData[CompassXOffset] = compassXOffset;
                    CalibrationData[CompassYOffset] = compassYOffset;
                    CalibrationData[CompassZOffset] = compassZOffset;
                });
            std::cin >> i;
            finish = false;
            compass.join();
            std::cout << "compassYScaler :" << CalibrationData[CompassYScaler] << "\n";
            std::cout << "compassZScaler :" << CalibrationData[CompassZScaler] << "\n";
            std::cout << "compassXOffset :" << CalibrationData[CompassXOffset] << "\n";
            std::cout << "compassYOffset :" << CalibrationData[CompassYOffset] << "\n";
            std::cout << "compassZOffset :" << CalibrationData[CompassZOffset] << "\n";
        }
        else
        {
            CompassCalibrationData[CompassYScaler] = CalibrationData[CompassYScaler];
            CompassCalibrationData[CompassZScaler] = CalibrationData[CompassZScaler];
            CompassCalibrationData[CompassXOffset] = CalibrationData[CompassXOffset];
            CompassCalibrationData[CompassYOffset] = CalibrationData[CompassYOffset];
            CompassCalibrationData[CompassZOffset] = CalibrationData[CompassZOffset];
        }
    }

    void CompassUpdate()
    {
        CompassRead(RawMAGIX, RawMAGIY, RawMAGIZ);
        RawMAGIX += CompassCalibrationData[CompassXOffset];
        RawMAGIX *= -1;

        RawMAGIY += CompassCalibrationData[CompassYOffset];
        RawMAGIY *= -1 * CompassCalibrationData[CompassYScaler];

        RawMAGIZ += CompassCalibrationData[CompassZOffset];
        RawMAGIZ *= -1 * CompassCalibrationData[CompassZScaler];
    }

    void CompassGetUnfixAngle(double &UnFixAngle)
    {
        UnFixAngle = atan2((RawMAGIX), (RawMAGIY)) * 180.f / PI;
        if (UnFixAngle < 0)
            UnFixAngle += 360;
        else if (UnFixAngle >= 360)
            UnFixAngle -= 360;
    }

    void CompassGetFixAngle(double &FixAngle, double CompassRoll, double CompassPitch)
    {
        double MAGXFix = RawMAGIX * cos(-1 * CompassPitch * (PI / 180.f)) +
                         RawMAGIY * sin(CompassRoll * (PI / 180.f)) * sin(-1 * CompassPitch * (PI / 180.f)) -
                         RawMAGIZ * cos(CompassRoll * (PI / 180.f)) * sin(-1 * CompassPitch * (PI / 180.f));
        double MAGYFix = RawMAGIY * cos(CompassRoll * (PI / 180.f)) +
                         RawMAGIZ * sin(CompassRoll * (PI / 180.f));
        FixAngle = atan2(MAGXFix, MAGYFix) * 180.f / PI;
        if (FixAngle < 0)
            FixAngle += 360;
        else if (FixAngle >= 360)
            FixAngle -= 360;
    }

    void CompassGetRaw(long &RawMAGX, long &RawMAGY, long &RawMAGZ)
    {
        RawMAGX = RawMAGIX;
        RawMAGY = RawMAGIY;
        RawMAGZ = RawMAGIZ;
    }

private:
    void CompassRead(long &RawMAGX, long &RawMAGY, long &RawMAGZ)
    {
        switch (CompassType)
        {
        case COMPASS_QMC5883L:
        {
            DataBuffer[0] = wiringPiI2CReadReg8(CompassFD, 0x00);
            DataBuffer[1] = wiringPiI2CReadReg8(CompassFD, 0x01);
            unsigned long TMPRawMAGX = (DataBuffer[0] | DataBuffer[1] << 8);
            RawMAGX = (short)TMPRawMAGX;
            DataBuffer[0] = wiringPiI2CReadReg8(CompassFD, 0x02);
            DataBuffer[1] = wiringPiI2CReadReg8(CompassFD, 0x03);
            unsigned long TMPRawMAGY = (DataBuffer[0] | DataBuffer[1] << 8);
            RawMAGY = (short)TMPRawMAGY;
            DataBuffer[0] = wiringPiI2CReadReg8(CompassFD, 0x04);
            DataBuffer[1] = wiringPiI2CReadReg8(CompassFD, 0x05);
            unsigned long TMPRawMAGZ = (DataBuffer[0] | DataBuffer[1] << 8);
            RawMAGZ = (short)TMPRawMAGZ;
        }
        break;
        case COMPASS_HMC5883L:
        {
            DataBuffer[0] = wiringPiI2CReadReg8(CompassFD, 0x03);
            DataBuffer[1] = wiringPiI2CReadReg8(CompassFD, 0x04);
            unsigned long TMPRawMAGY = (DataBuffer[0] | DataBuffer[1] << 8) * -1;
            RawMAGY = (short)TMPRawMAGY;
            DataBuffer[0] = wiringPiI2CReadReg8(CompassFD, 0x05);
            DataBuffer[1] = wiringPiI2CReadReg8(CompassFD, 0x06);
            unsigned long TMPRawMAGZ = (DataBuffer[0] | DataBuffer[1] << 8);
            RawMAGZ = (short)TMPRawMAGZ;
            DataBuffer[0] = wiringPiI2CReadReg8(CompassFD, 0x07);
            DataBuffer[1] = wiringPiI2CReadReg8(CompassFD, 0x08);
            unsigned long TMPRawMAGX = (DataBuffer[0] | DataBuffer[1] << 8) * -1;
            RawMAGX = (short)TMPRawMAGX;
        }
        break;
        }
    }

    long RawMAGIX;
    long RawMAGIY;
    long RawMAGIZ;

    int CompassFD;
    int DataBuffer[2];
    int CompassType;
    double CompassCalibrationData[10] = {0};
};