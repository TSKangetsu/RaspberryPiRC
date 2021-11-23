#pragma once
#ifdef DEBUG
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
#include <thread>
#include <linux/i2c-dev.h>
#include <iostream>
#include <math.h>
#define termios asmtermios
#include <asm/termbits.h>
#undef termios
#include <termios.h>
#define PI 3.1415926
#define QMC5883LADDR 0x0D
#define HMC5883LADDR 0x1E
#define COMPASS_QMC5883L 0
#define COMPASS_HMC5883L 1

#define CompassXOffset 0
#define CompassXScaler 1
#define CompassYOffset 2
#define CompassYScaler 3
#define CompassZOffset 4
#define CompassZScaler 5

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

#define QMC5883_REG_RESET 0x0a
#define QMC5883_REG_MODE 0x09
#define QMC5883_REG_STATUS 0x06
#define QMC5883_REG_DATA 0x00

#define QMC5883_OP_RESET 0x80
#define QMC5883_OP_200HZ 0x1d

class GPSI2CCompass
{
public:
    GPSI2CCompass(const char *i2cDevice, uint8_t i2caddr, int Compass_Type)
    {
        CompassFD = open(i2cDevice, O_RDWR);
        if (ioctl(CompassFD, I2C_SLAVE, i2caddr) < 0)
            throw - 1;
        CompassType = Compass_Type;
        //
        switch (CompassType)
        {
        case COMPASS_QMC5883L:
        {
            {
                uint8_t wdata[2] = {QMC5883_REG_RESET, QMC5883_OP_RESET};
                if (write(CompassFD, &wdata, 2) < 0)
                    throw - 2;
            }
            //
            usleep(1000);
            {
                uint8_t wdata[2] = {QMC5883_REG_MODE, QMC5883_OP_200HZ};
                if (write(CompassFD, &wdata, 2) < 0)
                    throw - 2;
            }
            usleep(5000);
        }
        break;
        case COMPASS_HMC5883L:
        {
        }
        break;
        }
        //
        CompassCalibrationData[0] = 0;
        CompassCalibrationData[1] = 0;
        CompassCalibrationData[2] = 0;
        CompassCalibrationData[7] = 1;
        CompassCalibrationData[8] = 1;
        CompassCalibrationData[9] = 1;
    };

    void CompassCalibration(bool Calibrating, int *CalibratedData)
    {
        if (Calibrating)
        {
            CompassRead(RawMAGIX, RawMAGIY, RawMAGIZ);
            // X Y Z MAX MIN
            CalibratedData[0] = RawMAGIX > CalibratedData[0] ? RawMAGIX : CalibratedData[0];
            CalibratedData[1] = RawMAGIX < CalibratedData[1] ? RawMAGIX : CalibratedData[1];
            CalibratedData[2] = RawMAGIY > CalibratedData[2] ? RawMAGIY : CalibratedData[2];
            CalibratedData[3] = RawMAGIY < CalibratedData[3] ? RawMAGIY : CalibratedData[3];
            CalibratedData[4] = RawMAGIZ > CalibratedData[4] ? RawMAGIZ : CalibratedData[4];
            CalibratedData[5] = RawMAGIZ < CalibratedData[5] ? RawMAGIZ : CalibratedData[5];
        }
    }

    void CompassApply(int XMAX, int XMIN, int YMAX, int YMIN, int ZMAX, int ZMIN)
    {
        CompassCalibrationData[0] = (XMAX + XMIN) / 2;
        CompassCalibrationData[1] = (YMAX + YMIN) / 2;
        CompassCalibrationData[2] = (ZMAX + ZMIN) / 2;
        CompassCalibrationData[3] = (XMAX - XMIN) / 2;
        CompassCalibrationData[4] = (YMAX - YMIN) / 2;
        CompassCalibrationData[5] = (ZMAX - ZMIN) / 2;
        CompassCalibrationData[6] = (CompassCalibrationData[3] + CompassCalibrationData[4] + CompassCalibrationData[5]) / 3;
        CompassCalibrationData[7] = CompassCalibrationData[6] / CompassCalibrationData[3];
        CompassCalibrationData[8] = CompassCalibrationData[6] / CompassCalibrationData[4];
        CompassCalibrationData[9] = CompassCalibrationData[6] / CompassCalibrationData[5];
    }

    void CompassUpdate()
    {
    }

    void CompassGetUnfixAngle(double &UnFixAngle)
    {
        UnFixAngle = atan2((float)RawMAGCY * -1, (float)RawMAGCX) * 180.f / PI;
        if (UnFixAngle < 0)
            UnFixAngle += 360.f;
        else if (UnFixAngle >= 360)
            UnFixAngle -= 360.f;
        UnFixAngle = 360.f - UnFixAngle;
    }

    void CompassGetFixAngle(double &FixAngle, double CompassRoll, double CompassPitch)
    {
        double MAGXFix = RawMAGCX * cos(CompassPitch * (PI / 180.f)) +
                         RawMAGCY * sin(CompassRoll * (PI / -180.f)) * sin(CompassPitch * (PI / 180.f)) -
                         RawMAGCZ * cos(CompassRoll * (PI / -180.f)) * sin(CompassPitch * (PI / 180.f));
        double MAGYFix = RawMAGCY * cos(CompassRoll * (PI / 180.f)) +
                         RawMAGCZ * sin(CompassRoll * (PI / 180.f));

        if (MAGYFix < 0)
            FixAngle = 180 + (180 + ((atan2(MAGYFix, MAGXFix) * 180.f / PI)));
        else
            FixAngle = atan2(MAGYFix, MAGXFix) * 180.f / PI;

        if (FixAngle < 0)
            FixAngle += 360;
        else if (FixAngle >= 360)
            FixAngle -= 360;
    }

    int CompassGetRaw(int &RawMAGX, int &RawMAGY, int &RawMAGZ)
    {
        int error = CompassRead(RawMAGIX, RawMAGIY, RawMAGIZ);
        int tRawMAGX = ((double)RawMAGIX - CompassCalibrationData[0]) * CompassCalibrationData[7];
        int tRawMAGY = ((double)RawMAGIY - CompassCalibrationData[1]) * CompassCalibrationData[8];
        int tRawMAGZ = ((double)RawMAGIZ - CompassCalibrationData[2]) * CompassCalibrationData[9];
        RawMAGCX = tRawMAGY * -1;
        RawMAGCY = tRawMAGX * -1;
        RawMAGCZ = tRawMAGZ * 1;
        RawMAGX = RawMAGCX;
        RawMAGY = RawMAGCY;
        RawMAGZ = RawMAGCZ;
        return error;
    }

private:
    int CompassRead(int &RawMAGX, int &RawMAGY, int &RawMAGZ)
    {
        uint8_t cdata[6] = {0x00};
        uint8_t wdata[1] = {QMC5883_REG_DATA};
        int error = write(CompassFD, &wdata, 1);
        error = read(CompassFD, cdata, 6);
        if (error == 6)
        {
            RawMAGX = (short)(cdata[1] << 8 | cdata[0]);
            RawMAGY = (short)(cdata[3] << 8 | cdata[2]);
            RawMAGZ = (short)(cdata[5] << 8 | cdata[4]);
        }

        return error;
    }

    int RawMAGIX = 0;
    int RawMAGIY = 0;
    int RawMAGIZ = 0;

    int RawMAGCX = 0;
    int RawMAGCY = 0;
    int RawMAGCZ = 0;

    int CompassFD;
    int CompassType;
    double CompassCalibrationData[10] = {0};
};
