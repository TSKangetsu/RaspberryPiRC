#pragma once
#ifdef DEBUG
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
#include <thread>
#include <linux/i2c-dev.h>
#include <iostream>
#include <math.h>
#include <stdexcept>
#define termios asmtermios
#include <asm/termbits.h>
#undef termios
#include <termios.h>
#include <vector>
#define PI 3.1415926
#define QMC5883LADDR 0x0D
#define HMC5883LADDR 0x1E

#define CompassXOffset 0
#define CompassXScaler 1
#define CompassYOffset 2
#define CompassYScaler 3
#define CompassZOffset 4
#define CompassZScaler 5
#define CompassVOffset 6
#define CompassVScaler 7

#define GGAData_LAT 2
#define GGAData_North 3
#define GGAData_LNG 4
#define GGAData_East 5
#define GGAData_Quality 6
#define GGAData_SATNUM 7
#define GGAData_HDOP 8
#define GGAData_Altitude 9
#define GGAData_GeoidalSP 11

#define DEG2RAD(x) (x * PI / 180.f)

struct GPSUartData
{
    double lat = 0;
    bool lat_North_Mode;
    double lng = 0;
    bool lat_East_Mode;

    int GPSQuality = 0;

    int satillitesCount = 0;
    double HDOP = 0;

    double GPSAlititude = 0;
    double GPSGeoidalSP = 0;
    //
    bool DataUnCorrect;
};

class GPSUart
{
public:
    inline GPSUart(const char *UartDevice, int baudRate)
    {
        GPSDevice = UartDevice;
        GPSUart_fd = open(UartDevice, O_RDWR | O_NOCTTY | O_NDELAY);

        // TODO: better expection expected
        if (GPSUart_fd == -1)
            throw std::invalid_argument("[UART] GPS Unable to open device:" + std::string(UartDevice));

        fcntl(GPSUart_fd, F_SETFL, 0);

        struct termios2 options;
        if (0 != ioctl(GPSUart_fd, TCGETS2, &options))
        {
            close(GPSUart_fd);
            GPSUart_fd = -1;
        }
        options.c_cflag &= ~CBAUD;
        options.c_cflag |= BOTHER;
        options.c_ispeed = baudRate;
        options.c_ospeed = baudRate;
        options.c_cflag = (options.c_cflag & ~CSIZE) | CS8;
        options.c_cflag |= CLOCAL | CREAD;
        options.c_cflag &= ~PARENB;
        options.c_cflag &= ~CSTOPB;
        options.c_cflag &= ~CRTSCTS;
        options.c_lflag &= ~ICANON;
        options.c_lflag &= ~ECHO;
        options.c_lflag &= ~ECHOE;
        options.c_lflag &= ~ISIG;
        options.c_iflag &= ~(IXON | IXOFF | IXANY);
        options.c_iflag &= ~(ICRNL | INLCR);
        options.c_oflag &= ~OPOST;
        options.c_cc[VMIN] = 0;
        options.c_cc[VTIME] = 10;

        if (0 != ioctl(GPSUart_fd, TCSETS2, &options))
        {
            close(GPSUart_fd);
            GPSUart_fd = -1;
        }
    }

    inline void GPSReOpen()
    {
        while (true)
        {
            std::string outputData;
            int bytes_avaiable = 0;
            ioctl(GPSUart_fd, FIONREAD, &bytes_avaiable);
            std::cout << "GPS finding bytes_avaiable" << bytes_avaiable << "\r\n";
            if (bytes_avaiable > 0)
            {
                char TmpData[bytes_avaiable];
                int ReadCount = read(GPSUart_fd, TmpData, bytes_avaiable);
                if (ReadCount > 0)
                {
                    for (size_t i = 0; i < ReadCount; i++)
                    {
                        outputData += TmpData[i];
                    }
                    if (outputData.find("$GNGSA") != std::string::npos)
                    {
                        std::cout << "GPS find" << "\r\n";
                        break;
                    }
                }
            }
            if (currentBaudRateIndex < baudRates.size())
            {
                setBaudRate(baudRates[currentBaudRateIndex]);
                std::cout << "Switching to baud rate: " << baudRates[currentBaudRateIndex] << "\r\n";
                currentBaudRateIndex++;
            }
            else
            {
                currentBaudRateIndex = 0;
                errorcount++;
            }
            if (errorcount > 5)
                throw std::invalid_argument("[UART] GPS not found the right baud rate");

            usleep(1000000);
        }
    }

    inline void GPSkDataAvaliable()
    {
        if (write(GPSUart_fd, GPSDisableGPGSVConfig, sizeof(GPSDisableGPGSVConfig)) == -1)
            throw std::invalid_argument("[UART] GPSWriteConfigError");
        else
        {
            tcdrain(GPSUart_fd);
            tcflush(GPSUart_fd, TCOFLUSH);
            if (write(GPSUart_fd, GPS5HzConfig, sizeof(GPS5HzConfig)) == -1)
                throw std::invalid_argument("[UART] GPSWriteConfig5HZError");
            else
            {
                tcdrain(GPSUart_fd);
                tcflush(GPSUart_fd, TCOFLUSH);
            }
        };
    }

    inline int GPSRead(std::string &outputData)
    {
        int bytes_avaiable = 0;
        ioctl(GPSUart_fd, FIONREAD, &bytes_avaiable);
        if (bytes_avaiable > 0)
        {
            char TmpData[bytes_avaiable];
            int ReadCount = read(GPSUart_fd, TmpData, bytes_avaiable);

            if (ReadCount > 0)
            {
                for (size_t i = 0; i < ReadCount; i++)
                {
                    outputData += TmpData[i];
                }
            }
            return ReadCount;
        }
        return -1;
    }

    inline GPSUartData GPSParse()
    {
        int DataCount = 0;
        int GGADataCrash = 0;
        GPSUartData myData;
        myData.DataUnCorrect = true;
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
            if (Count < (sizeof(GPSData) / sizeof(GPSData[0])))
            {
                // step 1: search GNGGA data
                if (strncmp("$GNGGA", GPSData[Count].c_str(), 5) == 0)
                {
                    GGADataCrash++;
                    if (GGADataCrash > 1)
                    {
                        break;
                    }
                    // step 2: Check CRC data is correct
                    // std::cout << "GPSData[" <<GPSData[Count].c_str() << std::endl;

                    if (!NMEA_CheckSum(GPSData[Count]))
                        myData.DataUnCorrect = false;
                    else
                        myData.DataUnCorrect = true;
                    // std::cout << "myData.DataUnCorrect [" <<myData.DataUnCorrect << std::endl;

                    // step 3: get lat
                    dataParese(GPSData[Count], GPSDataSub, ',', 40);
                    std::string GPSDataTmpLat = std::to_string(std::atof(GPSDataSub[GGAData_LAT].c_str()) / 100.0);
                    dataParese(GPSDataTmpLat, GPSTmpData, '.', 2);
                    myData.lat = std::atof(GPSTmpData[0].c_str()) * 10000.0;
                    myData.lat += std::atof(GPSTmpData[1].c_str()) / 60.0;
                    myData.lat = (int)(myData.lat * 100);
                    // step 4: get lng
                    std::string GPSDataTmpLng = std::to_string(std::atof(GPSDataSub[GGAData_LNG].c_str()) / 100.0);
                    dataParese(GPSDataTmpLng, GPSTmpData, '.', 2);
                    myData.lng = std::atof(GPSTmpData[0].c_str()) * 10000.0;
                    myData.lng += std::atof(GPSTmpData[1].c_str()) / 60.0;
                    myData.lng = (int)(myData.lng * 100);
                    // step 5: get North of lat
                    if (strncmp(GPSDataSub[GGAData_North].c_str(), "N", 1) == 0)
                        myData.lat_North_Mode = true;
                    else
                        myData.lat_North_Mode = false;
                    // step 6: get East of lng
                    if (strncmp(GPSDataSub[GGAData_East].c_str(), "E", 1) == 0)
                        myData.lat_East_Mode = true;
                    else
                        myData.lat_East_Mode = false;
                    // step ...parse other ...
                    myData.GPSQuality = std::atof(GPSDataSub[GGAData_Quality].c_str());
                    myData.satillitesCount = std::atof(GPSDataSub[GGAData_SATNUM].c_str());
                    myData.HDOP = std::atof(GPSDataSub[GGAData_HDOP].c_str());
                    myData.GPSAlititude = std::atof(GPSDataSub[GGAData_Altitude].c_str());
                    myData.GPSGeoidalSP = std::atof(GPSDataSub[GGAData_GeoidalSP].c_str());
                }
                else if (strncmp("$GNGLL", GPSData[Count].c_str(), 5) == 0)
                {
                    dataParese(GPSData[Count], GPSDataSub, ',', 40);
                }
                Count++;
                GPSLastDebug = GPSDataStr;
            }
            else
                break;
        }
        return myData;
    };

    inline void setBaudRate(int baudRate)
    {
        struct termios2 options;
        if (0 != ioctl(GPSUart_fd, TCGETS2, &options))
        {
            close(GPSUart_fd);
            GPSUart_fd = -1;
        }

        options.c_cflag &= ~CBAUD;
        options.c_cflag |= BOTHER;
        options.c_ispeed = baudRate;
        options.c_ospeed = baudRate;

        if (0 != ioctl(GPSUart_fd, TCSETS2, &options))
        {
            close(GPSUart_fd);
            GPSUart_fd = -1;
        }
    }

    inline ~GPSUart()
    {
        close(GPSUart_fd);
    }

private:
    int GPSUart_fd;
    std::vector<int> baudRates = {460800, 921600, 9600, 14400, 19200, 38400, 57600, 115200, 230400};
    uint8_t GPS5HzConfig[14] = {0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0xC8, 0x00, 0x01, 0x00, 0x01, 0x00, 0xDE, 0x6A};
    uint8_t GPS10HzConfig[14] = {0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0x64, 0x00, 0x01, 0x00, 0x01, 0x00, 0x7A, 0x12};
    uint8_t GPSDisableGPGSVConfig[11] = {0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x03, 0x00, 0xFD, 0x15};
    size_t currentBaudRateIndex = 0;
    std::string GPSDevice;
    int errorcount = 0;
    std::string GPSLastDebug;

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

    unsigned char AsciiToHex(char *str, unsigned char size, unsigned char *result)
    {
        unsigned char temp;

        for (*result = 0; size; size--, str++)
        {
            if (('9' >= *str) && (*str >= '0'))
                temp = *str - '0';
            else if (('F' >= *str) && (*str >= 'A'))
                temp = *str - 'A' + 10;
            else if (('f' >= *str) && (*str >= 'a'))
                temp = *str - 'a' + 10;
            else
                return 1;
            *result |= temp << ((size - 1) * 4);
        }

        return 0;
    }

    unsigned char NMEA_CheckSum(std::string buf)
    {
        unsigned char i;
        unsigned char chk, result;

        for (chk = buf[1], i = 2; (buf[i] != '*') && (i < 255); i++)
        {
            chk ^= buf[i];
        }

        if (AsciiToHex(&buf[i + 1], 2, &result))
            return 3;
        if (i >= 255)
            return 2;
        if (chk != result)
            return 1;

        return 0;
    }
};

#define COMPASS_FLIP_X 0
#define COMPASS_FLIP_Y 1
#define COMPASS_FLIP_Z 2

#define QMC5883_REG_HRESET 0x0b
#define QMC5883_REG_RESET 0x0a
#define QMC5883_REG_MODE 0x09
#define QMC5883_REG_STATUS 0x06
#define QMC5883_REG_DATA 0x00

#define HMC5883_REG_CONFIGA 0x00
#define HMC5883_REG_CONFIGB 0x01
#define HMC5883_REG_MODE 0x02
#define HMC5883_REG_DATA 0x03
#define HMC5883_REG_STATUS 0x09
//
#define QMC5883_OP_HRESET 0x01
#define QMC5883_OP_RESET 0x80
#define QMC5883_OP_200HZ 0x1d

#define QMC5883_REG_PRODUCTID 0x0D
#define QMC5883_REG_CTRL1 0x09
#define QMC5883_CMD_MODE_CON 0x01
#define QMC5883_CMD_ODR_200HZ 0x0C
#define QMC5883_CMD_RNG_8G 0x10
#define QMC5883_CMD_OSR_512 0x00

enum CompassType
{
    COMPASS_QMC5883L,
    COMPASS_HMC5883L,
};

class GPSI2CCompass
{
public:
    GPSI2CCompass(const char *i2cDevice, uint8_t i2caddr, CompassType Compass_Type, int FlipConfig[3])
    {
        CompassFD = open(i2cDevice, O_RDWR);
        if (ioctl(CompassFD, I2C_SLAVE, i2caddr) < 0)
            std::invalid_argument("[I2C] COMPASS Unable to open device:" + std::string(i2cDevice));
        if (ioctl(CompassFD, I2C_TIMEOUT, 0x01) < 0) // set to 10ms?
            throw -1;

        CompassType = Compass_Type;
        //
        switch (CompassType)
        {
        case COMPASS_QMC5883L:
        {
            {
                uint8_t wdata[2] = {QMC5883_REG_HRESET, QMC5883_OP_HRESET};
                if (write(CompassFD, &wdata, 2) < 0)
                    throw std::invalid_argument("[I2C] init compass error");
            }
            //
            usleep(1000);
            {
                uint8_t wdata[1] = {QMC5883_REG_PRODUCTID};
                uint8_t rdata[1] = {0};
                write(CompassFD, &wdata, 1);
                read(CompassFD, &rdata, 1);
                if (static_cast<int>(rdata[0]) != 255)
                    throw std::invalid_argument("[i2c] init QMC5883 error");
            }
            usleep(1000);
            {
                uint8_t wdata[2] = {QMC5883_REG_MODE, QMC5883_CMD_MODE_CON | QMC5883_CMD_ODR_200HZ | QMC5883_CMD_RNG_8G | QMC5883_CMD_OSR_512};
                if (write(CompassFD, &wdata, 2) < 0)
                    throw std::invalid_argument("[I2C] init compass error");
            }
            usleep(5000);
        }
        break;
        case COMPASS_HMC5883L:
        {
            {
                uint8_t wdata[2] = {HMC5883_REG_CONFIGA, 0x18};
                if (write(CompassFD, &wdata, 2) < 0)
                    throw std::invalid_argument("[I2C] init compass error");
            }

            {
                uint8_t wdata[2] = {HMC5883_REG_CONFIGB, 0xE0};
                if (write(CompassFD, &wdata, 2) < 0)
                    throw std::invalid_argument("[I2C] init compass error");
            }
            {
                uint8_t wdata[2] = {HMC5883_REG_MODE, 0x00};
                if (write(CompassFD, &wdata, 2) < 0)
                    throw std::invalid_argument("[I2C] init compass error");
            }
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

        flipConfig[0] = FlipConfig[0];
        flipConfig[1] = FlipConfig[1];
        flipConfig[2] = FlipConfig[2];
    };

    void CompassCaliInit()
    {
        {
            uint8_t wdata[2] = {QMC5883_REG_HRESET, QMC5883_OP_HRESET};
            if (write(CompassFD, &wdata, 2) < 0)
                throw std::invalid_argument("[I2C] init compass error");
        }
        usleep(1000);
        //
        {
            uint8_t wdata[2] = {QMC5883_REG_RESET, QMC5883_OP_RESET};
            if (write(CompassFD, &wdata, 2) < 0)
                throw std::invalid_argument("[I2C] init compass error");
        }
        //
        usleep(1000);
        {
            uint8_t wdata[2] = {QMC5883_REG_MODE, QMC5883_OP_200HZ};
            if (write(CompassFD, &wdata, 2) < 0)
                throw std::invalid_argument("[I2C] init compass error");
        }
        usleep(5000);
    }

    void CompassCalibration(bool Calibrating, int *CalibratedData)
    {
        CompassCalibrationData[0] = 0;
        CompassCalibrationData[1] = 0;
        CompassCalibrationData[2] = 0;
        CompassCalibrationData[7] = 1;
        CompassCalibrationData[8] = 1;
        CompassCalibrationData[9] = 1;

        if (Calibrating)
        {
            // X Y Z MAX MIN
            CalibratedData[0] = RawMAGCX > CalibratedData[0] ? RawMAGCX : CalibratedData[0];
            CalibratedData[1] = RawMAGCX < CalibratedData[1] ? RawMAGCX : CalibratedData[1];
            CalibratedData[2] = RawMAGCY > CalibratedData[2] ? RawMAGCY : CalibratedData[2];
            CalibratedData[3] = RawMAGCY < CalibratedData[3] ? RawMAGCY : CalibratedData[3];
            CalibratedData[4] = RawMAGCZ > CalibratedData[4] ? RawMAGCZ : CalibratedData[4];
            CalibratedData[5] = RawMAGCZ < CalibratedData[5] ? RawMAGCZ : CalibratedData[5];
        }
        else
        {
            CalibratedData[5] = RawMAGCZ;
            CalibratedData[4] = RawMAGCZ;
            CalibratedData[3] = RawMAGCY;
            CalibratedData[2] = RawMAGCY;
            CalibratedData[1] = RawMAGCX;
            CalibratedData[0] = RawMAGCX;
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
        UnFixAngle = atan2((float)RawMAGCY, (float)-1 * RawMAGCX) * 180.f / PI;
        if (UnFixAngle < 0)
            UnFixAngle += 360.f;
        else if (UnFixAngle >= 360)
            UnFixAngle -= 360.f;
        UnFixAngle = 360.f - UnFixAngle;
    }

    void CompassGetFixAngle(double &FixAngle, double CompassRoll, double CompassPitch)
    {
        double MAGXFix = RawMAGCX * cos(CompassPitch * (PI / 180.f)) +
                         RawMAGCY * sin(CompassRoll * (PI / 180.f)) * sin(CompassPitch * (PI / 180.f)) -
                         RawMAGCZ * cos(CompassRoll * (PI / 180.f)) * sin(CompassPitch * (PI / 180.f));
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
        RawMAGCX = tRawMAGX;
        RawMAGCY = tRawMAGY;
        RawMAGCZ = tRawMAGZ;
        RawMAGX = RawMAGCX;
        RawMAGY = RawMAGCY;
        RawMAGZ = RawMAGCZ;
        return error;
    }

private:
    int CompassRead(int &RawMAGX, int &RawMAGY, int &RawMAGZ)
    {
        switch (CompassType)
        {
        case COMPASS_QMC5883L:
        {
            int error = -1;
            uint8_t cadata[1] = {QMC5883_REG_STATUS};
            uint8_t cxdata[1];
            error = write(CompassFD, &cadata, 1);
            error = read(CompassFD, cxdata, 1);

            if (((cxdata[0] & (1 << 0)) >> 0) == 1 && ((cxdata[0] & (1 << 1)) >> 1) == 0)
            {
                uint8_t cdata[6] = {0x00};
                uint8_t wdata[1] = {QMC5883_REG_DATA};
                error = write(CompassFD, &wdata, 1);
                error = read(CompassFD, cdata, 6);
                if (error == 6)
                {
                    int Tmp_MX = (short)(cdata[1] << 8 | cdata[0]);
                    int Tmp_MY = (short)(cdata[3] << 8 | cdata[2]);
                    int Tmp_MZ = (short)(cdata[5] << 8 | cdata[4]) * 1; // Revert as HMC5883L

                    int Tmp_M2X = Tmp_MX * cos(DEG2RAD((flipConfig[2]))) + Tmp_MY * sin(DEG2RAD((flipConfig[2])));
                    int Tmp_M2Y = Tmp_MY * cos(DEG2RAD((flipConfig[2]))) + Tmp_MX * sin(DEG2RAD((180 + flipConfig[2])));
                    // Step 2: rotate Pitch
                    int Tmp_M3X = Tmp_M2X * cos(DEG2RAD(flipConfig[0])) + Tmp_MZ * sin(DEG2RAD((flipConfig[0])));
                    int Tmp_M3Z = Tmp_MZ * cos(DEG2RAD((flipConfig[0]))) + Tmp_M2X * sin(DEG2RAD((180 + flipConfig[0])));
                    // Step 3: rotate Roll
                    RawMAGY = Tmp_M2Y * cos(DEG2RAD((flipConfig[1]))) + Tmp_M3Z * sin(DEG2RAD((180 + flipConfig[1])));
                    RawMAGZ = Tmp_M3Z * cos(DEG2RAD((flipConfig[1]))) + Tmp_M2Y * sin(DEG2RAD((flipConfig[1])));
                    RawMAGX = Tmp_M3X;
                }
            }
            else
            {
                return -1;
            }

            return error;
        }
        break;

        case COMPASS_HMC5883L:
        {
            int error = -1;
            uint8_t cadata[1] = {HMC5883_REG_STATUS};
            uint8_t cxdata[1] = {0x00};
            error = write(CompassFD, &cadata, 1);
            error = read(CompassFD, cxdata, 1);
            if (((cxdata[0] & (1 << 0)) >> 0) == 1)
            {
                uint8_t cdata[6] = {0x00};
                uint8_t wdata[1] = {HMC5883_REG_DATA};
                error = write(CompassFD, &wdata, 1);
                error = read(CompassFD, cdata, 6);
                if (error == 6)
                {
                    int Tmp_MX = (short)(cdata[1] << 8 | cdata[0]);
                    int Tmp_MY = (short)(cdata[5] << 8 | cdata[4]);
                    int Tmp_MZ = (short)(cdata[3] << 8 | cdata[2]);

                    int Tmp_M2X = Tmp_MX * cos(DEG2RAD((flipConfig[2]))) + Tmp_MY * sin(DEG2RAD((flipConfig[2])));
                    int Tmp_M2Y = Tmp_MY * cos(DEG2RAD((flipConfig[2]))) + Tmp_MX * sin(DEG2RAD((180 + flipConfig[2])));
                    // Step 2: rotate Pitch
                    int Tmp_M3X = Tmp_M2X * cos(DEG2RAD(flipConfig[0])) + Tmp_MZ * sin(DEG2RAD((flipConfig[0])));
                    int Tmp_M3Z = Tmp_MZ * cos(DEG2RAD((flipConfig[0]))) + Tmp_M2X * sin(DEG2RAD((180 + flipConfig[0])));
                    // Step 3: rotate Roll
                    RawMAGY = Tmp_M2Y * cos(DEG2RAD((flipConfig[1]))) + Tmp_M3Z * sin(DEG2RAD((180 + flipConfig[1])));
                    RawMAGZ = Tmp_M3Z * cos(DEG2RAD((flipConfig[1]))) + Tmp_M2Y * sin(DEG2RAD((flipConfig[1])));
                    RawMAGX = Tmp_M3X;
                }
            }
            else
            {
                return -1;
            }

            return error;
        }
        break;
        }
    }

    int flipConfig[3];

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
