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

#define DEG2RAD(x) (x * PI / 180.f)

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
        flipConfig[0] = FlipConfig[0];
        flipConfig[1] = FlipConfig[1];
        flipConfig[2] = FlipConfig[2];
    };

    void CompassCalibration(float &RawMAGX, float &RawMAGY, float &RawMAGZ, double (&V)[3], double (&W)[3][3], double learning_rate)
    {
        double dx = RawMAGX - V[0];
        double dy = RawMAGY - V[1];
        double dz = RawMAGZ - V[2];

        double cx = W[0][0] * dx + W[0][1] * dy + W[0][2] * dz;
        double cy = W[1][0] * dx + W[1][1] * dy + W[1][2] * dz;
        double cz = W[2][0] * dx + W[2][1] * dy + W[2][2] * dz;

        double error = (cx * cx + cy * cy + cz * cz) - 1.0;
        double err_mult = 4.0 * error * learning_rate;

        V[0] -= err_mult * (-(W[0][0] * cx + W[1][0] * cy + W[2][0] * cz));
        V[1] -= err_mult * (-(W[0][1] * cx + W[1][1] * cy + W[2][1] * cz));
        V[2] -= err_mult * (-(W[0][2] * cx + W[1][2] * cy + W[2][2] * cz));

        W[0][0] -= err_mult * cx * dx;
        W[0][1] -= err_mult * cx * dy;
        W[0][2] -= err_mult * cx * dz;
        W[1][0] -= err_mult * cy * dx;
        W[1][1] -= err_mult * cy * dy;
        W[1][2] -= err_mult * cy * dz;
        W[2][0] -= err_mult * cz * dx;
        W[2][1] -= err_mult * cz * dy;
        W[2][2] -= err_mult * cz * dz;
    }

    void CompassApply(double (&V)[3], double (&W)[3][3])
    {
        std::memcpy(Calibration_V, V, sizeof(V));
        std::memcpy(Calibration_W, W, sizeof(W));
    }

    void CompassGetUnfixAngle(double &UnFixAngle)
    {
        float angle = atan2((float)RawMAGCY, (float)RawMAGCX) * 180.f / PI;

        if (angle < 0)
            angle += 360.f;

        UnFixAngle = angle;
    }

    int CompassGetRaw(int &RawMAGX, int &RawMAGY, int &RawMAGZ)
    {
        int error = CompassRead(RawMAGIX, RawMAGIY, RawMAGIZ);

        float temp_x = RawMAGIX - (Calibration_V[0]);
        float temp_y = RawMAGIY - (Calibration_V[1]);
        float temp_z = RawMAGIZ - (Calibration_V[2]);
        int tRawMAGX = Calibration_W[0][0] * temp_x + Calibration_W[0][1] * temp_y + Calibration_W[0][2] * temp_z;
        int tRawMAGY = Calibration_W[1][0] * temp_x + Calibration_W[1][1] * temp_y + Calibration_W[1][2] * temp_z;
        int tRawMAGZ = Calibration_W[2][0] * temp_x + Calibration_W[2][1] * temp_y + Calibration_W[2][2] * temp_z;

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
    double Calibration_V[3];
    double Calibration_W[3][3];
};
