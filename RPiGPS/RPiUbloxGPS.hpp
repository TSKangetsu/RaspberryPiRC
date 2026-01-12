#pragma once

#include <iostream>
#include <vector>
#include <string>
#include <cstring>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <asm-generic/ioctls.h>
#include <asm/termbits.h>
#include <thread>
#include <chrono>
#include <cmath>

// UBX Protocol Constants
#define UBX_PREAMBLE1 0xB5
#define UBX_PREAMBLE2 0x62

// Message Classes
#define UBX_CLASS_NAV 0x01
#define UBX_CLASS_ACK 0x05
#define UBX_CLASS_CFG 0x06
#define UBX_CLASS_MON 0x0A

// Message IDs
#define UBX_ID_NAV_PVT 0x07
#define UBX_ID_NAV_VELNED 0x12
#define UBX_ID_NAV_POSLLH 0x02
#define UBX_ID_NAV_STATUS 0x03
#define UBX_ID_NAV_SOL 0x06
#define UBX_ID_ACK_ACK 0x01
#define UBX_ID_ACK_NAK 0x00
#define UBX_ID_CFG_PRT 0x00
#define UBX_ID_CFG_MSG 0x01
#define UBX_ID_CFG_RATE 0x08
#define UBX_ID_CFG_NAV5 0x24
#define UBX_ID_MON_VER 0x04

// UBX Structures (Packed)
#pragma pack(push, 1)

struct ubx_header
{
    uint8_t preamble1;
    uint8_t preamble2;
    uint8_t msg_class;
    uint8_t msg_id;
    uint16_t length;
};

struct ubx_nav_pvt
{
    uint32_t iTOW;
    uint16_t year;
    uint8_t month;
    uint8_t day;
    uint8_t hour;
    uint8_t min;
    uint8_t sec;
    uint8_t valid;
    uint32_t tAcc;
    int32_t nano;
    uint8_t fixType;
    uint8_t flags;
    uint8_t flags2;
    uint8_t numSV;
    int32_t lon;
    int32_t lat;
    int32_t height;
    int32_t hMSL;
    uint32_t hAcc;
    uint32_t vAcc;
    int32_t velN;
    int32_t velE;
    int32_t velD;
    int32_t gSpeed;
    int32_t headMot;
    uint32_t sAcc;
    uint32_t headAcc;
    uint16_t pDOP;
    uint16_t flags3;      // reserved1 in older versions
    uint8_t reserved1[5]; // reserved1 + headVeh
    int32_t headVeh;
    int16_t magDec;
    uint16_t magAcc;
};

struct ubx_cfg_rate
{
    uint16_t measRate; // ms
    uint16_t navRate;  // cycles
    uint16_t timeRef;
};

struct ubx_cfg_nav5
{
    uint16_t mask;
    uint8_t dynModel;
    uint8_t fixMode;
    int32_t fixedAlt;
    uint32_t fixedAltVar;
    int8_t minElev;
    uint8_t drLimit;
    uint16_t pDop;
    uint16_t tDop;
    uint16_t pAcc;
    uint16_t tAcc;
    uint8_t staticHoldThresh;
    uint8_t dgnssTimeout;
    uint8_t cnoThreshNumSVs;
    uint8_t cnoThresh;
    uint16_t reserved1;
    uint16_t staticHoldMaxDist;
    uint8_t utcStandard;
    uint8_t reserved2[5];
};

struct ubx_cfg_msg
{
    uint8_t msgClass;
    uint8_t msgID;
    uint8_t rate;
};

#pragma pack(pop)

struct GPSData
{
    bool fix = false;
    int numSat = 0;
    int32_t lat = 0;     // deg * 1e7
    int32_t lon = 0;     // deg * 1e7
    int32_t alt = 0;     // mm above MSL
    int32_t velN = 0;    // mm/s
    int32_t velE = 0;    // mm/s
    int32_t velD = 0;    // mm/s
    int32_t gSpeed = 0;  // mm/s
    int32_t heading = 0; // deg * 1e5
    uint32_t hAcc = 0;   // mm
    uint32_t vAcc = 0;   // mm
    bool valid = false;  // Indicates if this specific parse call retrieved new data
};

class RPiUbloxGPS
{
public:
    // Constructor now handles full initialization
    RPiUbloxGPS(const std::string &device, int baudrate = 115200)
        : device_(device), targetBaud_(baudrate), fd_(-1), step_(0)
    {

        if (!openSerial(baudrate))
        {
            throw std::runtime_error("Failed to open GPS serial port");
        }

        // Initialize GPS immediately
        init();
    }

    ~RPiUbloxGPS()
    {
        if (fd_ >= 0)
            close(fd_);
    }

    // Combined function: Reads serial buffer, parses data, and returns the latest GPS state
    // The 'valid' flag in returned GPSData indicates if a new packet was successfully parsed in this call.
    // If no new data, it returns the last known good state with valid=false.
    GPSData parse()
    {
        if (fd_ < 0)
        {
            data_.valid = false;
            return data_;
        }

        uint8_t buf[512]; // Increased buffer size for efficiency
        int n = read(fd_, buf, sizeof(buf));

        data_.valid = false; // Reset valid flag for this iteration

        if (n > 0)
        {
            for (int i = 0; i < n; ++i)
            {
                if (processByte(buf[i]))
                {
                    data_.valid = true; // Mark as valid only if we got a fresh packet
                }
            }
        }
        return data_;
    }

private:
    std::string device_;
    int targetBaud_;
    int fd_;

    // Parsing State
    int step_;
    uint8_t msgClass_;
    uint8_t msgID_;
    uint16_t payloadLen_;
    uint16_t payloadCnt_;
    uint8_t ckA_, ckB_;
    std::vector<uint8_t> payloadBuffer_;

    GPSData data_;

    bool openSerial(int baudrate)
    {
        fd_ = open(device_.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
        if (fd_ < 0)
            return false;

        fcntl(fd_, F_SETFL, 0);

        struct termios2 options;
        ioctl(fd_, TCGETS2, &options);

        options.c_cflag &= ~CBAUD;
        options.c_cflag |= BOTHER;
        options.c_ispeed = baudrate;
        options.c_ospeed = baudrate;

        options.c_cflag |= (CLOCAL | CREAD);
        options.c_cflag &= ~CSIZE;
        options.c_cflag |= CS8;
        options.c_cflag &= ~PARENB;
        options.c_cflag &= ~CSTOPB;
        options.c_cflag &= ~CRTSCTS;

        options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
        options.c_iflag &= ~(IXON | IXOFF | IXANY);
        options.c_oflag &= ~OPOST;

        ioctl(fd_, TCSETS2, &options);
        return true;
    }

    void init()
    {
        std::cout << "Configuring GPS..." << std::endl;

        // 1. Configure Update Rate to 10Hz (100ms)
        configureRate(100);

        // 2. Configure Dynamic Model to Airborne 4G
        configureNav5(8);

        // 3. Disable NMEA messages
        configureMsg(0xF0, 0x00, 0); // GGA
        configureMsg(0xF0, 0x01, 0); // GLL
        configureMsg(0xF0, 0x02, 0); // GSA
        configureMsg(0xF0, 0x03, 0); // GSV
        configureMsg(0xF0, 0x04, 0); // RMC
        configureMsg(0xF0, 0x05, 0); // VTG

        // 4. Enable UBX NAV-PVT
        configureMsg(UBX_CLASS_NAV, UBX_ID_NAV_PVT, 1);

        std::cout << "GPS Configured." << std::endl;
    }

    void sendUBX(uint8_t msgClass, uint8_t msgID, void *payload, uint16_t len)
    {
        std::vector<uint8_t> packet;
        packet.push_back(UBX_PREAMBLE1);
        packet.push_back(UBX_PREAMBLE2);
        packet.push_back(msgClass);
        packet.push_back(msgID);
        packet.push_back(len & 0xFF);
        packet.push_back((len >> 8) & 0xFF);

        uint8_t *p = (uint8_t *)payload;
        for (int i = 0; i < len; ++i)
        {
            packet.push_back(p[i]);
        }

        uint8_t ck_a = 0, ck_b = 0;
        for (size_t i = 2; i < packet.size(); ++i)
        {
            ck_a += packet[i];
            ck_b += ck_a;
        }

        packet.push_back(ck_a);
        packet.push_back(ck_b);

        write(fd_, packet.data(), packet.size());
    }

    void configureRate(uint16_t measRateMs)
    {
        ubx_cfg_rate cfg = {};
        cfg.measRate = measRateMs;
        cfg.navRate = 1;
        cfg.timeRef = 1;
        sendUBX(UBX_CLASS_CFG, UBX_ID_CFG_RATE, &cfg, sizeof(cfg));
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    void configureNav5(uint8_t dynModel)
    {
        ubx_cfg_nav5 cfg = {};
        cfg.mask = 0x0001;
        cfg.dynModel = dynModel;
        sendUBX(UBX_CLASS_CFG, UBX_ID_CFG_NAV5, &cfg, sizeof(cfg));
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    void configureMsg(uint8_t msgClass, uint8_t msgID, uint8_t rate)
    {
        ubx_cfg_msg cfg = {};
        cfg.msgClass = msgClass;
        cfg.msgID = msgID;
        cfg.rate = rate;
        sendUBX(UBX_CLASS_CFG, UBX_ID_CFG_MSG, &cfg, sizeof(cfg));
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    bool processByte(uint8_t b)
    {
        bool parsed = false;
        switch (step_)
        {
        case 0:
            if (b == UBX_PREAMBLE1)
                step_++;
            break;
        case 1:
            if (b == UBX_PREAMBLE2)
                step_++;
            else
                step_ = 0;
            break;
        case 2:
            msgClass_ = b;
            ckA_ = b;
            ckB_ = b;
            step_++;
            break;
        case 3:
            msgID_ = b;
            ckA_ += b;
            ckB_ += ckA_;
            step_++;
            break;
        case 4:
            payloadLen_ = b;
            ckA_ += b;
            ckB_ += ckA_;
            step_++;
            break;
        case 5:
            payloadLen_ |= (b << 8);
            ckA_ += b;
            ckB_ += ckA_;
            payloadCnt_ = 0;
            payloadBuffer_.resize(payloadLen_);
            step_++;
            break;
        case 6:
            if (payloadCnt_ < payloadLen_)
            {
                payloadBuffer_[payloadCnt_++] = b;
                ckA_ += b;
                ckB_ += ckA_;
            }
            if (payloadCnt_ == payloadLen_)
                step_++;
            break;
        case 7:
            if (ckA_ == b)
                step_++;
            else
                step_ = 0;
            break;
        case 8:
            if (ckB_ == b)
            {
                parseMessage();
                parsed = true;
            }
            step_ = 0;
            break;
        }
        return parsed;
    }

    void parseMessage()
    {
        if (msgClass_ == UBX_CLASS_NAV && msgID_ == UBX_ID_NAV_PVT)
        {
            if (payloadLen_ != sizeof(ubx_nav_pvt))
                return;

            ubx_nav_pvt *pvt = (ubx_nav_pvt *)payloadBuffer_.data();

            data_.fix = (pvt->fixType == 3);
            data_.numSat = pvt->numSV;
            data_.lat = pvt->lat;
            data_.lon = pvt->lon;
            data_.alt = pvt->hMSL;
            data_.hAcc = pvt->hAcc;
            data_.vAcc = pvt->vAcc;
            data_.velN = pvt->velN;
            data_.velE = pvt->velE;
            data_.velD = pvt->velD;
            data_.gSpeed = pvt->gSpeed;
            data_.heading = pvt->headMot;
        }
    }
};