#pragma once

#include <vector>
#include <string>
#include <cstring>
#include <cstdint>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <asm-generic/ioctls.h>
#include <asm/termbits.h>
#include <thread>
#include <chrono>
#include <sys/time.h>
#include <stdexcept>

#pragma pack(push, 1)
struct ubx_nav_pvt
{
    uint32_t iTOW;
    uint16_t year;
    uint8_t month, day, hour, min, sec, valid;
    uint32_t tAcc;
    int32_t nano;
    uint8_t fixType, flags, flags2, numSV;
    int32_t lon, lat, height, hMSL;
    uint32_t hAcc, vAcc;
    int32_t velN, velE, velD, gSpeed, headMot;
    uint32_t sAcc, headAcc;
    uint16_t pDOP;
    uint8_t reserved1[6];
    int32_t headVeh;
    int16_t magDec;
    uint16_t magAcc;
};
#pragma pack(pop)

struct GPSData
{
    bool fix = false;
    int numSat = 0;
    int32_t lat = 0;     // 1e-7 deg
    int32_t lon = 0;     // 1e-7 deg
    int32_t alt = 0;     // mm
    int32_t velN = 0;    // mm/s
    int32_t velE = 0;    // mm/s
    int32_t velD = 0;    // mm/s
    int32_t gSpeed = 0;  // mm/s
    int32_t heading = 0; // 1e-5 deg
    bool valid = false;
    uint32_t intervalUs = 0;
    uint64_t lastFrameTime = 0;
};

class RPiUbloxGPS
{
public:
    RPiUbloxGPS(const std::string &device, int baudrate = 115200, int rateHz = 5)
        : fd_(-1), step_(0), payloadLen_(0), payloadCnt_(0), ckA_(0), ckB_(0)
    {
        payloadBuffer_.reserve(512);
        if (!openSerial(device, baudrate))
            throw std::runtime_error("Failed to open GPS port");
        init(rateHz);
    }

    ~RPiUbloxGPS()
    {
        if (fd_ >= 0)
            close(fd_);
    }

    GPSData parse()
    {
        data_.valid = false;
        uint8_t buf[1024];
        ssize_t n = read(fd_, buf, sizeof(buf));
        for (ssize_t i = 0; i < n; i++)
        {
            uint8_t b = buf[i];
            switch (step_)
            {
            case 0:
                if (b == 0xB5)
                    step_++;
                break;
            case 1:
                if (b == 0x62)
                    step_++;
                else if (b == 0xB5)
                    step_ = 1;
                else
                    step_ = 0;
                break;
            case 2:
                msgClass_ = b;
                ckA_ = ckB_ = b;
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
                if (payloadLen_ > 512)
                {
                    step_ = 0;
                    break;
                }
                payloadBuffer_.resize(payloadLen_);
                step_ = (payloadLen_ > 0) ? 6 : 7;
                break;
            case 6:
                payloadBuffer_[payloadCnt_++] = b;
                ckA_ += b;
                ckB_ += ckA_;
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
                    if (msgClass_ == 0x01 && msgID_ == 0x07 && payloadBuffer_.size() == sizeof(ubx_nav_pvt))
                    {
                        ubx_nav_pvt *p = reinterpret_cast<ubx_nav_pvt *>(payloadBuffer_.data());
                        data_.numSat = p->numSV;
                        data_.fix = (p->fixType >= 3);
                        data_.lon = p->lon;
                        data_.lat = p->lat;
                        data_.alt = p->hMSL;
                        data_.velN = p->velN;
                        data_.velE = p->velE;
                        data_.velD = p->velD;
                        data_.gSpeed = p->gSpeed;
                        data_.heading = p->headMot;
                        struct timeval tv;
                        gettimeofday(&tv, NULL);
                        uint64_t now = (uint64_t)tv.tv_sec * 1000000 + tv.tv_usec;
                        if (data_.lastFrameTime > 0)
                            data_.intervalUs = now - data_.lastFrameTime;
                        data_.lastFrameTime = now;
                        data_.valid = true;
                    }
                }
                step_ = 0;
                break;
            }
        }
        return data_;
    }

private:
    int fd_, step_;
    uint8_t msgClass_, msgID_, ckA_, ckB_;
    uint16_t payloadLen_, payloadCnt_;
    std::vector<uint8_t> payloadBuffer_;
    GPSData data_;

    bool openSerial(const std::string &device, int baudrate)
    {
        fd_ = open(device.c_str(), O_RDWR | O_NOCTTY);
        if (fd_ < 0)
            return false;
        struct termios2 options;
        if (ioctl(fd_, TCGETS2, &options) < 0)
            return false;
        options.c_cflag &= ~CBAUD;
        options.c_cflag |= BOTHER;
        options.c_ispeed = options.c_ospeed = baudrate;
        options.c_cflag |= (CLOCAL | CREAD);
        options.c_cflag &= ~(PARENB | CSTOPB | CSIZE | CRTSCTS);
        options.c_cflag |= CS8;
        options.c_lflag = options.c_iflag = options.c_oflag = 0;
        options.c_cc[VMIN] = 0;
        options.c_cc[VTIME] = 1;
        if (ioctl(fd_, TCSETS2, &options) < 0)
            return false;
        tcflush(fd_, TCIOFLUSH);
        return true;
    }

    void init(int hz)
    {
        for (int i = 0; i < 6; i++)
        {
            uint8_t p[3] = {0xF0, (uint8_t)i, 0};
            sendUBX(0x06, 0x01, p, 3);
        }
        uint16_t period = 1000 / hz;
        uint8_t r[6] = {(uint8_t)(period & 0xFF), (uint8_t)(period >> 8), 0x01, 0x00, 0x01, 0x00};
        sendUBX(0x06, 0x08, r, 6);
        uint8_t n[36] = {0};
        n[0] = 0x01;
        n[2] = 8;
        sendUBX(0x06, 0x24, n, 36);
        uint8_t m[3] = {0x01, 0x07, 1};
        sendUBX(0x06, 0x01, m, 3);
    }

    void sendUBX(uint8_t cls, uint8_t id, uint8_t *payload, uint16_t len)
    {
        std::vector<uint8_t> pkt = {0xB5, 0x62, cls, id, (uint8_t)(len & 0xFF), (uint8_t)(len >> 8)};
        for (int i = 0; i < len; ++i)
            pkt.push_back(payload[i]);
        uint8_t a = 0, b = 0;
        for (size_t i = 2; i < pkt.size(); ++i)
        {
            a += pkt[i];
            b += a;
        }
        pkt.push_back(a);
        pkt.push_back(b);
        write(fd_, pkt.data(), pkt.size());
    }
};