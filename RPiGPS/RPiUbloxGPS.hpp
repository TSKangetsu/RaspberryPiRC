#pragma once

#include <iostream>
#include <vector>
#include <string>
#include <cstring>
#include <cstdio>
#include <cstdint>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <asm-generic/ioctls.h>
#include <asm/termbits.h>
#include <thread>
#include <chrono>
#include <sys/time.h>

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
        : device_(device), fd_(-1), step_(0)
    {
        if (!openSerial(baudrate))
            throw std::runtime_error("Failed to open GPS serial port");
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
        if (fd_ < 0)
            return data_;

        uint8_t buf[2048];
        int n = read(fd_, buf, sizeof(buf));

        if (n > 0)
        {
            for (int i = 0; i < n; i++)
            {
                if (processByte(buf[i]))
                {
                    if (msgClass_ == 0x01 && msgID_ == 0x07)
                    {
                        struct timeval tv;
                        gettimeofday(&tv, NULL);
                        uint64_t now = (uint64_t)tv.tv_sec * 1000000 + tv.tv_usec;
                        if (data_.lastFrameTime > 0)
                            data_.intervalUs = now - data_.lastFrameTime;
                        data_.lastFrameTime = now;
                        data_.valid = true;
                    }
                }
            }
        }
        return data_;
    }

private:
    std::string device_;
    int fd_;
    int step_;
    uint8_t msgClass_, msgID_;
    uint16_t payloadLen_, payloadCnt_;
    uint8_t ckA_, ckB_;
    std::vector<uint8_t> payloadBuffer_;
    GPSData data_;

    bool openSerial(int baudrate)
    {
        fd_ = open(device_.c_str(), O_RDWR | O_NOCTTY);
        if (fd_ < 0)
            return false;

        struct termios2 options;
        if (ioctl(fd_, TCGETS2, &options) < 0)
            return false;

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
        options.c_iflag &= ~(IXON | IXOFF | IXANY | ICRNL | INLCR | IGNCR);
        options.c_oflag &= ~OPOST;

        if (ioctl(fd_, TCSETS2, &options) < 0)
            return false;
        return true;
    }

    void init(int hz)
    {
        for (int i = 0; i < 6; i++)
            configureMsg(0xF0, i, 0);
        uint16_t period = 1000 / hz;
        uint8_t rate[6] = {(uint8_t)(period & 0xFF), (uint8_t)(period >> 8), 0x01, 0x00, 0x01, 0x00};
        sendUBX(0x06, 0x08, rate, 6);
        uint8_t nav5[36] = {0};
        nav5[0] = 0x01;
        nav5[1] = 0x00;
        nav5[2] = 8;
        sendUBX(0x06, 0x24, nav5, 36);
        configureMsg(0x01, 0x07, 1);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    void configureMsg(uint8_t msgClass, uint8_t msgID, uint8_t rate)
    {
        uint8_t payload[3] = {msgClass, msgID, rate};
        sendUBX(0x06, 0x01, payload, 3);
    }

    void sendUBX(uint8_t msgClass, uint8_t msgID, uint8_t *payload, uint16_t len)
    {
        std::vector<uint8_t> pkt;
        pkt.push_back(0xB5);
        pkt.push_back(0x62);
        pkt.push_back(msgClass);
        pkt.push_back(msgID);
        pkt.push_back(len & 0xFF);
        pkt.push_back((len >> 8) & 0xFF);
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

    bool processByte(uint8_t b)
    {
        switch (step_)
        {
        case 0:
            if (b == 0xB5)
                step_++;
            break;
        case 1:
            if (b == 0x62)
                step_++;
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
            payloadBuffer_.resize(payloadLen_);
            step_ = (payloadLen_ > 0) ? 6 : 7;
            if (payloadLen_ > 1024)
                step_ = 0;
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
                parseMessage();
                step_ = 0;
                return true;
            }
            step_ = 0;
            break;
        }
        return false;
    }

    int32_t readI32(const uint8_t *p)
    {
        return (int32_t)(p[0] | (p[1] << 8) | (p[2] << 16) | (p[3] << 24));
    }

    void parseMessage()
    {
        const uint8_t *p = payloadBuffer_.data();
        if (msgClass_ == 0x01 && msgID_ == 0x07 && payloadLen_ >= 92)
        {
            data_.numSat = p[23];
            data_.fix = (p[20] >= 3);
            data_.lon = readI32(&p[24]);
            data_.lat = readI32(&p[28]);
            data_.alt = readI32(&p[36]);
            data_.velN = readI32(&p[48]);
            data_.velE = readI32(&p[52]);
            data_.velD = readI32(&p[56]);
            data_.gSpeed = readI32(&p[60]);
            data_.heading = readI32(&p[64]);
        }
    }
};