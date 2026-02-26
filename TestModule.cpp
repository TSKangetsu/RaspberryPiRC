#include <unistd.h>
#include <iostream>
#include <iomanip>
#include <thread>
#include <csignal>
#include <sys/time.h>
#include "RPiGPS/RPiGPS.hpp"
#include "RPiGPS/RPiUbloxGPS.hpp"
#include "RPiSBus/RPiSBus.hpp"
#include "RPiIBus/RPiIBus.hpp"
#include "RPiFlow/RPiFlow.hpp"
#include "CRSF/CRSFUartRC.hpp"
#include "Drive_Json.hpp"
#include <fstream>

int signalIn = 0;
int TimestartUpLoad = 0;

double configSettle(const char *configDir, const char *Target);

int GetTimestamp()
{
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return ((tv.tv_sec * (uint64_t)1000000 + tv.tv_usec));
}

int main(int argc, char *argv[])
{
    int argvs;
    TimestartUpLoad = GetTimestamp();
    while ((argvs = getopt(argc, argv, "vhi:s:g:G:c:C:f:R:")) != -1)
    {
        switch (argvs)
        {
        case 'v':
            std::cout << "[RPiSingleAPM] version 1.0.f Beta , Acess By TSKangetsu\n"
                      << "	checkout : https://github.com/TSKangetsu/RPiSingleAPM \n";
            break;
        case 'h':
            std::cout << "-i /dev/ttyS0 for Ibus\n"
                      << "-s /dev/ttyS0 for Sbus\n"
                      << "-g /dev/ttyS0 for M8NGPS\n"
                      << "-G /dev/ttyS0 for M8NGPS Parsed Data\n"
                      << "-c for QML5883 Compass\n"
                      << "-f /dev/ttyS0 for MatekSys 3901-L0X\n";
            break;
        case 'i':
        {
            int ChannelsData[14];
            int sbusData[32];
            int lose;
            long int time;
            long int timee;
            Ibus *IbusT;
            IbusT = new Ibus(optarg);
            while (true)
            {
                time = GetTimestamp() - TimestartUpLoad;
                lose = IbusT->IbusRead(ChannelsData, 4000, 2);
                if (lose != -1)
                {
                    for (size_t i = 0; i < 14; i++)
                    {
                        std::cout << ChannelsData[i] << " ";
                    }
                    timee = GetTimestamp() - TimestartUpLoad;
                    std::cout << "T: " << timee - time;
                    std::cout << " \n";
                }
            }
        }
        break;

        case 's':
        {
            long int time;
            long int timee;

            int ChannelsData[16];
            int sbusData[25];
            int lose;
            Sbus newSBUS(optarg);
            while (true)
            {
                time = GetTimestamp() - TimestartUpLoad;
                lose = newSBUS.SbusRead(ChannelsData, 4700, 2);
                if (lose != -1)
                {
                    std::cout << lose << " ";
                    for (size_t i = 0; i < 16; i++)
                    {
                        std::cout << ChannelsData[i] << " ";
                    }
                    timee = GetTimestamp() - TimestartUpLoad;
                    std::cout << timee - time << " ";
                    std::cout << "\n";
                }
            }
        }
        break;

        case 'G':
        {
            int baudrate = 115200;
            int rateHz = 5;
            GPSData mydata;
            RPiUbloxGPS myUart(optarg, baudrate, rateHz);
            system("clear");
            while (true)
            {
                mydata = myUart.parse();
                if (mydata.valid)
                {
                    double freq = (mydata.intervalUs > 0) ? (1000000.0 / mydata.intervalUs) : 0;
                    std::cout << "\033[H";
                    std::cout << "--- DRONE GPS NAV MONITOR [" << rateHz << "Hz] ---\n";
                    std::cout << "Sats: " << std::setw(2) << mydata.numSat
                              << " | Fix: " << (int)mydata.fix
                              << " | Freq: " << std::fixed << std::setprecision(1) << freq << " Hz    \n";
                    std::cout << "-------------------------------------\n";
                    // Raw lat/lon is 1e-7 deg
                    std::cout << "Pos LLA: " << std::fixed << std::setprecision(8)
                              << (mydata.lat / 10000000.0) << ", " << (mydata.lon / 10000000.0) << "    \n";
                    // Raw alt is mm, convert to cm
                    std::cout << "Alt MSL: " << std::fixed << std::setprecision(1) << (mydata.alt / 10.0) << " cm    \n";
                    std::cout << "-------------------------------------\n";
                    // Raw velocity is mm/s, convert to cm/s
                    std::cout << "Velocity NED (cm/s):\n";
                    std::cout << "  North: " << std::setw(7) << std::setprecision(1) << (mydata.velN / 10.0)
                              << " | East: " << std::setw(7) << std::setprecision(1) << (mydata.velE / 10.0)
                              << " | Down: " << std::setw(7) << std::setprecision(1) << (mydata.velD / 10.0) << "    \n";
                    std::cout << "-------------------------------------\n";
                    std::cout << "\033[K";
                }
                usleep(100);
            }
        }
        break;

        case 'C':
        {
        }
        break;

        case 'c':
        {
            int rawx = 0;
            int rawy = 0;
            int rawz = 0;
            double angleUnfix = 0;
            int flip[3] = {0, 12, 0};
            GPSI2CCompass mycompassTest(optarg, 0x0d, COMPASS_QMC5883L, flip);
            while (true)
            {
                mycompassTest.CompassGetRaw(rawx, rawy, rawz);
                mycompassTest.CompassGetUnfixAngle(angleUnfix);
                std::cout << "Angle: " << std::setw(7) << std::setfill(' ') << angleUnfix << "\n";
                std::cout << "X:" << std::setw(7) << std::setfill(' ') << rawx << " "
                          << "Y:" << std::setw(7) << std::setfill(' ') << rawy << " "
                          << "Z:" << std::setw(7) << std::setfill(' ') << rawz << " \n";
                if (signalIn == SIGINT)
                    break;
                usleep(50 * 1000);
            }
        }
        break;

        case 'f':
        {
            int error = 0;
            long int time;
            long int timee;
            int x;
            int y;
            int alt;
            int rfqulity;
            int opqulity;
            std::string myData;
            MSPUartFlow myUart(optarg);
            system("clear");
            while (true)
            {
                std::cout << "\n";
                std::cout << "\033[100A";
                std::cout << "\033[K";
                int Status = myUart.MSPDataRead(x, y, opqulity, alt, rfqulity);
                if (Status > 0)
                {
                    std::cout << "x:" << x << " \n";
                    std::cout << "y:" << y << " \n";
                    std::cout << "opquality:" << opqulity << " \n";
                    std::cout << "alt:" << alt << " \n";
                    std::cout << "rfquality:" << rfqulity << " \n";
                    std::cout << "Status:" << Status << " \n";
                    std::cout << "error: " << error << "\n";
                }
                else
                {
                    std::cout << "error frame recv\n";
                    error++;
                }
                timee = GetTimestamp() - TimestartUpLoad;
                std::cout << "last frame time : " << timee - time << "\n";
                time = GetTimestamp() - TimestartUpLoad;
                if ((timee - time) > 15000)
                    usleep(50);
                else
                    usleep(15000 - (timee - time));
            }
        }
        break;

        case 'R':
        {
            long int time;
            long int timee;
            CRSF test(optarg);
            int channelData[50];
            int st = 300;
            //
            std::thread telethread =
                std::thread(
                    [&]
                    {
                        while (true)
                        {
                            test.CRSFTelemtry(
                                CRSFTelemetry::crsfFrameBatterySensor(
                                    crsfProtocol::CRSF_SYNC_BYTE,
                                    160, 160, 800, 80));

                            test.CRSFTelemtry(
                                CRSFTelemetry::crsfFrameAttitude(
                                    crsfProtocol::CRSF_SYNC_BYTE,
                                    50, 150, 300));

                            test.CRSFTelemtry(CRSFTelemetry::crsfFrameFlightMode(crsfProtocol::CRSF_SYNC_BYTE, "HELO"));

                            test.CRSFTelemtry(
                                CRSFTelemetry::crsfFrameVarioSensor(
                                    crsfProtocol::CRSF_SYNC_BYTE, -20));

                            // FIXME: GPSheading should be unsign,but telemetry is sign, so..
                            test.CRSFTelemtry(CRSFTelemetry::crsfFrameGps(
                                crsfProtocol::CRSF_SYNC_BYTE,
                                100, -100, 10, 32800, 1005, 8));

                            usleep(200 * 1000);
                        }
                    });

            while (true)
            {
                time = GetTimestamp() - TimestartUpLoad;
                //
                int retValue = test.CRSFRead(channelData);
                if (retValue > 0)
                {
                    for (size_t i = 0; i < 15; i++)
                    {
                        std::cout << test.rcToUs(channelData[i]) << " ";
                    }
                    std::cout << "\n";
                }
                else
                {
                    std::cout << "error frame recived"
                              << "\n";
                }
                //
                timee = GetTimestamp() - TimestartUpLoad;
                std::cout << "ret: " << retValue
                          << " last frame time : " << timee - time << " "
                          << "\n";
            }
        }
        break;
        }
    }
}

double configSettle(const char *configDir, const char *Target)
{
    std::ifstream config(configDir);
    std::string content((std::istreambuf_iterator<char>(config)),
                        (std::istreambuf_iterator<char>()));
    nlohmann::json Configdata = nlohmann::json::parse(content);
    return Configdata[Target].get<double>();
}