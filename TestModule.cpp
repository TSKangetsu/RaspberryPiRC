#include <unistd.h>
#include <iostream>
#include <iomanip>
#include <thread>
#include <csignal>
#include <sys/time.h>
#include "RPiGPS/RPiGPS.hpp"
#include "RPiSBus/RPiSBus.hpp"
#include "RPiIBus/RPiIBus.hpp"
#include "RPiFlow/RPiFlow.hpp"
#include "CRSF/CRSFUartRC.hpp"

int signalIn = 0;
int TimestartUpLoad = 0;

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

        case 'g':
        {
        }
        break;

        case 'G':
        {
        }
        break;

        case 'C':
        {

            std::signal(SIGINT, [](int signal)
                        { signalIn = signal; });
            int rawx = 0;
            int rawy = 0;
            int rawz = 0;
            int calibration[10];
            calibration[0] = -5000;
            calibration[2] = -5000;
            calibration[4] = -5000;
            calibration[1] = 5000;
            calibration[3] = 5000;
            calibration[5] = 5000;
            int flip[3] = {0, 12, 0};
            GPSI2CCompass mycompassTest(optarg, 0x0d, COMPASS_QMC5883L, flip);
            // mycompassTest.CompassCaliInit();

            while (true)
            {
                mycompassTest.CompassGetRaw(rawx, rawy, rawz);
                std::cout << "X:" << std::setw(7) << std::setfill(' ') << rawx << " "
                          << "Y:" << std::setw(7) << std::setfill(' ') << rawy << " "
                          << "Z:" << std::setw(7) << std::setfill(' ') << rawz << " "
                          << "V:" << std::setw(7) << std::setfill(' ') << (int)sqrt(rawx * rawx + rawy * rawy + rawz * rawz) << " \n";
                mycompassTest.CompassCalibration(true, calibration);
                usleep(50 * 1000);
                if (signalIn == SIGINT)
                    break;
            }
            std::cout << "\n\n";
            std::cout << "XMAX: " << calibration[0] << "\n";
            std::cout << "XMIN: " << calibration[1] << "\n";
            std::cout << "YMAX: " << calibration[2] << "\n";
            std::cout << "YMIN: " << calibration[3] << "\n";
            std::cout << "ZMAX: " << calibration[4] << "\n";
            std::cout << "ZMIN: " << calibration[5] << "\n";
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
            mycompassTest.CompassApply(3966, 647, 3517, 298, 573, -2574);
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
                }
                else
                {
                    std::cout << "error frame recv\n";
                }
                timee = GetTimestamp() - TimestartUpLoad;
                std::cout << "last frame time : " << timee - time << "\n";
                time = GetTimestamp() - TimestartUpLoad;
                if ((timee - time) > 35000)
                    usleep(50);
                else
                    usleep(35000 - (timee - time));
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
