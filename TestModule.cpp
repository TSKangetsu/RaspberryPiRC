#include <unistd.h>
#include <unistd.h>
#include <iostream>
#include <iomanip>
#include <thread>
#include <csignal>
#include <wiringPi.h>
#include <wiringSerial.h>
#include "RPiGPS/RPiGPS.hpp"
#include "RPiSBus/RPiSBus.hpp"
#include "RPiIBus/RPiIBus.hpp"
#include "RPiFlow/RPiFlow.hpp"

int signalIn = 0;

int main(int argc, char *argv[])
{
    int argvs;
    wiringPiSetup();
    while ((argvs = getopt(argc, argv, "vhi:s:g:G:cCf:")) != -1)
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
                time = micros();
                lose = IbusT->IbusRead(ChannelsData, 4000, 2);
                if (lose != -1)
                {
                    for (size_t i = 0; i < 14; i++)
                    {
                        std::cout << ChannelsData[i] << " ";
                    }
                    timee = micros();
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
                time = micros();
                lose = newSBUS.SbusRead(ChannelsData, 4700, 2);
                if (lose != -1)
                {
                    std::cout << lose << " ";
                    for (size_t i = 0; i < 16; i++)
                    {
                        std::cout << ChannelsData[i] << " ";
                    }
                    timee = micros();
                    std::cout << timee - time << " ";
                    std::cout << "\n";
                }
            }
        }
        break;

        case 'g':
        {
            long int time;
            long int timee;
            std::string GPSData;
            std::string dataP[10];
            GPSUart myUart(optarg);
            myUart.GPSReOpen();
            while (true)
            {
                if (myUart.GPSCheckDataAvaliable())
                {
                    long int timees = micros();
                    std::cout << "\nlast frame time Get : " << timees - time << "\n";
                    myUart.GPSRead(GPSData);
                    std::cout << GPSData;
                    time = micros();
                }
                usleep(180000);
            }
        }
        break;

        case 'G':
        {
            long int time;
            long int timee;
            std::string GPSData;
            GPSUartData mydata;
            GPSUart *myUart = new GPSUart(optarg);
            myUart->GPSReOpen();
            while (true)
            {
                time = micros();
                mydata = myUart->GPSParse();
                std::cout << "satillites: " << mydata.satillitesCount << " ";
                std::cout << "DataError: " << mydata.DataUnCorrect << " ";
                std::cout << "lat: " << std::setprecision(9) << mydata.lat << " ";
                std::cout << "lng: " << std::setprecision(10) << mydata.lng << " \n";
                std::cout << "ALT: " << std::setprecision(4) << mydata.GPSAlititude << "M "
                          << "HDOP " << std::setprecision(4) << mydata.HDOP << " "
                          << "Quailty: " << mydata.GPSQuality << " "
                          << "GeoidalSP: " << mydata.GPSGeoidalSP << "\n";
                long int timees = micros();
                std::cout << "last frame time : " << timees - time << "\n";
                timee = micros();
                if ((timee - time) > 200000)
                    usleep(1500);
                else
                    usleep(200000 - (timee - time));
            }
        }
        break;

        case 'C':
        {

            std::signal(SIGINT, [](int signal) {
                signalIn = signal;
            });
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
            GPSI2CCompass mycompassTest("/dev/i2c-1", 0x0d, COMPASS_QMC5883L);
            for (size_t i = 0; i < 2000; i++)
            {
                mycompassTest.CompassGetRaw(rawx, rawy, rawz);
                std::cout << rawx << " " << rawy << " " << rawz << " " << sqrt(rawx * rawx + rawy * rawy + rawz * rawz) << "\n";
            }

            while (true)
            {
                mycompassTest.CompassGetRaw(rawx, rawy, rawz);
                std::cout << rawx << " " << rawy << " " << rawz << " " << sqrt(rawx * rawx + rawy * rawy + rawz * rawz) << "\n";
                mycompassTest.CompassCalibration(true, calibration);
                usleep(((float)1 / 200 * 1000000));
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
            GPSI2CCompass mycompassTest("/dev/i2c-1", 0x0d, COMPASS_QMC5883L);
            mycompassTest.CompassApply(3104, 2965, 2338, 2080, 1288, 972);
            while (true)
            {
                mycompassTest.CompassGetRaw(rawx, rawy, rawz);
                mycompassTest.CompassGetUnfixAngle(angleUnfix);
                std::cout << angleUnfix << "\n";
                std::cout << rawx << " " << rawy << " " << rawz << "\n";
                usleep(((float)1 / 200 * 1000000));
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
            int qulity;
            std::string myData;
            MSPUartFlow myUart(optarg);
            while (true)
            {
                int Status = myUart.MSPDataRead(x, y, alt, qulity);
                std::cout << "x:" << x << " \n";
                std::cout << "y:" << y << " \n";
                std::cout << "alt:" << alt << " \n";
                std::cout << "FLowq:" << qulity << " \n";
                std::cout << "Status:" << Status << " \n";
                timee = micros();
                std::cout << "last frame time : " << timee - time << "\n";
                time = micros();
                if ((timee - time) > 35000)
                    usleep(50);
                else
                    usleep(35000 - (timee - time));
            }
        }
        break;
        }
    }
}
