#include <unistd.h>
#include <unistd.h>
#include <iostream>
#include <iomanip>
#include <thread>
#include <wiringPi.h>
#include <wiringSerial.h>
#include "RPiGPS/RPiGPS.hpp"
#include "RPiSBus/RPiSBus.hpp"
#include "RPiIBus/RPiIBus.hpp"

int main(int argc, char *argv[])
{
    int argvs;
    while ((argvs = getopt(argc, argv, "vhi:s:S:g:G:c")) != -1)
    {
        switch (argvs)
        {
        case 'v':
            std::cout << "[RPiSingleAPM] version 1.0.f Beta , Acess By TSKangetsu\n"
                      << "	checkout : https://github.com/TSKangetsu/RPiSingleAPM \n";
            break;
        case 'h':
            std::cout << "-i /dev/ttyS0 to test ibus, -s /dev/ttyS0 for Sbus normal,-S /dev/ttyS0 for sbus highspeed";
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
            wiringPiSetup();
            long int time;
            long int timee;

            int ChannelsData[16];
            int sbusData[25];
            int lose;
            Sbus newSBUS(optarg, SbusMode::Normal);
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

        case 'S':
        {
            wiringPiSetup();
            long int time;
            long int timee;

            int ChannelsData[16];
            int sbusData[25];
            int lose;

            Sbus newSBUS(optarg, SbusMode::HighSpeed);
            while (true)
            {
                lose = newSBUS.SbusQuickRead();
                if (lose == 15)
                {
                    time = micros();
                    for (size_t i = 0; i < 25; i++)
                    {
                        sbusData[i] = newSBUS.SbusQuickRead();
                        usleep(50);
                    }
                    newSBUS.SbusPaser(sbusData, ChannelsData);
                    timee = micros();
                    for (size_t i = 0; i < 16; i++)
                    {
                        std::cout << ChannelsData[i] << " ";
                    }
                    std::cout << timee - time << " ";
                    std::cout << " \n";
                }
            }
        }
        break;

        case 'g':
        {
            long int time;
            long int timee;
            std::string GPSData;
            GPSUart myUart(optarg);
            while (true)
            {
                time = micros();
                for (size_t i = 0; i < 6; i++)
                {
                    myUart.GPSRead(GPSData);
                    std::cout << GPSData;
                }
                timee = micros();
                std::cout << "last frame time : " << timee - time << "\n";
                if ((timee - time) > 150000)
                    usleep(200000);
                else
                    usleep(200000 - (timee - time));
            }
        }
        break;

        case 'G':
        {
            long int time;
            long int timee;
            int mal = 0;
            std::string GPSData;
            GPSUartData mydata;
            GPSUart *myUart = new GPSUart(optarg);
            sleep(5);
            std::thread test([&] {
                mal = 10;
                myUart->GPSReOpen();
                while (true)
                {
                    time = micros();
                    if (mal == 10)
                    {
                        mal = 0;
                        mydata = myUart->GPSParse();
                        std::cout << "satillites: " << mydata.satillitesCount << " ";
                        std::cout << "lat: " << std::setprecision(9) << mydata.lat << " ";
                        std::cout << "lng: " << std::setprecision(10) << mydata.lng << " \n";
                        long int timees = micros();
                        std::cout << "last frame time : " << timees - time << "\n";
                    }
                    timee = micros();
                    mal++;
                    if ((timee - time) > 15000)
                        usleep(1500);
                    else
                        usleep(20000 - (timee - time));
                }
            });
            test.join();
        }
        break;

        case 'c':
        {
            long rawx;
            long rawy;
            long rawz;
            GPSI2CCompass_QMC5883L mycompassTest;
            mycompassTest.GPSI2CCompass_QMC5883LInit();
            while (true)
            {
                mycompassTest.GPSI2CCompass_QMC5883LRead(rawx, rawy, rawz);
                std::cout << rawx << " " << rawy << " " << rawz << "\n";
            }
        }
        break;
        }
    }
}
