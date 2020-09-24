#include <unistd.h>
#include <unistd.h>
#include <iostream>
#include <wiringPi.h>
#include "RPiSBus/RPiSBus.hpp"
#include "RPiIBus/RPiIBus.hpp"
int main()
{
    int ChannelsData[14];
    int sbusData[32];
    int lose;
    long int time;
    long int timee;

    Ibus Ibus("/dev/ttyAMA0");
    while (true)
    {
        time = micros();
        lose = Ibus.IbusRead(ChannelsData, 4000, 2);
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

// int main(void)
// {
//     wiringPiSetup();
//     long int time;
//     long int timee;

//     int ChannelsData[16];
//     int sbusData[25];
//     int lose;
//     Sbus newSBUS("/dev/ttyAMA0", SbusMode::Normal);
//     //Sbus newSBUS("/dev/ttyS0", SbusMode::HighSpeed);

//     // this is HighSpeed mode example

//     //while (true)
//     //{
//     //	lose = newSBUS.SbusQuickRead();
//     //	if (lose == 15)
//     //	{
//     //		time = micros();
//     //		for (size_t i = 0; i < 25; i++)
//     //		{
//     //			sbusData[i] = newSBUS.SbusQuickRead();
//     //			usleep(50);
//     //		}
//     //		newSBUS.SbusPaser(sbusData, ChannelsData);
//     //		timee = micros();
//     //		for (size_t i = 0; i < 16; i++)
//     //		{
//     //			std::cout << ChannelsData[i] << " ";
//     //		}
//     //		std::cout << timee - time << " ";
//     //		std::cout << " \n";
//     //	}
//     //}

//     // this is Normal mode example

//     while (true)
//     {
//         time = micros();
//         lose = newSBUS.SbusRead(ChannelsData, 4700, 2);
//         if (lose != -1)
//         {
//             std::cout << lose << " ";
//             for (size_t i = 0; i < 16; i++)
//             {
//                 std::cout << ChannelsData[i] << " ";
//             }
//             timee = micros();
//             std::cout << timee - time << " ";
//             std::cout << "\n";
//         }
//     }
// }