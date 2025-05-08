// #include <iostream>
// #include "Xeryon.h"

// #include <stdio.h>

// int main() {
//     //Xeryon * controller = new Xeryon("/dev/tty.usbmodem14201", 115200);
//     //Xeryon * controller = new Xeryon("/dev/tty.usbserial-FTBT792D", 115200);
//     Xeryon * controller = new Xeryon("/dev/ttyACM0", 115200);
//     if (controller == nullptr) {
//         controller = new Xeryon("/dev/ttyACM1", 115200);
//     }
//     Axis * axisX = controller->addAxis('X', &XLA_1250_3N);
//     //Axis * axisY = controller->addAxis('Y', &XLS_312);
//     //Axis * axisZ = controller->addAxis('Z', &XLS_312);
//     printf("axis DPOS %d\n", axisX->getData("DPOS"));
//     controller->start();
//     axisX->findIndex();

//     printf("axis DPOS %d\n", axisX->getData("DPOS"));
//     printf("axis EPOS %d\n", axisX->getData("EPOS"));
//     printf("axis STAT %d\n", axisX->getData("STAT"));
	
//     axisX->setDPOS(5_mm);
//     axisX->setDPOS(-5_mm);
//     axisX->setDPOS(50_mu);
//     printf("DPOS: %lf\n", (double)axisX->getDPOS());
//     printf("EPOS: %lf\n", (double)axisX->getEPOS());
//     axisX->setDPOS(0_mm);
//     for (int i = 0; i < 5; i++)
//         axisX->step(1_mm);

//     axisX->setDPOS(0_mm);
//     axisX->setSpeed(1_mm);
//     axisX->startScan(-1);

//     axisX->stopScan();
//     axisX->setDPOS(0_mm);

//     axisX->startScan(1, 2);

//     axisX->setSpeed(50_mm);
//     axisX->setDPOS(0_mm);
//     axisX->setSpeed(120_mm);

//     // axisX.startLogging();
//     axisX->step(0.01_mm);
//     // logs = axisX.endLogging();
//     axisX->setSpeed(50_mm);
//     axisX->setDPOS(0_mm);

//     std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    
//     controller->stop();
//     return 0;
// }
