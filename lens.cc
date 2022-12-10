#include "lens.h"

#include <iostream>

int lens::initmotor(int PortToUse) {
    int homed = 0; //Error handling
    // try {
    //     // 9600 Baud, No Parity, 8 Data bits, 1 stop bit, no flow control
    //     port.Open(nPortToUse, 9600, CSerialPort::Parity::NoParity, 8, CSerialPort::StopBits::OneStopBit, CSerialPort::FlowControl::NoFlowControl);

    //     //Clear the RX and TX buffers
    //     Sleep(50);
    //     port.Purge(PURGE_RXCLEAR | PURGE_TXCLEAR);
    //     Sleep(50);   

    //     COMMTIMEOUTS timeouts;
    //     timeouts.ReadTotalTimeoutConstant = 5000;
    //     timeouts.WriteTotalTimeoutConstant = 5000;
    //     port.SetTimeouts(timeouts);

    // }
    // catch (CSerialException& e) {
    
    //     ATLTRACE(_T("Unexpected CSerialPort exception, Error:%u\n"), e.m_dwError);
    //     UNREFERENCED_PARAMETER(e);        
    //     std::cout << "Serial port failed to initialize";

    // }
    // const char* pszBuf = "0so00000400\r\n";
    // port.Write(pszBuf, static_cast<DWORD>(strlen(pszBuf)));
    // Sleep(100);
    // const char* pszBuf2 = "0ho0\r\n";
    // port.Write(pszBuf2, static_cast<DWORD>(strlen(pszBuf2)));
    // Sleep(2000);
    // const char* pszBuf3 = "0ma00002E00\r\n";
    // port.Write(pszBuf3, static_cast<DWORD>(strlen(pszBuf3)));
    // Sleep(2000);
    // const char* pszBuf4 = "0vv25\r\n";
    // port.Write(pszBuf4, static_cast<DWORD>(strlen(pszBuf4)));
    // Sleep(1000);

    //dummy functions for testing

    std::cout << "Serial Port initialized!" << "\n";
    return 1;
}

void lens::mov_rel(double mmToMove) {
    // int rate = 1024; //1024 encoder pulses per mm
    // int mmUpperBound = 16;
    // double current_pos = get_pos();  //current position in encoder pulses
    // double move = rate * mmToMove; //encoder pulses to move
    // double new_pos = move + current_pos;

    // if (TRUE) {
    //     std::string pszBufStr = "0mr" + double2hexstr(mmToMove) + "\r\n";
    //     const char* pszBuf = pszBufStr.c_str();
        
    //     port.Write(pszBuf, static_cast<DWORD>(strlen(pszBuf))); 
        
    //     //Sleep(20);

    //     int wait_move = 0;
    //     if (wait_move == 1) {
    //         std::vector<char> rxBuf1;
    //         rxBuf1.resize(13); //11 data bits and then 2 for the /l /n which terminates every ASCII string

    //         int timeout = 0;
    //         const DWORD dwRead = 1;

    //         while (timeout < 50) {
    //             const DWORD dwRead = port.Read(rxBuf1.data(), static_cast<DWORD>(rxBuf1.size()));
    //             if (dwRead == 13) {
    //                 break;
    //             }
    //             ++timeout;
    //         }
    //         //std::cout << "Motor stopped";
    //         if (timeout == 50) {
    //             printf("mov_rel function timed out");
    //         }
    //     }
    // }
    // else { std::cout << "Out of bounds"; }

};


double lens::get_pos() {
    // const char* pszBuf = "0gp\r\n";
    // port.Write(pszBuf, static_cast<DWORD>(strlen(pszBuf)));

    // std::vector<char> rxBuf;
    // rxBuf.resize(13); //11 data bits and then 2 for the /l /n which terminates every ASCII string
    
    // const DWORD dwRead = port.Read(rxBuf.data(), static_cast<DWORD>(rxBuf.size()));
    // /*int timeout = 0;
    // const DWORD dwRead = 0;
    // do {
    //     const DWORD dwRead = port.Read(rxBuf.data(), static_cast<DWORD>(rxBuf.size()));
    //     ++timeout;
    // } while (dwRead < 11 && timeout < 5000);

    // if (timeout == 5000) {
    //     printf("Get_pos function timed out");
    //     return 0;
    // }*/
    // rxBuf.resize(11); //gets rid of the /l and /n
    // std::string position{ rxBuf.begin(), rxBuf.end() };
    // std::string hex_pos = position.substr(7, 4);
    // double positiond = hexstr2double(hex_pos);
    // port.TerminateOutstandingWrites();
    // return positiond;

    std::cout << "lens position is <THIS IS A TEST>" << "\n";
    return 2.0;
}
  
  //Converts a hexadecimal string to a double and back again
double lens::hexstr2double (const std::string& hexstr) {
    return 1.0;
}
std::string lens::double2hexstr(double x) {
    return "hello";
}


void lens::closemotor() {
    // port.ClearWriteBuffer();
    // port.ClearReadBuffer();
    // port.Flush();
    // port.CancelIo();
    // port.Close();
    
    //Dummy
    std::cout << "Motor Closed" <<"\n";
}