#pragma once
#include <windows.h>
#include <string>
#include <iostream>

class Serial
{
private:
    HANDLE hSerial; // 串口句柄
    bool connected;
    COMMTIMEOUTS timeouts;

public:
    Serial(const char* portName, DWORD baudRate)
        : connected(false), hSerial(NULL), timeouts({ 0 })
    {
        std::cout << "正在打开串口: " << portName << "..." << std::endl;
        hSerial = CreateFileA(portName,
            GENERIC_READ | GENERIC_WRITE,
            0,
            NULL,
            OPEN_EXISTING,
            FILE_ATTRIBUTE_NORMAL,
            NULL);
        if (hSerial == INVALID_HANDLE_VALUE) {
            if (GetLastError() == ERROR_FILE_NOT_FOUND) {
                std::cerr << "错误: 串口 " << portName << " 未找到。" << std::endl;
            }
            else {
                std::cerr << "错误: 打开串口失败，错误代码: " << GetLastError() << std::endl;
            }
            return;
        }

        // --- [!!! 关键修复：防止 STM32 复位 !!!] ---
        // 某些开发板会利用 DTR/RTS 引脚进行复位。
        // 我们显式地清除这两个信号，确保板子正常运行。
        EscapeCommFunction(hSerial, CLRDTR); // 清除 DTR
        EscapeCommFunction(hSerial, CLRRTS); // 清除 RTS
        // ---------------------------------------

        DCB dcbSerialParams = { 0 };
        if (!GetCommState(hSerial, &dcbSerialParams)) {
            std::cerr << "错误: 获取串口状态失败。" << std::endl;
            CloseHandle(hSerial);
            return;
        }

        dcbSerialParams.BaudRate = baudRate;
        dcbSerialParams.ByteSize = 8;
        dcbSerialParams.StopBits = ONESTOPBIT;
        dcbSerialParams.Parity = NOPARITY;

        // 再次确保 DTR/RTS 控制被禁用
        dcbSerialParams.fDtrControl = DTR_CONTROL_DISABLE;
        dcbSerialParams.fRtsControl = RTS_CONTROL_DISABLE;

        if (!SetCommState(hSerial, &dcbSerialParams)) {
            std::cerr << "错误: 设置串口状态失败。" << std::endl;
            CloseHandle(hSerial);
            return;
        }

        timeouts.ReadIntervalTimeout = 50;
        timeouts.ReadTotalTimeoutConstant = 50;
        timeouts.ReadTotalTimeoutMultiplier = 10;
        timeouts.WriteTotalTimeoutConstant = 50;
        timeouts.WriteTotalTimeoutMultiplier = 10;
        if (!SetCommTimeouts(hSerial, &timeouts)) {
            std::cerr << "错误: 设置串口超时失败。" << std::endl;
            CloseHandle(hSerial);
            return;
        }

        connected = true;
        std::cout << "串口 " << portName << " 打开成功 (DTR/RTS 已清除)。" << std::endl;
    }

    ~Serial()
    {
        if (connected) {
            CloseHandle(hSerial);
            std::cout << "串口已关闭。" << std::endl;
        }
    }

    bool isConnected()
    {
        return connected;
    }

    bool write(std::string data)
    {
        if (!connected) return false;
        DWORD bytesSend;
        if (!WriteFile(hSerial, data.c_str(), (DWORD)data.length(), &bytesSend, 0)) {
            std::cerr << "错误: 写入串口失败。" << std::endl;
            return false;
        }
        return true;
    }
};