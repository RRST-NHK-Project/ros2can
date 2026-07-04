// Copyright © 2023 Being24. MIT.
#ifndef SDM15_H_
#define SDM15_H_

#include <Arduino.h>

struct VersionInfo {
    bool checksum_error;
    int model;
    int hardware_version;
    int firmware_version_major;
    int firmware_version_minor;
    String serial_number;
};

struct TestResult {
    bool checksum_error;
    bool self_check_result;
    byte self_check_error_code;
    byte test_data[32];
};

struct ScanData {
    bool checksum_error;
    int distance;
    int intensity;
    int disturb;
};

const int PACKET_HED1 = 0xAA;
const int PACKET_HED2 = 0x55;
const int START_SCAN = 0x60;
const int STOP_SCAN = 0x61;
const int GET_DEVICE_INFO = 0x62;
const int SELF_TEST = 0x63;
const int SET_OUTPUT_FREQ = 0x64;
const int SET_FILTER = 0x65;
const int SET_SERIAL_BAUD = 0x66;
const int SET_FORMAT_OUTPUT_DATA = 0x67;
const int RESTORE_FACTORY_SETTINGS = 0x68;
const int NO_DATA = 0x00;

enum Freq {
    Freq_10Hz = 0x00,
    Freq_100Hz = 0x01,
    Freq_200Hz = 0x02,
    Freq_500Hz = 0x03,
    Freq_1000Hz = 0x04,
    Freq_1800Hz = 0x05
};

enum FilterHex {
    Off = 0x00,
    On = 0x01
};

enum OutputDataFormatHex {
    Standard = 0x00,
    Pixhawk = 0x01
};

enum BaudRateHex {
    BAUD_230400 = 0x00,
    BAUD_460800 = 0x01,
    BAUD_512000 = 0x02,
    BAUD_921600 = 0x03,
    BAUD_1500000 = 0x04
};

class SDM15 {
public:
    explicit SDM15(HardwareSerial &serial) : _sensor_serial(serial) {}
    VersionInfo ObtainVersionInfo();
    TestResult SelfCheckTest();
    bool StartScan();
    bool StopScan();
    ScanData GetScanData();
    bool SetOutputFrequency(Freq freq);
    bool SetFilter(FilterHex filter);
    bool SetBaudRate(BaudRateHex baud);
    bool SetOutputDataFormat(OutputDataFormatHex format);
    bool RestoreFactorySettings();

private:
    byte CalculateChecksum(byte *recv, int size);
    void ClearBuffer();
    HardwareSerial &_sensor_serial;
};

#endif // SDM15_H_