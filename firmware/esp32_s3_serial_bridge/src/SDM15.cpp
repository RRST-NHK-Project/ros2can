// Copyright © 2023 Being24. MIT.
#include "SDM15.h"

ScanData sdm15_data;
byte scan_recv[9];
byte checksum = 0;

byte SDM15::CalculateChecksum(byte *recv, int size) {
    checksum = 0;
    for (int i = 0; i < size - 1; i++) {
        checksum += recv[i];
    }
    checksum = checksum & 0xFF;
    return checksum;
}

void SDM15::ClearBuffer() {
    while (_sensor_serial.available() > 0) {
        _sensor_serial.read();
    }
}

VersionInfo SDM15::ObtainVersionInfo() {
    // clear buffer
    ClearBuffer();

    // send cmd : PACKET_HED1, PACKET_HED2, GET_DEVICE_INFO, NO_DATA, 0x61
    byte cmd[5] = {PACKET_HED1, PACKET_HED2, GET_DEVICE_INFO, NO_DATA, 0x61};
    _sensor_serial.write(cmd, sizeof(cmd));
    _sensor_serial.flush();

    byte recv[25];

    // receive byte data response : PACKET_HED1, PACKET_HED2, GET_DEVICE_INFO,
    // data_length, data[20], checksum
    _sensor_serial.readBytes(recv, sizeof(recv));

    // calculate checksum
    checksum = CalculateChecksum(recv, sizeof(recv));

    VersionInfo info;

    // check checksum
    if (checksum != recv[24]) {
        info.checksum_error = true;
        return info;
    }

    info.checksum_error = false;

    // int型で取得
    info.model = static_cast<int>(recv[4]);
    info.hardware_version = recv[5];
    info.firmware_version_major = recv[6];
    info.firmware_version_minor = recv[7];

    // serial_numberはrecvの8~23番目のデータを結合して取得
    // 1バイト1桁として結合する
    info.serial_number = "";
    for (int i = 8; i < 24; i++) {
        info.serial_number += String(recv[i], HEX);
    }

    // バージョン情報を返す
    return info;
}

TestResult SDM15::SelfCheckTest() {
    // clear buffer
    ClearBuffer();

    // send cmd : PACKET_HED1, PACKET_HED2, SELF_TEST, NO_DATA, 0x62
    byte cmd[5] = {PACKET_HED1, PACKET_HED2, SELF_TEST, NO_DATA, 0x62};
    _sensor_serial.write(cmd, sizeof(cmd));
    _sensor_serial.flush();

    byte recv[39];

    // receive byte data response : PACKET_HED1, PACKET_HED2, SELF_TEST,
    // data_length, data[34], checksum
    _sensor_serial.readBytes(recv, sizeof(recv));

    // calculate checksum
    checksum = CalculateChecksum(recv, sizeof(recv));

    TestResult test;

    // check checksum
    if (checksum != recv[38]) {
        test.checksum_error = true;
        return test;
    }

    test.checksum_error = false;

    if (recv[4] == 0x01)
        test.self_check_result = true;
    else
        test.self_check_result = false;

    // int型で取得
    test.self_check_error_code = recv[5];

    return test;
}

bool SDM15::StartScan() {
    // clear buffer
    ClearBuffer();

    // send cmd : PACKET_HED1, PACKET_HED2, START_SCAN, NO_DATA, 0x5F
    byte cmd[5] = {PACKET_HED1, PACKET_HED2, START_SCAN, NO_DATA, 0x5F};
    _sensor_serial.write(cmd, sizeof(cmd));
    _sensor_serial.flush();

    byte recv[5];

    // receive byte data response : PACKET_HED1, PACKET_HED2, START_SCAN, NO_DATA,
    // checksum
    if (_sensor_serial.readBytes(recv, sizeof(recv)) != sizeof(recv)) {
        return false;
    }

    // calculate checksum
    checksum = CalculateChecksum(recv, sizeof(recv));

    // check checksum
    if (checksum != recv[4])
        return false;
    else
        return true;
}

bool SDM15::StopScan() {
    // clear buffer
    ClearBuffer();

    // send cmd : PACKET_HED1, PACKET_HED2, STOP_SCAN, NO_DATA, 0x60
    byte cmd[5] = {PACKET_HED1, PACKET_HED2, STOP_SCAN, NO_DATA, 0x60};
    _sensor_serial.write(cmd, sizeof(cmd));
    _sensor_serial.flush();

    byte recv[5];

    // receive byte data response : PACKET_HED1, PACKET_HED2, STOP_SCAN, NO_DATA,
    // checksum
    _sensor_serial.readBytes(recv, sizeof(recv));

    // calculate checksum
    checksum = CalculateChecksum(recv, sizeof(recv));

    // check checksum
    if (checksum != recv[4])
        return false;
    else
        return true;
}

ScanData SDM15::GetScanData() {
    // receive byte data response : PACKET_HED1, PACKET_HED2, START_SCAN,
    // data_length, data[4], checksum
    if (_sensor_serial.readBytes(scan_recv, sizeof(scan_recv)) != sizeof(scan_recv)) {
        sdm15_data.checksum_error = true;
        return sdm15_data;
    }

    // calculate checksum
    checksum = CalculateChecksum(scan_recv, sizeof(scan_recv));

    // check checksum
    if (checksum != scan_recv[8]) {
        sdm15_data.checksum_error = true;
        return sdm15_data;
    }

    sdm15_data.checksum_error = false;

    sdm15_data.distance =
        static_cast<int>(scan_recv[4]) + (static_cast<int>(scan_recv[5]) << 8);
    sdm15_data.intensity = static_cast<int>(scan_recv[6]);
    sdm15_data.disturb = static_cast<int>(scan_recv[7]);

    return sdm15_data;
}

bool SDM15::SetOutputFrequency(Freq freq = Freq_100Hz) {
    // clear buffer
    ClearBuffer();

    // send cmd : PACKET_HED1, PACKET_HED2, SET_OUTPUT_FREQUENCY, data_length,
    // Freq,checksum
    byte cmd[6] = {PACKET_HED1, PACKET_HED2, SET_OUTPUT_FREQ,
                   0x01, freq, NO_DATA};
    byte send_checksum = CalculateChecksum(cmd, sizeof(cmd));
    cmd[5] = send_checksum;
    _sensor_serial.write(cmd, sizeof(cmd));
    _sensor_serial.flush();

    byte recv[6];

    // receive byte data response : PACKET_HED1, PACKET_HED2,
    // SET_OUTPUT_FREQUENCY, data_length, freq, checksum
    _sensor_serial.readBytes(recv, sizeof(recv));

    // calculate checksum
    byte recv_checksum = CalculateChecksum(recv, sizeof(recv));

    // check checksum
    if (recv_checksum != recv[5])
        return false;
    else
        return true;
}

bool SDM15::SetFilter(FilterHex filter = On) {
    // clear buffer
    ClearBuffer();

    // send cmd : PACKET_HED1, PACKET_HED2, SET_FILTER, data_length, filter,
    // checksum
    byte cmd[6] = {PACKET_HED1, PACKET_HED2, SET_FILTER,
                   0x01, filter, NO_DATA};
    byte send_checksum = CalculateChecksum(cmd, sizeof(cmd));
    cmd[5] = send_checksum;
    _sensor_serial.write(cmd, sizeof(cmd));
    _sensor_serial.flush();

    byte recv[6];

    // receive byte data response : PACKET_HED1, PACKET_HED2,
    // SET_FILTERING, data_length, filter, checksum
    _sensor_serial.readBytes(recv, sizeof(recv));

    // calculate checksum
    byte recv_checksum = CalculateChecksum(recv, sizeof(recv));

    // check checksum
    if (recv_checksum != recv[5])
        return false;
    else
        return true;
}

bool SDM15::SetBaudRate(BaudRateHex baud_rate = BAUD_460800) {
    // clear buffer
    ClearBuffer();

    // send cmd : PACKET_HED1, PACKET_HED2, SET_SERIAL_BAUD, data_length, baud,
    // checksum
    byte cmd[6] = {PACKET_HED1, PACKET_HED2, SET_SERIAL_BAUD,
                   0x01, baud_rate, NO_DATA};
    byte send_checksum = CalculateChecksum(cmd, sizeof(cmd));
    cmd[5] = send_checksum;
    _sensor_serial.write(cmd, sizeof(cmd));
    _sensor_serial.flush();

    byte recv[6];

    // receive byte data response : PACKET_HED1, PACKET_HED2,
    // SET_SERIAL_BAUD, data_length, baud, checksum
    _sensor_serial.readBytes(recv, sizeof(recv));

    // calculate checksum
    byte recv_checksum = CalculateChecksum(recv, sizeof(recv));

    // check checksum
    if (recv_checksum != recv[5])
        return false;
    else
        return true;
}

bool SDM15::SetOutputDataFormat(OutputDataFormatHex data_format = Standard) {
    // clear buffer
    ClearBuffer();

    // send cmd : PACKET_HED1, PACKET_HED2, SET_FORMAT_OUTPUT_DATA, data_length,
    // data_format, checksum
    byte cmd[6] = {PACKET_HED1, PACKET_HED2, SET_FORMAT_OUTPUT_DATA,
                   0x01, data_format, NO_DATA};
    byte send_checksum = CalculateChecksum(cmd, sizeof(cmd));
    cmd[5] = send_checksum;
    _sensor_serial.write(cmd, sizeof(cmd));
    _sensor_serial.flush();

    byte recv[6];

    // receive byte data response : PACKET_HED1, PACKET_HED2,
    // SET_FORMAT_OUTPUT_DATA, data_length, data_format, checksum
    _sensor_serial.readBytes(recv, sizeof(recv));

    // calculate checksum
    byte recv_checksum = CalculateChecksum(recv, sizeof(recv));

    // check checksum
    if (recv_checksum != recv[5])
        return false;
    else
        return true;
}

bool SDM15::RestoreFactorySettings() {
    // clear buffer
    ClearBuffer();

    // send cmd : PACKET_HED1, PACKET_HED2, RESTORE_FACTORY_SETTINGS, NO_DATA,
    // checksum
    byte cmd[5] = {PACKET_HED1, PACKET_HED2, RESTORE_FACTORY_SETTINGS,
                   0x00, 0x67};
    byte send_checksum = CalculateChecksum(cmd, sizeof(cmd));
    cmd[4] = send_checksum;
    _sensor_serial.write(cmd, sizeof(cmd));
    _sensor_serial.flush();

    byte recv[5];

    // receive byte data response : PACKET_HED1, PACKET_HED2,
    // RESTORE_FACTORY_SETTINGS, NO_DATA, checksum
    if (_sensor_serial.readBytes(recv, sizeof(recv)) != sizeof(recv)) {
        return false;
    }

    // calculate checksum
    byte recv_checksum = CalculateChecksum(recv, sizeof(recv));

    // check checksum
    if (recv_checksum != recv[4])
        return false;
    else
        return true;
}