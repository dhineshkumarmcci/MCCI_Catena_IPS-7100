/*

Module: MCCI_Catena_IPS-7100.cpp

Function:
    Implementation code for MCCI Catena IPS-7100 sensor library.

Copyright and License:
    See accompanying LICENSE file for copyright and license information.

Author:
    Pranau R, MCCI Corporation   October 2022

*/

#include "MCCI_Catena_IPS-7100.h"
#include <Arduino.h>
#include <Wire.h>

using namespace McciCatenaIps7100;

cIPS7100::cIPS7100()
    {
    //default constructor
    }

cIPS7100::~cIPS7100()
    {
    }

bool cIPS7100::begin()
    {
  	// if no Wire is bound, fail.
    if (this->m_wire == nullptr)
        {
        return this->setLastError(Error::NoWire);
        }

    if (this->isRunning())
        {
        return true;
        }

    this->m_wire->begin();
    this->m_state = this->m_state == State::End ? State::Triggered : State::Initial;

    return this->start();
    }

bool cIPS7100::start()
    {
    bool result;
    result = this->writeRegister(Command::StartContinuousMeasurement, 1);
    return result;
    }

void cIPS7100::end()
    {
    if (this->isRunning())
        this->m_state = State::End;
    }

void cIPS7100::readRegister(cIPS7100::Command command, int nBuf, uint8_t *buf, bool checksum)
    {
    bool checksumPass = false;
    unsigned nResult;
    uint8_t nReadFrom;

    while (!checksumPass)
        {
        this->m_wire->beginTransmissions(std::uint8_t(this->m_address));
        this->m_wire->write(command);

        if (this->m_wire->endTransmission() != 0)
            {
            return this->setLastError(Error::CommandWriteFailed);
            }

        nReadFrom = this->m_wire->requestFrom((uint8_t) Address::IPS7100, (uint8_t) nBuf);

        if (nReadFrom != nBuf)
            {
            return this->setLastError(Error::I2cReadRequest);
            }

        nResult = this->m_wire->available();

        if (nResult > nBuf)
            return this->setLastError(Error::I2cReadLong);

        for (unsigned n = 0; n < nResult; n++)
            {
            buf[n] = this->m_wire->.read();
            }

        if (nResult != nBuf)
            {
            return this->setLastError(Error::I2cReadShort);
            }

        // Debug raw bytes
        if (this->isDebug())
            {
            Serial.print("[ ");

            for (unsigned n = 0; n < nResult; n++)
                {
                Serial.print(buf[n]);
                Serial.print(" ");
                }

            Serial.print("]\n");
            }

        if (!checksum)
            {
            break;
            }

        uint16_t messageChecksum = this->get_checksum(buf, nBuf - 2);
        uint16_t receivedChecksum = (buf[nBuf - 2] * 256) + buf[nBuf - 1];

        if (this->isDebug())
            {
            Serial.print("Expected checksum: ");
            Serial.print(messageChecksum);

            Serial.print(" Received checksum: ");
            Serial.print(receivedChecksum);
            Serial.print("\n");
            }

        if (messageChecksum == receivedChecksum)
            {
            checksumPass = true;
            }
        else
            {
            // Checksum failed;
            if (this->isDebug())
                {
                Serial.println("Checksum Failed.");
                }

            delay(100);
            }
        }
    }

boolean cIPS7100::writeRegister(cIPS7100::Command command, unsigned char parameter)
    {
    this->m_wire->beginTransmissions(std::uint8_t(this->m_address));
    this->m_wire->write(command);
    this->m_wire->write(parameter);

    if (this->m_wire->endTransmission() != 0)
        {
        return this->setLastError(Error::CommandWriteFailed);
        }

    return true;
    }

void cIPS7100::update()
    {
    // Read PC data
    uint8_t pcRawValues[30];

    this->readRegister(cIPS7100::Command::ReadPC, sizeof(pcRawValues), pcRawValues, true);

    // Assemble PC values (unsigned long) from 4 bytes via bitwise
    this->m_pcValues[0] = pcRawValues[3] | (pcRawValues[2] << 8) | (pcRawValues[1] << 16) | (pcRawValues[0] << 24);
    this->m_pcValues[1] = pcRawValues[7] | (pcRawValues[6] << 8) | (pcRawValues[5] << 16) | (pcRawValues[4] << 24);
    this->m_pcValues[2] = pcRawValues[11] | (pcRawValues[10] << 8) | (pcRawValues[9] << 16) | (pcRawValues[8] << 24);
    this->m_pcValues[3] = pcRawValues[15] | (pcRawValues[14] << 8) | (pcRawValues[13] << 16) | (pcRawValues[12] << 24);
    this->m_pcValues[4] = pcRawValues[19] | (pcRawValues[18] << 8) | (pcRawValues[17] << 16) | (pcRawValues[16] << 24);
    this->m_pcValues[5] = pcRawValues[23] | (pcRawValues[22] << 8) | (pcRawValues[21] << 16) | (pcRawValues[20] << 24);
    this->m_pcValues[6] = pcRawValues[27] | (pcRawValues[26] << 8) | (pcRawValues[25] << 16) | (pcRawValues[24] << 24);

    // Read PM data
    uint8_t pmRawValues[32];

    this->readRegister(cIPS7100::Command::ReadPM, sizeof(pmRawValues), pmRawValues, true);

    // Assemble PM values (float) from 4 bytes via union
    for (size_t i = 0; i < 7; ++i)
        {
        bytesToPM b;
        for (size_t j = 0; j < 4; ++j)
            {
            b.byte[j] = pmRawValues[j + (i * 4)];
            }

        this->m_pmValues[i] = b.f;
        }
    }

// Get CRC16 checksum
uint16_t cIPS7100::get_checksum(uint8_t *byte, int length)
    {
    int i, j;
    uint16_t data = 0;
    uint16_t crc = 0xffff;
    for (j = 0; j < length; j++)
        {
        data = (uint16_t)0xff & byte[j];
        for (i = 0; i < 8; i++, data >>= 1)
            {
            if ((crc & 0x0001) ^ (data & 0x0001))
                crc = (crc >> 1) ^ POLY;
            else
                crc >>= 1;
            }
        }

    crc = ~crc;
    data = crc;
    crc = (crc << 8) | (data >> 8 & 0xff);
    return crc;
    }

/**** end of MCCI_Catena_IPS-7100.cpp ****/