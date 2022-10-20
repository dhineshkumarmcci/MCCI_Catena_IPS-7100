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
    this->setState(State::Initial);

    bool result = this->startMeasurement();
    return result;
    }

bool cIPS7100::startMeasurement()
    {
    bool result = this->writeCommand(Command::StartStop, 1);
    this->setState(State::Triggered);
    return result;
    }

void cIPS7100::end()
    {
    if (this->isRunning())
        this->setState(State::End);
    }

bool cIPS7100::readResponse(cIPS7100::Command command, int nBuffer, uint8_t *pBuffer, bool checksum)
    {
    bool checksumPass = false;
    unsigned nResult;
    uint8_t nReadFrom;

    if (! this->setState(State::Triggered));
        {
        return this->setLastError(Error::NotMeasuring);
        }

    while (!checksumPass)
        {
        this->m_wire->beginTransmissions(std::uint8_t(this->m_address));
        this->m_wire->write(command);

        if (this->m_wire->endTransmission() != 0)
            {
            return this->setLastError(Error::CommandWriteFailed);
            }

        nReadFrom = this->m_wire->requestFrom((uint8_t) Address::IPS7100, (uint8_t) nBuffer);

        if (nReadFrom != nBuffer)
            {
            return this->setLastError(Error::I2cReadRequest);
            }

        nResult = this->m_wire->available();

        if (nResult > nBuffer)
            return this->setLastError(Error::I2cReadLong);

        for (unsigned n = 0; n < nResult; n++)
            {
            pBuffer[n] = this->m_wire->.read();
            }

        if (nResult != nBuffer)
            {
            return this->setLastError(Error::I2cReadShort);
            }

        // Debug raw bytes
        if (this->isDebug())
            {
            Serial.print("[ ");

            for (unsigned n = 0; n < nResult; n++)
                {
                Serial.print(pBuffer[n]);
                Serial.print(" ");
                }

            Serial.print("]\n");
            }

        if (!checksum)
            {
            break;
            }

        uint16_t messageChecksum = this->get_checksum(pBuffer, nBuffer - 2);
        uint16_t receivedChecksum = (pBuffer[nBuffer - 2] * 256) + pBuffer[nBuffer - 1];

        if (this->isDebug())
            {
            Serial.print("Expected checksum: ");
            Serial.print(messageChecksum);

            Serial.print("Received checksum: ");
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

boolean cIPS7100::writeCommand(cIPS7100::Command command, unsigned char value)
    {
    this->m_wire->beginTransmissions(std::uint8_t(this->m_address));
    this->m_wire->write(command);
    this->m_wire->write(value);

    if (this->m_wire->endTransmission() != 0)
        {
        return this->setLastError(Error::CommandWriteFailed);
        }

    return true;
    }

void cIPS7100::updateData()
    {
    // Read PC data
    uint8_t pcRawValues[30];

    this->readResponse(Command::ReadPC, sizeof(pcRawValues), pcRawValues, true);

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

    this->readResponse(Command::ReadPM, sizeof(pmRawValues), pmRawValues, true);

    // Assemble PM values (float) from 4 bytes via union
    for (size_t i = 0; i < 7; ++i)
        {
        bytesToPM PM;
        for (size_t j = 0; j < 4; ++j)
            {
            PM.byte[j] = pmRawValues[j + (i * 4)];
            }

        this->m_pmValues[i] = PM.value;
        }
    }

// Get CRC16 checksum
uint16_t cIPS7100::getChecksum(uint8_t *byte, int length)
    {
    if (byte == nullptr)
        {
        return this->setLastError(Error::InternalInvalidParameter);
        }

    int i, j;
    uint16_t data = 0;
    uint16_t crc = 0xffff;

    for (j = 0; j < length; j++)
        {
        data = (uint16_t)0xff & byte[j];
        for (i = 0; i < 8; i++, data >>= 1)
            {
            if ((crc & 0x0001) ^ (data & 0x0001))
                crc = (crc >> 1) ^ CRC16;
            else
                crc >>= 1;
            }
        }

    crc = ~crc;
    data = crc;
    crc = (crc << 8) | (data >> 8 & 0xff);

    return crc;
    }

unsigned long *cIPS7100::getPCData()
    {
    return this->m_pcValues;
    };

unsigned long cIPS7100::getPC01Data()
    {
    return this->m_pcValues[0];
    };

unsigned long cIPS7100::getPC03Data()
    {
    return this->m_pcValues[1];
    };

unsigned long cIPS7100::getPC05Data()
    {
    return this->m_pcValues[2];
    };

unsigned long cIPS7100::getPC10Data()
    {
    return this->m_pcValues[3];
    };

unsigned long cIPS7100::getPC25Data()
    {
    return this->m_pcValues[4];
    };

unsigned long cIPS7100::getPC50Data()
    {
    return this->m_pcValues[5];
    };

unsigned long cIPS7100::getPC100Data()
    {
    return this->m_pcValues[6];
    };

float *cIPS7100::getPMData()
    {
    return this->m_pmValues;
    };

float cIPS7100::getPM01Data()
    {
    return this->m_pmValues[0];
    }

float cIPS7100::getPM03Data()
    {
    return this->m_pmValues[1];
    }

float cIPS7100::getPM05Data()
    {
    return this->m_pmValues[2];
    }

float cIPS7100::getPM10Data()
    {
    return this->m_pmValues[3];
    }

float cIPS7100::getPM25Data()
    {
    return this->m_pmValues[4];
    }

float cIPS7100::getPM50Data()
    {
    return this->m_pmValues[5];
    }

float cIPS7100::getPM100Data()
    {
    return this->m_pmValues[6];
    }

int cIPS7100::getVref()
    {
    // Read Vref
    uint8_t data[4];
    unsigned short int vref;

    this->readResponse(Command::ReadVref, sizeof(data), data, true);

    vref = data[1] | (data[0] << 8);
    return vref;
    }

int cIPS7100::getStatus()
    {
    // Read Status
    uint8_t data[3];
    unsigned short int status;

    this->readResponse(Command::ReadStatus, sizeof(data), data, true);

    status = data[0];
    return status;
    }

bool cIPS7100::setDataUnit(int unit)
    {
    // Set data unit
    uint8_t data[3];
    unsigned short int unit;

    this->read_i2c(Command::SetDataUnit, sizeof(data), data, true);

    unit = data[0];
    return unit;
    }

int cIPS7100::getDataUnit()
    {
    // Get data unit for PC and PM values
    uint8_t data[3];
    unsigned short int unit;

    this->read_i2c(Command::ReadDataUnit, 3, data, true);

    unit = data[0];
    return unit;
    }

void cIPS7100::getSerial(uint8_t* data)
    {
    this->read_i2c(Command::ReadSerialNumber, 19, data, true);
    data[17] = 0;
    }

void cIPS7100::getVersion(uint8_t* data)
    {
    this->read_i2c(Command::ReadRevisionNumber, 9, data, true);
    data[7] = 0;
    }

bool cIPS7100::enableFan(bool status)
    {
    bool result;
    // Enable or disable fan
    if (status)
        {
        result = this->writeCommand(Command::SetFan, 1);
        }
    else
        {
        result = this->writeCommand(Command::SetFan, 0);
        }

    return result;
    }

bool cIPS7100::enablePowerSavingMode(bool status)
    {
    bool result;
    // Enable or disable power saving mode
    if (status)
        {
        result = this->writeCommand(Command::PowerSavingMode, 1);
        }
    else
        {
        result = this->writeCommand(Command::PowerSavingMode, 0);
        }

    return result;
    }

bool cIPS7100::enableCleaning(bool status)
    {
    bool result;
    // Enable or disable cleaning
    if (status)
        {
        result = this->writeCommand(Command::SetCleaning, 1);
        }
    else
        {
        result = this->writeCommand(Command::SetCleaning, 0);
        }

    return result;
    }

/**** end of MCCI_Catena_IPS-7100.cpp ****/