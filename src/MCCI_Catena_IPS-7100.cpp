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

    bool result = this->reset();
    return result;
    }

bool cIPS7100::startMeasurement(uint8_t timer)
    {
    bool result = this->writeCommand(Command::StartStop, timer);
    this->setState(State::Triggered);

    return result;
    }

void cIPS7100::end()
    {
    if (this->isRunning())
        {
        this->setState(State::End);
        }
    }

bool cIPS7100::reset()
    {
    if (this->writeCommand(cIPS7100::Command::SoftReset))
        {
        delay(5000);
        return true;
        }
    else
        return false;
    }

bool cIPS7100::readResponse(cIPS7100::Command command, size_t nBuffer, std::uint8_t *pBuffer, bool checksum)
    {
    bool checksumPass = false;

    auto const state = this->getState();
    if (state != State::Triggered)
        {
        return this->setLastError(Error::NotMeasuring);
        }

    while (!checksumPass)
        {
        this->m_wire->beginTransmission(std::uint8_t(this->m_address));
        this->m_wire->write(std::uint8_t(command));

        if (this->m_wire->endTransmission() != 0)
            {
            return this->setLastError(Error::CommandWriteFailed);
            }

        auto nReadFrom = this->m_wire->requestFrom((uint8_t) Address::IPS7100, (uint8_t) nBuffer);

        if (nReadFrom != nBuffer)
            {
            return this->setLastError(Error::I2cReadRequest);
            }

        auto const nResult = unsigned (this->m_wire->available());

        if (nResult > nBuffer)
            {
            return this->setLastError(Error::I2cReadLong);
            }

        for (unsigned i = 0; i < nResult; ++i)
            {
            pBuffer[i] = this->m_wire->read();
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

        uint16_t messageChecksum = this->getChecksum(pBuffer, nBuffer - 2);
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
            // Checksum failed
            if (this->isDebug())
                {
                Serial.println("Checksum Failed.");
                }

            delay(100);
            }
        }
    }

// to write reset
bool cIPS7100::writeCommand(cIPS7100::Command command)
    {
    this->m_wire->beginTransmission(std::uint8_t(this->m_address));
    this->m_wire->write(std::uint8_t(command));

    if (this->m_wire->endTransmission() != 0)
        {
        return this->setLastError(Error::CommandWriteFailed);
        }

    return true;
    }

bool cIPS7100::writeCommand(cIPS7100::Command command, std::uint8_t value)
    {
    this->m_wire->beginTransmission(std::uint8_t(this->m_address));
    this->m_wire->write(std::uint8_t(command));
    this->m_wire->write(value);

    if (this->m_wire->endTransmission() != 0)
        {
        return this->setLastError(Error::CommandWriteFailed);
        }

    return true;
    }

// to write cleaning interval
bool cIPS7100::writeCommand(cIPS7100::Command command, std::uint32_t value)
    {
    this->m_wire->beginTransmission(std::uint8_t(this->m_address));
    this->m_wire->write(std::uint8_t(command));
    this->m_wire->write((std::uint8_t)((value >> 24) & 0xFF));
    this->m_wire->write((std::uint8_t)((value >> 16) & 0xFF));
    this->m_wire->write((std::uint8_t)((value >> 8) & 0xFF));
    this->m_wire->write((std::uint8_t)(value & 0xFF));

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

    // Assemble PC values (uint32_t) from 4 bytes via bitwise
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
    for (unsigned i = 0; i < 7; ++i)
        {
        bytesToPM PM;
        for (unsigned j = 0; j < 4; ++j)
            {
            PM.pByte[j] = pmRawValues[j + (i * 4)];
            }

        this->m_pmValues[i] = PM.value;
        }

    // Get event status
    this->m_eventStatus = (pmRawValues[28] * 256) + pmRawValues[29];
    }

// Get CRC16 checksum
uint16_t cIPS7100::getChecksum(uint8_t *pByte, uint8_t length)
    {
    if (pByte == nullptr)
        {
        return this->setLastError(Error::InternalInvalidParameter);
        }

    unsigned i, j;
    uint16_t data = 0;
    uint16_t crc = 0xffff;

    for (j = 0; j < length; j++)
        {
        data = (uint16_t)0xff & pByte[j];
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

std::uint32_t *cIPS7100::getPCData()
    {
    return this->m_pcValues;
    };

std::uint32_t cIPS7100::getPC01Data()
    {
    return this->m_pcValues[0];
    };

std::uint32_t cIPS7100::getPC03Data()
    {
    return this->m_pcValues[1];
    };

std::uint32_t cIPS7100::getPC05Data()
    {
    return this->m_pcValues[2];
    };

std::uint32_t cIPS7100::getPC10Data()
    {
    return this->m_pcValues[3];
    };

std::uint32_t cIPS7100::getPC25Data()
    {
    return this->m_pcValues[4];
    };

std::uint32_t cIPS7100::getPC50Data()
    {
    return this->m_pcValues[5];
    };

std::uint32_t cIPS7100::getPC100Data()
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

uint16_t cIPS7100::getEventStatus()
    {
    return this->m_eventStatus;
    }

std::uint16_t cIPS7100::getVref()
    {
    // Read Vref
    uint8_t pData[4];
    uint16_t vref;

    this->readResponse(Command::ReadVref, sizeof(pData), pData, true);

    vref = pData[1] | (pData[0] << 8);
    return vref;
    }

std::uint16_t cIPS7100::getStatus()
    {
    // Read Status
    uint8_t pData[3];
    uint16_t status;

    this->readResponse(Command::ReadStatus, sizeof(pData), pData, true);

    status = pData[0];
    return status;
    }

bool cIPS7100::setDataUnit(std::uint8_t unit)
    {
    // Set data unit
    bool result;

    if (unit >= 0 && unit <= 3)
        {
        result = this->writeCommand(Command::SetDataUnit, unit);
        }
    else
        {
        result = this->setLastError(Error::NoWire);
        }

    return result;
    }

std::uint16_t cIPS7100::getDataUnit()
    {
    // Get data unit for PC and PM values
    uint8_t pData[3];
    uint16_t unit;

    this->readResponse(Command::ReadDataUnit, sizeof(pData), pData, true);

    unit = pData[0];
    return unit;
    }

bool cIPS7100::setCleaningInterval(std::uint32_t interval)
    {
    bool result = this->writeCommand(Command::SetCleaningInterval, interval);

    return result;
    }

std::uint32_t cIPS7100::getCleaningInterval()
    {
    uint8_t rawCleanValues[4];
    this->readResponse(Command::ReadCleaningInterval, sizeof(rawCleanValues), rawCleanValues, true);

    // Assemble PC values (uint32_t) from 4 bytes via bitwise
    return rawCleanValues[3] | (rawCleanValues[2] << 8) | (rawCleanValues[1] << 16) | (rawCleanValues[0] << 24);
    };

void cIPS7100::getSerial(uint8_t* pData)
    {
    this->readResponse(Command::ReadSerialNumber, sizeof(pData), pData, true);
    pData[17] = 0;
    }

void cIPS7100::getVersion(uint8_t* pData)
    {
    this->readResponse(Command::ReadRevisionNumber, sizeof(pData), pData, true);
    pData[7] = 0;
    }

bool cIPS7100::enableFan(bool status)
    {
    bool result;
    // Enable or disable fan
    if (status)
        {
        result = this->writeCommand(Command::SetFan, (std::uint8_t)1);
        }
    else
        {
        result = this->writeCommand(Command::SetFan, (std::uint8_t)0);
        }

    return result;
    }

bool cIPS7100::enablePowerSavingMode(bool status)
    {
    bool result;
    // Enable or disable power saving mode
    if (status)
        {
        result = this->writeCommand(Command::PowerSavingMode, (std::uint8_t)1);
        this->setState(State::Idle);
        }
    else
        {
        result = this->writeCommand(Command::PowerSavingMode, (std::uint8_t)0);
        this->setState(State::Triggered);
        }

    return result;
    }

bool cIPS7100::enableCleaning(bool status)
    {
    bool result;
    // Enable or disable cleaning
    if (status)
        {
        result = this->writeCommand(Command::SetCleaning, (std::uint8_t)1);
        }
    else
        {
        result = this->writeCommand(Command::SetCleaning, (std::uint8_t)0);
        }

    return result;
    }

/**** end of MCCI_Catena_IPS-7100.cpp ****/