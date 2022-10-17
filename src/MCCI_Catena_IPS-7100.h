/*

Module: MCCI_Catena_IPS-7100.h

Function:
    Top-level include file for MCCI Catena IPS-7100 library.

Copyright and License:
    See accompanying LICENSE file for copyright and license information.

Author:
    Pranau R, MCCI Corporation   October 2022

*/

/// \file

#ifndef _MCCI_CATENA_IPS_7100_H_
#define _MCCI_CATENA_IPS_7100_H_ /* prevent multiple includes */

#pragma once

#include <cstdint>
#include <Wire.h>

namespace McciCatenaIps7100 {

// create a version number for comparison
static constexpr std::uint32_t
makeVersion(
    std::uint8_t major, std::uint8_t minor, std::uint8_t patch, std::uint8_t local = 0
    )
    {
    return ((std::uint32_t)major << 24u) | ((std::uint32_t)minor << 16u) | ((std::uint32_t)patch << 8u) | (std::uint32_t)local;
    }

// extract major number from version
static constexpr std::uint8_t
getMajor(std::uint32_t v)
    {
    return std::uint8_t(v >> 24u);
    }

// extract minor number from version
static constexpr std::uint8_t
getMinor(std::uint32_t v)
    {
    return std::uint8_t(v >> 16u);
    }

// extract patch number from version
static constexpr std::uint8_t
getPatch(std::uint32_t v)
    {
    return std::uint8_t(v >> 8u);
    }

// extract local number from version
static constexpr std::uint8_t
getLocal(std::uint32_t v)
    {
    return std::uint8_t(v);
    }

// version of library, for use by clients in static_asserts
static constexpr std::uint32_t kVersion = makeVersion(1,0,0,1);

class cIPS7100
    {
private:
    static constexpr bool kfDebug = false;

public:
// the address type:
    enum class Address : std::int8_t
        {
        Error = -1,
        IPS7100 = 0x4b,
        };

    // the type for pin assignments, in case the ready pin is used
    using Pin_t = std::int8_t;

    // constructor
    cIPS7100(TwoWire &wire, Address Address = Address::IPS7100, Pin_t pinReady = -1)
        : m_wire(&wire)
        , m_address(Address)
        , m_pinReady(pinReady)
        {}

    // neither copyable nor movable
    cIPS7100(const cIPS7100&) = delete;
    cIPS7100& operator=(const cIPS7100&) = delete;
    cIPS7100(const cIPS7100&&) = delete;
    cIPS7100& operator=(const cIPS7100&&) = delete;

    // I2C commands
    enum class Command : std::int16_t
        {
        // sorted in ascending numerical order.
        StartContinuousMeasurement              =   0x10,       // Takes n= 1, 2 and 3 for 200ms, 500ms and 1,000ms respectivly. Takes 0 for stop measurement
        ReadPC                                  =   0x11,
        ReadPM                                  =   0x12,
        SetCleaningInterval                     =   0x21,
        CommandMode                             =   0x22,
        PowerSavingMode                         =   0x23,
        SetDataUnit                             =   0x24,
        SetVth                                  =   0x26,
        SetVref                                 =   0x29,
        SetFan                                  =   0x2B,
        StartCleaning                           =   0x2C,
        Reset                                   =   0x2D,
        FactoryReset                            =   0x2E,
        ReadCleaningInterval                    =   0x61,
        ReadMode                                =   0x62,
        ReadDataUnit                            =   0x64,
        ReadStart                               =   0x65,
        ReadVth                                 =   0x66,
        ReadVref                                =   0x69,
        ReadStatus                              =   0x6A,
        ReadSerialNumber                        =   0x77,
        ReadRevisionNumber                      =   0x78,
        ReadNetworkSerialKey                    =   0x79,
        };

    // the errors
    enum class Error : std::uint8_t
        {
        Success = 0,
        NoWire,
        CommandWriteFailed,
        CommandWriteBufferFailed,
        InternalInvalidParameter,
        I2cReadShort,
        I2cReadRequest,
        I2cReadLong,
        WakeupFailed,
        Busy,
        NotMeasuring,
        Crc,
        Uninitialized,
        };

    enum class State : std::uint8_t
        {
        Uninitialized,      /// this->begin() has never succeeded.
        End,                /// this->begin() succeeded, followed by this->end()
        Initial,            /// initial after begin [indeterminate]
        Idle,               /// idle (not measuring)
        Triggered,          /// continuous measurement running, no data available.
        Ready,              /// continuous measurement running, data availble.
        };

private:
    /// \brief table of error messages
    ///
    /// \internal
    // this is internal -- centralize it but require that clients call the
    // public method (which centralizes the strings and the search)
    static constexpr const char * const m_szErrorMessages =
        "Success\0"
        "NoWire\0"
        "CommandWriteFailed\0"
        "CommandWriteBufferFailed\0"
        "InternalInvalidParameter\0"
        "I2cReadShort\0"
        "I2cReadRequest\0"
        "I2cReadLong\0"
        "WakeupFailed\0"
        "Busy\0"
        "NotMeasuring\0"
        "Crc\0"
        "Uninitialized\0"
        ;

    /// \brief table of state names, '\0'-separated.
    ///
    /// \internal
    // this is internal -- centralize it but require that clients call the
    // public method (which centralizes the strings and the search)
    static constexpr const char * const m_szStateNames =
        "Uninitialized" "\0"
        "End"           "\0"
        "Initial"       "\0"
        "Idle"          "\0"
        "Triggered"     "\0"
        "Ready"         "\0"
        ;

public:
    cIPS7100();
    virtual ~cIPS7100();
    bool begin();
    void end();
    void updateData();
    unsigned long *getPCData();
    unsigned long getPC01Data();
    unsigned long getPC03Data();
    unsigned long getPC05Data();
    unsigned long getPC10Data();
    unsigned long getPC25Data();
    unsigned long getPC50Data();
    unsigned long getPC100Data();
    float *getPMData();
    float getPM01Data();
    float getPM03Data();
    float getPM05Data();
    float getPM10Data();
    float getPM25Data();
    float getPM50Data();
    float getPM100Data();
    int getVref();
    int getStatus();
    bool enableFan(bool);
    bool start();
    void enableDebug(bool);
    bool enablePowerSavingMode(bool);

protected:
    // void readRegister(unsigned char, int, uint8_t[], bool checksum = false);
    void readRegister(cIPS7100::Command, int, uint8_t[], bool checksum = false);
    // bool writeRegister(unsigned char, unsigned char);
    bool writeRegister(cIPS7100::Command, unsigned char);
    uint16_t getChecksum(uint8_t *byte, int);
    bool checkRunning()
        {
        if (! this->isRunning())
            return this->setLastError(Error::Uninitialized);
        else
            return true;
        }

private:
    TwoWire *m_wire;                /// pointer to bus to be used for this device
    std::uint32_t m_tReady;         /// estimated time next measurement will be ready (millis)
    unsigned long m_pcValues[7] = {0, 0, 0, 0, 0, 0, 0};
    float m_pmValues[7] = {0, 0, 0, 0, 0, 0, 0};
    Address m_address;              /// I2C address to be used
    Pin_t m_pinReady;               /// alert pin, or -1 if none.
    Error m_lastError;              /// last error.
    State m_state                   /// current state
        { State::Uninitialized };   // initially not yet started.

    static constexpr std::uint16_t getUint16BE(const std::uint8_t *p)
        {
        return (p[0] << 8) + p[1];
        }

    static constexpr std::int16_t getInt16BE(const std::uint8_t *p)
        {
        return std::int16_t((p[0] << 8) + p[1]);
        }

    static float getFloat32BE(const std::uint8_t *p);
    };

} // end namespace McciCatenaIps7100

#endif /* _MCCI_CATENA_IPS_7100_H_ */