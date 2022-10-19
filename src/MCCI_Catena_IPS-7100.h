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

/// \brief namespace for this library
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

/// \brief For CRC16 checksum
#define CRC16 0x8408

union bytesToPM
    {
    float f;
    unsigned char byte[4];
    };

/// \brief instance object for LTR-329als
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


    /// \brief Error codes
    enum class Error : std::uint8_t
        {
        Success = 0,
        NoWire,
        CommandWriteFailed,
        InternalInvalidParameter,
        I2cReadShort,
        I2cReadRequest,
        I2cReadLong,
        Busy,
        NotMeasuring,
        Crc,
        Uninitialized,
        };

    /// \brief state of the meaurement engine
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
    /// \brief the constructor
    cIPS7100();
    virtual ~cIPS7100();

    ///
    /// \brief Power up the IPS 7100 sensor and start operation.
    ///
    /// \return
    ///     \c true for success, \c false for failure (in which case the the last
    ///     error is set to the error reason).
    ///
    bool begin();

    /// \brief end operation;
    void end();

    /// \brief update fresh PC and PM data
    void updateData();

    /// \brief Return PC value
    float *getPCData();

    /// \brief Return PC value
    float getPC01Data();

    float getPC03Data();
    float getPC05Data();
    float getPC10Data();
    float getPC25Data();
    float getPC50Data();
    float getPC100Data();

    float *getPMData();
    float getPM01Data();
    float getPM03Data();
    float getPM05Data();
    float getPM10Data();
    float getPM25Data();
    float getPM50Data();
    float getPM100Data();

    /// \brief Return voltage reference
    int getVref();

    /// \brief Return current status of sensor 
    int getStatus();

    ///
    /// \brief Return true if the fan is enabled
    ///
    /// \param [in] status is the new state (TRUE or FALSE)
    ///
    bool enableFan(bool status);

    /// \brief Return true for a successful start register write
    bool startMeasurement();

    ///
    /// \brief Return true if the power saving mode is enabled
    ///
    /// \param [in] status is the new state (TRUE or FALSE)
    ///
    bool enablePowerSavingMode(bool status);

    /// \brief Return true if the debug is enabled
    static constexpr bool isDebug() { return kfDebug; }

protected:
    ///
    /// \brief read a series of bytes starting with a given register.
    ///
    /// \param [in] r indicates the starting register to be read.
    /// \param [out] pBuffer points to the buffer to receive the data
    /// \param [in] nBuffer is the number of bytes to read.
    ///
    /// \return
    ///     \c true for success, \c false for failure. The
    ///     last error is set in case of error.
    ///
    bool readRegister(cIPS7100::Command command, int nBuffer, std::uint8_t *pBuffer, bool checksum = false);

    ///
    /// \brief Write a byte to a given register.
    ///
    /// \param [in] command selects the register to write
    /// \param [in] value is the value to be written.
    ///
    /// \return
    ///     \c true for success, \c false for failure. The
    ///     last error is set in case of error.
    ///
    bool writeRegister(cIPS7100::Command command, unsigned char value);

    /// \brief Return checksum.
    ///
    /// \param [in] byte points to the buffer to receive the data
    /// \param [in] lenght is the number of bytes to read.
    ///
    uint16_t getChecksum(uint8_t *byte, int lenght);

    ///
    /// \brief Make sure the driver is running
    ///
    /// If not running, set last error to Error::Uninitialized, and return \c false.
    /// Otherwise return \c true.
    ///
    /// Normally used in the following pattern:
    ///
    /// \code
    ///     if (! this->checkRunning())
    ///         return false;
    ///     // otherwise do some work...
    /// \endcode
    ///
    bool checkRunning()
        {
        if (! this->isRunning())
            return this->setLastError(Error::Uninitialized);
        else
            return true;
        }

private:
    TwoWire *m_wire;                                        ///< pointer to bus to be used for this device
    std::uint32_t m_tReady;                                 ///< estimated time next measurement will be ready (millis)
    float m_pcValues[7] = {0, 0, 0, 0, 0, 0, 0};    ///< buffer to store PC values
    float m_pmValues[7] = {0, 0, 0, 0, 0, 0, 0};            ///< buffer to store PM values
    Address m_address;                                      ///< I2C address to be used
    Pin_t m_pinReady;                                       ///< alert pin, or -1 if none.
    Error m_lastError;                                      ///< last error.
    State m_state                                           ///< current state
        { State::Uninitialized };                           ///< initially not yet started.

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