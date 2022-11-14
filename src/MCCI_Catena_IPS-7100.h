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

union bytesToPM
    {
    float value;
    unsigned char byte[4];
    };

/// \brief namespace for this library
namespace McciCatenaIps7100 {

/// \brief create a version constant uint32_t
static constexpr std::uint32_t
makeVersion(
    std::uint8_t major, std::uint8_t minor, std::uint8_t patch, std::uint8_t local = 0
    )
    {
    return ((std::uint32_t)major << 24u) | ((std::uint32_t)minor << 16u) | ((std::uint32_t)patch << 8u) | (std::uint32_t)local;
    }

/// \brief extract major number from version
static constexpr std::uint8_t
getMajor(std::uint32_t v)
    {
    return std::uint8_t(v >> 24u);
    }

/// \brief extract minor number from version
static constexpr std::uint8_t
getMinor(std::uint32_t v)
    {
    return std::uint8_t(v >> 16u);
    }

/// \brief extract patch number from version
static constexpr std::uint8_t
getPatch(std::uint32_t v)
    {
    return std::uint8_t(v >> 8u);
    }

/// \brief extract local number from version
static constexpr std::uint8_t
getLocal(std::uint32_t v)
    {
    return std::uint8_t(v);
    }

/// \brief version of library, for use by clients in static_asserts
static constexpr std::uint32_t kVersion = makeVersion(1,0,0,1);

// for CRC16 checksum
#define CRC16 0x8408

/*union bytesToPM
    {
    float value;
    unsigned char byte[4];
    };*/

/// \brief instance object for IPS-7100 Sensor
class cIPS7100
    {
private:
    static constexpr bool kfDebug = false;

public:
    ///
    /// \brief the address type
    ///
    enum class Address : std::int8_t
        {
        Error = -1,
        IPS7100 = 0x4b,
        };

    // the type for pin assignments, in case the ready pin is used
    using Pin_t = std::int8_t;

    ///
    /// \brief the constructor
    ///
    /// \param [in] wire is the TwoWire bus to use for this sensor.
    /// \param [in] Address is the device address.
    /// \param [in] pinReady is the alert pin. It is set -1 if there is none
    ///
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

    /// \brief I2C commands
    enum class Command : std::uint8_t
        {
        // sorted in ascending numerical order.
        StartStop                               =   0x10,       ///< Takes n= 1, 2 and 3 for 200ms, 500ms and 1,000ms respectivly for start measurement. Takes 0 for stop measurement
        ReadPC                                  =   0x11,       ///< Reads PC value
        ReadPM                                  =   0x12,       ///< Reads PM value
        SetCleaningInterval                     =   0x21,       ///< Set cleaning interval
        PowerSavingMode                         =   0x23,       ///< Enable or disable power saving mode
        SetDataUnit                             =   0x24,       ///< Set PC and PM data unit
        SetVth                                  =   0x26,       ///< Set detection range control voltage 
        SetVref                                 =   0x29,       ///< Set sensitivity control voltage
        SetFan                                  =   0x2B,       ///< Enable or disable fan
        SetCleaning                             =   0x2C,       ///< Enable or disable cleaning
        Reset                                   =   0x2D,       ///< Resets the sensor module 
        FactoryReset                            =   0x2E,       ///< Restore all factory default settings
        ReadCleaningInterval                    =   0x61,       ///< Read cleaning interval
        ReadDataUnit                            =   0x64,       ///< Read PC and PM data unit
        ReadStartStop                           =   0x65,       ///< Read measurement period in ms
        ReadVth                                 =   0x66,       ///< Detection range control voltage reading
        ReadVref                                =   0x69,       ///< Sensitivity control voltage reading
        ReadStatus                              =   0x6A,       ///< Read status for fan, cleaning, PSM and communication mode
        ReadSerialNumber                        =   0x77,       ///< Read Serial Number
        ReadRevisionNumber                      =   0x78,       ///< Read Version number
        ReadNetworkSerialKey                    =   0x79,       ///< Network Serial key 
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
        InvalidDataUnit,
        };

    /// \brief state of the meaurement engine
    enum class State : std::uint8_t
        {
        Uninitialized,      ///< this->begin() has never succeeded.
        End,                ///< this->begin() succeeded, followed by this->end()
        Initial,            ///< initial after begin [indeterminate]
        Idle,               ///< idle (not measuring)
        Triggered,          ///< continuous measurement running, no data available.
        Ready,              ///< continuous measurement running, data availble.
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
        "InvalidDataUnit\0"
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
    /// \returns
    ///     \c true for success, \c false for failure (in which case the the last
    ///     error is set to the error reason).
    ///
    bool begin();

    /// \brief end operation;
    void end();

    /// \brief update fresh PC and PM data
    void updateData();

    /// \brief Return PC value
    unsigned long *getPCData();

    /// \brief Return PC value of range: ≤0.1um 
    unsigned long getPC01Data();

    /// \brief Return PC value of range: 0.1 - 0.3um
    unsigned long getPC03Data();

    /// \brief Return PC value of range: 0.3 - 0.5um
    unsigned long getPC05Data();

    /// \brief Return PC value of range: 0.5 - 1.0um
    unsigned long getPC10Data();

    /// \brief Return PC value of range: 1.0 - 2.5um
    unsigned long getPC25Data();

    /// \brief Return PC value of range: 2.5 - 5.0um
    unsigned long getPC50Data();

    /// \brief Return PC value of range: 5.0 - 10um
    unsigned long getPC100Data();

    /// \brief Return PM value
    float *getPMData();

    /// \brief Return PM value of range: ≤0.1um
    float getPM01Data();

    /// \brief Return PM value of range: ≤0.3um
    float getPM03Data();

    /// \brief Return PM value of range: ≤0.5um
    float getPM05Data();

    /// \brief Return PM value of range: ≤1.0um
    float getPM10Data();

    /// \brief Return PM value of range: ≤2.5um
    float getPM25Data();

    /// \brief Return PM value of range: ≤5.0um
    float getPM50Data();

    /// \brief Return PM value of range: ≤10um
    float getPM100Data();

    /// \brief Return voltage reference
    int getVref();

    /// \brief Return current status of sensor 
    int getStatus();

    ///
    /// \brief Return true if the fan is enabled
    ///
    /// \param [in] status is the new state
    /// status '0' for disable and status '1'for enable
    ///
    bool enableFan(bool status);

    ///
    /// \brief Return true for a successful write
    ///
    /// \param [in] timer is the measurement timer
    /// timer '0' for stop measurement
    /// timer '1' for 200ms
    /// timer '2' for 500ms
    /// timer '3' for 1000ms
    ///
    bool startMeasurement(int timer);

    ///
    /// \brief Return true if the power saving mode is enabled
    ///
    /// \param [in] status is the new state
    /// status '0' for disable and status '1'for enable
    ///
    bool enablePowerSavingMode(bool status);

    ///
    /// \brief Return true if the fan is enabled
    ///
    /// \param [in] status is the new state
    /// status '0' for disable and status '1'for enable
    ///
    bool enableCleaning(bool status);

    ///
    /// \brief Return true if the fan is enabled
    ///
    /// \param [in] unit is measuring unit for PC and PM values
    /// PC units: unit '0' for '#/L' unit '1' for '#/ft3' unit '2' for ' #/m3' unit '3' for '#/L'
    /// PM units: unit '0' for 'ug/m3' unit '1' for 'ug/ft3' unit '2' for 'ug/m3' unit '3' for 'ug/L'
    ///
    bool setDataUnit(int unit);

    ///
    /// \brief Return PC and PM data unit
    ///
    int getDataUnit();

    ///
    /// \brief Return true if the fan is enabled
    ///
    /// \param [in] interval is period of time between cleaning in seconds
    /// Default cleaning interval is 604800 (1 Week)
    ///
    bool setCleaningInterval(unsigned long interval);

    ///
    /// \brief Return cleaning interval
    ///
    unsigned long getCleaningInterval();

    ///
    /// \brief Read serial number
    ///
    /// \param [in] data is a buffer to store serial number
    ///
    void getSerial(uint8_t* data);

    ///
    /// \brief Read version number
    ///
    /// \param [in] data is a buffer to store version number
    ///
    void getVersion(uint8_t* data);

    ///
    /// \brief Read event status
    ///
    uint16_t getEventStatus();

    /// \brief return true if the driver is running.
    bool isRunning() const
        {
        return this->m_state > State::End;
        }

    ///
    /// \brief Change state of driver.
    ///
    /// \param [in] s is the new state
    ///
    /// \details
    ///     This function changes the recorded state of the driver instance.
    ///     When debugging, this might also log state changes; you can do that
    ///     by overriding this method in a derived class.
    ///
    virtual void setState(State s)
        {
        this->m_state = s;
        }

    /// \brief return current state of driver.
    State getState() const { return this->m_state; }

    /// \brief get the last error reported from this instance
    Error getLastError() const
        {
        return this->m_lastError;
        }

    /// \brief set the last error code.
    bool setLastError(Error e)
        {
        this->m_lastError = e;
        return e == Error::Success;
        }

    /// \brief Return true if the debug is enabled
    static constexpr bool isDebug() { return kfDebug; }

protected:
    ///
    /// \brief read a series of bytes starting with a given register.
    ///
    /// \param [in] command indicates the read operation to be performed.
    /// \param [out] pBuffer points to the buffer to receive the data
    /// \param [in] nBuffer is the number of bytes to read.
    ///
    /// \return
    ///     \c true for success, \c false for failure. The
    ///     last error is set in case of error.
    ///
    bool readResponse(cIPS7100::Command command, int nBuffer, std::uint8_t *pBuffer, bool checksum = false);

    ///
    /// \brief Write a byte of command.
    ///
    /// \param [in] command indicates the write operation to be performed
    /// \param [in] value is the value to be written.
    ///
    /// \return
    ///     \c true for success, \c false for failure. The
    ///     last error is set in case of error.
    ///
    bool writeCommand(cIPS7100::Command command, unsigned char value);

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
    unsigned long m_pcValues[7] = {0, 0, 0, 0, 0, 0, 0};    ///< buffer to store PC values
    float m_pmValues[7] = {0, 0, 0, 0, 0, 0, 0};            ///< buffer to store PM values
    uint16_t m_eventStatus = 0;                             ///< event status
    Address m_address;                                      ///< I2C address to be used
    Pin_t m_pinReady;                                       ///< alert pin, or -1 if none.
    Error m_lastError;                                      ///< last error.
    State m_state;                                           ///< current state
        //{ State::Uninitialized };                           ///< initially not yet started.
    };

} // end namespace McciCatenaIps7100

#endif /* _MCCI_CATENA_IPS_7100_H_ */