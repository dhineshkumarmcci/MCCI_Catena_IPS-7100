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

/**** end of MCCI_Catena_IPS-7100.cpp ****/