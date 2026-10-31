/*
    Starpoint Australis Filter Wheel INDI Driver

    Copyright (C) 2026 Stellar Focus

    This library is free software; you can redistribute it and/or
    modify it under the terms of the GNU Lesser General Public
    License as published by the Free Software Foundation; either
    version 2.1 of the License, or (at your option) any later version.

    This library is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public
    License along with this library; if not, write to the Free Software
    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA

*/

#pragma once

#include "indifilterwheel.h"
#include "indipowerinterface.h"
#include "starpoint_bluetooth.h"
#include "starpoint_protocol.h"

#include <chrono>
#include <functional>
#include <string>
#include <vector>

/**
 * Starpoint Australis filter wheels, over USB serial or Bluetooth LE. StarpointProtocol talks
 * to them; this class follows the wheel's state. The filter names are stored on the wheel.
 * Wheels with power outputs, a dew heater or a weather sensor say so in their CONFIG? reply.
 */
class StarpointFilterWheel : public INDI::FilterWheel, public INDI::PowerInterface
{
    public:
        StarpointFilterWheel();
        virtual ~StarpointFilterWheel() override = default;

        virtual const char *getDefaultName() override;
        virtual bool initProperties() override;
        virtual bool updateProperties() override;

        virtual bool ISNewNumber(const char *dev, const char *name, double values[], char *names[], int n) override;
        virtual bool ISNewText(const char *dev, const char *name, char *texts[], char *names[], int n) override;
        virtual bool ISNewSwitch(const char *dev, const char *name, ISState *states, char *names[], int n) override;

    protected:
        virtual bool Handshake() override;
        virtual bool Disconnect() override;
        virtual void TimerHit() override;

        virtual bool saveConfigItems(FILE *fp) override;

        virtual int QueryFilter() override;
        virtual bool SelectFilter(int position) override;
        virtual bool SetFilterNames() override;
        virtual bool GetFilterNames() override;

        virtual bool SetPowerPort(size_t port, bool enabled) override;
        virtual bool SetDewPort(size_t port, bool enabled, double dutyCycle) override;
        virtual bool SetAutoDewEnabled(size_t port, bool enabled) override;

    private:
        bool usingBluetooth();

        bool readConfig();
        bool poll();
        void pollFailed();
        bool isMoving() const;
        bool setLED(int index);

        // Power outputs, dew heater and weather sensor
        void switchEach(INDI::PropertySwitch &property, ISState *states, char *names[], int n,
                        const std::function<bool(size_t, bool)> &set);
        bool pollAuxiliary();
        bool readPower();
        bool readDewHeater();
        bool readConditions();
        bool setDewHeater(int mode, int duty);
        void showDewHeater();

        StarpointBluetoothConnection *m_BluetoothConnection {nullptr};
        StarpointProtocol m_Protocol;

        // Slots are counted from 0, INDI counts them from 1.
        int m_Slots {0};
        int m_Position {0};
        int m_Target {0};
        std::vector<std::string> m_Names;
        std::chrono::steady_clock::time_point m_MoveCommandedAt;
        std::chrono::steady_clock::time_point m_LastPollAt;
        int m_FailedPolls {0};
        int m_TimerID {-1};

        bool m_HasLED {false};

        bool m_HasPower {false};
        bool m_HasDewHeater {false};
        bool m_HasConditions {false};
        bool m_ConditionsAvailable {false};
        int m_DewMode {0};
        int m_DewDuty {0};
        std::chrono::steady_clock::time_point m_AuxiliaryPolledAt;

        INDI::PropertyText DeviceInfoTP {3};
        enum
        {
            INFO_MODEL,
            INFO_FIRMWARE,
            INFO_NAME
        };

        INDI::PropertySwitch LEDSP {4};
        enum
        {
            LED_OFF,
            LED_LOW,
            LED_MEDIUM,
            LED_HIGH
        };

        INDI::PropertyNumber ConditionsNP {4};
        enum
        {
            CONDITIONS_TEMPERATURE,
            CONDITIONS_HUMIDITY,
            CONDITIONS_PRESSURE,
            CONDITIONS_DEW_POINT
        };

        class Simulator
        {
            public:
                std::string reply(const std::string &command);

            private:
                void update();
                std::string ack(bool ok, const char *error = nullptr) const;

                std::vector<std::string> m_Names {"Luminance", "Red", "Green", "Blue", "H-Alpha", "OIII", "SII"};
                int m_Position {0};
                int m_Target {0};
                int m_LED {10};
                std::chrono::steady_clock::time_point m_MoveEndsAt;

                // A wheel with power outputs, a dew heater and a weather sensor
                bool m_Outputs[2] {true, false};
                int m_DewMode {2};
                int m_DewDuty {30};
        };

        Simulator m_Simulator;
};