/*
    Starpoint Australis Focuser INDI Driver

    Copyright (C) 2025-2026 Stellar Focus

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

#include "indifocuser.h"
#include "starpoint_bluetooth.h"
#include "starpoint_protocol.h"

#include <chrono>
#include <cstdint>
#include <string>

/**
* Starpoint Australis focusers, over USB serial or Bluetooth LE. StarpointProtocol talks to
* them; this class follows the focuser's state.
*/
class StarpointFocuser : public INDI::Focuser
{
    public:
        StarpointFocuser();
        virtual ~StarpointFocuser() override = default;

        virtual const char *getDefaultName() override;
        virtual bool initProperties() override;
        virtual bool updateProperties() override;

        virtual bool ISNewNumber(const char *dev, const char *name, double values[], char *names[], int n) override;
        virtual bool ISNewSwitch(const char *dev, const char *name, ISState *states, char *names[], int n) override;

    protected:
        virtual bool Handshake() override;
        virtual bool Disconnect() override;
        virtual void TimerHit() override;

        virtual bool saveConfigItems(FILE *fp) override;

        virtual IPState MoveAbsFocuser(uint32_t targetTicks) override;
        virtual IPState MoveRelFocuser(FocusDirection dir, uint32_t ticks) override;
        virtual bool SyncFocuser(uint32_t ticks)override;
        virtual bool AbortFocuser() override;
        virtual bool ReverseFocuser(bool enabled) override;
        virtual bool SetFocuserSpeed(int speed) override;

    private:
        bool usingBluetooth();

        // Focuser state
        bool readConfig();
        bool readTemperature();
        bool poll();
        void pollFailed();
        bool isMoving() const;
        void updateLimitProperties();
        bool setLimits(int32_t minimum, int32_t maximum);
        bool setLED(int index);
        bool calibrateTemperature(double actual);
        bool setTemperatureOffset(double offset);

        StarpointBluetoothConnection *m_BluetoothConnection {nullptr};
        StarpointProtocol m_Protocol;

        int32_t m_Position {0};
        bool m_ReportedMoving {false};
        bool m_Halted {false};
        std::chrono::steady_clock::time_point m_MoveCommandedAt;
        std::chrono::steady_clock::time_point m_LastPollAt;
        std::chrono::steady_clock::time_point m_TemperatureReadAt;
        int m_FailedPolls {0};
        int m_TimerID {-1};
        int32_t m_Target {0};
        int32_t m_MinPosition {0};
        int32_t m_MaxPosition {0};

        bool m_HasTemperature {false};
        double m_Temperature {0};
        double m_TemperatureOffset {0};

        bool m_HasLED {false};
        bool m_HasTemperatureOffset {false};

        INDI::PropertyNumber TemperatureNP {1};
        bool m_TemperatureDefined {0};

        INDI::PropertyNumber FocusMinPosNP {1};

        INDI::PropertyText DeviceInfoTP {3};
        enum
        {
            INFO_MODEL,
            INFO_FIRMWARE,
            INFO_NAME
        };

        INDI::PropertySwitch LEDSP {4};
        enum{
            LED_OFF,
            LED_LOW,
            LED_MEDIUM,
            LED_HIGH
        };

        INDI::PropertyNumber TemperatureOffsetNP {1};
        INDI::PropertyNumber TemperatureCalibrationNP {1};
        INDI::PropertySwitch TemperatureOffsetClearSP {1};

        // A focuser for simulation mode. It answers the same commands as the real focuser.
        class Simulator{
            public:
                std::string reply(const std::string &command);

            private:
                void update();
                std::string ack(bool ok, const char *error = nullptr) const;

                int32_t m_Position {10000};
                int32_t m_Target {10000};
                int32_t m_Min {0};
                int32_t m_Max {25000};
                int m_Speed {180};
                int m_Direction {1};
                int m_LED {10};
                double m_TemperatureOffset {0};
                bool m_Moving {false};
                double m_Travelled {0};
                std::chrono::steady_clock::time_point m_MoveStartedAt;
                std::chrono::steady_clock::time_point m_UpdatedAt;
                std::chrono::steady_clock::time_point m_CreatedAt {std::chrono::steady_clock::now()};
        };

        Simulator m_Simulator;
};