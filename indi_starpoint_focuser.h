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

#include "libindi/indifocuser.h"
#include "libindi/connectionplugins/connectionserial.h"
#include "libindi/indicom.h"

#include <mutex>
#include <string>

namespace Connection { class Serial; }

class StarpointFocuser : public INDI::Focuser
{
public: 
    StarpointFocuser();
    virtual ~StarpointFocuser() = default;

    // Identity
    virtual const char *getDefaultName() override;

    // Properties
    virtual bool initProperties() override;
    virtual bool updateProperties() override;

    virtual void TimerHit() override;

protected:
    virtual bool Handshake() override;
    virtual bool SyncFocuser(uint32_t ticks) override;
    virtual IPState MoveAbsFocuser(uint32_t targetTicks) override;
    virtual IPState MoveRelFocuser(FocusDirection dir, uint32_t ticks) override;
    virtual bool AbortFocuser() override;
    virtual bool SetFocuserMaxPosition(uint32_t ticks) override;

private:
    // Write-only
    bool sendCommand(const char *cmd);

    // Write + optional read (until terminator)
    bool sendCommand(const char *cmd,
                     std::string &response,
                     bool expectResponse = true,
                     char terminator = '#',
                     int timeoutSec = 2,
                     size_t maxLen = 512);

    // Serial helpers
    bool writeRaw(const char *buf);
    bool readUntil(std::string &out, char terminator = '#', int timeoutSec = 2, size_t maxLen = 512);
    static void trim(std::string &s);

    // Hardware calls
    bool hwProbe();
    bool hwSerialNumber(std::string& serialOut);
    bool hwReadPosition(uint32_t &pos);
    bool hwMoveAbsolute(uint32_t position);
    bool hwMoveRelative(FocusDirection dir, uint32_t ticks);
    bool hwIsMoving(bool &moving);
    bool hwReadTemperature(double &temp);
    bool hwSetMaxPosition(uint32_t ticks);

    bool m_isMoving{false};
    bool Ack();

    static std::vector<std::string_view> split_sv(std::string_view s, char delim = '%');
    static std::string_view trim_sv(std::string_view sv);
    static bool to_double(std::string_view sv, double& out);
    static bool to_u32(std::string_view sv, uint32_t& out, int base = 10);

    std::mutex ioMutex;

    INDI::PropertyNumber TemperatureNP {1};
    INDI::PropertyText FirmwareTP {2};
    enum
    {
        SERIAL_NUMBER,
        FIRMWARE_VERSION
    };
};