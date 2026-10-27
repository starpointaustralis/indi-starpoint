/*
    Starpoint Australis INDI Drivers: the devices' text protocol

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

#ifdef _USE_SYSTEM_JSONLIB
#include <nlohmann/json.hpp>
#else
#include <indijson.hpp>
#endif

#include <chrono>
#include <cstdint>
#include <functional>
#include <string>

namespace INDI
{
class DefaultDevice;
}

class StarpointBluetoothLink;

/**
 * The text protocol of Starpoint Australis devices. Commands and replies are ASCII text ending
 * in '#'; commands that change something are answered with a JSON acknowledgement,
 * {"ok":true} or {"ok":false,"error":"..."}. One command is in flight at a time, and each
 * command is sent and its reply read in a single call.
 */
class StarpointProtocol
{
    public:
        // Where commands go. Each is asked again for every command, as the connection and
        // simulation can change while the driver runs.
        struct Route
        {
            std::function<bool()> isSimulation;
            std::function<std::string(const std::string &)> simulate;
            std::function<StarpointBluetoothLink *()> bluetooth;
            std::function<int()> serialPort;
        };

        // `kind` is what to call the device in messages, such as "focuser"
        StarpointProtocol(INDI::DefaultDevice *device, const char *kind);

        void setRoute(Route route);

        // Checks the device on `port` answers the handshake with `expected`. Over USB, lets a
        // device that has just restarted finish its boot output first.
        bool identify(const std::string &port, const std::string &expected);

        // Each sets lastError() if it fails
        bool query(const std::string &command, std::string &reply);
        bool queryInteger(const std::string &command, int32_t &value);
        bool sendCommand(const std::string &command);
        bool sendWithoutReply(const std::string &command);

        const std::string &lastError() const
        {
            return m_LastError;
        }
        void setLastError(const std::string &error)
        {
            m_LastError = error;
        }

        // The last failure was the device refusing the command (a failed acknowledgement)
        bool lastRefused() const
        {
            return m_LastRefused;
        }

        // For INDI's log macros
        const char *getDeviceName() const;

    private:
        enum class Exchange
        {
            OK,
            TIMEOUT,
            FAILED
        };

        Exchange exchange(const std::string &command, std::string &reply);
        void discardInput();
        Exchange readReply(std::string &reply);
        Exchange readByte(char &c, std::chrono::steady_clock::time_point deadline);
        std::string explainRefusal(const std::string &error) const;

        INDI::DefaultDevice *m_Device;
        std::string m_Kind;
        Route m_Route;
        std::string m_LastError;
        bool m_LastRefused {false};
};

// Helpers for the devices' replies
namespace Starpoint
{
// A command without its '#', for logs
std::string withoutTerminator(const std::string &command);

std::string trim(const std::string &text);

// Raw device text for debug logs, with line breaks and other control characters made visible
std::string printable(const std::string &text);

bool parseInteger(const std::string &text, int32_t &value);

// Firmware has been seen to print a missing reading as a bare nan or inf, which is not valid
// JSON. Those are read as null, meaning no value.
std::string withoutNonNumbers(const std::string &text);

// Reply fields are matched ignoring case
const nlohmann::json *findField(const nlohmann::json &object, const char *name);
bool numberField(const nlohmann::json &object, const char *name, double &value);
bool textField(const nlohmann::json &object, const char *name, std::string &value);
}
