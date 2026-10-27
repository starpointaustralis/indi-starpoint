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

#include "starpoint_protocol.h"

#include "starpoint_bluetooth.h"

#include "defaultdevice.h"
#include "indicom.h"
#include "indilogger.h"

#include <cerrno>
#include <climits>
#include <cstdlib>
#include <regex>
#include <thread>

#include <strings.h>

using json = nlohmann::json;

namespace
{
    // Serial protocol timing
    constexpr auto BOOT_DRAIN = std::chrono::milliseconds(200);
    constexpr auto REPLY_TIMEOUT = std::chrono::seconds(2);
    constexpr int QUERY_ATTEMPTS = 3;
    constexpr auto RETRY_DELAY = std::chrono::milliseconds(200);
    constexpr int HANDSHAKE_ATTEMPTS = 5;
    constexpr auto HANDSHAKE_RETRY_DELAY = std::chrono::seconds(1);
    constexpr size_t MAX_REPLY_LENGTH = 4096;

    // Reads an acknowledgement. Returns false if the reply is not one.
    bool parseAck(const std::string &reply, bool &ok, std::string &error)
    {
        if (reply.empty() || reply.front() != '{')
            return false;

        try
        {
            const json doc = json::parse(reply);
            if (!doc.is_object())
                return false;

            const auto okField = doc.find("ok");
            if (okField == doc.end() || !okField->is_boolean())
                return false;

            ok = okField->get<bool>();
            const auto errorField = doc.find("error");
            error = (errorField != doc.end() && errorField->is_string()) ? errorField->get<std::string>() : std::string();
            return true;
        }
        catch (const json::exception &)
        {
            return false;
        }
    }
}

namespace Starpoint
{
std::string withoutTerminator(const std::string &command)
{
    return (!command.empty() && command.back() == '#') ? command.substr(0, command.size() - 1) : command;
}

std::string trim(const std::string &text)
{
    const auto first = text.find_first_not_of(" \t");
    if (first == std::string::npos)
        return std::string();
    return text.substr(first, text.find_last_not_of(" \t") - first + 1);
}

std::string printable(const std::string &text)
{
    std::string result;
    for (unsigned char c : text)
    {
        if (c == '\r')
            result += "\\r";
        else if (c == '\n')
            result += "\\n";
        else if (c < 0x20 || c == 0x7F)
            result += '?';
        else
            result += static_cast<char>(c);
    }
    return result;
}

bool parseInteger(const std::string &text, int32_t &value)
{
    if (text.empty())
        return false;

    char *end = nullptr;
    errno = 0;
    const long result = std::strtol(text.c_str(), &end, 10);
    if (errno != 0 || *end != '\0' || result < INT32_MIN || result > INT32_MAX)
        return false;

    value = static_cast<int32_t>(result);
    return true;
}

std::string withoutNonNumbers(const std::string &text)
{
    static const std::regex nonNumber(R"(([:\[,]\s*)[-+]?(nan|inf|infinity)(?=\s*[,}\]]))", std::regex::icase);
    return std::regex_replace(text, nonNumber, "$1null");
}

const json *findField(const json &object, const char *name)
{
    if (!object.is_object())
        return nullptr;

    for (auto it = object.begin(); it != object.end(); ++it)
    {
        if (strcasecmp(it.key().c_str(), name) == 0)
            return &it.value();
    }
    return nullptr;
}

bool numberField(const json &object, const char *name, double &value)
{
    const json *field = findField(object, name);
    if (field == nullptr || !field->is_number())
        return false;

    value = field->get<double>();
    return true;
}

bool textField(const json &object, const char *name, std::string &value)
{
    const json *field = findField(object, name);
    if (field == nullptr || !field->is_string())
        return false;

    value = field->get<std::string>();
    return true;
}
}

using Starpoint::printable;
using Starpoint::withoutTerminator;

StarpointProtocol::StarpointProtocol(INDI::DefaultDevice *device, const char *kind) : m_Device(device), m_Kind(kind)
{
}

void StarpointProtocol::setRoute(Route route)
{
    m_Route = std::move(route);
}

const char *StarpointProtocol::getDeviceName() const
{
    return m_Device->getDeviceName();
}

bool StarpointProtocol::identify(const std::string &port, const std::string &expected)
{
    // Over USB, the device may have just restarted
    if (!m_Route.isSimulation() && m_Route.bluetooth() == nullptr)
    {
        std::this_thread::sleep_for(BOOT_DRAIN);
        discardInput();
    }

    for (int attempt = 1; ; attempt++)
    {
        std::string reply;
        const Exchange result = exchange("PING?#", reply);
        bool ok = false;
        std::string error;
        const bool refused = result == Exchange::OK && parseAck(reply, ok, error) && !ok;

        if (result == Exchange::OK && !refused)
        {
            if (reply == expected)
                return true;

            LOGF_ERROR("The device on %s is not a Starpoint Australis %s.", port.c_str(), m_Kind.c_str());
            return false;
        }

        if (result == Exchange::FAILED)
        {
            LOGF_ERROR("Could not talk to the device on %s. %s", port.c_str(), m_LastError.c_str());
            return false;
        }

        if (attempt == HANDSHAKE_ATTEMPTS)
        {
            LOGF_ERROR("No response from %s.", port.c_str());
            return false;
        }

        LOGF_DEBUG("No handshake from %s (attempt %d) - retrying", port.c_str(), attempt);
        std::this_thread::sleep_for(HANDSHAKE_RETRY_DELAY);
    }
}

bool StarpointProtocol::query(const std::string &command, std::string &reply)
{
    m_LastRefused = false;

    // Read only, so a timeout is retried
    auto delay = RETRY_DELAY;
    for (int attempt = 1; ; attempt++)
    {
        const Exchange result = exchange(command, reply);
        if (result == Exchange::OK)
            break;
        if (result == Exchange::FAILED || attempt == QUERY_ATTEMPTS)
            return false;

        LOGF_DEBUG("Retrying [%s] (attempt %d)", command.c_str(), attempt + 1);
        std::this_thread::sleep_for(delay);
        delay *= 2;
    }

    // A query can also be answered with a failed acknowledgement
    bool ok = false;
    std::string error;
    if (parseAck(reply, ok, error) && !ok)
    {
        LOGF_DEBUG("%s -> %s", withoutTerminator(command).c_str(), error.c_str());
        m_LastError = explainRefusal(error);
        m_LastRefused = true;
        return false;
    }

    return true;
}

bool StarpointProtocol::queryInteger(const std::string &command, int32_t &value)
{
    std::string reply;
    if (!query(command, reply))
        return false;

    if (!Starpoint::parseInteger(reply, value))
    {
        LOGF_DEBUG("%s -> %s", withoutTerminator(command).c_str(), printable(reply).c_str());
        m_LastError = "The " + m_Kind + " gave an unexpected reply.";
        return false;
    }

    return true;
}

bool StarpointProtocol::sendCommand(const std::string &command)
{
    m_LastRefused = false;

    // Every change is acknowledged, and the acknowledgement must be read.
    std::string reply;
    if (exchange(command, reply) != Exchange::OK)
        return false;

    bool ok = false;
    std::string error;
    if (!parseAck(reply, ok, error))
    {
        LOGF_DEBUG("%s -> %s", withoutTerminator(command).c_str(), printable(reply).c_str());
        m_LastError = "The " + m_Kind + " gave an unexpected reply.";
        return false;
    }

    if (!ok)
    {
        LOGF_DEBUG("%s -> %s", withoutTerminator(command).c_str(), error.c_str());
        m_LastError = explainRefusal(error);
        m_LastRefused = true;
        return false;
    }

    return true;
}

bool StarpointProtocol::sendWithoutReply(const std::string &command)
{
    LOGF_DEBUG("Sent [%s]", command.c_str());
    if (m_Route.isSimulation())
        return true;

    if (StarpointBluetoothLink *link = m_Route.bluetooth())
    {
        if (link->write(command))
            return true;
        m_LastError = link->lastError();
        return false;
    }

    int written = 0;
    const int rc = tty_write_string(m_Route.serialPort(), command.c_str(), &written);
    if (rc != TTY_OK)
    {
        char message[MAXRBUF];
        tty_error_msg(rc, message, MAXRBUF);
        LOGF_DEBUG("Writing [%s] failed: %s", command.c_str(), message);
        m_LastError = "Could not send a command to the " + m_Kind + ".";
        return false;
    }

    return true;
}

StarpointProtocol::Exchange StarpointProtocol::exchange(const std::string &command, std::string &reply)
{
    reply.clear();
    if (m_Route.isSimulation())
    {
        LOGF_DEBUG("Sent [%s]", command.c_str());
        reply = m_Route.simulate(command);
        LOGF_DEBUG("received [%s]", reply.c_str());
        return Exchange::OK;
    }
    discardInput();

    if (!sendWithoutReply(command))
        return Exchange::FAILED;

    const Exchange result = readReply(reply);
    if (result == Exchange::OK)
        LOGF_DEBUG("Received [%s]", printable(reply).c_str());
    else if (result == Exchange::TIMEOUT)
    {
        LOGF_DEBUG("No reply to [%s]", command.c_str());
        m_LastError = "The " + m_Kind + " did not reply.";
    }
    return result;
}

void StarpointProtocol::discardInput()
{
    std::string discarded;
    char buffer[256];
    if (StarpointBluetoothLink *link = m_Route.bluetooth())
        discarded = link->takeReceived();
    else for (;;)
    {
        int nread = 0;
        const int rc = tty_read_expanded(m_Route.serialPort(), buffer, sizeof(buffer), 0, 0, &nread);
        if (nread > 0)
            discarded.append(buffer, nread);
        if (rc != TTY_OK)
            break;
    }

    if (!discarded.empty())
        LOGF_DEBUG("Discarded [%s]", printable(discarded).c_str());
}

StarpointProtocol::Exchange StarpointProtocol::readReply(std::string &reply)
{
    // Replies end in '#' and have no line breaks. Boot and crash logs have line breaks and no
    // '#', so only the text after the last line break is kept.
    const auto deadline = std::chrono::steady_clock::now() + REPLY_TIMEOUT;
    std::string pending;
    for (;;)
    {
        char c = 0;
        const Exchange result = readByte(c, deadline);
        if (result != Exchange::OK)
            return result;

        if (c == '#')
        {
            reply = Starpoint::trim(pending);
            pending.clear();
            if (!reply.empty())
                return Exchange::OK;
        }
        else if (c == '\r' || c == '\n')
        {
            if (!pending.empty())
                LOGF_DEBUG("Discarded [%s]", printable(pending).c_str());
            pending.clear();
        }
        else if (pending.size() < MAX_REPLY_LENGTH)
            pending += c;
    }
}

StarpointProtocol::Exchange StarpointProtocol::readByte(char &c, std::chrono::steady_clock::time_point deadline)
{
    if (StarpointBluetoothLink *link = m_Route.bluetooth())
    {
        switch (link->read(c, deadline))
        {
            case StarpointBluetoothLink::Read::OK:
                return Exchange::OK;
            case StarpointBluetoothLink::Read::TIMEOUT:
                return Exchange::TIMEOUT;
            case StarpointBluetoothLink::Read::LOST:
                break;
        }
        m_LastError = "Lost contact with the " + m_Kind + " over Bluetooth.";
        return Exchange::FAILED;
    }

    const auto remaining = std::chrono::duration_cast<std::chrono::microseconds>(deadline - std::chrono::steady_clock::now()).count();
    if (remaining <= 0)
        return Exchange::TIMEOUT;

    int nread = 0;
    const int rc = tty_read_expanded(m_Route.serialPort(), &c, 1, remaining / 1000000, remaining % 1000000, &nread);
    if (rc == TTY_TIME_OUT)
        return Exchange::TIMEOUT;
    if (rc != TTY_OK)
    {
        char message[MAXRBUF];
        tty_error_msg(rc, message, MAXRBUF);
        LOGF_DEBUG("Reading failed: %s", message);
        m_LastError = "Could not read from the " + m_Kind + ".";
        return Exchange::FAILED;
    }
    return Exchange::OK;
}

// What a refused command means in plain English
std::string StarpointProtocol::explainRefusal(const std::string &error) const
{
    if (error == "range")
        return "The " + m_Kind + " rejected the value as out of range.";
    if (error == "unsupported")
        return "The " + m_Kind + "'s firmware does not support this command.";
    if (error == "security_required")
        return "The " + m_Kind + " needs a paired Bluetooth connection for this. Disconnect, then connect again to pair.";
    if (error == "busy")
        return "The " + m_Kind + " is busy and refused the command. Try again in a moment.";
    return "The " + m_Kind + " refused the command.";
}
