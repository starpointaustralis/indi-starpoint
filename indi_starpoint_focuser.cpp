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

#include "config.h"
#include "indi_starpoint_focuser.h"

#include <cstring>
#include <vector>
#include <cctype>
#include <algorithm>
#include <memory>
#include <charconv>
#include <string_view>
#include <thread>
#include <chrono>

static std::unique_ptr<StarpointFocuser> starpointFocuser(new StarpointFocuser());

StarpointFocuser::StarpointFocuser()
{
    setSupportedConnections(CONNECTION_SERIAL);
    SetCapability(FOCUSER_CAN_ABS_MOVE | FOCUSER_CAN_REL_MOVE | FOCUSER_CAN_SYNC | FOCUSER_CAN_ABORT);
    setVersion(CDRIVER_VERSION_MAJOR, CDRIVER_VERSION_MINOR);
    setDefaultPollingPeriod(1000);
}

const char *StarpointFocuser::getDefaultName()
{
    return "Starpoint Focuser";
}

bool StarpointFocuser::initProperties()
{
    INDI::Focuser::initProperties();
    serialConnection->setDefaultBaudRate(Connection::Serial::B_115200);

    FirmwareTP[SERIAL_NUMBER].fill("SERIAL_NUMBER", "Serial Number", "");
    FirmwareTP[FIRMWARE_VERSION].fill("FIRMWARE_VERSION", "Firmware Version", "");
    FirmwareTP.fill(getDeviceName(), "FOCUSER_DETAILS", "Focuser Details", CONNECTION_TAB, IP_RO, 0, IPS_IDLE);

    TemperatureNP[0].fill("TEMPERATURE", "Temperature (°C)", "%.2f", -100, 100, 0, 0);
    TemperatureNP.fill(getDeviceName(), "FOCUSER_TEMPERATURE", "Temperature", MAIN_CONTROL_TAB, IP_RO, 0, IPS_IDLE); 

    return true;
}

bool StarpointFocuser::updateProperties()
{
    INDI::Focuser::updateProperties();

    if (isConnected())
    {
        defineProperty(FirmwareTP);
        defineProperty(TemperatureNP);
    }
    else
    {
        deleteProperty(FirmwareTP);
        deleteProperty(TemperatureNP);
    }

    return true;
}

bool StarpointFocuser::Handshake()
{
    if (Ack())
    {
        LOG_INFO("Connected to Starpoint Focuser");
        return true;
    }       
    
    return false;
}

void StarpointFocuser::TimerHit()
{
    if (!isConnected())
        return;

    // Update position
    uint32_t pos;
    hwReadPosition(pos);
    FocusAbsPosNP[0].setValue(pos);
    
    // Check if the focuser is moving
    bool isMoving;
    hwIsMoving(isMoving);

    FocusAbsPosNP.setState(isMoving ? IPS_BUSY : IPS_OK);
    FocusAbsPosNP.apply();
    FocusRelPosNP.setState(isMoving ? IPS_BUSY : IPS_OK);
    FocusRelPosNP.apply();

    // Update temperature
    double temp;
    hwReadTemperature(temp);
    TemperatureNP[0].setValue(temp);
    TemperatureNP.apply();

    SetTimer(getPollingPeriod());
}

bool StarpointFocuser::Ack()
{
    bool probe = hwProbe();
    if (!probe)
    {
        LOG_INFO("No response, or device is not a Starpoint Focuser");
        return false;
    }

    std::string resp;
    if (!sendCommand("STARTSETUP#", resp))
    {
        LOG_ERROR("Unable to obtain focuser properties.");
        return false;
    }

    trim(resp);
    auto parts = split_sv(resp);
    
    if (parts.size() < 16)
    {
        LOG_ERROR("Focuser properties returned an incorrect amount of data.");
        return false;
    }

    std::string_view firmware = parts[2];
    std::string_view temperature = parts[3];
    std::string_view position = parts[5];
    std::string_view maxPos = parts[6];
    std::string_view minPos = parts[7];

    // Parse doubles
    double tempVal = 0.0, posVal = 0.0, maxPosVal = 0.0, minPosVal = 0.0;
    if (!to_double(temperature, tempVal)) { LOG_ERROR("Unable to parse temperature value."); return false; }
    if (!to_double(position, posVal)) { LOG_ERROR("Unable to parse position value."); return false; }
    if (!to_double(maxPos, maxPosVal)) { LOG_ERROR("Unable to parse maximum position value."); return false; }
    if (!to_double(minPos, minPosVal)) { LOG_ERROR("Unable to parse minimum position value."); return false; }

    // Push to properties
    FirmwareTP[FIRMWARE_VERSION].setText(std::string(firmware).c_str());
    TemperatureNP[0].setValue(tempVal);
    FocusAbsPosNP[0].setValue(posVal);
    FocusAbsPosNP[0].setMax(maxPosVal);
    FocusAbsPosNP[0].setMin(minPosVal);

    // Get device serial number
    std::string serialNumber;
    if (hwSerialNumber(serialNumber))
    {
        //FirmwareTP[SERIAL_NUMBER].setText(serialNumber);
    } 
    
    return true;
}

IPState StarpointFocuser::MoveAbsFocuser(uint32_t targetTicks)
{
    bool moveA = hwMoveAbsolute(targetTicks);
    if (!moveA)
        return IPS_ALERT;

    FocusAbsPosNP.setState(IPS_BUSY);
    return IPS_BUSY;
}

IPState StarpointFocuser::MoveRelFocuser(FocusDirection dir, uint32_t ticks)
{
    bool moveR = hwMoveRelative(dir, ticks);
    if (!moveR)
        return IPS_ALERT;

    FocusRelPosNP.setState(IPS_BUSY);
    return IPS_BUSY;
}

bool StarpointFocuser::SyncFocuser(uint32_t ticks)
{
    FocusAbsPosNP.setState(IPS_BUSY);

    std::string cmd = "SYNC" + std::to_string(ticks) + "#";
    if (!sendCommand(cmd.c_str()))
    {
        LOG_ERROR("Unable to sync focuser position");
        return false;
    }

    FocusAbsPosNP[0].setValue(ticks);
    return true;
}

bool StarpointFocuser::AbortFocuser()
{
    if (!sendCommand("HALT#"))
    {
        LOG_ERROR("Unable to halt focuser movement,");
        return false;
    }

    return true;
}

bool StarpointFocuser::SetFocuserMaxPosition(uint32_t ticks)
{
    FocusAbsPosNP.setState(IPS_BUSY);
    bool succeeded = hwSetMaxPosition(ticks);
    FocusAbsPosNP[0].setMax(ticks);
    FocusAbsPosNP.setState(IPS_OK);
    FocusAbsPosNP.apply();

    return succeeded;
}

bool StarpointFocuser::hwProbe()
{
    LOG_DEBUG("Checking if device is a starpoint focuser");

    std::string resp;
    if (!sendCommand("PING#", resp))
    {
        LOGF_ERROR("No response from  port %s", serialConnection->getPortFD());
        return false;
    }

    trim(resp);
    if (resp == "FOUND1982")
        return true;

    return false;  
}

bool StarpointFocuser::hwSerialNumber(std::string& serialOut)
{
    serialOut.clear();

    std::string resp;
    if (!sendCommand("GSNUM#", resp))
    {
        LOG_ERROR("Unable to read device serial number");
        return false;
    }

    trim(resp);
    serialOut = resp;

    LOGF_DEBUG("Reading device serial number: %s", serialOut);
    return true;
}

bool StarpointFocuser::hwReadPosition(uint32_t &pos)
{
    std::string resp;
    if (!sendCommand("GET#", resp))
    {
        LOG_ERROR("Unable to get read position");
        return false;
    }

    trim(resp);
    if (!to_u32(resp, pos))
    {
        LOG_ERROR("Read position returned an invalid value.");
        return false;
    }

    LOGF_DEBUG("Reading position: %d", pos);
    return true;
}

bool StarpointFocuser::hwIsMoving(bool &moving)
{
    std::string resp;
    if (!sendCommand("MOVING#", resp))
    {
        LOG_ERROR("Unable to check if the Focuser is moving.");
        return false;
    }

    trim(resp);
    moving = resp == "0";

    LOGF_DEBUG("Checking if focuser is moving: %s", moving ? "true" : "false");
    return true;
}

bool StarpointFocuser::hwReadTemperature(double &temp)
{
    std::string resp;
    if (!sendCommand("TEMP#", resp))
    {
        LOG_ERROR("Unable to get temperature.");
        return false;
    }

    trim(resp);
    to_double(resp.c_str(), temp);

    LOGF_DEBUG("Reading temperature: %d", temp);
    return true;
}

bool StarpointFocuser::hwMoveAbsolute(uint32_t targetTicks)
{
    // Ensure we are within range
    targetTicks = std::max(static_cast<uint32_t>(FocusAbsPosNP[0].getMin()), 
                        std::min(static_cast<uint32_t>(FocusAbsPosNP[0].getMax()), targetTicks));

    std::string cmd = "MOVEA" + std::to_string(targetTicks) + "#";
    if (!sendCommand(cmd.c_str()))
    {
        LOG_ERROR("Unable to move focuser.");
        return false;
    }

    LOGF_DEBUG("Moving focuser to position: %d", targetTicks);
    return true;
}

bool StarpointFocuser::hwMoveRelative(FocusDirection dir, uint32_t ticks)
{
    if (dir == FOCUS_INWARD)
        ticks = ticks * -1;
    
    std::string cmd = std::string("MOVER") + (dir == FOCUS_INWARD ? "1" : "0") + std::to_string(ticks) + "#";
    if (!sendCommand(cmd.c_str()))
    {
        LOG_ERROR("Unable to move focuser.");
        return false;
    }

    LOGF_DEBUG("Moving focuser relatively by: %d %s", ticks, dir == FOCUS_INWARD ? "Inwards" : "Outwards");
    return true;
}

bool StarpointFocuser::hwSetMaxPosition(uint32_t ticks)
{
    std::string cmd = "SMAX" + std::to_string(ticks) + "#";
    if (!sendCommand(cmd.c_str()))
    {
        LOG_ERROR("Unable to set Max position.");
        return false;
    }

    LOGF_DEBUG("Max position set to %d", ticks);
    return true;
}

bool StarpointFocuser::sendCommand(const char *cmd)
{
    return writeRaw(cmd);
}

bool StarpointFocuser::sendCommand(const char *cmd, std::string &response, bool expectResponse, char terminator, int timeoutSec, size_t maxLen)
{
    if (!writeRaw(cmd))
        return false;

    if (!expectResponse)
        return true;

    std::this_thread::sleep_for(std::chrono::milliseconds(200)); // Short delay before reading
    return readUntil(response, terminator, timeoutSec, maxLen);
}

bool StarpointFocuser::writeRaw(const char *buf)
{
    std::lock_guard<std::mutex> lk(ioMutex);
    const int fd = serialConnection->getPortFD();
    if (fd < 0)
        return false;

    int nWrite = 0;
    const int rc = tty_write_string(fd, buf, &nWrite);
    if (rc != TTY_OK)
    {
        char errtxt[128] = {};
        tty_error_msg(rc, errtxt, sizeof(errtxt));
        LOGF_ERROR("Failed to send command: %s", errtxt);
        return false;
    }

    return true;
}

bool StarpointFocuser::readUntil(std::string &out, char terminator, int timeoutSec, size_t maxLen)
{
    std::lock_guard<std::mutex> lk(ioMutex);
    const int fd = serialConnection->getPortFD();
    if (fd < 0)
        return false;

    std::vector<char> buf(maxLen + 1, 0);
    int nread = 0;

    const int rc = tty_nread_section(fd, buf.data(), static_cast<int>(maxLen), terminator, timeoutSec, &nread);
    if (rc != TTY_OK)
    {
        char errtxt[128] = {};
        tty_error_msg(rc, errtxt, sizeof(errtxt));
        LOGF_ERROR("Failed to send command: %s", errtxt);
        return false;
    }

    buf[std::min<int>(nread, static_cast<int>(maxLen))] = '\0';
    out.assign(buf.data(), nread);
    return true;
}

void StarpointFocuser::trim(std::string &s)
{
    auto issp = [](unsigned char c){ return std::isspace(c) != 0; };

    s.erase(s.begin(), std::find_if(s.begin(), s.end(),
        [&](unsigned char c){ return !issp(c); }));
    while (!s.empty() && issp(static_cast<unsigned char>(s.back())))
        s.pop_back();
    if (!s.empty() && s.back() == '#')
        s.pop_back();
}

/**
* Zero-alloc splitter for splitting the STARTSETUP response values
*/
std::vector<std::string_view>
StarpointFocuser::split_sv(std::string_view s, char delim)
{
    std::vector<std::string_view> out;
    size_t start = 0;
    for (size_t i = 0; i < s.size(); ++i)
    {
        if (s[i] == delim) 
        {
            out.emplace_back(s.data() + start, i - start);
            start = i + 1;
        }
    }

    out.emplace_back(s.data() + start, s.size() - start);
    return out;
}

/**
 * Trim a view without copying
 */
std::string_view StarpointFocuser::trim_sv(std::string_view sv)
{
    auto isws = [](unsigned char c){ return std::isspace(c) != 0; };
    while (!sv.empty() && isws((unsigned char)sv.front())) sv.remove_prefix(1);
    while (!sv.empty() && isws((unsigned char)sv.back()))  sv.remove_suffix(1);
    return sv;
}

bool StarpointFocuser::to_double(std::string_view sv, double& out)
{
    sv = StarpointFocuser::trim_sv(sv);
    const char* b = sv.data();
    const char* e = sv.data() + sv.size();
    auto res = std::from_chars(b, e, out);
    return res.ec == std::errc{} && res.ptr == e;
}

bool StarpointFocuser::to_u32(std::string_view sv, uint32_t& out, int base)
{
    auto res = std::from_chars(sv.data(), sv.data() + sv.size(), out, base);
    return res.ec == std::errc{} && res.ptr == sv.data() + sv.size();
}