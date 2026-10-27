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

#include "starpoint_focuser.h"

#include "config.h"
#include "indicom.h"
#include "indicontroller.h"
#include "connectionplugins/connectionserial.h"

#include <algorithm>
#include <climits>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <iterator>
#include <memory>

using json = nlohmann::json;

namespace
{
    // Polling
    constexpr uint32_t POLL_TICK_MS = 250;
    constexpr uint32_t IDLE_POLL_MS = 2000;
    constexpr auto MOVE_START_GRACE = std::chrono::seconds(1);
    constexpr auto TEMPERATURE_INTERVAL = std::chrono::seconds(5);
    constexpr int FAILED_POLLS_BEFORE_LOST = 3;

    const std::string FOCUSER_REPLY = "FOUND1982";

    // The focuser's Bluetooth LE services: commands go to the transmit characteristic and replies
    // come back as notifications on the receive characteristic
    const StarpointBluetoothLink::Profile BLUETOOTH_PROFILE = 
    {
        "focuser",
        "019163f5-e364-728c-b7ab-68ad229cc2a2", "019163ea-728c-717d-beaf-5f7189ba85e2",
        "019163e4-e656-7e07-af4f-b645cc34d96e", "019163e5-db4b-777b-b4a6-7fcd6c1341ba"
    };

    // LED brightness as the firmware's raw values: Off, Low, Medium, High
    constexpr int LED_LEVELS[] = {0, 10, 80, 150};

    constexpr int MIN_SPEED = 1;
    constexpr int MAX_SPEED = 255;
    constexpr double MAX_TEMPERATURE_OFFSET = 20;

    // A value to two decimal places without trailing zeros
    std::string formatOffset(double value)
    {
        char text[32];
        snprintf(text, sizeof(text), "%.2f", value);
        std::string result(text);
        result.erase(result.find_last_not_of('0') + 1);
        if (result.back() == '.')
            result.pop_back();
        return result == "-0" ? "0" : result;
    }

    // The nearest LED level at or below a raw value the focuser reported
    int ledIndexFromRaw(int raw)
    {
        for (int i = static_cast<int>(std::size(LED_LEVELS)) - 1; i > 0; i--)
        {
            if (raw >= LED_LEVELS[i])
                return i;
        }
        return 0;
    }

    int32_t toSteps(double value)
    {
        return static_cast<int32_t>(std::lround(std::clamp(value, static_cast<double>(INT32_MIN), static_cast<double>(INT32_MAX))));
    }
}

using Starpoint::findField;
using Starpoint::numberField;
using Starpoint::printable;
using Starpoint::textField;
using Starpoint::withoutNonNumbers;
using Starpoint::withoutTerminator;

static std::unique_ptr<StarpointFocuser> starpointFocuser(new StarpointFocuser());

StarpointFocuser::StarpointFocuser() : m_Protocol(this, "focuser")
{
    // Commands go to the simulator, the Bluetooth link or the serial port, whichever is in use
    m_Protocol.setRoute(
    {
        [this]()
        {
            return isSimulation();
        },
        [this](const std::string & command)
        {
            return m_Simulator.reply(command);
        },
        [this]() -> StarpointBluetoothLink *
        {
            return usingBluetooth() ? &m_BluetoothConnection->link() : nullptr;
        },
        [this]()
        {
            return PortFD;
        }
    });

    setVersion(CDRIVER_VERSION_MAJOR, CDRIVER_VERSION_MINOR);
    setSupportedConnections(CONNECTION_SERIAL);
    SetCapability(FOCUSER_CAN_ABS_MOVE | FOCUSER_CAN_REL_MOVE | FOCUSER_CAN_ABORT | FOCUSER_CAN_SYNC);
    setDefaultPollingPeriod(IDLE_POLL_MS);
}

const char *StarpointFocuser::getDefaultName()
{
    return "Starpoint Australis Focuser";
}

bool StarpointFocuser::initProperties()
{
    INDI::Focuser::initProperties();

    serialConnection->setDefaultBaudRate(Connection::Serial::B_115200);

    m_BluetoothConnection = new StarpointBluetoothConnection(this, BLUETOOTH_PROFILE);
    m_BluetoothConnection->registerHandshake([&]()
    {
        return Handshake();
    });
    m_BluetoothConnection->registerSimulation([&]()
    {
        return isSimulation();
    });
    registerConnection(m_BluetoothConnection);

    FocusMaxPosNP[0].setMinMax(1, INT32_MAX);
    FocusMinPosNP[0].fill("FOCUS_MIN_VALUE", "Steps", "%.f", 0, INT32_MAX, 1000, 0);
    FocusMinPosNP.fill(getDeviceName(), "FOCUS_MIN", "Min. Position", MAIN_CONTROL_TAB, IP_RW, 60, IPS_OK);

    FocusSpeedNP[0].setMinMax(MIN_SPEED, MAX_SPEED);

    TemperatureNP[0].fill("TEMPERATURE", "Celsius", "%.2f", -50, 70, 0, 0);
    TemperatureNP.fill(getDeviceName(), "FOCUS_TEMPERATURE", "Temperature", MAIN_CONTROL_TAB, IP_RO, 0, IPS_IDLE);

    DeviceInfoTP[INFO_MODEL].fill("MODEL", "Model", "");
    DeviceInfoTP[INFO_FIRMWARE].fill("FIRMWARE", "Firmware", "");
    DeviceInfoTP[INFO_NAME].fill("NAME", "Name", "");
    DeviceInfoTP.fill(getDeviceName(), "DEVICE_INFO", "Focuser", INFO_TAB, IP_RO, 0, IPS_IDLE);

    LEDSP[LED_OFF].fill("LED_OFF", "Off", ISS_OFF);
    LEDSP[LED_LOW].fill("LED_LOW", "Low", ISS_OFF);
    LEDSP[LED_MEDIUM].fill("LED_MEDIUM", "Medium", ISS_OFF);
    LEDSP[LED_HIGH].fill("LED_HIGH", "High", ISS_OFF);
    LEDSP.fill(getDeviceName(), "LED_BRIGHTNESS", "Status Light", OPTIONS_TAB, IP_RW, ISR_1OFMANY, 60, IPS_IDLE);

    TemperatureOffsetNP[0].fill("OFFSET", "Offset (°C)", "%.2f", -MAX_TEMPERATURE_OFFSET, MAX_TEMPERATURE_OFFSET, 0, 0);
    TemperatureOffsetNP.fill(getDeviceName(), "TEMPERATURE_OFFSET", "Temperature Offset", OPTIONS_TAB, IP_RO, 0, IPS_IDLE);

    TemperatureCalibrationNP[0].fill("ACTUAL", "Actual temperature (°C)", "%.2f", -50, 70, 0.1, 0);
    TemperatureCalibrationNP.fill(getDeviceName(), "TEMPERATURE_CALIBRATION", "Calibrate Temperature", OPTIONS_TAB, IP_RW, 60,IPS_IDLE);

    TemperatureOffsetClearSP[0].fill("CLEAR", "Clear offset", ISS_OFF);
    TemperatureOffsetClearSP.fill(getDeviceName(), "TEMPERATURE_OFFSET_CLEAR", "Temperature Offset", OPTIONS_TAB, IP_RW,ISR_ATMOST1, 60, IPS_IDLE);

    // Debug, simulation and configuration controls
    addAuxControls();

    return true;
}

bool StarpointFocuser::updateProperties()
{
    INDI::Focuser::updateProperties();

    if (isConnected())
    {
        defineProperty(FocusMinPosNP);
        if (m_HasTemperature)
        {
            defineProperty(TemperatureNP);
            m_TemperatureDefined = true;
        }
        defineProperty(DeviceInfoTP);
        if (m_HasLED)
            defineProperty(LEDSP);
        if (m_HasTemperatureOffset)
        {
            defineProperty(TemperatureOffsetNP);
            defineProperty(TemperatureCalibrationNP);
            defineProperty(TemperatureOffsetClearSP);
        }
    }
    else
    {
        deleteProperty(FocusMinPosNP);
        if (m_TemperatureDefined)
        {
            deleteProperty(TemperatureNP);
            m_TemperatureDefined = false;
        }
        deleteProperty(DeviceInfoTP);
        if (m_HasLED)
            deleteProperty(LEDSP);
        if (m_HasTemperatureOffset)
        {
            deleteProperty(TemperatureOffsetNP);
            deleteProperty(TemperatureCalibrationNP);
            deleteProperty(TemperatureOffsetClearSP);
        }
    }

    return true;
}

bool StarpointFocuser::ISNewNumber(const char *dev, const char *name, double values[], char *names[], int n)
{
    if (dev != nullptr && strcmp(dev, getDeviceName()) == 0)
    {
        // The focuser stores its limits and speed itself. A copy saved by an older version of
        // this driver must not overwrite them when the configuration is loaded.
        if (isConfigLoading() && (FocusMaxPosNP.isNameMatch(name) || FocusMinPosNP.isNameMatch(name) || FocusSpeedNP.isNameMatch(name)))
            return true;

        // The limits are handled here, not by INDI::Focuser, which would set the minimum to 0
        if (FocusMaxPosNP.isNameMatch(name))
        {
            FocusMaxPosNP.setState(setLimits(m_MinPosition, toSteps(values[0])) ? IPS_OK : IPS_ALERT);
            FocusMaxPosNP.apply();
            return true;
        }

        if (FocusMinPosNP.isNameMatch(name))
        {
            FocusMinPosNP.setState(setLimits(toSteps(values[0]), m_MaxPosition) ? IPS_OK : IPS_ALERT);
            FocusMinPosNP.apply();
            return true;
        }

        if (TemperatureCalibrationNP.isNameMatch(name))
        {
            TemperatureCalibrationNP.update(values, names, n);
            TemperatureCalibrationNP.setState(calibrateTemperature(TemperatureCalibrationNP[0].getValue()) ? IPS_OK : IPS_ALERT);
            TemperatureCalibrationNP.apply();
            return true;
        }
    }

    return INDI::Focuser::ISNewNumber(dev, name, values, names, n);
}

bool StarpointFocuser::ISNewSwitch(const char *dev, const char *name, ISState *states, char *names[], int n)
{
    if (dev != nullptr && strcmp(dev, getDeviceName()) == 0)
    {
        // As for the limits - the focuser stores direction itself
        if (isConfigLoading() && FocusReverseSP.isNameMatch(name))
            return true;

        if (LEDSP.isNameMatch(name))
        {
            const int previous = LEDSP.findOnSwitchIndex();
            LEDSP.update(states, names, n);
            if (setLED(LEDSP.findOnSwitchIndex()))
                LEDSP.setState(IPS_OK);
            else
            {
                LEDSP.reset();
                if (previous >= 0)
                    LEDSP[previous].setState(ISS_ON);
                LEDSP.setState(IPS_ALERT);
            }
            LEDSP.apply();
            return true;
        }

        if (TemperatureOffsetClearSP.isNameMatch(name))
        {
            TemperatureOffsetClearSP.reset();
            TemperatureOffsetClearSP.setState(setTemperatureOffset(0) ? IPS_OK : IPS_ALERT);
            TemperatureOffsetClearSP.apply();
            return true;
        }
    }

    return INDI::Focuser::ISNewSwitch(dev, name, states, names, n);
}

bool StarpointFocuser::Handshake()
{
    std::string port = "simulator";
    if (!isSimulation())
        port = usingBluetooth() ? "Bluetooth " + m_BluetoothConnection->address() : std::string(serialConnection->port());

    if (!m_Protocol.identify(port, FOCUSER_REPLY))
        return false;

    if (!readConfig())
        return false;

    if (!readTemperature())
    {
        LOGF_ERROR("Could not read the focuser's temperature. %s", m_Protocol.lastError().c_str());
        return false;
    }

    if (!m_HasTemperature)
        LOG_INFO("The focuser has no temperature reading. Is a temperature probe plugged in?");

    m_FailedPolls = 0;
    m_LastPollAt = std::chrono::steady_clock::now();

    return true;
}

bool StarpointFocuser::Disconnect()
{
    RemoveTimer(m_TimerID);
    m_TimerID = -1;

    m_Protocol.sendWithoutReply("DISCON#");
    return INDI::Focuser::Disconnect();
}

bool StarpointFocuser::saveConfigItems(FILE *fp)
{
    // INDI::Focuser would also save the maximum and the direction. The focuser stores these
    // itself, and loading an old copy would overwrite what is on the device
    INDI::DefaultDevice::saveConfigItems(fp);
    PresetNP.save(fp);
    controller->saveConfigItems(fp);
    return true;
}

void StarpointFocuser::TimerHit()
{
    if (!isConnected())
        return;

    // Poll on every tick while moving, and at the polling period while idle.
    // A move command makes the focuser count as moving, so the next tick polls it.
    const auto now = std::chrono::steady_clock::now();
    if (isMoving() || now - m_LastPollAt >= std::chrono::milliseconds(getPollingPeriod()))
    {
        const bool wasLost = m_FailedPolls >= FAILED_POLLS_BEFORE_LOST;
        if (poll())
        {
            if (wasLost)
            {
                LOG_INFO("Contact with the focuser restored.");
                if (!isMoving())
                {
                    FocusAbsPosNP.setState(IPS_OK);
                    FocusAbsPosNP.apply();
                    FocusRelPosNP.setState(IPS_OK);
                    FocusRelPosNP.apply();
                }
            }
            m_FailedPolls = 0;
        }
        else
            pollFailed();

        m_LastPollAt = std::chrono::steady_clock::now();
    }

    m_TimerID = SetTimer(POLL_TICK_MS);
}

IPState StarpointFocuser::MoveAbsFocuser(uint32_t targetTicks)
{
    if (targetTicks < static_cast<uint32_t>(m_MinPosition) || targetTicks > static_cast<uint32_t>(m_MaxPosition))
    {
        LOGF_ERROR("The focuser accepts positions from %d to %d.", m_MinPosition, m_MaxPosition);
        return IPS_ALERT;
    }

    if (!m_Protocol.sendCommand("MOVE" + std::to_string(targetTicks) + "#"))
    {
        LOGF_ERROR("The focuser did not move to %u. %s", targetTicks, m_Protocol.lastError().c_str());
        return IPS_ALERT;
    }

    m_Target = static_cast<int32_t>(targetTicks);
    m_MoveCommandedAt = std::chrono::steady_clock::now();
    m_Halted = false;
    return IPS_BUSY;
}

IPState StarpointFocuser::MoveRelFocuser(FocusDirection dir, uint32_t ticks)
{
    // Follow the move in 64 bits so an inward move cannot wrap around
    const int64_t offset = dir == FOCUS_INWARD ? -static_cast<int64_t>(ticks) : static_cast<int64_t>(ticks);
    const int64_t target = static_cast<int64_t>(m_Position) + offset;
    if (target < m_MinPosition || target > m_MaxPosition)
    {
        LOGF_ERROR("The focuser accepts positions from %d to %d.", m_MinPosition, m_MaxPosition);
        return IPS_ALERT;
    }

    if (!m_Protocol.sendCommand("MOVEREL" + std::to_string(offset) + "#"))
    {
        LOGF_ERROR("The focuser did not move %u steps %s. %s", ticks, dir == FOCUS_INWARD ? "inward" : "outward",m_Protocol.lastError().c_str());
        return IPS_ALERT;
    }

    m_Target = static_cast<int32_t>(target);
    m_MoveCommandedAt = std::chrono::steady_clock::now();
    m_Halted = false;
    return IPS_BUSY;
}

bool StarpointFocuser::SyncFocuser(uint32_t ticks)
{
    if (isMoving())
    {
        LOG_ERROR("The focuser is moving. Wait for it to stop before syncing.");
        return false;
    }

    if (ticks < static_cast<uint32_t>(m_MinPosition) || ticks > static_cast<uint32_t>(m_MaxPosition))
    {
        LOGF_ERROR("The focuser accepts positions from %d to %d.", m_MinPosition, m_MaxPosition);
        return false;
    }

    if (!m_Protocol.sendCommand("SYNC" + std::to_string(ticks) + "#"))
    {
        LOGF_ERROR("Could not sync the focuser to %u. %s", ticks, m_Protocol.lastError().c_str());
        return false;
    }

    m_Position = m_Target = static_cast<int32_t>(ticks);
    LOGF_INFO("Focuser synced to position %u", ticks);
    return true;
}

bool StarpointFocuser::AbortFocuser()
{
    if (!m_Protocol.sendCommand("HALT#"))
    {
        LOGF_ERROR("Failed to abort the focuser movement. %s", m_Protocol.lastError().c_str());
        return false;
    }

    // Read where the focuser stopped
    m_Halted = true;
    if (!poll())
        pollFailed();

    m_Target = m_Position;
    m_MoveCommandedAt = {};
    return true;
}

bool StarpointFocuser::ReverseFocuser(bool enabled)
{
    // 1 is normal and -1 reversed. The focuser refuses 0.
    if (!m_Protocol.sendCommand(enabled ? "DIR-1#" : "DIR1#"))
    {
        LOGF_ERROR("Could not change the direction. %s", m_Protocol.lastError().c_str());
        return false;
    }

    LOGF_INFO("Direction set to %s.", enabled ? "reversed" : "normal");
    return true;
}

bool StarpointFocuser::SetFocuserSpeed(int speed)
{
    if (speed < MIN_SPEED || speed > MAX_SPEED)
    {
        LOGF_ERROR("The focuser speed must be between %d and %d.", MIN_SPEED, MAX_SPEED);
        return false;
    }

    if (!m_Protocol.sendCommand("SPEED" + std::to_string(speed) + "#"))
    {
        LOGF_ERROR("Unable to change focuser speed. %s", m_Protocol.lastError().c_str());
        return false;
    }

    LOGF_INFO("Speed set to %d.", speed);
    return true;
}

bool StarpointFocuser::usingBluetooth()
{
    return m_BluetoothConnection != nullptr && getActiveConnection() == m_BluetoothConnection;
}

bool StarpointFocuser::readConfig()
{
    std::string reply;
    if (!m_Protocol.query("CONFIG?#", reply))
    {
        LOGF_ERROR("Could not read the focuser's settings. %s", m_Protocol.lastError().c_str());
        return false;
    }

    json config;
    try
    {
        config = json::parse(withoutNonNumbers(reply));
    }
    catch (const json::exception &e)
    {
        LOGF_DEBUG("CONFIG? -> %s (%s)", printable(reply).c_str(), e.what());
    }

    double position = 0, minimum = 0, maximum = 0;
    if (!numberField(config, "pos", position) || !numberField(config, "min", minimum) || !numberField(config, "max", maximum))
    {
        LOGF_DEBUG("CONFIG -> %s", printable(reply).c_str());
        LOG_ERROR("Could not read the focuser's settings - its response did not include position and limits");
        return false;
    }

    m_Position = m_Target = toSteps(position);
    m_MinPosition = toSteps(minimum);
    m_MaxPosition = toSteps(maximum);
    m_ReportedMoving = false;
    m_MoveCommandedAt = {};
    m_Halted = false;

    std::string model = "Focuser", firmware = "unknown", name;
    textField(config, "model", model);
    textField(config, "fw", firmware);
    textField(config, "deviceName", name);
    DeviceInfoTP[INFO_MODEL].setText(model);
    DeviceInfoTP[INFO_FIRMWARE].setText(firmware);
    DeviceInfoTP[INFO_NAME].setText(name);
    DeviceInfoTP.setState(IPS_OK);

     // Settings stored on the focuser. Older firmware may leave some out; those are hidden.
    uint32_t capability = FOCUSER_CAN_ABS_MOVE | FOCUSER_CAN_REL_MOVE | FOCUSER_CAN_ABORT | FOCUSER_CAN_SYNC;
    double value = 0;

    if (numberField(config, "dir", value))
    {
        capability |= FOCUSER_CAN_REVERSE;
        FocusReverseSP.reset();
        FocusReverseSP[value < 0 ? INDI_ENABLED : INDI_DISABLED].setState(ISS_ON);
        FocusReverseSP.setState(IPS_OK);
    }

    if (numberField(config, "speed", value))
    {
        capability |= FOCUSER_HAS_VARIABLE_SPEED;
        FocusSpeedNP[0].setValue(value);
        FocusSpeedNP.setState(IPS_OK);
    }

    m_HasLED = numberField(config, "led", value);
    if (m_HasLED)
    {
        LEDSP.reset();
        LEDSP[ledIndexFromRaw(static_cast<int>(std::lround(value)))].setState(ISS_ON);
        LEDSP.setState(IPS_OK);
    }

    m_HasTemperatureOffset = numberField(config, "tempOffsetC", value);
    if (m_HasTemperatureOffset)
    {
        m_TemperatureOffset = std::round(value * 100) / 100;
        TemperatureOffsetNP[0].setValue(m_TemperatureOffset);
        TemperatureOffsetNP.setState(IPS_OK);
    }

    SetCapability(capability);

    FocusAbsPosNP[0].setValue(m_Position);
    FocusAbsPosNP.setState(IPS_OK);
    updateLimitProperties();

    if (name.empty())
        LOGF_INFO("Connected to %s (firmware %s).", model.c_str(), firmware.c_str());
    else
        LOGF_INFO("Connected to %s (%s, firmware %s).", name.c_str(), model.c_str(), firmware.c_str());
    LOGF_DEBUG("Position %d, limits %d to %d.", m_Position, m_MinPosition, m_MaxPosition);
    return true;
}

bool StarpointFocuser::readTemperature()
{
    std::string reply;
    bool available = false;
    double celsius = 0;

    if (m_Protocol.query("TEMP?#", reply))
    {
        json doc;
        try{
            doc = json::parse(withoutNonNumbers(reply));
        }
        catch (const json::exception &e)
        {
            LOGF_DEBUG("TEMP? => %s (%s)", printable(reply).c_str(), e.what());
            m_Protocol.setLastError("The focuser gave an unexpected reply.");
            return false;
        }

        const json *availableField = findField(doc, "available");
        if (availableField == nullptr || !availableField->is_boolean())
        {
            LOGF_DEBUG("TEMP? -> %s", printable(reply).c_str());
            m_Protocol.setLastError("The focuser gave an unexpected reply.");
            return false;
        }

        available = availableField->get<bool>() && numberField(doc, "tempC", celsius);
    }
    else if (!m_Protocol.lastRefused())
        return false;

    // A refused TEMP? counts as no reading
    m_TemperatureReadAt = std::chrono::steady_clock::now();
    const bool wasAvailable = m_HasTemperature;
    m_HasTemperature = available;

    if (available)
    {
        m_Temperature = celsius;
        TemperatureNP[0].setValue(celsius);
        TemperatureNP.setState(IPS_OK);
    }
    else
        TemperatureNP.setState(IPS_ALERT);

    // While connecting, updateProperties defines the property afterwards
    if (!isConnected())
        return true;

    if (available)
    {
        if (!m_TemperatureDefined)
        {
            defineProperty(TemperatureNP);
            m_TemperatureDefined = true;
        }
        else
            TemperatureNP.apply();

        if (!wasAvailable)
            LOG_INFO("The focuser's temperature reading is available.");
    }
    else if (wasAvailable)
    {
        TemperatureNP.apply();
        LOG_WARN("The focuser has no temperature reading. Is the temperature probe plugged in?");
    }

    return true;
}

bool StarpointFocuser::poll()
{
    // MOVING? first. Once it says stopped, the position read next is final.
    int32_t moving = 0, position = 0;
    if (!m_Protocol.queryInteger("MOVING?#", moving) || !m_Protocol.queryInteger("POS?#", position))
        return false;

    if (std::chrono::steady_clock::now() - m_TemperatureReadAt >= TEMPERATURE_INTERVAL && !readTemperature())
        return false;

    const int32_t target = m_Target;
    m_Position = position;
    m_ReportedMoving = moving == 1;
    if (m_ReportedMoving)
        m_MoveCommandedAt = {};
    else if (!isMoving())
        m_Target = position;

    const bool positionChanged = FocusAbsPosNP[0].getValue() != position;
    FocusAbsPosNP[0].setValue(position);

    if (isMoving())
    {
        FocusAbsPosNP.setState(IPS_BUSY);
        FocusAbsPosNP.apply();
    }
    else if (FocusAbsPosNP.getState() == IPS_BUSY || FocusRelPosNP.getState() == IPS_BUSY)
    {
        FocusAbsPosNP.setState(IPS_OK);
        FocusAbsPosNP.apply();
        if (FocusRelPosNP.getState() == IPS_BUSY)
        {
            FocusRelPosNP.setState(IPS_OK);
            FocusRelPosNP.apply();
        }

        if (m_Halted)
            LOGF_INFO("Focuser stopped at %d.", position);
        else if (position == target)
            LOGF_INFO("Focuser reached position %d.", position);
        else
            LOGF_WARN("The focuser stopped at %d, short of %d.", position, target);
    }
    else if (positionChanged)
        FocusAbsPosNP.apply();

    return true;
}

void StarpointFocuser::pollFailed()
{
    m_FailedPolls++;
    LOGF_DEBUG("Poll failed (%d in a row): %s", m_FailedPolls, m_Protocol.lastError().c_str());

    if (m_FailedPolls == FAILED_POLLS_BEFORE_LOST)
    {
        LOGF_ERROR("Lost contact with the focuser. %s Check its USB cable and power. If it does not recover, disconnect and connect again.", m_Protocol.lastError().c_str());
        FocusAbsPosNP.setState(IPS_ALERT);
        FocusAbsPosNP.apply();
        FocusRelPosNP.setState(IPS_ALERT);
        FocusRelPosNP.apply();
    }
}

bool StarpointFocuser::isMoving() const
{
    return m_ReportedMoving || (m_Position != m_Target && std::chrono::steady_clock::now() - m_MoveCommandedAt < MOVE_START_GRACE);
}

void StarpointFocuser::updateLimitProperties()
{
    const double range = static_cast<double>(m_MaxPosition) - m_MinPosition;

    FocusAbsPosNP[0].setMinMax(m_MinPosition, m_MaxPosition);
    FocusAbsPosNP[0].setStep(std::max(1.0, range / 50));
    FocusSyncNP[0].setMinMax(m_MinPosition, m_MaxPosition);
    FocusSyncNP[0].setStep(std::max(1.0, range / 50));
    FocusRelPosNP[0].setMinMax(0, range);
    FocusRelPosNP[0].setStep(std::max(1.0, range / 100));

    // Presets keep a minimum of 0, as INDI::Focuser leaves them, so saved presets still load
    for (auto &preset : PresetNP)
        preset.setMax(m_MaxPosition);

    FocusMaxPosNP[0].setValue(m_MaxPosition);
    FocusMinPosNP[0].setValue(m_MinPosition);

    if (isConnected())
    {
        FocusAbsPosNP.updateMinMax();
        FocusSyncNP.updateMinMax();
        FocusRelPosNP.updateMinMax();
        PresetNP.updateMinMax();
    }
}

bool StarpointFocuser::setLimits(int32_t minimum, int32_t maximum)
{
    if (minimum < 0 || maximum <= minimum)
    {
        LOGF_ERROR("The maximum position must be above the minimum, and the minimum 0 or above (asked for %d to %d).", minimum, maximum);
        updateLimitProperties();
        return false;
    }

    // Send the changes in the order that never puts minimum above maximum
    const bool maximumFirst = minimum > m_MaxPosition;
    for (int i = 0; i < 2; i++)
    {
        const bool isMaximum = (i == 0) == maximumFirst;
        const int32_t value = isMaximum ? maximum : minimum;
        int32_t &current = isMaximum ? m_MaxPosition : m_MinPosition;
        if (value == current)
            continue;

        if (!m_Protocol.sendCommand(std::string(isMaximum ? "MAX" : "MIN") + std::to_string(value) + "#"))
        {
            LOGF_ERROR("Could not change the %s position. %s", isMaximum ? "maximum" : "minimum", m_Protocol.lastError().c_str());
            updateLimitProperties();
            return false;
        }

        current = value;
    }

    updateLimitProperties();
    LOGF_INFO("Focuser limits set to %d to %d.", m_MinPosition, m_MaxPosition);
    return true;
}

bool StarpointFocuser::setLED(int index)
{
    if (index < 0)
        return false;

    if (!m_Protocol.sendCommand("LED" + std::to_string(LED_LEVELS[index]) + "#"))
    {
        LOGF_ERROR("Could not change the status light brightness level. %s", m_Protocol.lastError().c_str());
        return false;
    }

    LOGF_INFO("Status light brightness level set to %s.", LEDSP[index].getLabel());
    return true;
}

bool StarpointFocuser::calibrateTemperature(double actual)
{
    if (!readTemperature())
    {
        LOGF_ERROR("Could not read the focuser's temperature. %s", m_Protocol.lastError().c_str());
        return false;
    }

    if (!m_HasTemperature)
    {
        LOG_ERROR("The focuser has no temperature to calibrate against.");
        return false;
    }

    // The reading includes the current offset, so the probe's own reading is the difference 
    const double probe = m_Temperature - m_TemperatureOffset;
    const double offset = std::round((actual - probe) * 100) / 100;
    if (std::fabs(offset) > MAX_TEMPERATURE_OFFSET)
    {
        LOGF_ERROR("%.2f °C is more than %.0f °C from the probe's reading of %.2f °C. A temperature offset this high cannot be set.", actual, MAX_TEMPERATURE_OFFSET, probe);
        return false;
    }

    return setTemperatureOffset(offset);
}

bool StarpointFocuser::setTemperatureOffset(double offset)
{
    if (!m_Protocol.sendCommand("TEMPOFFSET" + formatOffset(offset) + "#"))
    {
        LOGF_ERROR("Could not set the temperature offset. %s", m_Protocol.lastError().c_str());
        TemperatureOffsetNP.setState(IPS_ALERT);
        TemperatureOffsetNP.apply();
        return false;
    }

    m_TemperatureOffset = offset;
    TemperatureOffsetNP[0].setValue(offset);
    TemperatureOffsetNP.setState(IPS_OK);
    TemperatureOffsetNP.apply();

    // Show the new reading at the next poll
    m_TemperatureReadAt = {};
    LOGF_INFO("Temperature offset set to %s °C.", formatOffset(offset).c_str());
    return true;
}

std::string StarpointFocuser::Simulator::reply(const std::string &command)
{
    update();

    const std::string body = withoutTerminator(command);
    const auto now = std::chrono::steady_clock::now();

    // Commands that set something carry a number after their name
    double value = 0;
    auto valueAfter = [&body, &value](const char *name)
    {
        const size_t length = strlen(name);
        if (body.size() <= length || body.compare(0, length, name) != 0)
            return false;
        char *end = nullptr;
        value = std::strtod(body.c_str() + length, &end);
        return *end == '\0';
    };

    if (body == "PING?")
        return FOCUSER_REPLY;

    if (body == "CONFIG?")
    {
        const json config =
        {
            {"model", "Simulator"}, {"deviceName", "Simulated focuser"}, {"fw", "simulator"},
            {"pos", m_Position}, {"min", m_Min}, {"max", m_Max},
            {"speed", m_Speed}, {"dir", m_Direction}, {"led", m_LED}, {"tempOffsetC", m_TemperatureOffset}
        };
        return config.dump();
    }

    if (body == "POS?")
        return std::to_string(m_Position);

    if (body == "MOVING?")
        return m_Moving ? "1" : "0";

    if (body == "TEMP?")
    {
        // Drifts by up to a degree over an hour so changes can be seen
        const double hours = std::chrono::duration<double>(now - m_CreatedAt).count() / 3600;
        const double celsius = 12.5 + std::sin(hours * 2 * 3.14159265358979) + m_TemperatureOffset;
        const json temperature = {{"available", true}, {"tempC", std::round(celsius * 100) / 100}};
        return temperature.dump();
    }

    if (body == "HALT")
    {
        m_Target = m_Position;
        m_Moving = false;
        return ack(true);
    }

    if (valueAfter("MOVEREL"))
    {
        m_Target = static_cast<int32_t>(std::clamp<double>(m_Position + value, m_Min, m_Max));
        m_MoveStartedAt = now;
        return ack(true);
    }

    if (valueAfter("MOVE"))
    {
        if (value < m_Min || value > m_Max)
            return ack(false, "range");
        m_Target = static_cast<int32_t>(value);
        m_MoveStartedAt = now;
        return ack(true);
    }

    if (valueAfter("SYNC"))
    {
        if (m_Moving)
            return ack(false, "busy");
        if (value < m_Min || value > m_Max)
            return ack(false, "range");
        m_Position = m_Target = static_cast<int32_t>(value);
        return ack(true);
    }

    if (valueAfter("MIN"))
    {
        if (value < 0 || value >= m_Max)
            return ack(false, "range");
        m_Min = static_cast<int32_t>(value);
        return ack(true);
    }

    if (valueAfter("MAX"))
    {
        if (value <= m_Min)
            return ack(false, "range");
        m_Max = static_cast<int32_t>(value);
        return ack(true);
    }

    if (valueAfter("DIR"))
    {
        if (value != 1 && value != -1)
            return ack(false, "range");
        m_Direction = static_cast<int>(value);
        return ack(true);
    }

    if (valueAfter("SPEED"))
    {
        if (value < MIN_SPEED || value > MAX_SPEED)
            return ack(false, "range");
        m_Speed = static_cast<int>(value);
        return ack(true);
    }

    if (valueAfter("LED"))
    {
        if (value < 0 || value > 255)
            return ack(false, "range");
        m_LED = static_cast<int>(value);
        return ack(true);
    }

    if (valueAfter("TEMPOFFSET"))
    {
        if (std::fabs(value) > MAX_TEMPERATURE_OFFSET)
            return ack(false, "range");
        m_TemperatureOffset = value;
        return ack(true);
    }

    return ack(false, "unsupported");
}

void StarpointFocuser::Simulator::update()
{
    const auto now = std::chrono::steady_clock::now();
    if (m_Position == m_Target)
    {
        m_Moving = false;
        return;
    }

    if (!m_Moving)
    {
        if (now - m_MoveStartedAt < std::chrono::milliseconds(300))
            return;
        m_Moving = true;
        m_UpdatedAt = now;
        m_Travelled = 0;
    }

    m_Travelled += m_Speed * 2000.0 / 180 * std::chrono::duration<double>(now - m_UpdatedAt).count();
    m_UpdatedAt = now;
    const int32_t steps = static_cast<int32_t>(m_Travelled);
    m_Travelled -= steps;

    const int32_t remaining = m_Target - m_Position;
    if (std::abs(remaining) <= steps)
    {
        m_Position = m_Target;
        m_Moving = false;
    }
    else
        m_Position += remaining > 0 ? steps : -steps;
}

std::string StarpointFocuser::Simulator::ack(bool ok, const char *error) const
{
    json reply = {{"ok", ok}};
    if (error != nullptr)
        reply["error"] = error;
    return reply.dump();
}