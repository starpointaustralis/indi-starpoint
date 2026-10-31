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

#include "starpoint_filterwheel.h"

#include "config.h"
#include "indicom.h"
#include "indicontroller.h"
#include "connectionplugins/connectionserial.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <iterator>
#include <memory>

using json = nlohmann::json;

using Starpoint::findField;
using Starpoint::numberField;
using Starpoint::printable;
using Starpoint::textField;
using Starpoint::trim;
using Starpoint::withoutNonNumbers;
using Starpoint::withoutTerminator;

namespace
{
    // Polling
    constexpr uint32_t POLL_TICK_MS = 250;
    constexpr uint32_t IDLE_POLL_MS = 2000;
    constexpr int FAILED_POLLS_BEFORE_LOST = 3;

    // Power, dew heater and conditions are read this often while the wheel is still
    constexpr auto AUXILIARY_POLL = std::chrono::seconds(5);

    // A move has finished when the wheel reports the slot it was sent to, or after this long
    constexpr auto MOVE_TIMEOUT = std::chrono::seconds(30);

    const std::string FILTER_WHEEL_REPLY = "FOUND1985";

    constexpr size_t MAX_NAME_LENGTH = 32;

    // The filter wheel's Bluetooth LE services: commands go to the transmit characteristic and
    // replies come back as notifications on the receive characteristic
    const StarpointBluetoothLink::Profile BLUETOOTH_PROFILE
    {
        "filter wheel",
        "e3261367-0e2c-4822-bc50-4cf24dd11a9d", "71df3c63-d369-4b1f-8002-e282a51b3f72",
        "ab0c8585-b0de-4b0a-83f3-77489fcc27a6", "3df517c5-fd4c-4fbb-8095-e9d7bc0d5215"
    };

    // LED brightness as the firmware's raw values: Off, Low, Medium, High
    constexpr int LED_LEVELS[] {0, 10, 80, 150};

    // The nearest LED level at or below a raw value the wheel reported
    int ledIndexFromRaw(int raw)
    {
        for (int i = static_cast<int>(std::size(LED_LEVELS)) - 1; i > 0; i--)
        {
            if (raw >= LED_LEVELS[i])
                return i;
        }
        return 0;
    }

    // A slot's name when the wheel has none for it, counted from 1 as the users see them
    std::string defaultName(int slot)
    {
        return "Filter " + std::to_string(slot + 1);
    }

    // Power outputs 1 and 2, which the wheel calls A and B
    constexpr int POWER_OUTPUTS = 2;

    // The dew heater's modes, as DEWMODE numbers them
    enum DewMode
    {
        DEW_OFF,
        DEW_MANUAL,
        DEW_AUTO
    };

    const char *const DEW_MODE_NAMES[] = {"off", "manual", "automatic"};

    constexpr int MAX_DEW_DUTY = 100;

    // Whether the wheel's CONFIG? capabilities include one, such as "dewHeater"
    bool hasCapability(const json *capabilities, const char *name)
    {
        const json *field = capabilities != nullptr ? findField(*capabilities, name) : nullptr;
        return field != nullptr && field->is_boolean() && field->get<bool>();
    }
}

static std::unique_ptr<StarpointFilterWheel> starpointFilterWheel(new StarpointFilterWheel());

StarpointFilterWheel::StarpointFilterWheel() : INDI::PowerInterface(this), m_Protocol(this, "filter wheel")
{
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
    setFilterConnection(CONNECTION_SERIAL);
    setDefaultPollingPeriod(IDLE_POLL_MS);
}

const char *StarpointFilterWheel::getDefaultName()
{
    return "Starpoint Australis Filter Wheel";
}

bool StarpointFilterWheel::initProperties()
{
    INDI::FilterWheel::initProperties();

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

    DeviceInfoTP[INFO_MODEL].fill("MODEL", "Model", "");
    DeviceInfoTP[INFO_FIRMWARE].fill("FIRMWARE", "Firmware", "");
    DeviceInfoTP[INFO_NAME].fill("NAME", "Name", "");
    DeviceInfoTP.fill(getDeviceName(), "DEVICE_INFO", "Filter Wheel", INFO_TAB, IP_RO, 0, IPS_IDLE);

    LEDSP[LED_OFF].fill("LED_OFF", "Off", ISS_OFF);
    LEDSP[LED_LOW].fill("LED_LOW", "Low", ISS_OFF);
    LEDSP[LED_MEDIUM].fill("LED_MEDIUM", "Medium", ISS_OFF);
    LEDSP[LED_HIGH].fill("LED_HIGH", "High", ISS_OFF);
    LEDSP.fill(getDeviceName(), "LED_BRIGHTNESS", "Status Light", OPTIONS_TAB, IP_RW, ISR_1OFMANY, 60, IPS_IDLE);

    // Two power outputs and one dew heater, with automatic control. Whether a wheel has them is
    // only known once connected.
    PI::initProperties(POWER_TAB, POWER_OUTPUTS, 1, 0, 1, 0);

    ConditionsNP[CONDITIONS_TEMPERATURE].fill("TEMPERATURE", "Temperature (C)", "%.1f", -100, 100, 0, 0);
    ConditionsNP[CONDITIONS_HUMIDITY].fill("HUMIDITY", "Humidity (%)", "%.0f", 0, 100, 0, 0);
    ConditionsNP[CONDITIONS_PRESSURE].fill("PRESSURE", "Pressure (hPa)", "%.1f", 0, 2000, 0, 0);
    ConditionsNP[CONDITIONS_DEW_POINT].fill("DEW_POINT", "Dew Point (C)", "%.1f", -100, 100, 0, 0);
    ConditionsNP.fill(getDeviceName(), "CONDITIONS", "Conditions", DEW_TAB, IP_RO, 60, IPS_IDLE);

    // Debug, simulation and configuration controls
    addAuxControls();

    return true;
}

bool StarpointFilterWheel::updateProperties()
{
    INDI::FilterWheel::updateProperties();

    // INDI::PowerInterface::updateProperties would also define over-voltage protection and power
    // off on disconnect, which the wheel does not have, so the power properties are defined here
    if (isConnected())
    {
        defineProperty(DeviceInfoTP);
        if (m_HasLED)
            defineProperty(LEDSP);
        if (m_HasPower)
        {
            defineProperty(PowerSensorsNP);
            defineProperty(PowerChannelsSP);
            defineProperty(PowerChannelCurrentNP);
            defineProperty(PowerChannelLabelsTP);
        }
        if (m_HasDewHeater)
        {
            defineProperty(DewChannelsSP);
            defineProperty(DewChannelDutyCycleNP);
            defineProperty(AutoDewSP);
            defineProperty(DewChannelLabelsTP);
        }
        if (m_HasConditions)
            defineProperty(ConditionsNP);
    }
    else
    {
        deleteProperty(DeviceInfoTP);
        if (m_HasLED)
            deleteProperty(LEDSP);
        if (m_HasPower)
        {
            deleteProperty(PowerSensorsNP);
            deleteProperty(PowerChannelsSP);
            deleteProperty(PowerChannelCurrentNP);
            deleteProperty(PowerChannelLabelsTP);
        }
        if (m_HasDewHeater)
        {
            deleteProperty(DewChannelsSP);
            deleteProperty(DewChannelDutyCycleNP);
            deleteProperty(AutoDewSP);
            deleteProperty(DewChannelLabelsTP);
        }
        if (m_HasConditions)
            deleteProperty(ConditionsNP);
    }

    return true;
}

bool StarpointFilterWheel::ISNewNumber(const char *dev, const char *name, double values[], char *names[], int n)
{
    if (dev != nullptr && strcmp(dev, getDeviceName()) == 0 && DewChannelDutyCycleNP.isNameMatch(name))
    {
        // The wheel stores its dew heater setting. A copy in INDI's configuration must not change
        // it when the configuration is loaded.
        if (isConfigLoading())
            return true;

        // While the heater is off, the duty cycle is kept for when it is turned on. INDI rejects
        // a value outside 0 to 100%.
        const double previous = DewChannelDutyCycleNP[0].getValue();
        if (!DewChannelDutyCycleNP.update(values, names, n))
            DewChannelDutyCycleNP.setState(IPS_ALERT);
        else if (DewChannelsSP[0].getState() == ISS_OFF || SetDewPort(0, true, DewChannelDutyCycleNP[0].getValue()))
            DewChannelDutyCycleNP.setState(IPS_OK);
        else
        {
            DewChannelDutyCycleNP[0].setValue(previous);
            DewChannelDutyCycleNP.setState(IPS_ALERT);
        }
        DewChannelDutyCycleNP.apply();
        return true;
    }

    return INDI::FilterWheel::ISNewNumber(dev, name, values, names, n);
}

bool StarpointFilterWheel::ISNewText(const char *dev, const char *name, char *texts[], char *names[], int n)
{
    // Renames are handled here: INDI::FilterWheel passes one that failed on to INDI::DefaultDevice,
    // which would keep the rejected names, and adds a second, vaguer error message
    if (dev != nullptr && strcmp(dev, getDeviceName()) == 0 && FilterNameTP.isNameMatch(name))
    {
        // The wheel stores its filter names. A copy in INDI's configuration must not change them
        // when the configuration is loaded.
        if (isConfigLoading())
            return true;

        FilterNameTP.update(texts, names, n);
        FilterNameTP.setState(SetFilterNames() ? IPS_OK : IPS_ALERT);
        FilterNameTP.apply();
        return true;
    }

    // Labels for the power outputs and the dew heater, which INDI's configuration keeps
    if (PI::processText(dev, name, texts, names, n))
        return true;

    return INDI::FilterWheel::ISNewText(dev, name, texts, names, n);
}

bool StarpointFilterWheel::ISNewSwitch(const char *dev, const char *name, ISState *states, char *names[], int n)
{
    if (dev != nullptr && strcmp(dev, getDeviceName()) == 0 && LEDSP.isNameMatch(name))
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

    if (dev != nullptr && strcmp(dev, getDeviceName()) == 0
            && (PowerChannelsSP.isNameMatch(name) || DewChannelsSP.isNameMatch(name) || AutoDewSP.isNameMatch(name)))
    {
        // The wheel stores its outputs and dew heater setting. A copy in INDI's configuration must
        // not change them when the configuration is loaded.
        if (isConfigLoading())
            return true;

        if (PowerChannelsSP.isNameMatch(name))
        {
            switchEach(PowerChannelsSP, states, names, n, [this](size_t port, bool on)
            {
                return SetPowerPort(port, on);
            });
        }
        else if (DewChannelsSP.isNameMatch(name))
        {
            switchEach(DewChannelsSP, states, names, n, [this](size_t port, bool on)
            {
                return SetDewPort(port, on, DewChannelDutyCycleNP[port].getValue());
            });
        }
        else
        {
            switchEach(AutoDewSP, states, names, n, [this](size_t port, bool on)
            {
                return SetAutoDewEnabled(port, on);
            });
        }
        return true;
    }

    return INDI::FilterWheel::ISNewSwitch(dev, name, states, names, n);
}

bool StarpointFilterWheel::Handshake()
{
    std::string port = "simulator";
    if (!isSimulation())
        port = usingBluetooth() ? "Bluetooth " + m_BluetoothConnection->address() : std::string(serialConnection->port());

    if (!m_Protocol.identify(port, FILTER_WHEEL_REPLY))
        return false;

    if (!readConfig())
        return false;

    if (!pollAuxiliary())
    {
        LOGF_ERROR("Could not read the filter wheel's power outputs, dew heater or weather sensor. %s",
                   m_Protocol.lastError().c_str());
        return false;
    }

    m_FailedPolls = 0;
    m_LastPollAt = std::chrono::steady_clock::now();
    return true;
}

bool StarpointFilterWheel::Disconnect()
{
    RemoveTimer(m_TimerID);
    m_TimerID = -1;

    m_Protocol.sendWithoutReply("DISCON#");
    return INDI::FilterWheel::Disconnect();
}

bool StarpointFilterWheel::saveConfigItems(FILE *fp)
{
    // INDI::FilterWheel would also save the slot and the filter names. The wheel stores these
    // itself, and loading an old copy would turn the wheel or rename its filters
    INDI::DefaultDevice::saveConfigItems(fp);
    controller->saveConfigItems(fp);

    // Of INDI::PowerInterface's settings, only the labels. The wheel stores its outputs and dew
    // heater setting.
    PowerChannelLabelsTP.save(fp);
    DewChannelLabelsTP.save(fp);
    return true;
}

void StarpointFilterWheel::TimerHit()
{
    if (!isConnected())
        return;

    // Poll on every tick while moving or finishing a selection, and at the polling period while idle
    const auto now = std::chrono::steady_clock::now();
    if (isMoving() || FilterSlotNP.getState() == IPS_BUSY || now - m_LastPollAt >= std::chrono::milliseconds(getPollingPeriod()))
    {
        const bool wasLost = m_FailedPolls >= FAILED_POLLS_BEFORE_LOST;
        if (poll())
        {
            if (wasLost)
            {
                LOG_INFO("Contact with the filter wheel restored.");
                if (!isMoving() && FilterSlotNP.getState() == IPS_ALERT)
                {
                    FilterSlotNP.setState(IPS_OK);
                    FilterSlotNP.apply();
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

int StarpointFilterWheel::QueryFilter()
{
    return m_Position >= 0 ? m_Position + 1 : -1;
}

bool StarpointFilterWheel::SelectFilter(int position)
{
    // INDI has already checked the slot is between 1 and m_Slots
    if (!m_Protocol.sendCommand("MOVE" + std::to_string(position - 1) + "#"))
    {
        LOGF_ERROR("The filter wheel did not move to slot %d. %s", position, m_Protocol.lastError().c_str());
        return false;
    }

    m_Target = position - 1;
    m_MoveCommandedAt = std::chrono::steady_clock::now();
    return true;
}

bool StarpointFilterWheel::SetFilterNames()
{
    // FilterNameTP holds the names asked for. Write the ones that changed to the wheel; if one
    // fails, show the wheel's names again.
    for (int i = 0; i < m_Slots && i < static_cast<int>(FilterNameTP.size()); i++)
    {
        const char *text = FilterNameTP[i].getText();
        const std::string name = trim(text != nullptr ? text : "");
        if (name == m_Names[i])
            continue;

        std::string problem;
        if (name.empty())
            problem = "needs a name.";
        else if (name.size() > MAX_NAME_LENGTH)
            problem = "can have a name of up to " + std::to_string(MAX_NAME_LENGTH) + " characters.";
        else if (name.find('#') != std::string::npos)
            problem = "cannot have '#' in its name.";
        else
        {
            // The name goes as JSON, with anything outside ASCII escaped, as the Starpoint Hub sends it
            const json payload = {{"i", i}, {"name", name}};
            if (!m_Protocol.sendCommand("FILNAME" + payload.dump(-1, ' ', true) + "#"))
                problem = "could not be renamed. " + m_Protocol.lastError();
        }

        if (!problem.empty())
        {
            LOGF_ERROR("Slot %d %s", i + 1, problem.c_str());
            for (int j = 0; j < m_Slots && j < static_cast<int>(FilterNameTP.size()); j++)
                FilterNameTP[j].setText(m_Names[j]);
            return false;
        }

        m_Names[i] = name;
        LOGF_INFO("Slot %d renamed to %s.", i + 1, name.c_str());
    }

    return true;
}

bool StarpointFilterWheel::GetFilterNames()
{
    // The names as the wheel stores them
    FilterNameTP.resize(0);
    for (int i = 0; i < m_Slots; i++)
    {
        char name[MAXINDINAME];
        char label[MAXINDILABEL];
        snprintf(name, MAXINDINAME, "FILTER_SLOT_NAME_%d", i + 1);
        snprintf(label, MAXINDILABEL, "Filter#%d", i + 1);

        INDI::WidgetText slot;
        slot.fill(name, label, m_Names[i]);
        FilterNameTP.push(std::move(slot));
    }

    FilterNameTP.fill(getDeviceName(), "FILTER_NAME", "Filter", FilterSlotNP.getGroupName(), IP_RW, 0, IPS_IDLE);
    FilterNameTP.shrink_to_fit();
    return true;
}

bool StarpointFilterWheel::SetPowerPort(size_t port, bool enabled)
{
    // INDI's outputs 1 and 2 are the wheel's outputs A and B
    const int output = static_cast<int>(port) + 1;
    const std::string command = std::string("OUT") + (port == 0 ? "A" : "B") + "ENABLE" + (enabled ? "1" : "0") + "#";
    if (!m_Protocol.sendCommand(command))
    {
        LOGF_ERROR("Could not switch power output %d %s. %s", output, enabled ? "on" : "off", m_Protocol.lastError().c_str());
        return false;
    }

    LOGF_INFO("Power output %d switched %s.", output, enabled ? "on" : "off");

    // Its readings follow on the next poll
    m_AuxiliaryPolledAt = {};
    return true;
}

bool StarpointFilterWheel::SetDewPort(size_t port, bool enabled, double dutyCycle)
{
    // The wheel has one dew heater. On is its manual mode, at the duty cycle asked for.
    INDI_UNUSED(port);
    return setDewHeater(enabled ? DEW_MANUAL : DEW_OFF, static_cast<int>(std::lround(dutyCycle)));
}

bool StarpointFilterWheel::SetAutoDewEnabled(size_t port, bool enabled)
{
    // Turning automatic off leaves the heater on, in manual mode at the duty cycle shown
    INDI_UNUSED(port);
    return setDewHeater(enabled ? DEW_AUTO : DEW_MANUAL, static_cast<int>(std::lround(DewChannelDutyCycleNP[0].getValue())));
}

bool StarpointFilterWheel::usingBluetooth()
{
    return m_BluetoothConnection != nullptr && getActiveConnection() == m_BluetoothConnection;
}

bool StarpointFilterWheel::readConfig()
{
    std::string reply;
    if (!m_Protocol.query("CONFIG?#", reply))
    {
        LOGF_ERROR("Could not read the filter wheel's settings. %s", m_Protocol.lastError().c_str());
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

    double slots = 0;
    if (!numberField(config, "positions", slots) || slots < 1)
    {
        LOGF_DEBUG("CONFIG -> %s", printable(reply).c_str());
        LOG_ERROR("Could not read the filter wheel's settings - its response did not include its filter slots");
        return false;
    }

    m_Slots = static_cast<int>(slots);

    // -1 while the wheel is turning
    double position = 0;
    numberField(config, "currentPosition", position);
    m_Position = static_cast<int>(position);
    m_Target = std::max(m_Position, 0);
    m_MoveCommandedAt = {};

    // The names stored on the wheel, by slot; a slot without one gets a plain name
    m_Names.clear();
    for (int i = 0; i < m_Slots; i++)
        m_Names.push_back(defaultName(i));
    const json *filters = findField(config, "filters");
    if (filters != nullptr && filters->is_array())
    {
        for (const json &filter : *filters)
        {
            double slot = -1;
            std::string name;
            if (numberField(filter, "position", slot) && textField(filter, "name", name) && slot >= 0 && slot < m_Slots
                    && !trim(name).empty())
                m_Names[static_cast<int>(slot)] = trim(name);
        }
    }

    std::string model = "Filter wheel", firmware = "unknown", name;
    textField(config, "device", model);
    textField(config, "fw", firmware);
    textField(config, "deviceName", name);
    DeviceInfoTP[INFO_MODEL].setText(model);
    DeviceInfoTP[INFO_FIRMWARE].setText(firmware);
    DeviceInfoTP[INFO_NAME].setText(name);
    DeviceInfoTP.setState(IPS_OK);

    // Settings stored on the wheel. Older firmware may leave some out; those are hidden.
    double value = 0;
    m_HasLED = numberField(config, "led", value);
    if (m_HasLED)
    {
        LEDSP.reset();
        LEDSP[ledIndexFromRaw(static_cast<int>(std::lround(value)))].setState(ISS_ON);
        LEDSP.setState(IPS_OK);
    }

    // Power outputs, a dew heater and a weather sensor, on wheels that have them
    const json *capabilities = findField(config, "capabilities");
    m_HasPower = hasCapability(capabilities, "powerMonitor");
    m_HasDewHeater = hasCapability(capabilities, "dewHeater");
    m_HasConditions = hasCapability(capabilities, "weather");
    m_ConditionsAvailable = true;
    m_AuxiliaryPolledAt = {};

    uint32_t power = 0;
    if (m_HasPower)
        power |= POWER_HAS_DC_OUT | POWER_HAS_VOLTAGE_SENSOR | POWER_HAS_OVERALL_CURRENT | POWER_HAS_PER_PORT_CURRENT;
    if (m_HasDewHeater)
        power |= POWER_HAS_DEW_OUT | POWER_HAS_AUTO_DEW;
    SetCapability(power);
    setDriverInterface(FILTER_INTERFACE | (power != 0 ? POWER_INTERFACE : 0));
    syncDriverInfo();

    FilterSlotNP[0].setMinMax(1, m_Slots);
    FilterSlotNP[0].setValue(m_Position >= 0 ? m_Position + 1 : 1);
    FilterSlotNP.setState(m_Position >= 0 ? IPS_OK : IPS_BUSY);
    CurrentFilter = m_Position >= 0 ? m_Position + 1 : 1;
    GetFilterNames();

    if (name.empty())
        LOGF_INFO("Connected to %s (firmware %s), with %d filter slots.", model.c_str(), firmware.c_str(), m_Slots);
    else
        LOGF_INFO("Connected to %s (%s, firmware %s), with %d filter slots.", name.c_str(), model.c_str(), firmware.c_str(),
                  m_Slots);
    for (int i = 0; i < m_Slots; i++)
        LOGF_DEBUG("Slot %d: %s", i + 1, m_Names[i].c_str());
    LOGF_DEBUG("Power outputs: %s. Dew heater: %s. Weather sensor: %s.", m_HasPower ? "yes" : "no",
               m_HasDewHeater ? "yes" : "no", m_HasConditions ? "yes" : "no");
    return true;
}

bool StarpointFilterWheel::poll()
{
    // The wheel reports -1 while it is turning
    int32_t position = 0;
    if (!m_Protocol.queryInteger("POS?#", position))
        return false;

    // A move the driver did not send, such as one under way when it connected
    const bool commanded = m_MoveCommandedAt != std::chrono::steady_clock::time_point();
    const int requested = m_Target;
    m_Position = position;
    if (position >= 0 && (position == m_Target || std::chrono::steady_clock::now() - m_MoveCommandedAt >= MOVE_TIMEOUT))
    {
        m_Target = position;
        m_MoveCommandedAt = {};
    }

    if (isMoving())
    {
        if (FilterSlotNP.getState() != IPS_BUSY)
        {
            FilterSlotNP.setState(IPS_BUSY);
            FilterSlotNP.apply();
        }
    }
    else if (FilterSlotNP.getState() == IPS_BUSY && position >= 0 && position < m_Slots)
    {
        if (!commanded || position == requested)
            LOGF_INFO("Filter wheel at slot %d (%s).", position + 1, m_Names[position].c_str());
        else
            LOGF_WARN("The filter wheel stopped at slot %d (%s), not slot %d.", position + 1, m_Names[position].c_str(),
                      requested + 1);
        CurrentFilter = position + 1;
        SelectFilterDone(position + 1);
    }

    // Power, dew heater and conditions, while the wheel is still
    if (!isMoving() && std::chrono::steady_clock::now() - m_AuxiliaryPolledAt >= AUXILIARY_POLL)
        return pollAuxiliary();

    return true;
}

void StarpointFilterWheel::pollFailed()
{
    m_FailedPolls++;
    LOGF_DEBUG("Poll failed (%d in a row): %s", m_FailedPolls, m_Protocol.lastError().c_str());

    if (m_FailedPolls == FAILED_POLLS_BEFORE_LOST)
    {
        LOGF_ERROR("Lost contact with the filter wheel. %s Check its USB cable and power. If it does not recover, disconnect and connect again.",
                   m_Protocol.lastError().c_str());
        FilterSlotNP.setState(IPS_ALERT);
        FilterSlotNP.apply();
    }
}

bool StarpointFilterWheel::isMoving() const
{
    // Moving while the wheel reports -1, and until it reports the slot it was sent to
    return m_Position < 0 || (m_Position != m_Target && std::chrono::steady_clock::now() - m_MoveCommandedAt < MOVE_TIMEOUT);
}

bool StarpointFilterWheel::setLED(int index)
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

// Makes the changes a client sent to a set of switches, one switch at a time, matched by name.
// INDI::PowerInterface would match them by position, which goes wrong when a client sends only
// some of them, and would save them in INDI's configuration, though the wheel stores them.
void StarpointFilterWheel::switchEach(INDI::PropertySwitch &property, ISState *states, char *names[], int n,
                                      const std::function<bool(size_t, bool)> &set)
{
    bool ok = true;
    for (size_t i = 0; i < property.size(); i++)
    {
        for (int j = 0; j < n; j++)
        {
            if (!property[i].isNameMatch(names[j]) || property[i].getState() == states[j])
                continue;

            if (set(i, states[j] == ISS_ON))
                property[i].setState(states[j]);
            else
                ok = false;
        }
    }

    property.setState(ok ? IPS_OK : IPS_ALERT);
    property.apply();
}

// Power, dew heater and conditions, for those the wheel has
bool StarpointFilterWheel::pollAuxiliary()
{
    m_AuxiliaryPolledAt = std::chrono::steady_clock::now();
    return (!m_HasPower || readPower()) && (!m_HasDewHeater || readDewHeater()) && (!m_HasConditions || readConditions());
}

bool StarpointFilterWheel::readPower()
{
    std::string reply;
    if (!m_Protocol.query("POWER?#", reply))
        return false;

    json power;
    try
    {
        power = json::parse(withoutNonNumbers(reply));
    }
    catch (const json::exception &e)
    {
        LOGF_DEBUG("POWER? -> %s (%s)", printable(reply).c_str(), e.what());
        m_Protocol.setLastError("The filter wheel gave an unexpected reply.");
        return false;
    }

    // The input, which is always on
    double value = 0;
    const json *input = findField(power, "input");
    if (input != nullptr)
    {
        if (numberField(*input, "voltage", value))
            PowerSensorsNP[SENSOR_VOLTAGE].setValue(value);
        if (numberField(*input, "current", value))
            PowerSensorsNP[SENSOR_CURRENT].setValue(value);
        if (numberField(*input, "powerW", value))
            PowerSensorsNP[SENSOR_POWER].setValue(value);
    }
    PowerSensorsNP.setState(IPS_OK);

    // The outputs, "out1" and "out2"
    bool switched = false;
    for (int i = 0; i < POWER_OUTPUTS; i++)
    {
        const json *output = findField(power, ("out" + std::to_string(i + 1)).c_str());
        if (output == nullptr)
            continue;

        if (numberField(*output, "current", value))
            PowerChannelCurrentNP[i].setValue(value);

        const json *enabled = findField(*output, "enabled");
        if (enabled != nullptr && enabled->is_boolean())
        {
            const ISState state = enabled->get<bool>() ? ISS_ON : ISS_OFF;
            switched = switched || PowerChannelsSP[i].getState() != state;
            PowerChannelsSP[i].setState(state);
        }
    }
    PowerChannelCurrentNP.setState(IPS_OK);
    PowerChannelsSP.setState(IPS_OK);

    // While connecting, updateProperties defines the properties afterwards
    if (!isConnected())
        return true;

    PowerSensorsNP.apply();
    PowerChannelCurrentNP.apply();
    if (switched)
        PowerChannelsSP.apply();
    return true;
}

bool StarpointFilterWheel::readDewHeater()
{
    int32_t mode = 0, duty = 0;
    if (!m_Protocol.queryInteger("DEWMODE?#", mode) || !m_Protocol.queryInteger("DEWDUTY?#", duty))
        return false;

    // A mode the driver does not know counts as off, as in the Starpoint Hub
    if (mode != DEW_MANUAL && mode != DEW_AUTO)
        mode = DEW_OFF;
    duty = std::clamp(duty, 0, MAX_DEW_DUTY);

    // In automatic mode the wheel changes the duty cycle as it goes, so this is only for debugging
    if (mode != m_DewMode || duty != m_DewDuty)
        LOGF_DEBUG("Dew heater %s, at %d%%.", DEW_MODE_NAMES[mode], duty);

    m_DewMode = mode;
    m_DewDuty = duty;
    showDewHeater();
    return true;
}

bool StarpointFilterWheel::readConditions()
{
    std::string reply;
    if (!m_Protocol.query("WEATHER?#", reply))
        return false;

    json weather;
    try
    {
        weather = json::parse(withoutNonNumbers(reply));
    }
    catch (const json::exception &e)
    {
        LOGF_DEBUG("WEATHER? -> %s (%s)", printable(reply).c_str(), e.what());
        m_Protocol.setLastError("The filter wheel gave an unexpected reply.");
        return false;
    }

    const json *availableField = findField(weather, "available");
    if (availableField == nullptr || !availableField->is_boolean())
    {
        LOGF_DEBUG("WEATHER? -> %s", printable(reply).c_str());
        m_Protocol.setLastError("The filter wheel gave an unexpected reply.");
        return false;
    }

    const bool available = availableField->get<bool>();
    if (available)
    {
        double value = 0;
        if (numberField(weather, "tempC", value))
            ConditionsNP[CONDITIONS_TEMPERATURE].setValue(value);
        if (numberField(weather, "humidity", value))
            ConditionsNP[CONDITIONS_HUMIDITY].setValue(value);
        if (numberField(weather, "pressureHpa", value))
            ConditionsNP[CONDITIONS_PRESSURE].setValue(value);
        if (numberField(weather, "dewPointC", value))
            ConditionsNP[CONDITIONS_DEW_POINT].setValue(value);
    }
    ConditionsNP.setState(available ? IPS_OK : IPS_ALERT);

    if (available && !m_ConditionsAvailable)
        LOG_INFO("The filter wheel's weather sensor is reading again.");
    else if (!available && m_ConditionsAvailable)
        LOG_WARN("The filter wheel's weather sensor has no reading.");
    m_ConditionsAvailable = available;

    // While connecting, updateProperties defines the property afterwards
    if (isConnected())
        ConditionsNP.apply();
    return true;
}

// Sets the wheel's dew heater mode, and in manual mode its duty cycle
bool StarpointFilterWheel::setDewHeater(int mode, int duty)
{
    // Read it back soon, whether or not this works
    m_AuxiliaryPolledAt = {};

    if (!m_Protocol.sendCommand("DEWMODE" + std::to_string(mode) + "#"))
    {
        LOGF_ERROR("Could not set the dew heater to %s. %s", DEW_MODE_NAMES[mode], m_Protocol.lastError().c_str());
        return false;
    }
    m_DewMode = mode;

    if (mode == DEW_MANUAL)
    {
        if (!m_Protocol.sendCommand("DEWDUTY" + std::to_string(duty) + "#"))
        {
            LOGF_ERROR("Could not set the dew heater to %d%%. %s", duty, m_Protocol.lastError().c_str());
            return false;
        }
        m_DewDuty = duty;
        LOGF_INFO("Dew heater on at %d%%.", duty);
    }
    else if (mode == DEW_AUTO)
        LOG_INFO("Dew heater set to automatic.");
    else
        LOG_INFO("Dew heater off.");

    showDewHeater();
    return true;
}

// The wheel's dew heater mode as INDI's switches: the heater is on in manual and automatic modes.
// While it is off, the duty cycle stays as last shown, for when it is turned on again.
void StarpointFilterWheel::showDewHeater()
{
    DewChannelsSP[0].setState(m_DewMode != DEW_OFF ? ISS_ON : ISS_OFF);
    AutoDewSP[0].setState(m_DewMode == DEW_AUTO ? ISS_ON : ISS_OFF);
    DewChannelsSP.setState(IPS_OK);
    AutoDewSP.setState(IPS_OK);
    DewChannelDutyCycleNP.setState(IPS_OK);

    // While connecting, show the wheel's duty cycle whatever the mode; updateProperties defines
    // the properties afterwards
    if (!isConnected())
    {
        DewChannelDutyCycleNP[0].setValue(m_DewDuty);
        return;
    }

    if (m_DewMode != DEW_OFF)
        DewChannelDutyCycleNP[0].setValue(m_DewDuty);
    DewChannelsSP.apply();
    AutoDewSP.apply();
    DewChannelDutyCycleNP.apply();
}

std::string StarpointFilterWheel::Simulator::reply(const std::string &command)
{
    update();

    const std::string body = withoutTerminator(command);
    const int slots = static_cast<int>(m_Names.size());
    const bool moving = m_Position != m_Target;

    // In automatic mode the heater runs at 40%, for the conditions below
    const int dewDuty = m_DewMode == DEW_AUTO ? 40 : m_DewDuty;

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
        return FILTER_WHEEL_REPLY;

    if (body == "CONFIG?")
    {
        json filters = json::array();
        for (int i = 0; i < slots; i++)
            filters.push_back({{"position", i}, {"name", m_Names[i]}, {"offset", 0}});

        const json config =
        {
            {"device", "Simulator"}, {"deviceName", "Simulated filter wheel"}, {"fw", "simulator"},
            {"positions", slots}, {"currentPosition", moving ? -1 : m_Position}, {"filters", filters},
            {"capabilities", {{"powerMonitor", true}, {"dewHeater", true}, {"weather", true}, {"homing", false}}},
            {"led", m_LED}
        };
        return config.dump();
    }

    if (body == "POS?")
        return std::to_string(moving ? -1 : m_Position);

    if (body == "POWER?")
    {
        // Output 1 draws 0.9 A and output 2 0.5 A when on, and the heater up to 2 A
        const double volts = 12.3;
        const double outputs[POWER_OUTPUTS] = {m_Outputs[0] ? 0.9 : 0, m_Outputs[1] ? 0.5 : 0};
        const double heater = m_DewMode == DEW_OFF ? 0 : 0.02 * dewDuty;
        const double input = 0.12 + outputs[0] + outputs[1] + heater;

        json power = {{"input", {{"voltage", volts}, {"current", input}, {"powerW", volts * input}}}};
        for (int i = 0; i < POWER_OUTPUTS; i++)
            power["out" + std::to_string(i + 1)] =
            {
                {"enabled", m_Outputs[i]}, {"voltage", m_Outputs[i] ? volts : 0}, {"current", outputs[i]},
                {"powerW", volts * outputs[i]}
            };
        return power.dump();
    }

    if (body == "DEWMODE?")
        return std::to_string(m_DewMode);

    if (body == "DEWDUTY?")
        return std::to_string(dewDuty);

    if (body == "WEATHER?")
        return R"({"available":true,"tempC":9.5,"tempF":49.1,"humidity":81.0,"pressureHpa":1008.2,"dewPointC":6.4,"dewPointF":43.5})";

    if (body.compare(0, 7, "FILNAME") == 0)
    {
        try
        {
            const json payload = json::parse(body.substr(7));
            const int slot = payload.at("i").get<int>();
            const std::string name = payload.at("name").get<std::string>();
            if (slot < 0 || slot >= slots || name.empty())
                return ack(false, "range");
            m_Names[slot] = name;
            return ack(true);
        }
        catch (const json::exception &)
        {
            return ack(false, "range");
        }
    }

    if (valueAfter("MOVE"))
    {
        if (value < 0 || value >= slots)
            return ack(false, "range");
        if (moving)
            return ack(false, "busy");

        // About 0.4 s a slot, the shorter way round, after a moment to start
        m_Target = static_cast<int>(value);
        const int distance = std::abs(m_Target - m_Position);
        const int slotsToTurn = std::min(distance, slots - distance);
        m_MoveEndsAt = std::chrono::steady_clock::now() + std::chrono::milliseconds(300 + 400 * slotsToTurn);
        return ack(true);
    }

    if (valueAfter("LED"))
    {
        if (value < 0 || value > 255)
            return ack(false, "range");
        m_LED = static_cast<int>(value);
        return ack(true);
    }

    for (int i = 0; i < POWER_OUTPUTS; i++)
    {
        if (valueAfter(i == 0 ? "OUTAENABLE" : "OUTBENABLE"))
        {
            if (value != 0 && value != 1)
                return ack(false, "range");
            m_Outputs[i] = value == 1;
            return ack(true);
        }
    }

    if (valueAfter("DEWMODE"))
    {
        if (value != DEW_OFF && value != DEW_MANUAL && value != DEW_AUTO)
            return ack(false, "range");
        m_DewMode = static_cast<int>(value);
        return ack(true);
    }

    if (valueAfter("DEWDUTY"))
    {
        if (value < 0 || value > MAX_DEW_DUTY)
            return ack(false, "range");
        m_DewDuty = static_cast<int>(value);
        return ack(true);
    }

    return ack(false, "unsupported");
}

void StarpointFilterWheel::Simulator::update()
{
    if (m_Position != m_Target && std::chrono::steady_clock::now() >= m_MoveEndsAt)
        m_Position = m_Target;
}

std::string StarpointFilterWheel::Simulator::ack(bool ok, const char *error) const
{
    json reply = {{"ok", ok}};
    if (error != nullptr)
        reply["error"] = error;
    return reply.dump();
}