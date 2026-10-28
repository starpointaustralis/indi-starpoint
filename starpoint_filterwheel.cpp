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
}

static std::unique_ptr<StarpointFilterWheel> starpointFilterWheel(new StarpointFilterWheel());

StarpointFilterWheel::StarpointFilterWheel() : m_Protocol(this, "filter wheel")
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

    // Debug, simulation and configuration controls
    addAuxControls();

    return true;
}

bool StarpointFilterWheel::updateProperties()
{
    INDI::FilterWheel::updateProperties();

    if (isConnected())
    {
        defineProperty(DeviceInfoTP);
        if (m_HasLED)
            defineProperty(LEDSP);
    }
    else
    {
        deleteProperty(DeviceInfoTP);
        if (m_HasLED)
            deleteProperty(LEDSP);
    }

    return true;
}

bool StarpointFilterWheel::ISNewText(const char *dev, const char *name, char *texts[], char *names[], int n)
{
    // INDI::FilterWheel passes a rename that failed on to INDI::DefaultDevice, which would keep
    // the rejected names. Stop once the filter interface has handled it
    if (dev != nullptr && strcmp(dev, getDeviceName()) == 0 && FilterNameTP.isNameMatch(name))
    {
        FilterInterface::processText(dev, name, texts, names, n);
        return true;
    }

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

std::string StarpointFilterWheel::Simulator::reply(const std::string &command)
{
    update();

    const std::string body = withoutTerminator(command);
    const int slots = static_cast<int>(m_Names.size());
    const bool moving = m_Position != m_Target;

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
            {"capabilities", {{"powerMonitor", false}, {"dewHeater", false}, {"weather", false}, {"homing", false}}},
            {"led", m_LED}
        };
        return config.dump();
    }

    if (body == "POS?")
        return std::to_string(moving ? -1 : m_Position);

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