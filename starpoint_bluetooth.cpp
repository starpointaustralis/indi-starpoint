/*
    Starpoint Australis INDI Drivers: Bluetooth LE connection

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

#include "starpoint_bluetooth.h"

#include "defaultdevice.h"
#include "indidriver.h"
#include "indilogger.h"

#include <systemd/sd-bus.h>

#include <algorithm>
#include <cctype>
#include <cerrno>
#include <cstdlib>
#include <cstring>
#include <map>

#include <strings.h>

namespace
{
constexpr const char *BLUEZ = "org.bluez";
constexpr const char *OBJECT_MANAGER = "org.freedesktop.DBus.ObjectManager";
constexpr const char *PROPERTIES = "org.freedesktop.DBus.Properties";
constexpr const char *ADAPTER = "org.bluez.Adapter1";
constexpr const char *DEVICE = "org.bluez.Device1";
constexpr const char *GATT_SERVICE = "org.bluez.GattService1";
constexpr const char *GATT_CHARACTERISTIC = "org.bluez.GattCharacteristic1";
constexpr const char *AGENT_MANAGER = "org.bluez.AgentManager1";
constexpr const char *AGENT = "org.bluez.Agent1";
constexpr const char *NO_REPLY = "org.freedesktop.DBus.Error.NoReply";

// The driver's pairing agent. Passkey entry only: the devices use a fixed PIN and have no display.
constexpr const char *AGENT_PATH = "/com/starpointaustralis/indi/agent";
constexpr const char *AGENT_CAPABILITY = "KeyboardOnly";

constexpr auto CALL_TIMEOUT = std::chrono::seconds(5);
constexpr auto FIND_TIMEOUT = std::chrono::seconds(10);
constexpr auto CONNECT_TIMEOUT = std::chrono::seconds(20);
constexpr auto SERVICES_TIMEOUT = std::chrono::seconds(15);
constexpr auto PAIRING_TIMEOUT = std::chrono::seconds(30);
constexpr auto CLOSE_TIMEOUT = std::chrono::seconds(2);
constexpr auto SEARCH_DURATION = std::chrono::seconds(10);

// Right after pairing, the devices' Bluetooth stack can refuse the notification subscription
// before it has caught up with the link's encryption. The firmware developer asks clients to
// retry once after a few hundred milliseconds. A device that is really not paired refuses again.
constexpr auto SUBSCRIBE_RETRY_DELAY = std::chrono::milliseconds(300);

// One D-Bus property value, of the kinds BlueZ uses
struct Value
{
    std::string text; 
    bool flag {false};
    int64_t number {0};
    std::vector<std::string> list;
    std::string bytes; 
};

using Properties = std::map<std::string, Value>;

// The reply to an asynchronous call
struct PendingReply
{
    bool done {false};
    sd_bus_message *reply {nullptr};
    std::string errorName;
    std::string errorMessage;
};

int readVariant(sd_bus_message *message, Value &value)
{
    char type = 0;
    const char *contents = nullptr;
    int rc = sd_bus_message_peek_type(message, &type, &contents);
    if (rc <= 0 || type != SD_BUS_TYPE_VARIANT || contents == nullptr)
        return rc < 0 ? rc : -EINVAL;

    rc = sd_bus_message_enter_container(message, SD_BUS_TYPE_VARIANT, contents);
    if (rc < 0)
        return rc;

    const std::string signature(contents);
    if (signature == "s" || signature == "o")
    {
        const char *text = nullptr;
        rc = sd_bus_message_read_basic(message, signature[0], &text);
        if (rc > 0 && text != nullptr)
            value.text = text;
    }
    else if (signature == "b")
    {
        int flag = 0;
        rc = sd_bus_message_read_basic(message, 'b', &flag);
        value.flag = flag != 0;
    }
    else if (signature == "n" || signature == "q" || signature == "y")
    {
        uint64_t number = 0;
        rc = sd_bus_message_read_basic(message, signature[0], &number);
        value.number = signature == "n" ? static_cast<int16_t>(number) : signature == "q" ? static_cast<uint16_t>(number) : static_cast<uint8_t>(number);
    }
    else if (signature == "i" || signature == "u")
    {
        uint32_t number = 0;
        rc = sd_bus_message_read_basic(message, signature[0], &number);
        value.number = signature == "i" ? static_cast<int32_t>(number) : number;
    }
    else if (signature == "x" || signature == "t")
    {
        int64_t number = 0;
        rc = sd_bus_message_read_basic(message, signature[0], &number);
        value.number = number;
    }
    else if (signature == "as" || signature == "ao")
    {
        char **list = nullptr;
        rc = sd_bus_message_read_strv(message, &list);
        if (rc >= 0 && list != nullptr)
        {
            for (char **item = list; *item != nullptr; item++)
            {
                value.list.push_back(*item);
                free(*item);
            }
            free(list);
        }
    }
    else if (signature == "ay")
    {
        const void *data = nullptr;
        size_t size = 0;
        rc = sd_bus_message_read_array(message, 'y', &data, &size);
        if (rc >= 0 && data != nullptr)
            value.bytes.assign(static_cast<const char *>(data), size);
    }
    else
        rc = sd_bus_message_skip(message, contents);

    if (rc < 0)
        return rc;
    return sd_bus_message_exit_container(message);
}

int readProperties(sd_bus_message *message, Properties &properties)
{
    int rc = sd_bus_message_enter_container(message, SD_BUS_TYPE_ARRAY, "{sv}");
    while (rc >= 0 && (rc = sd_bus_message_enter_container(message, SD_BUS_TYPE_DICT_ENTRY, "sv")) > 0)
    {
        const char *name = nullptr;
        rc = sd_bus_message_read_basic(message, 's', &name);
        if (rc >= 0)
            rc = readVariant(message, properties[name]);
        if (rc >= 0)
            rc = sd_bus_message_exit_container(message);
    }
    if (rc < 0)
        return rc;
    return sd_bus_message_exit_container(message);
}

const Value *field(const Properties &properties, const char *name)
{
    auto value = properties.find(name);
    return value == properties.end() ? nullptr : &value->second;
}

std::string textOf(const Properties &properties, const char *name)
{
    const Value *value = field(properties, name);
    return value == nullptr ? std::string() : value->text;
}

bool flagOf(const Properties &properties, const char *name)
{
    const Value *value = field(properties, name);
    return value != nullptr && value->flag;
}

bool sameUuid(const std::string &uuid, const char *other)
{
    return strcasecmp(uuid.c_str(), other) == 0;
}

std::string toUpper(std::string text)
{
    std::transform(text.begin(), text.end(), text.begin(), [](unsigned char c)
    {
        return static_cast<char>(std::toupper(c));
    });
    return text;
}

std::string trim(const std::string &text)
{
    const auto first = text.find_first_not_of(" \t");
    if (first == std::string::npos)
        return std::string();
    return text.substr(first, text.find_last_not_of(" \t") - first + 1);
}

// Twelve hexadecimal digits in pairs separated by colons: 12:34:56:78:9A:BC
bool validAddress(const std::string &address)
{
    if (address.size() != 17)
        return false;
    for (size_t i = 0; i < address.size(); i++)
    {
        if (i % 3 == 2 ? address[i] != ':' : !std::isxdigit(static_cast<unsigned char>(address[i])))
            return false;
    }
    return true;
}

bool validPin(const std::string &pin)
{
    return pin.size() == 6 && std::all_of(pin.begin(), pin.end(), [](unsigned char c)
    {
        return std::isdigit(c) != 0;
    });
}

int reject(sd_bus_message *request)
{
    const int rc = sd_bus_reply_method_errorf(request, "org.bluez.Error.Rejected", "This driver is not pairing that device.");
    return rc < 0 ? rc : 1;
}

int replyEmpty(sd_bus_message *request)
{
    const int rc = sd_bus_reply_method_return(request, "");
    return rc < 0 ? rc : 1;
}
}

struct StarpointBluetoothLink::Objects
{
    // Object path, then interface, then properties
    std::map<std::string, std::map<std::string, Properties>> paths;

    const Properties *find(const std::string &path, const char *interface) const
    {
        auto object = paths.find(path);
        if (object == paths.end())
            return nullptr;
        auto properties = object->second.find(interface);
        return properties == object->second.end() ? nullptr : &properties->second;
    }
};

StarpointBluetoothLink::StarpointBluetoothLink(const Profile &profile) : m_Profile(profile)
{
}

StarpointBluetoothLink::~StarpointBluetoothLink()
{
    disconnect();
    closeBus();
}

void StarpointBluetoothLink::setDebugLog(std::function<void(const std::string &)> log)
{
    m_DebugLog = std::move(log);
}

bool StarpointBluetoothLink::scan(std::chrono::seconds duration, std::vector<Found> &found)
{
    found.clear();
    m_LastError.clear();

    std::string adapter;
    if (!openBus() || !findAdapter(adapter) || !startDiscovery(adapter))
        return false;

    const auto until = std::chrono::steady_clock::now() + duration;
    for (auto now = std::chrono::steady_clock::now(); now < until; now = std::chrono::steady_clock::now())
        pump(std::min<std::chrono::microseconds>(std::chrono::duration_cast<std::chrono::microseconds>(until - now),
                std::chrono::milliseconds(200)));
    stopDiscovery(adapter);

    // Devices found now, and ones BlueZ already knows (paired ones may not advertise)
    Objects objects;
    if (!getObjects(objects))
        return false;

    const std::string prefix = adapter + "/dev_";
    for (const auto &[path, interfaces] : objects.paths)
    {
        if (path.compare(0, prefix.size(), prefix) != 0 || path.find('/', prefix.size()) != std::string::npos)
            continue;

        auto device = interfaces.find(DEVICE);
        if (device == interfaces.end())
            continue;

        const Value *uuids = field(device->second, "UUIDs");
        auto isOurs = [this](const std::string & uuid)
        {
            return sameUuid(uuid, m_Profile.receiveService);
        };
        if (uuids == nullptr || std::none_of(uuids->list.begin(), uuids->list.end(), isOurs))
            continue;

        Found one;
        one.address = textOf(device->second, "Address");
        one.name = textOf(device->second, "Name");
        if (one.name.empty())
            one.name = textOf(device->second, "Alias");
        one.paired = flagOf(device->second, "Paired");
        found.push_back(one);
    }

    // Devices already paired with this computer first
    std::stable_partition(found.begin(), found.end(), [](const Found & one)
    {
        return one.paired;
    });
    return true;
}

bool StarpointBluetoothLink::connect(const std::string &address, const std::string &pin)
{
    disconnect();
    m_LastError.clear();
    m_Address = address;

    std::string adapter;
    if (!openBus() || !findAdapter(adapter))
        return false;

    m_DevicePath = adapter + "/dev_" + address;
    std::replace(m_DevicePath.begin() + static_cast<std::ptrdiff_t>(adapter.size()), m_DevicePath.end(), ':', '_');
    debug("Connecting to " + address + " over Bluetooth (" + adapter.substr(adapter.rfind('/') + 1) + ")");

    // Watch the device for replies, and for the link dropping
    m_Drops = 0;
    const std::string match = "type='signal',sender='org.bluez',interface='org.freedesktop.DBus.Properties',"
                              "member='PropertiesChanged',path_namespace='" + m_DevicePath + "'";
    const int rc = sd_bus_add_match(m_Bus, &m_SignalSlot, match.c_str(), &StarpointBluetoothLink::onPropertiesChanged, this);
    if (rc < 0)
    {
        debug(std::string("AddMatch: ") + strerror(-rc));
        return fail(std::string("Could not watch the ") + m_Profile.kind + " over Bluetooth.");
    }

    bool paired = false, connected = false;
    const bool ok = findDevice(adapter, paired, connected)
                    && (paired || pair(pin))
                    && (connected || connectDevice())
                    && waitForServices()
                    && findCharacteristics()
                    && subscribe();
    if (!ok)
    {
        // A paired device that lets the link come up and then drops it most often no longer
        // accepts this computer's pairing. BlueZ does not say why, so this is inferred.
        if (paired && m_Drops > 0 && m_ErrorName == NO_REPLY)
            m_LastError = std::string("The ") + m_Profile.kind + " accepted the Bluetooth connection, then dropped it. "
                          "It may no longer accept this computer's pairing. Remove the pairing (bluetoothctl remove " + address +
                          ") and connect again to pair afresh.";
        abandon();
        return false;
    }

    m_Received.clear();
    m_Connected = true;
    debug("Bluetooth connection to " + address + " is ready");
    return true;
}

void StarpointBluetoothLink::disconnect()
{
    if (m_Bus != nullptr && !m_DevicePath.empty())
    {
        if (m_Connected && !m_ReceivePath.empty())
            call(newCall(m_ReceivePath, GATT_CHARACTERISTIC, "StopNotify"), CLOSE_TIMEOUT);
        call(newCall(m_DevicePath, DEVICE, "Disconnect"), CLOSE_TIMEOUT);
    }

    m_SignalSlot = sd_bus_slot_unref(m_SignalSlot);
    m_Connected = false;
    m_DevicePath.clear();
    m_TransmitPath.clear();
    m_ReceivePath.clear();
    m_Received.clear();
}

bool StarpointBluetoothLink::write(const std::string &data)
{
    if (!m_Connected)
        return fail(std::string("Lost contact with the ") + m_Profile.kind + " over Bluetooth.");

    sd_bus_message *request = newCall(m_TransmitPath, GATT_CHARACTERISTIC, "WriteValue");
    if (request != nullptr
            && (sd_bus_message_append_array(request, 'y', data.data(), data.size()) < 0
                || sd_bus_message_append(request, "a{sv}", 1, "type", "s", m_WriteWithoutResponse ? "command" : "request") < 0))
        request = sd_bus_message_unref(request);

    if (!call(request, CALL_TIMEOUT))
        return fail(std::string("Could not send to the ") + m_Profile.kind + " over Bluetooth.");
    return true;
}

StarpointBluetoothLink::Read StarpointBluetoothLink::read(char &c, std::chrono::steady_clock::time_point deadline)
{
    while (m_Received.empty())
    {
        if (!m_Connected)
            return Read::LOST;

        const auto now = std::chrono::steady_clock::now();
        if (now >= deadline)
            return Read::TIMEOUT;

        if (!pump(std::min<std::chrono::microseconds>(std::chrono::duration_cast<std::chrono::microseconds>(deadline - now),
                  std::chrono::milliseconds(100))))
            return Read::LOST;
    }

    c = m_Received.front();
    m_Received.erase(0, 1);
    return Read::OK;
}

std::string StarpointBluetoothLink::takeReceived()
{
    if (m_Bus != nullptr)
        pump(std::chrono::microseconds(0));

    std::string received;
    received.swap(m_Received);
    return received;
}

bool StarpointBluetoothLink::openBus()
{
    if (m_Bus != nullptr)
        return true;

    int rc = sd_bus_open_system(&m_Bus);
    if (rc < 0)
    {
        m_Bus = nullptr;
        debug(std::string("sd_bus_open_system: ") + strerror(-rc));
        return fail("Could not reach the system's message bus (D-Bus), so Bluetooth cannot be used.");
    }

    // The agent BlueZ asks for the PIN while this driver pairs. It is not the default agent, so
    // desktop pairing dialogs are unaffected.
    rc = sd_bus_add_object(m_Bus, &m_AgentSlot, AGENT_PATH, &StarpointBluetoothLink::onAgentRequest, &m_Pairing);
    if (rc < 0)
    {
        debug(std::string("sd_bus_add_object: ") + strerror(-rc));
        closeBus();
        return fail("Could not set up Bluetooth pairing.");
    }

    sd_bus_message *request = newCall("/org/bluez", AGENT_MANAGER, "RegisterAgent");
    if (request != nullptr && sd_bus_message_append(request, "os", AGENT_PATH, AGENT_CAPABILITY) < 0)
        request = sd_bus_message_unref(request);

    if (!call(request, CALL_TIMEOUT) && m_ErrorName != "org.bluez.Error.AlreadyExists")
    {
        const std::string reason = describe();
        closeBus();
        return fail(reason);
    }

    return true;
}

void StarpointBluetoothLink::closeBus()
{
    // BlueZ forgets the agent when its connection to the bus closes
    m_SignalSlot = sd_bus_slot_unref(m_SignalSlot);
    m_AgentSlot = sd_bus_slot_unref(m_AgentSlot);
    if (m_Bus != nullptr)
        m_Bus = sd_bus_flush_close_unref(m_Bus);
}

sd_bus_message *StarpointBluetoothLink::newCall(const std::string &path, const char *interface, const char *member)
{
    sd_bus_message *request = nullptr;
    const int rc = sd_bus_message_new_method_call(m_Bus, &request, BLUEZ, path.c_str(), interface, member);
    if (rc < 0)
    {
        debug(std::string(member) + ": " + strerror(-rc));
        return nullptr;
    }
    return request;
}

bool StarpointBluetoothLink::call(sd_bus_message *request, std::chrono::milliseconds timeout, sd_bus_message **reply)
{
    m_ErrorName.clear();
    m_ErrorMessage.clear();
    if (request == nullptr)
    {
        m_ErrorName = "org.freedesktop.DBus.Error.Failed";
        m_ErrorMessage = "could not build the request";
        return false;
    }

    const char *member = sd_bus_message_get_member(request);
    const std::string what = member != nullptr ? member : "call";

    // Asynchronous, so BlueZ's own requests (to the agent, while pairing) and notifications are
    // handled while waiting. sd-bus answers a call that times out with a NoReply error.
    PendingReply pending;
    sd_bus_slot *slot = nullptr;
    const int rc = sd_bus_call_async(m_Bus, &slot, request, &StarpointBluetoothLink::onReply, &pending,
                                     static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::microseconds>(timeout).count()));
    sd_bus_message_unref(request);
    if (rc < 0)
    {
        m_ErrorName = "org.freedesktop.DBus.Error.Failed";
        m_ErrorMessage = strerror(-rc);
        debug(what + ": " + m_ErrorMessage);
        return false;
    }

    const auto deadline = std::chrono::steady_clock::now() + timeout + std::chrono::seconds(2);
    while (!pending.done && std::chrono::steady_clock::now() < deadline)
    {
        if (!pump(std::chrono::milliseconds(100)))
            break;
    }
    sd_bus_slot_unref(slot);

    if (!pending.done)
    {
        m_ErrorName = NO_REPLY;
        m_ErrorMessage = "no reply";
        debug(what + ": no reply");
        return false;
    }

    if (!pending.errorName.empty())
    {
        m_ErrorName = pending.errorName;
        m_ErrorMessage = pending.errorMessage;
        debug(what + ": " + m_ErrorName + " (" + m_ErrorMessage + ")");
        return false;
    }

    if (reply != nullptr)
        *reply = pending.reply;
    else
        sd_bus_message_unref(pending.reply);
    return true;
}

bool StarpointBluetoothLink::pump(std::chrono::microseconds wait)
{
    // Handle what has arrived, wait up to `wait` for more, then handle that too
    auto processAll = [this]()
    {
        int rc = 0;
        while ((rc = sd_bus_process(m_Bus, nullptr)) > 0)
            ;
        return rc;
    };

    int rc = processAll();
    if (rc >= 0 && wait.count() > 0)
    {
        rc = sd_bus_wait(m_Bus, static_cast<uint64_t>(wait.count()));
        if (rc >= 0 || rc == -EINTR)
            rc = processAll();
    }

    if (rc < 0)
    {
        debug(std::string("D-Bus: ") + strerror(-rc));
        m_Connected = false;
        return false;
    }
    return true;
}

bool StarpointBluetoothLink::getObjects(Objects &objects)
{
    objects.paths.clear();

    sd_bus_message *request = nullptr;
    if (sd_bus_message_new_method_call(m_Bus, &request, BLUEZ, "/", OBJECT_MANAGER, "GetManagedObjects") < 0)
        request = nullptr;

    sd_bus_message *reply = nullptr;
    if (!call(request, CALL_TIMEOUT, &reply))
        return fail(describe());

    // a{oa{sa{sv}}}: object path, then interface, then properties
    int rc = sd_bus_message_enter_container(reply, SD_BUS_TYPE_ARRAY, "{oa{sa{sv}}}");
    while (rc >= 0 && (rc = sd_bus_message_enter_container(reply, SD_BUS_TYPE_DICT_ENTRY, "oa{sa{sv}}")) > 0)
    {
        const char *path = nullptr;
        rc = sd_bus_message_read_basic(reply, 'o', &path);
        if (rc >= 0)
            rc = sd_bus_message_enter_container(reply, SD_BUS_TYPE_ARRAY, "{sa{sv}}");
        while (rc >= 0 && (rc = sd_bus_message_enter_container(reply, SD_BUS_TYPE_DICT_ENTRY, "sa{sv}")) > 0)
        {
            const char *interface = nullptr;
            rc = sd_bus_message_read_basic(reply, 's', &interface);
            if (rc >= 0)
                rc = readProperties(reply, objects.paths[path][interface]);
            if (rc >= 0)
                rc = sd_bus_message_exit_container(reply);
        }
        if (rc >= 0)
            rc = sd_bus_message_exit_container(reply);
        if (rc >= 0)
            rc = sd_bus_message_exit_container(reply);
    }
    sd_bus_message_unref(reply);

    if (rc < 0)
    {
        debug(std::string("GetManagedObjects: ") + strerror(-rc));
        return fail("Could not read Bluetooth's list of devices.");
    }
    return true;
}

bool StarpointBluetoothLink::findAdapter(std::string &adapter)
{
    Objects objects;
    if (!getObjects(objects))
        return false;

    // The first adapter that is switched on. Paths sort hci0 before hci1.
    bool any = false;
    for (const auto &[path, interfaces] : objects.paths)
    {
        auto properties = interfaces.find(ADAPTER);
        if (properties == interfaces.end())
            continue;

        any = true;
        if (flagOf(properties->second, "Powered"))
        {
            adapter = path;
            return true;
        }
    }

    return fail(any ? "Bluetooth is turned off. Turn it on with: bluetoothctl power on" : "This computer has no Bluetooth adapter.");
}

bool StarpointBluetoothLink::startDiscovery(const std::string &adapter)
{
    // Only Bluetooth LE devices that offer this kind of device's service. Without the filter,
    // discovery still works, so its failure does not matter.
    sd_bus_message *request = newCall(adapter, ADAPTER, "SetDiscoveryFilter");
    if (request != nullptr
            && sd_bus_message_append(request, "a{sv}", 2, "Transport", "s", "le", "UUIDs", "as", 1, m_Profile.receiveService) < 0)
        request = sd_bus_message_unref(request);
    call(request, CALL_TIMEOUT);

    if (!call(newCall(adapter, ADAPTER, "StartDiscovery"), CALL_TIMEOUT) && m_ErrorName != "org.bluez.Error.InProgress")
        return fail(describe());
    return true;
}

void StarpointBluetoothLink::stopDiscovery(const std::string &adapter)
{
    call(newCall(adapter, ADAPTER, "StopDiscovery"), CALL_TIMEOUT);
}

bool StarpointBluetoothLink::findDevice(const std::string &adapter, bool &paired, bool &connected)
{
    Objects objects;
    if (!getObjects(objects))
        return false;

    // BlueZ only knows devices it has seen, so after a restart the device may need looking for
    const Properties *device = objects.find(m_DevicePath, DEVICE);
    if (device == nullptr)
    {
        if (!startDiscovery(adapter))
            return false;

        const auto deadline = std::chrono::steady_clock::now() + FIND_TIMEOUT;
        while (device == nullptr && std::chrono::steady_clock::now() < deadline)
        {
            pump(std::chrono::milliseconds(500));
            if (!getObjects(objects))
            {
                stopDiscovery(adapter);
                return false;
            }
            device = objects.find(m_DevicePath, DEVICE);
        }
        stopDiscovery(adapter);

        if (device == nullptr)
            return fail("No Bluetooth device " + m_Address + " found. Check the " + m_Profile.kind + " is switched on and in range.");
    }

    paired = flagOf(*device, "Paired");
    connected = flagOf(*device, "Connected");
    return true;
}

bool StarpointBluetoothLink::pair(const std::string &pin)
{
    debug("Pairing with " + m_Address);
    m_Pairing.devicePath = m_DevicePath;
    m_Pairing.pin = pin;
    const bool paired = call(newCall(m_DevicePath, DEVICE, "Pair"), PAIRING_TIMEOUT);
    m_Pairing = Pairing();

    if (!paired && m_ErrorName != "org.bluez.Error.AlreadyExists")
    {
        if (m_ErrorName == "org.bluez.Error.AuthenticationFailed" || m_ErrorName == "org.bluez.Error.AuthenticationRejected"
                || m_ErrorName == "org.bluez.Error.AuthenticationCanceled")
            return fail(std::string("The ") + m_Profile.kind +
                        " refused the Bluetooth PIN. Enter its PIN under Bluetooth on the Connection tab, and connect again.");
        if (m_ErrorName == NO_REPLY)
            return fail(std::string("Pairing with the ") + m_Profile.kind + " timed out.");
        return fail(describe());
    }

    // A trusted device reconnects without pairing again
    sd_bus_message *request = newCall(m_DevicePath, PROPERTIES, "Set");
    if (request != nullptr && sd_bus_message_append(request, "ssv", DEVICE, "Trusted", "b", 1) < 0)
        request = sd_bus_message_unref(request);
    call(request, CALL_TIMEOUT);
    return true;
}

bool StarpointBluetoothLink::connectDevice()
{
    if (call(newCall(m_DevicePath, DEVICE, "Connect"), CONNECT_TIMEOUT) || m_ErrorName == "org.bluez.Error.AlreadyConnected")
        return true;
    return fail(describe());
}

bool StarpointBluetoothLink::waitForServices()
{
    const auto deadline = std::chrono::steady_clock::now() + SERVICES_TIMEOUT;
    for (;;)
    {
        Objects objects;
        if (!getObjects(objects))
            return false;

        const Properties *device = objects.find(m_DevicePath, DEVICE);
        if (device != nullptr && flagOf(*device, "ServicesResolved"))
            return true;

        if (std::chrono::steady_clock::now() >= deadline)
        {
            m_ErrorName = NO_REPLY;
            return fail(describe());
        }
        pump(std::chrono::milliseconds(200));
    }
}

bool StarpointBluetoothLink::findCharacteristics()
{
    Objects objects;
    if (!getObjects(objects))
        return false;

    const std::string devicePrefix = m_DevicePath + "/";
    auto servicePath = [&](const char *uuid)
    {
        for (const auto &[path, interfaces] : objects.paths)
        {
            auto service = interfaces.find(GATT_SERVICE);
            if (path.compare(0, devicePrefix.size(), devicePrefix) == 0 && service != interfaces.end()
                    && sameUuid(textOf(service->second, "UUID"), uuid))
                return path;
        }
        return std::string();
    };
    auto characteristic = [&](const std::string & service, const char *uuid, const Properties **properties)
    {
        for (const auto &[path, interfaces] : objects.paths)
        {
            auto one = interfaces.find(GATT_CHARACTERISTIC);
            if (!service.empty() && one != interfaces.end() && textOf(one->second, "Service") == service
                    && sameUuid(textOf(one->second, "UUID"), uuid))
            {
                *properties = &one->second;
                return path;
            }
        }
        return std::string();
    };

    const std::string transmitService = servicePath(m_Profile.transmitService);
    const std::string receiveService = servicePath(m_Profile.receiveService);
    if (transmitService.empty() || receiveService.empty())
        return fail(m_Address + " is not a Starpoint Australis " + m_Profile.kind + ".");

    const Properties *transmit = nullptr;
    const Properties *receive = nullptr;
    m_TransmitPath = characteristic(transmitService, m_Profile.transmitCharacteristic, &transmit);
    m_ReceivePath = characteristic(receiveService, m_Profile.receiveCharacteristic, &receive);
    if (m_TransmitPath.empty() || m_ReceivePath.empty())
        return fail(std::string("The ") + m_Profile.kind + " does not offer its commands over Bluetooth.");

    const Value *flags = field(*transmit, "Flags");
    m_WriteWithoutResponse = flags != nullptr
                             && std::find(flags->list.begin(), flags->list.end(), "write-without-response") != flags->list.end();
    return true;
}

bool StarpointBluetoothLink::subscribe()
{
    if (call(newCall(m_ReceivePath, GATT_CHARACTERISTIC, "StartNotify"), CALL_TIMEOUT))
        return true;

    if (m_ErrorName == "org.bluez.Error.NotPermitted")
    {
        debug("Replies refused right after connecting (" + m_ErrorMessage + "); trying once more");
        const auto until = std::chrono::steady_clock::now() + SUBSCRIBE_RETRY_DELAY;
        for (auto now = std::chrono::steady_clock::now(); now < until; now = std::chrono::steady_clock::now())
            pump(std::chrono::duration_cast<std::chrono::microseconds>(until - now));

        if (call(newCall(m_ReceivePath, GATT_CHARACTERISTIC, "StartNotify"), CALL_TIMEOUT))
            return true;
    }

    return fail(std::string("The ") + m_Profile.kind + " would not send replies over Bluetooth.");
}

void StarpointBluetoothLink::abandon()
{
    // Giving up on BlueZ's Connect does not stop it: it keeps trying and refuses other connections
    // meanwhile ("InProgress"). Disconnect cancels the attempt and drops a half-set-up link.
    if (m_Bus != nullptr && !m_DevicePath.empty())
    {
        const std::string name = m_ErrorName, message = m_ErrorMessage;
        call(newCall(m_DevicePath, DEVICE, "Disconnect"), CLOSE_TIMEOUT);
        m_ErrorName = name;
        m_ErrorMessage = message;
    }

    m_SignalSlot = sd_bus_slot_unref(m_SignalSlot);
    m_Connected = false;
}

bool StarpointBluetoothLink::fail(const std::string &message)
{
    m_LastError = message;
    return false;
}

std::string StarpointBluetoothLink::describe() const
{
    if (m_ErrorName == "org.freedesktop.DBus.Error.AccessDenied")
        return "Not allowed to use Bluetooth. Add your user to the 'bluetooth' group, then log out and in.";
    if (m_ErrorName == "org.freedesktop.DBus.Error.ServiceUnknown")
        return "The Bluetooth service (BlueZ) is not running. Start it with: sudo systemctl start bluetooth";
    if (m_ErrorName == "org.bluez.Error.NotReady")
        return "Bluetooth is turned off. Turn it on with: bluetoothctl power on";
    if (m_ErrorName == NO_REPLY || m_ErrorName == "org.freedesktop.DBus.Error.Timeout")
        return std::string("The ") + m_Profile.kind + " did not respond over Bluetooth. Check it is switched on and in range.";
    return "Bluetooth error: " + (m_ErrorMessage.empty() ? m_ErrorName : m_ErrorMessage);
}

void StarpointBluetoothLink::debug(const std::string &text) const
{
    if (m_DebugLog)
        m_DebugLog(text);
}

int StarpointBluetoothLink::onReply(sd_bus_message *message, void *userdata, sd_bus_error *)
{
    auto *pending = static_cast<PendingReply *>(userdata);
    pending->done = true;

    const sd_bus_error *error = sd_bus_message_get_error(message);
    if (error != nullptr && sd_bus_error_is_set(error))
    {
        pending->errorName = error->name != nullptr ? error->name : "";
        pending->errorMessage = error->message != nullptr ? error->message : "";
    }
    else
        pending->reply = sd_bus_message_ref(message);
    return 1;
}

int StarpointBluetoothLink::onPropertiesChanged(sd_bus_message *message, void *userdata, sd_bus_error *)
{
    auto *link = static_cast<StarpointBluetoothLink *>(userdata);
    const char *path = sd_bus_message_get_path(message);
    const char *interface = nullptr;
    Properties changed;
    if (path == nullptr || sd_bus_message_read_basic(message, 's', &interface) < 0 || interface == nullptr
            || readProperties(message, changed) < 0)
        return 0;

    if (link->m_ReceivePath == path && strcmp(interface, GATT_CHARACTERISTIC) == 0)
    {
        const Value *value = field(changed, "Value");
        if (value != nullptr)
            link->m_Received += value->bytes;
    }
    else if (link->m_DevicePath == path && strcmp(interface, DEVICE) == 0)
    {
        const Value *connected = field(changed, "Connected");
        if (connected != nullptr && !connected->flag)
        {
            link->m_Drops++;
            if (link->m_Connected)
            {
                link->m_Connected = false;
                link->debug(link->m_Address + " disconnected over Bluetooth");
            }
        }
    }
    return 0;
}

int StarpointBluetoothLink::onAgentRequest(sd_bus_message *message, void *userdata, sd_bus_error *)
{
    if (sd_bus_message_is_method_call(message, AGENT, nullptr) <= 0)
        return 0;

    auto *pairing = static_cast<Pairing *>(userdata);
    const char *member = sd_bus_message_get_member(message);
    const std::string request = member != nullptr ? member : "";

    // Requests about a device answer only for the one this driver is pairing
    const char *device = nullptr;
    if (request == "RequestPinCode" || request == "RequestPasskey" || request == "RequestConfirmation"
            || request == "RequestAuthorization" || request == "AuthorizeService")
        sd_bus_message_read_basic(message, 'o', &device);
    const bool ours = device != nullptr && !pairing->devicePath.empty() && pairing->devicePath == device;

    if (request == "RequestPinCode")
    {
        if (!ours)
            return reject(message);
        const int rc = sd_bus_reply_method_return(message, "s", pairing->pin.c_str());
        return rc < 0 ? rc : 1;
    }

    if (request == "RequestPasskey")
    {
        if (!ours)
            return reject(message);
        // Bluetooth LE passkeys are six-digit numbers: "000000" is passkey 0
        const uint32_t passkey = static_cast<uint32_t>(strtoul(pairing->pin.c_str(), nullptr, 10));
        const int rc = sd_bus_reply_method_return(message, "u", passkey);
        return rc < 0 ? rc : 1;
    }

    if (request == "RequestConfirmation" || request == "RequestAuthorization" || request == "AuthorizeService")
        return ours ? replyEmpty(message) : reject(message);

    // DisplayPasskey, DisplayPinCode, Release, Cancel
    return replyEmpty(message);
}

StarpointBluetoothConnection::StarpointBluetoothConnection(INDI::DefaultDevice *device,
        const StarpointBluetoothLink::Profile &profile)
    : Connection::Interface(device, CONNECTION_CUSTOM), m_Link(profile), m_Kind(profile.kind)
{
    m_Link.setDebugLog([this](const std::string & text)
    {
        LOGF_DEBUG("%s", text.c_str());
    });

    // The address and PIN saved in the configuration, if any
    char address[MAXINDINAME] = {0};
    char pin[MAXINDINAME] = "000000";
    IUGetConfigText(device->getDeviceName(), "BLUETOOTH_DEVICE", "ADDRESS", address, MAXINDINAME);
    IUGetConfigText(device->getDeviceName(), "BLUETOOTH_DEVICE", "PIN", pin, MAXINDINAME);

    BluetoothTP[BLUETOOTH_ADDRESS].fill("ADDRESS", "Address", address);
    BluetoothTP[BLUETOOTH_PIN].fill("PIN", "PIN", pin);
    BluetoothTP.fill(getDeviceName(), "BLUETOOTH_DEVICE", "Bluetooth", CONNECTION_TAB, IP_RW, 60, IPS_IDLE);

    SearchSP[0].fill("SEARCH", "Search", ISS_OFF);
    SearchSP.fill(getDeviceName(), "BLUETOOTH_SEARCH", "Bluetooth Search", CONNECTION_TAB, IP_RW, ISR_ATMOST1, 60, IPS_IDLE);
}

void StarpointBluetoothConnection::registerSimulation(std::function<bool()> isSimulation)
{
    m_IsSimulation = std::move(isSimulation);
}

bool StarpointBluetoothConnection::Connect()
{
    if (m_IsSimulation && m_IsSimulation())
        return Handshake();

    // With no address, look for the device first
    if (address().empty() && !search(true))
        return false;

    const std::string pin = BluetoothTP[BLUETOOTH_PIN].getText();
    if (!validPin(pin))
    {
        LOG_ERROR("The Bluetooth PIN must be six digits.");
        return false;
    }

    LOGF_INFO("Connecting to %s over Bluetooth...", address().c_str());
    if (!m_Link.connect(address(), pin))
    {
        LOGF_ERROR("%s", m_Link.lastError().c_str());
        return false;
    }

    if (!Handshake())
    {
        m_Link.disconnect();
        return false;
    }
    return true;
}

bool StarpointBluetoothConnection::Disconnect()
{
    m_Link.disconnect();
    return true;
}

void StarpointBluetoothConnection::Activated()
{
    m_Device->defineProperty(BluetoothTP);
    m_Device->defineProperty(SearchSP);
}

void StarpointBluetoothConnection::Deactivated()
{
    m_Device->deleteProperty(BluetoothTP);
    m_Device->deleteProperty(SearchSP);
}

std::string StarpointBluetoothConnection::name()
{
    return "CONNECTION_BLUETOOTH";
}

std::string StarpointBluetoothConnection::label()
{
    return "Bluetooth";
}

bool StarpointBluetoothConnection::ISNewText(const char *dev, const char *name, char *texts[], char *names[], int n)
{
    if (dev == nullptr || strcmp(dev, getDeviceName()) != 0 || !BluetoothTP.isNameMatch(name))
        return false;

    BluetoothTP.update(texts, names, n);
    BluetoothTP[BLUETOOTH_ADDRESS].setText(address());
    if (!address().empty() && !validAddress(address()))
    {
        LOG_ERROR("The Bluetooth address should look like 12:34:56:78:9A:BC. Leave it empty to search for the device.");
        BluetoothTP.setState(IPS_ALERT);
    }
    else if (!validPin(BluetoothTP[BLUETOOTH_PIN].getText()))
    {
        LOG_ERROR("The Bluetooth PIN must be six digits.");
        BluetoothTP.setState(IPS_ALERT);
    }
    else
        BluetoothTP.setState(IPS_OK);

    BluetoothTP.apply();
    return true;
}

bool StarpointBluetoothConnection::ISNewSwitch(const char *dev, const char *name, ISState *states, char *names[], int n)
{
    INDI_UNUSED(states);
    INDI_UNUSED(names);
    INDI_UNUSED(n);

    if (dev == nullptr || strcmp(dev, getDeviceName()) != 0 || !SearchSP.isNameMatch(name))
        return false;

    SearchSP.reset();
    SearchSP.setState(IPS_BUSY);
    SearchSP.apply();
    SearchSP.setState(search(false) ? IPS_OK : IPS_ALERT);
    SearchSP.apply();
    return true;
}

bool StarpointBluetoothConnection::saveConfigItems(FILE *fp)
{
    BluetoothTP.save(fp);
    return true;
}

std::string StarpointBluetoothConnection::address()
{
    const char *text = BluetoothTP[BLUETOOTH_ADDRESS].getText();
    return toUpper(trim(text != nullptr ? text : ""));
}

bool StarpointBluetoothConnection::search(bool fillAddress)
{
    LOGF_INFO("Searching for Starpoint Australis %ss over Bluetooth...", m_Kind);

    std::vector<StarpointBluetoothLink::Found> found;
    if (!m_Link.scan(SEARCH_DURATION, found))
    {
        LOGF_ERROR("%s", m_Link.lastError().c_str());
        return false;
    }

    if (found.empty())
    {
        LOGF_ERROR("No Starpoint Australis %s found over Bluetooth. Check it is switched on and in range.", m_Kind);
        return false;
    }

    for (const auto &one : found)
        LOGF_INFO("Found %s at %s%s.", one.name.empty() ? m_Kind : one.name.c_str(), one.address.c_str(),
                  one.paired ? " (paired with this computer)" : "");

    // Use the first one found (paired ones come first) unless a valid address is already set
    if (fillAddress || !validAddress(address()))
    {
        BluetoothTP[BLUETOOTH_ADDRESS].setText(found.front().address);
        BluetoothTP.setState(IPS_OK);
        BluetoothTP.apply();
        if (found.size() > 1)
            LOGF_INFO("Using %s. To use another, enter its address under Bluetooth.", found.front().address.c_str());
    }
    return true;
}