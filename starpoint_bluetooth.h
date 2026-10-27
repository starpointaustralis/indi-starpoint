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

#pragma once

#include "connectionplugins/connectioninterface.h"
#include "indipropertyswitch.h"
#include "indipropertytext.h"

#include <systemd/sd-bus.h>

#include <chrono>
#include <functional>
#include <string>
#include <vector>

/**
 * A Starpoint Australis device over Bluetooth LE, through BlueZ on the system's D-Bus. Commands
 * are written to the device's transmit characteristic and replies arrive as notifications on its
 * receive characteristic, in the same text protocol as over USB. Pairing uses the driver's own
 * BlueZ agent, which answers only for the device being paired, with the PIN it was given.
 *
 * Everything runs on the caller's thread: waiting for a reply also handles BlueZ's messages, so
 * the agent can answer BlueZ while a pairing request is still in progress.
 */
class StarpointBluetoothLink
{
    public:
        // The device's Bluetooth LE services and characteristics
        struct Profile
        {
            const char *kind;
            const char *transmitService;
            const char *transmitCharacteristic;
            const char *receiveService;
            const char *receiveCharacteristic;
        };

         struct Found
        {
            std::string address;
            std::string name;
            bool paired {false};
        };

        enum class Read
        {
            OK,
            TIMEOUT,
            LOST
        };

        explicit StarpointBluetoothLink(const Profile &profile);
        ~StarpointBluetoothLink();

        // Raw BlueZ errors and replies go to this log only
        void setDebugLog(std::function<void(const std::string &)> log);

        // Each returns false on failure, and lastError() says why
        bool scan(std::chrono::seconds duration, std::vector<Found> &found);
        bool connect(const std::string &address, const std::string &pin);
        void disconnect();
        bool isConnected() const
        {
            return m_Connected;
        }
        bool write(const std::string &data);

        // One byte the device has sent, waiting for it until the deadline
        Read read(char &c, std::chrono::steady_clock::time_point deadline);

        // Everything received and not yet read
        std::string takeReceived();

        const std::string &lastError() const
        {
            return m_LastError;
        }

        // The device being paired and its PIN, for the agent
        struct Pairing
        {
            std::string devicePath;
            std::string pin;
        };

    private:
        struct Objects;

        bool openBus();
        void closeBus();
        sd_bus_message *newCall(const std::string &path, const char *interface, const char *member);
        bool call(sd_bus_message *request, std::chrono::milliseconds timeout, sd_bus_message **reply = nullptr);
        bool pump(std::chrono::microseconds wait);
        bool getObjects(Objects &objects);

         bool findAdapter(std::string &adapter);
        bool startDiscovery(const std::string &adapter);
        void stopDiscovery(const std::string &adapter);
        bool findDevice(const std::string &adapter, bool &paired, bool &connected);
        bool pair(const std::string &pin);
        bool connectDevice();
        bool waitForServices();
        bool findCharacteristics();
        bool subscribe();
        void abandon();
        bool fail(const std::string &message);
        std::string describe() const;
        void debug(const std::string &text) const;

        static int onReply(sd_bus_message *message, void *userdata, sd_bus_error *error);
        static int onPropertiesChanged(sd_bus_message *message, void *userdata, sd_bus_error *error);
        static int onAgentRequest(sd_bus_message *message, void *userdata, sd_bus_error *error);

        Profile m_Profile;
        std::function<void(const std::string &)> m_DebugLog;

        sd_bus *m_Bus {nullptr};
        sd_bus_slot *m_AgentSlot {nullptr};
        sd_bus_slot *m_SignalSlot {nullptr};

        std::string m_Address;
        std::string m_DevicePath;
        std::string m_TransmitPath;
        std::string m_ReceivePath;
        bool m_WriteWithoutResponse {false};
        bool m_Connected {false};
        int m_Drops {0};
        Pairing m_Pairing;
        std::string m_Received;

        std::string m_LastError;
        std::string m_ErrorName;
        std::string m_ErrorMessage;
};

/**
 * Bluetooth as an INDI connection, next to Serial in the Connection tab's mode switch. The
 * device's address and PIN are kept in INDI's configuration; with no address, connecting looks
 * for the device first.
 */
class StarpointBluetoothConnection : public Connection::Interface
{
    public:
        StarpointBluetoothConnection(INDI::DefaultDevice *device, const StarpointBluetoothLink::Profile &profile);

        virtual bool Connect() override;
        virtual bool Disconnect() override;
        virtual void Activated() override;
        virtual void Deactivated() override;
        virtual std::string name() override;
        virtual std::string label() override;

        virtual bool ISNewText(const char *dev, const char *name, char *texts[], char *names[], int n) override;
        virtual bool ISNewSwitch(const char *dev, const char *name, ISState *states, char *names[], int n) override;
        virtual bool saveConfigItems(FILE *fp) override;

        // In simulation, connecting goes straight to the handshake
        void registerSimulation(std::function<bool()> isSimulation);

        StarpointBluetoothLink &link()
        {
            return m_Link;
        }
        std::string address();

     private:
        bool search(bool fillAddress);

        StarpointBluetoothLink m_Link;
        const char *m_Kind;
        std::function<bool()> m_IsSimulation;

        INDI::PropertyText BluetoothTP {2};
        enum
        {
            BLUETOOTH_ADDRESS,
            BLUETOOTH_PIN
        };

        INDI::PropertySwitch SearchSP {1};
};