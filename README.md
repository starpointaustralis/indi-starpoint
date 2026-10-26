# Starpoint Australis Focuser INDI Driver
This repository contains the **INDI driver** for the Starpoint Australis Focuser.
It allows KStars/Ekos and any INDI-compatible automation system to control the focuser over USB.

<img width="4000" height="3000" alt="Starpoint Australis Focuser" src="https://github.com/user-attachments/assets/0b0680c9-8c98-4680-8334-1c64d935eb63" />

## Features
- Absolute and relative moves, abort and sync
- Minimum and maximum positions, stored on the focuser
- Reverse direction and motor speed
- Temperature reading for Ekos, with calibration against a thermometer you trust
- Status light brightness (Off, Low, Medium or High), so it can be turned off while imaging
- The focuser's model, firmware version and name
- Simulation mode, for trying the driver without a focuser

Firmware updates are made with the Starpoint Hub app, which can be downloaded from
[our website](https://www.starpointaustralis.com.au).
See [doc/index.md](doc/index.md) for connecting and using the driver.

## Dependencies
The following packages must be installed before building:
```bash
sudo apt install -y cmake g++ libindi-dev
```
The driver needs INDI 2.0 or later and CMake 3.16 or later. Check the version your system offers with
`apt policy libindi-dev`. If it is older than 2.0, install INDI from the INDI project instead; on Ubuntu,
add its PPA first with `sudo add-apt-repository ppa:mutlaqja/ppa`.

## Building and Installing the Driver
The following steps are for Debian-based Linux, such as Ubuntu and Raspberry Pi OS.
1. Clone the repository
```bash
mkdir -p ~/Projects
cd ~/Projects
git clone https://github.com/starpointaustralis/indi_starpoint_focuser.git
cd indi_starpoint_focuser
```
2. Create and enter the build directory
```bash
mkdir build
cd build
```
3. Configure the build
Install to `/usr` so the driver appears in KStars automatically:
```bash
cmake .. -DCMAKE_INSTALL_PREFIX=/usr
```
4. Build the driver
```bash
make -j4
```
5. Install the driver
```bash
sudo make install
```
This installs:
- The driver executable `/usr/bin/indi_starpoint_focuser`
- The XML definition `/usr/share/indi/indi_starpoint_focuser.xml`

## Trying It Without a Focuser
In KStars, start an Ekos profile that uses **Starpoint Australis Focuser**. In the INDI Control Panel,
turn on **Simulation** on the Options tab, then click **Connect**. If Ekos has already connected the
driver, disconnect it first.

Or from a terminal, start the driver:
```bash
indiserver -v indi_starpoint_focuser
```
and in another terminal:
```bash
indi_setprop "Starpoint Australis Focuser.SIMULATION.ENABLE=On"
indi_setprop "Starpoint Australis Focuser.CONNECTION.CONNECT=On"
indi_getprop "Starpoint Australis Focuser.*.*"
```

## Removing the Driver
```bash
sudo rm -f /usr/bin/indi_starpoint_focuser
sudo rm -f /usr/share/indi/indi_starpoint_focuser.xml
```

## Licence
Copyright (C) 2025-2026 Stellar Focus. Released under the GNU Lesser General Public License,
version 2.1 or (at your option) any later version. See [LICENSE](LICENSE).

## About Starpoint Australis
Starpoint Australis creates precision astrophotography and astronomy equipment engineered in Australia.
Learn more at [our website](https://www.starpointaustralis.com.au)
