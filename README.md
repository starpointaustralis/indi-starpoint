# Starpoint Australis Focuser INDI Driver
This repository contains the **INDI driver** for the Starpoint Australis Focuser.
It allows KStars/Ekos and any INDI-compatible automation system to control the focuser over USB.

<img width="4000" height="3000" alt="SP3" src="https://github.com/user-attachments/assets/0b0680c9-8c98-4680-8334-1c64d935eb63" />

## Features
- Full INDI-compatible focuser device
- Uses standard `INDI::Focuser` API
- Supports temperature reporting
- Supports move-in / move-out and absolute positioning
- Clean XML device description integrated with Ekos
- Auto-installs into `/usr/bin` and `/usr/share/indi`

## Dependencies
The following packages must be installed before building:
```bash
sudo apt install -y cmake g++ libindi-dev libnova-dev zlib1g-dev libgsl-dev
```
These are available on StellarMate OS by default.

## Building and Installing the Driver
The following steps work on StellarMate OS, Ubuntu and Raspberry Pi OS.
1. Clone the Repository
```bash
mkdir -p ~/Projects
cd ~/Projects
git clone https://github.starpointaustralis/indi_starpoint_focuser.git
cd indi_starpoint_focuser
```
2. Create and enter the build directory
```bash
mkdir build
cd build
```
3. Configure the build
Install to `/usr` so the driver appears in Kstars automatically:
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
sudo ldconfig
```
This installs:
- The driver executable `/usr/bin/indi_starpoint_focuser`
- The XML definition `/usr/share/indi/indi_starpoint_focuser.xml`

## Removing the Driver
If you want to reinstall from scratch:
```bash
sudo rm -f /usr/bin/indi_starpoint_focuser
sudo rm -f /usr/share/indi/indi_starpoint_focuser.xml
sudo ldconfig
```

## About Starpoint Australis
Starpoint Australis creates precision astrophotography and astronomy equipment engineered in Australia.
Learn more at [our website](https://www.starpointaustralis.com.au)

