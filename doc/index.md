# Starpoint Australis Focuser

## Device Overview

Starpoint Australis creates precision astrophotography and astronomy equipment engineered in Australia.

This driver controls the Starpoint Australis Focuser over USB serial. The focuser moves to absolute
step positions and reads the temperature from a plug-in probe. Its model and firmware version are
shown once connected.

## Features

- Absolute and relative moves, abort and sync
- Maximum position (INDI's standard `FOCUS_MAX`) and minimum position (`FOCUS_MIN`)
- Reverse direction and motor speed (1 slowest to 255 fastest)
- Temperature (`FOCUS_TEMPERATURE`) for Ekos, read every 5 seconds, with optional calibration against
  a thermometer
- Status light brightness: Off, Low, Medium or High
- The focuser's model, firmware version and name, read-only
- Settings kept on the focuser itself (see [Settings Stored on the Focuser](#settings-stored-on-the-focuser))
- Simulation mode

## Installation

The driver needs only INDI's own library. To build it from source, see the
[README](../README.md).

Firmware updates are made with the Starpoint Hub app, which can be downloaded from
[our website](https://www.starpointaustralis.com.au). See [Starpoint Hub](#starpoint-hub-optional).

No udev rules are needed: on most Linux systems, members of the `dialout` group can use the
focuser's port. To add yourself, run `sudo usermod -aG dialout $USER`, then log out and in again.

## Configuration

1. Plug the focuser in by USB. It usually appears as `/dev/ttyACM0`.
2. In the Ekos profile editor, choose **Starpoint Australis Focuser** under Focusers.
3. Start the profile and connect. Set the focuser's port on the **Connection** tab; if the
   focuser is not on that port, the driver searches the other serial ports for one that answers
   as a Starpoint Australis focuser (turn off **Auto Search** there to stop this). The baud rate
   is 115200.

Only one program can use the focuser's port at a time. If the Starpoint Hub app is using the
focuser on the same computer, disconnect it there first.

| Property | Tab | Meaning |
|----------|-----|---------|
| Absolute Position | Main Control | Move to a position in steps |
| Relative Position, Direction | Main Control | Move in or out by a number of steps |
| Abort Motion | Main Control | Stop the focuser |
| Sync | Main Control | Tell the focuser its current position, without moving |
| Max. Position, Min. Position | Main Control | The focuser's travel limits |
| Reverse Motion | Main Control | Move the other way for the same command |
| Speed | Main Control | Motor speed, 1 (slowest) to 255 (fastest) |
| Temperature | Main Control | The probe's reading in °C, including the temperature offset |
| Status Light | Options | Brightness of the focuser's light |
| Temperature Offset | Options | The current correction to the probe's reading, in °C |
| Calibrate Temperature | Options | Enter the actual temperature to set the offset |
| Temperature Offset: Clear offset | Options | Remove the offset |
| Focuser | General Info | Model, firmware version and the name stored on the focuser |

## Settings Stored on the Focuser

The focuser keeps these settings itself, so they come with it to any computer it is connected to:

- Minimum and maximum position
- Direction (normal or reversed)
- Speed
- Status light brightness
- Temperature offset

The driver reads them from the focuser each time it connects, and does not keep them in INDI's
configuration file.

## Usage and Tips

- **Temperature calibration:** read the temperature from a thermometer you trust, enter it in
  **Calibrate Temperature**, and the driver sets the offset to the difference from the probe's
  reading (up to 20 °C either way). **Clear offset** removes it.
- **Status light:** turn it off for imaging if it shows in your frames.
- **No temperature probe:** without a probe plugged in, the temperature is not shown. It
  appears when a reading becomes available, and turns red if the reading stops.
- **Lost contact:** if three polls in a row fail, the position turns red and a message says
  so. Check the USB cable and power. If the focuser does not recover by itself, disconnect and
  connect again.
- **Logging:** the driver logs connecting, moves and any problems, and KStars/Ekos shows these
  in its INDI log. If the focuser refuses a command or behaves unexpectedly, please report it to
  Starpoint Australis with the firmware version (General Info tab) and a debug log.
- **Debug log:** on the **Options** tab, enable **Debug**, then tick **Driver Debug** under
  **Debug Levels** to see it in the client, or under **Logging Levels** with **To Log File** under
  **Log Output** to write it to a file in `~/.indi/logs`. The commands sent to the focuser and
  its replies then appear in the log.

### Trying it without a focuser

Enable **Simulation** on the Options tab before connecting. The simulated focuser has a travel
of 0 to 25,000 steps, starts at 10,000, and has a temperature reading that drifts slowly.

From a terminal, start the driver:

```bash
indiserver -v indi_starpoint_focuser
```

and in another terminal:

```bash
indi_setprop "Starpoint Australis Focuser.SIMULATION.ENABLE=On"
indi_setprop "Starpoint Australis Focuser.CONNECTION.CONNECT=On"
indi_getprop "Starpoint Australis Focuser.*.*"
```

## Starpoint Hub (Optional)

Starpoint Hub is an app from Starpoint Australis, available from
[our website](https://www.starpointaustralis.com.au). This driver does not need it, but it adds:

- Firmware updates for the focuser
- Changing the name stored on the focuser
- An ASCOM Alpaca server, for imaging software that does not use INDI

## Testing

Tested with INDI 2.1.7 on Ubuntu 24.04 (x86_64), in simulation: absolute and relative moves,
abort, sync (and sync refused while moving), changing the limits, refusing invalid limits and
moves outside them, reverse, speed, status light, temperature calibration and clearing, and
checking that saved INDI configuration does not overwrite the focuser's own settings.
