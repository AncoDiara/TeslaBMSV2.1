# Windows setup – always flash the latest version

Setting up a Windows laptop (for example the one next to the battery pack) so that you can
fetch the latest version of TeslaBMS_Victron from GitHub with one click and flash it to the Teensy.

The repository is public: you do **not** need a GitHub account for this.

---

## 1. Arduino IDE with Teensy support (one-time)

1. Download **Arduino IDE 2** from <https://www.arduino.cc/en/software> (Windows installer) and install it.
2. Open the IDE: **File → Preferences → Additional boards manager URLs** and paste:
   ```
   https://www.pjrc.com/teensy/package_teensy_index.json
   ```
3. **Tools → Board → Boards Manager**, search for **Teensy** and click Install.
   This already includes the FlexCAN and ADC libraries. No other libraries are needed
   (the low-pass filter is built into the sketch).
4. **Tools → Board → Teensy → Teensy 3.2 / 3.1**.
   On Windows 10/11 no separate driver is needed.
5. Close the IDE (so that the folder `Documents\Arduino` exists).

## 2. GitHub Desktop and fetching the repository (one-time)

1. Download **GitHub Desktop** from <https://desktop.github.com> and install it.
   You can skip signing in ("Skip this step").
2. **File → Clone repository →** tab **URL**:
   - Repository URL: `https://github.com/AncoDiara/TeslaBMSV2.1`
   - Local path: `C:\Users\<your-name>\Documents\Arduino`
3. Click **Clone**. The folder `Documents\Arduino\TeslaBMSV2.1` now exists.
4. In the Arduino IDE: **File → Sketchbook → TeslaBMSV2.1 → TeslaBMS_Victron**.

## 3. Every time: fetch the latest version and flash

1. Open **GitHub Desktop**. It checks for new changes by itself; if there are any,
   a blue **Pull origin** button appears at the top (with the number of new changes).
2. Click **Pull origin**.
3. Open the sketch in the Arduino IDE (if it was already open: close it and open it again,
   so the IDE loads the new files).
4. Connect the Teensy via USB and click **Upload**.

Under **History** in GitHub Desktop you can see per change what has been modified.

## Good to know

- **Do not change anything on this laptop.** Treat it as "fetch and flash only".
  Did you accidentally change a file? Then *Pull* may get stuck: in GitHub Desktop,
  right-click the changed files → **Discard changes**, then *Pull* again.
- **Settings stay in the Teensy.** Scells, Pstrings, balancing limits etc. are stored in the
  Teensy's EEPROM, not on the computer. Flashing a new version keeps them
  (settings from older versions are taken over automatically).
- **Serial monitor:** Tools → Serial Monitor, 115200 baud. Type `s` for the menu,
  `m` for *Modules & Strings*, `b` for battery settings, `q` to save and go back.
- More about what this version does: [README.md](README.md).
