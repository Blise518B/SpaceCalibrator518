# Space Calibrator 518

A glitch-robust fork of [Space Calibrator 2.0](https://github.com/hyblocker/OpenVR-SpaceCalibrator) for mixed VR setups: a Quest or other inside-out headset together with SteamVR lighthouse trackers and controllers.

<p align="center">
  <a href="https://github.com/Blise518B/SpaceCalibrator518/releases/latest/download/SpaceCalibrator518-Windows.zip">
    <img src="docs/images/download-badge.svg" alt="Download for Windows: SpaceCalibrator518-Windows.zip, latest release">
  </a><br>
  <sub>Every version on the <a href="https://github.com/Blise518B/SpaceCalibrator518/releases">Releases</a> page</sub>
</p>

![The Live tab: head tracker error, guard state, calibration changes](docs/images/live-tab.png)

## Why this fork

Space Calibrator keeps the lighthouse playspace lined up with your headset by watching one tracker on your head. Two things knock that over:

- **A glitching tracker.** The head tracker jumps 20 to 30 cm for a few seconds or minutes (reflections, base station mix-ups). The calibration follows it, and your whole body moves away from your head.
- **SteamVR moving its base stations.** SteamVR keeps re-measuring where its base stations are and moves them on its internal map, up to 30 cm at a time. Every tracker tracked from a moved station jumps, although nothing in your room moved. In one recorded four-hour session this happened 68 times, and trackers jumped by more than 2 cm 129 times, by up to 24 cm.

This fork tells these cases apart from real movement and keeps your body where it belongs.

## What it adds

- **Glitch guard.** Every lighthouse device is checked against its own reported velocity and against all the others. A tracker that jumps alone is flagged and recorded; a jump of the whole playspace is corrected at once. Switch on hold in the Guard tab and the calibration is frozen while the head tracker is in doubt, so its glitches can't move your body (off by default until you have checked a few recordings).
- **Base station moves corrected exactly.** The fork reads SteamVR's map changes from the transforms that come with every pose, so it never mistakes them for your movement. Each tracker stays in place when the station it is tracked from moves, and any held offset slides back to SteamVR's map at 1.5 cm/s. In the session above, jumps over 2 cm went from 129 to 0.
- **Black box recorder.** Records every raw pose. Press F9, the Mark button or hold both triggers when something looks wrong, and the five minutes before and after are saved, with a note of what happened.
- **Live tab.** Real-time graphs of the head tracker error, the guard's verdicts, calibration changes, glitch levels per device and solver health. Every graph explains itself on hover.
- **Trigger hold.** Hold both triggers for 1.5 s to force a fresh calibration and trust every device again.
- **Updates without restarting SteamVR.** All logic lives in the overlay; the driver only applies corrections and publishes poses. The overlay starts with SteamVR.
- **Tools** (`tools/`, Python): replay a recording through the guard, analyse a whole session, turn a session into a one-page report.

## Install

1. Download the zip with the button above and unpack it to a folder that can stay where it is. SteamVR loads the driver from there.
2. Close SteamVR and double-click `use-fork-driver.bat`. It tells SteamVR to load this driver instead of the Steam version of Space Calibrator; both use the same driver name, so only one of them can be active.
3. Start SteamVR, then start `SpaceCalibrator.exe` once. From then on it starts with SteamVR.

`use-steam-driver.bat` switches back to the Steam version.

### Build from source

On Windows with Visual Studio 2022 (C++ workload) and CMake 3.24 or newer:

```
git clone --recursive https://github.com/Blise518B/SpaceCalibrator518
cd SpaceCalibrator518
build.bat
```

`build.bat` builds everything into `dist\` and registers the overlay to start with SteamVR. Then, with SteamVR closed, tell SteamVR to load this driver:

```
powershell -ExecutionPolicy Bypass -File tools\use-driver.ps1 fork
```

`tools\use-driver.ps1 steam` switches back to the Steam version. `tools\make_release_zip.ps1` builds the zip from the download button.

Guard and recorder settings are in `%APPDATA%\space-calibrator\guard.json` and in the Guard tab. How it works in detail: [docs/DESIGN.md](docs/DESIGN.md).

## Credits and license

An unofficial fork, not made or supported by the Space Calibrator developers; please report problems here, not upstream. Based on Space Calibrator 2.0 by Hyblocker and contributors and on the original OpenVR-SpaceCalibrator by Justin Li. MIT license, see [LICENSE](LICENSE).

<sub>Built with the help of AI.</sub>
