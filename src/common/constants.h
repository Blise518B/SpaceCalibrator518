#pragma once

namespace spacecal {

constexpr const char* c_LEGACY_OPENVR_APPLICATION_KEY = "pushrax.SpaceCalibrator";
constexpr const char* c_OPENVR_APPLICATION_KEY = "steam.overlay.3368750"; // dashboard overlay key (upstream)
// fork: this install's SteamVR application key (manifest.vrmanifest). The Steam release keeps
// steam.overlay.3368750; SteamVR keeps only the first manifest it reads for one key.
constexpr const char* c_FORK_OPENVR_APPLICATION_KEY = "blise518b.spacecalibrator518";
constexpr const char* c_SPACE_CALIBRATOR_STEAM_APP_ID = "3368750";
constexpr const char* c_STEAMVR_STEAM_APP_ID = "250820";

// @NOTE: you may see this version referred to as Nova across the code-base. Nova was the internal codename for this version of the app.
#define SPACECAL_VERSION_STRING "v2.0.1-BETA"

#ifndef M_PI
#define M_PI 3.1415926535
#endif // M_PI
}