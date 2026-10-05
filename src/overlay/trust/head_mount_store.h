#pragma once

// head_mount.json next to config.json: the head tracker's remembered place on the headset (head_mount.h)

#include "head_mount.h"

namespace spacecal::trust {

bool loadHeadMountState(HeadMountState& out);
bool saveHeadMountState(const HeadMountState& state);

} // namespace spacecal::trust
