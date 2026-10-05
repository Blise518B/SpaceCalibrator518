#include "head_mount_store.h"

#include "log.h"
#include "platform.h"
#include "util.h"

BEGIN_EXTERNAL_HEADERS
#include <glaze/glaze.hpp>
END_EXTERNAL_HEADERS

#include <filesystem>

namespace spacecal::trust {

namespace {
    constexpr glz::opts k_JSON_OPTS {
        .comments = true,
        .error_on_unknown_keys = false,
        .prettify = true,
        .indentation_width = 2,
        .error_on_missing_keys = false,
    };

    std::string headMountPath()
    {
        return (util::getSpaceCalibratorConfigDir() / "head_mount.json").string();
    }
}

bool loadHeadMountState(HeadMountState& out)
{
    const std::string path = headMountPath();
    if (!std::filesystem::exists(path))
        return false;
    HeadMountState loaded;
    glz::error_ctx ec = glz::read_file_jsonc<k_JSON_OPTS>(loaded, path, std::string {});
    if (ec.ec != glz::error_code::none) {
        LOG_WARNING("head_mount.json could not be parsed ({}), learning the head tracker's place anew", glz::format_error(ec, std::string {}));
        return false;
    }
    out = loaded;
    return true;
}

bool saveHeadMountState(const HeadMountState& state)
{
    glz::error_ctx ec = glz::write_file_json<k_JSON_OPTS>(state, headMountPath(), std::string {});
    if (ec.ec != glz::error_code::none) {
        LOG_WARNING("head_mount.json could not be written to {}", headMountPath());
        return false;
    }
    return true;
}

} // namespace spacecal::trust
