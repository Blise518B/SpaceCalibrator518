#include "guard_config.h"

#include "log.h"
#include "platform.h"
#include "util.h"

BEGIN_EXTERNAL_HEADERS
#include <glaze/glaze.hpp>
END_EXTERNAL_HEADERS

#include <filesystem>

namespace spacecal::guard {

GuardConfigManager* GuardConfigManager::s_instance = nullptr;

namespace {
    constexpr glz::opts k_JSON_OPTS {
        .comments = true,
        .error_on_unknown_keys = false,
        .prettify = true,
        .indentation_width = 2,
        .error_on_missing_keys = false,
    };
}

bool GuardConfigManager::init()
{
    s_instance = this;
    m_path = (util::getSpaceCalibratorConfigDir() / "guard.json").string();
    if (!std::filesystem::exists(m_path)) {
        LOG_INFO("guard.json not found, writing defaults to {}", m_path);
        return save();
    }
    return load();
}

bool GuardConfigManager::load()
{
    GuardConfig loaded;
    glz::error_ctx ec = glz::read_file_jsonc<k_JSON_OPTS>(loaded, m_path, std::string {});
    if (ec.ec != glz::error_code::none) {
        LOG_ERROR("guard.json could not be parsed ({}), keeping defaults", glz::format_error(ec, std::string {}));
        return false;
    }
    m_config = loaded;
    return true;
}

bool GuardConfigManager::save() const
{
    glz::error_ctx ec = glz::write_file_json<k_JSON_OPTS>(m_config, m_path, std::string {});
    if (ec.ec != glz::error_code::none) {
        LOG_ERROR("guard.json could not be written to {}", m_path);
        return false;
    }
    return true;
}

std::string vkName(int vk)
{
    if (vk >= 0x70 && vk <= 0x87)
        return "F" + std::to_string(vk - 0x70 + 1);
    switch (vk) {
    case 0x13: return "Pause";
    case 0x2C: return "PrintScreen";
    case 0x2D: return "Insert";
    case 0x24: return "Home";
    case 0x23: return "End";
    case 0x21: return "PageUp";
    case 0x22: return "PageDown";
    case 0x91: return "ScrollLock";
    default: break;
    }
    if (vk >= 0x30 && vk <= 0x39)
        return std::string(1, static_cast<char>('0' + (vk - 0x30)));
    if (vk >= 0x41 && vk <= 0x5A)
        return std::string(1, static_cast<char>('A' + (vk - 0x41)));
    if (vk >= 0x60 && vk <= 0x69)
        return "Num" + std::to_string(vk - 0x60);
    char buf[16];
    std::snprintf(buf, sizeof(buf), "VK 0x%02X", vk);
    return buf;
}

} // namespace spacecal::guard
