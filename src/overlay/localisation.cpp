#include "localisation.h"
#include "configuration.h"
#include "log.h"
#include "platform.h"
#include "util.h"

#include <errno.h>
#include <mutex> // fork: warn once per missing string
#include <unordered_set>

BEGIN_EXTERNAL_HEADERS
#include <filesystem>
#include <glaze/glaze.hpp>
END_EXTERNAL_HEADERS

namespace spacecal {

// enum -> string mapping
// clang-format off
#define LOCALE_LIST \
    X(System,                 "system") \
    X(English_UK,             "en_GB") \
    X(English_US,             "en_US") \
    X(French,                 "fr") \
    X(Italian,                "it") \
    X(German,                 "de") \
    X(Dutch,                  "nl") \
    X(Spanish_Spain,          "es_ES") \
    X(Spanish_LatinAmerica,   "es_US") \
    X(Danish,                 "da") \
    X(Swedish,                "sv") \
    X(Finnish,                "fi") \
    X(Norwegian,              "no") \
    X(Bulgarian,              "bg") \
    X(Polish,                 "pl") \
    X(Czech,                  "cs") \
    X(Greek,                  "el") \
    X(Hungarian,              "hu") \
    X(Portuguese_Portugal,    "pt_PT") \
    X(Portuguese_Brazil,      "pt_BR") \
    X(Romanian,               "ro") \
    X(Russian,                "ru") \
    X(Turkish,                "tr") \
    X(Ukrainian,              "uk") \
    X(Chinese_Simplified,     "zh_HANS") \
    X(Chinese_Traditional,    "zh_HANT") \
    X(Japanese,               "ja") \
    X(Korean,                 "ko") \
    X(Thai,                   "th") \
    X(Vietnamese,             "vi")
// clang-format on

LocalisationManager* LocalisationManager::m_instance = nullptr;

void LocalisationManager::init()
{
    if (m_instance != nullptr) {
        LOG_FATAL("Tried creating LocalisationManager more than once! Breaking singleton. Aborting...");
        return;
    }
    m_instance = this;

    m_nativeLanguageNames.clear();

    // config will be valid by this point, so we can read it
    m_selectedLocale = getLocaleFromRegionString(ConfigurationManager::getInstance()->getConfiguration()->ui_locale);
    if (m_selectedLocale == Locale::System) {
        m_selectedLocale = estimateSystemLocale();
    }

    loadLocalisationStrings(m_selectedLocale);

    // build native tongue locale name list
    LOG_INFO("Building native locale name table...");
#define X(enum_val, str_val) loadNativeNameForLocale(Locale::enum_val, str_val);
    LOCALE_LIST
#undef X
    m_nativeLanguageNames[Locale::System] = m_nativeLanguageNames[estimateSystemLocale()];
}

bool LocalisationManager::setLocale(const Locale locale)
{
    if (locale == Locale::System) {
        m_selectedLocale = estimateSystemLocale();
    } else {
        m_selectedLocale = locale;
    }
    return loadLocalisationStrings(m_selectedLocale);
}

std::string LocalisationManager::getString(const std::string& input) const
{
    if (m_localisedStrings.contains(input)) {
        return m_localisedStrings.at(input);
    }

    // fork: once per key; a missing key drawn every frame wrote 60 lines a second (2026-10-02)
    static std::mutex s_warnedLock;
    static std::unordered_set<std::string> s_warned;
    std::lock_guard<std::mutex> lock(s_warnedLock);
    if (s_warned.insert(input).second)
        LOG_WARN("String \"{0}\" was not defined in a localisation file. Falling back to input key.", input.c_str());

    return input;
}

bool LocalisationManager::loadLocalisationStrings(const Locale locale)
{
    // To load locale strings we first:
    //   - Unload all existing locales
    //   - Load the en_GB locale
    //   - If the locale is not en_GB, load the desired locale, replacing all entries with the new data. This enables english fallback strings

    m_localisedStrings.clear();

    Locale targetLocale = locale;
    if (locale == Locale::System) {
        targetLocale = estimateSystemLocale();
    }

    bool result = loadLocaleFromFile(Locale::English_UK);
    if (locale != Locale::English_UK) {
        result |= loadLocaleFromFile(locale);
    }

    return result;
}

bool LocalisationManager::loadLocaleFromFile(const Locale locale)
{
    auto rootDir = util::getSpaceCalibratorLangsDir();
    std::string langPath = (rootDir / (getLocaleAsRegionString(locale) + ".json")).string();

    errno = 0;
    FILE* localeFile = fopen(langPath.c_str(), "rb");
    if (!localeFile) {
        switch (errno) {
        case ENOENT:
            LOG_WARNING("The locale file \"{0}\" does not exist on disk.", langPath);
            return false;
        case EACCES:
            LOG_WARNING("The locale file \"{0}\" could not be loaded due to misconfigured permissions.", langPath);
            return false;
        case EMFILE:
        case ENFILE:
            LOG_WARNING("The locale file \"{0}\" could not be loaded as too many files are open on the system.", langPath);
            return false;
        case EFBIG:
            LOG_WARNING("The locale file \"{0}\" could not be loaded as it was too large.", langPath);
            return false;
        default:
            LOG_WARNING("The locale file \"{0}\" could not be loaded due to an unknown error ({1}).", langPath, errno);
            return false;
        }
    }
    if (localeFile) {
        // Read to std::string
        fseek(localeFile, 0, SEEK_END);
        size_t localeFileSize = ftell(localeFile);
        std::string buffer;
        buffer.resize(localeFileSize);
        rewind(localeFile);
        fread(&buffer[0], 1, localeFileSize, localeFile);
        fclose(localeFile);
        localeFile = nullptr;

        glz::generic json {};
        glz::error_ctx jsonError = glz::read_jsonc(json, buffer);
        if (jsonError.ec != glz::error_code::none) {
            LOG_WARNING("Failed to parse JSON for locale file \"{0}\". {1}.", langPath, jsonError.custom_error_message);
            return false;
        }
        if (json.empty()) {
            LOG_WARNING("Failed to parse JSON for locale file \"{0}\". Expected JSON object, got empty file.", langPath);
            return false;
        }
        if (!json.is_object()) {
            LOG_WARNING("Failed to parse JSON for locale file \"{0}\". Expected JSON object, got another type.", langPath);
            return false;
        }

        // JSON is in expected format, awesome!
        // Therefore, we should iterate through it and begin loading the key value pairs into memory
        glz::generic::object_t jsonObject = json.as<glz::generic::object_t>();
        for (std::pair<const std::string, glz::generic> elem : jsonObject) {
            if (elem.first.empty()) {
                LOG_WARNING("Failed to parse JSON for locale file \"{0}\". Expected JSON key-value pair of type string, got invalid key.", langPath);
                continue;
            }
            if (!elem.second.is_string()) {
                LOG_WARNING("Failed to parse JSON for locale file \"{0}\". Expected JSON key-value pair of type string, got another value type.", langPath);
                continue;
            }
            m_localisedStrings[elem.first] = elem.second.get_string();
        }
        LOG_INFO("Loaded locale file \"{0}\"...", langPath);

        return true;
    } else {
        LOG_WARNING("The locale file \"{0}\" could not be found on the disk.", langPath);
        return false;
    }

    return false;
}

void LocalisationManager::loadNativeNameForLocale(const Locale locale, const std::string& regionCode)
{
    if (locale == Locale::System || locale == Locale::Count) {
        return;
    }

    const auto rootDir = util::getSpaceCalibratorLangsDir();
    std::string langPath = (rootDir / (regionCode + ".json")).string();
    std::string expectedKey = "languages_" + regionCode;

    // fallback to the loaded locale string
    m_nativeLanguageNames[locale] = getString(expectedKey);

    errno = 0;
    FILE* file = fopen(langPath.c_str(), "rb");
    if (!file) {
        switch (errno) {
        case ENOENT:
            LOG_WARNING("The locale file \"{0}\" does not exist on disk.", langPath);
            return;
        case EACCES:
            LOG_WARNING("The locale file \"{0}\" could not be loaded due to misconfigured permissions.", langPath);
            return;
        case EMFILE:
        case ENFILE:
            LOG_WARNING("The locale file \"{0}\" could not be loaded as too many files are open on the system.", langPath);
            return;
        case EFBIG:
            LOG_WARNING("The locale file \"{0}\" could not be loaded as it was too large.", langPath);
            return;
        default:
            LOG_WARNING("The locale file \"{0}\" could not be loaded due to an unknown error ({1}).", langPath, errno);
            return;
        }
    }

    fseek(file, 0, SEEK_END);
    size_t localeFileSize = ftell(file);
    std::string buffer;
    buffer.resize(localeFileSize);
    rewind(file);
    fread(&buffer[0], 1, localeFileSize, file);
    fclose(file);

    glz::generic json {};
    glz::error_ctx jsonError = glz::read_jsonc(json, buffer);
    if (jsonError.ec != glz::error_code::none) {
        LOG_WARNING("Failed to parse JSON for locale file \"{0}\". {1}.", langPath, jsonError.custom_error_message);
        return;
    }
    if (json.empty()) {
        LOG_WARNING("Failed to parse JSON for locale file \"{0}\". Expected JSON object, got empty file.", langPath);
        return;
    }
    if (!json.is_object()) {
        LOG_WARNING("Failed to parse JSON for locale file \"{0}\". Expected JSON object, got another type.", langPath);
        return;
    }

    glz::generic::object_t jsonObject = json.as<glz::generic::object_t>();

    if (jsonObject.contains(expectedKey)) {
        const glz::generic& value = jsonObject.at(expectedKey);
        if (value.is_string()) {
            m_nativeLanguageNames[locale] = value.get_string();
            return;
        }
    }

    LOG_WARNING("Key \"{0}\" missing or invalid type in \"{1}\".", expectedKey, langPath);
}

std::string LocalisationManager::getNativeTongueLocaleName(const Locale locale) const
{
    auto it = m_nativeLanguageNames.find(locale);
    if (it != m_nativeLanguageNames.end()) {
        return it->second;
    }
    if (locale == Locale::Count) {
#if _DEBUG
        return "cc_CC";
#else
        return "English (UK)";
#endif
    }
#if _DEBUG
    return "xx_XX";
#else
    return "English (UK)";
#endif
}

std::string LocalisationManager::getLocaleAsRegionString(const Locale locale) const
{
    switch (locale) {
#define X(enum_val, str_val) \
    case Locale::enum_val:   \
        return str_val;
        LOCALE_LIST
#undef X
    case Locale::Count:
#if _DEBUG
        return "cc_CC";
#else
        return "en_GB";
#endif
    }
#if _DEBUG
    return "xx_XX";
#else
    return "en_GB";
#endif
}

const Locale LocalisationManager::getLocaleFromRegionString(const std::string& szLocale) const
{
#define X(enum_val, str_val) \
    if (szLocale == str_val) \
        return Locale::enum_val;
    LOCALE_LIST
#undef X

    return Locale::System;
}
}