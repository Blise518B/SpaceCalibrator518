#include "housekeeping.h"

#include "guard_config.h"
#include "log.h"
#include "platform.h" // OS_WINDOWS: without it the body below was compiled out and the collection never ran
#include "util.h"

#include <filesystem>
#include <string>
#include <system_error>

#if OS_WINDOWS
#include <windows.h>
#endif

namespace spacecal::guard {

void startGlitchCollection()
{
    auto* cfg = GuardConfigManager::getInstance();
    if (!cfg || !cfg->get().recorder.enabled || !cfg->get().recorder.glitch_collection)
        return;
#if OS_WINDOWS
    namespace fs = std::filesystem;
    const fs::path repo = util::getSpaceCalibratorInstallDir().parent_path();
    const fs::path python = repo / ".venv" / "Scripts" / "python.exe";
    const fs::path script = repo / "tools" / "glitch_collection.py";
    std::error_code ec;
    if (!fs::exists(python, ec) || !fs::exists(script, ec)) {
        LOG_INFO("Glitch collection: no tools\\ and .venv\\ next to the install, skipped");
        return;
    }
    fs::create_directories(repo / "logs", ec);
    const fs::path logFile = repo / "logs" / "glitch_collection.log";
    // a minute's delay (ping, as timeout needs a console), then the script; output appended to the log
    std::wstring cmd = L"cmd.exe /s /c \"ping -n 61 127.0.0.1 >nul & \"" + python.wstring() + L"\" -u -m tools.glitch_collection --apply --logs >> \"" + logFile.wstring() + L"\" 2>&1\"";
    std::wstring cwd = repo.wstring();
    STARTUPINFOW si {};
    si.cb = sizeof(si);
    PROCESS_INFORMATION pi {};
    if (CreateProcessW(nullptr, cmd.data(), nullptr, nullptr, FALSE, CREATE_NO_WINDOW | BELOW_NORMAL_PRIORITY_CLASS, nullptr, cwd.c_str(), &si, &pi)) {
        CloseHandle(pi.hThread);
        CloseHandle(pi.hProcess);
        LOG_INFO("Glitch collection starts in a minute in the background (log: {})", logFile.string());
    } else {
        LOG_WARN("Glitch collection could not be started (error {})", static_cast<unsigned long>(GetLastError()));
    }
#endif
}

} // namespace spacecal::guard
