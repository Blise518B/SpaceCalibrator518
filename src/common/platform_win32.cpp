#include "platform.h"

#if OS_WINDOWS
#include "constants.h"
#include "log.h"
#include <cwchar>
#include <filesystem>

#define WIN32_LEAN_AND_MEAN
#include <combaseapi.h>
#include <knownfolders.h>
#include <shellapi.h>
#include <shlobj.h>
#include <windows.h>

namespace platform {
std::filesystem::path getUserConfigDir()
{
    PWSTR path_pwstr = NULL;
    HRESULT hr = SHGetKnownFolderPath(FOLDERID_RoamingAppData, 0, NULL, &path_pwstr);

    if (SUCCEEDED(hr)) {
        auto thePath = std::filesystem::path(path_pwstr);
        CoTaskMemFree(path_pwstr);
        return thePath;
    }

    return std::filesystem::path();
}

std::filesystem::path getExeDir()
{
    wchar_t path_buf[MAX_PATH] = {};
    DWORD size = GetModuleFileNameW(NULL, path_buf, MAX_PATH);
    if (size > 0 && size <= MAX_PATH) {
        return std::filesystem::path(path_buf).parent_path();
    }

    return std::filesystem::path();
}

// %LOCALAPPDATA%/openvr/openvrpaths.vrpath
std::filesystem::path getSteamvrVrPathsPath()
{

    PWSTR path_pwstr = NULL;
    HRESULT hr = SHGetKnownFolderPath(FOLDERID_LocalAppData, 0, NULL, &path_pwstr);
    if (SUCCEEDED(hr)) {
        auto thePath = std::filesystem::path(path_pwstr) / "openvr" / "openvrpaths.vrpath";
        CoTaskMemFree(path_pwstr);
        return thePath;
    }

    return std::filesystem::path();
}

std::string getEnvVariable(const std::string& szEnvVarName)
{
    char szStringBuffer[256] = {};
    DWORD result = GetEnvironmentVariableA(szEnvVarName.c_str(), szStringBuffer, sizeof(szStringBuffer));
    if (result > 0 && szStringBuffer[0] != 0) {
        return szStringBuffer;
    }
    return "";
}

constexpr const char* s_STEAM_MUTEX_KEY = "Global\\MUTEX__SpaceCalibrator_Steam";
HANDLE s_hSteamMutex = INVALID_HANDLE_VALUE;
bool s_isGitHubVersionInstalled = false;

// fork: one Space Calibrator 518 per desktop session, however it was launched (a launcher, Explorer, install
// script). Upstream only guards Steam launches, so two launcher starts ran two calibrators and two
// recorders side by side (2026-09-30: lost chunks, competing solves). A second launch now brings the
// running window to the front and exits. The handle stays open until shutdownCurrentInstance() or exit.
constexpr const wchar_t* s_FORK_MUTEX_KEY = L"Local\\SpaceCalibrator518_Overlay";
HANDLE s_hForkMutex = nullptr;

static BOOL CALLBACK FocusOtherInstanceWindow(HWND hwnd, LPARAM)
{
    DWORD pid = 0;
    GetWindowThreadProcessId(hwnd, &pid);
    if (pid == 0 || pid == GetCurrentProcessId() || GetWindow(hwnd, GW_OWNER) != nullptr) {
        return TRUE;
    }
    wchar_t className[32] = {};
    if (GetClassNameW(hwnd, className, 32) == 0 || std::wcscmp(className, L"GLFW30") != 0) {
        return TRUE;
    }
    HANDLE process = OpenProcess(PROCESS_QUERY_LIMITED_INFORMATION, FALSE, pid);
    if (!process) {
        return TRUE;
    }
    wchar_t image[MAX_PATH] = {};
    DWORD imageLen = MAX_PATH;
    const bool bGotImage = QueryFullProcessImageNameW(process, 0, image, &imageLen) != 0;
    CloseHandle(process);
    if (!bGotImage || _wcsicmp(std::filesystem::path(image).filename().c_str(), L"SpaceCalibrator.exe") != 0) {
        return TRUE;
    }
    ShowWindow(hwnd, IsIconic(hwnd) ? SW_RESTORE : SW_SHOW);
    // a launcher running in the background may not pass on foreground rights: raise it anyway, flash if refused
    SetWindowPos(hwnd, HWND_TOPMOST, 0, 0, 0, 0, SWP_NOMOVE | SWP_NOSIZE | SWP_SHOWWINDOW);
    SetWindowPos(hwnd, HWND_NOTOPMOST, 0, 0, 0, 0, SWP_NOMOVE | SWP_NOSIZE | SWP_SHOWWINDOW);
    if (!SetForegroundWindow(hwnd)) {
        FLASHWINFO flash = { sizeof(FLASHWINFO), hwnd, FLASHW_ALL | FLASHW_TIMERNOFG, 3, 0 };
        FlashWindowEx(&flash);
    }
    return FALSE;
}

static BOOL CALLBACK FindOwnMainWindow(HWND hwnd, LPARAM lParam)
{
    DWORD pid = 0;
    GetWindowThreadProcessId(hwnd, &pid);
    if (pid != GetCurrentProcessId() || GetWindow(hwnd, GW_OWNER) != nullptr) {
        return TRUE;
    }
    wchar_t className[32] = {};
    if (GetClassNameW(hwnd, className, 32) == 0 || std::wcscmp(className, L"GLFW30") != 0) {
        return TRUE;
    }
    *reinterpret_cast<HWND*>(lParam) = hwnd;
    return FALSE;
}

// A background process may not take the foreground; sharing the input state of the thread that has it
// for the moment of the switch is allowed (the hotkey is polled, so no foreground right comes with it).
static bool forceForeground(HWND hwnd)
{
    const HWND current = GetForegroundWindow();
    const DWORD self = GetCurrentThreadId();
    const DWORD other = current ? GetWindowThreadProcessId(current, nullptr) : 0;
    const bool attached = other != 0 && other != self && AttachThreadInput(self, other, TRUE);
    BringWindowToTop(hwnd);
    SetForegroundWindow(hwnd);
    SetFocus(hwnd);
    if (attached) {
        AttachThreadInput(self, other, FALSE);
    }
    return GetForegroundWindow() == hwnd;
}

void* raiseOwnWindowForInput()
{
    HWND own = nullptr;
    EnumWindows(FindOwnMainWindow, reinterpret_cast<LPARAM>(&own));
    if (!own) {
        return nullptr;
    }
    const HWND previous = GetForegroundWindow();
    if (previous == own) {
        return nullptr;
    }
    ShowWindow(own, IsIconic(own) ? SW_RESTORE : SW_SHOW);
    if (!forceForeground(own)) {
        FLASHWINFO flash = { sizeof(FLASHWINFO), own, FLASHW_ALL | FLASHW_TIMERNOFG, 3, 0 };
        FlashWindowEx(&flash);
        LOG_INFO("Marker note: Windows did not let the window come to the front, flashing it instead");
        return nullptr;
    }
    return previous;
}

void restoreForegroundWindow(void* previous)
{
    const HWND hwnd = static_cast<HWND>(previous);
    if (hwnd && IsWindow(hwnd)) {
        forceForeground(hwnd);
    }
}

bool isAnotherInstanceRunning(bool& bIsRunningViaSteam)
{

    bIsRunningViaSteam = false;
    s_hForkMutex = CreateMutexW(NULL, FALSE, s_FORK_MUTEX_KEY);
    if (s_hForkMutex != nullptr && GetLastError() == ERROR_ALREADY_EXISTS) {
        CloseHandle(s_hForkMutex);
        s_hForkMutex = nullptr;
        EnumWindows(FocusOtherInstanceWindow, 0);
        return true;
    }

    std::string szSteamAppIdEnv = getEnvVariable("SteamAppId");
    if (spacecal::c_SPACE_CALIBRATOR_STEAM_APP_ID == szSteamAppIdEnv || spacecal::c_STEAMVR_STEAM_APP_ID == szSteamAppIdEnv) {
        // We got launched via the Steam client UI.
        s_hSteamMutex = CreateMutexA(NULL, FALSE, s_STEAM_MUTEX_KEY);
        bIsRunningViaSteam = true;
        if (s_hSteamMutex == nullptr) {
            s_hSteamMutex = INVALID_HANDLE_VALUE;
        } else {
            // mutex opened, check if we opened one, if so exit
            if (GetLastError() == ERROR_ALREADY_EXISTS) {
                CloseHandle(s_hSteamMutex);
                s_hSteamMutex = INVALID_HANDLE_VALUE;
                return true;
            }
        }
        return false;
    }

    return false;
}
void shutdownCurrentInstance()
{
    if (s_hForkMutex != nullptr) {
        CloseHandle(s_hForkMutex);
        s_hForkMutex = nullptr;
    }
    if (s_hSteamMutex != INVALID_HANDLE_VALUE && s_hSteamMutex != nullptr) {
        CloseHandle(s_hSteamMutex);
        s_hSteamMutex = nullptr;
    }
}

void showMessageDialog(const std::string& title, const std::string& message)
{
    LOG_INFO("displaying dialog [{}]: {}", title, message);
    MessageBoxA(nullptr, message.c_str(), title.c_str(), 0);
}

void launchDirInFileBrowser(const std::filesystem::path& szDirectory)
{
    PIDLIST_ABSOLUTE nativeFolder = ILCreateFromPathW(szDirectory.c_str());
    if (!nativeFolder) {
        return;
    }

    PCUITEMID_CHILD_ARRAY fileArray = (PCUITEMID_CHILD_ARRAY)&nativeFolder;
    HRESULT hr = SHOpenFolderAndSelectItems(nativeFolder, 1, fileArray, 0);

    if (FAILED(hr)) {
        ShellExecuteW(NULL, L"open", L"explorer.exe", szDirectory.c_str(), NULL, SW_SHOWNORMAL);
    }

    CoTaskMemFree(nativeFolder);
}

void launchWebpage(const std::string& szUrl)
{
    // https://www.betaarchive.com/wiki/index.php/Microsoft_KB_Archive/224816#How_ShellExecute_Interprets_the_URL_Passed
    ShellExecuteA(NULL, "open", szUrl.c_str(), NULL, NULL, SW_SHOWNORMAL);
}

void setThreadName(const std::string& threadName)
{
    std::wstring szWinThreadName(threadName.size(), L' ');
    szWinThreadName.resize(std::mbstowcs(&szWinThreadName[0], threadName.c_str(), threadName.size()));
    SetThreadDescription(GetCurrentThread(), szWinThreadName.c_str());
}
}
#endif