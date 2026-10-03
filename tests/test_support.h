#pragma once

// Minimal test harness for the fork's tests: no external dependency, one executable per test
// file, exit code = number of failed checks.

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <exception>
#include <functional>
#include <string>
#include <vector>

#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <crtdbg.h>
#include <windows.h>
#endif

namespace spacecal::test {

struct TestCase {
    const char* name;
    std::function<void()> fn;
};

inline std::vector<TestCase>& registry()
{
    static std::vector<TestCase> tests;
    return tests;
}

inline int& failures()
{
    static int n = 0;
    return n;
}

struct Registrar {
    Registrar(const char* name, std::function<void()> fn)
    {
        registry().push_back({ name, std::move(fn) });
    }
};

inline void fail(const char* file, int line, const std::string& what);

inline int runAll()
{
    // logs are usually redirected to a file: keep them complete even if a test dies, and never
    // block a headless run with a Windows abort / assertion dialog
    std::setvbuf(stdout, nullptr, _IONBF, 0);
#ifdef _WIN32
    SetErrorMode(SEM_FAILCRITICALERRORS | SEM_NOGPFAULTERRORBOX);
    _set_abort_behavior(0, _WRITE_ABORT_MSG | _CALL_REPORTFAULT);
#ifdef _DEBUG
    _CrtSetReportMode(_CRT_ASSERT, _CRTDBG_MODE_FILE);
    _CrtSetReportFile(_CRT_ASSERT, _CRTDBG_FILE_STDERR);
    _CrtSetReportMode(_CRT_ERROR, _CRTDBG_MODE_FILE);
    _CrtSetReportFile(_CRT_ERROR, _CRTDBG_FILE_STDERR);
#endif
#endif
    int failedTests = 0;
    for (const TestCase& t : registry()) {
        const int before = failures();
        std::printf("[ RUN  ] %s\n", t.name);
        try {
            t.fn();
        } catch (const std::exception& e) {
            fail(__FILE__, __LINE__, std::string("uncaught exception: ") + e.what());
        } catch (...) {
            fail(__FILE__, __LINE__, "uncaught non-standard exception");
        }
        if (failures() == before) {
            std::printf("[  OK  ] %s\n", t.name);
        } else {
            std::printf("[ FAIL ] %s (%d checks failed)\n", t.name, failures() - before);
            failedTests++;
        }
    }
    std::printf("%zu tests, %d failed, %d checks failed\n", registry().size(), failedTests, failures());
    return failures() == 0 ? 0 : 1;
}

inline void fail(const char* file, int line, const std::string& what)
{
    failures()++;
    std::printf("  CHECK FAILED %s:%d: %s\n", file, line, what.c_str());
}

} // namespace spacecal::test

#define TEST(name)                                                                       \
    static void test_##name();                                                           \
    static ::spacecal::test::Registrar registrar_##name(#name, test_##name);             \
    static void test_##name()

#define CHECK(cond)                                                                      \
    do {                                                                                 \
        if (!(cond))                                                                     \
            ::spacecal::test::fail(__FILE__, __LINE__, #cond);                           \
    } while (0)

#define CHECK_EQ(a, b)                                                                   \
    do {                                                                                 \
        if (!((a) == (b)))                                                               \
            ::spacecal::test::fail(__FILE__, __LINE__, std::string(#a " == " #b));       \
    } while (0)

#define CHECK_NEAR(a, b, eps)                                                            \
    do {                                                                                 \
        if (!(std::abs((a) - (b)) <= (eps)))                                             \
            ::spacecal::test::fail(__FILE__, __LINE__, std::string(#a " ~= " #b));       \
    } while (0)
