#pragma once

#ifndef RACER3_RTTASK_GLOBALS_H
#define RACER3_RTTASK_GLOBALS_H

#include <cstdio>
#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <type_traits>
#include <utility>

#include "rsi.h"
#include "rttask.h"

#if defined(WIN32)
#define RACER3_RTTASK_EXPORT __declspec(dllexport)
#define RACER3_RTTASK_IMPORT __declspec(dllimport)
#elif defined(__linux__)
#define RACER3_RTTASK_EXPORT __attribute__((visibility("default")))
#define RACER3_RTTASK_IMPORT __attribute__((visibility("default")))
#else
#define RACER3_RTTASK_EXPORT
#define RACER3_RTTASK_IMPORT
#endif

#define RACER3_NAME(name) name
#define RACER3_CONCAT(left, right) left##right

#define RACER3_RSI_TASK(name)                                                                                                                                                                  \
    void RACER3_CONCAT(name, Core)(RSI::RapidCode::RealTimeTasks::GlobalData*);                                                                                                                \
    extern "C" RACER3_RTTASK_EXPORT int32_t RACER3_NAME(name)(RSI::RapidCode::RealTimeTasks::GlobalData* data, char* buffer, const uint32_t size)                                             \
    {                                                                                                                                                                                          \
        return Racer3RTTaskCallFunction(RACER3_CONCAT(name, Core), data, buffer, size);                                                                                                       \
    }                                                                                                                                                                                          \
    void RACER3_CONCAT(name, Core)(RSI::RapidCode::RealTimeTasks::GlobalData* data)

template <typename FunctionType>
int32_t Racer3RTTaskCallFunction(FunctionType&& function, RSI::RapidCode::RealTimeTasks::GlobalData* data, char* buffer, const uint32_t size)
{
    int32_t result = 0;
    try
    {
        function(data);
    }
    catch (const std::exception& error)
    {
        result = -1;
        std::snprintf(buffer, size, "%s", error.what());
    }
    catch (...)
    {
        result = -1;
        std::snprintf(buffer, size, "Unknown Racer3 RTTask error.");
    }
    return result;
}

namespace RSI
{
namespace RapidCode
{
namespace RealTimeTasks
{

struct GlobalData
{
    GlobalData() { std::memset(this, 0, sizeof(*this)); }
    GlobalData(GlobalData&& other) { std::memcpy(this, &other, sizeof(*this)); }

    // Lifecycle / heartbeat. These are the first globals the host should read
    // when validating that the RTTask manager and DLL are wired correctly.
    RSI_GLOBAL(bool, initialized);
    RSI_GLOBAL(bool, metadataReady);
    RSI_GLOBAL(bool, runtimeReady);
    RSI_GLOBAL(bool, statusSamplerReady);
    // Official RSI-sample-compatible counter used by Increment.  Keeping this
    // name/function pair makes Racer3's first RTTask probe match the published
    // Hello RTTasks sample before any Racer3-specific object access is attempted.
    RSI_GLOBAL(int64_t, counter);
    // Incremented by Racer3BasicHeartbeat without touching RapidCode objects.
    // This proves task dispatch separately from controller-object sampling.
    RSI_GLOBAL(int64_t, basicHeartbeat);
    RSI_GLOBAL(int64_t, heartbeat);
    RSI_GLOBAL(int64_t, initializationCount);
    RSI_GLOBAL(int64_t, lastSampleCounter);
    RSI_GLOBAL(int64_t, lastNetworkCounter);
    RSI_GLOBAL(int64_t, statusSampleCount);
    RSI_GLOBAL(double, samplePeriodSeconds);
    RSI_GLOBAL(int32_t, lastInitializationStep);
    RSI_GLOBAL(int32_t, lastStatusSamplerStep);
    RSI_GLOBAL(int32_t, lastStatusReadinessCode);

    // Host-to-RTTask jog intent shadow. This v15 scaffold does not command
    // motion; it only proves that intent can be delivered to deterministic
    // controller-side code without changing the stable v14 jog path.
    RSI_GLOBAL(bool, jogEnabled);
    RSI_GLOBAL(bool, jogStopRequested);
    RSI_GLOBAL(bool, jogMotionArmed);
    RSI_GLOBAL(int64_t, jogCommandSequence);
    RSI_GLOBAL(int32_t, jogDirectionCode);
    RSI_GLOBAL(int32_t, jogTargetAxis);
    RSI_GLOBAL(double, jogSpeedMetersPerSecond);
    RSI_GLOBAL(double, jogSpeedUserUnitsPerSecond);
    RSI_GLOBAL(double, jogStepUserUnits);
    RSI_GLOBAL(int64_t, lastJogCommandSequenceSeen);
    RSI_GLOBAL(int64_t, jogIntentTransitions);
    RSI_GLOBAL(int64_t, lastJogStopSeen);
    RSI_GLOBAL(int64_t, lastMotionCommandSequenceAccepted);
    RSI_GLOBAL(int64_t, lastMotionCommandSequenceRejected);
    RSI_GLOBAL(int32_t, lastMotionRejectCode);
    RSI_GLOBAL(bool, lastMotionCommandIssued);
    RSI_GLOBAL(bool, lastMotionCommandDone);
    RSI_GLOBAL(int32_t, lastMotionAxis);
    RSI_GLOBAL(double, lastMotionStepUserUnits);
    RSI_GLOBAL(double, lastMotionSpeedUserUnitsPerSecond);

    // Controller / motion status sampled by the RTTask side.
    RSI_GLOBAL(bool, multiAxisReady);
    RSI_GLOBAL(bool, multiAxisAmpEnabled);
    RSI_GLOBAL(bool, multiAxisMotionDone);
    RSI_GLOBAL(int32_t, multiAxisState);
    RSI_GLOBAL(int32_t, multiAxisMotionId);
    RSI_GLOBAL(int32_t, multiAxisExecutingMotionId);
    RSI_GLOBAL(bool, allAxisAmpEnabled);
    RSI_GLOBAL(bool, allAxisMotionDone);
    RSI_GLOBAL(bool, targetAxisAmpEnabled);

    // Axis command/actual positions. One scalar per axis keeps the global
    // metadata simple and compatible with the RSI_GLOBAL registration model.
    RSI_GLOBAL(double, axis0CommandPosition);
    RSI_GLOBAL(double, axis1CommandPosition);
    RSI_GLOBAL(double, axis2CommandPosition);
    RSI_GLOBAL(double, axis3CommandPosition);
    RSI_GLOBAL(double, axis4CommandPosition);
    RSI_GLOBAL(double, axis5CommandPosition);
    RSI_GLOBAL(double, axis0ActualPosition);
    RSI_GLOBAL(double, axis1ActualPosition);
    RSI_GLOBAL(double, axis2ActualPosition);
    RSI_GLOBAL(double, axis3ActualPosition);
    RSI_GLOBAL(double, axis4ActualPosition);
    RSI_GLOBAL(double, axis5ActualPosition);

    // Diagnostics / fault containment for the future motion task.
    RSI_GLOBAL(int64_t, taskErrorCount);
    RSI_GLOBAL(int32_t, lastErrorCode);
};

inline constexpr GlobalMetadataMap<RSI::RapidCode::RealTimeTasks::GlobalMaxSize> GlobalMetadata(
    {
        REGISTER_GLOBAL(initialized),
        REGISTER_GLOBAL(metadataReady),
        REGISTER_GLOBAL(runtimeReady),
        REGISTER_GLOBAL(statusSamplerReady),
        REGISTER_GLOBAL(counter),
        REGISTER_GLOBAL(basicHeartbeat),
        REGISTER_GLOBAL(heartbeat),
        REGISTER_GLOBAL(initializationCount),
        REGISTER_GLOBAL(lastSampleCounter),
        REGISTER_GLOBAL(lastNetworkCounter),
        REGISTER_GLOBAL(statusSampleCount),
        REGISTER_GLOBAL(samplePeriodSeconds),
        REGISTER_GLOBAL(lastInitializationStep),
        REGISTER_GLOBAL(lastStatusSamplerStep),
        REGISTER_GLOBAL(lastStatusReadinessCode),
        REGISTER_GLOBAL(jogEnabled),
        REGISTER_GLOBAL(jogStopRequested),
        REGISTER_GLOBAL(jogMotionArmed),
        REGISTER_GLOBAL(jogCommandSequence),
        REGISTER_GLOBAL(jogDirectionCode),
        REGISTER_GLOBAL(jogTargetAxis),
        REGISTER_GLOBAL(jogSpeedMetersPerSecond),
        REGISTER_GLOBAL(jogSpeedUserUnitsPerSecond),
        REGISTER_GLOBAL(jogStepUserUnits),
        REGISTER_GLOBAL(lastJogCommandSequenceSeen),
        REGISTER_GLOBAL(jogIntentTransitions),
        REGISTER_GLOBAL(lastJogStopSeen),
        REGISTER_GLOBAL(lastMotionCommandSequenceAccepted),
        REGISTER_GLOBAL(lastMotionCommandSequenceRejected),
        REGISTER_GLOBAL(lastMotionRejectCode),
        REGISTER_GLOBAL(lastMotionCommandIssued),
        REGISTER_GLOBAL(lastMotionCommandDone),
        REGISTER_GLOBAL(lastMotionAxis),
        REGISTER_GLOBAL(lastMotionStepUserUnits),
        REGISTER_GLOBAL(lastMotionSpeedUserUnitsPerSecond),
        REGISTER_GLOBAL(multiAxisReady),
        REGISTER_GLOBAL(multiAxisAmpEnabled),
        REGISTER_GLOBAL(multiAxisMotionDone),
        REGISTER_GLOBAL(multiAxisState),
        REGISTER_GLOBAL(multiAxisMotionId),
        REGISTER_GLOBAL(multiAxisExecutingMotionId),
        REGISTER_GLOBAL(allAxisAmpEnabled),
        REGISTER_GLOBAL(allAxisMotionDone),
        REGISTER_GLOBAL(targetAxisAmpEnabled),
        REGISTER_GLOBAL(axis0CommandPosition),
        REGISTER_GLOBAL(axis1CommandPosition),
        REGISTER_GLOBAL(axis2CommandPosition),
        REGISTER_GLOBAL(axis3CommandPosition),
        REGISTER_GLOBAL(axis4CommandPosition),
        REGISTER_GLOBAL(axis5CommandPosition),
        REGISTER_GLOBAL(axis0ActualPosition),
        REGISTER_GLOBAL(axis1ActualPosition),
        REGISTER_GLOBAL(axis2ActualPosition),
        REGISTER_GLOBAL(axis3ActualPosition),
        REGISTER_GLOBAL(axis4ActualPosition),
        REGISTER_GLOBAL(axis5ActualPosition),
        REGISTER_GLOBAL(taskErrorCount),
        REGISTER_GLOBAL(lastErrorCode),
    });

extern "C"
{
    RACER3_RTTASK_EXPORT int32_t GlobalMemberOffsetGet(const char* const name)
    {
        return GlobalMetadata[name].offset;
    }
    static_assert(std::is_same<decltype(&GlobalMemberOffsetGet), GlobalMemberOffsetGetter>::value,
        "GlobalMemberOffsetGet function signature does not match GlobalMemberOffsetGetter type.");

    RACER3_RTTASK_EXPORT int32_t GlobalNamesFill(const char* names[], int32_t capacity)
    {
        int32_t index = 0;
        for (; index < GlobalMetadata.Size() && index < capacity; ++index)
        {
            names[index] = GlobalMetadata[index].key;
        }
        return index;
    }
    static_assert(std::is_same<decltype(&GlobalNamesFill), GlobalNamesGetter>::value,
        "GlobalNamesFill function signature does not match GlobalNamesGetter type.");

    RACER3_RTTASK_EXPORT int32_t GlobalMemberTypeGet(const char* const name)
    {
        return static_cast<int32_t>(GlobalMetadata[name].type);
    }
    static_assert(std::is_same<decltype(&GlobalMemberTypeGet), GlobalMemberTypeGetter>::value,
        "GlobalMemberTypeGet function signature does not match GlobalMemberTypeGetter type.");
}

static_assert(sizeof(GlobalData) <= RSI::RapidCode::RealTimeTasks::GlobalMaxSize,
    "Racer3 RTTask GlobalData is too large for the RTTask global buffer.");

} // namespace RealTimeTasks
} // namespace RapidCode
} // namespace RSI

extern "C"
{
    RACER3_RTTASK_IMPORT RSI::RapidCode::MotionController* MotionControllerGet(char* errorBuffer, const uint32_t errorBufferSize);
    RACER3_RTTASK_IMPORT RSI::RapidCode::Axis* AxisGet(const int32_t axisIndex, char* errorBuffer, const uint32_t errorBufferSize);
    RACER3_RTTASK_IMPORT RSI::RapidCode::MultiAxis* MultiAxisGet(const int32_t index, char* errorBuffer, const uint32_t errorBufferSize);
}

template <typename FunctionType, typename... Args>
auto Racer3RTObjectGet(FunctionType&& function, Args&&... args)
{
    char errorBuffer[256] = {};
    auto* object = std::forward<FunctionType>(function)(std::forward<Args>(args)..., errorBuffer, sizeof(errorBuffer));
    if (object == nullptr)
    {
        throw std::runtime_error(errorBuffer);
    }
    return object;
}

inline auto Racer3RTMotionControllerGet() { return Racer3RTObjectGet(MotionControllerGet); }
inline auto Racer3RTAxisGet(const int32_t index) { return Racer3RTObjectGet(AxisGet, index); }
inline auto Racer3RTMultiAxisGet(const int32_t index) { return Racer3RTObjectGet(MultiAxisGet, index); }

#endif // RACER3_RTTASK_GLOBALS_H
