#include "app_trace_context.h"

namespace {
float g_traceCurrentsA[kTraceSampleCapacity] = {0.0f};
float g_traceVoltagesV[kTraceSampleCapacity] = {0.0f};
size_t g_traceHead = 0;
size_t g_traceCount = 0;
uint32_t g_traceToken = 0;
unsigned long g_lastSampleMs = 0;
unsigned long g_effectiveSampleIntervalMs = kTraceSampleIntervalMs;
bool g_traceWasEnabled = false;

void compact_trace_history() {
  if (g_traceCount < kTraceSampleCapacity) {
    return;
  }

  const size_t oldest = (g_traceHead + kTraceSampleCapacity - g_traceCount) % kTraceSampleCapacity;
  const size_t compactedCount = kTraceSampleCapacity / 2U;
  for (size_t i = 0; i < compactedCount; ++i) {
    const size_t firstSlot = (oldest + (i * 2U)) % kTraceSampleCapacity;
    const size_t secondSlot = (oldest + (i * 2U) + 1U) % kTraceSampleCapacity;
    g_traceCurrentsA[i] = (g_traceCurrentsA[firstSlot] + g_traceCurrentsA[secondSlot]) * 0.5f;
    g_traceVoltagesV[i] = (g_traceVoltagesV[firstSlot] + g_traceVoltagesV[secondSlot]) * 0.5f;
  }

  g_traceHead = compactedCount;
  g_traceCount = compactedCount;
  g_effectiveSampleIntervalMs *= 2UL;
}

void record_sample(float currentA, float voltageV, unsigned long nowMs) {
  compact_trace_history();
  g_traceCurrentsA[g_traceHead] = currentA;
  g_traceVoltagesV[g_traceHead] = voltageV;
  g_traceHead = (g_traceHead + 1U) % kTraceSampleCapacity;
  if (g_traceCount < kTraceSampleCapacity) {
    ++g_traceCount;
  }
  g_lastSampleMs = nowMs;
  ++g_traceToken;
}
}

void app_trace_reset() {
  g_traceHead = 0;
  g_traceCount = 0;
  g_lastSampleMs = 0;
  g_effectiveSampleIntervalMs = kTraceSampleIntervalMs;
  g_traceWasEnabled = false;
  ++g_traceToken;
}

void app_trace_update_cc(bool loadEnabled, float currentA, float voltageV, unsigned long nowMs) {
  if (!loadEnabled) {
    g_traceWasEnabled = false;
    return;
  }

  if (!g_traceWasEnabled) {
    g_traceWasEnabled = true;
    record_sample(currentA, voltageV, nowMs);
    return;
  }

  if (g_traceCount == 0 || (nowMs - g_lastSampleMs) >= g_effectiveSampleIntervalMs) {
    record_sample(currentA, voltageV, nowMs);
  }
}

size_t app_trace_sample_count() {
  return g_traceCount;
}

uint32_t app_trace_update_token() {
  return g_traceToken;
}

float app_trace_duration_seconds() {
  if (g_traceCount <= 1) return 0.0f;
  return static_cast<float>((g_traceCount - 1U) * g_effectiveSampleIntervalMs) / 1000.0f;
}

unsigned long app_trace_effective_interval_ms() {
  return g_effectiveSampleIntervalMs;
}

bool app_trace_read_sample(size_t index, float *currentA, float *voltageV) {
  if (index >= g_traceCount || currentA == nullptr || voltageV == nullptr) {
    return false;
  }

  const size_t oldest = (g_traceHead + kTraceSampleCapacity - g_traceCount) % kTraceSampleCapacity;
  const size_t slot = (oldest + index) % kTraceSampleCapacity;
  *currentA = g_traceCurrentsA[slot];
  *voltageV = g_traceVoltagesV[slot];
  return true;
}
