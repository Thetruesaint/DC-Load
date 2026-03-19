#pragma once

#include <stddef.h>
#include <stdint.h>

constexpr size_t kTraceSampleCapacity = 240;
constexpr unsigned long kTraceSampleIntervalMs = 500UL;

void app_trace_reset();
void app_trace_update_cc(bool loadEnabled, float currentA, float voltageV, unsigned long nowMs);
size_t app_trace_sample_count();
uint32_t app_trace_update_token();
float app_trace_duration_seconds();
bool app_trace_read_sample(size_t index, float *currentA, float *voltageV);
