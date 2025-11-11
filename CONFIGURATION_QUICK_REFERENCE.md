# Configuration Quick Reference

## Most Important Hardcoded Values to Make Configurable

### 1. LED Power (Biggest Impact)
```c
// File: app/src/main.c, lines 61-171
ppg_set_led_pa(PPG_LED_IR, 128);   // ← HIGHEST power consumer
ppg_set_led_pa(PPG_LED_RED, 32);
ppg_set_led_pa(PPG_LED_GREEN, 5);

// Recommendation: Make runtime configurable
// Saving IR: 128→64 = ~5mA reduction (40% of LED power)
```

### 2. Temperature Wear Detection Thresholds
```c
// File: app/src/main.c, lines 25-26
#define DEVICE_WORN_TEMPERATURE_THRESHOLD 2550    // 25.50°C (turns ON red/IR LEDs)
#define DEVICE_NOT_WORN_TEMPERATURE_THRESHOLD 2450 // 24.50°C (turns OFF red/IR LEDs)

// Recommendation: Make runtime configurable
// Different users may have different skin temperatures
```

### 3. Measurement Intervals
```c
// File: app/src/battery.c, line 88
CONFIG_BATTERY_MEASUREMENT_INTERVAL = 300  // Every 5 minutes - CONFIGURABLE AT BUILD

// File: app/src/main.c, line 203
CONFIG_TEMPERATURE_MEASUREMENT_INTERVAL = 1 // Every 1 second - CONFIGURABLE AT BUILD

// Recommendation: Make both runtime configurable
// Trade-off between latency and power consumption
```

## Current BLE Capabilities

### Read/Write PPG Registers (Already Exists!)
```
Read:  Write register address to UUID 0x3a0ff007 → get value in notification
Write: Write [register_address, data] to UUID 0x3a0ff008 → get confirmation

// Can directly control:
// - PPG sampling rate (Register 0x12, bits 2-4)
// - ACC sampling rate (Register 0x20, bits 4-6) - via acc_write_reg when implemented
// - LED ranges and configurations
```

### Current Notifications
- `0x3a0ff001`: PPG data (Red, IR, Green per sample)
- `0x3a0ff002`: Accelerometer data (X, Y, Z)
- `0x3a0ff003`: Temperature (centidegrees)
- `0x3a0ff004`: Battery voltage (mV)
- `0x3a0ff005`: Device UUID (unique ID)
- `0x3a0ff006`: Firmware version

## Recommended New BLE Characteristics (for Runtime Config)

### 1. LED Configuration
```
UUID: 0x3a0ff100 (NEW)
Write: 3 bytes [green_pa, red_pa, ir_pa]
Read:  3 bytes [green_pa, red_pa, ir_pa] (echo back current values)
Range: Each 0-255 (each count = 0.12mA)
```

### 2. Temperature Thresholds
```
UUID: 0x3a0ff101 (NEW)
Write: 4 bytes [worn_threshold_H, worn_threshold_L, not_worn_H, not_worn_L]
       Little-endian int16_t values in centidegrees
Read:  4 bytes (current values)
Range: 20°C-35°C (2000-3500 centidegrees)
```

### 3. Measurement Intervals
```
UUID: 0x3a0ff102 (NEW)
Write: 4 bytes [temp_interval_secs, battery_interval_secs, reserved, reserved]
       Little-endian uint16_t values
Read:  4 bytes (current values)
Range: temp: 1-60, battery: 10-3600 seconds
```

## Files to Modify for Runtime Configuration

### Priority 1 (LED Power Control)
- `/app/src/main.c` - Replace hardcoded LED values with variables
- `/app/src/tgm_service.c` - Add new characteristics for LED configuration
- `/app/src/tgm_service.h` - Add new UUIDs and callbacks

### Priority 2 (Temperature Thresholds)
- `/app/src/main.c` - Replace #define with runtime variables
- `/app/src/tgm_service.c` - Add characteristics for threshold configuration

### Priority 3 (Measurement Intervals)
- `/app/src/battery.c` - Make interval configurable
- `/app/src/main.c` - Make temperature interval configurable
- `/app/src/tgm_service.c` - Add characteristics for interval configuration

## Power Consumption Breakdown (Estimated)

| Component | Current | Configurable |
|-----------|---------|--------------|
| IR LED (PPG) | 15.4mA @ PA=128 | Yes - reduce to 64-96 |
| Red LED (PPG) | 3.8mA @ PA=32 | Yes - reduce as needed |
| Green LED (PPG) | 0.6mA @ PA=5 | Yes - reduce when not charging |
| PPG Sensor | ~3mA @ 50Hz | Yes - reduce sampling rate |
| Accelerometer | ~0.04mA @ 50Hz | Yes - reduce sampling rate |
| BLE Radio | ~5mA | No (always active when connected) |
| MCU Core | 2-5mA | No (depends on work) |

**Total when worn: ~28mA** (without optimizations)
**Total when not worn: ~10mA** (lower LED power)

## Sensor Configuration Details

### PPG (MAXM86161) Accessible Registers
```
0x11 - PPG Config 1: Pulse width, ADC range
0x12 - PPG Config 2: Sample rate (bits 2-4), averaging
0x23-0x28 - LED Pulse Amplitudes (1 byte each, 0-255)
0x15 - Photodiode bias
```

### Accelerometer (LIS2DTW12) Accessible Registers
```
0x20 (CTRL1) - Enable, sample rate (bits 4-6)
0x25 (CTRL6) - Noise mode, full-scale range (currently 2g)
0x2E (FIFO_CTRL) - FIFO threshold (bits 0-5)
```

## Quick Implementation Checklist

- [ ] Create new BLE characteristic for LED PA values
- [ ] Create new BLE characteristic for temperature thresholds
- [ ] Create new BLE characteristic for measurement intervals
- [ ] Add write callbacks for each new characteristic
- [ ] Store configuration values in static variables
- [ ] Add range validation for all inputs
- [ ] Consider NVS (flash) storage for persistence (Phase 3)
- [ ] Test with actual power measurements
- [ ] Document all new UUIDs and data formats
- [ ] Create mobile app controls for new settings

