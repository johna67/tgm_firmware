# TGM Firmware Configuration Analysis

## Executive Summary
The firmware currently has limited runtime configuration capabilities. Most parameters are compile-time only, with only basic BLE register read/write for PPG sensor tuning. This analysis identifies what can be made configurable and how to implement runtime configuration.

---

## 1. CURRENT CONFIGURABLE PARAMETERS

### A. Compile-Time Configuration (prj.conf & Kconfig)

#### Sampling Rates (Compile-time)
```
CONFIG_PPG_SAMPLES_PER_FRAME=20        # Currently set to 20
CONFIG_ACC_SAMPLES_PER_FRAME=25        # Currently set to 25  
CONFIG_TEMPERATURE_MEASUREMENT_INTERVAL=1   # Seconds (currently 1s)
CONFIG_BATTERY_MEASUREMENT_INTERVAL=300     # Seconds (currently 300s = 5min)
```

**Defined in:**
- /home/user/tgm_firmware/app/prj.conf (lines 32-43)
- /home/user/tgm_firmware/app/Kconfig (lines 13-35)

#### Driver Configuration (Compile-time)
```
CONFIG_MAXM86161=y                # PPG sensor driver enabled
CONFIG_LIS2DTW12=y                # Accelerometer driver enabled
CONFIG_BT_DEVICE_NAME="Oralable"  # Bluetooth device name
CONFIG_LOG=y                      # Logging enabled
```

### B. Runtime Configuration (Currently Limited)

#### PPG Sensor Register Access (BLE)
- **Read PPG Register**: Write register address to characteristic UUID `0x3a0ff007`, get response in notify
- **Write PPG Register**: Write register + data (2 bytes) to UUID `0x3a0ff008`, get confirmation in notify
- **LED Control via ppg_set_led_pa()**: API exists but currently hardcoded in state machine

**BLE Characteristic APIs:**
- `ppg_read_reg(uint8_t reg)` - Read single register
- `ppg_write_reg(uint8_t reg, uint8_t data)` - Write single register
- `ppg_set_led_pa(enum ppg_led_t led, uint8_t pa)` - Set LED pulse amplitude

---

## 2. HARDCODED VALUES THAT COULD BE CONFIGURABLE

### A. LED Pulse Amplitude (Light Intensity)
**File:** /home/user/tgm_firmware/app/src/main.c (lines 61-171)

```c
ppg_set_led_pa(PPG_LED_GREEN, 5);    // Idle state indicator
ppg_set_led_pa(PPG_LED_GREEN, 32);   // Charging indicator
ppg_set_led_pa(PPG_LED_RED, 32);     // Worn - Red LED
ppg_set_led_pa(PPG_LED_IR, 128);     // Worn - IR LED (most power intensive)
```

**Values:** 0-255 (each count = 0.12mA with LED range at 31mA)

**Currently State-Driven:**
- Not-Worn, Not-Charging: Green=5
- Not-Worn, Charging: Green=32
- Worn: Red=32, IR=128
- Power savings: Can adjust IR (128 → lower = less power)

### B. Temperature Thresholds (Hysteresis)
**File:** /home/user/tgm_firmware/app/src/main.c (lines 25-26)

```c
#define DEVICE_WORN_TEMPERATURE_THRESHOLD 2550     // 25.50°C - Turn ON red/IR LEDs
#define DEVICE_NOT_WORN_TEMPERATURE_THRESHOLD 2450 // 24.50°C - Turn OFF red/IR LEDs
```

**Use:** Detects when device is on skin (worn state) vs off skin
**Current Hysteresis:** 1°C (2550-2450 = 100 centidegrees)

### C. Measurement Intervals (Timing)
**File:** /home/user/tgm_firmware/app/src/battery.c, main.c

```c
K_SECONDS(CONFIG_BATTERY_MEASUREMENT_INTERVAL)    // 300s default
K_SECONDS(CONFIG_TEMPERATURE_MEASUREMENT_INTERVAL) // 1s default
K_SLEEP(K_MSEC(1))                               // Battery stabilization (hardcoded)
K_SLEEP(K_MSEC(100))                             // Sensor power-up delay (hardcoded)
K_SLEEP(K_SECONDS(1))                            // Main loop sleep (hardcoded)
```

### D. Sensor Configuration (Register-Level Hardcoded)
**File:** /home/user/tgm_firmware/drivers/sensor/maxm86161/maxm86161.c

```c
// PPG Sensor Configuration
uint8_t ppg_config1 = 0b00010111;    // 8192nA full scale, max pulse width
uint8_t ppg_config2 = 0b00001000;    // Sample rate 50Hz, no averaging
uint8_t led_range[2] = {0x0, 0x0};   // LED range: 31mA max

// Initial LED amplitudes (all off)
uint8_t led_pa[3] = {0, 0, 0};       // Green=0, IR=0, Red=0 at startup

// FIFO threshold
uint8_t fifo_config1 = (128 - (COLORS * CONFIG_PPG_SAMPLES_PER_FRAME));
```

**File:** /home/user/tgm_firmware/drivers/sensor/lis2dtw12/lis2dtw12.c

```c
// Accelerometer Configuration
uint8_t ctrl6 = 0b00000100;  // Low-noise config, 2g scale
uint8_t ctrl1 = 0b01000011;  // Low power mode 4 @ 50Hz sampling

// FIFO threshold
uint8_t fifo_ctrl = (0x20 | CONFIG_ACC_SAMPLES_PER_FRAME);
```

### E. Power Management (Hardcoded Sleep)
**File:** /home/user/tgm_firmware/app/src/main.c (lines 398-401)

```c
while (1) {
    k_sleep(K_SECONDS(1));  // Wake every 1 second - HARDCODED
}
```

**Impact:** Cannot adjust power saving vs responsiveness

### F. ADC Battery Voltage Divider
**File:** /home/user/tgm_firmware/app/src/battery.c (line 17)

```c
#define VOLTAGE_DIVIDER_SCALE 11  // Scaling factor for voltage calculation
```

---

## 3. CURRENT BLE API (TGM Service)

### Service UUID
`3a0ff000-98c4-46b2-94af-1aee0fd4c48e`

### Characteristics

| UUID | Type | Access | Purpose |
|------|------|--------|---------|
| 0x3a0ff000 (Service) | - | - | TGM Service |
| 0x3a0ff001 | NOTIFY | Read | PPG Data (with frame counter) |
| 0x3a0ff002 | NOTIFY | Read | Accelerometer Data (with frame counter) |
| 0x3a0ff003 | NOTIFY | Read | Temperature Data (centidegrees) |
| 0x3a0ff004 | NOTIFY/READ | Read | Battery Voltage (mV) |
| 0x3a0ff005 | READ | Read | Device UUID (unique ID) |
| 0x3a0ff006 | READ | Read | Firmware Version (string) |
| 0x3a0ff007 | WRITE/NOTIFY | Write | Read PPG Register (1 byte register address) |
| 0x3a0ff008 | WRITE/NOTIFY | Write | Write PPG Register (2 bytes: register + data) |

### Data Structures Sent

```c
// PPG Data
struct tgm_service_ppg_data_t {
    uint32_t frame_counter;
    struct ppg_sample ppg_data[CONFIG_PPG_SAMPLES_PER_FRAME];
};
struct ppg_sample { uint32_t red; uint32_t ir; uint32_t green; };

// Accelerometer Data  
struct tgm_service_acc_data_t {
    uint32_t frame_counter;
    struct acc_sample acc_data[CONFIG_ACC_SAMPLES_PER_FRAME];
};
struct acc_sample { int16_t x; int16_t y; int16_t z; };

// Temperature Data
struct tgm_service_temp_data_t {
    uint32_t frame_counter;
    int16_t centitemp;  // Temperature in 0.01°C units
};

// Battery: int32_t in mV
```

### Notification Control
All characteristics with NOTIFY have Client Characteristic Configuration (CCC) descriptors that can be enabled/disabled by the mobile app.

---

## 4. RECOMMENDED RUNTIME-CONFIGURABLE PARAMETERS

### Priority: HIGH (Immediate Impact)

#### A. LED Pulse Amplitudes (Power Management)
**Current Status:** Hardcoded in state machine
**Recommendation:** Create BLE characteristic for LED configuration

```
Configurable:
- Green LED PA: 0-255 (for idle/charging indicator)
- Red LED PA: 0-255 (for worn state)
- IR LED PA: 0-255 (highest power consumer - 128 currently)

Benefit: ~20-30% power saving by reducing IR from 128 to 64-96
```

#### B. Temperature Thresholds
**Current Status:** Hardcoded defines (2550/2450 centidegrees)
**Recommendation:** BLE write characteristic

```
Configurable:
- Worn threshold: Default 25.5°C, range 20-35°C
- Not-worn threshold: Default 24.5°C, range 20-35°C
- Hysteresis: Currently 1°C fixed

Benefit: Adapt to user's wear patterns, prevent flicker
```

#### C. Measurement Intervals
**Current Status:** Compile-time only via Kconfig
**Recommendation:** BLE write characteristic

```
Configurable:
- Temperature update rate: 1-60 seconds (default 1s)
- Battery check interval: 10-3600 seconds (default 300s)

Benefit: Trade latency for power consumption
```

### Priority: MEDIUM (Advanced Features)

#### D. Sensor Sampling Rates
**Current Status:** Hardcoded at 50Hz (in driver)
**Recommendation:** Runtime register write via existing API

```
Currently achievable via ppg_write_reg/acc_write_reg:
- PPG sample rate: Register 0x12, bits 2-4
  - 0: 50Hz, 1: 100Hz, 2: 200Hz, 3: 400Hz, 4: 800Hz, 5: 1600Hz
- ACC sample rate: Register 0x20, bits 4-6
  - 01: 25Hz, 10: 50Hz (current), 11: 100Hz, 100: 200Hz

Benefit: Adjust data rate vs power for different use cases
```

#### E. Sensor Thresholds & Gains
**Current Status:** Hardcoded in driver initialization
**Recommendation:** Configurable via register write API

```
Accessible:
- PPG LED pulse width (Register 0x11, bits 0-1)
- PPG photodiode bias (Register 0x15)
- ACC full-scale range (Register 0x25, bits 0-1) - currently 2g
- ACC noise filtering (Register 0x25, bits 2-4)
```

### Priority: LOW (Advanced/Future)

#### F. Power Management Strategies
**Current Status:** Fixed 1-second wake in main loop
**Recommendation:** Configurable sleep strategy

```
Options:
- Idle mode during no-wear (reduce all sampling)
- Dynamic LED adjustment based on signal quality
- Conditional sensor startup (wake only when worn)
```

#### G. Alert Thresholds
**Current Status:** Not implemented
**Recommendation:** New characteristics for triggers

```
Potential:
- Temperature alert range (fever detection)
- Motion threshold (fall detection)
- Data quality thresholds
```

---

## 5. EXISTING CONFIGURATION MECHANISMS

### What Exists:
1. **PPG Register Read/Write** (via BLE characteristics 0x3a0ff007/0x3a0ff008)
   - Allows runtime tuning of PPG sensor registers
   - Requires knowledge of MAXM86161 register map
   - Limited to one register at a time

2. **Compile-Time Kconfig Options**
   - Device name
   - Logging level
   - Sampling frame sizes
   - Measurement intervals

### What's Missing:
1. **Persistent Configuration Storage** (NVS/Flash)
   - Currently disabled: `# CONFIG_FLASH=y`, `# CONFIG_FLASH_MAP=y`
   - No mechanism to save settings between power cycles

2. **Configuration Read/Write BLE Characteristics**
   - Only PPG register access exists
   - No generic configuration service

3. **Configuration Validation**
   - No range checking on inputs
   - No error handling for invalid values

4. **Configuration Presets**
   - No predefined profiles (e.g., "low power", "high quality")

---

## 6. POWER CONSUMPTION OPPORTUNITIES

### Current Power Consumers (Estimated)
1. **IR LED (PPG)** - ~15.4mA @ PA=128 (highest impact)
2. **Red LED (PPG)** - ~3.8mA @ PA=32
3. **Green LED (PPG)** - ~0.6mA @ PA=5
4. **PPG Sensor** - ~3mA (50Hz continuous)
5. **Accelerometer** - ~0.04mA (50Hz, low power mode)
6. **BLE Radio** - ~5mA (active connection)
7. **MCU Core** - ~2-5mA (depends on active work)

### Optimization Levers (via Configuration)
- Reduce IR LED from 128 → 80 = ~5mA saving
- Reduce sampling rate 50Hz → 25Hz = ~20% sensor saving
- Increase battery check 300s → 600s = ~1% saving
- Implement idle mode when not worn = significant saving

---

## 7. IMPLEMENTATION RECOMMENDATIONS

### Phase 1: Basic Runtime Configuration
Add new BLE characteristics for:
1. LED PA values (write 3 bytes: Green, Red, IR)
2. Temperature thresholds (write 4 bytes: worn_threshold, not_worn_threshold)
3. Measurement intervals (write 4 bytes: temp_interval, batt_interval)

### Phase 2: Enhanced BLE Service
1. Implement settings read characteristic (current values)
2. Add validation and range checking
3. Add configuration reset to defaults
4. Support sensor sampling rate configuration

### Phase 3: Persistent Storage
1. Enable NVS (settings subsystem)
2. Save configuration to flash on write
3. Load configuration on startup
4. Implement factory reset

### Phase 4: Advanced Features
1. Configuration presets (power saving, balanced, performance)
2. Data quality monitoring and auto-adjustment
3. Contextual profiles (exercise mode, sleep mode, etc.)

---

## 8. FILE LOCATIONS SUMMARY

| Item | File | Lines |
|------|------|-------|
| Temperature Thresholds | `/app/src/main.c` | 25-26 |
| LED PA Values | `/app/src/main.c` | 61-171 |
| Measurement Intervals | `/app/src/battery.c` | 44, 88 |
| Battery Voltage Scaling | `/app/src/battery.c` | 17 |
| Main Loop Sleep | `/app/src/main.c` | 400 |
| PPG Sensor Config | `/drivers/sensor/maxm86161/maxm86161.c` | 14-94 |
| ACC Sensor Config | `/drivers/sensor/lis2dtw12/lis2dtw12.c` | 63-108 |
| BLE Service Definition | `/app/src/tgm_service.c` | 158-214 |
| Kconfig Options | `/app/Kconfig` | 13-35 |
| Project Config | `/app/prj.conf` | 1-59 |

---

## Summary Table: Current vs. Configurable

| Parameter | Current | Type | Configurable |
|-----------|---------|------|--------------|
| PPG Samples/Frame | 20 | Compile-time | Via Kconfig |
| ACC Samples/Frame | 25 | Compile-time | Via Kconfig |
| Temp Measurement Rate | 1s | Compile-time | Via Kconfig |
| Battery Check Rate | 300s | Compile-time | Via Kconfig |
| Green LED PA | 5/32 | Hardcoded | No (only state-driven) |
| Red LED PA | 32 | Hardcoded | No (only state-driven) |
| IR LED PA | 128 | Hardcoded | No (only state-driven) |
| Worn Temp Threshold | 25.5°C | Hardcoded | No |
| Not-Worn Temp Threshold | 24.5°C | Hardcoded | No |
| PPG Sample Rate | 50Hz | Hardcoded | Yes (via reg write) |
| ACC Sample Rate | 50Hz | Hardcoded | Yes (via reg write) |
| Sensor Power Scale | Various | Hardcoded | Yes (via reg write) |
| Main Loop Sleep | 1s | Hardcoded | No |
| Persistent Config | None | - | No |

