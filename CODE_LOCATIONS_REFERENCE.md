# Code Locations Reference - Configurable Parameters

## Hardcoded LED Pulse Amplitudes

### Location: `/app/src/main.c` (lines 54-183)

**Current Implementation - State Machine Approach:**
```c
// Lines 25-26: Temperature thresholds (hardcoded)
#define DEVICE_WORN_TEMPERATURE_THRESHOLD 2550
#define DEVICE_NOT_WORN_TEMPERATURE_THRESHOLD 2450

// Lines 61-171: LED PA values set based on state
update_state(bool charging_new_state, bool worn_new_state) {
    ...
    if (device_state == DEVICE_STATE_INIT) {
        err = ppg_set_led_pa(PPG_LED_GREEN, 5);        // HARDCODED
    }
    
    if (charging) {
        err = ppg_set_led_pa(PPG_LED_GREEN, 32);       // HARDCODED
    } else {
        err = ppg_set_led_pa(PPG_LED_GREEN, 0);        // HARDCODED
    }
    
    if (worn) {
        err = ppg_set_led_pa(PPG_LED_RED, 32);         // HARDCODED
        err = ppg_set_led_pa(PPG_LED_IR, 128);         // HARDCODED - HIGHEST POWER
    } else {
        err = ppg_set_led_pa(PPG_LED_RED, 0);          // HARDCODED
        err = ppg_set_led_pa(PPG_LED_IR, 0);           // HARDCODED
    }
    ...
}
```

**To Make Configurable:**
1. Create global variables for LED PA values
2. Initialize with defaults matching current hardcoded values
3. Add BLE characteristic to modify these variables
4. Update state machine to use variables instead of literals

---

## Temperature Thresholds

### Location: `/app/src/main.c` (lines 25-26, 219-226)

**Current Implementation:**
```c
#define DEVICE_WORN_TEMPERATURE_THRESHOLD 2550     // 25.50°C
#define DEVICE_NOT_WORN_TEMPERATURE_THRESHOLD 2450 // 24.50°C

static void temperature_work_handler(struct k_work *work) {
    ...
    centitemp = temp_value.val1 * 100 + temp_value.val2 / 10000;
    
    // Check if the device is worn
    if (centitemp > DEVICE_WORN_TEMPERATURE_THRESHOLD) {          // HARDCODED
        update_state(charging, true);
    }
    else if (centitemp < DEVICE_NOT_WORN_TEMPERATURE_THRESHOLD) {  // HARDCODED
        update_state(charging, false);
    }
    ...
}
```

**To Make Configurable:**
1. Replace #defines with runtime variables
2. Initialize with defaults (2550, 2450)
3. Add BLE characteristic for runtime modification
4. Add validation (20°C-35°C range = 2000-3500 centidegrees)

---

## Measurement Intervals

### Location A: Temperature Interval - `/app/src/main.c` (lines 393-403)

**Current Implementation (Compile-time):**
```c
// From prj.conf: CONFIG_TEMPERATURE_MEASUREMENT_INTERVAL=1
// From Kconfig app/Kconfig lines 31-35

static void temperature_work_handler(struct k_work *work) {
    ...
    k_work_reschedule(&temperature_work, K_SECONDS(CONFIG_TEMPERATURE_MEASUREMENT_INTERVAL));
    // Hardcoded at compile-time
}
```

**To Make Configurable:**
1. Create global variable for temperature interval
2. Initialize from CONFIG_TEMPERATURE_MEASUREMENT_INTERVAL at startup
3. Add BLE characteristic to modify (range: 1-60 seconds)
4. Update reschedule calls to use variable

---

### Location B: Battery Interval - `/app/src/battery.c` (lines 88, and `/app/src/main.c` line 320)

**Current Implementation (Compile-time):**
```c
// From prj.conf: CONFIG_BATTERY_MEASUREMENT_INTERVAL=300
// From Kconfig app/Kconfig lines 25-29

static void take_battery_measurement(struct k_work *work) {
    ...
    k_work_reschedule(&battery_measurement_work, 
        K_SECONDS(CONFIG_BATTERY_MEASUREMENT_INTERVAL));  // Hardcoded at compile-time
}

// Stabilization delays (also hardcoded):
k_sleep(K_MSEC(1));                    // Battery ADC stabilization
```

**To Make Configurable:**
1. Create global variable for battery interval
2. Initialize from CONFIG_BATTERY_MEASUREMENT_INTERVAL at startup
3. Add BLE characteristic to modify (range: 10-3600 seconds)
4. Update reschedule calls to use variable

---

## Sensor Driver Configuration

### Location A: PPG Sensor (MAXM86161) - `/drivers/sensor/maxm86161/maxm86161.c` (lines 13-94)

**Hardcoded Register Values:**
```c
int ppg_sensor_start(const struct i2c_dt_spec *i2c) {
    // FIFO configuration (line 18)
    uint8_t fifo_config1 = (128 - (COLORS * CONFIG_PPG_SAMPLES_PER_FRAME));
    // CONFIG_PPG_SAMPLES_PER_FRAME is compile-time configurable
    
    // LED sequence (lines 25-27)
    uint8_t led_seq_reg[3] = {0x23, 0x01, 0x00};  // Red, IR, Green order
    
    // LED range (lines 34-35) - HARDCODED
    uint8_t led_range[2] = {0x0, 0x0};  // 31mA max per LED
    
    // PPG Config 1 (lines 42-43) - HARDCODED
    uint8_t ppg_config1 = 0b00010111;  // 8192nA scale, max pulse width
    
    // PPG Config 2 (lines 50-51) - HARDCODED
    uint8_t ppg_config2 = 0b00001000;  // 50Hz sample rate, no averaging
    // Sample rate bits 2-4: 0=50Hz, 1=100Hz, 2=200Hz, 3=400Hz, 4=800Hz, 5=1600Hz
    
    // Initial LED pulse amplitudes (lines 59-60) - all OFF
    uint8_t led_pa[3] = {0, 0, 0};
    ...
}
```

**Currently Accessible via BLE:**
- Register read/write API exists (ppg_read_reg, ppg_write_reg)
- Can modify any register directly but requires knowledge of register map
- LEDs controlled via ppg_set_led_pa() which calls ppg_write_reg()

**To Make More User-Friendly:**
1. Add high-level BLE characteristics for common adjustments
2. Sample rate configuration (more intuitive than raw register values)
3. LED intensity ranges
4. Document useful register values

---

### Location B: Accelerometer (LIS2DTW12) - `/drivers/sensor/lis2dtw12/lis2dtw12.c` (lines 63-108)

**Hardcoded Register Values:**
```c
int acc_sensor_start(const struct i2c_dt_spec *i2c) {
    // FIFO configuration (lines 68-69)
    uint8_t fifo_ctrl = (0x20 | CONFIG_ACC_SAMPLES_PER_FRAME);
    // CONFIG_ACC_SAMPLES_PER_FRAME is compile-time configurable
    
    // Control Register 6 (lines 76-77) - HARDCODED
    uint8_t ctrl6 = 0b00000100;  // Low-noise mode, 2g full-scale
    
    // Control Register 4 (lines 84-85) - HARDCODED
    uint8_t ctrl4 = 0b00000010;  // Route FIFO threshold to INT1
    
    // Control Register 7 (lines 92-93) - HARDCODED
    uint8_t ctrl7 = 0b00100000;  // Enable FIFO threshold interrupt
    
    // Control Register 1 (lines 100-101) - HARDCODED
    uint8_t ctrl1 = 0b01000011;  // Low power mode 4, 50Hz sample rate
    // Bits 4-6 control sample rate: 01=25Hz, 10=50Hz, 11=100Hz, 100=200Hz
    ...
}
```

**Currently Accessible via BLE:**
- Only PPG register read/write exposed (ppg_read_reg, ppg_write_reg)
- No accelerometer equivalent (acc_read_reg, acc_write_reg methods exist but not exposed in BLE)
- Could add BLE characteristic to call these methods

**To Make Configurable:**
1. Expose acc_read_reg/acc_write_reg via BLE (similar to PPG)
2. Add high-level characteristics for ACC sample rate
3. Add ACC range configuration (2g hardcoded, could allow 4g, 8g, 16g)

---

## Main Application Flow

### Location: `/app/src/main.c` (lines 278-404)

**Initialization Sequence:**
```c
int main(void) {
    // Battery power latch (lines 285-287) - CRITICAL FIRST
    // GPIO and sensor power setup (lines 289-332)
    
    // Service initialization (lines 299-306)
    ble_init();
    tgm_service_init(&tgm_service_callbacks);
    
    // Work queue initialization (lines 309-310)
    k_work_init_delayable(&temperature_work, temperature_work_handler);
    
    // Sensor initialization (lines 313-347)
    battery_init(NULL);
    ppg_init();
    acc_init();
    
    // State machine setup for charging detection (lines 349-374)
    // Sensor startup (lines 377-387)
    ppg_start();
    acc_start();
    
    // Temperature monitoring startup (lines 389-391)
    k_work_reschedule(&temperature_work, K_NO_WAIT);
    
    // Main loop (lines 398-401) - HARDCODED 1 SECOND SLEEP
    while (1) {
        k_sleep(K_SECONDS(1));  // ← HARDCODED - could be made configurable
    }
}
```

**Impact of 1-second sleep:**
- Limits power savings potential
- Could be reduced for responsiveness or increased for power savings
- Currently only used to keep BLE connection alive

---

## BLE Service Definition

### Location: `/app/src/tgm_service.c` (lines 158-293)

**Current Characteristics:**
```c
BT_GATT_SERVICE_DEFINE(
    tgm_service_svc,
    BT_GATT_PRIMARY_SERVICE(BT_UUID_TGM),              // 0x3a0ff000
    
    // UUID 0x3a0ff005 - Device UUID (READ only)
    BT_GATT_CHARACTERISTIC(BT_UUID_TGM_UUID, ...),
    
    // UUID 0x3a0ff006 - Firmware Version (READ only)
    BT_GATT_CHARACTERISTIC(BT_UUID_TGM_FW, ...),
    
    // UUID 0x3a0ff004 - Battery (READ + NOTIFY)
    BT_GATT_CHARACTERISTIC(BT_UUID_TGM_BAT, 
        BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY, ...),
    
    // UUID 0x3a0ff001 - PPG Data (NOTIFY only)
    BT_GATT_CHARACTERISTIC(BT_UUID_TGM_PPG, 
        BT_GATT_CHRC_NOTIFY, ...),
    
    // UUID 0x3a0ff002 - Accelerometer Data (NOTIFY only)
    BT_GATT_CHARACTERISTIC(BT_UUID_TGM_ACC, 
        BT_GATT_CHRC_NOTIFY, ...),
    
    // UUID 0x3a0ff003 - Temperature Data (NOTIFY only)
    BT_GATT_CHARACTERISTIC(BT_UUID_TGM_TEMP, 
        BT_GATT_CHRC_NOTIFY, ...),
    
    // UUID 0x3a0ff007 - Read PPG Register (WRITE + NOTIFY)
    BT_GATT_CHARACTERISTIC(BT_UUID_TGM_READ_PPG_REG, 
        BT_GATT_CHRC_WRITE | BT_GATT_CHRC_NOTIFY, ...),
    
    // UUID 0x3a0ff008 - Write PPG Register (WRITE + NOTIFY)
    BT_GATT_CHARACTERISTIC(BT_UUID_TGM_WRITE_PPG_REG, 
        BT_GATT_CHRC_WRITE | BT_GATT_CHRC_NOTIFY, ...),
);
```

**To Add New Configurable Characteristics:**
```c
// Template for LED Configuration (NEW)
BT_GATT_CHARACTERISTIC(
    BT_UUID_TGM_LED_CONFIG,  // 0x3a0ff100 (NEW)
    BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE,
    BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,
    get_led_config, set_led_config,
    &led_config_data),
BT_GATT_CCC(led_config_ccc_changed, ...),

// Similar patterns for:
// - Temperature threshold configuration (0x3a0ff101)
// - Measurement interval configuration (0x3a0ff102)
```

---

## Configuration Callbacks

### Location: `/app/src/tgm_service.c` (lines 36-156)

**Callback Pattern:**
```c
// For read characteristics (lines 73-108)
static ssize_t get_bat_value(struct bt_conn *conn, 
    const struct bt_gatt_attr *attr, void *buf, 
    uint16_t len, uint16_t offset) {
    // Called when client reads the characteristic
    // Return the data to send
}

// For write characteristics (lines 112-156)
static ssize_t read_ppg_reg(struct bt_conn *conn, 
    const struct bt_gatt_attr *attr, const void *buf, 
    uint16_t len, uint16_t offset, uint8_t flags) {
    // Called when client writes to the characteristic
    // Extract data from buf, validate, apply changes
    // Return number of bytes consumed
}

// For CCC (notifications) changed (lines 36-70)
static void tgm_service_ccc_ppg_data_cfg_changed(
    const struct bt_gatt_attr *attr, uint16_t value) {
    // Called when client enables/disables notifications
    notify_ppg_data = (value == BT_GATT_CCC_NOTIFY);
}
```

**New callbacks would follow same pattern:**
```c
// Configuration getter
static ssize_t get_led_config(...) { ... }

// Configuration setter with validation
static ssize_t set_led_config(...) {
    if (len != 3) return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
    
    uint8_t *data = (uint8_t *)buf;
    // Validate ranges
    if (data[0] > 255 || data[1] > 255 || data[2] > 255)
        return BT_GATT_ERR(BT_ATT_ERR_VALUE_NOT_ALLOWED);
    
    // Update global variables
    led_config.green = data[0];
    led_config.red = data[1];
    led_config.ir = data[2];
    
    // Apply changes
    ppg_set_led_pa(PPG_LED_GREEN, led_config.green);
    ppg_set_led_pa(PPG_LED_RED, led_config.red);
    ppg_set_led_pa(PPG_LED_IR, led_config.ir);
    
    return len;
}
```

---

## UUIDs Definition

### Location: `/app/src/tgm_service.h` (lines 22-57)

**Current UUIDs:**
```c
#define BT_UUID_TGM_VAL \
    BT_UUID_128_ENCODE(0x3a0ff000, 0x98c4, 0x46b2, 0x94af, 0x1aee0fd4c48e)
    
#define BT_UUID_TGM_PPG_VAL \
    BT_UUID_128_ENCODE(0x3a0ff001, ...)  // PPG data notify
    
#define BT_UUID_TGM_ACC_VAL \
    BT_UUID_128_ENCODE(0x3a0ff002, ...)  // Accelerometer data notify
    
#define BT_UUID_TGM_TEMP_VAL \
    BT_UUID_128_ENCODE(0x3a0ff003, ...)  // Temperature data notify
    
#define BT_UUID_TGM_BAT_VAL \
    BT_UUID_128_ENCODE(0x3a0ff004, ...)  // Battery value read/notify
    
#define BT_UUID_TGM_UUID_VAL \
    BT_UUID_128_ENCODE(0x3a0ff005, ...)  // Device UUID read
    
#define BT_UUID_TGM_FW_VAL \
    BT_UUID_128_ENCODE(0x3a0ff006, ...)  // Firmware version read
    
#define BT_UUID_TGM_READ_PPG_REG_VAL \
    BT_UUID_128_ENCODE(0x3a0ff007, ...)  // PPG register read
    
#define BT_UUID_TGM_WRITE_PPG_REG_VAL \
    BT_UUID_128_ENCODE(0x3a0ff008, ...)  // PPG register write
```

**Suggested New UUIDs:**
```c
#define BT_UUID_TGM_LED_CONFIG_VAL \
    BT_UUID_128_ENCODE(0x3a0ff100, 0x98c4, 0x46b2, 0x94af, 0x1aee0fd4c48e)
    
#define BT_UUID_TGM_TEMP_THRESHOLD_VAL \
    BT_UUID_128_ENCODE(0x3a0ff101, 0x98c4, 0x46b2, 0x94af, 0x1aee0fd4c48e)
    
#define BT_UUID_TGM_MEASUREMENT_INTERVAL_VAL \
    BT_UUID_128_ENCODE(0x3a0ff102, 0x98c4, 0x46b2, 0x94af, 0x1aee0fd4c48e)
```

