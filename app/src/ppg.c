/*
 * Copyright (c) 2024 WeeGee bv
 */

#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>

#include "tgm_service.h"
#include "ppg.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(ppg, CONFIG_APP_LOG_LEVEL);

static struct gpio_callback ppg_int_cb;
static struct k_work read_ppg_data_work;

struct ppg_reg_work_t
{
    struct k_work reg_work;
    uint8_t reg;
    uint8_t data;
    bool read;
};

static void ppg_reg_work_handler(struct k_work *work);

static struct ppg_reg_work_t ppg_reg_work;

#if CONFIG_MAXM86161
#include <app/drivers/maxm86161.h>

#define MAXM86161_NODE DT_NODELABEL(maxm86161)
static struct gpio_dt_spec ppg_int = GPIO_DT_SPEC_GET(MAXM86161_NODE, int_gpios);

static struct i2c_dt_spec i2c = I2C_DT_SPEC_GET(MAXM86161_NODE);
#else
// Give a build error
#error "No valid PPG sensor driver enabled"
#endif

/**
 * @brief Callback function for the PPG sensor interrupt
 *
 * @param dev Pointer to the gpio port that triggered the callback
 * @param cb Pointer to the callback data that was used to set up the callback
 * @param pins Bitmask of pins that triggered the callback
 * @return int
 */
static void ppg_int_handler(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
    if (pins & BIT(ppg_int.pin))
    {
        // Read the PPG data outside of the ISR
        k_work_submit(&read_ppg_data_work);
    }

    return;
}

static void ppg_read_data(struct k_work *work)
{
    struct ppg_sample ppg_data[CONFIG_PPG_SAMPLES_PER_FRAME];
    uint8_t sample_count;

    // Get the PPG data
    int err = ppg_sensor_get_data(&i2c, ppg_data, &sample_count);
    if (err)
    {
        LOG_ERR("Failed to read PPG data");
        return;
    }

    // Notify the client of the PPG data
    err = tgm_service_send_ppg_notify(ppg_data, sample_count);
    if (err)
    {
        LOG_DBG("Failed to send PPG data notification");
        return;
    }

    return;
}

// app/src/ppg.c
#include "ppg.h"
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(ppg, CONFIG_PPG_LOG_LEVEL);

// Masseter-specific configuration constants
#define MASSETER_LED_RED_CURRENT    0x1F    // 6.2mA - lower for thin tissue
#define MASSETER_LED_IR_CURRENT     0x2F    // 9.4mA - better penetration
#define MASSETER_LED_GREEN_CURRENT  0x1F    // 6.2mA - surface monitoring
#define MASSETER_SAMPLE_RATE        25      // 25Hz optimal for jaw activity
#define MASSETER_FIFO_THRESHOLD     15      // Trigger at 15 samples (600ms)

int ppg_init(void)
{
    int err;
    
    LOG_INF("Initializing PPG sensor for masseter monitoring");
    
    // Initialize MAXM86161
    err = maxm86161_init();
    if (err) {
        LOG_ERR("MAXM86161 init failed: %d", err);
        return err;
    }
    
    // Configure LED currents for masseter (thin cheek tissue)
    // Lower power = better battery life
    err = maxm86161_write_reg(MAXM86161_REG_LED1_PA, MASSETER_LED_RED_CURRENT);
    if (err) {
        LOG_ERR("Failed to set RED LED current: %d", err);
        return err;
    }
    
    err = maxm86161_write_reg(MAXM86161_REG_LED2_PA, MASSETER_LED_IR_CURRENT);
    if (err) {
        LOG_ERR("Failed to set IR LED current: %d", err);
        return err;
    }
    
    err = maxm86161_write_reg(MAXM86161_REG_LED3_PA, MASSETER_LED_GREEN_CURRENT);
    if (err) {
        LOG_ERR("Failed to set GREEN LED current: %d", err);
        return err;
    }
    
    // Set sample rate to 25Hz (battery efficient)
    // Register value: 0x27 = 25Hz, 18-bit resolution
    err = maxm86161_write_reg(MAXM86161_REG_SPO2_CONFIG, 0x27);
    if (err) {
        LOG_ERR("Failed to set sample rate: %d", err);
        return err;
    }
    
    // Configure FIFO to minimize interrupts (battery saving)
    // Trigger when 15 samples are ready (600ms @ 25Hz)
    uint8_t fifo_threshold = 128 - (MASSETER_FIFO_THRESHOLD * 3);  // 3 channels
    err = maxm86161_write_reg(MAXM86161_REG_FIFO_CONFIG, fifo_threshold);
    if (err) {
        LOG_ERR("Failed to configure FIFO: %d", err);
        return err;
    }
    
    LOG_INF("PPG configured: 25Hz, LEDs optimized for masseter");
    LOG_INF("  RED: %d (6.2mA), IR: %d (9.4mA), GREEN: %d (6.2mA)",
            MASSETER_LED_RED_CURRENT, MASSETER_LED_IR_CURRENT, MASSETER_LED_GREEN_CURRENT);
    
    return 0;
}

// Keep all other existing ppg.c functions unchanged

int ppg_start(void)
{
    // Enable the interrupt
    int err = gpio_pin_interrupt_configure_dt(&ppg_int, GPIO_INT_EDGE_TO_ACTIVE);
    if (err)
    {
        LOG_ERR("Failed to configure PPG sensor int pin interrupt");
        return err;
    }

    // Start the PPG sensor
    err = ppg_sensor_start(&i2c);
    if (err)
    {
        LOG_ERR("Failed to start PPG sensor");
        return err;
    }

    return 0;
}

int ppg_stop(void)
{
    // Stop the PPG sensor
    int err = ppg_sensor_stop(&i2c);
    if (err)
    {
        LOG_ERR("Failed to stop PPG sensor");
        return err;
    }

    // Disable the interrupt
    err = gpio_pin_interrupt_configure_dt(&ppg_int, GPIO_INT_DISABLE);
    if (err)
    {
        LOG_ERR("Failed to disable PPG sensor int pin interrupt");
        return err;
    }

    return 0;
}

int ppg_read_reg(uint8_t reg)
{
    uint8_t data;
    int err = ppg_sensor_read_reg(&i2c, reg, &data);
    if (err)
    {
        LOG_ERR("Failed to read PPG sensor register 0x%02X", reg);
        return err;
    }

    return 0;
}

int ppg_write_reg(uint8_t reg, uint8_t data)
{
    int err = ppg_sensor_write_reg(&i2c, reg, data);
    if (err)
    {
        LOG_ERR("Failed to write PPG sensor register 0x%02X", reg);
        return err;
    }

    return 0;
}

static void ppg_reg_work_handler(struct k_work *work)
{
    int err;
    struct ppg_reg_work_t *ppg_reg_work = CONTAINER_OF(work, struct ppg_reg_work_t, reg_work);

    uint8_t final_reg_data;
    if (ppg_reg_work->read)
    {
        err = ppg_sensor_read_reg(&i2c, ppg_reg_work->reg, &final_reg_data);
        if (err)
        {
            LOG_ERR("Failed to read PPG sensor register 0x%02X", ppg_reg_work->reg);
            return;
        }

        err = tgm_service_send_read_ppg_reg_notify(final_reg_data);
    }
    else
    {
        err = ppg_sensor_write_reg(&i2c, ppg_reg_work->reg, ppg_reg_work->data);
        if (err)
        {
            LOG_ERR("Failed to write PPG sensor register 0x%02X", ppg_reg_work->reg);
            return;
        }
        err = ppg_sensor_read_reg(&i2c, ppg_reg_work->reg, &final_reg_data);
        if (err)
        {
            LOG_ERR("Failed to read PPG sensor register 0x%02X after writing", ppg_reg_work->reg);
            // Allow to continue since the write was successful
        }

        err = tgm_service_send_write_ppg_reg_notify(final_reg_data);
    }

    if (err)
    {
        LOG_ERR("Failed to send PPG register value notification");
        return;
    }

    return;
}

int ppg_set_led_pa(enum ppg_led_t led, uint8_t pa)
{
    int err;

    err = ppg_write_reg(MAXM86161_REG_LED1_PA + led, pa);
    if (err)
    {
        LOG_ERR("Failed to set PPG LED PA for LED %d", led);
    }

    return 0;
}
