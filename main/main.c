#include "main.h"
#include "as5600.h"
#include "as5600_config.h"
#include "gpio.h"
#include "i2c.h"
#include "stm32l4xx_hal.h"
#include "usart.h"
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

typedef struct {
    I2C_HandleTypeDef* i2c_handle;
    uint16_t i2c_address;
} as5600_bus_user_t;

as5600_err_t as5600_bus_write_data(void* user,
                                   uint8_t address,
                                   uint8_t const* data,
                                   size_t data_size)
{
    as5600_bus_user_t* bus_user = (as5600_bus_user_t*)user;
    HAL_I2C_Mem_Write(bus_user->i2c_handle,
                      bus_user->i2c_address,
                      address,
                      1,
                      (uint8_t*)data,
                      data_size,
                      100);

    return AS5600_ERR_OK;
}

as5600_err_t as5600_bus_initialize(void* user)
{
    as5600_bus_user_t* bus_user = (as5600_bus_user_t*)user;

    if (HAL_I2C_IsDeviceReady(bus_user->i2c_handle, bus_user->i2c_address, 3, 100) != HAL_OK) {
        return AS5600_ERR_FAIL;
    }

    return AS5600_ERR_OK;
}

as5600_err_t as5600_bus_read_data(void* user, uint8_t address, uint8_t* data, size_t data_size)
{
    as5600_bus_user_t* bus_user = (as5600_bus_user_t*)user;
    HAL_I2C_Mem_Read(bus_user->i2c_handle, bus_user->i2c_address, address, 1, data, data_size, 100);

    return AS5600_ERR_OK;
}

as5600_err_t as5600_gpio_write_pin(void* user, uint32_t pin, bool state)
{
    HAL_GPIO_WritePin(user, (uint16_t)pin, (GPIO_PinState)state);

    return AS5600_ERR_OK;
}

as5600_err_t as5600_init_chip(as5600_t* as5600, float32_t min_angle, float32_t max_angle)
{
    as5600_status_reg_t status;
    as5600_err_t err = as5600_get_status_reg(as5600, &status);
    if (err != AS5600_ERR_OK)
        return err;

    float32_t angle_range = (max_angle - min_angle);

    uint16_t min_raw = (uint16_t)(min_angle / angle_range * 4095.0F);
    uint16_t max_raw = (uint16_t)(max_angle / angle_range * 4095.0F);

    as5600_zpos_reg_t zpos = {.zpos = min_raw & 0x0FFF};
    err = as5600_set_zpos_reg(as5600, &zpos);
    if (err != AS5600_ERR_OK)
        return err;

    as5600_mpos_reg_t mpos = {.mpos = max_raw & 0x0FFF};
    err = as5600_set_mpos_reg(as5600, &mpos);
    if (err != AS5600_ERR_OK)
        return err;

    as5600_conf_reg_t conf = {.wd = AS5600_WATCHDOG_OFF,
                              .fth = AS5600_SLOW_FILTER_X16,
                              .sf = AS5600_SLOW_FILTER_X16,
                              .pwmf = AS5600_PWM_FREQUENCY_115HZ,
                              .outs = AS5600_FAST_FILTER_THRESH_SLOW,
                              .hyst = AS5600_HYSTERESIS_OFF,
                              .pm = AS5600_POWER_MODE_NOM};
    err = as5600_set_conf_reg(as5600, &conf);
    if (err != AS5600_ERR_OK)
        return err;

    as5600_zmco_reg_t zmco;
    err = as5600_get_zmco_reg(as5600, &zmco);
    if (err != AS5600_ERR_OK)
        return err;

    return AS5600_ERR_OK;
}

int main(void)
{
    HAL_Init();
    SystemClock_Config();

    MX_I2C3_Init();
    MX_GPIO_Init();
    MX_USART2_UART_Init();

    HAL_Delay(500);

    as5600_t as5600 = {};

    as5600_err_t err = as5600_initialize(
        &as5600,
        &(as5600_config_t){.dir_pin = GPIO_PIN_2, .min_angle = 0.0F, .max_angle = 180.0F},
        &(as5600_interface_t){
            .gpio_user = GPIOC,
            .gpio_write_pin = as5600_gpio_write_pin,
            .bus_user =
                &(as5600_bus_user_t){.i2c_address = AS5600_DEV_ADDRESS << 1, .i2c_handle = &hi2c3},
            .bus_initialize = as5600_bus_initialize,
            .bus_write_data = as5600_bus_write_data,
            .bus_read_data = as5600_bus_read_data,
        });

    printf("Error: %d\n\r", err);

    as5600_init_chip(&as5600, 0.0F, 180.0F);

    as5600_set_direction(&as5600, AS5600_DIRECTION_CW);

    float32_t scaled;

    while (1) {
        as5600_get_angle_data_scaled_bus(&as5600, &scaled);
        printf("scaled: %f\n\r", scaled);
    }
}
