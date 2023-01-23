#include "error.h"
#include "cfg.h"
#include "currentMeasurement.h"
#include "hardware/gpio.h"
#include "slave.h"

// the first few seconds no error will be latched
#define ERROR_SKIP_TIME_ms 10000
#define HANDLE_TIME_ms 1000
volatile int startup_time_counter_ms = 0;

#define ERROR_TIME_OT_ms 2000
volatile int time_counter_OT_ms = 0;

volatile t_error_flags error_flags;




t_error_flags ERROR_getErrorState(void)
{
    return error_flags;
}

void ERROR_init(void)
{
    gpio_init(GPIO_RELAIS);
    gpio_set_dir(GPIO_RELAIS, GPIO_OUT);
    gpio_put(GPIO_RELAIS, 0);
}

void ERROR_Handle_1s(void)
{
    volatile float error_minVoltage_mV = SLAVE_Get_min_voltage_mV();
    volatile float error_maxVoltage_mV = SLAVE_Get_max_voltage_mV();
    volatile float error_minTemperature_C = SLAVE_Get_min_temperature_C();
    volatile float error_maxTemperature_C = SLAVE_Get_max_temperature_C();
    volatile float error_current_A = 0;
    CURRENTMEASURMENT_GetCurrent_A(&error_current_A);

    // warnings can be resetted
    error_flags.warning_SlaveTimeout = 0; // TODO WARNING_LIMIT_SlaveTimeout_s
    error_flags.warning_UV = error_minVoltage_mV < WARNING_UV_mV;
    error_flags.warning_OV = error_maxVoltage_mV > WARNING_OV_mV;
    error_flags.warning_UT = error_minTemperature_C < WARNING_UT_C;
    error_flags.warning_OT = error_maxTemperature_C > WARNING_OT_C;
    error_flags.warning_OC = fabs(error_current_A) > WARNING_OC_A;

    error_flags.warning_AnyWarning = 0x00;
    error_flags.warning_AnyWarning |= error_flags.warning_SlaveTimeout;
    error_flags.warning_AnyWarning |= error_flags.warning_UV;
    error_flags.warning_AnyWarning |= error_flags.warning_OV;
    error_flags.warning_AnyWarning |= error_flags.warning_UT;
    error_flags.warning_AnyWarning |= error_flags.warning_OT;
    error_flags.warning_AnyWarning |= error_flags.warning_OC;

    if (startup_time_counter_ms <= ERROR_SKIP_TIME_ms)
    {
        startup_time_counter_ms += HANDLE_TIME_ms;

        gpio_put(GPIO_RELAIS, 0);
    }else{
        // errors are not resetted

        // add some tolerance for the over temperature error handling
        int isOverTempearture = error_maxTemperature_C > ERROR_OT_C;
        if(isOverTempearture)
        {
            if(time_counter_OT_ms <= ERROR_TIME_OT_ms)
                time_counter_OT_ms += HANDLE_TIME_ms;
        }else{
            time_counter_OT_ms = 0;
        }

        error_flags.error_SlaveTimeout |= 0; // TODO ERROR_LIMIT_SlaveTimeout_s
        error_flags.error_UV |= error_minVoltage_mV < ERROR_UV_mV;
        error_flags.error_OV |= error_maxVoltage_mV > ERROR_OV_mV;
        error_flags.error_UT |= error_minTemperature_C < ERROR_UT_C;
        error_flags.error_OT |= (int)(time_counter_OT_ms >= ERROR_TIME_OT_ms);
        error_flags.error_OC |= fabs(error_current_A) > ERROR_OC_A;

        error_flags.error_AnyError = 0x00;
        error_flags.error_AnyError |= error_flags.error_SlaveTimeout;
        error_flags.error_AnyError |= error_flags.error_UV;
        error_flags.error_AnyError |= error_flags.error_OV;
        error_flags.error_AnyError |= error_flags.error_UT;
        error_flags.error_AnyError |= error_flags.error_OT;
        error_flags.error_AnyError |= error_flags.error_OC;

        if (error_flags.error_AnyError)
        {
            gpio_put(GPIO_RELAIS, 0);
        }else {
            gpio_put(GPIO_RELAIS, 1);
        }
    }
}
