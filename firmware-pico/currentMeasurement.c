#include "currentMeasurement.h"
#include "cfg.h"
#include "hardware/adc.h"

#define RING_BUFFER_SIZE 10

#define ADC_VREF_V              3.3
#define ADC_MAX_VALUE           4095
#define ADC_CONVERSION_FACTOR   (ADC_VREF_V / ADC_MAX_VALUE)
#define LEM_VOLTAGE_OFFSET_V    2.5
#define LEM_NOMINAL_RANGE_A     100.0
#define LEM_V_PER_NOMINAL_RANGE 0.625
#define LEM_A_PER_V             (LEM_NOMINAL_RANGE_A/LEM_V_PER_NOMINAL_RANGE)
#define LEM_OUTPUT_VOLTAGE_DIVIDER ((18.0+10.0)/18.0)



float current_A_ring_buffer[RING_BUFFER_SIZE];

void updateRingBuffer(float current_A){
    static int counter = 0;
    current_A_ring_buffer[counter] = current_A;

    counter++;
	if(counter >= RING_BUFFER_SIZE)
	{
		counter = 0;
	}
}

void CURRENTMEASURMENT_init(void)
{
  adc_init();
  adc_gpio_init(GPIO_LEM_REF_PIN_NUMBER);
  adc_gpio_init(GPIO_LEM_VALUE_PIN_NUMBER);
}

void CURRENTMEASURMENT_Handle_100ms(void)
{
    adc_select_input(GPIO_LEM_VALUE_ADC);
    uint16_t result_value = adc_read();
    // The sensor is connected to a voltage divider
    float voltage_value = result_value * ADC_CONVERSION_FACTOR * LEM_OUTPUT_VOLTAGE_DIVIDER;
    float current_A = (voltage_value-LEM_VOLTAGE_OFFSET_V) * LEM_A_PER_V;

    // TODO -> evaluate accuracy with LEM VREF
    adc_select_input(GPIO_LEM_REF_ADC);
    uint16_t result_ref = adc_read();
    float voltage_ref = result_ref * ADC_CONVERSION_FACTOR;

    updateRingBuffer(current_A);
}

void CURRENTMEASURMENT_GetCurrent_A(float* value)
{
    float sum_A = 0;
    for(int i = 0; i < RING_BUFFER_SIZE; i++)
    {
        sum_A += current_A_ring_buffer[i];
    }

    float average_A = sum_A / RING_BUFFER_SIZE;

    *value = average_A;
}
