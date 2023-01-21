#ifndef ERROR_H
#define ERROR_H

#include "pico.h"

typedef struct {
  // combining flags
  uint8_t warning_AnyWarning; // error flag, to combine all warning cases
  uint8_t error_AnyError; // error flag, to combine all error cases
  // warnings
  uint8_t warning_SlaveTimeout;
  uint8_t warning_UV;
  uint8_t warning_OV;
  uint8_t warning_UT;
  uint8_t warning_OT;
  uint8_t warning_OC;
  // errors
  uint8_t error_SlaveTimeout;
  uint8_t error_UV; // undervoltage
  uint8_t error_OV; // overvoltage
  uint8_t error_UT; // undertemperature
  uint8_t error_OT; // overtemperature
  uint8_t error_OC; // overcurrent
} t_error_flags;

extern t_error_flags ERROR_getErrorState(void);
extern void ERROR_init(void);
extern void ERROR_Handle_1s(void);

#endif
