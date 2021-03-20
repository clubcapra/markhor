#pragma once

#include "ctre/phoenix/cci/CCI.h"
#include <stdint.h>
#include "ctre/phoenix/ErrorCode.h"

extern "C" {
    CCIEXPORT int32_t c_SetCANInterface(const char * interface);
    CCIEXPORT ctre::phoenix::ErrorCode c_DestroyAll();
    CCIEXPORT ctre::phoenix::ErrorCode c_StartAll();
}
