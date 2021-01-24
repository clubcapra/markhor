#pragma once

#include "ctre/phoenix/cci/CCI.h"

extern "C" {
    CCIEXPORT void c_FeedEnable(int timeoutMs);
    CCIEXPORT bool c_GetEnableState(); 
}
