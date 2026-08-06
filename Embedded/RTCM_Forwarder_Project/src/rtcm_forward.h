// Header for RTCM forwarder
#pragma once

#include "ch.h"
#include "hal.h"
#include "lwip/sockets.h"

// Function to start the RTCM forwarder thread
void rtcm_forward_start(BaseSequentialStream *dbg,
                        BaseAsynchronousChannel *uart_out);
