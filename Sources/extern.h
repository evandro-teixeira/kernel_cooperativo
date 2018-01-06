/*
 * extern.h
 *
 *  Created on: 02/01/2018
 *      Author: evandro
 */

#ifndef SOURCES_EXTERN_H_
#define SOURCES_EXTERN_H_

#include "MKE06Z4.h"
#include <stdbool.h>

#include "common/common.h"
#include "common/flag.h"
#include "common/queue.h"

#include "drivers/acmp/acmp.h"
#include "drivers/adc/adc.h"
#include "drivers/bitband/bitband.h"
#include "drivers/bme/BME.h"
#include "drivers/can/mscan_api.h"
#include "drivers/can/mscan.h"
#include "drivers/crc/crc.h"
#include "drivers/ftm/ftm.h"
#include "drivers/gpio/gpio.h"
#include "drivers/ics/ics.h"
#include "drivers/iic/i2c.h"
#include "drivers/kbi/kbi.h"
#include "drivers/nvm/flash.h"
#include "drivers/pit/pit.h"
#include "drivers/PMC/pmc.h"
#include "drivers/pwt/pwt.h"
#include "drivers/rtc/rtc.h"
#include "drivers/sim/sim.h"
#include "drivers/spi/spi.h"
#include "drivers/uart/uart.h"
#include "drivers/wdog/wdog.h"

#include "kernel/kernel.h"

#include "app/app.h"

#endif /* SOURCES_EXTERN_H_ */
