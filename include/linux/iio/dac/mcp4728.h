/*
 * MCP4728 DAC driver
 *
 * Copyright (C) 2016 Evgeny Boger <boger@contactless.ru>
 *
 * Licensed under the GPL-2 or later.
 */

#ifndef IIO_DAC_MCP4728_H_
#define IIO_DAC_MCP4728_H_

struct mcp4728_platform_data {
	u16 vref_mv;
};

#endif /* IIO_DAC_MCP4728_H_ */
