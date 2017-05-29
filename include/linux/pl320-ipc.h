/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __MAILBOX_H
#define __MAILBOX_H

enum xfer_result {
	XFER_OK = 0,
	XFER_ERR,
};

typedef unsigned request_token_t;

#ifdef CONFIG_DEBUG_MBOX
#define mbox_dbg		printk
#else
#define mbox_dbg(fmt, args...)	do {} while (0)
#endif

#endif /* __MAILBOX_H */

