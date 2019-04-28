/* SPDX-License-Identifier: (GPL-2.0-or-later WITH Linux-syscall-note) */
/*
 * FSK Netlink interface
 *
 * Copyright (c) 2018-2019 Andreas Färber
 */
#ifndef __LINUX_NLFSK_H
#define __LINUX_NLFSK_H

#define NLFSK_GENL_NAME "nlfsk"

enum nlfsk_attrs {
	NLFSK_ATTR_UNSPEC = 0,

	NLFSK_ATTR_IFINDEX,

	NLFSK_ATTR_FREQ,

	__NLFSK_ATTR_AFTER_LAST,
	NLFSK_ATTR_MAX = __NLFSK_ATTR_AFTER_LAST - 1,
};

enum nlfsk_commands {
	NLFSK_CMD_UNSPEC = 0,

	NLFSK_CMD_GET_FREQ,
	NLFSK_CMD_SET_FREQ,

	__NLFSK_CMD_AFTER_LAST,
	NLFSK_CMD_MAX = __NLFSK_CMD_AFTER_LAST - 1,
};

#endif
