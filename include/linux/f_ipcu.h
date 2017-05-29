#ifndef __F_IPCU_H
#define __F_IPCU_H

struct ipcu_mssg {
	/*
	 * TX : bit mask of destinations
	 * RX : bit mask of source
	 */
	u32 mask;
	/* Packet */
	u32 data[9];
};

struct ipcu_client {
	/* CPU i/f this client runs on */
	int iface;
	/* If client need to only receive messages */
	bool ro;
};

#endif /* __F_IPCU_H */
