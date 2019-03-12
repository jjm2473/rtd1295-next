// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright (c) 2018-2019 Andreas Färber
 */

#include <linux/if_arp.h>
#include <linux/module.h>
#include <linux/nlfsk.h>
#include <net/cfgfsk.h>
#include <net/genetlink.h>
#include <net/sock.h>

#include "cfg.h"

enum nlfsk_multicast_groups {
	NLFSK_MCGRP_CONFIG = 0,
};

static const struct genl_multicast_group nlfsk_mcgrps[] = {
	[NLFSK_MCGRP_CONFIG] = { .name = "config" },
};

static struct genl_family nlfsk_fam;

static int nlfsk_cmd_get_freq(struct sk_buff *skb, struct genl_info *info)
{
	struct nlattr **attrs = genl_family_attrbuf(&nlfsk_fam);
	bool have_ifindex = attrs[NLFSK_ATTR_IFINDEX];
	struct sk_buff *msg;
	struct cfgfsk_registered_phy *rphy;
	void *hdr;
	u32 val;
	int ifindex = -1;
	int ret;

	if (have_ifindex)
		ifindex = nla_get_u32(attrs[NLFSK_ATTR_IFINDEX]);

	rphy = cfgfsk_get_phy_by_ifindex(ifindex);
	if (!rphy)
		return -ENOBUFS;

	msg = nlmsg_new(NLMSG_DEFAULT_SIZE, GFP_KERNEL);
	if (!msg) {
		return -ENOMEM;
	}

	hdr = genlmsg_put(msg, info->snd_portid, info->snd_seq, &nlfsk_fam, 0, NLFSK_CMD_GET_FREQ);
	nla_put_u32(msg, NLFSK_ATTR_IFINDEX, ifindex);

	if (!rphy->ops->get_freq) {
		genlmsg_cancel(msg, hdr);
		nlmsg_free(msg);
		return -ENOBUFS;
	}

	ret = rphy->ops->get_freq(&rphy->fsk_phy, &val);
	if (ret) {
		genlmsg_cancel(msg, hdr);
		nlmsg_free(msg);
		return -ENOBUFS;
	}

	nla_put_u32(msg, NLFSK_ATTR_FREQ, val);

	genlmsg_end(msg, hdr);

	return genlmsg_reply(msg, info);
}

static const struct nla_policy nlfsk_policy[NLFSK_ATTR_MAX + 1] = {
	[NLFSK_ATTR_IFINDEX] = { .type = NLA_U32 },
	[NLFSK_ATTR_FREQ] = { .type = NLA_U32 },
};

static const struct genl_ops nlfsk_ops[] = {
	{
		.cmd = NLFSK_CMD_GET_FREQ,
		.doit = nlfsk_cmd_get_freq,
		.flags = 0/*GENL_ADMIN_PERM*/,
		.internal_flags = 0,
	},
};

static struct genl_family nlfsk_fam __ro_after_init = {
	.name = NLFSK_GENL_NAME,
	.hdrsize = 0,
	.version = 1,
	.maxattr = NLFSK_ATTR_MAX,
	.netnsok = true,
	.module = THIS_MODULE,
	.policy = nlfsk_policy,
	.ops = nlfsk_ops,
	.n_ops = ARRAY_SIZE(nlfsk_ops),
	.mcgrps = nlfsk_mcgrps,
	.n_mcgrps = ARRAY_SIZE(nlfsk_mcgrps),
};

int __init nlfsk_init(void)
{
	int ret;

	ret = genl_register_family(&nlfsk_fam);
	if (ret)
		return ret;

	return 0;
}

void __exit nlfsk_exit(void)
{
	genl_unregister_family(&nlfsk_fam);
}
