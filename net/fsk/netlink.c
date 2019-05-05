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
	struct cfgfsk_registered_phy *rphy = info->user_ptr[0];
	struct sk_buff *msg;
	void *hdr;
	u32 val;
	int ret;

	if (!rphy->ops->get_freq)
		return -EOPNOTSUPP;

	msg = nlmsg_new(NLMSG_DEFAULT_SIZE, GFP_KERNEL);
	if (!msg) {
		return -ENOMEM;
	}

	hdr = genlmsg_put(msg, info->snd_portid, info->snd_seq, &nlfsk_fam, 0, NLFSK_CMD_GET_FREQ);
	nla_put_u32(msg, NLFSK_ATTR_IFINDEX, rphy->fsk_phy.netdev->ifindex);

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

static int nlfsk_cmd_set_freq(struct sk_buff *skb, struct genl_info *info)
{
	struct cfgfsk_registered_phy *rphy = info->user_ptr[0];
	u32 val;

	if (!info->attrs[NLFSK_ATTR_FREQ])
		return -EINVAL;

	if (!rphy->ops->set_freq)
		return -EOPNOTSUPP;

	val = nla_get_u32(info->attrs[NLFSK_ATTR_FREQ]);
	return rphy->ops->set_freq(&rphy->fsk_phy, val);
}

static int nlfsk_cmd_get_freq_dev(struct sk_buff *skb, struct genl_info *info)
{
	struct cfgfsk_registered_phy *rphy = info->user_ptr[0];
	struct sk_buff *msg;
	void *hdr;
	u32 val;
	int ret;

	if (!rphy->ops->get_freq_dev)
		return -EOPNOTSUPP;

	msg = nlmsg_new(NLMSG_DEFAULT_SIZE, GFP_KERNEL);
	if (!msg) {
		return -ENOMEM;
	}

	hdr = genlmsg_put(msg, info->snd_portid, info->snd_seq, &nlfsk_fam, 0, NLFSK_CMD_GET_FREQ_DEV);
	nla_put_u32(msg, NLFSK_ATTR_IFINDEX, rphy->fsk_phy.netdev->ifindex);

	ret = rphy->ops->get_freq_dev(&rphy->fsk_phy, &val);
	if (ret) {
		genlmsg_cancel(msg, hdr);
		nlmsg_free(msg);
		return -ENOBUFS;
	}

	nla_put_u32(msg, NLFSK_ATTR_FREQ, val);

	genlmsg_end(msg, hdr);

	return genlmsg_reply(msg, info);
}

static int nlfsk_cmd_set_freq_dev(struct sk_buff *skb, struct genl_info *info)
{
	struct cfgfsk_registered_phy *rphy = info->user_ptr[0];
	u32 val;

	if (!info->attrs[NLFSK_ATTR_FREQ])
		return -EINVAL;

	if (!rphy->ops->set_freq_dev)
		return -EOPNOTSUPP;

	val = nla_get_u32(info->attrs[NLFSK_ATTR_FREQ]);
	return rphy->ops->set_freq_dev(&rphy->fsk_phy, val);
}

static int nlfsk_cmd_get_tx_power(struct sk_buff *skb, struct genl_info *info)
{
	struct cfgfsk_registered_phy *rphy = info->user_ptr[0];
	struct sk_buff *msg;
	void *hdr;
	s32 val;
	int ret;

	if (!rphy->ops->get_tx_power)
		return -EOPNOTSUPP;

	msg = nlmsg_new(NLMSG_DEFAULT_SIZE, GFP_KERNEL);
	if (!msg) {
		return -ENOMEM;
	}

	hdr = genlmsg_put(msg, info->snd_portid, info->snd_seq, &nlfsk_fam, 0, NLFSK_CMD_GET_TX_POWER);
	nla_put_u32(msg, NLFSK_ATTR_IFINDEX, rphy->fsk_phy.netdev->ifindex);

	ret = rphy->ops->get_tx_power(&rphy->fsk_phy, &val);
	if (ret) {
		genlmsg_cancel(msg, hdr);
		nlmsg_free(msg);
		return -ENOBUFS;
	}

	nla_put_s32(msg, NLFSK_ATTR_TX_POWER, val);

	genlmsg_end(msg, hdr);

	return genlmsg_reply(msg, info);
}

static int nlfsk_cmd_set_tx_power(struct sk_buff *skb, struct genl_info *info)
{
	struct cfgfsk_registered_phy *rphy = info->user_ptr[0];
	s32 val;

	if (!info->attrs[NLFSK_ATTR_TX_POWER])
		return -EINVAL;

	if (!rphy->ops->set_tx_power)
		return -EOPNOTSUPP;

	val = nla_get_s32(info->attrs[NLFSK_ATTR_TX_POWER]);
	return rphy->ops->set_tx_power(&rphy->fsk_phy, val);
}

static const struct nla_policy nlfsk_policy[NLFSK_ATTR_MAX + 1] = {
	[NLFSK_ATTR_IFINDEX]	= { .type = NLA_U32 },
	[NLFSK_ATTR_FREQ]	= { .type = NLA_U32 },
	[NLFSK_ATTR_TX_POWER]	= { .type = NLA_S32 },
};

#define NLFSK_FLAG_NEED_PHY	BIT(0)

static int nlfsk_pre_doit(const struct genl_ops *ops, struct sk_buff *skb,
			  struct genl_info *info)
{
	struct nlattr **attrs = info->attrs;

	if (ops->internal_flags & NLFSK_FLAG_NEED_PHY) {
		struct cfgfsk_registered_phy *rphy;
		int ifindex = -1;

		if (attrs[NLFSK_ATTR_IFINDEX])
			ifindex = nla_get_u32(attrs[NLFSK_ATTR_IFINDEX]);

		rphy = cfgfsk_get_phy_by_ifindex(ifindex);
		if (!rphy)
			return -ENODEV;

		info->user_ptr[0] = rphy;
	}

	return 0;
}

static const struct genl_ops nlfsk_ops[] = {
	{
		.cmd = NLFSK_CMD_GET_FREQ,
		.doit = nlfsk_cmd_get_freq,
		.flags = 0,
		.internal_flags = NLFSK_FLAG_NEED_PHY,
	},
	{
		.cmd = NLFSK_CMD_SET_FREQ,
		.doit = nlfsk_cmd_set_freq,
		.flags = GENL_ADMIN_PERM,
		.internal_flags = NLFSK_FLAG_NEED_PHY,
	},
	{
		.cmd = NLFSK_CMD_GET_FREQ_DEV,
		.doit = nlfsk_cmd_get_freq_dev,
		.flags = 0,
		.internal_flags = NLFSK_FLAG_NEED_PHY,
	},
	{
		.cmd = NLFSK_CMD_SET_FREQ_DEV,
		.doit = nlfsk_cmd_set_freq_dev,
		.flags = GENL_ADMIN_PERM,
		.internal_flags = NLFSK_FLAG_NEED_PHY,
	},
	{
		.cmd = NLFSK_CMD_GET_TX_POWER,
		.doit = nlfsk_cmd_get_tx_power,
		.flags = 0,
		.internal_flags = NLFSK_FLAG_NEED_PHY,
	},
	{
		.cmd = NLFSK_CMD_SET_TX_POWER,
		.doit = nlfsk_cmd_set_tx_power,
		.flags = GENL_ADMIN_PERM,
		.internal_flags = NLFSK_FLAG_NEED_PHY,
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
	.pre_doit = nlfsk_pre_doit,
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
