// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright (c) 2018-2019 Andreas Färber
 */

#include <linux/if_arp.h>
#include <linux/module.h>
#include <linux/nllora.h>
#include <net/cfglora.h>
#include <net/genetlink.h>
#include <net/sock.h>

#include "cfg.h"

enum nllora_multicast_groups {
	NLLORA_MCGRP_CONFIG = 0,
};

static const struct genl_multicast_group nllora_mcgrps[] = {
	[NLLORA_MCGRP_CONFIG] = { .name = "config" },
};

static struct genl_family nllora_fam;

static int nllora_cmd_get_freq(struct sk_buff *skb, struct genl_info *info)
{
	struct cfglora_registered_phy *rphy = info->user_ptr[0];
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

	hdr = genlmsg_put(msg, info->snd_portid, info->snd_seq, &nllora_fam, 0, NLLORA_CMD_GET_FREQ);
	nla_put_u32(msg, NLLORA_ATTR_IFINDEX, rphy->lora_phy.netdev->ifindex);

	ret = rphy->ops->get_freq(&rphy->lora_phy, &val);
	if (ret) {
		genlmsg_cancel(msg, hdr);
		nlmsg_free(msg);
		return -ENOBUFS;
	}

	nla_put_u32(msg, NLLORA_ATTR_FREQ, val);

	genlmsg_end(msg, hdr);

	return genlmsg_reply(msg, info);
}

static int nllora_cmd_set_freq(struct sk_buff *skb, struct genl_info *info)
{
	struct cfglora_registered_phy *rphy = info->user_ptr[0];
	u32 val;

	if (!info->attrs[NLLORA_ATTR_FREQ])
		return -EINVAL;

	if (!rphy->ops->set_freq)
		return -EOPNOTSUPP;

	val = nla_get_u32(info->attrs[NLLORA_ATTR_FREQ]);
	return rphy->ops->set_freq(&rphy->lora_phy, val);
}

static int nllora_cmd_get_tx_power(struct sk_buff *skb, struct genl_info *info)
{
	struct cfglora_registered_phy *rphy = info->user_ptr[0];
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

	hdr = genlmsg_put(msg, info->snd_portid, info->snd_seq, &nllora_fam, 0, NLLORA_CMD_GET_TX_POWER);
	nla_put_u32(msg, NLLORA_ATTR_IFINDEX, rphy->lora_phy.netdev->ifindex);

	ret = rphy->ops->get_tx_power(&rphy->lora_phy, &val);
	if (ret) {
		genlmsg_cancel(msg, hdr);
		nlmsg_free(msg);
		return -ENOBUFS;
	}

	nla_put_s32(msg, NLLORA_ATTR_TX_POWER, val);

	genlmsg_end(msg, hdr);

	return genlmsg_reply(msg, info);
}

static int nllora_cmd_set_tx_power(struct sk_buff *skb, struct genl_info *info)
{
	struct cfglora_registered_phy *rphy = info->user_ptr[0];
	s32 val;

	if (!info->attrs[NLLORA_ATTR_TX_POWER])
		return -EINVAL;

	if (!rphy->ops->set_tx_power)
		return -EOPNOTSUPP;

	val = nla_get_s32(info->attrs[NLLORA_ATTR_TX_POWER]);
	return rphy->ops->set_tx_power(&rphy->lora_phy, val);
}

static const struct nla_policy nllora_policy[NLLORA_ATTR_MAX + 1] = {
	[NLLORA_ATTR_IFINDEX] = { .type = NLA_U32 },
	[NLLORA_ATTR_FREQ] = { .type = NLA_U32 },
	[NLLORA_ATTR_TX_POWER] = { .type = NLA_S32 },
};

#define NLLORA_FLAG_NEED_PHY	BIT(0)

static int nllora_pre_doit(const struct genl_ops *ops, struct sk_buff *skb,
			   struct genl_info *info)
{
	struct nlattr **attrs = info->attrs;

	if (ops->internal_flags & NLLORA_FLAG_NEED_PHY) {
		struct cfglora_registered_phy *rphy;
		int ifindex = -1;

		if (attrs[NLLORA_ATTR_IFINDEX])
			ifindex = nla_get_u32(attrs[NLLORA_ATTR_IFINDEX]);

		rphy = cfglora_get_phy_by_ifindex(ifindex);
		if (!rphy)
			return -ENODEV;

		info->user_ptr[0] = rphy;
	}

	return 0;
}

static const struct genl_ops nllora_ops[] = {
	{
		.cmd = NLLORA_CMD_GET_FREQ,
		.doit = nllora_cmd_get_freq,
		.flags = 0,
		.internal_flags = NLLORA_FLAG_NEED_PHY,
	},
	{
		.cmd = NLLORA_CMD_SET_FREQ,
		.doit = nllora_cmd_set_freq,
		.flags = GENL_ADMIN_PERM,
		.internal_flags = NLLORA_FLAG_NEED_PHY,
	},
	{
		.cmd = NLLORA_CMD_GET_TX_POWER,
		.doit = nllora_cmd_get_tx_power,
		.flags = 0,
		.internal_flags = NLLORA_FLAG_NEED_PHY,
	},
	{
		.cmd = NLLORA_CMD_SET_TX_POWER,
		.doit = nllora_cmd_set_tx_power,
		.flags = GENL_ADMIN_PERM,
		.internal_flags = NLLORA_FLAG_NEED_PHY,
	},
};

static struct genl_family nllora_fam __ro_after_init = {
	.name = NLLORA_GENL_NAME,
	.hdrsize = 0,
	.version = 1,
	.maxattr = NLLORA_ATTR_MAX,
	.netnsok = true,
	.module = THIS_MODULE,
	.policy = nllora_policy,
	.ops = nllora_ops,
	.n_ops = ARRAY_SIZE(nllora_ops),
	.pre_doit = nllora_pre_doit,
	.mcgrps = nllora_mcgrps,
	.n_mcgrps = ARRAY_SIZE(nllora_mcgrps),
};

int __init nllora_init(void)
{
	int ret;

	ret = genl_register_family(&nllora_fam);
	if (ret)
		return ret;

	return 0;
}

void __exit nllora_exit(void)
{
	genl_unregister_family(&nllora_fam);
}
