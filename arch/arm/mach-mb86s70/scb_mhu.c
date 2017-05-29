
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/list.h>
#include <linux/mutex.h>
#include <linux/module.h>
#include <linux/mailbox.h>
#include <linux/spinlock.h>
#include <linux/of_irq.h>
#include <linux/of_device.h>
#include <linux/interrupt.h>
#include <linux/scb_mhu_api.h>
#include <linux/mailbox_client.h>

#include <linux/platform_data/mb86s70-iomap.h>

#define INTR_STAT_OFS	0x0
#define INTR_SET_OFS	0x8
#define INTR_CLR_OFS	0x10

#define LPNONSEC	0
#define HPNONSEC	1
#define SECURE		2

int __initdata shm_offset = 0x800;

static void __iomem *cmd_from_scb = MB86S70_SHM_FROM_SCB_VIRT;
static void __iomem *rsp_from_scb = MB86S70_SHM_FROM_SCB_VIRT + 0x100;
static void __iomem *cmd_to_scb = MB86S70_SHM_FROM_SCB_VIRT + 0x200;
static void __iomem *rsp_to_scb = MB86S70_SHM_FROM_SCB_VIRT + 0x300;
static mhu_handler_t handler[MHU_NUM_CMDS];
static DEFINE_SPINLOCK(fsm_lock);
static LIST_HEAD(pending_xfers);
static LIST_HEAD(free_xfers);
static struct ipc_client mhu_cl;
static struct completion fsm_rsp;
static void *mhu_chan;
static int do_xfer(void);

static DEFINE_MUTEX(mhu_mutex);

static enum {
	MHU_PARK = 0,
	MHU_WRR, /* Waiting to get Remote's Reply */
	MHU_WRL, /* Waiting to send Reply */
	MHU_WRRL, /* WAIT_Ra && WAIT_Rb */
	MHU_INVLD,
} fsm_state;

enum fsm_event {
	EV_LC = 0, /* Local sent a command */
	EV_RC, /* Remote sent a command */
	EV_RR, /* Remote sent a reply */
	EV_LR, /* Local sent a reply */
};

static int mhu_fsm[4][4] = {
	[MHU_PARK] = {
		[EV_LC] = MHU_WRR,
		[EV_RC] = MHU_WRL,
		[EV_RR] = MHU_INVLD,
		[EV_LR] = MHU_INVLD,
	},
	[MHU_WRR] = {
		[EV_LC] = MHU_INVLD,
		[EV_RC] = MHU_WRRL,
		[EV_RR] = MHU_PARK,
		[EV_LR] = MHU_INVLD,
	},
	[MHU_WRL] = {
		[EV_LC] = MHU_WRRL,
		[EV_RC] = MHU_INVLD,
		[EV_RR] = MHU_INVLD,
		[EV_LR] = MHU_PARK,
	},
	[MHU_WRRL] = {
		[EV_LC] = MHU_INVLD,
		[EV_RC] = MHU_INVLD,
		[EV_RR] = MHU_WRL,
		[EV_LR] = MHU_WRR,
	},
};

struct mhu_xfer {
	int code;
	int len;
	void *buf;
	struct completion *c;
	struct list_head node;
};
static struct mhu_xfer *ax; /* stages of xfer */

int skip_mhu = 0;
EXPORT_SYMBOL_GPL(skip_mhu);

static int __init skip_mhu_param(char *str)
{
	skip_mhu = 1;
	return 1;
}
__setup("skipmhu", skip_mhu_param);

static int __init set_shm_offset(char *str)
{
	get_option(&str, &shm_offset);

	if (shm_offset < 0)
		shm_offset = 0;

	cmd_from_scb += shm_offset;
	cmd_to_scb += shm_offset;
	rsp_from_scb += shm_offset;
	rsp_to_scb += shm_offset;

	return 1;
}
__setup("shm_offset=", set_shm_offset);

static int mhu_alloc_xfers(int n, struct list_head *list)
{
	struct mhu_xfer *x = kzalloc(n * sizeof(struct mhu_xfer), GFP_ATOMIC);
	int i;

	if (!x)
		return -ENOMEM;
	for (i = 0; i < n; i++)
		list_add(&x[i].node, &free_xfers);
	return 0;
}

static void got_data(u32 code)
{
	unsigned long flags;
	struct completion *c = NULL;
	mhu_handler_t hndlr = NULL;
	int ev;

	if (code & RESP_BIT)
		ev = EV_RR;
	else
		ev = EV_RC;

	spin_lock_irqsave(&fsm_lock, flags);
	if (mhu_fsm[fsm_state][ev] == MHU_INVLD) {
		spin_unlock_irqrestore(&fsm_lock, flags);
		pr_err("State-%d EV-%d FSM Broken!\n", fsm_state, ev);
		return;
	}
	fsm_state = mhu_fsm[fsm_state][ev];

	if (code & RESP_BIT) {
		c = ax->c;
		memcpy_fromio(ax->buf, rsp_from_scb, ax->len);
		list_move(&ax->node, &free_xfers);
		ax = NULL;
		if (c)
			complete(c);
	} else {
		/* Find and dispatch relevant registered handler */
		if (code < MHU_NUM_CMDS)
			hndlr = handler[code];
		if (hndlr)
			hndlr(code, cmd_from_scb);
		else
			pr_err("No handler for CMD_%u\n", code);
	}
	spin_unlock_irqrestore(&fsm_lock, flags);
}

static void mhu_recv(void *data)
{
	if ((u32)data & RESP_BIT) {
		/* Now that we got a reply to last TX, that
		 * must mean the last TX was successful */
		ipc_client_txdone(mhu_chan, XFER_OK);

		ax->code = (u32)data; /* Save response */
		complete(&fsm_rsp);
	} else
		got_data((u32)data);
}

static int do_xfer(void)
{
	struct mhu_xfer *x;
	unsigned long flags;
	int ev;
	int code;

	if (skip_mhu) {
		WARN_ON(1);
		return 0;
	}

	spin_lock_irqsave(&fsm_lock, flags);
	if (list_empty(&pending_xfers)) {
		void *_ch = NULL;
		int cmd;

		for (cmd = 0; cmd < MHU_NUM_CMDS && !handler[cmd]; cmd++)
			;
		/* Don't free channel if any user is listening */
		if (cmd != MHU_NUM_CMDS) {
			spin_unlock_irqrestore(&fsm_lock, flags);
			return 0;
		}

		if (ax != NULL) {
			spin_unlock_irqrestore(&fsm_lock, flags);
			return 1;
		}

		if (fsm_state == MHU_PARK) {
			_ch = mhu_chan;
			mhu_chan = NULL;
		}

		spin_unlock_irqrestore(&fsm_lock, flags);

		if (_ch)
			ipc_free_channel(_ch);

		return 0;
	}

	x = list_first_entry(&pending_xfers, struct mhu_xfer, node);
	code = x->code;

	ev = code & RESP_BIT ? EV_LR : EV_LC;
	if (mhu_fsm[fsm_state][ev] == MHU_INVLD) {
		spin_unlock_irqrestore(&fsm_lock, flags);
		mbox_dbg("%s:%d\n", __func__, __LINE__);
		return 1;
	}
	list_del_init(&x->node);

	/* Layout the SHM */
	if (code & RESP_BIT)
		memcpy_toio(rsp_to_scb, x->buf, x->len);
	else
		memcpy_toio(cmd_to_scb, x->buf, x->len);

	if (ev == EV_LC)
		ax = x;
	else
		list_move(&x->node, &free_xfers);
	fsm_state = mhu_fsm[fsm_state][ev];

	spin_unlock_irqrestore(&fsm_lock, flags);

	/* Prefer mailbox API */
	if ((!mhu_chan) && (!irqs_disabled())) {
		mhu_cl.tx_block = true;
		mhu_cl.knows_txdone = true;
		mhu_cl.rxcb = mhu_recv;
		mhu_cl.chan_name = "f_mhu:HP_NonSec";
		mhu_chan = ipc_request_channel(&mhu_cl);
	}

	if (mhu_chan && (!irqs_disabled())) {
		int ret;

		init_completion(&fsm_rsp);
		/* Send via generic api */
		ret = ipc_send_message(mhu_chan, (void *)code);
		if (!ret) {
			pr_err("%s:%d CMD_%d Send Failed\n",
					__func__, __LINE__, code);
			BUG();
		}
		if (!(code & RESP_BIT)) {
			ret = wait_for_completion_timeout(&fsm_rsp,
						msecs_to_jiffies(1000));
			if (!ret) {
				pr_err("%s:%d CMD_%d Got No Reply\n",
					__func__, __LINE__, code);
				BUG();
			}
			got_data(ax->code);
		}
	} else {
		void __iomem *tx_reg = MB86S70_MHU_VIRT + 0x120; /* HP-NonSec */
		void __iomem *rx_reg = MB86S70_MHU_VIRT + 0x20; /* HP-NonSec */
		u32 val, count;

		/* Send via early-boot api */
		val = __raw_readl(tx_reg + INTR_STAT_OFS);
		if (val) {
			pr_err("Last CMD not yet read by SCB\n");
			__raw_writel(val, tx_reg + INTR_CLR_OFS);
		}

		__raw_writel(x->code, tx_reg + INTR_SET_OFS);

		/* Wait until this message is read */
		count = 0x1000000;
		do {
			cpu_relax();
			val = __raw_readl(tx_reg + INTR_STAT_OFS);
		} while (--count && val);
		if (val)
			pr_err("%s:%d SCB not listening!\n",
				__func__, __LINE__);

		if (!ax) {
			/* A quick poll for pending remote cmd */
			val = __raw_readl(rx_reg + INTR_STAT_OFS);
			if (val) {
				got_data(val);
				__raw_writel(val, rx_reg + INTR_CLR_OFS);
			}
		} else {
			do {
retry:
				/* Wait until we get reply */
				count = 0x1000000;
				do {
					cpu_relax();
					val = __raw_readl(rx_reg + INTR_STAT_OFS);
				} while (--count && !val);

				if (!val) {
					pr_err("%s:%d SCB didn't reply\n",
								__func__, __LINE__);
					goto retry;
				} else {
					got_data(val);
					__raw_writel(val, rx_reg + INTR_CLR_OFS);
				}
			} while(!(val & RESP_BIT));
		}
		if (list_empty(&pending_xfers))
			return 0;
	}

	return do_xfer();
}

int mhu_hndlr_set(u32 cmd, mhu_handler_t hndlr)
{
	unsigned long flags;
	int ret = -EINVAL;

	mutex_lock(&mhu_mutex);

	spin_lock_irqsave(&fsm_lock, flags);
	if (cmd < MHU_NUM_CMDS && !handler[cmd]) {
		ret = 0;
		handler[cmd] = hndlr;
	}
	spin_unlock_irqrestore(&fsm_lock, flags);

	if (!mhu_chan) {
		void *_ch;
		mhu_cl.tx_block = true;
		mhu_cl.knows_txdone = true;
		mhu_cl.rxcb = mhu_recv;
		mhu_cl.chan_name = "f_mhu:HP_NonSec";
		_ch = ipc_request_channel(&mhu_cl);
		if (_ch)
			mhu_chan = _ch;
	}

	mutex_unlock(&mhu_mutex);

	return ret;
}
EXPORT_SYMBOL_GPL(mhu_hndlr_set);

void mhu_hndlr_clr(u32 cmd, mhu_handler_t hndlr)
{
	unsigned long flags;

	mutex_lock(&mhu_mutex);

	spin_lock_irqsave(&fsm_lock, flags);

	if (cmd < MHU_NUM_CMDS && handler[cmd] == hndlr)
		handler[cmd] = NULL;

	if (list_empty(&pending_xfers)) {
		void *_ch = NULL;

		for (cmd = 0; cmd < MHU_NUM_CMDS && !handler[cmd]; cmd++)
			;
		/* Don't free channel if any user is listening */
		if (cmd != MHU_NUM_CMDS) {
			spin_unlock_irqrestore(&fsm_lock, flags);
			mutex_unlock(&mhu_mutex);
			return;
		}

		if (fsm_state == MHU_PARK) {
			_ch = mhu_chan;
			mhu_chan = NULL;
		}

		spin_unlock_irqrestore(&fsm_lock, flags);

		if (_ch)
			ipc_free_channel(_ch);

		mutex_unlock(&mhu_mutex);

		return;
	}
	spin_unlock_irqrestore(&fsm_lock, flags);

	mutex_unlock(&mhu_mutex);

}
EXPORT_SYMBOL_GPL(mhu_hndlr_clr);

int mhu_send_packet(int code, void *buf, int len, struct completion *c)
{
	struct mhu_xfer *x;
	unsigned long flags;
	int ret;

	if (code & ~0xff) {
		WARN_ON(1);
		return -EINVAL;
	}

	mutex_lock(&mhu_mutex);
	if ((code & RESP_BIT) &&
		fsm_state != MHU_WRRL && fsm_state != MHU_WRL) {
		WARN_ON(1);
		mutex_unlock(&mhu_mutex);
		return -EINVAL;
	}

	spin_lock_irqsave(&fsm_lock, flags);

	if (list_empty(&free_xfers) && mhu_alloc_xfers(5, &free_xfers)) {
		spin_unlock_irqrestore(&fsm_lock, flags);
		mbox_dbg("%s:%d OOM\n", __func__, __LINE__);
		mutex_unlock(&mhu_mutex);
		return -EAGAIN;
	}

	x = list_first_entry(&free_xfers, struct mhu_xfer, node);
	x->code = code;
	x->buf = buf;
	x->len = len;
	x->c = c;

	if (code & RESP_BIT)
		list_move(&x->node, &pending_xfers);
	else
		list_move_tail(&x->node, &pending_xfers);

	spin_unlock_irqrestore(&fsm_lock, flags);

	ret = do_xfer();

	mutex_unlock(&mhu_mutex);

	return ret;
}
EXPORT_SYMBOL_GPL(mhu_send_packet);

void mb86s70_reboot(u32 delay)
{
	void __iomem *tx_reg = MB86S70_MHU_VIRT + 0x120; /* HP-NonSec */
	struct cmd_hard_reset cmd;
	u32 val;

	cmd.payload_size = sizeof(cmd);
	cmd.delay = delay;

	val = __raw_readl(tx_reg + INTR_STAT_OFS);
	if (val) /* Flush anything pending */
		__raw_writel(val, tx_reg + INTR_CLR_OFS);

	memcpy_toio(cmd_to_scb, &cmd, sizeof(cmd));
	__raw_writel(CMD_HARD_RESET_REQ, tx_reg + INTR_SET_OFS);
}
EXPORT_SYMBOL_GPL(mb86s70_reboot);
