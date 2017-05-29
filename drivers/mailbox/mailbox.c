/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/interrupt.h>
#include <linux/spinlock.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/err.h>
#include <linux/module.h>
#include <linux/mailbox_client.h>
#include <linux/mailbox_controller.h>

/*
 * The length of circular buffer for queuing messages from a client.
 * 'msg_count' tracks the number of buffered messages while 'msg_free'
 * is the index where the next message would be buffered.
 * We shouldn't need it too big because every transferr is interrupt
 * triggered and if we have lots of data to transfer, the interrupt
 * latencies are going to be the bottleneck, not the buffer length.
 * Besides, ipc_send_message could be called from atomic context and
 * the client could also queue another message from the notifier 'txcb'
 * of the last transfer done.
 */
#define MBOX_TX_QUEUE_LEN	10

#define TXDONE_BY_IRQ	(1 << 0) /* controller has remote RTR irq */
#define TXDONE_BY_POLL 	(1 << 1) /* controller can read status of last TX */
#define TXDONE_BY_ACK	(1 << 2) /* S/W ACK recevied by Client ticks the TX */

struct ipc_chan {
	char chan_name[32]; /* controller_name:link_name */
	unsigned txdone_method;

	/* Cached values from controller */
	struct ipc_link *link;
	struct ipc_link_ops *link_ops;

	/* Cached values from client */
	void (*rxcb)(void *data);
	void (*txcb)(request_token_t t, enum xfer_result r);
	bool tx_block;
	unsigned long tx_tout;
	struct completion tx_complete;

	request_token_t active_token;
	unsigned msg_count, msg_free;
	void *msg_data[MBOX_TX_QUEUE_LEN];
	/* Timer shared by all links of a controller */
	struct tx_poll_timer *timer;
	bool assigned;
	/* Serialize access to the channel */
	spinlock_t lock;
	/* Hook to add to the global list of channels */
	struct list_head node;
	/* Notifier to all clients waiting on aquiring this channel */
	struct blocking_notifier_head avail;
};

/*
 * If the controller supports only TXDONE_BY_POLL, this
 * timer polls all the links for txdone.
 */
struct tx_poll_timer {
	struct timer_list poll;
	unsigned period;
};

static LIST_HEAD(ipc_channels);
static DEFINE_MUTEX(chpool_mutex);

static request_token_t _add_to_rbuf(struct ipc_chan *chan, void *data)
{
	request_token_t idx;
	unsigned long flags;

	spin_lock_irqsave(&chan->lock, flags);

	/* See if there is any space left */
	if (chan->msg_count == MBOX_TX_QUEUE_LEN) {
		spin_unlock_irqrestore(&chan->lock, flags);
		return 0;
	}

	idx = chan->msg_free;
	chan->msg_data[idx] = data;
	chan->msg_count++;

	if (idx == MBOX_TX_QUEUE_LEN - 1)
		chan->msg_free = 0;
	else
		chan->msg_free++;

	spin_unlock_irqrestore(&chan->lock, flags);

	return idx + 1;
}

static void _msg_submit(struct ipc_chan *chan)
{
	struct ipc_link *link = chan->link;
	unsigned count, idx;
	unsigned long flags;
	void *data;
	int err;

	spin_lock_irqsave(&chan->lock, flags);

	if (!chan->msg_count || chan->active_token) {
		spin_unlock_irqrestore(&chan->lock, flags);
		return;
	}

	count = chan->msg_count;
	idx = chan->msg_free;
	if (idx >= count)
		idx -= count;
	else
		idx += MBOX_TX_QUEUE_LEN - count;

	data = chan->msg_data[idx];

	/* Try to submit a message to the IPC controller */
	err = chan->link_ops->send_data(link, data);
	if (!err) {
		chan->active_token = idx + 1;
		chan->msg_count--;
	}

	spin_unlock_irqrestore(&chan->lock, flags);
}

static void tx_tick(struct ipc_chan *chan, enum xfer_result r)
{
	unsigned long flags;
	request_token_t t;

	spin_lock_irqsave(&chan->lock, flags);
	t = chan->active_token;
	chan->active_token = 0;
	spin_unlock_irqrestore(&chan->lock, flags);

	/* Submit next message */
	_msg_submit(chan);

	/* Notify the client */
	if (chan->tx_block)
		complete(&chan->tx_complete);
	else if (t && chan->txcb)
		chan->txcb(t, r);
}

static void poll_txdone(unsigned long data)
{
	struct tx_poll_timer *timer = (struct tx_poll_timer *)data;
	bool txdone, resched = false;
	struct ipc_chan *chan;

	list_for_each_entry(chan, &ipc_channels, node) {
		if (chan->timer == timer && chan->active_token) {
			resched = true;
			txdone = chan->link_ops->last_tx_done(chan->link);
			if (txdone)
				tx_tick(chan, XFER_OK);
		}
	}

	if (resched)
		mod_timer(&timer->poll,
			jiffies + msecs_to_jiffies(timer->period));
}

/*
 * After 'startup' and before 'shutdown', the IPC controller driver
 * notifies the API of data received over the link.
 * The controller driver should make sure the 'RTR' is de-asserted since
 * reception of the packet and until after this call returns.
 * This call could be made from atomic context.
 */
void ipc_link_received_data(struct ipc_link *link, void *data)
{
	struct ipc_chan *chan = (struct ipc_chan *)link->api_priv;

	mbox_dbg("%s:%d link=%p chan=%p\n", __func__, __LINE__, link, chan);
	/* No buffering the received data */
	if (chan->rxcb)
		chan->rxcb(data);
}
EXPORT_SYMBOL(ipc_link_received_data);

/*
 * The IPC controller driver notifies the API that the remote has
 * asserted RTR and it could now send another message on the link.
 */
void ipc_link_txdone(struct ipc_link *link, enum xfer_result r)
{
	struct ipc_chan *chan = (struct ipc_chan *)link->api_priv;

	if (unlikely(!(chan->txdone_method & TXDONE_BY_IRQ))) {
		printk("Controller can't run the TX ticker\n");
		return;
	}

	tx_tick(chan, r);
}
EXPORT_SYMBOL(ipc_link_txdone);

/*
 * The client/protocol had received some 'ACK' packet and it notifies
 * the API that the last packet was sent successfully. This only works
 * if the controller doesn't get IRQ for TX done.
 */
void ipc_client_txdone(void *channel, enum xfer_result r)
{
	struct ipc_chan *chan = (struct ipc_chan *)channel;
	bool txdone = true;

	if (unlikely(!(chan->txdone_method & TXDONE_BY_ACK))) {
		printk("Client can't run the TX ticker\n");
		return;
	}

	if (chan->txdone_method & TXDONE_BY_POLL)
		txdone = chan->link_ops->last_tx_done(chan->link);

	if (txdone)
		tx_tick(chan, r);
}
EXPORT_SYMBOL(ipc_client_txdone);

/*
 * Called by a client to "put data on the h/w channel" so that if
 * everything else is fine we don't need to do anything more locally
 * for the remote to receive the data intact.
 * In reality, the remote may receive it intact, corrupted or not at all.
 * This could be called from atomic context as it simply
 * queues the data and returns a token (request_token_t)
 * against the request.
 * The client is later notified of successful transmission of
 * data over the channel via the 'txcb'. The client could in
 * turn queue more messages from txcb.
 */
request_token_t ipc_send_message(void *channel, void *data)
{
	struct ipc_chan *chan = (struct ipc_chan *)channel;
	request_token_t t;

	if (!chan) {
		pr_err("%s:%d!!!\n", __func__, __LINE__);
		return 0;
	}

	if (chan->tx_block)
		init_completion(&chan->tx_complete);

	t = _add_to_rbuf(chan, data);
	if (!t)
		printk("Try increasing MBOX_TX_QUEUE_LEN\n");

	_msg_submit(chan);

	if (chan->txdone_method	& TXDONE_BY_POLL)
		poll_txdone((unsigned long)chan->timer);

	if (chan->tx_block && chan->active_token) {
		int ret;
		ret = wait_for_completion_timeout(&chan->tx_complete,
			chan->tx_tout);
		if (ret == 0) {
			t = 0;
			pr_err("%s:%d!!!\n", __func__, __LINE__);
			tx_tick(chan, XFER_ERR);
		}
	}

	return t;
}
EXPORT_SYMBOL(ipc_send_message);

/*
 * A client driver asks for exclusive use of a channel/mailbox.
 * If assigned, the channel has to be 'freed' before it could
 * be assigned to some other client.
 * After assignment, any packet received on this channel will be
 * handed over to the client via the 'rxcb' callback.
 * The 'txcb' callback is used to notify client upon sending the
 * packet over the channel, which may or may not have been yet
 * read by the remote processor.
 */
void *ipc_request_channel(struct ipc_client *cl)
{
	struct ipc_chan *chan;
	unsigned long flags;
	int ret = 0;

	mutex_lock(&chpool_mutex);

	list_for_each_entry(chan, &ipc_channels, node) {
		spin_lock_irqsave(&chan->lock, flags);
		if(!chan->assigned
				&& !strcmp(cl->chan_name, chan->chan_name)) {
			chan->msg_free = 0;
			chan->msg_count = 0;
			chan->active_token = 0;
			chan->rxcb = cl->rxcb;
			chan->txcb = cl->txcb;
			chan->assigned = true;
			chan->tx_block = cl->tx_block;
			mbox_dbg("%s:%d chan=%p rxcb=%p\n",
				__func__, __LINE__, chan, chan->rxcb);
			if (!cl->tx_tout)
				chan->tx_tout = msecs_to_jiffies(500); /* 500ms default */
			else
				chan->tx_tout = msecs_to_jiffies(cl->tx_tout);
			if (chan->txdone_method	== TXDONE_BY_POLL
					&& cl->knows_txdone)
				chan->txdone_method |= TXDONE_BY_ACK;
			ret = 1;
		}
		spin_unlock_irqrestore(&chan->lock, flags);
		if (ret)
			break;
	}

	mutex_unlock(&chpool_mutex);

	if (!ret) {
		mbox_dbg("Unable to assign mailbox(%s)\n", cl->chan_name);
		return NULL;
	}

	ret = chan->link_ops->startup(chan->link, cl->cntlr_data);
	if (ret) {
		printk("Unable to startup the link\n");
		ipc_free_channel((void *)chan);
		return NULL;
	}

	return (void *)chan;
}
EXPORT_SYMBOL(ipc_request_channel);

/* Drop any messages queued and release the channel */
void ipc_free_channel(void *ch)
{
	struct ipc_chan *chan = (struct ipc_chan *)ch;
	unsigned long flags;

	if (!chan)
		return;

	spin_lock_irqsave(&chan->lock, flags);
	if (!chan->assigned) {
		spin_unlock_irqrestore(&chan->lock, flags);
		return;
	}

	/* The queued TX requests are simply aborted, no callbacks are made */
	chan->assigned = false;
	if (chan->txdone_method == (TXDONE_BY_POLL | TXDONE_BY_ACK))
		chan->txdone_method = TXDONE_BY_POLL;
	spin_unlock_irqrestore(&chan->lock, flags);

	chan->link_ops->shutdown(chan->link);
	blocking_notifier_call_chain(&chan->avail, 0, NULL);
}
EXPORT_SYMBOL(ipc_free_channel);

int ipc_notify_chan_register(const char *name, struct notifier_block *nb)
{
	struct ipc_chan *chan;

	if (!nb)
		return -EINVAL;

	mutex_lock(&chpool_mutex);

	list_for_each_entry(chan, &ipc_channels, node)
		if (!strcmp(name, chan->chan_name))
			blocking_notifier_chain_register(&chan->avail, nb);

	mutex_unlock(&chpool_mutex);

	return 0;
}
EXPORT_SYMBOL(ipc_notify_chan_register);

void ipc_notify_chan_unregister(const char *name, struct notifier_block *nb)
{
	struct ipc_chan *chan;

	if (!nb)
		return;

	mutex_lock(&chpool_mutex);

	list_for_each_entry(chan, &ipc_channels, node)
		if (!strcmp(name, chan->chan_name))
			blocking_notifier_chain_unregister(&chan->avail, nb);

	mutex_unlock(&chpool_mutex);
}
EXPORT_SYMBOL(ipc_notify_chan_unregister);

/*
 * Call for IPC controller drivers to register a controller, adding
 * its channels/mailboxes to the global pool.
 */
int ipc_links_register(struct ipc_controller *ipc_con)
{
	struct tx_poll_timer *timer = NULL;
	struct ipc_chan *channel;
	int i, num_links, txdone;

	/* Are you f***ing with us, sir? */
	if (!ipc_con || !ipc_con->ops)
		return -EINVAL;

	for (i = 0; ipc_con->links[i]; i++)
		;
	if (!i)
		return -EINVAL;
	num_links = i;

	if (ipc_con->txdone_irq)
		txdone = TXDONE_BY_IRQ;
	else if (ipc_con->txdone_poll)
		txdone = TXDONE_BY_POLL;
	else /* It has to be at least ACK */
		txdone = TXDONE_BY_ACK;

	if (txdone == TXDONE_BY_POLL) {
		timer = kzalloc(sizeof(struct tx_poll_timer), GFP_KERNEL);
		timer->period = ipc_con->txpoll_period;
		timer->poll.function = &poll_txdone;
		timer->poll.data = (unsigned long)timer;
		init_timer(&timer->poll);
	}

	channel = kzalloc(sizeof(struct ipc_chan) * num_links, GFP_KERNEL);

	for (i = 0; i < num_links; i++) {
		channel[i].timer = timer;
		channel[i].assigned = false;
		channel[i].txdone_method = txdone;
		channel[i].link_ops = ipc_con->ops;
		channel[i].link = ipc_con->links[i];
		channel[i].link->api_priv = &channel[i];
		snprintf(channel[i].chan_name, 32, "%s:%s",
			ipc_con->controller_name,
			ipc_con->links[i]->link_name);
		spin_lock_init(&channel[i].lock);
		BLOCKING_INIT_NOTIFIER_HEAD(&channel[i].avail);
		INIT_LIST_HEAD(&channel[i].node);
		mutex_lock(&chpool_mutex);
		list_add_tail(&channel[i].node, &ipc_channels);
		mutex_unlock(&chpool_mutex);
		mbox_dbg("%s:%d chan=%p-link=%p\n",
			__func__, __LINE__, &channel[i], ipc_con->links[i]);
	}

	return 0;
}
EXPORT_SYMBOL(ipc_links_register);

/* Free any occupied channels */
void ipc_links_unregister(struct ipc_controller *ipc_con)
{
	/* TBD */
}
EXPORT_SYMBOL(ipc_links_unregister);

