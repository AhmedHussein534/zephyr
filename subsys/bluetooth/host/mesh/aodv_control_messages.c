/** @file ctrMsg.c
 *  @brief Routng Control Messages File
 *
 *  Bluetooth routing control messages following AODV protocol.
 *	The file contains RREQ, RREP and RWAIT data and functions.
 *  @bug No known bugs.
 */

/* -- Includes -- */
#include <zephyr.h>
#include <net/buf.h>
#include <bluetooth/mesh.h>

#include "common/log.h"
#include "routing_table.h"
#include "mesh.h"
#include "net.h"
#include "transport.h"
#include "access.h"
#include "foundation.h"

#include "aodv_control_messages.h"

/**
 *  @brief Defines a linked list used by bt_mesh_trans_ring_search
 * 				- Ring search initiates a RREQ message and waits for RREP / RWAIT in return
 *						by checking this linked list.
 *				- When a RREP is received, it creates a new node with destination address
 *						of RREP and hop count = 0.
 *				- When a RWAIT is received, it creates a new node with destination address
 *						of RREQ's destination and hop count = hop count from intermediate
 *						node to destination.
 */
sys_slist_t rrep_rwait_list;
struct k_mem_slab rrep_slab;
/**
 *	@brief Ring search timer to opt out of ring search and
 * 				 increase the TTL value
 */
struct ring_search_flag_timer {
	struct k_timer ring_timer;
	/** When the timer expires, this flag is set */
	bool ring_flag;
};

/* DEFINITIONS */
#define INRANGE(new_seq, existing_seq) ((new_seq > existing_seq) ? 1 : 0)  /* FIXME handle wrapping and IV index? */
#define rrep_rwait_list_NUMBER_OF_ENTRIES 20
#define RREP_ENTRY_SIZE sizeof(struct rrep_rwait_list_entry)

K_MEM_SLAB_DEFINE(rrep_slab, RREP_ENTRY_SIZE, rrep_rwait_list_NUMBER_OF_ENTRIES, ALLIGNED);
K_SEM_DEFINE(rrep_rwait_list_sem, 1, 1);  /* Binary semaphore for RREP linked list critical section */

/* FUNCTIONS PROTOTYPES */
static bool rreq_send(struct rreq_data *data, u8_t TTL, u16_t net_idx);
static void rreq_recv_cb(struct k_timer *timer_id);
static void ring_search_timer(struct k_timer *timer_id);
static bool rrep_send(struct rrep_data *RREP_msg,u16_t net_idx, u16_t dst );
static bool rrep_rwait_list_create_entry(struct rrep_rwait_list_entry *entry_data);
static void rwait_send(struct rreq_data* rreq_recv_data,struct bt_mesh_route_entry *destination_entry,
	struct rwait_data rwait_data, struct bt_mesh_net_rx* rx, bool relay);
/*static void view_rrep_rwait_list();*/

/* FUNCTIONS IMPLEMENTATION */
/* RREQ Functions */

/**
 *	@brief Called when a RREQ needs to be sent. It sets the content of ctx and
 *				 tx structs, the values of RREQ data and sends the constructed values
 *				 to the transport layer.
 *
 *	@param data: Pointer to a structure of type rreq_data that holds the data
 * 							 received from the transport and network layer.
 *	@param TTL: Unsigned integer that holds range of RREQ to be relayed.
 *							Usually set by bt_mesh_trans_ring_search.
 *	@param net_idx: Unsigned integer
 *
 *	@return RANA:
 */
static bool rreq_send(struct rreq_data *data, u8_t TTL, u16_t net_idx)
{
	/*data -> destination_address = 0x0e0e;*/ /* TESTING */

	/* Concatenate RREQ flags into 1 byte */
	u8_t flags = data->G + (data->D << 1) + (data->U << 2) + (data->I << 3);
	/* Default network layer next hop is to broadcast to all nodes */
	u16_t network_next_hop = BT_MESH_ADDR_ALL_NODES;
	/* Create a buffer to store RREQ data */
	NET_BUF_SIMPLE_DEFINE(buf, RREQ_SDU_MAX_SIZE);

	struct bt_mesh_route_entry *entry;
	/* If Intermediate flag is set to 1 and a valid destination is found
	 *	then it's an intermediate node that has received a flooded RREQ
	 *	and will proceed to send a directed RREQ to destination.
	 */
	if (data->I && bt_mesh_search_valid_destination_without_source(data->destination_address, &entry))
	{
		network_next_hop = entry->next_hop;
	}

	struct bt_mesh_msg_ctx ctx =
	{
		.app_idx  = BT_MESH_KEY_UNUSED, /* Control messages have no app index */
		.net_idx  = net_idx,
		.addr = network_next_hop,
		.send_ttl = TTL
	};
	struct bt_mesh_net_tx tx =
	{
		.ctx  = &ctx,
		.sub  = bt_mesh_subnet_get(net_idx),
		.src  = bt_mesh_primary_addr(),
		.aszmic = 1,
		.xmit = bt_mesh_net_transmit_get(),
		.routing = true
	};

	/* Add RREQ data in a buffer to be sent */
	net_buf_simple_add_mem(&buf, &data->hop_count, 1);
	net_buf_simple_add_mem(&buf, &data->source_address, 2);
	net_buf_simple_add_mem(&buf, &data->destination_address, 2);
	net_buf_simple_add_mem(&buf, &data->source_number_of_elements, 2);
	net_buf_simple_add_mem(&buf, &data->hop_count, 1);
	net_buf_simple_add_mem(&buf, &flags, 1);
	net_buf_simple_add_mem(&buf, &data->source_sequence_number, 3);
	if (data->U == 0)
	{
		/* Add the destination sequence number if it's known */
		net_buf_simple_add_mem(&buf, &data->destination_sequence_number, 3);
	}

	/* Send the constructed buffer to the transport layer */
	int err = bt_mesh_ctl_send(&tx, TRANS_CTL_OP_RREQ, buf.data, buf.len, NULL, NULL, NULL);

	if (err != 0) { /* FIXME */
		return false;
	} else   {
		return true;
	}
}

/**
 *	@brief Called by the invalid entry timer created by the first
 *				 received RREQ to send a RREP in response.
 *
 *	@param timer_id: Pointer to a structure of type k_timer that represents timer
 *									 of the invalid entry.
 *
 *	@return N/A
 */
static void rreq_recv_cb(struct k_timer *timer_id)
{
	/* TODO: ADD SEMAPHORE so this fn doesn't work with RREQ_RECEIVED */
	/* Pull out the container of the timer to access the entry */
	struct bt_mesh_route_entry *entry = CONTAINER_OF(timer_id, struct bt_mesh_route_entry, lifetime);
	/* Change route status from invalid to valid */
	bt_mesh_validate_route(entry);

	/* Construct RREP data to be sent in a response to the recv RREQ */
	struct rrep_data data;
	data.R=1; /* TODO: who sets R ? */
	data.source_address = entry -> destination_address ;
	data.destination_address = entry -> source_address;
	data.destination_sequence_number = bt_mesh.seq;
	data.hop_count = 0;
	data.destination_number_of_elements = bt_mesh_elem_count();

	rrep_send(&data,entry->net_idx,entry->next_hop);
}

/**
 *	@brief Called by the ring search timer to set the ring_flag to indicate
 *				 the need to increment the TTL.
 *
 *	@param timer_id: Pointer to a structure of type k_timer that represents
 *									 the timer of the ring search.
 *	@return N/A
 */
static void ring_search_timer(struct k_timer *timer_id)
{
	/* Pull out the container containing the timer to access the ring_flag*/
	struct ring_search_flag_timer *entry = CONTAINER_OF(timer_id, struct ring_search_flag_timer, ring_timer); /* container of timer_id to be deleted*/
	entry->ring_flag = true;
}

/**
 *	@brief Called by ctl_recv in the transport layer when the OP code refers to
 *				 a RREQ
 *
 *	@param rx: Pointer to a structure of type bt_mesh_net_rx that holds
 *						 the received network layer data.
 *	@param buf: Pointer to a structure of type net_buf_simple that holds
 *							the received RREQ data.
 *	@return RANA:
 */
bool bt_mesh_trans_rreq_recv(struct bt_mesh_net_rx *rx, struct net_buf_simple *buf)
{
	/* Dissect the received RREQ into fields */
	struct rreq_data temp;
	struct rreq_data *data = &temp;
	data->source_address = RREQ_GET_SRC_ADDR(buf);
	data->destination_address = RREQ_GET_DST_ADDR(buf);
	data->source_number_of_elements = RREQ_GET_SRC_NUMBER_OF_ELEMENTS(buf);
	data->hop_count = RREQ_GET_HOP_COUNT(buf);
	data->next_hop = rx->ctx.addr;
	data->G = RREQ_GET_G_FLAG(buf);
	data->D = RREQ_GET_D_FLAG(buf);
	data->U = RREQ_GET_U_FLAG(buf);
	data->I = RREQ_GET_I_FLAG(buf);
	data->destination_sequence_number = RREQ_GET_DST_SEQ(buf);
	data->source_sequence_number = RREQ_GET_SRC_SEQ(buf);

	struct bt_mesh_route_entry *entry = NULL;
	/* If element is requesting data transaction from an element in the same node,
	 * Drop rreq. This case is to prevent receiving RREQ from neighbouring elements */
	if (bt_mesh_elem_find(data->source_address))
	{
		return 0;
	}
	/* If a RREQ is received by the destination node */
	else if (bt_mesh_elem_find(data->destination_address))
	{
		/* Drop any received RREQ after the expiry of the ring search timer */
		if (bt_mesh_search_valid_destination(data->destination_address, data->source_address, &entry)) {
			printk("RREQ dropped - RREQ received after RREP Interval\n");
			return 0;
		}
		/* Multiple RREQs are received in the interval of ring search timer */
		else if (bt_mesh_search_invalid_destination(data->destination_address, data->source_address, &entry))
		{
			/* If it contains better data, replace */
			if (data->hop_count < entry->hop_count) {
				printk("Modifying existing entry \n");
				entry->destination_sequence_number = data->destination_sequence_number;
				entry->hop_count                   = data->hop_count;
				entry->next_hop                    = data->next_hop;
			}
			return 0;
		}
		/* Destination has received the first RREQ */
		else {
			printk("Creating entry and waiting for RREQ wait interval \n");
			/* Create a reverse entry */
			struct bt_mesh_route_entry *entry_data;
			if(bt_mesh_create_entry_invalid_with_cb(&entry_data, rreq_recv_cb))
			{
			entry_data->source_address                 = data->destination_address;
			entry_data->destination_address            = data->source_address;
			entry_data->destination_sequence_number    = data->source_sequence_number;
			entry_data->next_hop                       = data->next_hop;
			entry_data->source_number_of_elements      = bt_mesh_elem_count();
			entry_data->destination_number_of_elements = data->source_number_of_elements;
			entry_data->hop_count                      = data->hop_count;
			entry_data->net_idx 											 = rx -> ctx.net_idx;
			}
			return 0;
		}
	}
	/* Intermediate node having route to destination should
	 *   - reply to RREQ originator with RWAIT
	 *   - send a directed RREQ to RREQ's destination
	*/
	else if (bt_mesh_search_valid_destination_without_source(data->destination_address, &entry)
						&& data->D == false && data->I == false)
		{
		printk("Intermediate Node received a flooded RREQ and has route to destination \n");
		/* Create a reverse entry */
		struct bt_mesh_route_entry *entry_data;
		if (bt_mesh_create_entry_invalid(&entry_data))
		{
		entry_data->source_address                 = data->destination_address;
		entry_data->destination_address            = data->source_address;
		entry_data->destination_sequence_number    = data->source_sequence_number;
		entry_data->next_hop                       = data->next_hop;
		entry_data->source_number_of_elements      = 1; /* Will be corrected by RREP */
		entry_data->destination_number_of_elements = data->source_number_of_elements;
		entry_data->hop_count                      = data->hop_count;
		entry_data->net_idx 												= rx -> ctx.net_idx;
		}
		else
		{
			return false;
		}

		/* If the stored destination sequence number is fresher:
		 *   - reply to RREQ originator with RWAIT
		 *   - send a directed RREQ to RREQ's destination
		 */
		if (entry->destination_sequence_number >= data->destination_sequence_number)
		{
			printk("SEND RWAIT and SEND RREQ with flag I=1 \n");
			data->I = 1;
			data->hop_count = data->hop_count + 1;
			rreq_send(data, 1, rx->ctx.net_idx); /* To RREQ's destination */
			struct rwait_data temp; /* Dummy struct */
			entry_data->hop_count = entry -> hop_count;
			rwait_send(data,entry_data,temp,rx,false); /* To RREQ's originator */
		}
	}
	else {
		/* Intermediate nodes that has no route to destination shall relay */
		printk("Intermediate Node received a flooded RREQ - Relaying \n");
		struct bt_mesh_route_entry *entry;

		/* If the reverse route wasn't created, create it */
		if (!bt_mesh_search_invalid_destination(data->destination_address, data->source_address, &entry))
		{
			struct bt_mesh_route_entry *entry_data;
			if(bt_mesh_create_entry_invalid(&entry_data))
			{
			entry_data->source_address                 = data->destination_address;
			entry_data->destination_address            = data->source_address;
			entry_data->destination_sequence_number    = data->source_sequence_number;
			entry_data->next_hop                       = data->next_hop;
			entry_data->source_number_of_elements      = 1; /* UNKNOWN. Will be corrected by RREP */
			entry_data->destination_number_of_elements = data->source_number_of_elements;
			entry_data->hop_count                      = data->hop_count;
			entry_data->net_idx 											 = rx -> ctx.net_idx;
			}
			else {
				return false;
			}
			data->next_hop = data->next_hop + 1;
			/* Relay the received RREQ */
			return rreq_send(data, rx->ctx.recv_ttl - 1, rx->ctx.net_idx);
		}

		/* If an invalid entry was found and the stored destination sequence
		 * 	is fresher than the received one, refresh the route entry timer
		 */
		else if (entry->destination_sequence_number < data->source_sequence_number)
		{
			entry->destination_sequence_number = data->source_sequence_number;
			bt_mesh_refresh_lifetime_invalid(entry);
			return rreq_send(data, rx->ctx.recv_ttl - 1, rx->ctx.net_idx);
		}
	}

	/* TESTING: Test sent RREQ data is correct.
		 printk("Source Address=%04x \n",RREQ_GET_SRC_ADDR(buf));
	   printk("Destination Address=%04x \n",RREQ_GET_DST_ADDR(buf));
	   printk("Source Number of Elements=%04x \n",RREQ_GET_SRC_NUMBER_OF_ELEMENTS(buf));
	   printk("Hop Count=%d \n",RREQ_GET_HOP_COUNT(buf));
	   printk("G Flag=%d \n",RREQ_GET_G_FLAG(buf));
	   printk("D Flag=%d \n",RREQ_GET_D_FLAG(buf));
	   printk("U Flag=%d \n",RREQ_GET_U_FLAG(buf));
	   printk("I Flag=%d \n",RREQ_GET_I_FLAG(buf));
	   printk("Destination Sequence Number=%08x \n",RREQ_GET_DST_SEQ(buf));
	 */
	 return 0;
}

/**
 *	@brief Called by bt_mesh_trans_send in the transport layer when no route
 *				 to destination is found.
 *
 *	@param tx: Pointer to a structure of type bt_mesh_net_tx that holds
 *						 the transmitted network layer data.
 *
 *	@return RANA:
 */
u8_t bt_mesh_trans_ring_search(struct bt_mesh_net_tx *tx)
{
	u16_t source_address = tx->src; /* Primary element source address */
	u16_t destination_address = tx->ctx->addr; /* TODO: BT_MESH_ADDR_ALL_NODES ? */

	/* The following 2 fields will be set if
	 * an invalid route is found to destination */
	u32_t destination_sequence_number = 0;
	bool U_flag = 0; /* Unknown destination sequence number flag */

	/* Create a ring search timer */
	struct ring_search_flag_timer ring_struct;
	ring_struct.ring_flag = false;
	struct rrep_rwait_list_entry *temp;
	k_timer_init(&ring_struct.ring_timer, ring_search_timer, NULL);                                 /*init lifetime timer*/
	k_timer_start(&ring_struct.ring_timer, RREQ_RING_SEARCH_WAIT_INTERVAL, RREQ_RING_SEARCH_WAIT_INTERVAL);   /* Reset Timer */

	/* Mesh specs prohibits the use of TTL = 1 */
	u8_t TTL = 2; /* Initial TTL */
	printk("current TTL=%d \n", TTL);

	struct bt_mesh_route_entry *entry;
	if (bt_mesh_search_invalid_destination(source_address, destination_address, &entry))
	{
		destination_sequence_number = entry->destination_sequence_number;
		U_flag = 1;
	}

	/* Construct RREQ data to be sent */
	struct rreq_data data = {
		.source_address = bt_mesh_primary_addr(),
		.destination_address = destination_address,
		.U = U_flag,
		.hop_count = 0,
		.source_sequence_number = bt_mesh.seq,
		.source_number_of_elements = bt_mesh_elem_count(),
		.destination_sequence_number = destination_sequence_number
	};
	rreq_send(&data, TTL, tx->ctx->net_idx);

	/* Keep searching the rrep_rwait_list till an entry is found.
	 * An entry means a RREP or RWAIT has been received. */
	while (1)
	{
		k_sem_take(&rrep_rwait_list_sem, K_FOREVER);
		SYS_SLIST_FOR_EACH_CONTAINER(&rrep_rwait_list, temp, node)
		{
			/* RWAIT Received */
			if(temp->hop_count != 0)
			{
				printk("Delaying Ring Search with hop count =%d",temp->hop_count);
				/* Refresh ring search timer*/
				k_timer_stop(&ring_struct.ring_timer);
				struct k_timer temp_timer;
				ring_struct.ring_timer = temp_timer;
				k_timer_init(&ring_struct.ring_timer, ring_search_timer, NULL);
				k_timer_start(&ring_struct.ring_timer, RREQ_RING_SEARCH_WAIT_INTERVAL*4, 0); /* TODO: correct this */

				/* delete entry */
				sys_slist_find_and_remove(&rrep_rwait_list, &temp->node);
				k_sem_give(&rrep_rwait_list_sem);
				k_mem_slab_free(&rrep_slab, (void **)&temp);
			}

			/* RREP Received */
			if (temp->destination_address== destination_address)
			 {
				/*Stop ring search timer and delete the entry*/
				k_timer_stop(&ring_struct.ring_timer);
				sys_slist_find_and_remove(&rrep_rwait_list, &temp->node);
				k_sem_give(&rrep_rwait_list_sem);
				k_mem_slab_free(&rrep_slab, (void **)&temp);
				return 1;
			}
		}
		k_sem_give(&rrep_rwait_list_sem); /*return semaphore */

		/* If the ring search timer expires:
		 *	- Increment the TTL by 1
		 *	- Fetch the latest sequence number
		 */
		if (ring_struct.ring_flag == true)
		{
			ring_struct.ring_flag = false;
			TTL = TTL + 1;
			data.source_sequence_number = bt_mesh.seq;
			rreq_send(&data, TTL, tx->ctx->net_idx);
			printk("current TTL=%d \n", TTL);
			/* Opt out if the max TTL is reached */
			if (TTL == RREQ_RING_SEARCH_MAX_TTL)
			{
				k_timer_stop(&ring_struct.ring_timer);
				return 0;
			}
		}
		/* Sleep so as not to search the list continiously */
		k_sleep(K_MSEC(50));
	}
}

/* RREP Functions */

/**
 *	@brief Called when a RREP needs to be sent in response to a received RREQ.
 *	       It sets the content of ctx and tx structs, the values of RREP data
 * 				 and sends the constructed values to the transport layer.
 *
 *	@param data: Pointer to a structure of type rrep_data that holds the data
 * 							 from the transport and network layer to be sent.
 *	@param net_idx: Unsigned integer
 *	@param destination_address: Unsigned integer that holds the next hop
 *															destination from the reverse route entry.
 *
 *	@return RANA:
 */
static bool rrep_send(struct rrep_data *data,u16_t net_idx, u16_t destination_address )
{
	/* TODO : check when rreq_recv is calling rrep_send */
	struct bt_mesh_msg_ctx ctx =
	{
		.app_idx  = BT_MESH_KEY_UNUSED,
		.net_idx  = net_idx,
		.send_ttl = 3 /* FIXME */
	};

	struct bt_mesh_net_tx tx = {
		.sub = bt_mesh_subnet_get(net_idx),
		.ctx = &ctx,
		.xmit = bt_mesh_net_transmit_get(),
	};
	tx.ctx->addr = destination_address;
	tx.src = bt_mesh_primary_addr();
	tx.ctx->send_ttl--;

	/* TESTING: View RREP data
	printk("RREP R 0x%01x \n", RREP_msg->R);
	printk("RREP source_address 0x%02x \n", RREP_msg->source_address);
	printk("RREP dst 0x%02x \n", RREP_msg->destination_address);
	printk("RREP seq 0x%04x \n", RREP_msg->destination_sequence_number);
	printk("RREP hop_count 0x%01x \n", RREP_msg->hop_count);
	printk("RREP elem 0x%04x \n", RREP_msg->destination_number_of_elements);
	*/

	/* Create a buffer for RREP data */
	NET_BUF_SIMPLE_DEFINE(buf, RREP_SDU_MAX_SIZE);
	net_buf_simple_add_mem(&buf, &data->R, 1); /* FIXME: should be 1 bit only. */
	net_buf_simple_add_mem(&buf, &data->source_address, 2);
	net_buf_simple_add_mem(&buf, &data->destination_address, 2);
	net_buf_simple_add_mem(&buf, &data->destination_sequence_number, 4);
	net_buf_simple_add_mem(&buf, &data->hop_count, 1);
	net_buf_simple_add_mem(&buf, &data->destination_number_of_elements, 2);

	return bt_mesh_ctl_send(&tx, TRANS_CTL_OP_RREP, buf.data,
				buf.len, NULL, NULL, NULL);
}

/**
 *	@brief Creates a new entry in rrep_rwait_list when RREP or RWAIT is received.
 *
 *	@param data: Pointer to a structure of type rrep_data that holds the
 *							 received hop count and RREQ's destination.
 *
 *	@return RANA:
 */
static bool rrep_rwait_list_create_entry(struct rrep_rwait_list_entry *entry_data)
{
  struct rrep_rwait_list_entry* entry_location=NULL;
	/* Insert a new node into rrep_rwait_list */
	if (k_mem_slab_alloc(&rrep_slab, (void **)&entry_location, 100) == 0)
	{
		memset(entry_location, 0, ENTRY_SIZE);
		k_sem_take(&rrep_rwait_list_sem, K_FOREVER);
		sys_slist_append(&rrep_rwait_list, &entry_location->node);
		k_sem_give(&rrep_rwait_list_sem);
	}
	else {
		/* Memory Allocation timeout */
		return false;
	}
	entry_location -> destination_address = entry_data -> destination_address;
	entry_location -> hop_count = entry_data -> hop_count;
	return true;
}

/**
 *	@brief Called by ctl_recv in the transport layer when the OP code refers to
 *				 a RREP.
 *
 *	@param rx: Pointer to a structure of type bt_mesh_net_rx that holds
 *						 the received network layer data.
 *	@param buf: Pointer to a structure of type net_buf_simple that holds
 *							the received RREP data.
 *
 *	@return RANA:
 */
bool bt_mesh_trans_rrep_recv(struct bt_mesh_net_rx *rx, struct net_buf_simple *buf)
{
	/* Dissect the RREP into its fields */
	struct rrep_data temp;
	struct rrep_data *data = &temp;
	data->R = RREP_GET_R(buf);
	data->source_address = RREP_GET_SRC_ADDR(buf);
	data->destination_address = RREP_GET_DST_ADDR(buf);
	data->destination_sequence_number = RREP_GET_SEQ_NUM(buf);
	data->hop_count = RREP_GET_HOP_COUNT(buf);
	data->destination_number_of_elements = RREP_GET_SRC_NUMBER_OF_ELEMENTS(buf);

	/* Testing: View received RREP data
	printk("RREP R 0x%01x \n", data->R);
	printk("RREP source_address 0x%04x \n", data->source_address);
	printk("RREP dst 0x%04x \n", data->destination_address);
	printk("RREP seq 0x%04x \n", data->destination_sequence_number);
	printk("RREP hop_count 0x%02x \n", data->hop_count);
	printk("RREP elem 0x%02x \n", data->destination_number_of_elements);
	printk("Network Src 0x%02x \n", rx->ctx.addr);
	printk("Network dst 0x%02x \n", rx->dst);
	printk("Network recieved TL 0x%02x \n", rx->ctx.send_ttl);
	printk("msg->src 0x%04x \n", data->source_address);
	printk("bt_mesh_primary_addr() 0x%04x \n", bt_mesh_primary_addr());
	*/

	/* If the RREP is received by the RREQ originator */
	if (data->source_address == bt_mesh_primary_addr())
	{
		struct bt_mesh_route_entry *found_entry = NULL;
		if (!bt_mesh_search_valid_destination(data->source_address, data->destination_address, &found_entry) ||
				(INRANGE(data->destination_sequence_number,found_entry->destination_sequence_number)
				&& bt_mesh_invalidate_route(found_entry)))
			{
			/* Create forward entry */
			struct bt_mesh_route_entry *table_entry;
			if(bt_mesh_create_entry_valid(&table_entry))
			{
			table_entry->source_address                  = data->source_address;
			table_entry->destination_address             = data->destination_address;
			table_entry->destination_sequence_number     = data->destination_sequence_number;
			table_entry->next_hop                        = rx->ctx.addr;
			table_entry->hop_count                       = data->hop_count;
			table_entry->destination_number_of_elements  = data->destination_number_of_elements;
			table_entry->source_number_of_elements 			 = bt_mesh_elem_count();
			table_entry->net_idx 												 = rx -> ctx.net_idx;
			}
			else {
				return false;
			}
			/* Create entry in rrep_rwait_list */
			struct rrep_rwait_list_entry rrep_entry_temp;
			struct rrep_rwait_list_entry *rrep_entry=&rrep_entry_temp;
			rrep_entry->destination_address = data->destination_address;
			rrep_entry->hop_count = data->hop_count;
			return rrep_rwait_list_create_entry(rrep_entry);
		}
	}
	/* RREP is received by an intermediate node and should be directed
	 * to RREQ originator by the invalid table entry created by RREQ
	 */
	else {
		struct bt_mesh_route_entry *existing_entry;
		/* Get the entry of reverse route created by RREQ */
		if (bt_mesh_search_invalid_destination_with_range(data->destination_address,data->source_address,data->destination_number_of_elements, &existing_entry))
		{
			/* Modify the RREQ's destination number of elements */
			existing_entry->source_number_of_elements=data->destination_number_of_elements;
			/* Modify the RREQ's destination address with the primary one */
			existing_entry->source_address=data->destination_address;
			/* Validate the reverse route created by RREQ */
			bt_mesh_validate_route(existing_entry);
			/* Create a forward route */
			struct bt_mesh_route_entry *table_entry;
			if(bt_mesh_create_entry_valid(&table_entry))
			{
			table_entry->source_address                  = data->source_address;
			table_entry->destination_address             = data->destination_address;
			table_entry->destination_sequence_number     = data->destination_sequence_number;
			table_entry->next_hop                        = rx->ctx.addr;
			table_entry->hop_count                       = data->hop_count;
			table_entry->destination_number_of_elements  = data->destination_number_of_elements;
			table_entry->source_number_of_elements 			= existing_entry->destination_number_of_elements;
			table_entry->net_idx 												= rx -> ctx.net_idx;
			}

			data->hop_count++;
			rrep_send(data, rx -> ctx.net_idx ,existing_entry->next_hop);
		}
	}
	return true;
}

/**
 *	@brief Initializes the rrep_rwait_list
 *
 *	@param N/A
 *
 *	@return N/A
 */
void bt_mesh_trans_rrep_rwait_list_init()
{
	sys_slist_init(&rrep_rwait_list);
}

/* TESTING: view rrep_rwait_list content to check addresses
static void view_rrep_rwait_list()
{
	if (sys_slist_is_empty(&rrep_rwait_list)) {
		printk("RREP List is empty \n");
		return;
	}
	struct rrep_rwait_list_entry *entry = NULL;
	k_sem_take(&rrep_rwait_list_sem, K_FOREVER);
	SYS_SLIST_FOR_EACH_CONTAINER(&rrep_rwait_list, entry, node){
		printk("RREP List:Destination address=%04x\n",entry->destination_address);
	}
	k_sem_give(&rrep_rwait_list_sem);
}
*/


/* RWAIT Functions */

/**
 *	@brief Called when a RREQ needs to be sent. It sets the content of ctx and
 *				 tx structs, the values of RREQ data and sends the constructed values
 *				 to the transport layer.
 *
 *	@param rreq_recv_data: Pointer to a structure of type rreq_data that holds
 * 							 					the data received from the transport and network layer.
 *	@param destination_entry: Pointer to a structure of type bt_mesh_route_entry
 *														from the reverese route entry.
 *	@param rwait_data: Structure of type rwait_data holding the data to be sent.
 *	@param rx: Pointer to a structure of type bt_mesh_net_rx that holds
 *						 the received network layer data.
 *	@param relay: RANA:
 *
 *	@return N/A
 */
static void rwait_send(struct rreq_data* rreq_recv_data,struct bt_mesh_route_entry *destination_entry,
	struct rwait_data rwait_data, struct bt_mesh_net_rx* rx, bool relay )
{
	/* FIXME : pass rwait_data by reference */
	u16_t rreq_net_idx 					 = rx->ctx.net_idx;
	u16_t destination_address 	 = rreq_recv_data->destination_address;
	u16_t source_address 				 = rreq_recv_data->source_address;
	u32_t source_sequence_number = rreq_recv_data->source_sequence_number;

	struct bt_mesh_msg_ctx ctx;
	struct bt_mesh_net_tx tx;
	struct rwait_data data; /* Stores RWAIT data */

	if (!relay)
	{
		data.destination_address = destination_address;
		data.source_address = source_address;
		data.source_sequence_number = source_sequence_number;
		data.hop_count = destination_entry->hop_count;

		ctx.net_idx  = rreq_net_idx;
		ctx.app_idx  = BT_MESH_KEY_UNUSED;
		ctx.addr     = destination_entry->next_hop;  /* Next hop fetched from the routing table */
		ctx.send_ttl = 3; /* FIXME */
		tx.ctx  = &ctx;
	} else   {
		data = rwait_data;
		ctx = rx->ctx;
		tx.ctx  = &ctx;
	}
	tx.sub  = bt_mesh_subnet_get(rreq_net_idx);
	tx.src  = bt_mesh_primary_addr();
	tx.xmit = bt_mesh_net_transmit_get();

	/* Construct a buffer with RWAIT's data */
	struct net_buf_simple *sdu = NET_BUF_SIMPLE(BT_MESH_TX_SDU_MAX);
	net_buf_simple_init(sdu, 0);
	net_buf_simple_add_u8(sdu, TRANS_CTL_OP_RWAIT);
	net_buf_simple_add_le16(sdu, data.destination_address);
	net_buf_simple_add_le16(sdu, data.source_address);
	net_buf_simple_add_le32(sdu, data.source_sequence_number);
	net_buf_simple_add_u8(sdu, data.hop_count);

	if (!bt_mesh_is_provisioned()) {
		BT_ERR("Local node is not yet provisioned");
		return;
	}

	if (net_buf_simple_tailroom(sdu) < 4) {
		BT_ERR("Not enough tailroom for TransMIC");
		return;
	}

	if (sdu->len > BT_MESH_TX_SDU_MAX - 4) {
		BT_ERR("Too big message");
		return;
}

	/* TESTING: View sent RWAIT data
	 printk("Source Address=%04x \n",data.source_address);
   printk("Destination Address=%04x \n",data.destination_address);
   printk("Hop Count=%d \n",data.hop_count);
	 printk("[send_Rwait]:: entry hop_count : 0x%01x \n", destination_entry->hop_count);
	*/
	bt_mesh_ctl_send(&tx, TRANS_CTL_OP_RWAIT, sdu->data,sdu->len, NULL, NULL, NULL);

}

/**
 *	@brief Called by ctl_recv in the transport layer when the OP code refers to
 *				 a RWAIT.
 *
 *	@param rx: Pointer to a structure of type bt_mesh_net_rx that holds
 *						 the received network layer data.
 *	@param buf: Pointer to a structure of type net_buf_simple that holds
 *							the received RWAIT data.
 *
 *	@return N/A
 */
void bt_mesh_trans_rwait_recv(struct bt_mesh_net_rx *rx, struct net_buf_simple *buf)
{
	struct rwait_data temp_data;
	struct rwait_data *data = &temp_data;
	struct bt_mesh_route_entry *temp = NULL;

	if (buf->len < sizeof(*data))
	{
		BT_WARN("Too short data");
	}

	/* Dissect the received RWAIT */
	data -> destination_address = RWAIT_GET_DST_ADDR(buf);
	data -> source_address = RWAIT_GET_SRC_ADDR(buf);
	data -> source_sequence_number = RWAIT_GET_SRC_SEQ_NUM(buf);
	data -> hop_count= RWAIT_GET_HOP_COUNT(buf);

	/* TESTING : Display the received RWAit
	printk("Rwait dst 0x%04x \n", data->destination_address);
	printk("Rwait src 0x%04x \n", data->source_address);
	printk("Rwait src_seq 0x%08x \n", data->source_sequence_number);
	printk("Rwait hop_count 0x%01x \n", data->hop_count);
	printk("Network Src 0x%04x \n", rx->ctx.addr);
	printk("Network dst 0x%04x \n", rx->dst);
	printk("Network recieved TL 0x%02x \n", rx->ctx.send_ttl);
	*/

	/* The RWAIT was received by the flooded RREQ originator */
	if (data->source_address == bt_mesh_primary_addr())
	{
		if (data->hop_count == 0)
		{
			data->hop_count++;
		}
		/* If the destination isn't found in the valid destination,
 		 *  then it hasn't been verified yet which means it's still
		 *  in the ring search function
		*/
		if (!bt_mesh_search_valid_destination(rx->ctx.addr, rx->dst, &temp))
		{
			/* Insert a new node in the rrep_rwait_list */
			struct rrep_rwait_list_entry temp_entry;
			struct rrep_rwait_list_entry *rrep_entry=&temp_entry;
			rrep_entry->destination_address = data->destination_address;
			rrep_entry->hop_count 					= data->hop_count;
			rrep_rwait_list_create_entry(rrep_entry);
		}
	}
	/* RWAIT is received by an intermediate node */
	else {
		/* FIXME: Comment this section */
		if (!bt_mesh_search_invalid_destination(rx->ctx.addr, rx->dst, &temp))
		{
			/* FIXME: remove the struct */
			struct rwait_data send_data = {
				.destination_address 		= data->destination_address,
				.source_address 				= data->source_address,
				.source_sequence_number = data->source_sequence_number,
				.hop_count 							= data->hop_count,
			};
			rwait_send(NULL,NULL,send_data, rx, true);
		}
		else {
			printk("RWait has been dropped");
		}
	}
}
