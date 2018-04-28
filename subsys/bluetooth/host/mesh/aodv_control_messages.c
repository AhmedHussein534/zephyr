/** @file ctrMsg.c
 *  @brief Routng Control Messages File
 *
 *  Bluetooth routing control messages following AODV protocol.
 *	The file contains RREQ, RREP and RWAIT data and functions.
 *  @bug No known bugs.
 */

/* -- Includes -- */
#include <zephyr.h>
#include <errno.h>
#include <net/buf.h>
#include <bluetooth/mesh.h>

#include "common/log.h"
#include "mesh.h"
#include "net.h"
#include "transport.h"
#include "access.h"
#include "foundation.h"

//TODO:: Surround with configuration parameter
#include "routing_table.h"
#include "aodv_control_messages.h"

#define BT_DBG_ENABLED IS_ENABLED(CONFIG_BT_MESH_DEBUG_ROUTING)

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
 *  @brief Defines a linked list used to store the RERRs before sending them
 * 				- When a node doesn't recieve a hello message from one of its neightbours,
 *						it starts searching in its routing table to determine which neighbours it should
 *						inform with the unreachable node. For each neighbour a RERR is created and saved in
 *						RERR list till searching through the table is over. Hence, this list is looped over
 * 						and for each entry in this list the RERR is sent.
 *				- When a node receives a RERR, it starts searhcing in its routing table to determine which 
 *						concerned neighbours it should inform that the destinations received in the RERR are
 *						not reachable anymore.
 */

sys_slist_t rerr_list;
struct k_mem_slab rerr_slab;
/**
 *  @brief Defines a linked list used to store the hello messages.
 * 				- When a route is established to a certain destination, the next hop to this destination
 *						is stored in this list. 
 *
 *				- If a hello message for a node in this list isn't received within the hello message timer  
 *						interval, this node is considered unreachable and a RERR is established to inform all  
 *						the nodes that use the unreachable node as their next hop to delete its route entry. 
 */
sys_slist_t hello_msg_list;
struct k_mem_slab hello_msg_slab;


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
#define rerr_list_NUMBER_OF_ENTRIES 20
#define RERR_ENTRY_SIZE sizeof(struct rerr_list_entry)
#define hello_msg_list_NUMBER_OF_ENTRIES 20
#define HELLO_MSG_ENTRY_SIZE sizeof(struct hello_msg_list_entry)


K_MEM_SLAB_DEFINE(rrep_slab, RREP_ENTRY_SIZE, rrep_rwait_list_NUMBER_OF_ENTRIES, ALLIGNED);
K_SEM_DEFINE(rrep_rwait_list_sem, 1, 1);  /* Binary semaphore for RREP linked list critical section */
K_MEM_SLAB_DEFINE(rerr_slab, RERR_ENTRY_SIZE, rerr_list_NUMBER_OF_ENTRIES, ALLIGNED);
K_SEM_DEFINE(rerr_list_sem, 1, 1);  /* Binary semaphore for RERR linked list critical section */
K_MEM_SLAB_DEFINE(hello_msg_slab, HELLO_MSG_ENTRY_SIZE, hello_msg_list_NUMBER_OF_ENTRIES, ALLIGNED);
K_SEM_DEFINE(hello_msg_list_sem, 1, 1);  /* Binary semaphore for hello message linked list critical section */


/* FUNCTIONS PROTOTYPES */
static int rreq_send(struct rreq_data *data, u8_t TTL, u16_t net_idx);
static void rreq_recv_cb(struct k_timer *timer_id);
static void ring_search_timer(struct k_timer *timer_id);
static int rrep_send(struct rrep_data *RREP_msg,u16_t net_idx, u16_t dst );
static int rrep_rwait_list_create_entry(struct rrep_rwait_list_entry *entry_data);
static void rwait_send(struct rreq_data* rreq_recv_data,struct bt_mesh_route_entry *destination_entry,
						struct rwait_data rwait_data, struct bt_mesh_net_rx* rx, bool relay);
static bool rerr_send(struct rerr_list_entry *data);
static bool bt_mesh_search_rerr_list(u16_t next_hop,u16_t net_idx,struct rerr_list_entry **entry);
void search_callback(struct bt_mesh_route_entry *entry_found, struct bt_mesh_route_entry **temp);
static bool bt_mesh_create_rerr_entry(struct rerr_list_entry **entry_data);
static void bt_mesh_delete_rerr_entry(struct rerr_list_entry *entry);
static void view_rerr_list();
static bool is_empty_rerr_list();
static bool is_empty_hello_msg_list();
static bool hello_msg_list_search_entry(u16_t src, struct hello_msg_list_entry **entry_data);
static void bt_mesh_delete_hello_msg_entry(struct hello_msg_list_entry *entry);
static void bt_mesh_delete_hello_msg_entry_timer(struct k_timer *timer_id);
void add_neighbour(u16_t neighbour, u16_t net_idx);
void remove_neighbour(u16_t neighbour, u16_t net_idx);
/* _TEST_ */
//struct k_timer hello_send;        				/* hello timer (52B) */
/* _TEST_ */


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
 *	@return : 0 on success. Otherwise, sending control message failed 
 */
static int rreq_send(struct rreq_data *data, u8_t TTL, u16_t net_idx)
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

	BT_DBG("source_address 0x%04x destination_address 0x%04x next_hop 0x%04x",	
	 data->source_address, data->destination_address,data->next_hop);	
	BT_DBG("source_number_of_elements %04x hop_count %01x source_sequence_number %08x",	
		data->source_number_of_elements, data->hop_count, data->source_sequence_number)	;
	BT_DBG("destination_sequence_number  %08x ", data->destination_sequence_number);

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
	return err;
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
	add_neighbour(entry->next_hop, entry->net_idx);

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
 *	@return  0 on Success
 * 			 -ELOCAL when source address is a local element	
 *			 -ENORREQ when RREP interval has expired
 */
int bt_mesh_trans_rreq_recv(struct bt_mesh_net_rx *rx, struct net_buf_simple *buf)
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
	BT_DBG("source_address 0x%04x destination_address 0x%04x next_hop 0x%04x",	
	 data->source_address, data->destination_address,data->next_hop);	
	BT_DBG("source_number_of_elements %04x hop_count %01x source_sequence_number %08x",	
		data->source_number_of_elements, data->hop_count, data->source_sequence_number);	
	BT_DBG("destination_sequence_number  %08x ", data->destination_sequence_number);

	struct bt_mesh_route_entry *entry = NULL;
	/* If element is requesting data transaction from an element in the same node,
	 * Drop rreq. This case is to prevent receiving RREQ from neighbouring elements */
	if (bt_mesh_elem_find(data->source_address))
	{
		BT_ERR("Source address is a local element");
		return -ELOCAL;
	}
	/* If a RREQ is received by the destination node */
	else if (bt_mesh_elem_find(data->destination_address))
	{
		/* Drop any received RREQ after the expiry of the ring search timer */
		if (bt_mesh_search_valid_destination(data->destination_address, data->source_address, &entry)) {
			BT_ERR("RREQ dropped - RREQ received after RREP Interval");
			return -ENORREQ;
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
 *	@return :0 on success, -ENORREP if RREP interval has expired
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
				return 0;
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
				BT_ERR("max TTL is reached. Ring search has stopped");
				return -ENORREP;
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
 *	@return : 0 on success. Otherwise, sending control message failed
 */
static int rrep_send(struct rrep_data *data,u16_t net_idx, u16_t destination_address )
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
	BT_DBG("source_address 0x%04x destination_address 0x%04x destination_sequence_number 0x%08x",	
	 data->source_address, data->destination_address,data->destination_sequence_number);	
	BT_DBG("hop_count %01x destination_number_of_elements %04x",	
		data->hop_count, data->destination_number_of_elements);

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
 *	@return : 0 on sucess, -ENOSR if memory allocation timeout
 */
static int rrep_rwait_list_create_entry(struct rrep_rwait_list_entry *entry_data)
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
	else
	{
		/* Memory Allocation timeout */
		BT_ERR("Memory Allocation timeout");
		return -ENOSR;
	}
	entry_location -> destination_address = entry_data -> destination_address;
	entry_location -> hop_count = entry_data -> hop_count;
	return 0;
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
 *	@return : 0 on sucess, -ENOSR if memory allocation timeout
 */
int bt_mesh_trans_rrep_recv(struct bt_mesh_net_rx *rx, struct net_buf_simple *buf)
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

	BT_DBG("source_address 0x%04x destination_address 0x%04x destination_sequence_number 0x%08x",	
	 data->source_address, data->destination_address,data->destination_sequence_number);	
	BT_DBG("hop_count %01x destination_number_of_elements %04x",	
		data->hop_count, data->destination_number_of_elements);	

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
				add_neighbour(table_entry->next_hop, table_entry->net_idx);

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
			add_neighbour(existing_entry->next_hop, existing_entry->net_idx);

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
				add_neighbour(table_entry->next_hop, table_entry->net_idx);

			}

			data->hop_count++;
			rrep_send(data, rx -> ctx.net_idx ,existing_entry->next_hop);
		}
	}
	return 0;
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
 *	@param relay: bool flag to determine whether this node has the destination entry or if it's only relaying a preformed RWait 
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
	BT_DBG("source_address 0x%04x Destination Address 0x%04x Hop Count 0x%01x",	
	 data.source_address, data.destination_address,data.hop_count);	

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
		/* XXX: Comment this section */
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


/* _TEST_ */
//static void hello_publish(struct k_timer *timer_id);
/* _TEST_ */

/**
 *	@brief Initializes the rerr_list
 *
 *	@param N/A
 *
 *	@return N/A
 */
void bt_mesh_trans_rerr_list_init()
{
	sys_slist_init(&rerr_list);
  /* _TEST_ */
  //k_timer_init (&hello_send, hello_publish, NULL);
  //k_timer_start(&hello_send, HELLO_MSG_SEND, 0);
  /* _TEST_ */
}

/**
*	@brief Search in the rerr list by the next hop and the network index.
*
*	@param next_hop
*	@param net_idx: network index
*	@param entry: Pointer to structure of type rerr_list_entry
*
*	@return
*			- Explicit: True when found, False otherwise.
*			- Implicit: Pointer to the found entry (3rd param).
*/

bool bt_mesh_search_rerr_list(u16_t next_hop,u16_t net_idx,struct rerr_list_entry **entry)
{
	struct rerr_list_entry *entry1 = NULL;
	k_sem_take(&rerr_list_sem, K_FOREVER);
	SYS_SLIST_FOR_EACH_CONTAINER(&rerr_list, entry1, node)
	{
		if ((entry1->next_hop==next_hop) &&(entry1->net_idx==net_idx))
		{
			k_sem_give(&rerr_list_sem);
			*entry = entry1; //FIXME entry and entry1 might later point to deleted entries by another thread
			return true;
		}
	}
	k_sem_give(&rerr_list_sem);
	return false;
}


/**
 *	@brief Create entry in the rerr list.
 *
 *	@param entry_data: Pointer to structure of type rerr_list_entry
 *										 holding data to be stored.
 *
 *	@return True when allocation succeeds, False when no space is available.
 */

bool bt_mesh_create_rerr_entry(struct rerr_list_entry **entry_location)
{
	/* Insert a new node into rerr list */
	if (k_mem_slab_alloc(&rerr_slab, (void **)&(*entry_location), 100) == 0)
	{
		memset((*entry_location), 0, RERR_ENTRY_SIZE);
		k_sem_take(&rerr_list_sem, K_FOREVER);
		sys_slist_append(&rerr_list, &(*entry_location)->node);
		k_sem_give(&rerr_list_sem);
	}
	else
	{
		/* Memory Allocation timeout */
		return false;
	}
	return true;
}
/**
 *	@brief Delete rerr entry when lifetime expires.
 *
 *	@param entry: Pointer to struct of type rerr_list_entry of the entry to be deleted.
 */

void bt_mesh_delete_rerr_entry(struct rerr_list_entry *entry )
{
		k_sem_take(&rerr_list_sem, K_FOREVER);   							/* take semaphore */
		sys_slist_find_and_remove(&rerr_list, &entry->node);   /*delete node from linked list */
		k_sem_give(&rerr_list_sem);                            /*return semaphore */
		k_mem_slab_free(&rerr_slab, (void **)&entry);  /*free space in slab*/
}
/**
 *	@brief Called when a RERR needs to be sent. It sets the content of ctx and
 *				 tx structs, the values of RERR data and sends the constructed values
 *				 to the transport layer.
 *
 *	@param data: Pointer to a structure of type rerr_list_entry that holds the data
 * 							 received from the transport and network layer.
 *
 *	@return : 0 on success. Otherwise, sending control message failed RANA
 */

static bool rerr_send(struct rerr_list_entry *data)
{
	/*only used by intermediate nodes*/
	struct bt_mesh_msg_ctx ctx =
	{
		.app_idx  = BT_MESH_KEY_UNUSED,
		.net_idx  = data->net_idx,
		.send_ttl = 3  /* FIXME */
	};

	struct bt_mesh_net_tx tx = {
		.sub = bt_mesh_subnet_get(data->net_idx),
		.ctx = &ctx,
		.xmit = bt_mesh_net_transmit_get(),
	};
	tx.ctx->addr = data->next_hop;
	tx.src = bt_mesh_primary_addr();
	tx.ctx->send_ttl--;

	BT_DBG("RERR Send: \n");
	BT_DBG("destination_number =%01x : ", data->destination_number);
	BT_DBG("sent to =%04x : ", data->next_hop);

	/* Create a buffer for RRER data */
	NET_BUF_SIMPLE_DEFINE (buf, BT_MESH_TX_SDU_MAX);
	net_buf_simple_add_mem(&buf, &data->destination_number, 1);
	for (int i=0; i<data->destination_number && i< 10 ;i++) //FIXME
	{
		net_buf_simple_add_mem(&buf, &data->destination_address[i], 2);
		net_buf_simple_add_mem(&buf, &data->destination_sequence_number[i], 3);
		BT_DBG("destination_address =%04x , destination_sequence_number = %04x  ", data->destination_address[i], data->destination_sequence_number[i]);

	}
	
	return bt_mesh_ctl_send(&tx, TRANS_CTL_OP_RERR, buf.data,
				buf.len, NULL, NULL, NULL);
}

/**
 *	@brief Called by ctl_recv in the transport layer when the OP code refers to
 *				 a RERR.
 *
 *	@param rx: Pointer to a structure of type bt_mesh_net_rx that holds
 *						 the received network layer data.
 *	@param buf: Pointer to a structure of type net_buf_simple that holds
 *							the received RERR data.
 *
 *	@return : 0 on sucess, -ENOSR if memory allocation timeout RANA
 */
bool bt_mesh_trans_rerr_recv(struct bt_mesh_net_rx *rx, struct net_buf_simple *buf)
{
	struct rerr_list_entry temp;
	struct rerr_list_entry *data = &temp;
	data->destination_number= RERR_GET_DST_NUM(buf);
	u16_t destination_address;
	u32_t destination_sequence_number;
	/*Loop to obtain all destinations inside the buffer */
	BT_DBG("RERR RECV: \n");
	printk("RERR RECV: \n");
	BT_DBG("destination_number =%01x : ", data->destination_number);
	printk("destination_number =%01x : ", data->destination_number);
	
	for (int i=0; i<data->destination_number;i++)
	{
		destination_address = RERR_GET_DST_ADDR(buf,i*2 + i*3 +1);
		destination_sequence_number= RERR_GET_DST_SEQ_NUM(buf,i*2 + i*3 +1);
 		BT_DBG("destination_address =%04x , destination_sequence_number = %04x  ", destination_address, destination_sequence_number);
 		/*invoke the function destination_address destination_sequence_number */
		bt_mesh_search_valid_destination_nexthop_net_idx_with_cb(destination_address,rx->ctx.addr,rx->ctx.net_idx,search_callback);
	
	}
	BT_DBG("received from =%04x : ", rx->dst);

		/*	loop over the RERR list and send each entry	*/
	struct rerr_list_entry* rerr_rx_entry=NULL;
	//k_sem_take(&rerr_list_sem, K_FOREVER);
	while(!is_empty_rerr_list())
	{SYS_SLIST_FOR_EACH_CONTAINER(&rerr_list, rerr_rx_entry, node)
	{
		rerr_send(rerr_rx_entry);
	 bt_mesh_delete_rerr_entry(rerr_rx_entry);
	}
}
	//k_sem_give(&rerr_list_sem);
	view_valid_list();
  view_invalid_list();
  view_hello_msg_list();
  view_rerr_list();

	return true;
}

/**
 *	@brief A callback function called each time an entry that matches is found. It creates the rerr ir 
 *				 
 *
 *	@param entry_found: Pointer to a structure of type bt_mesh_route_entry that holds
 *						 the received network layer data.
 *	@param buf: Pointer to a structure of type net_buf_simple that holds
 *							the received RERR data.
 *
 *	@return : 0 on sucess, -ENOSR if memory allocation timeout RANA
 */

void search_callback(struct bt_mesh_route_entry *entry_found,struct bt_mesh_route_entry **temp)
{					/*Terminal node*/
	if ( entry_found->source_address==bt_mesh_primary_addr())
	{
		bt_mesh_invalidate_route(entry_found); 
		remove_neighbour(entry_found->next_hop, entry_found->net_idx);
	}
	else /*I-node*/
	{
		struct bt_mesh_route_entry *entry = NULL;
		struct rerr_list_entry *rerr_entry=NULL;
		bt_mesh_search_valid_destination_with_net_idx(entry_found->destination_address,entry_found->source_address,entry_found->net_idx,&entry);
		if (bt_mesh_search_rerr_list(entry->next_hop,entry->net_idx, &rerr_entry)) 
		{
			/*Add another dst*/
    		bool flag=true;
      		for (int i=0; i<rerr_entry->destination_number;i++)
            {
            	if (rerr_entry->destination_address[i]==entry_found->destination_address)
                    flag=false;
            }
        	if(flag)
			{
        		rerr_entry->destination_address[rerr_entry->destination_number]=entry_found->destination_address;
				rerr_entry->destination_sequence_number[rerr_entry->destination_number]=entry_found->destination_sequence_number;
				rerr_entry->destination_number++ ;
      		}
		}
		else /*create RERR entry */
		{
			bt_mesh_create_rerr_entry(&rerr_entry);
			rerr_entry->destination_number=1;
			rerr_entry->next_hop=entry->next_hop;
			rerr_entry->net_idx=entry_found->net_idx;
			rerr_entry->destination_address[0]=entry_found->destination_address;
			rerr_entry->destination_sequence_number[0]=entry_found->destination_sequence_number;
		}


			bt_mesh_invalidate_route(entry_found);
			sys_snode_t * temp_node=sys_slist_peek_next(&((*temp)->node));
			if(entry==(*temp) && (*temp) !=NULL && temp_node!= NULL)
				(*temp)=CONTAINER_OF(temp_node,struct bt_mesh_route_entry,node);
			remove_neighbour(entry_found->next_hop, entry_found->net_idx);		
			bt_mesh_invalidate_route(entry);
			remove_neighbour(entry->next_hop, entry->net_idx);
	}
}
/**
 *	@brief Displays the entries of the RERR list
 *				 
 *	@return : N/A
 */
void view_rerr_list()
{
	if (sys_slist_is_empty(&rerr_list)) {
		printk("rerr is empty \n");
		return;
	}
	struct rerr_list_entry *entry = NULL;
	k_sem_take(&rerr_list_sem, K_FOREVER);
	SYS_SLIST_FOR_EACH_CONTAINER(&rerr_list, entry, node){
		printk("\x1b[34mRERR List:destination number=%04x,nexthop address=%04x \x1b[0m\n", entry->destination_number, entry->next_hop);
	}
	k_sem_give(&rerr_list_sem);
}


bool is_empty_rerr_list()
{
	if (sys_slist_is_empty(&rerr_list)) {
		printk("RERR List is empty \n");
		return true;
	}
	return false;
}

bool is_empty_hello_msg_list()
{
	if (sys_slist_is_empty(&hello_msg_list))
	{
		printk("Hello msg List is empty \n");
		BT_DBG("Hello msg List is empty");	
		return true;
	}
	return false;
}



void bt_mesh_delete_hello_msg_entry_timer(struct k_timer *timer_id)
{
	//printk("timer expired \n");
	/*fetching the entry of the expired timer to get its next hop*/
	struct hello_msg_list_entry *entry = CONTAINER_OF(timer_id, struct hello_msg_list_entry, lifetime);
	printk("timer expired for source address=%04x \n", entry->source_address);
	BT_DBG("timer expired for source address=%04x \n", entry->source_address);
	/*start searching for the entry in the valid list */
	bt_mesh_search_valid_nexthop_net_idx_with_cb(entry->source_address,entry->net_idx,search_callback);
	/*Loop ends*/

	/*	loop over the RERR list and send each entry	*/
	struct rerr_list_entry* rerr_rx_entry=NULL;
	//k_sem_take(&rerr_list_sem, K_FOREVER);
	while(!is_empty_rerr_list())
	{
		SYS_SLIST_FOR_EACH_CONTAINER(&rerr_list, rerr_rx_entry, node)
		{ printk(" Sending RERR to nexthop %04x \n",rerr_rx_entry->next_hop);
			rerr_send(rerr_rx_entry);
			bt_mesh_delete_rerr_entry(rerr_rx_entry); //TODO?
		}
	}
	//k_sem_give(&rerr_list_sem);

	/*delete Hello Error entry*/
	//struct hello_msg_list_entry* hello_msg_entry=NULL;
	//k_sem_take(&hello_msg_list_sem, K_FOREVER);
	//while(!is_empty_hello_msg_list())
	//{
		//SYS_SLIST_FOR_EACH_CONTAINER(&hello_msg_list, hello_msg_entry, node)
		//{
      bt_mesh_delete_hello_msg_entry(entry);
			//bt_mesh_delete_hello_msg_entry(hello_msg_entry); //TODO?
		//}
	//}
	//k_sem_give(&hello_msg_list_sem);
	view_valid_list();
  view_invalid_list();
  view_hello_msg_list();
  view_rerr_list();
	return; /*TODO*/

}
/*Reham*/
void bt_mesh_delete_hello_msg_entry(struct hello_msg_list_entry *entry )
{
	printk("bt_mesh_delete_hello_msg_entry \n");
	k_sem_take(&hello_msg_list_sem, K_FOREVER);   							/* take semaphore */
	sys_slist_find_and_remove(&hello_msg_list, &entry->node);   /*delete node from linked list */
	k_sem_give(&hello_msg_list_sem);                            /*return semaphore */
	k_mem_slab_free(&hello_msg_slab, (void **)&entry);  /*free space in slab*/
	view_hello_msg_list();		
}

/* _TEST_*/
// static void hello_publish(struct k_timer *timer_id)
// {
//  k_timer_start(&hello_send, HELLO_MSG_SEND, 0);
//  u16_t feat = 0;
//  struct __packed {
//         u8_t  init_ttl;
//         u16_t feat;
//       } hb;
//
//   struct bt_mesh_msg_ctx ctx = {
//     //.net_idx = entry_location->net_idx, //FIXME???
//     .app_idx = BT_MESH_KEY_UNUSED,
//     .addr = BT_MESH_ADDR_ALL_NODES,
//     .send_ttl = 2,
//   };
//   struct bt_mesh_net_tx tx = {
//   //  .sub = bt_mesh_subnet_get(entry_location->net_idx), //FIXME???
//     .ctx = &ctx,
//     .src = bt_mesh_primary_addr(),
//     .xmit = bt_mesh_net_transmit_get(),
//   };
//
//   hb.init_ttl = 2;
//
//    if (bt_mesh_relay_get() == BT_MESH_RELAY_ENABLED) {
//      feat |= BT_MESH_FEAT_RELAY;
//    }
//
//    if (bt_mesh_gatt_proxy_get() == BT_MESH_GATT_PROXY_ENABLED) {
//      feat |= BT_MESH_FEAT_PROXY;
//    }
//
//    if (bt_mesh_friend_get() == BT_MESH_FRIEND_ENABLED) {
//      feat |= BT_MESH_FEAT_FRIEND;
//    }
//
//   #if defined(CONFIG_BT_MESH_LOW_POWER)
//    if (bt_mesh.lpn.state != BT_MESH_LPN_DISABLED) {
//      feat |= BT_MESH_FEAT_LOW_POWER;
//    }
//   #endif
//
//    hb.feat = sys_cpu_to_be16(feat);
//
//    BT_DBG("InitTTL %u feat 0x%04x", hb.init_ttl, feat);
//    bt_mesh_ctl_send(&tx, TRANS_CTL_OP_HEARTBEAT, &hb, sizeof(hb),
//         NULL, NULL, NULL);
//
// }
/* _TEST_*/

int hello_msg_list_create_entry(struct hello_msg_list_entry **entry_location)
{

	/*if space found in slab, Allocate new node */
	if (k_mem_slab_alloc(&hello_msg_slab, (void **)&(*entry_location), 100) == 0)
	{
		memset((*entry_location), 0, HELLO_MSG_ENTRY_SIZE);                  /* Initializing with zeros */
		k_sem_take(&hello_msg_list_sem, K_FOREVER);               /*take semaphore */
		sys_slist_append(&hello_msg_list, &(*entry_location)->node); /*insert node in linkedlist */
		k_sem_give(&hello_msg_list_sem);
	}
	else
	{
		printk("Memory Allocation timeout \n");
		return false; //TODO : fix return
	}

	/* Start the lifetime timer */
	k_timer_init (&(*entry_location)->lifetime, bt_mesh_delete_hello_msg_entry_timer, NULL);
	k_timer_start(&(*entry_location)->lifetime, HELLO_MSG_LIFETIME, 0);
  return true;//TODO :  fix return
}



bool hello_msg_list_search_entry(u16_t src, struct hello_msg_list_entry **entry_data)
{
	struct hello_msg_list_entry *entry1 = NULL;
	k_sem_take(&hello_msg_list_sem, K_FOREVER);
	SYS_SLIST_FOR_EACH_CONTAINER(&hello_msg_list, entry1, node)
	{
		if (src==entry1->source_address)
		{
			k_sem_give(&hello_msg_list_sem);
			*entry_data = entry1;
			return true;//TODO: fix return
		}

	}
	k_sem_give(&hello_msg_list_sem);
	return false;	//TODO: fix return
}


void bt_mesh_trans_hello_msg_recv(u16_t src)
{

  	struct hello_msg_list_entry temp_entry;
  	struct hello_msg_list_entry *entry = &temp_entry;
	entry->source_address=src;
    printk("hb source is: %04x",src);
    BT_DBG("hb source is: %04x",src);
  	if (hello_msg_list_search_entry(src, &entry))
  	{

//	k_timer_init (&(*entry_location)->lifetime, bt_mesh_delete_hello_msg_entry_timer, NULL);
//	k_timer_start(&(*entry_location)->lifetime, HELLO_MSG_LIFETIME, 0);

  		printk("entry foun src is %04x \n",entry->source_address);
		k_timer_stop(&entry->lifetime);
		struct k_timer temp_timer;
		entry->lifetime = temp_timer;
		k_timer_init(&entry->lifetime,bt_mesh_delete_hello_msg_entry_timer, NULL);
    	BT_DBG("COOL.");
    	printk("COOL.");

		k_timer_start(&entry->lifetime, HELLO_MSG_LIFETIME, 0);
		view_hello_msg_list();
  	}
  	else
  	{
  		BT_DBG("Hello message receivd from a node not of interest.");
  	}

}

void add_neighbour(u16_t neighbour, u16_t net_idx)
{
	printk("add_neighbour \n");
	/*search for this neighbour in the hello msg list, if not exist , create a new entry*/
  	struct hello_msg_list_entry temp_entry;
  	struct hello_msg_list_entry *entry = &temp_entry;
	entry->source_address=neighbour;
    printk("hb source is: %04x",neighbour);
    BT_DBG("hb source is: %04x",neighbour);
  	if (!hello_msg_list_search_entry(neighbour, &entry))
	{
		struct hello_msg_list_entry  temp_entry_hello;
		struct hello_msg_list_entry  *entry_hello=&temp_entry_hello;
		hello_msg_list_create_entry(&entry_hello);
		entry_hello->source_address=neighbour;
		entry_hello->net_idx=net_idx;
		view_hello_msg_list();
		view_valid_list();
	}

}

void remove_neighbour(u16_t neighbour, u16_t net_idx)
{
	/*check if there's no other route entry needs this neighbout. If not, remove the entry from the hello_msg list*/
	printk("remove_neighbour \n");
	struct bt_mesh_route_entry *entry;
	view_valid_list();
	if(!bt_mesh_search_valid_next_hop_with_net_idx(neighbour, net_idx, &entry))
	{
	  	struct hello_msg_list_entry temp_entry;
	  	struct hello_msg_list_entry *hello_msg_entry = &temp_entry;
		hello_msg_entry->source_address=neighbour;
	    printk("hb source is: %04x",neighbour);
	    BT_DBG("hb source is: %04x",neighbour);

	  	if (hello_msg_list_search_entry(neighbour, &hello_msg_entry))
		{
			printk("src to be deleted is %04x \n",neighbour);
			bt_mesh_delete_hello_msg_entry(hello_msg_entry);
			view_hello_msg_list();			
		}

	}
}

void view_hello_msg_list()
{
	if (is_empty_hello_msg_list())
	{
		return;
	}
	struct hello_msg_list_entry *entry = NULL;
	k_sem_take(&hello_msg_list_sem, K_FOREVER);
	SYS_SLIST_FOR_EACH_CONTAINER(&hello_msg_list, entry, node){
	printk("\x1b[32m Hello msg List:source address=%04x\x1b[0m \n", entry->source_address);
	//printk("Valid List:hop count=%01x \n", entry->hop_count);

	}
	k_sem_give(&hello_msg_list_sem);
}
