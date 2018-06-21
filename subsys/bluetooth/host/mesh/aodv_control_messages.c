/** @file aodv_control_messages.c
 *  @brief Routng Control Messages File
 *
 *  Bluetooth routing control messages following AODV protocol.
 *	The file contains RREQ, RREP, RWAIT and RERR data and functions.
 *  @bug No known bugs.
 */

/* -- Includes -- */
#include <zephyr.h>
#include <errno.h>
#include <net/buf.h>
#include <bluetooth/mesh.h>

#define BT_DBG_ENABLED IS_ENABLED(CONFIG_BT_MESH_DEBUG_ROUTING)
#include "common/log.h"
#include "mesh.h"
#include "net.h"
#include "transport.h"
#include "access.h"
#include "foundation.h"

//TODO:: Surround with configuration parameter
#include "aodv_control_messages.h"
#include "routing_table.h"



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


struct k_mem_slab destination_slab;

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
#define DESTINATION_LIST_NUMBER_OF_ENTRIES 20
#define DESTINATION_ENTRY_SIZE sizeof(struct destination_list_entry)
#define rerr_list_NUMBER_OF_ENTRIES 20
#define RERR_ENTRY_SIZE sizeof(struct rerr_list_entry)
#define hello_msg_list_NUMBER_OF_ENTRIES 20
#define HELLO_MSG_ENTRY_SIZE sizeof(struct hello_msg_list_entry)


K_MEM_SLAB_DEFINE(rrep_slab, RREP_ENTRY_SIZE, rrep_rwait_list_NUMBER_OF_ENTRIES, ALLIGNED);
K_SEM_DEFINE(rrep_rwait_list_sem, 1, 1);  /* Binary semaphore for RREP linked list critical section */
K_MEM_SLAB_DEFINE(destination_slab, DESTINATION_ENTRY_SIZE,  DESTINATION_LIST_NUMBER_OF_ENTRIES, ALLIGNED);
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
static int destination_list_create_entry(struct destination_list_entry **entry_location,sys_slist_t *destination_list);
static void destination_list_delete_entry(struct destination_list_entry *entry, sys_slist_t *destination_list );
/*static void view_destination_list(sys_slist_t *destination_list);*/
static int rerr_send(struct rerr_list_entry *data);
static bool rerr_list_search_entry(u16_t next_hop,u16_t net_idx,struct rerr_list_entry **entry);
void search_callback(struct bt_mesh_route_entry *entry_found, struct bt_mesh_route_entry **temp);
static int rerr_list_create_entry(struct rerr_list_entry **entry_data);
static void rerr_list_delete_entry(struct rerr_list_entry *entry);
static void view_rerr_list();
static bool is_empty_rerr_list();
static bool is_empty_hello_msg_list();
static bool hello_msg_list_search_entry(u16_t src, struct hello_msg_list_entry **entry_data);
static void hello_msg_list_delete_entry(struct hello_msg_list_entry *entry);
static void hello_msg_list_entry_expiry_fn(struct k_timer *timer_id);
static void add_neighbour(u16_t neighbour, u16_t net_idx);
void remove_neighbour(u16_t neighbour, u16_t net_idx);

static void overhead_control (unsigned int len)
{
	unsigned int n= (len-1)/8 ;       //number of segments -  1
	unsigned int overhead =0;
	if (len>11)              //segmented or data
	{
	 overhead=(n+1)*(9+4+8)+len;
	}
	else {
	overhead = 9+1+8+len;
	}
	printk("[GUI] PktOverhead - %d",overhead);
	return;
}


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
	if (data->I && bt_mesh_search_valid_destination_without_source(data->destination_address,net_idx, &entry))
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
	net_buf_simple_add_mem(&buf, &data->source_address, 2);
	net_buf_simple_add_mem(&buf, &data->destination_address, 2);
	net_buf_simple_add_mem(&buf, &data->source_number_of_elements, 2);
	net_buf_simple_add_mem(&buf, &data->hop_count, 1);
	net_buf_simple_add_mem(&buf, &data->rssi, 1);
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
	BT_DBG("  <<<<<<<<<<<< rreq_recv_cb >>>>>>>>>>>>>> ");
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
 *			 -ENOSR if memory allocation timeout
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
	data->rssi=(RREQ_GET_RSSI(buf) * (data->hop_count ) + rx->rssi)/(data->hop_count + 1);
	data->next_hop = rx->ctx.addr;
	data->G = RREQ_GET_G_FLAG(buf);
	data->D = RREQ_GET_D_FLAG(buf);
	data->U = RREQ_GET_U_FLAG(buf);
	data->I = RREQ_GET_I_FLAG(buf);
	data->destination_sequence_number = RREQ_GET_DST_SEQ(buf);
	data->source_sequence_number = RREQ_GET_SRC_SEQ(buf);
	overhead_control(buf->len);

	BT_DBG("RREQ:source_address 0x%04x destination_address 0x%04x next_hop 0x%04x",
	 data->source_address, data->destination_address,data->next_hop);
	BT_DBG("RREQ:source_number_of_elements %04x hop_count %01x source_sequence_number %08x",
		data->source_number_of_elements, data->hop_count, data->source_sequence_number)
	BT_DBG("RREQ:destination_sequence_number  %08x ", data->destination_sequence_number)
	BT_DBG("RREQ:RSSI average = %d",data->rssi);

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
		//TODO: compare sequence number before dropping packets
		if (bt_mesh_search_valid_destination(data->destination_address, data->source_address, rx->ctx.net_idx, &entry)) {
			BT_ERR("RREQ dropped - RREQ received after RREP Interval");
			return -ENORREQ;
		}
		//TODO: check for existing routes?
		/* Multiple RREQs are received in the interval of ring search timer */
		else if (bt_mesh_search_invalid_destination(data->destination_address, data->source_address, rx->ctx.net_idx, &entry))
		{
			/* If it contains better data, replace */
			if ((data->hop_count*10+(data->rssi*10)/RSSI_MIN) < (entry->hop_count*10+(entry->rssi*10)/RSSI_MIN)) {
				BT_DBG("Modifying existing entry ");
				entry->destination_sequence_number = data->destination_sequence_number;
				entry->hop_count                   = data->hop_count;
				entry->next_hop                    = data->next_hop;
				entry->rssi                    		 = data->rssi;
			}
			return 0;
		}
		/* Destination has received the first RREQ */
		else {
			BT_DBG("Creating entry and waiting for RREQ wait interval ");
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
			entry_data->rssi 													 = data-> rssi;
			entry_data->net_idx 											 = rx -> ctx.net_idx;
			return 0;
			}
			else {
				return false;
			}

		}
	}
	/* Intermediate node having route to destination should
	 *   - check relay feature is enabled
	 *   - reply to RREQ originator with RWAIT
	 *   - send a directed RREQ to RREQ's destination
	*/
	 if (IS_ENABLED(CONFIG_BT_MESH_RELAY)){
	 if (bt_mesh_search_valid_destination_without_source(data->destination_address, rx->ctx.net_idx, &entry)
						&& data->D == false && data->I == false)
		{
		BT_DBG("Intermediate Node received a flooded RREQ and has route to destination ");
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
			entry_data->rssi 													 = data-> rssi;
			entry_data->net_idx 											 = rx -> ctx.net_idx;
		}
		else
		{
			return -ENOSR;
		}

		/* If the stored destination sequence number is fresher:
		 *   - reply to RREQ originator with RWAIT
		 *   - send a directed RREQ to RREQ's destination
		 */
		if (entry->destination_sequence_number >= data->destination_sequence_number)
		{
			BT_DBG("SEND RWAIT and SEND RREQ with flag I=1 ");
			data->I = 1;
			data->hop_count = data->hop_count + 1;
			data->rssi=(RREQ_GET_RSSI(buf) * (data->hop_count + 1 ) + rx->rssi)/(data->hop_count + 2);
			rreq_send(data, 1, rx->ctx.net_idx); /* To RREQ's destination */
			struct rwait_data temp; /* Dummy struct */
			entry_data->hop_count = entry -> hop_count;
			rwait_send(data,entry_data,temp,rx,false); /* To RREQ's originator */
		}
	}
	else {
		/* Intermediate nodes that has no route to destination shall relay */
		BT_DBG("Intermediate Node received a flooded RREQ - Relaying ");
		struct bt_mesh_route_entry *entry;

		/* If the reverse route wasn't created, create it */
		if (!bt_mesh_search_invalid_destination(data->destination_address, data->source_address, rx->ctx.net_idx, &entry))
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
				entry_data->rssi 													 = data-> rssi;
				entry_data->net_idx 											 = rx -> ctx.net_idx;
			}
			else
			{
				return -ENOSR;
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
			entry->rssi = data-> rssi;
			data-> hop_count = data -> hop_count + 1;
			bt_mesh_refresh_lifetime_invalid(entry);
			return rreq_send(data, rx->ctx.recv_ttl - 1, rx->ctx.net_idx);
		}
	}
}

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
int bt_mesh_trans_ring_search(struct bt_mesh_net_tx *tx)
{
	u16_t source_address = tx->src; /* Primary element source address */
	u16_t destination_address = tx->ctx->addr;
	/* The following 2 fields will be set if
	 * an invalid route is found to destination */
	u32_t destination_sequence_number = 0;
	bool U_flag = 1; /* Unknown destination sequence number flag */

	/* Create a ring search timer */
	struct ring_search_flag_timer ring_struct;
	ring_struct.ring_flag = false;
	struct rrep_rwait_list_entry *temp;
	k_timer_init(&ring_struct.ring_timer, ring_search_timer, NULL);                                 /*init lifetime timer*/
	k_timer_start(&ring_struct.ring_timer, RREQ_RING_SEARCH_WAIT_INTERVAL, RREQ_RING_SEARCH_WAIT_INTERVAL);   /* Reset Timer */

	/* Mesh specs prohibits the use of TTL = 1 */
	u8_t TTL = 2; /* Initial TTL */
	BT_DBG("current TTL=%d", TTL);
	struct bt_mesh_route_entry *entry;
	if (bt_mesh_search_invalid_rerr_destination(source_address, destination_address,tx->ctx->net_idx, &entry))
	{
		destination_sequence_number = entry->destination_sequence_number;
		BT_DBG("destination sequence number %08x ",destination_sequence_number);
		U_flag = 0;
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
	BT_DBG("Destination Address : %04x",destination_address);
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
				BT_DBG("Delaying Ring Search with hop count =%d",temp->hop_count);
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
			BT_DBG("current TTL=%d ", TTL);
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
		.send_ttl = 0
	};

	struct bt_mesh_net_tx tx = {
		.sub = bt_mesh_subnet_get(net_idx),
		.ctx = &ctx,
		.xmit = bt_mesh_net_transmit_get(),
	};
	tx.ctx->addr = destination_address;
	tx.src = bt_mesh_primary_addr();

	BT_DBG("RREP_send:source_address 0x%04x destination_address 0x%04x destination_sequence_number 0x%08x",
	 data->source_address, data->destination_address,data->destination_sequence_number);
	BT_DBG("RREP_send:hop_count %01x destination_number_of_elements %04x",
		data->hop_count, data->destination_number_of_elements)


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
	BT_DBG("  <<<<<<<<<<<< rrep_rwait_list_create_entry >>>>>>>>>>>>>> ");
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
	overhead_control(buf->len);
	/* Testing: View received RREP  */
	BT_DBG("RREP R 0x%01x,RREP source_address 0x%04x,RREP dst 0x%04x ", data->R,data->source_address,data->destination_address);
	BT_DBG("RREP seq 0x%04x,RREP hop_count 0x%02,RREP elem 0x%02x ", data->destination_sequence_number, data->hop_count, data->destination_number_of_elements);
	BT_DBG("RREP Network Src 0x%02x,Network dst 0x%02x,Network recieved TTL 0x%02x ", rx->ctx.addr,rx->dst, rx->ctx.send_ttl);





	/* If the RREP is received by the RREQ originator */
	if (data->source_address == bt_mesh_primary_addr())
	{
		struct bt_mesh_route_entry *found_entry = NULL;
		if (!bt_mesh_search_valid_destination(data->source_address, data->destination_address,rx->ctx.net_idx, &found_entry) ||
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
			else
			{
				return -ENOSR;
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
	else
	{
		struct bt_mesh_route_entry *existing_entry;
		/* Get the entry of reverse route created by RREQ */
		if (bt_mesh_search_invalid_destination_with_range(data->destination_address,data->source_address,data->destination_number_of_elements, rx->ctx.net_idx, &existing_entry))
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
			else {
				return ENOSR;  //CHECKME
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
		ctx.send_ttl = 0;
		tx.ctx  = &ctx;
	}
	else
	{
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
	overhead_control(buf->len);
	if (buf->len < sizeof(*data))
	{
		BT_WARN("Too short data");
	}

	/* Dissect the received RWAIT */
	data -> destination_address = RWAIT_GET_DST_ADDR(buf);
	data -> source_address = RWAIT_GET_SRC_ADDR(buf);
	data -> source_sequence_number = RWAIT_GET_SRC_SEQ_NUM(buf);
	data -> hop_count= RWAIT_GET_HOP_COUNT(buf);


	BT_DBG("Rwait: dst 0x%04x,src 0x%04x,src_seq 0x%08x,hop_count 0x%01x ",
	data->destination_address, data->source_address, data->source_sequence_number, data->hop_count);
	BT_DBG("Rwait Network Src 0x%04x,dst 0x%04x,TLL 0x%02x ", rx->ctx.addr, rx->dst, rx->ctx.send_ttl);

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
		if (!bt_mesh_search_valid_destination(rx->ctx.addr, rx->dst,rx->ctx.net_idx, &temp))
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
		if (!bt_mesh_search_invalid_destination(rx->ctx.addr, rx->dst,rx->ctx.net_idx, &temp))
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
			BT_DBG("RWait has been dropped");
		}
	}
}



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
}

/**
 *	@brief Create entry in the destination list.
 *
 *	@param entry_location: Pointer to pointer to structure of type destination_list_entry
 *										 holding data to be stored.
 *	@param destination_list: Pointer to sys_slist_t to which this entry should be appended
 *
 *	@return : 0 on sucess, -ENOSR if memory allocation timeout
 */
static int destination_list_create_entry(struct destination_list_entry **entry_location,sys_slist_t *destination_list)
{
	/* Insert a new node into destination list */
	if (k_mem_slab_alloc(&destination_slab, (void **)&(*entry_location), 100) == 0)
	{
		memset((*entry_location), 0, DESTINATION_ENTRY_SIZE);
		sys_slist_append(destination_list, &(*entry_location)->node);
	}
	else
	{
		/* Memory Allocation timeout */
		return -ENOSR;
	}
	return 0;
}

/**
 *	@brief Delete the destination list associated with the rerr entry.
 *
 *	@param entry_location: Pointer to structure of type destination_list_entry
 *										 holding data to be deleted.
 *	@param destination_list: Pointer to sys_slist_t from which this entry should be found and removed
 *
 *	@return : N/A
 */

static void destination_list_delete_entry(struct destination_list_entry *entry, sys_slist_t *destination_list )
{
	sys_slist_find_and_remove(destination_list, &entry->node);   /*delete node from linked list */
	k_mem_slab_free(&rerr_slab, (void **)&entry);  			/*free space in slab*/
}
/**
*	@brief Search in the rerr list by the next hop and the network index.
*
*	@param next_hop
*	@param net_idx: network index
*	@param entry: Pointer to pointer to structure of type rerr_list_entry
*
*	@return
*			- Explicit: True when found, False otherwise.
*			- Implicit: Pointer to the found entry (3rd param).
*/

bool rerr_list_search_entry(u16_t next_hop,u16_t net_idx,struct rerr_list_entry **entry)
{
	struct rerr_list_entry *iterator_entry = NULL;
	k_sem_take(&rerr_list_sem, K_FOREVER);
	SYS_SLIST_FOR_EACH_CONTAINER(&rerr_list, iterator_entry, node)
	{
		if ((iterator_entry->next_hop==next_hop) &&(iterator_entry->net_idx==net_idx))
		{
			k_sem_give(&rerr_list_sem);
			*entry = iterator_entry; //FIXME entry and entry1 might later point to deleted entries by another thread
			return true;
		}
	}
	k_sem_give(&rerr_list_sem);
	return false;
}


/**
 *	@brief Create entry in the rerr list.
 *
 *	@param entry_data: Pointer to pointer to structure of type rerr_list_entry
 *										 holding data to be stored.
 *
 *	@return : 0 on sucess, -ENOSR if memory allocation timeout
 */

int rerr_list_create_entry(struct rerr_list_entry **entry_location)
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
		return -ENOSR;
	}
	return 0;
}


/**
 *	@brief Delete rerr entry when lifetime expires.
 *
 *	@param entry: Pointer to struct of type rerr_list_entry of the entry to be deleted.
 *
 *	@return : N/A
 */

static void rerr_list_delete_entry(struct rerr_list_entry *entry )
{
	k_sem_take(&rerr_list_sem, K_FOREVER);   				/* take semaphore */
	/*delete the destination list associated with this entry first*/
	struct destination_list_entry * iterator_destination_entry=NULL;
	while(!sys_slist_is_empty(&(entry->destination_list)))
	{
		SYS_SLIST_FOR_EACH_CONTAINER(&(entry->destination_list), iterator_destination_entry, node)
		{
	 		destination_list_delete_entry(iterator_destination_entry,&(entry->destination_list));
		}
	}
	sys_slist_find_and_remove(&rerr_list, &entry->node);   /*delete node from linked list */
	k_sem_give(&rerr_list_sem);                            /*return semaphore */
	k_mem_slab_free(&rerr_slab, (void **)&entry);  			/*free space in slab*/
}
/**
 *	@brief Called when a RERR needs to be sent. It sets the content of ctx and
 *				 tx structs, the values of RERR data and sends the constructed values
 *				 to the transport layer.
 *
 *	@param data: Pointer to a structure of type rerr_list_entry that holds the data
 * 							 received from the transport and network layer.
 *
 *	@return : 0 on success. Otherwise, sending control message failed
 */

	static int rerr_send(struct rerr_list_entry *data)
{
	/*only used by intermediate nodes*/
	struct bt_mesh_msg_ctx ctx =
	{
		.app_idx  = BT_MESH_KEY_UNUSED,
		.net_idx  = data->net_idx,
		.send_ttl = 0
	};

	struct bt_mesh_net_tx tx = {
		.sub = bt_mesh_subnet_get(data->net_idx),
		.ctx = &ctx,
		.xmit = bt_mesh_net_transmit_get(),
	};
	tx.ctx->addr = data->next_hop;
	tx.src = bt_mesh_primary_addr();

	BT_DBG("RERR Send:destination_number =%01x,sent to =%04x  ",data->destination_number,data->next_hop);

	/* Create a buffer for RRER data */
	NET_BUF_SIMPLE_DEFINE (buf, BT_MESH_TX_SDU_MAX);
	net_buf_simple_add_mem(&buf, &data->destination_number, 1);


	/*loop over the destination list*/
	struct destination_list_entry *iterator_destination_entry = NULL;
	SYS_SLIST_FOR_EACH_CONTAINER(&(data->destination_list), iterator_destination_entry, node)
	{
		net_buf_simple_add_mem(&buf, &(iterator_destination_entry->destination_address), 2);
		net_buf_simple_add_mem(&buf,&(iterator_destination_entry->destination_sequence_number), 3);
		BT_DBG("destination_address =%04x , destination_sequence_number = %04x  ", (iterator_destination_entry->destination_address), (iterator_destination_entry->destination_sequence_number));
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
 *	@return : 0 on sucess
 */
int bt_mesh_trans_rerr_recv(struct bt_mesh_net_rx *rx, struct net_buf_simple *buf)
{
	struct rerr_list_entry temp;
	struct rerr_list_entry *data = &temp;
	data->destination_number= RERR_GET_DST_NUM(buf);
	u16_t destination_address;
	u32_t destination_sequence_number;
	/*Loop to obtain all destinations inside the buffer */
	BT_DBG("RERR RECV:destination_number =%01x : ", data->destination_number);
  overhead_control(buf->len);
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
	while(!is_empty_rerr_list())
	{
		SYS_SLIST_FOR_EACH_CONTAINER(&rerr_list, rerr_rx_entry, node)
		{
			rerr_send(rerr_rx_entry);
	 		rerr_list_delete_entry(rerr_rx_entry);
		}
	}
	view_valid_list();
  	view_invalid_list();
  	view_invalid_rerr_list();
  	view_hello_msg_list();
  	view_rerr_list();
	return 0;
}

/**
 *	@brief A callback function called each time an entry that matches is found.
 *			 It forms the RERR packets that will be send
 *
 *	@param entry_found: Pointer to a structure of type bt_mesh_route_entry that holds
 *						 the received network layer data.
 *	@param temp: Pointer to pointer to a structure of type bt_mesh_route_entry that holds
 *							the received RERR data.
 *
 *	@return : N/A
 */

void search_callback(struct bt_mesh_route_entry *entry_found,struct bt_mesh_route_entry **temp)
{
	BT_DBG("  <<<<<<<<<<<< search_callback >>>>>>>>>>>>>> ");
	/*Current node is a terminal node in the found path*/
	if ( entry_found->source_address==bt_mesh_primary_addr())
	{
		bt_mesh_invalidate_rerr_route(entry_found);
		remove_neighbour(entry_found->next_hop, entry_found->net_idx);
		//bt_mesh_delete_entry_link_drop(entry_found);
	}
	else /*Current node is an intermediate node in the found path*/
	{
		struct bt_mesh_route_entry *entry = NULL;
		struct rerr_list_entry *rerr_entry=NULL;
		/*looking for the entry of the path from the destination to the source */
		bt_mesh_search_valid_destination_with_net_idx(entry_found->destination_address,entry_found->source_address,entry_found->net_idx,&entry);
		if (rerr_list_search_entry(entry->next_hop,entry->net_idx, &rerr_entry))
		{
			/*Add another dst, but check first if it wasn't added before*/
    		bool flag=true;
    		/*loop over this entry destination list*/
    		struct destination_list_entry *iterator_destination_entry = NULL;
			SYS_SLIST_FOR_EACH_CONTAINER(&rerr_entry->destination_list, iterator_destination_entry, node)
			{
				if (iterator_destination_entry->destination_address==entry_found->destination_address)
				{
					flag=false;
				}
			}

        	if(flag)
			{
        		struct destination_list_entry *destination_entry=NULL;
				destination_list_create_entry(&destination_entry,&(rerr_entry->destination_list));
				destination_entry->destination_address=entry_found->destination_address;
				destination_entry->destination_sequence_number=entry_found->destination_sequence_number;
				rerr_entry->destination_number++ ;
      		}
		}
		else /*create RERR entry */
		{
			rerr_list_create_entry(&rerr_entry);
			rerr_entry->destination_number=1;
			rerr_entry->next_hop=entry->next_hop;
			rerr_entry->net_idx=entry_found->net_idx;
			struct destination_list_entry *destination_entry=NULL;
			sys_slist_init(&(rerr_entry->destination_list));
			destination_list_create_entry(&destination_entry,&(rerr_entry->destination_list));
			destination_entry->destination_address=entry_found->destination_address;
			destination_entry->destination_sequence_number=entry_found->destination_sequence_number;
		}


		sys_snode_t * temp_node=sys_slist_peek_next(&((*temp)->node));
		if(entry==(*temp) && (*temp) !=NULL && temp_node!= NULL)
			(*temp)=CONTAINER_OF(temp_node,struct bt_mesh_route_entry,node);

		bt_mesh_invalidate_rerr_route(entry_found);
		remove_neighbour(entry_found->next_hop, entry_found->net_idx);
		//bt_mesh_delete_entry_link_drop(entry_found);
		bt_mesh_invalidate_rerr_route(entry);
		remove_neighbour(entry->next_hop, entry->net_idx);
		//bt_mesh_delete_entry_link_drop(entry);

	}
}

/**
 *	@brief Displays the entries of the destination list
 *
 *	@return : N/A
 */
/*
static void view_destination_list(sys_slist_t *destination_list)
{
	BT_DBG("  <<<<<<<<<<<< view_destination_list >>>>>>>>>>>>>> ");
	if (sys_slist_is_empty(destination_list))
	{
		BT_DBG("destination_list is empty ");
		return;
	}
	struct destination_list_entry *iterator_destination_entry = NULL;
	SYS_SLIST_FOR_EACH_CONTAINER(destination_list, iterator_destination_entry, node)
	{
		BT_DBG("\x1b[34mdestination List:destination address=%04x \x1b[0m", iterator_destination_entry->destination_address);
	}
}
*/

/**
 *	@brief Displays the entries of the RERR list
 *
 *	@return : N/A
 */
void view_rerr_list()
{
	if (sys_slist_is_empty(&rerr_list))
	{
		BT_DBG("rerr is empty ");
		return;
	}
	struct rerr_list_entry *entry = NULL;
	k_sem_take(&rerr_list_sem, K_FOREVER);
	SYS_SLIST_FOR_EACH_CONTAINER(&rerr_list, entry, node)
	{
		BT_DBG("\x1b[34mRERR List:destination number=%04x,nexthop address=%04x \x1b[0m", entry->destination_number, entry->next_hop);
	}
	k_sem_give(&rerr_list_sem);
}


/**
 *	@brief Checks if the RERR list is empty.
 *
 *	@return : True  if list is empty, false otherwise.
 */
bool is_empty_rerr_list()
{
	if (sys_slist_is_empty(&rerr_list)) {
		BT_DBG("RERR List is empty ");
		return true;
	}
	return false;
}

/**
 *	@brief Checks if the Hello message list is empty.
 *
 *	@return : True  if list is empty, false otherwise.
 */
bool is_empty_hello_msg_list()
{
	if (sys_slist_is_empty(&hello_msg_list))
	{
		BT_DBG("Hello msg List is empty");
		return true;
	}
	return false;
}

/**
 *	@brief invoked the hello message timer expires.
 *
 *	@param timer_id: Pointer to struct of type k_timer holding lifetime of an entry.
 *
 *  @return : N/A
 */
void hello_msg_list_entry_expiry_fn(struct k_timer *timer_id)
{
	/*fetching the entry of the expired timer to get its next hop*/
	struct hello_msg_list_entry *entry = CONTAINER_OF(timer_id, struct hello_msg_list_entry, lifetime);
	BT_DBG("timer expired for source address=%04x", entry->source_address);
	/*start searching for the entry in the valid list */
	bt_mesh_search_valid_nexthop_net_idx_with_cb(entry->source_address,entry->net_idx,search_callback);
	/*Loop ends*/

	/*	loop over the RERR list and send each entry	*/
	struct rerr_list_entry* rerr_rx_entry=NULL;
	while(!is_empty_rerr_list())
	{
		SYS_SLIST_FOR_EACH_CONTAINER(&rerr_list, rerr_rx_entry, node)
		{
			BT_DBG(" Sending RERR to nexthop %04x ",rerr_rx_entry->next_hop);
			rerr_send(rerr_rx_entry);
			rerr_list_delete_entry(rerr_rx_entry);
		}
	}
	hello_msg_list_delete_entry(entry);
	view_valid_list();
	view_invalid_list();
	view_invalid_rerr_list();
	view_hello_msg_list();
	view_rerr_list();
}


/**
 *	@brief Delete hello message entry when lifetime expires.
 *
 *	@param entry: pointer to struct of hello_msg_list_entry holding the entry to be deleted.
 *
 *  @return : N/A
 */

void hello_msg_list_delete_entry(struct hello_msg_list_entry *entry )
{
	BT_DBG("  <<<<<<<<<<<< hello_msg_list_delete_entry >>>>>>>>>>>>>> ");
	k_sem_take(&hello_msg_list_sem, K_FOREVER);   							/* take semaphore */
	sys_slist_find_and_remove(&hello_msg_list, &entry->node);   /*delete node from linked list */
	k_sem_give(&hello_msg_list_sem);                            /*return semaphore */
	k_mem_slab_free(&hello_msg_slab, (void **)&entry);  /*free space in slab*/
	view_hello_msg_list();
}

/**
 *	@brief Create entry in the hello message list.
 *
 *	@param entry_location: Pointer to structure of type hello_msg_list_entry
 *										 holding data to be stored.
 *
 *	@return : 0 on sucess, -ENOSR if memory allocation timeout
 */

int hello_msg_list_create_entry(struct hello_msg_list_entry **entry_location)
{
	BT_DBG("  <<<<<<<<<<<< hello_msg_list_create_entry >>>>>>>>>>>>>> ");
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

		BT_ERR("Memory Allocation timeout ");
		return -ENOSR;
	}

	/* Start the lifetime timer */
	k_timer_init (&(*entry_location)->lifetime, hello_msg_list_entry_expiry_fn, NULL);
	k_timer_start(&(*entry_location)->lifetime, HELLO_MSG_LIFETIME, 0);
	return 0;
}


/**
*	@brief Search in the hello message list by source.
*
*	@param source_address
*	@param entry_data: Pointer to structure of type hello_msg_list_entry
*
*	@return
*			- Explicit: True when found, False otherwise.
*			- Implicit: Pointer to the found entry (2nd param).
*/

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
			return true;
		}

	}
	k_sem_give(&hello_msg_list_sem);
	return false;
}

/**
 *	@brief Called by trans_heartbeat in the transport layer when a heartbeat is received.
 *
 *	@param src: heartbeat source address
 *
 *	@return  N/A
 */


void bt_mesh_trans_hello_msg_recv(u16_t src)
{
  	struct hello_msg_list_entry temp_entry;
  	struct hello_msg_list_entry *entry = &temp_entry;
	  entry->source_address=src;
		overhead_control(3); //size=3
  	if (hello_msg_list_search_entry(src, &entry))
  	{

  		BT_DBG("HB:entry found src is %04x ",entry->source_address);
			k_timer_stop(&entry->lifetime);
			struct k_timer temp_timer;
			entry->lifetime = temp_timer;
			k_timer_init(&entry->lifetime,hello_msg_list_entry_expiry_fn, NULL);
			k_timer_start(&entry->lifetime, HELLO_MSG_LIFETIME, 0);
			view_hello_msg_list();
	  	}
	  	else
	  	{
	  		BT_DBG("Hello message received from a node not of interest.");
	  	}

}
/**
 *	@brief it adds new neighbour to the hello message list if it wasn't added before.
 *	@param neighbour: neighbour source address
 *	@param net_idx: neighbour network index
 *
 *	@return  N/A
 */


static void add_neighbour(u16_t neighbour, u16_t net_idx)
{
	/*search for this neighbour in the hello msg list, if not exist , create a new entry*/
  	struct hello_msg_list_entry temp_entry;
  	struct hello_msg_list_entry *entry = &temp_entry;
  	entry->source_address=neighbour;
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


/**
 *	@brief it removes the neighbour from the hello message list if it wasn't used by any other route.
 *	@param neighbour: neighbour source address
 *	@param net_idx: neighbour network index
 *
 *	@return  N/A
 */


void remove_neighbour(u16_t neighbour, u16_t net_idx)
{
	/*check if there's no other route entry needs this neighbout. If not, remove the entry from the hello_msg list*/
	struct bt_mesh_route_entry *entry;
	view_valid_list();
	if(!bt_mesh_search_valid_next_hop_with_net_idx(neighbour, net_idx, &entry))
	{
	  	struct hello_msg_list_entry temp_entry;
	  	struct hello_msg_list_entry *hello_msg_entry = &temp_entry;
		  hello_msg_entry->source_address=neighbour;
	    BT_DBG("hb source is: %04x",neighbour);

	  	if (hello_msg_list_search_entry(neighbour, &hello_msg_entry))
			{
				BT_DBG("src to be deleted is %04x ",neighbour);
				hello_msg_list_delete_entry(hello_msg_entry);
				view_hello_msg_list();
			}

	}
}

/**
 *	@brief Displays the entries of the hello message list
 *
 *	@return : N/A
 */

void view_hello_msg_list()
{
	if (is_empty_hello_msg_list())
	{
		return;
	}
	struct hello_msg_list_entry *entry = NULL;
	k_sem_take(&hello_msg_list_sem, K_FOREVER);
	SYS_SLIST_FOR_EACH_CONTAINER(&hello_msg_list, entry, node)
	{
		BT_DBG("\x1b[32m Hello msg List:source address=%04x\x1b[0m ", entry->source_address);
	}
	k_sem_give(&hello_msg_list_sem);
}
