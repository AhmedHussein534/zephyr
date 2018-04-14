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


#include "ctrMsg.h"



sys_slist_t RREP_list;                                                          /*global RREP list */
/* _TABLE_*/
#define INRANGE(new_seq, existing_seq)   ((new_seq > existing_seq) ? 1 : 0)     // FIXME handle wrapping and IV index?
#define NUMBER_OF_ENTRIES_RREP 20
#define ENTRY_SIZE_RREP sizeof(struct bt_mesh_RREP_entry)
struct k_mem_slab RREP_slab;                                                            /*XXX Create a new slab for it?*/
K_MEM_SLAB_DEFINE(RREP_slab, ENTRY_SIZE_RREP, NUMBER_OF_ENTRIES_RREP, ALLIGNED);        /*Common slab*/
K_SEM_DEFINE(RREP_list_sem, 1, 1);                                                      /*Binary semaphore for RREP linked list critical section*/



/////////////////////////////////RREQ////////////////////////////////////////


void RREQ_received_cb(struct k_timer *timer_id)
{
	// TODO: ADD SEMAPHORE so this fn doesn't work with RREQ_RECEIVED
	struct bt_mesh_route_entry *entry = CONTAINER_OF(timer_id, struct bt_mesh_route_entry, lifetime);

	validate_route(entry->source_address, entry->destination_address);

	bool R;
	u16_t src;      /*src address  == 2 byte*/
	u16_t dst;      /*dst address  == 2 byte*/
	u32_t seq;      /*sequence number == 4 bytes*/
	u8_t hop_count; /* hop count == 1 byte */
	u16_t elem;     /*elements == 2 byte*/


	struct bt_mesh_RREP data;
	data.R=1;
	data.src = entry -> destination_address ;
	data.dst = entry -> source_address; // Orginator of RREQ
	data.seq = bt_mesh.seq;
	data.hop_count = 0;
	data.elem = bt_mesh_elem_count();

	RREP_send(&data,entry->net_idx,entry->next_hop);
	view_valid_list();
	view_invalid_list();
	printk("RREP SENDING \n \n \n");
}

bool send_RREQ(struct RREQ_data *data, u8_t TTL, u16_t net_idx)
{
	// Modifying
	data -> destination_address = 0x0e0e;
	printk("Sending RREQ - src=%04x  ,  dst=%04x \n", data->source_address, data->destination_address);
	u8_t flags = data->G + (data->D << 1) + (data->U << 2) + (data->I << 3);
	u16_t network_next_hop = BT_MESH_ADDR_ALL_NODES;
	NET_BUF_SIMPLE_DEFINE(buf, RREQ_SDU_MAX_SIZE);

	struct bt_mesh_route_entry *entry;
	if (data->I && search_valid_destination_without_source(data->destination_address, &entry)) {
		network_next_hop = entry->next_hop;
	}
	struct bt_mesh_msg_ctx ctx =
	{
		.app_idx  = BT_MESH_KEY_UNUSED,
		.net_idx  = net_idx,
		.addr = network_next_hop,
		.send_ttl = TTL
	};
	struct bt_mesh_net_tx tx =
	{
		.ctx  = &ctx,
		.sub  = bt_mesh_subnet_get(net_idx),
		.src  = bt_mesh_primary_addr(),
		.aszmic = 1,                    // TransMIC = 64 bit
		.xmit = bt_mesh_net_transmit_get(),
		.routing = true
	};
	// Elements inside Transport Layer

	net_buf_simple_add_mem(&buf, &data->hop_count, 1);
	net_buf_simple_add_mem(&buf, &data->source_address, 2);
	net_buf_simple_add_mem(&buf, &data->destination_address, 2);
	net_buf_simple_add_mem(&buf, &data->source_number_of_elements, 2);
	net_buf_simple_add_mem(&buf, &data->hop_count, 1);
	net_buf_simple_add_mem(&buf, &flags, 1);
	net_buf_simple_add_mem(&buf, &data->source_sequence_number, 3);
	if (data->U == 0) {
		net_buf_simple_add_mem(&buf, &data->destination_sequence_number, 3);
	}

	printk("Data before sending :%s \n",bt_hex(buf.data,buf.len));
	int err = bt_mesh_ctl_send(&tx, TRANS_CTL_OP_RREQ, buf.data, buf.len, NULL, NULL, NULL);

	if (err != 0) {
		printk("Sending RREQ Failed \n");
		return false;
	} else   {
		return true;
	}
}


bool RREQ_recv(struct bt_mesh_net_rx *rx, struct net_buf_simple *buf)
{

	struct RREQ_data temp;
	struct RREQ_data *data = &temp;

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
	printk("Destination Address=%04x,Source_address=%04x \n", data->destination_address, data->source_address);
	if (bt_mesh_elem_find(data->source_address)) {
		// If element is requesting data transaction from an element in the same node, Drop rreq
		return 0;
	}
	else if (bt_mesh_elem_find(data->destination_address)) { // if destination is me
		if (search_valid_destination(data->destination_address, data->source_address, &entry)) {
			printk("RREQ dropped - RREQ received after RREP Interval\n");
			return 0;
		}
		else if (search_invalid_destination(data->destination_address, data->source_address, &entry)) {
			if (data->hop_count < entry->hop_count) {
				printk("Modifying existing entry \n");
				entry->destination_sequence_number = data->destination_sequence_number;
				entry->hop_count                   = data->hop_count;
				entry->next_hop                    = data->next_hop;
			}
			return 0;
		} else   {
			printk("Creating entry and waiting for RREQ wait interval \n");
			struct bt_mesh_route_entry entry_data;
			entry_data.source_address                 = data->destination_address;
			entry_data.destination_address            = data->source_address;
			entry_data.destination_sequence_number    = data->source_sequence_number;
			entry_data.next_hop                       = data->next_hop;
			entry_data.source_number_of_elements      = bt_mesh_elem_count();
			entry_data.destination_number_of_elements = data->source_number_of_elements;
			entry_data.hop_count                      = data->hop_count;
			entry_data.net_idx 												= rx -> ctx.net_idx;
			create_entry_invalid_with_cb(&entry_data, RREQ_received_cb);
			return 0;
		}
	}
	else if (search_valid_destination_without_source(data->destination_address, &entry) && data->D == false && data->I == false) {
		// Intermediate node having route to destination should reply to RREQ originator
		printk("Intermediate Node received a flooded RREQ and has route to destination \n");
		struct bt_mesh_route_entry entry_data;
		entry_data.source_address                 = data->destination_address;
		entry_data.destination_address            = data->source_address;
		entry_data.destination_sequence_number    = data->source_sequence_number;
		entry_data.next_hop                       = data->next_hop;
		entry_data.source_number_of_elements      = 1; // UNKNOWN
		entry_data.destination_number_of_elements = data->source_number_of_elements;
		entry_data.hop_count                      = data->hop_count;
		entry_data.net_idx 												= rx -> ctx.net_idx;
		create_entry_invalid(&entry_data);
		printk("[RREQ_recv]::\n");
		view_valid_list();
		if (entry->destination_sequence_number >= data->destination_sequence_number) {
			printk("SEND RWAIT and SEND RREQ with flag I=1 \n");
			data->I = 1;
			data->hop_count = data->hop_count + 1;
			send_RREQ(data, 1, rx->ctx.net_idx);
			struct RWait_pdu_info temp;
			printk("[RREQ_recv]:: entry hop_count : 0x%01x \n", entry_data.hop_count);
			// search source without destination
			entry_data.hop_count = entry -> hop_count;
			printk("Hop count now : %d\n",entry_data.hop_count);
			send_Rwait(data,&entry_data,temp,rx,false);

		}
	}
	else   {
		printk("Intermediate Node received a flooded RREQ - Relaying \n");
		struct bt_mesh_route_entry *entry;
		if (!search_invalid_destination(data->destination_address, data->source_address, &entry)) {
			struct bt_mesh_route_entry entry_data;
			entry_data.source_address                 = data->destination_address;
			entry_data.destination_address            = data->source_address;
			entry_data.destination_sequence_number    = data->source_sequence_number;
			entry_data.next_hop                       = data->next_hop;
			entry_data.source_number_of_elements      = 1; // UNKNOWN
			entry_data.destination_number_of_elements = data->source_number_of_elements;
			entry_data.hop_count                      = data->hop_count;
			entry_data.net_idx 												= rx -> ctx.net_idx;
			create_entry_invalid(&entry_data);
			data->next_hop = data->next_hop + 1;
			return send_RREQ(data, rx->ctx.recv_ttl - 1, rx->ctx.net_idx);
		}
		else if (entry->destination_sequence_number < data->source_sequence_number) {
			entry->destination_sequence_number = data->source_sequence_number;
			refresh_lifetime_invalid(entry);
			return send_RREQ(data, rx->ctx.recv_ttl - 1, rx->ctx.net_idx);
		} else   {
			printk("RREQ Dropped - received same RREQ because of ring search\n");
		}
	}

	/*printk("Source Address=%04x \n",RREQ_GET_SRC_ADDR(buf));
	   printk("Destination Address=%04x \n",RREQ_GET_DST_ADDR(buf));
	   printk("Source Number of Elements=%04x \n",RREQ_GET_SRC_NUMBER_OF_ELEMENTS(buf));
	   printk("Hop Count=%d \n",RREQ_GET_HOP_COUNT(buf));
	   printk("G Flag=%d \n",RREQ_GET_G_FLAG(buf));
	   printk("D Flag=%d \n",RREQ_GET_D_FLAG(buf));
	   printk("U Flag=%d \n",RREQ_GET_U_FLAG(buf));
	   printk("I Flag=%d \n",RREQ_GET_I_FLAG(buf));
	   printk("Destination Sequence Number=%08x \n",RREQ_GET_DST_SEQ(buf));
	 */
}

struct ring_search_flag_timer {
	struct k_timer ring_timer;
	bool ring_flag;
};

static void ring_search_timer(struct k_timer *timer_id)
{
	struct ring_search_flag_timer *entry = CONTAINER_OF(timer_id, struct ring_search_flag_timer, ring_timer); /* container of timer_id to be deleted*/
	entry->ring_flag = true;
}


u8_t bt_mesh_trans_ring_search(struct bt_mesh_net_tx *tx)
{
	u16_t source_address = tx->src;
	u16_t destination_address = tx->ctx->addr;

	printk("Destination Address Test : %04x\n", destination_address);

	u32_t destination_sequence_number = 0;
	bool U_flag = 0;
	struct ring_search_flag_timer ring_struct;
	ring_struct.ring_flag = false;
	struct bt_mesh_RREP_entry *temp;
	k_timer_init(&ring_struct.ring_timer, ring_search_timer, NULL);                                 /*init lifetime timer*/
	k_timer_start(&ring_struct.ring_timer, RING_SEARCH_WAIT_INTERVAL, RING_SEARCH_WAIT_INTERVAL);   /* Reset Timer */
	u8_t TTL = 2;
	printk("current TTL=%d \n", TTL);

	struct bt_mesh_route_entry *entry;
	if (search_invalid_destination(source_address, destination_address, &entry)) {
		destination_sequence_number = entry->destination_sequence_number;
		U_flag = 1;
	}

	struct RREQ_data data = {
		.source_address = bt_mesh_primary_addr(),
		.destination_address = destination_address,
		.U = U_flag,
		.hop_count = 0,
		.source_sequence_number = bt_mesh.seq,
		.source_number_of_elements = bt_mesh_elem_count(),
		.destination_sequence_number = destination_sequence_number
	};
	send_RREQ(&data, TTL, tx->ctx->net_idx);
	while (1) {

		k_sem_take(&RREP_list_sem, K_FOREVER); /*take semaphore */
		/* search global linked list of RREP should be here */
		SYS_SLIST_FOR_EACH_CONTAINER(&RREP_list, temp, node){
			// RWAIT Received
			if(temp->hop_count != 0){
				printk("Delaying Ring Search with hop count =%d",temp->hop_count);
				// refresh timer
				k_timer_stop(&ring_struct.ring_timer);
				struct k_timer temp_timer;
				ring_struct.ring_timer = temp_timer;
				k_timer_init(&ring_struct.ring_timer, ring_search_timer, NULL);       /*init lifetime timer*/
				k_timer_start(&ring_struct.ring_timer, RING_SEARCH_WAIT_INTERVAL*4, 0);                   /*start timer 200s for each entry */

				// delete entry
				printk("Deleting Entry\n");
				sys_slist_find_and_remove(&RREP_list, &temp->node);     /*delete node from linked list */
				k_sem_give(&RREP_list_sem);                             /*return semaphore */
				k_mem_slab_free(&RREP_slab, (void **)&temp);            /*free space in slab*/
			}

			// RREP Received
			if (temp->dst== destination_address) {
				k_timer_stop(&ring_struct.ring_timer);
				sys_slist_find_and_remove(&RREP_list, &temp->node);     /*delete node from linked list */
				k_sem_give(&RREP_list_sem);                             /*return semaphore */
				k_mem_slab_free(&RREP_slab, (void **)&temp);            /*free space in slab*/
				printk("RREP is found \n");
				printk("RREP LIST \n\n");
				view_rrep_list();

				printk("\n\nRouting Table - Valid\n\n");
				view_valid_list();
				printk("\n\nRouting Table - INValid\n\n");
				view_invalid_list();

				return 1;
			}
		}
		k_sem_give(&RREP_list_sem); /*return semaphore */

		if (ring_struct.ring_flag == true) {
			ring_struct.ring_flag = false;
			TTL = TTL + 1;
			data.source_sequence_number = bt_mesh.seq;
			send_RREQ(&data, TTL, tx->ctx->net_idx);
			printk("Timer expired waiting for RREP \n");
			printk("current TTL=%d \n", TTL);
			if (TTL == RING_SEARCH_MAX_TTL) {
				printk("TTL max reached \n");
				k_timer_stop(&ring_struct.ring_timer);
				return 0;
			}
		}
		k_sleep(K_MSEC(50));
	}
}





// ////////////////////Rwait//////////////////////


/*This a temporary list for the RREP to insert the RWait in them*/
/*TODO to be called in the init process*/

void RREP_list_init()
{
	sys_slist_init(&RREP_list);
}


void send_Rwait(struct RREQ_data* RREQ,struct bt_mesh_route_entry *dst_entry,struct RWait_pdu_info Rwait, struct bt_mesh_net_rx* rx, bool relay )
{
	u16_t net_idx_RREQ = rx->ctx.net_idx;
	u16_t dst_RREQ=RREQ->destination_address;
	u16_t src_RREQ=RREQ->source_address;
	u32_t src_seq_RREQ=RREQ->source_sequence_number;

	struct bt_mesh_msg_ctx ctx;
	struct bt_mesh_net_tx tx;
	struct RWait_pdu_info Rwait_tosend;

	if (relay != true) {

		// transport layer
		struct RWait_pdu_info Rwait_temp = {
			.dst = dst_RREQ,
			.src = src_RREQ,
			.src_seq = src_seq_RREQ,
			.hop_count = dst_entry->hop_count,
		};

		// Network layer

		ctx.net_idx  = net_idx_RREQ;            // from RREQ
		ctx.app_idx  = BT_MESH_KEY_UNUSED;
		ctx.addr     = dst_entry->next_hop;     // Next hop fetched from the routing table
		ctx.send_ttl = 3; //FIXME
		tx.sub  = bt_mesh_subnet_get(net_idx_RREQ);
		tx.ctx  = &ctx;
		tx.src  = bt_mesh_primary_addr();
		tx.xmit = bt_mesh_net_transmit_get();
		Rwait_tosend = Rwait_temp;
	} else   {
		Rwait_tosend = Rwait;
		// Network layer
		ctx = rx->ctx;
		tx.sub  = bt_mesh_subnet_get(net_idx_RREQ);
		tx.ctx  = &ctx;
		tx.src  = bt_mesh_primary_addr();
		tx.xmit = bt_mesh_net_transmit_get();
	}


	BT_DBG("Send Rwait");
	printk("Send Rwait");
	// //////////////////////////////////////////////////////////

	struct net_buf_simple *sdu = NET_BUF_SIMPLE(BT_MESH_TX_SDU_MAX);
	net_buf_simple_init(sdu, 0);
	net_buf_simple_add_u8(sdu, TRANS_CTL_OP_RWAIT);
	net_buf_simple_add_le16(sdu, Rwait_tosend.dst);
	net_buf_simple_add_le16(sdu, Rwait_tosend.src);
	net_buf_simple_add_le32(sdu, Rwait_tosend.src_seq);
	net_buf_simple_add_u8(sdu, Rwait_tosend.hop_count);
	BT_DBG("len %u: %s", sdu->len, bt_hex(sdu->data, sdu->len));

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
	printk("Rwait data :%s\n",bt_hex(sdu->data,sdu->len));
	// send_unseg(&tx, sdu, NULL, NULL);
	// bt_mesh_trans_send(&tx, sdu, NULL, NULL);

	 printk("Source Address=%04x \n",Rwait_tosend.src);
   printk("Destination Address=%04x \n",Rwait_tosend.dst);
   printk("Hop Count=%d \n",Rwait_tosend.hop_count);
	 printk("[send_Rwait]:: entry hop_count : 0x%01x \n", dst_entry->hop_count);


	bt_mesh_ctl_send(&tx, TRANS_CTL_OP_RWAIT, sdu->data,sdu->len, NULL, NULL, NULL);

}

void RWAIT_recv(struct bt_mesh_net_rx *rx, struct net_buf_simple *buf)
{
	struct RWait_pdu_info temp2;
	struct RWait_pdu_info *Rwaitmsg = &temp2;
	struct bt_mesh_route_entry *temp = NULL;

	if (buf->len < sizeof(*Rwaitmsg)) {
		BT_WARN("Too short Rwaitmsg");
		printk("buf->len 0x%02x  sizeof(*Rwaitmsg) 0x%02x ", buf->len, sizeof(*Rwaitmsg));
		printk("Too short Rwaitmsg\n");
	}

	Rwaitmsg -> dst = RWAIT_GET_DST_ADDR(buf);
	Rwaitmsg -> src = RWAIT_GET_SRC_ADDR(buf);
	Rwaitmsg -> src_seq = RWAIT_GET_SRC_SEQ_NUM(buf);
	Rwaitmsg -> hop_count= RWAIT_GET_HOP_COUNT(buf);

	/*Display the RRWAit recieved*/
	printk("Rwait dst 0x%04x \n", Rwaitmsg->dst);
	printk("Rwait src 0x%04x \n", Rwaitmsg->src);
	printk("Rwait src_seq 0x%08x \n", Rwaitmsg->src_seq);
	printk("Rwait hop_count 0x%01x \n", Rwaitmsg->hop_count);
	printk("Network Src 0x%04x \n", rx->ctx.addr);
	printk("Network dst 0x%04x \n", rx->dst);
	printk("Network recieved TL 0x%02x \n", rx->ctx.send_ttl);

	/*Process the RWAit*/

	if (Rwaitmsg->src == bt_mesh_primary_addr()) {
		if (Rwaitmsg->hop_count == 0) {
			Rwaitmsg->hop_count++;
		}

		if (!search_valid_destination(rx->ctx.addr, rx->dst, &temp)) {
			struct bt_mesh_RREP_entry temp;
			struct bt_mesh_RREP_entry *RREP_entry=&temp;
			RREP_entry->dst = Rwaitmsg->dst;
			RREP_entry->hop_count = Rwaitmsg->hop_count;
			create_entry_RREP(RREP_entry);
			return 1;
		}
		// TODO : Ring Search will adjust the waiting interval based on the hop count
	} else   {
		// intermediate node
		if (!search_invalid_destination(rx->ctx.addr, rx->dst, &temp)) {
			struct RWait_pdu_info Rwait_tosend = {
				.dst = Rwaitmsg->dst,
				.src = Rwaitmsg->src,
				.src_seq = Rwaitmsg->src_seq,
				.hop_count = Rwaitmsg->hop_count,
			};

			send_Rwait(NULL,NULL,Rwait_tosend, rx, true);
		} else   {
			printk("RWait has been dropped");
		}

	}
	printk("Rwait Recieved");
}


// ////////////RREP//////////////


bool RREP_send(struct bt_mesh_RREP *RREP_msg,u16_t net_idx, u16_t dst )
{

	//TODO : check when RREQ_recv is calling RREP_send
	printk("RREP_send\n");

	struct bt_mesh_msg_ctx ctx =
	{
		.app_idx  = BT_MESH_KEY_UNUSED,
		.net_idx  = net_idx,
		.send_ttl = 3 // FIXME
	};

	struct bt_mesh_net_tx tx = {
		.sub = bt_mesh_subnet_get(net_idx),
		.ctx = &ctx,
		.xmit = bt_mesh_net_transmit_get(),
	};
	tx.ctx->addr = dst; // primary elem of dst
	tx.src = bt_mesh_primary_addr();
	tx.ctx->send_ttl--;

	printk("RREP R 0x%01x \n", RREP_msg->R);
	printk("RREP src 0x%02x \n", RREP_msg->src);
	printk("RREP dst 0x%02x \n", RREP_msg->dst);
	printk("RREP seq 0x%04x \n", RREP_msg->seq);
	printk("RREP hop_count 0x%01x \n", RREP_msg->hop_count);
	printk("RREP elem 0x%04x \n", RREP_msg->elem);

	NET_BUF_SIMPLE_DEFINE(buf, 20);
	net_buf_simple_add_mem(&buf, &RREP_msg->R, 1);
	net_buf_simple_add_mem(&buf, &RREP_msg->src, 2);
	net_buf_simple_add_mem(&buf, &RREP_msg->dst, 2);
	net_buf_simple_add_mem(&buf, &RREP_msg->seq, 4);
	net_buf_simple_add_mem(&buf, &RREP_msg->hop_count, 1);
	net_buf_simple_add_mem(&buf, &RREP_msg->elem, 2);

	//  printk("Payload %s", bt_hex(buf.data, data_len));
	return bt_mesh_ctl_send(&tx, TRANS_CTL_OP_RREP, buf.data,
				buf.len, NULL, NULL, NULL);
}

bool create_entry_RREP(struct bt_mesh_RREP_entry *entry_data)
{
  struct bt_mesh_RREP_entry* entry_location=NULL;
	if (k_mem_slab_alloc(&RREP_slab, (void **)&entry_location, 100) == 0) { /*if space found in slab*/
		memset(entry_location, 0, ENTRY_SIZE); /* Initializing with zeros */
		k_sem_take(&RREP_list_sem, K_FOREVER);                  /*take semaphore */
		sys_slist_append(&RREP_list, &entry_location->node);        /*insert node in linkedlist */
		k_sem_give(&RREP_list_sem);
	} else    {
		printk("Memory Allocation timeout \n");
		return false;
	}
	entry_location -> dst = entry_data -> dst;
	entry_location -> hop_count = entry_data -> hop_count;
	return true;
}

void view_rrep_list()
{
	if (sys_slist_is_empty(&RREP_list)) {
		printk("RREP List is empty \n");
		return;
	}
	struct bt_mesh_RREP_entry *entry = NULL;
	k_sem_take(&RREP_list_sem, K_FOREVER); /*take semaphore */
	SYS_SLIST_FOR_EACH_CONTAINER(&RREP_list, entry, node){
		printk("RREP List:Destination address=%04x\n",entry->dst);
	}
	k_sem_give(&RREP_list_sem);
}

bool RREP_recv(struct bt_mesh_net_rx *rx, struct net_buf_simple *buf2)
{
	printk("RREP_recv\n");
	printk("len %u: %s", buf2->len, bt_hex(buf2->data, buf2->len));
	struct bt_mesh_RREP temp;
	struct bt_mesh_RREP *msg = &temp;
	msg->R = RREP_GET_R(buf2);
	msg->src = RREP_GET_SRC_ADDR(buf2);
	msg->dst = RREP_GET_DST_ADDR(buf2);
	msg->seq = RREP_GET_SEQ_NUM(buf2);
	msg->hop_count = RREP_GET_HOP_COUNT(buf2);
	msg->elem = RREP_GET_SRC_NUMBER_OF_ELEMENTS(buf2);
	printk("RREP R 0x%01x \n", msg->R);
	printk("RREP src 0x%04x \n", msg->src);
	printk("RREP dst 0x%04x \n", msg->dst);
	printk("RREP seq 0x%04x \n", msg->seq);
	printk("RREP hop_count 0x%02x \n", msg->hop_count);
	printk("RREP elem 0x%02x \n", msg->elem);
	printk("Network Src 0x%02x \n", rx->ctx.addr);
	printk("Network dst 0x%02x \n", rx->dst);
	printk("Network recieved TL 0x%02x \n", rx->ctx.send_ttl);
	printk("msg->src 0x%04x \n", msg->src);
	printk("bt_mesh_primary_addr() 0x%04x \n", bt_mesh_primary_addr());



	if (msg->src == bt_mesh_primary_addr()) { /*I'M THE SRC*/
		/*Compare the sequence number*/
		printk("I am the src \n");
		struct bt_mesh_route_entry *found_entry = NULL;
		if (!search_valid_destination(msg->src, msg->dst, &found_entry) || (INRANGE(msg->seq,found_entry->destination_sequence_number) && invalidate_route(found_entry->source_address, found_entry->destination_address))) { /*New route, stop ring search XXX*/
			struct bt_mesh_RREP_entry RREP_entry_temp;
			struct bt_mesh_RREP_entry *RREP_entry=&RREP_entry_temp;

			RREP_entry->dst = msg->dst;
			RREP_entry->hop_count = msg->hop_count;
			// Create forward entry
			struct bt_mesh_route_entry table_entry;
			table_entry.source_address                  = msg->src;         // forward route
			table_entry.destination_address             = msg->dst;
			table_entry.destination_sequence_number     = msg->seq;         // From NL
			table_entry.next_hop                        = rx->ctx.addr;     // From NL
			table_entry.hop_count                       = msg->hop_count;
			table_entry.destination_number_of_elements  = msg->elem;
			table_entry.source_number_of_elements 			= bt_mesh_elem_count();
			table_entry.net_idx 												= rx -> ctx.net_idx;
			create_entry_valid(&table_entry);

			// Create entry in rrep list
			return create_entry_RREP(RREP_entry);
		}
	} else   {
		printk("I-NODE \n");
		struct bt_mesh_route_entry *existing_entry;
		view_invalid_list();
		//TODO--Reham
		if (search_invalid_destination_with_range(msg->dst,msg->src,msg->elem, &existing_entry)) {
			printk("search condition passed \n");
			existing_entry->source_number_of_elements=msg->elem;
			existing_entry->source_address=msg->dst;
			validate_route(existing_entry->source_address, existing_entry->destination_address); /*OVERHEAD*/
			printk("Validate is done \n");
			view_valid_list();
			struct bt_mesh_route_entry table_entry;
			table_entry.source_address                  = msg->src;         // forward route
			table_entry.destination_address             = msg->dst;
			table_entry.destination_sequence_number     = msg->seq;         // From NL
			table_entry.next_hop                        = rx->ctx.addr;     // From NL
			table_entry.hop_count                       = msg->hop_count;
			table_entry.destination_number_of_elements  = msg->elem;
			table_entry.source_number_of_elements 			= existing_entry->destination_number_of_elements;
			table_entry.net_idx 												= rx -> ctx.net_idx;
			create_entry_valid(&table_entry);

			printk("forward entry added \n");
			view_valid_list();
			msg->hop_count++;
			printk("send RREP");
			RREP_send(msg, rx -> ctx.net_idx ,existing_entry->next_hop);
		}
	}

	return true;
}
