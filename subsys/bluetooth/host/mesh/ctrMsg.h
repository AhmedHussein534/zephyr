/* RREQ */
#define RREQ_SDU_MAX_SIZE 14
#define RREQ_GET_SRC_ADDR(buf) buf->data[1] + (buf->data[2] << 8)               // 2 Bytes
#define RREQ_GET_DST_ADDR(buf) buf->data[3] + (buf->data[4] << 8)               // 2  Bytes
#define RREQ_GET_SRC_NUMBER_OF_ELEMENTS(buf) buf->data[5] + (buf->data[6] << 8) // 2  Bytes
#define RREQ_GET_HOP_COUNT(buf) buf->data[7]                                    // 1 Bytes
#define RREQ_GET_G_FLAG(buf) (buf->data[8] & 0x01)                              // All flags = 1 Bytes
#define RREQ_GET_D_FLAG(buf) (buf->data[8] & 0x02) >> 1
#define RREQ_GET_U_FLAG(buf) (buf->data[8] & 0x04) >> 2
#define RREQ_GET_I_FLAG(buf) (buf->data[8] & 0x08) >> 3
#define RREQ_GET_SRC_SEQ(buf) buf->data[9] + (buf->data[10] << 8) + (buf->data[11] << 16)       // 3 Bytes
#define RREQ_GET_DST_SEQ(buf) buf->data[12] + (buf->data[13] << 8) + (buf->data[14] << 16)      // 3 Bytes
#define RING_SEARCH_WAIT_INTERVAL K_SECONDS(10)
#define RING_SEARCH_MAX_TTL 10

struct RREQ_data {
	u16_t source_address;
	u16_t destination_address;
	u16_t next_hop; // from network pdu
	u16_t source_number_of_elements;
	u8_t G : 1, D : 1, U : 1, I : 1;
	u8_t hop_count;
	u32_t source_sequence_number;
	u32_t destination_sequence_number;
};


void RREQ_received_cb(struct k_timer *timer_id);
bool send_RREQ(struct RREQ_data *data, u8_t TTL, u16_t net_idx);
bool RREQ_recv(struct bt_mesh_net_rx *rx, struct net_buf_simple *buf);
u8_t bt_mesh_trans_ring_search(struct bt_mesh_net_tx *tx);


/*RREP*/

#define RREP_GET_R(buf) buf->data[0]                                                                            // 1 Bytes ,R
#define RREP_GET_SRC_ADDR(buf) buf->data[1] + (buf->data[2] << 8)                                               // 2 Bytes, src
#define RREP_GET_DST_ADDR(buf) buf->data[3] + (buf->data[4] << 8)                                               // 2  Bytes, dst
#define RREP_GET_SEQ_NUM(buf) buf->data[5] + (buf->data[6] << 8) + (buf->data[7] << 16) + (buf->data[8] << 24)  // 4  Bytes, seq
#define RREP_GET_HOP_COUNT(buf) buf->data[9]                                                                    // +(buf->data[9] << 8)  // 1 Bytes
#define RREP_GET_SRC_NUMBER_OF_ELEMENTS(buf) buf->data[10] + (buf->data[11] << 8)                               // 2  Bytes, src # of elem




struct RWait_pdu_info {
	u16_t dst;              /*Destination address==2 byte */
	u16_t src;              /*Source address  == 2 byte*/
	u32_t src_seq;          /*Source Sequence Number==4 byte */
	u8_t hop_count;         /* hop count == 1 byte */
	sys_snode_t node;       /* Linkedlist node == 4 byte, to be added in the RREP list*/
};


struct bt_mesh_RREP_entry {
	u16_t dst;              /*dst address  == 2 byte*/
	u8_t hop_count;         /* hop count == 1 byte */
	sys_snode_t node;       /* Linkedlist node == 4 byte */
};

struct bt_mesh_RREP {
	bool R;
	u16_t src;      /*src address  == 2 byte*/
	u16_t dst;      /*dst address  == 2 byte*/
	u32_t seq;      /*sequence number == 4 bytes*/
	u8_t hop_count; /* hop count == 1 byte */
	u16_t elem;     /*elements == 2 byte*/
};


#define RWAIT_GET_DST_ADDR(buf) buf->data[1] + (buf->data[2] << 8)                                               // 2  Bytes, dst
#define RWAIT_GET_SRC_ADDR(buf) buf->data[3] + (buf->data[4] << 8)                                               // 2 Bytes, src
#define RWAIT_GET_SRC_SEQ_NUM(buf) buf->data[5] + (buf->data[6] << 8) + (buf->data[7] << 16) + (buf->data[8] << 24)  // 4  Bytes, seq
#define RWAIT_GET_HOP_COUNT(buf) buf->data[9]                                                                    // +(buf->data[9] << 8)  // 1 Bytes

void send_Rwait(struct RREQ_data* RREQ,struct bt_mesh_route_entry *dst_entry,struct RWait_pdu_info Rwait, struct bt_mesh_net_rx* rx, bool relay );
void RWAIT_recv(struct bt_mesh_net_rx *rx, struct net_buf_simple *buf);
bool RREP_recv(struct bt_mesh_net_rx *rx, struct net_buf_simple *buf);
bool RREP_send(struct bt_mesh_RREP *RREP_msg,u16_t net_idx, u16_t dst );
bool create_entry_RREP(struct bt_mesh_RREP_entry *entry_data);
void RREP_list_init();
void view_rrep_list();
