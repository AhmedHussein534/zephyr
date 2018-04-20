/** @file ctrMsg.h
 *  @brief Rouitng Control Messages File
 *
 *  Bluetooth routing control messages following AODV protocol.
 *	The file contains RREQ, RREP and RWAIT data and functions.
 *  @bug No known bugs.
 */

 /* -- Includes -- */

/* DEFINITIONS */
/* RREQ DEFINITIONS */
#define RREQ_SDU_MAX_SIZE 14
/* Macros used to dissect a buffer containing RREQ data*/
#define RREQ_GET_SRC_ADDR(buf) buf->data[1] + (buf->data[2] << 8)
#define RREQ_GET_DST_ADDR(buf) buf->data[3] + (buf->data[4] << 8)
#define RREQ_GET_SRC_NUMBER_OF_ELEMENTS(buf) buf->data[5] + (buf->data[6] << 8)
#define RREQ_GET_HOP_COUNT(buf) buf->data[7]
#define RREQ_GET_G_FLAG(buf) (buf->data[8] & 0x01)
#define RREQ_GET_D_FLAG(buf) (buf->data[8] & 0x02) >> 1
#define RREQ_GET_U_FLAG(buf) (buf->data[8] & 0x04) >> 2
#define RREQ_GET_I_FLAG(buf) (buf->data[8] & 0x08) >> 3
#define RREQ_GET_SRC_SEQ(buf) buf->data[9] + (buf->data[10] << 8) + (buf->data[11] << 16)
#define RREQ_GET_DST_SEQ(buf) buf->data[12] + (buf->data[13] << 8) + (buf->data[14] << 16)

/* Ring search Macros */
#define RREQ_RING_SEARCH_WAIT_INTERVAL K_SECONDS(10)
#define RREQ_RING_SEARCH_MAX_TTL 10

/* RREP DEFINITIONS */
#define RREP_SDU_MAX_SIZE 20 //FIXME : CHANGE
#define RREP_GET_R(buf) buf->data[0]
#define RREP_GET_SRC_ADDR(buf) buf->data[1] + (buf->data[2] << 8)
#define RREP_GET_DST_ADDR(buf) buf->data[3] + (buf->data[4] << 8)
#define RREP_GET_SEQ_NUM(buf) buf->data[5] + (buf->data[6] << 8) + (buf->data[7] << 16) + (buf->data[8] << 24)
#define RREP_GET_HOP_COUNT(buf) buf->data[9]
#define RREP_GET_SRC_NUMBER_OF_ELEMENTS(buf) buf->data[10] + (buf->data[11] << 8)

/* RWAIT DEFINITIONS */
#define RWAIT_GET_DST_ADDR(buf) buf->data[1] + (buf->data[2] << 8)
#define RWAIT_GET_SRC_ADDR(buf) buf->data[3] + (buf->data[4] << 8)
#define RWAIT_GET_SRC_SEQ_NUM(buf) buf->data[5] + (buf->data[6] << 8) + (buf->data[7] << 16) + (buf->data[8] << 24)
#define RWAIT_GET_HOP_COUNT(buf) buf->data[9]

/* DATA */

/** @brief RREQ data for transmission or reception. Contains the transport layer
 *				 RREQ PDU and the network layer credentials.
 */
struct rreq_data {
	u16_t source_address; 						 /* Address of RREQ originator (2B) */
	u16_t destination_address;         /* Address of RREQ destination (2B)*/
	u16_t next_hop; 									 /* Address of the next hop from the Network Layer (2B) */
	u16_t source_number_of_elements; 	 /* Number of elements in RREQ originator (2B)*/
	u8_t G:1, 												 /* Gratious RREP (1b) */
			 D:1, 												 /* Destination shall only reply flag (1b) */
			 U:1, 												 /* Unknown destination sequence number flag (1b)*/
			 I:1; 									 			 /* Directed RREQ flag (1b)*/
	u8_t hop_count; 									 /* Number of hops between RREQ originator and destination (1B) */
	u32_t source_sequence_number;			 /* RREQ originator sequence number (3B) */
	u32_t destination_sequence_number; /* Last known sequence number of the RREQ destination (3B) */
};

/** @brief List holding some data received by RWAIT or RREP.
 */
 struct rrep_rwait_list_entry {
	u16_t destination_address;         /* RREQ destination (2B)*/
	u8_t hop_count;         	 	 			 /* Number of hops between RREQ originator and destination (1B) */
	sys_snode_t node;      						 /* Linkedlist node (4B) */
};

/** @brief RREP data for transmission or reception. Contains the transport layer
 *				 RREP PDU and the network layer credentials.
 */
 struct rrep_data {
	bool R;														 		/* Repairable Flag (1b) */
	u16_t source_address;      				 		/* RREQ originator address (2B) */
	u16_t destination_address;      	 		/* RREQ destination address (2B) */
	u32_t destination_sequence_number; 		/* RREQ destination sequence number (3B) */
	u8_t hop_count; 									    /* Number of hops between RREQ originator and destination (1B) */
	u16_t destination_number_of_elements; /* Number of elements in the RREQ destination (2B) */
};

/** @brief RWAIT data for transmission or reception. Contains the transport layer
 *				 RWAIT PDU and the network layer credentials.
 */
 struct rwait_data {
	u16_t destination_address;         /* RREQ Destination (2B) */
	u16_t source_address;              /* RREQ originator (2B) */
	u32_t source_sequence_number;      /* RREQ originator sequence number (3B) */
	u8_t hop_count;         					 /* Number of hops between RREQ originator and destination (1B) */
	sys_snode_t node;       					 /* Linkedlist node (4B) */
};

/* FUNCTIONS PROTOTYPES */
/* RREQ FUNCTIONS */
int bt_mesh_trans_rreq_recv(struct bt_mesh_net_rx *rx, struct net_buf_simple *buf);
u8_t bt_mesh_trans_ring_search(struct bt_mesh_net_tx *tx);

/* RREP FUNCTIONS */
int bt_mesh_trans_rrep_recv(struct bt_mesh_net_rx *rx, struct net_buf_simple *buf);
void bt_mesh_trans_rrep_rwait_list_init();

/* RWAIT FUNCTIONS */
void bt_mesh_trans_rwait_recv(struct bt_mesh_net_rx *rx, struct net_buf_simple *buf);
