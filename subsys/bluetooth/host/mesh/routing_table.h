#define NUMBER_OF_ENTRIES 20    // Maximum number of entries in the table
#define ALLIGNED 4              // Memory Allignment
#define ALLOCATION_INTERVAL 100 /* maximum time taken to allocate from slab */
#define ENTRY_SIZE sizeof(struct bt_mesh_route_entry)
#define LIFETIME  K_SECONDS(5)
#define RREQ_INTERVAL_WAIT K_MSEC(1000)


struct bt_mesh_route_entry {
	u16_t source_address;                   /*Destination address==2 byte */
	u16_t destination_address;              /*Source address  == 2 byte*/
	u32_t destination_sequence_number;      /*Source Sequence number == 4 byte*/
	u16_t next_hop;                         /*Next hop address == 2 byte*/
	u16_t source_number_of_elements;
	u16_t destination_number_of_elements;
	u8_t hop_count;                 /* hop count == 1 byte */
	bool repairable;
	u16_t net_idx;									/* Network Index */
	struct k_timer lifetime;        /*lifetime timer entry == 52 byte*/
	sys_snode_t node;               /* Linkedlist node == 4 byte */
};

void routing_table_init();
void delete_entry_valid(struct k_timer *timer_id);
void delete_entry_invalid(struct k_timer *timer_id);
bool search_valid_destination(u16_t source_address, u16_t destination_address, struct bt_mesh_route_entry **entry);
bool search_valid_destination_without_source(u16_t destination_address, struct bt_mesh_route_entry **entry);
bool search_valid_source_without_destination(u16_t source_address, struct bt_mesh_route_entry **entry);
bool search_invalid_destination(u16_t source_address, u16_t destination_address, struct bt_mesh_route_entry **entry);
bool search_invalid_destination_without_source(u16_t destination_address, struct bt_mesh_route_entry **entry);
bool search_invalid_source_without_destination(u16_t source_address, struct bt_mesh_route_entry **entry);
bool search_valid_destination_with_range(u16_t source_address, u16_t destination_address, u16_t destination_number_of_elements, struct bt_mesh_route_entry **entry);
bool search_valid_source_with_range(u16_t source_address, u16_t destination_address, u16_t source_number_of_elements, struct bt_mesh_route_entry **entry);
bool search_invalid_destination_with_range(u16_t source_address, u16_t destination_address, u16_t destination_number_of_elements, struct bt_mesh_route_entry **entry);
bool search_invalid_source_with_range(u16_t source_address, u16_t destination_address, u16_t source_number_of_elements, struct bt_mesh_route_entry **entry);
bool create_entry_valid(struct bt_mesh_route_entry *entry_data);
bool create_entry_invalid(struct bt_mesh_route_entry *entry_data);
bool create_entry_invalid_with_cb(struct bt_mesh_route_entry *entry_data, void (*timer_cb)(struct k_timer *timer_id));

bool validate_route(u16_t source_address, u16_t destination_address);
bool invalidate_route(u16_t source_address, u16_t destination_address);
void refresh_lifetime_valid(struct bt_mesh_route_entry *entry);
void refresh_lifetime_invalid(struct bt_mesh_route_entry *entry);
void view_valid_list();
void view_invalid_list();
