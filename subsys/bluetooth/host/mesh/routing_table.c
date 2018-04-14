#include <zephyr.h>
#include <misc/slist.h> /*include slist*/
#include <string.h>   //memset function to initialize by zeros
#include "routing_table.h"

sys_slist_t valid_list;                                                         /*global linked list for  entries */
K_SEM_DEFINE(valid_list_sem, 1, 1);                                             /*Binary semaphore for  linked list critical section*/
sys_slist_t invalid_list;                                                       /*global linked list for in entries */
K_SEM_DEFINE(invalid_list_sem, 1, 1);                                           /*Binary semaphore for in list critical section */
struct k_mem_slab routing_table_slab;                                           /*Memory slab for entries */
K_MEM_SLAB_DEFINE(routing_table_slab, ENTRY_SIZE, NUMBER_OF_ENTRIES, ALLIGNED); /*Macro to define memory slab*/

void routing_table_init()
{
	sys_slist_init(&valid_list);
	sys_slist_init(&invalid_list);
}

void delete_entry_valid(struct k_timer *timer_id)
{
	struct bt_mesh_route_entry *entry = CONTAINER_OF(timer_id, struct bt_mesh_route_entry, lifetime);       /* container of timer_id to be deleted*/

	k_sem_take(&valid_list_sem, K_FOREVER);                                                                 /* take semaphore */
	sys_slist_find_and_remove(&valid_list, &entry->node);                                                   /*delete node from linked list */
	k_sem_give(&valid_list_sem);                                                                            /*return semaphore */
	k_mem_slab_free(&routing_table_slab, (void **)&entry);                                                  /*free space in slab*/
	printk("valid Entry Deleted \n");
}

void delete_entry_invalid(struct k_timer *timer_id)
{
	struct bt_mesh_route_entry *entry = CONTAINER_OF(timer_id, struct bt_mesh_route_entry, lifetime);       /* container of timer_id to be deleted*/

	k_sem_take(&invalid_list_sem, K_FOREVER);                                                               /* take semaphore */
	sys_slist_find_and_remove(&invalid_list, &entry->node);                                                 /*delete node from linked list */
	k_sem_give(&invalid_list_sem);                                                                          /*return semaphore */
	k_mem_slab_free(&routing_table_slab, (void **)&entry);                                                  /*free space in slab*/
	printk("Invalid Entry Deleted \n");
}

bool search_valid_destination(u16_t source_address, u16_t destination_address, struct bt_mesh_route_entry **entry)
{
	struct bt_mesh_route_entry *entry1 = NULL;

	k_sem_take(&valid_list_sem, K_FOREVER); /*take semaphore */
	SYS_SLIST_FOR_EACH_CONTAINER(&valid_list, entry1, node){
		if ((destination_address >= entry1->destination_address) &&
		    (destination_address < (entry1->destination_address + entry1->destination_number_of_elements)) &&
		    (source_address >= entry1->source_address) &&
		    (source_address < (entry1->source_address + entry1->source_number_of_elements))) {
			k_sem_give(&valid_list_sem);
			*entry = entry1;
			return true;
		}

	}
	k_sem_give(&valid_list_sem);
	return false;
}

bool search_valid_destination_without_source(u16_t destination_address, struct bt_mesh_route_entry **entry)
{
	struct bt_mesh_route_entry *entry1 = NULL;
	k_sem_take(&valid_list_sem, K_FOREVER); /*take semaphore */
	SYS_SLIST_FOR_EACH_CONTAINER(&valid_list, entry1, node){
		if ((destination_address >= entry1->destination_address) &&
		    (destination_address < entry1->destination_address + entry1->destination_number_of_elements)) {
			k_sem_give(&valid_list_sem);
			*entry = entry1;
			return true;
		}

	}
	k_sem_give(&valid_list_sem);
	return false;
}

bool search_valid_source_without_destination(u16_t source_address, struct bt_mesh_route_entry **entry)
{
	struct bt_mesh_route_entry *entry1 = NULL;
	k_sem_take(&valid_list_sem, K_FOREVER); /*take semaphore */
	SYS_SLIST_FOR_EACH_CONTAINER(&valid_list, entry1, node){
		if ((source_address >= entry1->source_address) &&
		    (source_address < entry1->source_address + entry1->source_number_of_elements)) {
			k_sem_give(&valid_list_sem);
			*entry = entry1;
			return true;
		}

	}
	k_sem_give(&valid_list_sem);
	return false;
}

bool search_invalid_destination(u16_t source_address, u16_t destination_address, struct bt_mesh_route_entry **entry)
{
	struct bt_mesh_route_entry *entry1 = NULL;

	k_sem_take(&invalid_list_sem, K_FOREVER); /*take semaphore */
	SYS_SLIST_FOR_EACH_CONTAINER(&invalid_list, entry1, node){
		if ((destination_address >= entry1->destination_address && destination_address < entry1->destination_address + entry1->destination_number_of_elements)
		    && (source_address >= entry1->source_address && source_address < entry1->source_address + entry1->source_number_of_elements)) {
			k_sem_give(&invalid_list_sem);
			*entry = entry1;
			return true;
		}

	}
	k_sem_give(&invalid_list_sem);
	return false;
}

bool search_invalid_destination_without_source(u16_t destination_address, struct bt_mesh_route_entry **entry)
{
	struct bt_mesh_route_entry *entry1 = NULL;

	k_sem_take(&invalid_list_sem, K_FOREVER);
	SYS_SLIST_FOR_EACH_CONTAINER(&invalid_list, entry1, node){
		if ((destination_address >= entry1->destination_address) &&
		    (destination_address < entry1->destination_address + entry1->destination_number_of_elements)) {
			k_sem_give(&invalid_list_sem);
			*entry = entry1;
			return true;
		}

	}
	k_sem_give(&invalid_list_sem);
	return false;
}

bool search_invalid_source_without_destination(u16_t source_address, struct bt_mesh_route_entry **entry)
{
	struct bt_mesh_route_entry *entry1 = NULL;

	k_sem_take(&invalid_list_sem, K_FOREVER);
	SYS_SLIST_FOR_EACH_CONTAINER(&invalid_list, entry1, node){
		if ((source_address >= entry1->source_address) &&
		    (source_address < entry1->source_address + entry1->source_number_of_elements)) {
			k_sem_give(&invalid_list_sem);
			*entry = entry1;
			return true;
		}

	}
	k_sem_give(&invalid_list_sem);
	return false;
}

bool search_valid_destination_with_range(u16_t source_address, u16_t destination_address, u16_t destination_number_of_elements, struct bt_mesh_route_entry **entry)
{
	struct bt_mesh_route_entry *entry1 = NULL;

	k_sem_take(&valid_list_sem, K_FOREVER); /*take semaphore */
	SYS_SLIST_FOR_EACH_CONTAINER(&valid_list, entry1, node){
		if ((entry1->destination_address >= destination_address) &&
		    (entry1->destination_address < (destination_address + destination_number_of_elements)) &&
		    (source_address == entry1->source_address)) {
			k_sem_give(&valid_list_sem);
			*entry = entry1;
			return true;
		}

	}
	k_sem_give(&valid_list_sem);
	return false;
}

bool search_valid_source_with_range(u16_t source_address, u16_t destination_address, u16_t source_number_of_elements, struct bt_mesh_route_entry **entry)
{
	struct bt_mesh_route_entry *entry1 = NULL;

	k_sem_take(&valid_list_sem, K_FOREVER); /*take semaphore */
	SYS_SLIST_FOR_EACH_CONTAINER(&valid_list, entry1, node){
		if ((entry1->source_address >= source_address) &&
		    (entry1->source_address < (source_address + source_number_of_elements)) &&
		    (destination_address == entry1->destination_address)) {
			k_sem_give(&valid_list_sem);
			*entry = entry1;
			return true;
		}

	}
	k_sem_give(&valid_list_sem);
	return false;
}

bool search_invalid_destination_with_range(u16_t source_address, u16_t destination_address, u16_t destination_number_of_elements, struct bt_mesh_route_entry **entry)
{
	struct bt_mesh_route_entry *entry1 = NULL;

	k_sem_take(&invalid_list_sem, K_FOREVER); /*take semaphore */
	SYS_SLIST_FOR_EACH_CONTAINER(&invalid_list, entry1, node){
		if ((entry1->destination_address >= destination_address) &&
		    (entry1->destination_address < (destination_address + destination_number_of_elements)) &&
		    (source_address == entry1->source_address)) {
			k_sem_give(&invalid_list_sem);
			*entry = entry1;
			return true;
		}
	}
	k_sem_give(&invalid_list_sem);
	return false;
}

bool search_invalid_source_with_range(u16_t source_address, u16_t destination_address, u16_t source_number_of_elements, struct bt_mesh_route_entry **entry)
{
	struct bt_mesh_route_entry *entry1 = NULL;

	k_sem_take(&invalid_list_sem, K_FOREVER); /*take semaphore */
	SYS_SLIST_FOR_EACH_CONTAINER(&valid_list, entry1, node){
		if ((entry1->source_address >= source_address) &&
		    (entry1->source_address < (source_address + source_number_of_elements)) &&
		    (destination_address == entry1->destination_address)) {
			k_sem_give(&invalid_list_sem);
			*entry = entry1;
			return true;
		}
	}
	k_sem_give(&invalid_list_sem);
	return false;
}

bool create_entry_valid(struct bt_mesh_route_entry *entry_data)
{
	struct bt_mesh_route_entry *entry_location = NULL;

	if (k_mem_slab_alloc(&routing_table_slab, (void **)&entry_location, ALLOCATION_INTERVAL) == 0) { /*if space found in slab*/
		memset(entry_location, 0, ENTRY_SIZE);                  /* Initializing with zeros */
		k_sem_take(&valid_list_sem, K_FOREVER);                 /*take semaphore */
		sys_slist_append(&valid_list, &entry_location->node);   /*insert node in linkedlist */
		k_sem_give(&valid_list_sem);
	} else    {
		printk("Memory Allocation timeout \n");
		return false;
	}


	entry_location->source_address                    = entry_data->source_address;
	entry_location->destination_address               = entry_data->destination_address;
	entry_location->destination_sequence_number       = entry_data->destination_sequence_number;
	entry_location->hop_count                         = entry_data->hop_count;
	entry_location->next_hop                          = entry_data->next_hop;
	entry_location->repairable                        = entry_data->repairable;
	entry_location->source_number_of_elements         = entry_data->source_number_of_elements;
	entry_location->destination_number_of_elements    = entry_data->destination_number_of_elements;
	k_timer_init(&entry_location->lifetime, delete_entry_valid, NULL);      /*init lifetime timer*/
	k_timer_start(&entry_location->lifetime, LIFETIME, 0);                  /*start timer 200s for each entry */
	return true;
}

bool create_entry_invalid(struct bt_mesh_route_entry *entry_data)
{
	struct bt_mesh_route_entry *entry_location = NULL;

	if (k_mem_slab_alloc(&routing_table_slab, (void **)&entry_location, ALLOCATION_INTERVAL) == 0) { /*if space found in slab*/
		memset(entry_location, 0, ENTRY_SIZE);                  /* Initializing with zeros */
		k_sem_take(&invalid_list_sem, K_FOREVER);               /*take semaphore */
		sys_slist_append(&invalid_list, &entry_location->node); /*insert node in linkedlist */
		k_sem_give(&invalid_list_sem);
	} else    {
		printk("Memory Allocation timeout \n");
		return false;
	}

	entry_location->source_address                    = entry_data->source_address;
	entry_location->destination_address               = entry_data->destination_address;
	entry_location->destination_sequence_number       = entry_data->destination_sequence_number;
	entry_location->hop_count                         = entry_data->hop_count;
	entry_location->next_hop                          = entry_data->next_hop;
	entry_location->repairable                        = entry_data->repairable;
	entry_location->source_number_of_elements         = entry_data->source_number_of_elements;
	entry_location->destination_number_of_elements    = entry_data->destination_number_of_elements;

	k_timer_init(&entry_location->lifetime, delete_entry_invalid, NULL);    /*init lifetime timer*/
	k_timer_start(&entry_location->lifetime, LIFETIME, 0);                  /*start timer 200s for each entry */
	return true;
}

bool create_entry_invalid_with_cb(struct bt_mesh_route_entry *entry_data, \
				  void (*timer_cb)(struct k_timer *timer_id))
{
	struct bt_mesh_route_entry *entry_location = NULL;

	if (k_mem_slab_alloc(&routing_table_slab, (void **)&entry_location, ALLOCATION_INTERVAL) == 0) { /*if space found in slab*/
		memset(entry_location, 0, ENTRY_SIZE);                  /* Initializing with zeros */
		k_sem_take(&invalid_list_sem, K_FOREVER);               /*take semaphore */
		sys_slist_append(&invalid_list, &entry_location->node); /*insert node in linkedlist */
		k_sem_give(&invalid_list_sem);
	} else    {
		printk("Memory Allocation timeout \n");
		return false;
	}

	entry_location->source_address                    = entry_data->source_address;
	entry_location->destination_address               = entry_data->destination_address;
	entry_location->destination_sequence_number       = entry_data->destination_sequence_number;
	entry_location->hop_count                         = entry_data->hop_count;
	entry_location->next_hop                          = entry_data->next_hop;
	entry_location->repairable                        = entry_data->repairable;
	entry_location->source_number_of_elements         = entry_data->source_number_of_elements;
	entry_location->destination_number_of_elements    = entry_data->destination_number_of_elements;

	k_timer_init(&entry_location->lifetime, timer_cb, NULL);
	k_timer_start(&entry_location->lifetime, RREQ_INTERVAL_WAIT, 0);
	return true;
}

bool validate_route(u16_t source_address, u16_t destination_address)
{
	struct bt_mesh_route_entry *entry = NULL;

	if (search_invalid_destination(source_address, destination_address, &entry)) {
		k_timer_stop(&entry->lifetime);
		k_sem_take(&invalid_list_sem, K_FOREVER); /*take semaphore */
		sys_slist_find_and_remove(&invalid_list, &entry->node);
		k_sem_give(&invalid_list_sem);

		k_sem_take(&valid_list_sem, K_FOREVER);         /*take semaphore */
		sys_slist_append(&valid_list, &entry->node);    /*insert node in linkedlist */
		k_sem_give(&valid_list_sem);

		struct k_timer temp;
		entry->lifetime = temp;
		k_timer_init(&entry->lifetime, delete_entry_valid, NULL);       /*init lifetime timer*/
		k_timer_start(&entry->lifetime, LIFETIME, 0);                   /*start timer 200s for each entry */
		// printk("Timer STATUS=%d \n)",k_timer_status_get(&entry->lifetime));
		return true;

	} else   {
		return false;
	}
}

bool invalidate_route(u16_t source_address, u16_t destination_address)
{
	struct bt_mesh_route_entry *entry = NULL;

	if (search_valid_destination(source_address, destination_address, &entry)) {
		k_timer_stop(&entry->lifetime);
		k_sem_take(&valid_list_sem, K_FOREVER); /*take semaphore */
		sys_slist_find_and_remove(&valid_list, &entry->node);
		k_sem_give(&valid_list_sem);

		k_sem_take(&invalid_list_sem, K_FOREVER);       /*take semaphore */
		sys_slist_append(&invalid_list, &entry->node);  /*insert node in linkedlist */
		k_sem_give(&invalid_list_sem);

		struct k_timer temp;
		entry->lifetime = temp;
		k_timer_init(&entry->lifetime, delete_entry_invalid, NULL);     /*init lifetime timer*/
		k_timer_start(&entry->lifetime, LIFETIME, 0);                   /*start timer 200s for each entry */
		// printk("Timer STATUS=%d \n)",k_timer_status_get(&entry->lifetime));
		return true;

	} else   {
		return false;
	}
}

void refresh_lifetime_valid(struct bt_mesh_route_entry *entry)
{
	k_timer_stop(&entry->lifetime);
	struct k_timer temp;
	entry->lifetime = temp;
	k_timer_init(&entry->lifetime, delete_entry_valid, NULL);       /*init lifetime timer*/
	k_timer_start(&entry->lifetime, LIFETIME, 0);                   /*start timer 200s for each entry */
	printk("Lifetime of valid entry refreshed\n");
}

void refresh_lifetime_invalid(struct bt_mesh_route_entry *entry)
{
	k_timer_stop(&entry->lifetime);
	struct k_timer temp;
	entry->lifetime = temp;
	k_timer_init(&entry->lifetime, delete_entry_invalid, NULL);     /*init lifetime timer*/
	k_timer_start(&entry->lifetime, LIFETIME, 0);                   /*start timer 200s for each entry */
	printk("Lifetime of invalid entry refreshed\n");
}

void view_valid_list()
{
	if (sys_slist_is_empty(&valid_list)) {
		printk("Valid List is empty \n");
		return;
	}
	struct bt_mesh_route_entry *entry = NULL;
	k_sem_take(&valid_list_sem, K_FOREVER); /*take semaphore */
	SYS_SLIST_FOR_EACH_CONTAINER(&valid_list, entry, node){
		printk("Valid List:source address=%04x,destination address=%04x \n", entry->source_address, entry->destination_address);
	}
	k_sem_give(&valid_list_sem);
}

void view_invalid_list()
{
	if (sys_slist_is_empty(&invalid_list)) {
		printk("Invalid List is empty \n");
		return;
	}
	struct bt_mesh_route_entry *entry = NULL;
	k_sem_take(&invalid_list_sem, K_FOREVER); /*take semaphore */
	SYS_SLIST_FOR_EACH_CONTAINER(&invalid_list, entry, node){
		printk("Invalid List:source address=%04x,destination address=%04x \n", entry->source_address, entry->destination_address);
	}
	k_sem_give(&invalid_list_sem);
}
