#include <zephyr.h>
#include <misc/printk.h>
#include <misc/slist.h> /*include LinkedList*/

#define NUMBER_OF_ENTRIES 20  /*Maximum number of entries in the table */
#define ALLIGNED 4  /*Memory Allignment*/
#define ALLOCATION_INTERVAL 100 /* maximum time taken to allocate from slab */
#define ENTRY_SIZE sizeof(struct bt_mesh_route_entry)
#define LIFETIME  K_MSEC(1000)

struct bt_mesh_route_entry {
  u16_t source_address; /*Destination address==2 byte */
  u16_t destination_address; /*Source address  == 2 byte*/
  u32_t destination_sequence_number; /*Source Sequence number == 4 byte*/
  u16_t next_hop; /*Next hop address == 2 byte*/
  u8_t  hop_count; /* hop count == 1 byte */
  bool  repairable;
  struct k_timer lifetime; /*lifetime timer entry == 52 byte*/
  sys_snode_t node; /* Linkedlist node == 4 byte */
};

sys_slist_t valid_list;  /*global linked list for  entries */
K_SEM_DEFINE(valid_list_sem, 1, 1); /*Binary semaphore for  linked list critical section*/
sys_slist_t invalid_list;  /*global linked list for in entries */
K_SEM_DEFINE(invalid_list_sem, 1, 1); /*Binary semaphore for in list critical section */
struct k_mem_slab routing_table_slab;  /*Memory slab for entries */
K_MEM_SLAB_DEFINE(routing_table_slab, ENTRY_SIZE, NUMBER_OF_ENTRIES, ALLIGNED); /*Macro to define memory slab*/

void routing_table_init ()
{
  sys_slist_init(&valid_list);
  sys_slist_init(&invalid_list);
}

 void delete_entry_valid (struct k_timer *timer_id)
{
  struct bt_mesh_route_entry* entry=CONTAINER_OF(timer_id,struct bt_mesh_route_entry,lifetime); /* container of timer_id to be deleted*/
  k_sem_take(&valid_list_sem,K_FOREVER); /* take semaphore */
  sys_slist_find_and_remove(&valid_list,&entry->node); /*delete node from linked list */
  k_sem_give(&valid_list_sem); /*return semaphore */
	k_mem_slab_free(&routing_table_slab,(void **)&entry); /*free space in slab*/
	printk(" valid Entry Deleted \n");
}

 void delete_entry_invalid (struct k_timer *timer_id)
{
  struct bt_mesh_route_entry* entry=CONTAINER_OF(timer_id,struct bt_mesh_route_entry,lifetime); /* container of timer_id to be deleted*/
  k_sem_take(&invalid_list_sem,K_FOREVER); /* take semaphore */
  sys_slist_find_and_remove(&invalid_list,&entry->node); /*delete node from linked list */
  k_sem_give(&invalid_list_sem); /*return semaphore */
	k_mem_slab_free(&routing_table_slab,(void **)&entry); /*free space in slab*/
	printk("Invalid Entry Deleted \n");
}

bool search_valid_destination (u16_t source_addr,u16_t destination_addr,struct bt_mesh_route_entry** entry)
{
  struct bt_mesh_route_entry* entry1=NULL;
  k_sem_take(&valid_list_sem,K_FOREVER); /*take semaphore */
	SYS_SLIST_FOR_EACH_CONTAINER(&valid_list,entry1,node)
	{
	 if (entry1->destination_address==destination_addr && entry1->source_address==source_addr)
	 {
     k_sem_give(&valid_list_sem);
		 *entry=entry1;
		 return true;
	 }

	}
  k_sem_give(&valid_list_sem);
	return false;
}

bool search_invalid_destination (u16_t source_addr,u16_t destination_addr,struct bt_mesh_route_entry** entry)
{
  struct bt_mesh_route_entry* entry1=NULL;
  k_sem_take(&invalid_list_sem,K_FOREVER); /*take semaphore */
	SYS_SLIST_FOR_EACH_CONTAINER(&invalid_list,entry1,node)
	{
		if(entry1->destination_address==destination_addr && entry1->source_address==source_addr)
	 {
     k_sem_give(&invalid_list_sem);
		 *entry=entry1;
		 return true;
	 }

	}
  k_sem_give(&invalid_list_sem);
	return false;
}


bool create_entry_valid (struct bt_mesh_route_entry* entry_data,struct bt_mesh_route_entry* entry_location)
{
  if (entry_location == NULL)
  {
    if(k_mem_slab_alloc(&routing_table_slab,(void**)&entry_location, ALLOCATION_INTERVAL) == 0) /*if space found in slab*/
  	 {
       memset(entry_location, 0, ENTRY_SIZE); /* Initializing with zeros */
			 k_sem_take(&valid_list_sem,K_FOREVER); /*take semaphore */
			 sys_slist_append(&valid_list,&entry_location->node); /*insert node in linkedlist */
			 k_sem_give(&valid_list_sem);
  	 }
  	else {
        printk("Memory Allocation timeout \n");
        return false;
  			}
  }
  else {
    k_timer_stop(&entry_location->lifetime);
  }


  entry_location->source_address               = entry_data-> source_address;
  entry_location->destination_address          = entry_data-> destination_address;
  entry_location->destination_sequence_number  = entry_data-> destination_sequence_number;
  entry_location->hop_count                    = entry_data-> hop_count;
  entry_location->next_hop                     = entry_data-> next_hop;
  entry_location->repairable                   = entry_data-> repairable;



  k_timer_init (&entry_location->lifetime, delete_entry_valid, NULL); /*init lifetime timer*/
  k_timer_start(&entry_location->lifetime, LIFETIME, 0); /*start timer 200s for each entry */
  return true;
}

bool create_entry_invalid (struct bt_mesh_route_entry* entry_data,struct bt_mesh_route_entry* entry_location)
{
  if (entry_location == NULL)
  {
    if(k_mem_slab_alloc(&routing_table_slab,(void**)&entry_location, ALLOCATION_INTERVAL) == 0) /*if space found in slab*/
  	 {
       memset(entry_location, 0, ENTRY_SIZE); /* Initializing with zeros */
			 k_sem_take(&invalid_list_sem,K_FOREVER); /*take semaphore */
		   sys_slist_append(&invalid_list,&entry_location->node); /*insert node in linkedlist */
		   k_sem_give(&invalid_list_sem);
  	 }
  	else {
        printk("Memory Allocation timeout \n");
        return false;
  			}
  }
  else {
    k_timer_stop(&entry_location->lifetime);
  }


  entry_location->source_address               = entry_data-> source_address;
  entry_location->destination_address          = entry_data-> destination_address;
  entry_location->destination_sequence_number  = entry_data-> destination_sequence_number;
  entry_location->hop_count                    = entry_data-> hop_count;
  entry_location->next_hop                     = entry_data-> next_hop;
  entry_location->repairable                   = entry_data-> repairable;

  k_timer_init (&entry_location->lifetime, delete_entry_invalid, NULL); /*init lifetime timer*/
  k_timer_start(&entry_location->lifetime, LIFETIME, 0); /*start timer 200s for each entry */
  return true;
}

void view_valid_list ()
{
	struct bt_mesh_route_entry* entry =NULL;
	k_sem_take(&valid_list_sem,K_FOREVER); /*take semaphore */
	SYS_SLIST_FOR_EACH_CONTAINER(&valid_list,entry,node)
	{
  	printk("address1=%04x,address2=%04x \n",entry->source_address,entry->destination_address);
	}
	k_sem_give(&valid_list_sem);
}
