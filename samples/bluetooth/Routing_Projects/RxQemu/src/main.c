#include <misc/printk.h>
#include <misc/byteorder.h>
#include <bluetooth/bluetooth.h>
#include <bluetooth/conn.h>
#include <bluetooth/l2cap.h>
#include <bluetooth/hci.h>
#include <bluetooth/mesh.h>
#include <stdio.h>
#include </media/rana/DE6E144C6E142037/Engineering/GP/GP/Zephyr/github/after_finals/zephyr/subsys/bluetooth/host/mesh/net.h>
#include </media/rana/DE6E144C6E142037/Engineering/GP/GP/Zephyr/github/after_finals/zephyr/subsys/bluetooth/host/mesh/transport.h>
#define CID_INTEL 0x0002 /*Company identifier assigned by the Bluetooth SIG*/
#define NODE_ADDR 0x0005 /*Unicast Address*/
#define GROUP_ADDR 0xc000 /*The Address to use for pub and sub*/


static const u8_t net_key[16] = {
	0x01, 0x23, 0x45, 0x67, 0x89, 0xab, 0xcd, 0xef,
	0x01, 0x23, 0x45, 0x67, 0x89, 0xab, 0xcd, 0xef,
};
static const u8_t dev_key[16] = {
	0x01, 0x23, 0x45, 0x67, 0x89, 0xab, 0xcd, 0xef,
	0x01, 0x23, 0x45, 0x67, 0x89, 0xab, 0xcd, 0xef,
};
static const u8_t app_key[16] = {
	0x01, 0x23, 0x45, 0x67, 0x89, 0xab, 0xcd, 0xef,
	0x01, 0x23, 0x45, 0x67, 0x89, 0xab, 0xcd, 0xef,
};

static const u16_t net_idx;
static const u16_t app_idx;
static const u32_t iv_index;
static u8_t flags;
static u16_t addr = NODE_ADDR;
static u32_t seq=0;


#define BT_MESH_MODEL_OP_GEN_ONOFF_GET		BT_MESH_MODEL_OP_2(0x82, 0x01)
#define BT_MESH_MODEL_OP_GEN_ONOFF_SET		BT_MESH_MODEL_OP_2(0x82, 0x02)
#define BT_MESH_MODEL_OP_GEN_ONOFF_SET_UNACK	BT_MESH_MODEL_OP_2(0x82, 0x03)
#define BT_MESH_MODEL_OP_GEN_ONOFF_STATUS	BT_MESH_MODEL_OP_2(0x82, 0x04)

static void gen_onoff_set(struct bt_mesh_model *model, // Mesh Model instance (mesh/access.h line314)
			  struct bt_mesh_msg_ctx *ctx,//Message sending context
			  struct net_buf_simple *buf);//buffer

static void gen_onoff_set_unack(struct bt_mesh_model *model,
			  struct bt_mesh_msg_ctx *ctx,
			  struct net_buf_simple *buf);

static void gen_onoff_get(struct bt_mesh_model *model,
			  struct bt_mesh_msg_ctx *ctx,
			  struct net_buf_simple *buf);



static struct bt_mesh_cfg_srv cfg_srv = {
	.relay = BT_MESH_RELAY_ENABLED, // TODO: we modify this to enable
	.beacon = BT_MESH_BEACON_ENABLED,
	#if defined(CONFIG_BT_MESH_FRIEND)
	.frnd = BT_MESH_FRIEND_ENABLED,
	#else
	.frnd = BT_MESH_FRIEND_NOT_SUPPORTED,
	#endif
	#if defined(CONFIG_BT_MESH_GATT_PROXY)
	.gatt_proxy = BT_MESH_GATT_PROXY_ENABLED,
	#else
	.gatt_proxy = BT_MESH_GATT_PROXY_NOT_SUPPORTED,
	#endif
	.default_ttl = 7,

	/* 3 transmissions with 20ms interval */
	.net_transmit = BT_MESH_TRANSMIT(2, 20),
	.relay_retransmit = BT_MESH_TRANSMIT(2, 20),
};


static struct bt_mesh_cfg_cli cfg_cli = {
};

/*
 * Health Server Declaration
 */

static struct bt_mesh_health_srv health_srv = {
};

/*
 * Publication Declarations
 *
 * The publication messages are initialized to the
 * the size of the opcode + content
 *
 * For publication, the message must be in static or global as
 * it is re-transmitted several times. This occurs
 * after the function that called bt_mesh_model_publish() has
 * exited and the stack is no longer valid.
 *
 * Note that the additional 4 bytes for the AppMIC is not needed
 * because it is added to a stack variable at the time a
 * transmission occurs.
 *
 */

BT_MESH_HEALTH_PUB_DEFINE(health_pub, 0);
BT_MESH_MODEL_PUB_DEFINE(gen_onoff_pub_srv, NULL, 2 + 2);


/*Three server msgs*/
static const struct bt_mesh_model_op gen_onoff_srv_op[] = {
 { BT_MESH_MODEL_OP_GEN_ONOFF_GET, 0, gen_onoff_get }, /*{opcode, Minimum required message length, Message handler for the opcode}*/
 { BT_MESH_MODEL_OP_GEN_ONOFF_SET, 2, gen_onoff_set },
 { BT_MESH_MODEL_OP_GEN_ONOFF_SET_UNACK, 2, gen_onoff_set_unack },
 BT_MESH_MODEL_OP_END,
};




struct onoff_state {
	u8_t current;
	u8_t previous;
	u8_t led_gpio_pin;
};

static struct onoff_state onoff_state[] = {
	{ .led_gpio_pin = 0 },
};


/*
 *
 * Element Model Declarations
 *
 * Element 0 Root Models
 */

static struct bt_mesh_model root_models[] = {
	BT_MESH_MODEL_CFG_SRV(&cfg_srv),
	BT_MESH_MODEL_CFG_CLI(&cfg_cli),
	BT_MESH_MODEL_HEALTH_SRV(&health_srv, &health_pub),
	BT_MESH_MODEL(BT_MESH_MODEL_ID_GEN_ONOFF_SRV, gen_onoff_srv_op,
		      &gen_onoff_pub_srv, &onoff_state[0]),
};

struct bt_mesh_model *mod_srv_sw[] = {
		&root_models[3],
};

static struct bt_mesh_elem elements[] = {
	BT_MESH_ELEM(0, root_models, BT_MESH_MODEL_NONE),
};

static const struct bt_mesh_comp comp = {
	.cid = CID_INTEL,
	.elem = elements,
	.elem_count = ARRAY_SIZE(elements),
};


static void gen_onoff_get(struct bt_mesh_model *model,
			  struct bt_mesh_msg_ctx *ctx,
			  struct net_buf_simple *buf)
{
	NET_BUF_SIMPLE_DEFINE(msg, 2 + 1 + 4);
	struct onoff_state *onoff_state = model->user_data;

	printk("addr 0x%04x onoff 0x%02x \n",
		    model->elem->addr, onoff_state->current);
		bt_mesh_model_msg_init(&msg, BT_MESH_MODEL_OP_GEN_ONOFF_STATUS);
		net_buf_simple_add_u8(&msg, onoff_state->current);

		if (bt_mesh_model_send(model, ctx, &msg, NULL, NULL)) {
		printk("Unable to send On Off Status response \n");
	}
}

static void gen_onoff_set_unack(struct bt_mesh_model *model,
				struct bt_mesh_msg_ctx *ctx,
				struct net_buf_simple *buf)
{
	struct net_buf_simple *msg = model->pub->msg;
	struct onoff_state *onoff_state = model->user_data;
	int err;

	onoff_state->current = net_buf_simple_pull_u8(buf);
	printk("addr 0x%04x state 0x%02x ",model->elem->addr, onoff_state->current);

	printk("LED is toggled \n");

	/*
	 * If a server has a publish address, it is required to
	 * publish status on a state change
	 *
	 * See Mesh Profile Specification 3.7.6.1.2
	 *
	 * Only publish if there is an assigned address
	 */

	if (onoff_state->previous != onoff_state->current &&
	    model->pub->addr != BT_MESH_ADDR_UNASSIGNED) {
		printk("publish last 0x%02x cur 0x%02x \n",
			    onoff_state->previous,
			    onoff_state->current);
		onoff_state->previous = onoff_state->current;
		bt_mesh_model_msg_init(msg,
				       BT_MESH_MODEL_OP_GEN_ONOFF_STATUS);
		net_buf_simple_add_u8(msg, onoff_state->current);
		err = bt_mesh_model_publish(model);
		if (err) {
			printk("bt_mesh_model_publish err %d \n", err);
		}
	}
}

static void gen_onoff_set(struct bt_mesh_model *model,
			  struct bt_mesh_msg_ctx *ctx,
			  struct net_buf_simple *buf)
{
	gen_onoff_set_unack(model, ctx, buf);
	//gen_onoff_get(model, ctx, buf);
}



static int output_number(bt_mesh_output_action_t action, uint32_t number)
{
	printk("OOB Number %u \n", number);
	return 0;
}

static int output_string(const char *str)
{
	printk("OOB String %s \n", str);
	return 0;
}


static u8_t dev_uuid[16] = { 0xdd, 0xdd };




static const struct bt_mesh_prov prov = {
	.uuid = dev_uuid,
 #if 1
	.output_size = 6,
	.output_actions = (BT_MESH_DISPLAY_NUMBER | BT_MESH_DISPLAY_STRING),
	.output_number = output_number,
	.output_string = output_string,
 #else
	.output_size = 0,
	.output_actions = 0,
	.output_number = 0,
 #endif
};


static void configure(void)
 {

 	/* Add Application Key */
 	bt_mesh_cfg_app_key_add(net_idx, addr, net_idx, app_idx, app_key, NULL);

	/*
	   Bind the App key to BT_MESH_MODEL_ID_GEN_ONOFF_SRV (ONOFF Server Model)
	   node address,element address */
 	bt_mesh_cfg_mod_app_bind(net_idx,addr, addr, app_idx, BT_MESH_MODEL_ID_GEN_ONOFF_SRV, NULL);
	printk("Binding complete \n");
	bt_mesh_cfg_mod_sub_add(net_idx, addr, addr, NODE_ADDR + 1, BT_MESH_MODEL_ID_GEN_ONOFF_SRV, NULL);
	printk("Subscribing complete \n");
	/*Add Subscription, LED0 (ELEM 0) is Subscribing to GROUP_ADDR */
 	 bt_mesh_cfg_mod_sub_add(net_idx, addr, addr, GROUP_ADDR,BT_MESH_MODEL_ID_GEN_ONOFF_SRV, NULL);



 }


 static void bt_ready(int err)
{
	if (err) {
		printk("Bluetooth init failed (err %d) \n", err);
		return;
	}

	printk("Bluetooth initialized \n");

	err = bt_mesh_init(&prov, &comp);
	if (err) {
		printk("Initializing mesh failed (err %d) \n", err);
		return;
	}

	//bt_mesh_prov_enable(BT_MESH_PROV_GATT | BT_MESH_PROV_ADV);
	printk("Mesh initialized \n");

	err = bt_mesh_provision(net_key, net_idx, flags, iv_index, seq, addr,
				dev_key);
	if (err) {
		printk("Provisioning failed (err %d) \n", err);
		return;
	}

	printk("Provisioning completed\n");
	configure();
}


void main(void)
{
	int err;
	printk("Initializing...\n");
	err = bt_enable(bt_ready);
	if (err) {
	printk("Bluetooth init failed (err %d)\n", err);
	}
}
