#include "ble.h"
#include "main.h"
#include "app_io.h"
#include "ems_pwm.h"

#include <zephyr/kernel.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/logging/log.h>
#include <hal/nrf_gpio.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gap.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/settings/settings.h>
#include <zephyr/drivers/gpio.h>
#include <bluetooth/services/bms.h>
#include <zephyr/drivers/pwm.h>

LOG_MODULE_REGISTER(BLE, LOG_LEVEL_DBG);

#define DEVICE_NAME CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN (sizeof(DEVICE_NAME) - 1)
#define CONSOLE_LABEL                   DT_LABEL(DT_CHOSEN(zephyr_console))
#if !DT_NODE_EXISTS(DT_PATH(zephyr_user)) || \
    !DT_NODE_HAS_PROP(DT_PATH(zephyr_user), io_channels)
#error "No suitable devicetree overlay specified"
#endif

#define DT_SPEC_AND_COMMA(node_id, prop, idx) \
    ADC_DT_SPEC_GET_BY_IDX(node_id, idx),

/* STEP 3.2.1 - Define advertising parameter for no Accept List */
#define BT_LE_ADV_CONN_NO_ACCEPT_LIST                                   \
	BT_LE_ADV_PARAM(BT_LE_ADV_OPT_CONNECTABLE | BT_LE_ADV_OPT_USE_NAME, \
					BT_GAP_ADV_FAST_INT_MIN_2, BT_GAP_ADV_FAST_INT_MAX_2, NULL)
/* STEP 3.2.2 - Define advertising parameter for when Accept List is used */
#define BT_LE_ADV_CONN_ACCEPT_LIST                                          \
	BT_LE_ADV_PARAM(BT_LE_ADV_OPT_CONNECTABLE | BT_LE_ADV_OPT_FILTER_CONN | \
						BT_LE_ADV_OPT_USE_NAME,                             \
					BT_GAP_ADV_FAST_INT_MIN_2, BT_GAP_ADV_FAST_INT_MAX_2, NULL)

// EMS service UUIDs
#define EMS_SERVICE_UUID_VAL                BT_UUID_128_ENCODE(0x5cf7d300, 0x0ac9, 0x4df4, 0xb0e5, 0xe28a976fe53b)
#define EMS_CONTROL_UUID_VAL                BT_UUID_128_ENCODE(0x5cf7d301, 0x0ac9, 0x4df4, 0xb0e5, 0xe28a976fe53b)
#define EMS_SESSION_UUID_VAL                BT_UUID_128_ENCODE(0x5cf7d302, 0x0ac9, 0x4df4, 0xb0e5, 0xe28a976fe53b)
#define EMS_CHAN1_UUID_VAL                  BT_UUID_128_ENCODE(0x5cf7d304, 0x0ac9, 0x4df4, 0xb0e5, 0xe28a976fe53b)
#define EMS_CHAN2_UUID_VAL                  BT_UUID_128_ENCODE(0x5cf7d305, 0x0ac9, 0x4df4, 0xb0e5, 0xe28a976fe53b)
#define EMS_CHAN3_UUID_VAL                  BT_UUID_128_ENCODE(0x5cf7d306, 0x0ac9, 0x4df4, 0xb0e5, 0xe28a976fe53b)
#define EMS_CHAN4_UUID_VAL                  BT_UUID_128_ENCODE(0x5cf7d307, 0x0ac9, 0x4df4, 0xb0e5, 0xe28a976fe53b)

// EMS PARAMETERS CHANGING SERVICE UUIDs
#define EMS_CHANGE_SERVICE_UUID_VAL         BT_UUID_128_ENCODE(0x5cf7d501, 0x0ac9, 0x4df4, 0xb0e5, 0xe28a976fe53b)
#define CHAN1_PWM_PERIOD                    BT_UUID_128_ENCODE(0x5cf7d502, 0x0ac9, 0x4df4, 0xb0e5, 0xe28a976fe53b)
#define CHAN1_PWM_DEFAULT_ON_TIME           BT_UUID_128_ENCODE(0x5cf7d503, 0x0ac9, 0x4df4, 0xb0e5, 0xe28a976fe53b)
#define POWER_SIGNAL_DEFAULT_INTERVAL       BT_UUID_128_ENCODE(0x5cf7d504, 0x0ac9, 0x4df4, 0xb0e5, 0xe28a976fe53b)
#define POWER_SIGNAL_DEFAULT_DURATION       BT_UUID_128_ENCODE(0x5cf7d505, 0x0ac9, 0x4df4, 0xb0e5, 0xe28a976fe53b)

#define CHAN0_PLAYBACK_COUNT                BT_UUID_128_ENCODE(0x5cf7d506, 0x0ac9, 0x4df4, 0xb0e5, 0xe28a976fe53b)
#define CHAN0_BEFORE_INDUC                  BT_UUID_128_ENCODE(0x5cf7d507, 0x0ac9, 0x4df4, 0xb0e5, 0xe28a976fe53b)
#define CHAN0_DEFAULT_ON_DURATION           BT_UUID_128_ENCODE(0x5cf7d508, 0x0ac9, 0x4df4, 0xb0e5, 0xe28a976fe53b)
#define CHAN0_DEFAULT_DURATION              BT_UUID_128_ENCODE(0x5cf7d509, 0x0ac9, 0x4df4, 0xb0e5, 0xe28a976fe53b)

#define CHAN23_PWM_DEFAULT_PERIOD_NS        BT_UUID_128_ENCODE(0x5cf7d511, 0x0ac9, 0x4df4, 0xb0e5, 0xe28a976fe53b)
#define CHAN23_PWM_DEFAULT_ON_TIME_NS       BT_UUID_128_ENCODE(0x5cf7d512, 0x0ac9, 0x4df4, 0xb0e5, 0xe28a976fe53b)

#define CHARGER_UUID                        BT_UUID_128_ENCODE(0x5cf7d518, 0x0ac9, 0x4df4, 0xb0e5, 0xe28a976fe53b)
#define CHARGER_ON_OFF                      BT_UUID_128_ENCODE(0x5cf7d519, 0x0ac9, 0x4df4, 0xb0e5, 0xe28a976fe53b)


#define BATT_MAX_VOLTAGE_MV                 2100 // Max battery voltage (100%)
#define BATT_MIN_VOLTAGE_MV                 1500 // Minimum allowed battery voltage (0%)

#define PWM_PERIOD   PWM_MSEC(60)

#define PWM_MIN_DUTY_CYCLE 20000000
#define PWM_MAX_DUTY_CYCLE 50000000

// Read callbacks
static ssize_t ems_read_callback(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf, uint16_t len, uint16_t offset);
static ssize_t batt_read_callback(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf, uint16_t len, uint16_t offset);
static ssize_t charger_read_callback(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf, uint16_t len, uint16_t offset); 

// Write callbacks
static ssize_t ems_write_cb(struct bt_conn *conn, const struct bt_gatt_attr *attr, const void *buf, uint16_t len, uint16_t offset, uint8_t flags);

// CCC changed callbacks
static void batt_ccc_value_changed(const struct bt_gatt_attr *attr, uint16_t value);

// UUIDs
static struct bt_uuid_128 ems_service_uuid = BT_UUID_INIT_128(EMS_SERVICE_UUID_VAL);
static struct bt_uuid_128 ems_control_uuid = BT_UUID_INIT_128(EMS_CONTROL_UUID_VAL);
static struct bt_uuid_128 ems_session_uuid = BT_UUID_INIT_128(EMS_SESSION_UUID_VAL);
static struct bt_uuid_128 ems_chan1_uuid = BT_UUID_INIT_128(EMS_CHAN1_UUID_VAL);
static struct bt_uuid_128 ems_chan2_uuid = BT_UUID_INIT_128(EMS_CHAN2_UUID_VAL);
static struct bt_uuid_128 ems_chan3_uuid = BT_UUID_INIT_128(EMS_CHAN3_UUID_VAL);
static struct bt_uuid_128 ems_chan4_uuid = BT_UUID_INIT_128(EMS_CHAN4_UUID_VAL);

static struct bt_uuid_128 charger_uuid = BT_UUID_INIT_128(CHARGER_UUID);
static struct bt_uuid_128 charger_on_off_uuid = BT_UUID_INIT_128(CHARGER_ON_OFF);

static struct bt_uuid_128 ems_change_service_uuid = BT_UUID_INIT_128(EMS_CHANGE_SERVICE_UUID_VAL);
static struct bt_uuid_128 pwm_period_uuid = BT_UUID_INIT_128(CHAN1_PWM_PERIOD);
static struct bt_uuid_128 pwm_ontime_uuid = BT_UUID_INIT_128(CHAN1_PWM_DEFAULT_ON_TIME);
static struct bt_uuid_128 pwm_interval_uuid = BT_UUID_INIT_128(POWER_SIGNAL_DEFAULT_INTERVAL);
static struct bt_uuid_128 pwm_duration_uuid = BT_UUID_INIT_128(POWER_SIGNAL_DEFAULT_DURATION);

static struct bt_uuid_128 chan23_duration_uuid = BT_UUID_INIT_128(CHAN23_PWM_DEFAULT_PERIOD_NS);
static struct bt_uuid_128 chan23_on_duration_uuid = BT_UUID_INIT_128(CHAN23_PWM_DEFAULT_ON_TIME_NS);

static struct bt_uuid_128 chan0_playback_uuid = BT_UUID_INIT_128(CHAN0_PLAYBACK_COUNT);
static struct bt_uuid_128 chan0_induct_uuid = BT_UUID_INIT_128(CHAN0_BEFORE_INDUC);
static struct bt_uuid_128 chan0_on_duration_uuid = BT_UUID_INIT_128(CHAN0_DEFAULT_ON_DURATION );
static struct bt_uuid_128 chan0_duration_uuid = BT_UUID_INIT_128(CHAN0_DEFAULT_DURATION);

static struct bt_uuid_16 batt_service_uuid = BT_UUID_INIT_16(0x180f);
//static struct bt_uuid_16 batt_val_uuid = BT_UUID_INIT_16(0x2a19);
static struct bt_uuid_128 batt_val_uuid = BT_UUID_INIT_128(
    0xf0, 0xde, 0xbc, 0x9a, 0x78, 0x56, 0x34, 0x12,
    0xf0, 0xde, 0xbc, 0x9a, 0x78, 0x56, 0x34, 0x12);  // Just an example


static const uint8_t bms_auth_code[] = {'A', 'B', 'C', 'D'};

static const struct bt_data adv_data[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
    BT_DATA_BYTES(BT_DATA_UUID128_SOME, EMS_SERVICE_UUID_VAL)
};

static const struct bt_data sd[] = {
	BT_DATA_BYTES(BT_DATA_UUID16_ALL, BT_UUID_16_ENCODE(BT_UUID_BMS_VAL),BT_UUID_16_ENCODE(BT_UUID_DIS_VAL)),
};

K_MSGQ_DEFINE(ble_msgq, sizeof(struct ble_msg), 5, 4); // BLE message queue

extern struct k_msgq main_msgq; // Main message queue, defined in main.h

struct bt_conn *default_conn; // Default connection object (only one connection allowed)
struct ems_def ems_session_data = {0}; // EMS session data
uint8_t batt_percent; // Battery percentage
extern k_tid_t led_thread; // LED thread
uint8_t disconnect_reason = 0; // Disconnect reason
extern k_tid_t my_tid1;
extern k_tid_t my_tid2;

extern k_tid_t my_tid3;


static bool security_set = false;  // Flag to check if security is set
bool pairing_on = true;

// EMS Service definition
BT_GATT_SERVICE_DEFINE(
    ems_service,
    BT_GATT_PRIMARY_SERVICE(&ems_service_uuid.uuid),
    BT_GATT_CHARACTERISTIC(&ems_control_uuid.uuid,
                            (BT_GATT_CHRC_WRITE | BT_GATT_CHRC_WRITE_WITHOUT_RESP | BT_GATT_CHRC_READ),
                            (BT_GATT_PERM_WRITE_ENCRYPT | BT_GATT_PERM_READ_ENCRYPT),
                            ems_read_callback, ems_write_cb, NULL),
    BT_GATT_CHARACTERISTIC(&ems_session_uuid.uuid,
                            (BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE | BT_GATT_CHRC_WRITE_WITHOUT_RESP),
                            (BT_GATT_PERM_WRITE_ENCRYPT | BT_GATT_PERM_READ_ENCRYPT),
                            ems_read_callback, ems_write_cb, NULL),
    BT_GATT_CHARACTERISTIC(&ems_chan1_uuid.uuid,
                            (BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE | BT_GATT_CHRC_WRITE_WITHOUT_RESP),
                            (BT_GATT_PERM_WRITE_ENCRYPT | BT_GATT_PERM_READ_ENCRYPT),
                            ems_read_callback, ems_write_cb, NULL),
    BT_GATT_CHARACTERISTIC(&ems_chan2_uuid.uuid,
                            (BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE | BT_GATT_CHRC_WRITE_WITHOUT_RESP),
                            (BT_GATT_PERM_WRITE_ENCRYPT | BT_GATT_PERM_READ_ENCRYPT),
                            ems_read_callback, ems_write_cb, NULL),
    BT_GATT_CHARACTERISTIC(&ems_chan3_uuid.uuid,
                            (BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE | BT_GATT_CHRC_WRITE_WITHOUT_RESP),
                            (BT_GATT_PERM_WRITE_ENCRYPT | BT_GATT_PERM_READ_ENCRYPT),
                            ems_read_callback, ems_write_cb, NULL),
    BT_GATT_CHARACTERISTIC(&ems_chan4_uuid.uuid,
                            (BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE | BT_GATT_CHRC_WRITE_WITHOUT_RESP),
                            (BT_GATT_PERM_WRITE_ENCRYPT | BT_GATT_PERM_READ_ENCRYPT),
                            ems_read_callback, ems_write_cb, NULL),
);

// EMS PARAMETERS CHANGE SERVICE
BT_GATT_SERVICE_DEFINE(
    ems_change_service,
    BT_GATT_PRIMARY_SERVICE(&ems_change_service_uuid.uuid),
    BT_GATT_CHARACTERISTIC(&pwm_period_uuid.uuid,
                            (BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE | BT_GATT_CHRC_WRITE_WITHOUT_RESP),
                            (BT_GATT_PERM_WRITE_ENCRYPT | BT_GATT_PERM_READ_ENCRYPT),
                            ems_read_callback, ems_write_cb, NULL),
    BT_GATT_CHARACTERISTIC(&pwm_ontime_uuid.uuid,
                            (BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE | BT_GATT_CHRC_WRITE_WITHOUT_RESP),
                            (BT_GATT_PERM_WRITE_ENCRYPT | BT_GATT_PERM_READ_ENCRYPT),
                            ems_read_callback, ems_write_cb, NULL),
    BT_GATT_CHARACTERISTIC(&pwm_interval_uuid.uuid,
                            (BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE | BT_GATT_CHRC_WRITE_WITHOUT_RESP),
                            (BT_GATT_PERM_WRITE_ENCRYPT | BT_GATT_PERM_READ_ENCRYPT),
                            ems_read_callback, ems_write_cb, NULL),
    BT_GATT_CHARACTERISTIC(&pwm_duration_uuid.uuid,
                            (BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE | BT_GATT_CHRC_WRITE_WITHOUT_RESP),
                            (BT_GATT_PERM_WRITE_ENCRYPT | BT_GATT_PERM_READ_ENCRYPT),
                            ems_read_callback, ems_write_cb, NULL),
    BT_GATT_CHARACTERISTIC(&chan0_playback_uuid.uuid,
                            (BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE | BT_GATT_CHRC_WRITE_WITHOUT_RESP),
                            (BT_GATT_PERM_WRITE_ENCRYPT | BT_GATT_PERM_READ_ENCRYPT),
                            ems_read_callback, ems_write_cb, NULL),
    BT_GATT_CHARACTERISTIC(&chan0_induct_uuid.uuid,
                            (BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE | BT_GATT_CHRC_WRITE_WITHOUT_RESP),
                            (BT_GATT_PERM_WRITE_ENCRYPT | BT_GATT_PERM_READ_ENCRYPT),
                            ems_read_callback, ems_write_cb, NULL),  
    BT_GATT_CHARACTERISTIC(&chan0_on_duration_uuid.uuid,
                            (BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE | BT_GATT_CHRC_WRITE_WITHOUT_RESP),
                            (BT_GATT_PERM_WRITE_ENCRYPT | BT_GATT_PERM_READ_ENCRYPT),
                            ems_read_callback, ems_write_cb, NULL),
    BT_GATT_CHARACTERISTIC(&chan0_duration_uuid.uuid,
                            (BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE | BT_GATT_CHRC_WRITE_WITHOUT_RESP),
                            (BT_GATT_PERM_WRITE_ENCRYPT | BT_GATT_PERM_READ_ENCRYPT),
                            ems_read_callback, ems_write_cb, NULL),     
    BT_GATT_CHARACTERISTIC(&chan23_duration_uuid.uuid,
                            (BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE | BT_GATT_CHRC_WRITE_WITHOUT_RESP),
                            (BT_GATT_PERM_WRITE_ENCRYPT | BT_GATT_PERM_READ_ENCRYPT),
                            ems_read_callback, ems_write_cb, NULL),
    BT_GATT_CHARACTERISTIC(&chan23_on_duration_uuid.uuid,
                            (BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE | BT_GATT_CHRC_WRITE_WITHOUT_RESP),
                            (BT_GATT_PERM_WRITE_ENCRYPT | BT_GATT_PERM_READ_ENCRYPT),
                            ems_read_callback, ems_write_cb, NULL),                         
    BT_GATT_CHARACTERISTIC(&charger_on_off_uuid.uuid,
                            (BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE | BT_GATT_CHRC_WRITE_WITHOUT_RESP),
                            (BT_GATT_PERM_WRITE_ENCRYPT | BT_GATT_PERM_READ_ENCRYPT),
                            ems_read_callback, ems_write_cb, NULL),
    BT_GATT_CHARACTERISTIC(&charger_uuid.uuid,
                            BT_GATT_CHRC_READ,  // Read-only
                            BT_GATT_PERM_READ_ENCRYPT,  // Read permission with encryption
                            charger_read_callback, NULL, NULL),
);

// Battery Service definition
BT_GATT_SERVICE_DEFINE(
    battery_service,
    BT_GATT_PRIMARY_SERVICE(&batt_service_uuid.uuid),
    BT_GATT_CHARACTERISTIC(&batt_val_uuid.uuid,
                            BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
                            BT_GATT_PERM_READ_ENCRYPT,
                            batt_read_callback, NULL, NULL),
    BT_GATT_CCC(batt_ccc_value_changed, BT_GATT_PERM_READ_ENCRYPT | BT_GATT_PERM_WRITE),
    // New battery voltage read-only characteristic
     // New read callback
);

// Update main message queue
static void update_main_msgq(struct main_msg_data *data) {
    k_msgq_put(&main_msgq, data, K_NO_WAIT);
}

// Convert 4 byte array to 32-bit unsigned integer
static uint32_t u8_array_to_u32(uint8_t *src) {
    uint32_t res = 0;
    res |= src[0];
    res <<= 8;
    res |= src[1];
    res <<= 8;
    res |= src[2];
    res <<= 8;
    res |= src[3];
    return res;
}

// Function to handle disconnection
void disconnect_callback(struct k_timer *timer) {
    if (default_conn && !security_set) {
        LOG_INF("Disconnecting due to security setup failure");
        bt_conn_disconnect(default_conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
        default_conn = NULL;
    }
}

// Callback to add addresses to the Accept List
static void setup_accept_list_cb(const struct bt_bond_info *info, void *user_data) {
    int *bond_cnt = user_data;

    if ((*bond_cnt) < 0) {
        return;
    }

   int err = bt_le_filter_accept_list_add(&info->addr);
    if (err) {
        if (err == -ENOMEM) {
            bt_le_filter_accept_list_clear();
            // Accept list is full, clear the first devic
            LOG_INF("Accept list full, cleared all devices ");
        } else {
            LOG_INF("Cannot add peer to filter accept list (err: %d)\n", err);
            (*bond_cnt) = -EIO;
        }
    } else {
        (*bond_cnt)++;
    }
}

// Function to loop through the bond list
static int setup_accept_list(uint8_t local_id) {
    int err = bt_le_filter_accept_list_clear();

    if (err) {
        LOG_INF("Cannot clear accept list (err: %d)\n", err);
        return err;
    }

    int bond_cnt = 0;

    bt_foreach_bond(local_id, setup_accept_list_cb, &bond_cnt);

    return bond_cnt;
}


// Function to advertise with the Accept List
void advertise_with_acceptlist(struct k_work *work) {
    int err = 0;
    int allowed_cnt = setup_accept_list(BT_ID_DEFAULT);
    if (allowed_cnt < 0) {
        LOG_INF("Acceptlist setup failed (err:%d)\n", allowed_cnt);
    } else {
        if (allowed_cnt == 0) {
            LOG_INF("Advertising with no Accept list \n");
            err = bt_le_adv_start(BT_LE_ADV_CONN_NO_ACCEPT_LIST, adv_data, ARRAY_SIZE(adv_data), sd, ARRAY_SIZE(sd));
        } else {
            LOG_INF("Acceptlist setup number  = %d \n", allowed_cnt);
            err = bt_le_adv_start(BT_LE_ADV_CONN_ACCEPT_LIST, adv_data, ARRAY_SIZE(adv_data), sd, ARRAY_SIZE(sd));
        }
        if (err) {
            LOG_INF("Advertising failed to start (err %d)\n", err);
            return;
        }
        LOG_INF("Advertising successfully started\n");
    }
}

K_WORK_DEFINE(advertise_acceptlist_work, advertise_with_acceptlist);

// Start new BLE advertisement
void ble_new_adv() {
    
    int err = bt_le_adv_start(BT_LE_ADV_CONN_NO_ACCEPT_LIST, adv_data, ARRAY_SIZE(adv_data), sd, ARRAY_SIZE(sd));
}

// BLE connection callback - connected
static void ble_connected(struct bt_conn *conn, uint8_t err) {

    if (err) {
        LOG_ERR("BLE connection fail (err %d)", err);
        default_conn = NULL;
        return;
    }

    default_conn = conn;
    pairing_on = false;
    LOG_INF("BLE connected");

    if(battery_low == 0){
    if(is_pwm_modes_enabled){
   // start_green_led_thread(10000);
    }
    else{
    //start_green_led_thread(1000000);
    }
    k_thread_suspend(my_tid3);
    
    }

    struct main_msg_data conn_msg = {.src = MAIN_MSG_DATA_SRC_BLE, .data.ble.id = BLE_MSG_ID_CONNECTED};

    update_main_msgq(&conn_msg);
}

// BLE connection callback - disconnected
static void ble_disconnected(struct bt_conn *conn, uint8_t reason) {

    default_conn = NULL;
    disconnect_reason = reason;
    pairing_on = true;

    LOG_WRN("Disconnected (reason 0x%02x)\n", reason);
    LOG_WRN("BLE disconnected, reason %d", reason);

    // Start advertising with Accept List
    k_work_submit(&advertise_acceptlist_work);


    struct main_msg_data conn_msg = {.src = MAIN_MSG_DATA_SRC_BLE, .data.ble.id = BLE_MSG_ID_DISCONNECTED};
    update_main_msgq(&conn_msg);
    
}

// BLE connection PHY updated callback
void on_le_phy_updated(struct bt_conn *conn, struct bt_conn_le_phy_info *param) {
    // PHY Updated
    if (param->tx_phy == BT_CONN_LE_TX_POWER_PHY_1M) {
        LOG_INF("PHY updated. New PHY: 1M");
    } else if (param->tx_phy == BT_CONN_LE_TX_POWER_PHY_2M) {
        LOG_INF("PHY updated. New PHY: 2M");
    } else if (param->tx_phy == BT_CONN_LE_TX_POWER_PHY_CODED_S2) {
        LOG_INF("PHY updated. New PHY: Long Range");
    } else if (param->tx_phy == BT_CONN_LE_TX_POWER_PHY_CODED_S8) {
        LOG_INF("PHY updated. New PHY: Long Range with S8");
    }
}

// BLE security changed callback
static void on_security_changed(struct bt_conn *conn, bt_security_t level, enum bt_security_err err) {
    char addr[BT_ADDR_LE_STR_LEN];

    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

    if (!err) {
        LOG_INF("Security changed: %s level %u\n", addr, level);
    } else {
        LOG_INF("Security failed: %s level %u err %d\n", addr, level, err);
    }
}

// BLE connection callbacks
struct bt_conn_cb connection_callbacks = {
    .connected = ble_connected,
    .disconnected = ble_disconnected,
    .security_changed = on_security_changed,
    // .le_phy_updated = on_le_phy_updated,
};

// Read callback for EMS service
static ssize_t ems_read_callback(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf, uint16_t len, uint16_t offset) {
    LOG_DBG("EMS read");
    if (bt_uuid_cmp(attr->uuid, &ems_session_uuid.uuid) == 0) {
        return bt_gatt_attr_read(conn, attr, buf, len, offset, &ems_session_data.status.session.current_session_len,
                                 sizeof(ems_session_data.status.session.current_session_len));
    } else if (bt_uuid_cmp(attr->uuid, &ems_control_uuid.uuid) == 0) {
        return bt_gatt_attr_read(conn, attr, buf, len, offset, &ems_session_data.status.control,
                                 sizeof(ems_session_data.status.control));
    } else if (bt_uuid_cmp(attr->uuid, &ems_chan1_uuid.uuid) == 0) {
        return bt_gatt_attr_read(conn, attr, buf, len, offset, &ems_session_data.channels.chan0,
                                 sizeof(ems_session_data.channels.chan0));
    } else if (bt_uuid_cmp(attr->uuid, &ems_chan2_uuid.uuid) == 0) {
        return bt_gatt_attr_read(conn, attr, buf, len, offset, &ems_session_data.channels.chan1,
                                 sizeof(ems_session_data.channels.chan1));
    } else if (bt_uuid_cmp(attr->uuid, &ems_chan3_uuid.uuid) == 0) {
        return bt_gatt_attr_read(conn, attr, buf, len, offset, &ems_session_data.channels.chan2,
                                 sizeof(ems_session_data.channels.chan2));
    } else if (bt_uuid_cmp(attr->uuid, &ems_chan4_uuid.uuid) == 0) {
        return bt_gatt_attr_read(conn, attr, buf, len, offset, &ems_session_data.channels.chan3,
                                 sizeof(ems_session_data.channels.chan3));                          
    } else {
        return BT_GATT_ERR(BT_ATT_ERR_ATTRIBUTE_NOT_FOUND);
    }
}

static ssize_t charger_read_callback(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf, uint16_t len, uint16_t offset) {
    return bt_gatt_attr_read(conn, attr, buf, len, offset, &charger_state, sizeof(charger_state));
}

// Read callback for battery service
static ssize_t batt_read_callback(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf, uint16_t len, uint16_t offset) {
    return bt_gatt_attr_read(conn, attr, buf, len, offset, &batt_val_mv, sizeof(batt_val_mv));
}

// Write callback for EMS service
static ssize_t ems_write_cb(struct bt_conn *conn, const struct bt_gatt_attr *attr, const void *buf, uint16_t len, uint16_t offset,
                            uint8_t flags) {
    LOG_INF("EMS Data update");

    uint8_t *data = (uint8_t *)buf;
    struct main_msg_data m_data = {
        .src = MAIN_MSG_DATA_SRC_BLE
    };

    if (bt_uuid_cmp(attr->uuid, &ems_control_uuid.uuid) == 0) {
        m_data.data.ble.id = BLE_MSG_ID_EMS_CONTROL;
        m_data.data.ble.data.ems.status.control = *data;
        update_main_msgq(&m_data);
        LOG_INF("EMS Control updated to: %u", *data);
        return len;
    } else if (bt_uuid_cmp(attr->uuid, &ems_session_uuid.uuid) == 0) {
        uint32_t new_value = u8_array_to_u32(data);
        m_data.data.ble.id = BLE_MSG_ID_EMS_SESSION;
        m_data.data.ble.data.ems.status.session.target_session_len = new_value;
        update_main_msgq(&m_data);
        LOG_INF("EMS Session updated to: %u", new_value);
        return len;
    } else if (bt_uuid_cmp(attr->uuid, &ems_chan1_uuid.uuid) == 0) {
        uint32_t on_time = u8_array_to_u32(data);
        uint32_t off_time = u8_array_to_u32(&data[4]);
        m_data.data.ble.id = BLE_MSG_ID_EMS_CHAN1_CONTROL;
        m_data.data.ble.data.ems.channels.chan0.on_time = on_time;
        m_data.data.ble.data.ems.channels.chan0.off_time = off_time;
        update_main_msgq(&m_data);
        LOG_INF("EMS Channel 1 Control updated to: On Time = %u, Off Time = %u", on_time, off_time);
        return len;
    } else if (bt_uuid_cmp(attr->uuid, &ems_chan2_uuid.uuid) == 0) {
        uint32_t on_time = u8_array_to_u32(&data[0]);
        uint32_t interval = u8_array_to_u32(&data[4]);
        m_data.data.ble.id = BLE_MSG_ID_EMS_CHAN2_CONTROL;
        m_data.data.ble.data.ems.channels.chan1.on_time = on_time;
        m_data.data.ble.data.ems.channels.chan1.interval = interval;
        update_main_msgq(&m_data);
        LOG_INF("EMS Channel 2 Control updated to: On Time = %u, Interval = %u", on_time, interval);
        return len;
    } else if (bt_uuid_cmp(attr->uuid, &ems_chan3_uuid.uuid) == 0) {
        m_data.data.ble.id = BLE_MSG_ID_EMS_CHAN3_CONTROL;
        m_data.data.ble.data.ems.channels.chan2.duty_cycle = data[0];
        update_main_msgq(&m_data);
        LOG_INF("EMS Channel 3 Control updated to: Duty Cycle = %u", data[0]);
        return len;
    } else if (bt_uuid_cmp(attr->uuid, &ems_chan4_uuid.uuid) == 0) {
        m_data.data.ble.id = BLE_MSG_ID_EMS_CHAN4_CONTROL;
        m_data.data.ble.data.ems.channels.chan3.duty_cycle = data[0];
        update_main_msgq(&m_data);
        LOG_INF("EMS Channel 4 Control updated to: Duty Cycle = %u", data[0]);
        return len;
    } else if (bt_uuid_cmp(attr->uuid, &pwm_period_uuid.uuid) == 0) {
        m_data.data.ble.id = BLE_MSG_ID_PWM_PERIOD;
        m_data.data.ble.data.ems.status.period = *data;
        update_main_msgq(&m_data);
        LOG_INF("PWM Period updated to: %u", *data);
        return len;
    } else if (bt_uuid_cmp(attr->uuid, &pwm_ontime_uuid.uuid) == 0) {
        m_data.data.ble.id = BLE_MSG_ID_PWM_ONTIME;
        m_data.data.ble.data.ems.status.ontime = *data;
        update_main_msgq(&m_data);
        LOG_INF("PWM On Time updated to: %u", *data);
        return len;
    } else if (bt_uuid_cmp(attr->uuid, &pwm_interval_uuid.uuid) == 0) {
        m_data.data.ble.id = BLE_MSG_ID_PWM_INTERVAL;
        m_data.data.ble.data.ems.status.interval = *data;
        update_main_msgq(&m_data);
        LOG_INF("PWM Interval updated to: %u", *data);
        return len;
    } else if (bt_uuid_cmp(attr->uuid, &pwm_duration_uuid.uuid) == 0) {
        m_data.data.ble.id = BLE_MSG_ID_PWM_DURATION;
        m_data.data.ble.data.ems.status.duration = *data;
        update_main_msgq(&m_data);
        LOG_INF("PWM Duration updated to: %u", *data);
        return len;
    }
    else if (bt_uuid_cmp(attr->uuid, &chan0_playback_uuid.uuid) == 0) {
        m_data.data.ble.id = BLE_MSG_ID_CHAN0_PLAYBACK_COUNT;
        m_data.data.ble.data.ems.status.chan0playback = *data;
        update_main_msgq(&m_data);
        LOG_INF("CHAN0 playback updated to: %u", *data);
        return len;
    }
    else if (bt_uuid_cmp(attr->uuid, &chan0_induct_uuid.uuid) == 0) {
        m_data.data.ble.id = BLE_MSG_ID_CHAN0_INUCT_DURATION;
        m_data.data.ble.data.ems.status.chan0inductor = *data;
        update_main_msgq(&m_data);
        LOG_INF("CHAN0 INUCTOR updated to: %u", *data);
        return len;
    }
    else if (bt_uuid_cmp(attr->uuid, &chan0_on_duration_uuid.uuid) == 0) {
        m_data.data.ble.id = BLE_MSG_ID_CHAN0_ON_DURATION;
        m_data.data.ble.data.ems.status.chan0ontime = *data;
        update_main_msgq(&m_data);
        LOG_INF("CHAN0 Duration updated to: %u", *data);
        return len;
    }
    else if (bt_uuid_cmp(attr->uuid, &chan0_duration_uuid.uuid) == 0) {
        m_data.data.ble.id = BLE_MSG_ID_CHAN0_DURATION;
        m_data.data.ble.data.ems.status.chan0time = *data;
        update_main_msgq(&m_data);
        LOG_INF("CHAN0 ON_Duration updated to: %u", *data);
        return len;
    }

        else if (bt_uuid_cmp(attr->uuid, &chan23_duration_uuid.uuid) == 0) {
        m_data.data.ble.id = BLE_MSG_ID_CHAN23_PWM_PERIOD_NS;
        m_data.data.ble.data.ems.status.pwm23time = *data;
        update_main_msgq(&m_data);
        LOG_INF("PWM_23 Duration updated to: %u", *data);
        return len;
    }
    else if (bt_uuid_cmp(attr->uuid, &chan23_on_duration_uuid.uuid) == 0) {
        m_data.data.ble.id = BLE_MSG_ID_CHAN23_PWM_ON_TIME_NS;
        m_data.data.ble.data.ems.status.pwm23ontime = *data;
        update_main_msgq(&m_data);
        LOG_INF("PWM_23 ON_Duration updated to: %u", *data);
        return len;
    }

    else if (bt_uuid_cmp(attr->uuid, &charger_on_off_uuid.uuid) == 0) {
        m_data.data.ble.id = BLE_MSG_ID_CHARGER_ON_OFF;
        m_data.data.ble.data.ems.status.chargeronoff = *data;
        update_main_msgq(&m_data);
        LOG_INF("CHARGER ON OFF updated to: %u", *data);
        

        return len;
    }

    LOG_ERR("Unsupported EMS attribute");
    return BT_GATT_ERR(BT_ATT_ERR_NOT_SUPPORTED);
}

// CCC value changed callback for battery service
static void batt_ccc_value_changed(const struct bt_gatt_attr *attr, uint16_t value) {

    if (value & BT_GATT_CCC_NOTIFY) {
        //LOG_INF("BATT notifications enabled");
    } else {
        //LOG_INF("BATT notifications disabled");
    }
}

static void auth_passkey_display(struct bt_conn *conn, unsigned int passkey)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
}

static void auth_passkey_confirm(struct bt_conn *conn, unsigned int passkey)
{
    bt_conn_auth_passkey_confirm(conn); // Confirm passkey for numeric comparison
}

static void auth_cancel(struct bt_conn *conn)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
}

static void pairing_complete(struct bt_conn *conn, bool bonded)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
}

static void pairing_failed(struct bt_conn *conn, enum bt_security_err reason)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
}

static struct bt_conn_auth_cb conn_auth_callbacks = {
	.passkey_display = auth_passkey_display,
    .passkey_confirm = auth_passkey_confirm,
	.cancel = auth_cancel,
};

static struct bt_conn_auth_info_cb conn_auth_info_callbacks = {
	.pairing_complete = pairing_complete,
	.pairing_failed = pairing_failed
};

static bool bms_authorize(struct bt_conn *conn,
			  struct bt_bms_authorize_params *params)
{
	if ((params->code_len == sizeof(bms_auth_code)) &&
	    (memcmp(bms_auth_code, params->code, sizeof(bms_auth_code)) == 0)) {
		printk("Authorization of BMS operation is successful\n");
		return true;
	}

	printk("Authorization of BMS operation has failed\n");
	return false;
}

static struct bt_bms_cb bms_callbacks = {
	.authorize = bms_authorize,
};

static int bms_init(void)
{
	struct bt_bms_init_params init_params = {0};

	/* Enable all possible operation codes */
	init_params.features.delete_requesting.supported = true;
	init_params.features.delete_rest.supported = true;
	init_params.features.delete_all.supported = true;

	/* Require authorization code for operations that
	 * also delete bonding information for other devices
	 * than the requesting client.
	 */
	init_params.features.delete_rest.authorize = true;
	init_params.features.delete_all.authorize = true;

	init_params.cbs = &bms_callbacks;

	return bt_bms_init(&init_params);
}

// BLE thread main function
int ble_thread_main() {
    
    int err;

    struct ble_msg msgq_data;

    bt_conn_cb_register(&connection_callbacks);

    err = bt_conn_auth_cb_register(&conn_auth_callbacks);

	if (err) {
		printk("Failed to register authorization callbacks.\n");
		return 0;
	}

	err = bt_conn_auth_info_cb_register(&conn_auth_info_callbacks);

	if (err) {
		printk("Failed to register authorization info callbacks.\n");
		return 0;
	}

    err = bt_enable(NULL);

    if (err) {
        LOG_INF("Bluetooth init failed (err %d)\n", err);
        return -1;
    }

    LOG_INF("Bluetooth initialized\n");

    // Add setting load function
    settings_load();

    err = bms_init();

	if (err) {
		printk("Failed to init BMS (err:%d)\n", err);
		return 0;
	}

    // Start advertising with the Accept List
    k_work_submit(&advertise_acceptlist_work);

    while (1) {
        k_msgq_get(&ble_msgq, &msgq_data, K_FOREVER);

        switch (msgq_data.id) {
            case BLE_MSG_ID_EMS_CONTROL: {
                memcpy(&ems_session_data, &msgq_data.data.ems, sizeof(ems_session_data));
                break;
            }
            case BLE_MSG_ID_BATT_UPDATE: {
                if (bt_gatt_is_subscribed(default_conn, &battery_service.attrs[1], BT_GATT_CCC_NOTIFY)) {
                    bt_gatt_notify(default_conn, &battery_service.attrs[1], &batt_val_mv, sizeof(batt_val_mv));
                } else {
                   // LOG_WRN("BATT notification not enabled");
                }
                break;
            }
            case BLE_MSG_ID_REINIT: {
                (void)bt_le_adv_stop();
                k_sleep(K_MSEC(100));
                if (bt_disable() == 0) {
                    k_sleep(K_MSEC(100));
                    if (bt_enable(NULL) == 0) {
            
                        LOG_INF("BLE reinit");
                    }
                }
                break;
            }
            case BLE_MSG_ID_DEINIT: {
                (void)bt_le_adv_stop();
                k_sleep(K_MSEC(100));
                (void)bt_disable();
                k_sleep(K_FOREVER);
                break;
            }
            default: {
                break;
            }
        }
    }

    return 0;
}
K_THREAD_DEFINE(ble_thread, 4096, ble_thread_main, NULL, NULL, NULL, 1, K_ESSENTIAL, 0);