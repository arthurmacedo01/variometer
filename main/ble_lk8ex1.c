#include "ble_lk8ex1.h"
#include "esp_log.h"
#include "esp_bt.h"
#include "nvs_flash.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_main.h"
#include "esp_gatt_common_api.h"
#include <stdio.h>
#include <string.h>

#define TAG "BLE_LK8EX1"
#define MAX_BLE_PAYLOAD_SIZE 20

// // BLE service and characteristic UUIDs for Environmental Sensing Service (ESS)
#define SERVICE_UUID_LEN ESP_UUID_LEN_128

static const uint8_t SERVICE_UUID[ESP_UUID_LEN_128] = {
    0xfb, 0x34, 0x9b, 0x5f, // fb349b5f
    0x80, 0x00, 0x00, 0x80, // 80000080
    0x00, 0x10, 0x00, 0x00, // 00100000
    0x1a, 0x18, 0x00, 0x00  // 181a0000
};

#define CHARACTERISTIC_UUID_LEN ESP_UUID_LEN_128
static const uint8_t CHARACTERISTIC_UUID[ESP_UUID_LEN_128] = {0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0x6d, 0x2a, 0x00, 0x00};

// #define SERVICE_UUID_LEN ESP_UUID_LEN_16
// static const uint8_t SERVICE_UUID[ESP_UUID_LEN_16] = {0xfb, 0x34};

// #define CHARACTERISTIC_UUID_LEN ESP_UUID_LEN_16
// static const uint8_t CHARACTERISTIC_UUID[ESP_UUID_LEN_16] = {0xfb, 0x34};

// Global variables for GATT interface and handles
static esp_gatt_if_t gatts_if_global;
static uint16_t service_handle;
static uint16_t char_handle;

static bool is_connected = false;

// Função para calcular o checksum NMEA
static uint8_t calculate_checksum(const char *sentence)
{
    uint8_t checksum = 0;
    // O cálculo é feito em todos os caracteres da sentença
    while (*sentence)
    {
        checksum ^= *sentence++;
    }
    return checksum;
}

// Advertisement data
static esp_ble_adv_data_t adv_data = {
    .set_scan_rsp = false,
    .include_name = true,
    .include_txpower = false,
    .min_interval = 0x20,
    .max_interval = 0x40,
    .appearance = 0x00,
    .manufacturer_len = 0,
    .p_manufacturer_data = NULL,
    .service_data_len = 0,
    .p_service_data = NULL,
    .service_uuid_len = SERVICE_UUID_LEN,
    .p_service_uuid = NULL,
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};

// Advertisement parameters
static esp_ble_adv_params_t adv_params = {
    .adv_int_min = 0x20,
    .adv_int_max = 0x40,
    .adv_type = ADV_TYPE_IND,
    .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
    .channel_map = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

// GATT service ID
static esp_gatt_srvc_id_t service_id = {
    .is_primary = true,
    .id.inst_id = 0x00,
    .id.uuid.len = SERVICE_UUID_LEN,
};

// GATT event handler
static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    memcpy(service_id.id.uuid.uuid.uuid128, SERVICE_UUID, SERVICE_UUID_LEN);
    switch (event)
    {
    case ESP_GATTS_REG_EVT:
        gatts_if_global = gatts_if; // Assign the GATT interface
        ESP_LOGI(TAG, "GATT_REG_EVT, status %d, app_id %d", param->reg.status, param->reg.app_id);
        esp_ble_gap_set_device_name("ESP32_Vario");
        esp_ble_gap_config_adv_data(&adv_data);
        esp_ble_gatts_create_service(gatts_if, &service_id, 4);
        break;

    case ESP_GATTS_CREATE_EVT:
        ESP_LOGI(TAG, "CREATE_SERVICE_EVT, status %d, service_handle %d", param->create.status, param->create.service_handle);
        service_handle = param->create.service_handle;

        esp_ble_gatts_start_service(service_handle);

        esp_bt_uuid_t char_uuid = {
            .len = CHARACTERISTIC_UUID_LEN,
        };
        memcpy(char_uuid.uuid.uuid128, CHARACTERISTIC_UUID, CHARACTERISTIC_UUID_LEN);

        esp_gatt_perm_t perm = ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE;
        esp_gatt_char_prop_t property = ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_NOTIFY;

        esp_err_t ret = esp_ble_gatts_add_char(service_handle, &char_uuid, perm, property, NULL, NULL);
        if (ret != ESP_OK)
        {
            ESP_LOGE(TAG, "Failed to add characteristic, error code = %x", ret);
        }
        break;

    case ESP_GATTS_ADD_CHAR_EVT:
        ESP_LOGI(TAG, "ADD_CHAR_EVT, status %d, attr_handle %d, service_handle %d",
                 param->add_char.status, param->add_char.attr_handle, param->add_char.service_handle);
        char_handle = param->add_char.attr_handle;
        break;

    case ESP_GATTS_CONNECT_EVT:
        is_connected = true;
        ESP_LOGI(TAG, "BLE device connected");
        break;

    case ESP_GATTS_DISCONNECT_EVT:
        is_connected = false;
        ESP_LOGI(TAG, "BLE device disconnected");
        esp_ble_gap_start_advertising(&adv_params);
        break;

    default:
        break;
    }
}

// Initialize BLE
esp_err_t ble_init(void)
{
    esp_err_t ret;

    // Initialize NVS (required for Bluetooth)
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Initialize the Bluetooth stack
    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    ESP_ERROR_CHECK(ret);

    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    ESP_ERROR_CHECK(ret);

    // Set the BLE transmit power
    ret = esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_DEFAULT, ESP_PWR_LVL_N14);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to set BLE transmit power, error code = %x", ret);
    }

    ret = esp_bluedroid_init();
    ESP_ERROR_CHECK(ret);

    ret = esp_bluedroid_enable();
    ESP_ERROR_CHECK(ret);

    // Register the GATT event handler
    ret = esp_ble_gatts_register_callback(gatts_event_handler);
    if (ret)
    {
        ESP_LOGE(TAG, "GATT handler registration failed, error code = %x", ret);
        return ret;
    }

    // Register your application with the GATT server
    ret = esp_ble_gatts_app_register(0);
    if (ret)
    {
        ESP_LOGE(TAG, "GATT app registration failed, error code = %x", ret);
        return ret;
    }

    // Set the advertising data and start advertising
    ret = esp_ble_gap_config_adv_data(&adv_data);
    if (ret)
    {
        ESP_LOGE(TAG, "Failed to configure advertising data, error code = %x", ret);
        return ret;
    }

    ret = esp_ble_gap_start_advertising(&adv_params);
    if (ret)
    {
        ESP_LOGE(TAG, "Failed to start advertising, error code = %x", ret);
        return ret;
    }

    return ESP_OK;
}

// Send LK8EX1 data over BLE (focusing on pressure/altitude and vario)
void ble_send_lk8ex1(float pressure, float altitude, float vario, float temperature, float battery)
{
    if (!is_connected)
    {
        ESP_LOGW(TAG, "No BLE connection, data not sent.");
        return;
    }

    // char pre_sentence[32];
    // char sentence[64];

    // // // Format the LK8EX1 sentence
    // // snprintf(pre_sentence, sizeof(pre_sentence), "LK8EX1,%.0f,%.0f,%.0f,%.0f,%.0f,", pressure, altitude, vario * 100, temperature, battery);

    // // // Calculate checksum
    // // uint8_t checksum = calculate_checksum(pre_sentence);
    // // snprintf(sentence, sizeof(sentence), "$%s*%02X\r\n", pre_sentence, checksum);

    // // Format the LK8EX1 sentence
    // snprintf(pre_sentence, sizeof(pre_sentence), "LK8EX1,%.0f,%.0f,%.0f,%.0f,%.0f,", pressure, altitude, vario * 100, temperature, battery);

    // // Calculate checksum
    // uint8_t checksum = calculate_checksum(pre_sentence);
    // snprintf(sentence, sizeof(sentence), "$%s*%02X\r\n", pre_sentence, checksum);

    // size_t sentence_length = strlen(sentence);

    unsigned char sentence[4];

    unsigned int int_pressure = (unsigned int)pressure; // This discards the decimal part
    // Store the integer part of the pressure value into the sentence array, byte by byte
    sentence[0] = int_pressure & 0xFF;         // 1st byte (Least Significant Byte)
    sentence[1] = (int_pressure >> 8) & 0xFF;  // 2nd byte
    sentence[2] = (int_pressure >> 16) & 0xFF; // 3rd byte
    sentence[3] = (int_pressure >> 24) & 0xFF; // 4th byte (Most Significant Byte)    

    size_t sentence_length = sizeof(sentence);


    size_t offset = 0;

    while (offset < sentence_length)
    {
        size_t chunk_size = MAX_BLE_PAYLOAD_SIZE;
        if (offset + chunk_size > sentence_length)
        {
            chunk_size = sentence_length - offset;
        }

        esp_err_t ret = esp_ble_gatts_send_indicate(gatts_if_global, 0, char_handle, chunk_size, (uint8_t *)(sentence + offset), false);
        if (ret != ESP_OK)
        {
            ESP_LOGE(TAG, "Failed to send LK8EX1 sentence chunk over BLE, error code = %x", ret);
            break; // Stop sending further chunks if an error occurs
        }

        offset += chunk_size;
    }

    if (offset >= sentence_length)
    {
        ESP_LOGI(TAG, "LK8EX1 sentence sent successfully in chunks: %s", sentence);
    }
}
