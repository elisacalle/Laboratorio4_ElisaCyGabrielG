#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_rom_sys.h"  

// ===== BLE NimBLE =====
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

static const char *TAG = "PANEL_HMI";

// ===== CONFIGURACIÓN DE PINES =====
#define I2C_MASTER_SDA      21
#define I2C_MASTER_SCL      22
#define I2C_MASTER_NUM      0
#define I2C_MASTER_FREQ_HZ  100000

// ===== LCD I2C (PCF8574) =====
#define LCD_ADDR            0x27
#define LCD_BACKLIGHT       0x08
#define LCD_ENABLE          0x04
#define LCD_RS              0x01

#define LCD_CMD             0
#define LCD_DATA            1

#define LCD_CLEAR           0x01
#define LCD_HOME            0x02
#define LCD_ENTRY_MODE      0x06
#define LCD_DISPLAY_ON      0x0C
#define LCD_FUNCTION_SET    0x28
#define LCD_LINE1           0x80
#define LCD_LINE2           0xC0

// ===== RTC DS1307 =====
#define DS1307_ADDR         0x68

// ===== LEDs =====
#define LED_RED_PIN         25
#define LED_GREEN_PIN       26
#define LED_BLUE_PIN        27

// ===== BUZZER =====
#define BUZZER_PIN          33

// ===== RFID RC522 (SPI) =====
#define RC522_PIN_MOSI      23
#define RC522_PIN_MISO      19
#define RC522_PIN_SCK       18
#define RC522_PIN_CS         5
#define RC522_PIN_RST        4

// Registros del RC522
#define RC522_CommandReg     0x01
#define RC522_ComIEnReg      0x02
#define RC522_ComIrqReg      0x04
#define RC522_ErrorReg       0x06
#define RC522_FIFODataReg    0x09
#define RC522_FIFOLevelReg   0x0A
#define RC522_ControlReg     0x0C
#define RC522_BitFramingReg  0x0D
#define RC522_ModeReg        0x11
#define RC522_TxControlReg   0x14
#define RC522_TxASKReg       0x15
#define RC522_TModeReg       0x2A
#define RC522_TPrescalerReg  0x2B
#define RC522_TReloadRegH    0x2C
#define RC522_TReloadRegL    0x2D
#define RC522_VersionReg     0x37

#define RC522_CMD_IDLE       0x00
#define RC522_CMD_TRANSCEIVE 0x0C
#define RC522_CMD_SOFTRESET  0x0F

#define PICC_CMD_REQA        0x26
#define PICC_CMD_ANTICOLL    0x93

// ===== UIDs AUTORIZADOS =====
static const uint8_t AUTHORIZED_UIDS[][4] = {
    { 0xF4, 0x33, 0x15, 0x07 },    // Tarjeta del supervisor
};
#define NUM_AUTHORIZED (sizeof(AUTHORIZED_UIDS) / sizeof(AUTHORIZED_UIDS[0]))

// ===== ESTADOS DEL SISTEMA =====
typedef enum {
    STATE_LOCKED,       // Panel bloqueado
    STATE_ACTIVE,       // Sesión activa (BLE habilitado)
} system_state_t;

static system_state_t system_state = STATE_LOCKED;

// ===== VARIABLES GLOBALES =====
static spi_device_handle_t rc522_spi = NULL;

// BLE: mensaje recibido por NUS
static char ble_message[17] = {0};
static volatile bool ble_message_new = false;

// Mensaje actualmente mostrado en línea 1 (estado activo)
static char current_display_msg[17] = "Sin mensajes";

// UUIDs del servicio NUS
static const ble_uuid128_t nus_service_uuid =
    BLE_UUID128_INIT(0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0,
                     0x93, 0xF3, 0xA3, 0xB5, 0x01, 0x00, 0x40, 0x6E);

static const ble_uuid128_t nus_rx_char_uuid =
    BLE_UUID128_INIT(0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0,
                     0x93, 0xF3, 0xA3, 0xB5, 0x02, 0x00, 0x40, 0x6E);

// ========================================================
//                  FUNCIONES LCD I2C
// ========================================================

static esp_err_t lcd_write_byte(uint8_t data)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (LCD_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, data, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    return ret;
}

static void lcd_pulse_enable(uint8_t data)
{
    lcd_write_byte(data | LCD_ENABLE);
    esp_rom_delay_us(600);                  // ← delay exacto 600us
    lcd_write_byte(data & ~LCD_ENABLE);
    esp_rom_delay_us(600);                  // ← delay exacto 600us
}

static void lcd_send(uint8_t value, uint8_t mode)
{
    uint8_t high_nibble = (value & 0xF0) | LCD_BACKLIGHT | (mode ? LCD_RS : 0);
    uint8_t low_nibble  = ((value << 4) & 0xF0) | LCD_BACKLIGHT | (mode ? LCD_RS : 0);
    lcd_pulse_enable(high_nibble);
    lcd_pulse_enable(low_nibble);
}

static void lcd_command(uint8_t cmd)
{
    lcd_send(cmd, LCD_CMD);
}

static void lcd_char(char c)
{
    lcd_send((uint8_t)c, LCD_DATA);
}

static void lcd_print(const char *str)
{
    while (*str) {
        lcd_char(*str++);
    }
}

static void lcd_set_cursor(uint8_t row, uint8_t col)
{
    uint8_t addr = (row == 0) ? LCD_LINE1 : LCD_LINE2;
    lcd_command(addr + col);
}

static void lcd_clear(void)
{
    lcd_command(LCD_CLEAR);
    esp_rom_delay_us(5000);     // Clear necesita ~2ms, damos 5ms
}

static void lcd_print_line(uint8_t row, const char *str)
{
    lcd_set_cursor(row, 0);
    char buf[17];
    snprintf(buf, sizeof(buf), "%-16s", str);
    lcd_print(buf);
}

static void lcd_init(void)
{
    vTaskDelay(pdMS_TO_TICKS(100));         // Power-on wait (este sí puede ser largo)

    lcd_write_byte(0x30 | LCD_BACKLIGHT);
    lcd_pulse_enable(0x30 | LCD_BACKLIGHT);
    esp_rom_delay_us(5000);                 // 5ms

    lcd_pulse_enable(0x30 | LCD_BACKLIGHT);
    esp_rom_delay_us(5000);                 // 5ms

    lcd_pulse_enable(0x30 | LCD_BACKLIGHT);
    esp_rom_delay_us(2000);                 // 2ms

    lcd_pulse_enable(0x20 | LCD_BACKLIGHT);
    esp_rom_delay_us(2000);                 // 2ms

    lcd_command(LCD_FUNCTION_SET);
    esp_rom_delay_us(1000);
    lcd_command(LCD_DISPLAY_ON);
    esp_rom_delay_us(1000);
    lcd_command(LCD_ENTRY_MODE);
    esp_rom_delay_us(1000);
    lcd_clear();

    ESP_LOGI(TAG, "LCD inicializado correctamente");
}

// ========================================================
//                 FUNCIONES LEDs + BUZZER
// ========================================================

static void leds_buzzer_init(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << LED_RED_PIN) | (1ULL << LED_GREEN_PIN) |
                        (1ULL << LED_BLUE_PIN) | (1ULL << BUZZER_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);

    gpio_set_level(LED_RED_PIN, 0);
    gpio_set_level(LED_GREEN_PIN, 0);
    gpio_set_level(LED_BLUE_PIN, 0);
    gpio_set_level(BUZZER_PIN, 0);

    ESP_LOGI(TAG, "LEDs y Buzzer inicializados");
}

static void buzzer_short(void)
{
    gpio_set_level(BUZZER_PIN, 1);
    vTaskDelay(pdMS_TO_TICKS(500));
    gpio_set_level(BUZZER_PIN, 0);
}

static void buzzer_long(void)
{
    gpio_set_level(BUZZER_PIN, 1);
    vTaskDelay(pdMS_TO_TICKS(2000));
    gpio_set_level(BUZZER_PIN, 0);
}

static void led_red_blink_3(void)
{
    for (int i = 0; i < 4; i++) {
        gpio_set_level(LED_RED_PIN, 1);
        vTaskDelay(pdMS_TO_TICKS(100));
        gpio_set_level(LED_RED_PIN, 0);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

// ========================================================
//                  FUNCIONES RTC DS1307
// ========================================================

static uint8_t dec_to_bcd(uint8_t val)
{
    return ((val / 10) << 4) | (val % 10);
}

static uint8_t bcd_to_dec(uint8_t val)
{
    return ((val >> 4) * 10) + (val & 0x0F);
}

static esp_err_t ds1307_set_time(uint8_t hour, uint8_t min, uint8_t sec)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (DS1307_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, 0x00, true);
    i2c_master_write_byte(cmd, dec_to_bcd(sec), true);
    i2c_master_write_byte(cmd, dec_to_bcd(min), true);
    i2c_master_write_byte(cmd, dec_to_bcd(hour), true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);

    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "RTC configurado: %02d:%02d:%02d", hour, min, sec);
    } else {
        ESP_LOGE(TAG, "Error configurando RTC: %s", esp_err_to_name(ret));
    }
    return ret;
}

static esp_err_t ds1307_get_time(uint8_t *hour, uint8_t *min, uint8_t *sec)
{
    uint8_t data[3];

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (DS1307_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, 0x00, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);

    if (ret != ESP_OK) return ret;

    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (DS1307_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read_byte(cmd, &data[0], I2C_MASTER_ACK);
    i2c_master_read_byte(cmd, &data[1], I2C_MASTER_ACK);
    i2c_master_read_byte(cmd, &data[2], I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);

    if (ret == ESP_OK) {
        *sec  = bcd_to_dec(data[0] & 0x7F);
        *min  = bcd_to_dec(data[1] & 0x7F);
        *hour = bcd_to_dec(data[2] & 0x3F);
    }
    return ret;
}

// ========================================================
//                 FUNCIONES RFID RC522 (SPI)
// ========================================================

static void rc522_write_reg(uint8_t reg, uint8_t value)
{
    spi_transaction_t t = {
        .length = 16,
        .tx_data = { (uint8_t)((reg << 1) & 0x7E), value },
        .flags = SPI_TRANS_USE_TXDATA,
    };
    spi_device_transmit(rc522_spi, &t);
}

static uint8_t rc522_read_reg(uint8_t reg)
{
    spi_transaction_t t = {
        .length = 16,
        .tx_data = { (uint8_t)(((reg << 1) & 0x7E) | 0x80), 0x00 },
        .flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA,
    };
    spi_device_transmit(rc522_spi, &t);
    return t.rx_data[1];
}

static void rc522_init(void)
{
    gpio_set_direction(RC522_PIN_RST, GPIO_MODE_OUTPUT);
    gpio_set_level(RC522_PIN_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(50));
    gpio_set_level(RC522_PIN_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(50));

    spi_bus_config_t bus_cfg = {
        .mosi_io_num = RC522_PIN_MOSI,
        .miso_io_num = RC522_PIN_MISO,
        .sclk_io_num = RC522_PIN_SCK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 0,
    };
    spi_bus_initialize(SPI2_HOST, &bus_cfg, SPI_DMA_DISABLED);

    spi_device_interface_config_t dev_cfg = {
        .clock_speed_hz = 5000000,
        .mode = 0,
        .spics_io_num = RC522_PIN_CS,
        .queue_size = 5,
    };
    spi_bus_add_device(SPI2_HOST, &dev_cfg, &rc522_spi);

    rc522_write_reg(RC522_CommandReg, RC522_CMD_SOFTRESET);
    vTaskDelay(pdMS_TO_TICKS(50));

    rc522_write_reg(RC522_TModeReg, 0x8D);
    rc522_write_reg(RC522_TPrescalerReg, 0x3E);
    rc522_write_reg(RC522_TReloadRegH, 0x00);
    rc522_write_reg(RC522_TReloadRegL, 0x1E);
    rc522_write_reg(RC522_TxASKReg, 0x40);
    rc522_write_reg(RC522_ModeReg, 0x3D);

    uint8_t val = rc522_read_reg(RC522_TxControlReg);
    if ((val & 0x03) != 0x03) {
        rc522_write_reg(RC522_TxControlReg, val | 0x03);
    }

    uint8_t version = rc522_read_reg(RC522_VersionReg);
    ESP_LOGI(TAG, "RC522 inicializado - Version: 0x%02X %s",
             version, (version == 0x92) ? "(v2.0)" : (version == 0x91) ? "(v1.0)" : "(desconocida)");
}

static esp_err_t rc522_communicate(uint8_t command, uint8_t *send_data, uint8_t send_len,
                                    uint8_t *recv_data, uint8_t *recv_len)
{
    rc522_write_reg(RC522_CommandReg, RC522_CMD_IDLE);
    rc522_write_reg(RC522_ComIrqReg, 0x7F);
    rc522_write_reg(RC522_FIFOLevelReg, 0x80);

    for (uint8_t i = 0; i < send_len; i++) {
        rc522_write_reg(RC522_FIFODataReg, send_data[i]);
    }

    rc522_write_reg(RC522_CommandReg, command);

    if (command == RC522_CMD_TRANSCEIVE) {
        uint8_t bit_framing = rc522_read_reg(RC522_BitFramingReg);
        rc522_write_reg(RC522_BitFramingReg, bit_framing | 0x80);
    }

    uint16_t timeout = 2500;
    uint8_t irq;
    do {
        irq = rc522_read_reg(RC522_ComIrqReg);
        timeout--;
    } while (timeout > 0 && !(irq & 0x30));

    uint8_t bit_framing = rc522_read_reg(RC522_BitFramingReg);
    rc522_write_reg(RC522_BitFramingReg, bit_framing & ~0x80);

    if (timeout == 0) return ESP_ERR_TIMEOUT;

    uint8_t error = rc522_read_reg(RC522_ErrorReg);
    if (error & 0x13) return ESP_FAIL;

    if (recv_data && recv_len) {
        uint8_t n = rc522_read_reg(RC522_FIFOLevelReg);
        if (n > *recv_len) n = *recv_len;
        *recv_len = n;
        for (uint8_t i = 0; i < n; i++) {
            recv_data[i] = rc522_read_reg(RC522_FIFODataReg);
        }
    }

    return ESP_OK;
}

static esp_err_t rc522_request(void)
{
    rc522_write_reg(RC522_BitFramingReg, 0x07);
    uint8_t cmd = PICC_CMD_REQA;
    uint8_t recv[2];
    uint8_t recv_len = sizeof(recv);
    return rc522_communicate(RC522_CMD_TRANSCEIVE, &cmd, 1, recv, &recv_len);
}

static esp_err_t rc522_anticoll(uint8_t *uid)
{
    rc522_write_reg(RC522_BitFramingReg, 0x00);
    uint8_t cmd[2] = { PICC_CMD_ANTICOLL, 0x20 };
    uint8_t recv[5];
    uint8_t recv_len = sizeof(recv);

    esp_err_t ret = rc522_communicate(RC522_CMD_TRANSCEIVE, cmd, 2, recv, &recv_len);

    if (ret == ESP_OK && recv_len == 5) {
        uint8_t bcc = recv[0] ^ recv[1] ^ recv[2] ^ recv[3];
        if (bcc == recv[4]) {
            memcpy(uid, recv, 4);
            return ESP_OK;
        }
        return ESP_FAIL;
    }
    return ret;
}

static bool rc522_read_card(uint8_t *uid)
{
    if (rc522_request() != ESP_OK) return false;
    if (rc522_anticoll(uid) != ESP_OK) return false;
    return true;
}

static bool rc522_is_authorized(uint8_t *uid)
{
    for (int i = 0; i < NUM_AUTHORIZED; i++) {
        if (memcmp(uid, AUTHORIZED_UIDS[i], 4) == 0) return true;
    }
    return false;
}

// ========================================================
//                    BLE NUS (NimBLE)
// ========================================================

static int nus_rx_callback(uint16_t conn_handle, uint16_t attr_handle,
                           struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    if (ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR) {
        // Solo procesar si estamos en estado ACTIVO
        if (system_state == STATE_ACTIVE) {
            uint16_t len = OS_MBUF_PKTLEN(ctxt->om);
            if (len > 16) len = 16;

            os_mbuf_copydata(ctxt->om, 0, len, ble_message);
            ble_message[len] = '\0';
            ble_message_new = true;

            ESP_LOGI(TAG, "BLE NUS recibido: \"%s\"", ble_message);
        } else {
            ESP_LOGW(TAG, "BLE mensaje ignorado - Panel bloqueado");
        }
    }
    return 0;
}

static const struct ble_gatt_svc_def nus_gatt_svcs[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &nus_service_uuid.u,
        .characteristics = (struct ble_gatt_chr_def[]) {
            {
                .uuid = &nus_rx_char_uuid.u,
                .access_cb = nus_rx_callback,
                .flags = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_NO_RSP,
            },
            { 0 }
        },
    },
    { 0 }
};

static void ble_start_advertising(void)
{
    struct ble_gap_adv_params adv_params = {0};
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;

    struct ble_hs_adv_fields fields = {0};
    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    fields.name = (uint8_t *)ble_svc_gap_device_name();
    fields.name_len = strlen(ble_svc_gap_device_name());
    fields.name_is_complete = 1;

    ble_gap_adv_set_fields(&fields);
    ble_gap_adv_start(BLE_OWN_ADDR_PUBLIC, NULL, BLE_HS_FOREVER, &adv_params, NULL, NULL);

    ESP_LOGI(TAG, "BLE advertising iniciado como \"%s\"", ble_svc_gap_device_name());
}

static void ble_on_sync(void)
{
    ble_start_advertising();
}

static void ble_host_task(void *param)
{
    nimble_port_run();
    nimble_port_freertos_deinit();
}

static void ble_nus_init(void)
{
    nimble_port_init();

    ble_svc_gap_init();
    ble_svc_gatt_init();

    int rc = ble_gatts_count_cfg(nus_gatt_svcs);
    assert(rc == 0);
    rc = ble_gatts_add_svcs(nus_gatt_svcs);
    assert(rc == 0);

    ble_svc_gap_device_name_set("PanelHMI");
    ble_hs_cfg.sync_cb = ble_on_sync;

    nimble_port_freertos_init(ble_host_task);

    ESP_LOGI(TAG, "BLE NUS inicializado");
}

// ========================================================
//                    INIT I2C BUS
// ========================================================

static void i2c_master_init(void)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA,
        .scl_io_num = I2C_MASTER_SCL,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    i2c_param_config(I2C_MASTER_NUM, &conf);
    i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
    ESP_LOGI(TAG, "Bus I2C inicializado (SDA=%d, SCL=%d)", I2C_MASTER_SDA, I2C_MASTER_SCL);
}

// ========================================================
//          FUNCIONES DE ESTADO DEL SISTEMA
// ========================================================

/**
 * Muestra la pantalla del estado bloqueado.
 */
static void show_locked_screen(void)
{
    lcd_clear();
    lcd_print_line(0, "Panel bloqueado");
    lcd_print_line(1, "Acerque credenc.");
}

/**
 * Maneja credencial AUTORIZADA estando en estado BLOQUEADO.
 * → Acceso concedido → pasa a estado ACTIVO.
 */
static void handle_access_granted(void)
{
    uint8_t hour, min, sec;
    char time_str[17];

    ESP_LOGI(TAG, ">>> ACCESO CONCEDIDO <<<");

    // LED verde ON, rojo OFF
    gpio_set_level(LED_RED_PIN, 0);
    gpio_set_level(LED_GREEN_PIN, 1);

    // LCD: Acceso concedido + hora
    lcd_clear();
    lcd_print_line(0, "Acceso concedido");

    if (ds1307_get_time(&hour, &min, &sec) == ESP_OK) {
        snprintf(time_str, sizeof(time_str), "    %02d:%02d:%02d", hour, min, sec);
        lcd_print_line(1, time_str);
    }

    // Buzzer corto 500ms (durante el cual el LED verde está encendido)
    buzzer_short();

    // Apagar LED verde después de 1 segundo total
    vTaskDelay(pdMS_TO_TICKS(500));
    gpio_set_level(LED_GREEN_PIN, 0);

    // Entrar en estado ACTIVO
    system_state = STATE_ACTIVE;

    // Resetear mensaje BLE
    strncpy(current_display_msg, "Sin mensajes", sizeof(current_display_msg));
    ble_message_new = false;

    // LED azul ON (sesión activa)
    gpio_set_level(LED_BLUE_PIN, 1);

    // Mostrar pantalla activa
    lcd_clear();
    lcd_print_line(0, current_display_msg);

    ESP_LOGI(TAG, "Estado: ACTIVO - BLE habilitado");
}

/**
 * Maneja credencial NO AUTORIZADA.
 * → Acceso denegado → vuelve a estado bloqueado.
 */
static void handle_access_denied(void)
{
    ESP_LOGW(TAG, ">>> ACCESO DENEGADO <<<");

    // LCD: Acceso denegado
    lcd_clear();
    lcd_print_line(0, "Acceso denegado");
    lcd_print_line(1, "UID no registrado");

    // Apagar LED rojo antes de parpadear
    gpio_set_level(LED_RED_PIN, 0);
    vTaskDelay(pdMS_TO_TICKS(300));     // Pausa visible antes del parpadeo

    // Buzzer largo ON
    gpio_set_level(BUZZER_PIN, 1);

    // Parpadeo rojo 3 veces (más lento para que se note)
    for (int i = 0; i < 3; i++) {
        gpio_set_level(LED_RED_PIN, 1);
        vTaskDelay(pdMS_TO_TICKS(300));     // ON 300ms
        gpio_set_level(LED_RED_PIN, 0);
        vTaskDelay(pdMS_TO_TICKS(300));     // OFF 300ms
    }

    // Apagar buzzer (3 parpadeos × 600ms = 1800ms, faltan 200ms para 2s)
    vTaskDelay(pdMS_TO_TICKS(200));
    gpio_set_level(BUZZER_PIN, 0);

    // Pausa antes de volver al estado bloqueado
    vTaskDelay(pdMS_TO_TICKS(500));

    // Volver a estado bloqueado
    gpio_set_level(LED_RED_PIN, 1);     // Rojo fijo
    show_locked_screen();

    ESP_LOGI(TAG, "Estado: BLOQUEADO");
}

/**
 * Maneja cierre de sesión (credencial autorizada en estado activo).
 * → Vuelve a estado bloqueado.
 */
static void handle_session_close(void)
{
    ESP_LOGI(TAG, ">>> CIERRE DE SESION <<<");

    // Buzzer corto
    buzzer_short();

    // Apagar LED azul, encender rojo
    gpio_set_level(LED_BLUE_PIN, 0);
    gpio_set_level(LED_RED_PIN, 1);

    // Cambiar estado
    system_state = STATE_LOCKED;

    // Descartar mensajes BLE pendientes
    ble_message_new = false;

    // Volver a pantalla bloqueada
    show_locked_screen();

    ESP_LOGI(TAG, "Estado: BLOQUEADO");
}

// ========================================================
//                      APP MAIN
// ========================================================

void app_main(void)
{
    // 1. NVS (requerido por BLE)
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_flash_init();
    }

    // 2. SPI + RC522 primero
    rc522_init();

    // 3. BLE
    ble_nus_init();

    
    // 4. Esperar BLE
    vTaskDelay(pdMS_TO_TICKS(2000));

    // 5. GPIOs
    leds_buzzer_init();

    // 6. I2C + LCD (doble init por seguridad)
   
    i2c_master_init();
    lcd_init();

    // 7. RTC
    ds1307_set_time(23, 20, 0);    // ← PON TU HORA ACTUAL

    // 8. Estado inicial: BLOQUEADO
    system_state = STATE_LOCKED;
    gpio_set_level(LED_RED_PIN, 1);     // Rojo fijo
    show_locked_screen();

    ESP_LOGI(TAG, "=== PANEL HMI INICIADO - Estado: BLOQUEADO ===");

    // ===== LOOP PRINCIPAL =====
    uint8_t uid[4];
    uint8_t hour, min, sec;
    char time_str[17];

    while (1) {
        // --- Leer RFID ---
        if (rc522_read_card(uid)) {
            ESP_LOGI(TAG, "Tarjeta: %02X %02X %02X %02X",
                     uid[0], uid[1], uid[2], uid[3]);

            if (system_state == STATE_LOCKED) {
                // Estado BLOQUEADO: verificar credencial
                if (rc522_is_authorized(uid)) {
                    handle_access_granted();
                } else {
                    handle_access_denied();
                }
            } else if (system_state == STATE_ACTIVE) {
                // Estado ACTIVO: solo tarjeta autorizada cierra sesión
                if (rc522_is_authorized(uid)) {
                    handle_session_close();
                }
            }

            // Esperar a que retiren la tarjeta
            vTaskDelay(pdMS_TO_TICKS(1500));
            continue;
        }

        // --- Estado ACTIVO: actualizar LCD ---
        if (system_state == STATE_ACTIVE) {
            // Si llegó un mensaje BLE nuevo
            if (ble_message_new) {
                ble_message_new = false;
                strncpy(current_display_msg, ble_message, sizeof(current_display_msg) - 1);
                current_display_msg[sizeof(current_display_msg) - 1] = '\0';
                lcd_print_line(0, current_display_msg);
                ESP_LOGI(TAG, "Mensaje BLE: \"%s\"", current_display_msg);
            }

            // Actualizar hora en línea 2
            if (ds1307_get_time(&hour, &min, &sec) == ESP_OK) {
                snprintf(time_str, sizeof(time_str), "    %02d:%02d:%02d", hour, min, sec);
                lcd_print_line(1, time_str);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(200));
    }
}