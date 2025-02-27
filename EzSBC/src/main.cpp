#include <Arduino.h>
#include <LiquidCrystal_I2C.h>
#include <driver/spi_master.h>
#include <esp32_wifi/wifi.h>
#include <esp_event.h>
#include <freertos/event_groups.h>
#include <mqtt.h>

#define VSPI_MOSI 17
#define VSPI_MISO 18
#define VSPI_CLK 19
#define NRF_CS 16
#define NRF_IRQ 15

#define NRF_COMMAND_R_REGISTER 0b00000000
#define NRF_COMMAND_W_REGISTER 0b00100000
#define NRF_COMMAND_R_RX_PAYLOAD 0b01100001
#define NRF_COMMAND_FLUSH_RX 0b11100010

#define NRF_REGISTER_CONFIG 0x00
#define NRF_REGISTER_RF_SETUP 0x06
#define NRF_REGISTER_STATUS 0x07
#define NRF_REGISTER_RX_PW_P0 0x11
#define NRF_REGISTER_FIFO_STATUS 0x17

#define WAIT_ALL pdTRUE
#define WAIT_ONE pdFALSE
#define CLEAR_ON_EXIT pdTRUE
#define NO_CLEAR pdFALSE

#define WIFI_CONNECTED 0x01
#define WIFI_STARTED 0x02

#define MQTT_BROKER "192.168.68.106"
#define MQTT_PORT 1883
#define MQTT_CLIENT "nrf-bridge"

EventGroupHandle_t eventGroup;
ESP32Wifi network;
LiquidCrystal_I2C lcd(0x27, 20, 4);
spi_device_handle_t handleSPI;
char ipAddress[ESP32Wifi::IpAddressLength];
MQTT mqtt;

#define ARDUINO_STACK 8192
#define ARDUINO_PRIORITY 1
#define ARDUINO_CORE 1

TaskHandle_t startTask(TaskFunction_t pcode, const char *const pname, void *const pparameters = 0) {
  TaskHandle_t handle;
  xTaskCreatePinnedToCore(
      pcode,
      pname,
      ARDUINO_STACK,
      pparameters,
      ARDUINO_PRIORITY,
      &handle,
      ARDUINO_CORE);
  return handle;
}

uint8_t writeCommand(uint8_t command) {
  spi_transaction_t transaction;

  memset(&transaction, 0, sizeof transaction);
  transaction.flags = SPI_TRANS_USE_RXDATA | SPI_TRANS_USE_TXDATA;
  transaction.tx_data[0] = command;
  transaction.length = 8;
  spi_device_transmit(handleSPI, &transaction);

  return transaction.rx_data[0];
}

uint8_t writeRegister(uint8_t target, uint8_t value) {
  spi_transaction_t transaction;

  memset(&transaction, 0, sizeof transaction);
  transaction.flags = SPI_TRANS_USE_RXDATA | SPI_TRANS_USE_TXDATA;
  transaction.tx_data[0] = NRF_COMMAND_W_REGISTER | target;
  transaction.tx_data[1] = value;
  transaction.length = 16;
  spi_device_transmit(handleSPI, &transaction);

  return transaction.rx_data[0];
}

uint8_t readRegister(uint8_t target) {
  spi_transaction_t transaction;

  memset(&transaction, 0, sizeof transaction);
  transaction.flags = SPI_TRANS_USE_RXDATA | SPI_TRANS_USE_TXDATA;
  transaction.tx_data[0] = NRF_COMMAND_R_REGISTER | target;
  transaction.tx_data[1] = 0x00;
  transaction.length = 16;
  spi_device_transmit(handleSPI, &transaction);

  return transaction.rx_data[1];
}

uint8_t getStatus() {
  return writeCommand(0xFF);
}

void clearIRQs() {
  writeRegister(NRF_REGISTER_STATUS, 0b01110000);
}

void flushRx() {
  writeCommand(NRF_COMMAND_FLUSH_RX);
}

void enableRx() {
  clearIRQs();
  flushRx();

  // Set data packet length to 32.
  writeRegister(NRF_REGISTER_RX_PW_P0, 32);

  // Set 250kps air data rate.
  writeRegister(NRF_REGISTER_RF_SETUP, 0b00101110);

  // Power up radio in receive mode.
  writeRegister(NRF_REGISTER_CONFIG, 0b00001011);

  // Wait for radio to settle.
  delay(10);
}

void disableRx() {
  // Power down radio.
  writeRegister(NRF_REGISTER_CONFIG, 0b00001000);
}

void readRxFIFO(void *preceive, int length_bytes) {
  spi_transaction_t transaction;
  uint8_t *pcommand, *pdata;

  pcommand = new byte[length_bytes + 1];
  memset(pcommand + 1, 0, length_bytes);
  pcommand[0] = NRF_COMMAND_R_RX_PAYLOAD;

  pdata = new byte[length_bytes + 1];

  memset(&transaction, 0, sizeof transaction);
  transaction.rx_buffer = pdata;
  transaction.tx_buffer = pcommand;
  transaction.length = (length_bytes * 8) + 8;
  spi_device_transmit(handleSPI, &transaction);

  memcpy(preceive, pdata + 1, length_bytes);

  delete pcommand;
  delete pdata;
}

void display(int x, int y, char *pstring) {
  char trunc[21];
  int n, len = strlen(pstring);

  for (n = 0; n < 20; n++) {
    if (n < len) {
      trunc[n] = pstring[n];
    } else {
      trunc[n] = ' ';
    }
  }
  trunc[20] = 0;

  lcd.setCursor(x, y);
  lcd.print(trunc);
}

void processPacket(char *poriginal) {
  char s[64];
  
  // Packet is a string with the form TOPIC=VALUE.

  char *ppacket = strdup(poriginal);

  char *ptopic = strtok(ppacket, "=");
  char *pvalue = strtok(NULL, "=");

  if (!ptopic || !pvalue) {
    Serial.println("Packet can't be parsed");
    goto exit;
  }

  sprintf(s, "T=%s", ptopic);
  display(0, 2, s);
  sprintf(s, "V=%s", pvalue);
  display(0, 3, s);

  mqtt.publish(ptopic, pvalue, true);

exit:
  free(ppacket);
}

void setup() {
  spi_bus_config_t spi_config;
  spi_device_interface_config_t dev_config;

  lcd.init();
  lcd.backlight();
  Serial.println("LCD initialised");

  Serial.begin(115200);
  Serial.println("Starting");
  lcd.setCursor(0, 0);
  lcd.print("Starting...");

  eventGroup = xEventGroupCreate();
  esp_event_loop_create_default();
  Serial.println("Event groups created");

  network.init(eventGroup, WIFI_STARTED);
  xEventGroupWaitBits(eventGroup, WIFI_STARTED, NO_CLEAR, WAIT_ALL, pdMS_TO_TICKS(10000));
  Serial.println("WiFi initialised");

  network.connect(WIFI_CONNECTED);
  Serial.println("Starting WiFi connection");

  memset(&spi_config, 0, sizeof spi_config);
  spi_config.mosi_io_num = VSPI_MOSI;
  spi_config.miso_io_num = VSPI_MISO;
  spi_config.sclk_io_num = VSPI_CLK;
  spi_bus_initialize(SPI3_HOST, &spi_config, SPI_DMA_DISABLED);

  memset(&dev_config, 0, sizeof dev_config);
  dev_config.clock_speed_hz = 5000000;
  dev_config.spics_io_num = NRF_CS;
  dev_config.queue_size = 1;
  spi_bus_add_device(SPI3_HOST, &dev_config, &handleSPI);

  Serial.println("SPI initialised");

  xEventGroupWaitBits(eventGroup, WIFI_CONNECTED, NO_CLEAR, WAIT_ALL, pdMS_TO_TICKS(10000));
  network.getIpAddress(ipAddress);
  Serial.printf("WiFi connected at %s\n", ipAddress);
  lcd.setCursor(0, 0);
  lcd.printf("IP %s", ipAddress);

  if (!mqtt.connect(MQTT_BROKER, MQTT_PORT, MQTT_CLIENT)) {
    Serial.println("Error connecting to MQTT broker");
  } else {
    Serial.println("MQTT initialised");
  }

  enableRx();
  pinMode(NRF_IRQ, INPUT);
  Serial.println("Rx enabled");
  lcd.setCursor(0, 1);
  lcd.print("Rx enabled");
}

void indicatorTask(void *pparams) {
  lcd.setCursor(19, 3);
  lcd.print("*");

  vTaskDelay(pdMS_TO_TICKS(1000));

  lcd.setCursor(19, 3);
  lcd.print(" ");

  vTaskDelete(NULL);
}

bool rxAvailable() {
  return (readRegister(NRF_REGISTER_FIFO_STATUS) & 0b00000001) == 0;
}

void loop() {
  char data[33];

  while (rxAvailable()) {
    readRxFIFO(data, 32);
    data[32] = 0;
    Serial.printf("Data: [%s]\n", data);
    processPacket(data);
    startTask(indicatorTask, "indicator");
  }

  vTaskDelay(pdMS_TO_TICKS(100));
}
