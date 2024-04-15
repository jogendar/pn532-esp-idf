#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <esp_log.h>
#include <esp_log_internal.h>
#include <driver/spi_master.h>
#include <driver/gpio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "sdkconfig.h"

#include "pn532.h"

#define BLINK_GPIO CONFIG_BLINK_GPIO

#define PN532_SCK   CONFIG_PN532_SCK
#define PN532_MOSI  CONFIG_PN532_MOSI
#define PN532_SS    CONFIG_PN532_SS
#define PN532_MISO  CONFIG_PN532_MISO

static const char *TAG = "APP";

static pn532_t nfc;
spi_device_handle_t spi_device;

void blink_task(void *pvParameter)
{
    // gpio_pad_select_gpio(BLINK_GPIO);
    // gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);
    while (1)
    {
        // gpio_set_level(BLINK_GPIO, 0);
        // vTaskDelay(900 / portTICK_PERIOD_MS);
        // gpio_set_level(BLINK_GPIO, 1);
        // vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

void spi_master_init(int16_t GPIO_MOSI, int16_t GPIO_SCLK, int16_t GPIO_MISO)
{
	esp_err_t ret;
	ESP_LOGI(TAG, "GPIO_MOSI=%d",GPIO_MOSI);
	ESP_LOGI(TAG, "GPIO_SCLK=%d",GPIO_SCLK);
	spi_bus_config_t buscfg = {
		.sclk_io_num = GPIO_SCLK,
		.mosi_io_num = GPIO_MOSI,
		.miso_io_num = GPIO_MISO,
		.quadwp_io_num = -1,
		.quadhd_io_num = -1
	};

	//ret = spi_bus_initialize( HSPI_HOST, &buscfg, 1 );
	ret = spi_bus_initialize( SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO );
	ESP_LOGD(TAG, "spi_bus_initialize=%d",ret);
	assert(ret==ESP_OK);
}

void device_init(spi_device_handle_t* dev_handle, int16_t GPIO_CS)
{
    esp_err_t ret;
    ESP_LOGI(TAG, "GPIO_CS=%d",GPIO_CS);
	if ( GPIO_CS >= 0 ) {
		//gpio_pad_select_gpio( GPIO_CS );
		gpio_reset_pin( GPIO_CS );
		gpio_set_direction( GPIO_CS, GPIO_MODE_OUTPUT );
		gpio_set_level( GPIO_CS, 0 );
	}

    spi_device_interface_config_t devcfg={
		.clock_speed_hz = SPI_MASTER_FREQ_8M,
		.queue_size = 7,
		.mode = 0,
		.flags = SPI_DEVICE_NO_DUMMY,
	};

	if ( GPIO_CS >= 0 ) {
		devcfg.spics_io_num = GPIO_CS;
	} else {
		devcfg.spics_io_num = -1;
	}
	
	spi_device_handle_t handle;
	ret = spi_bus_add_device( SPI2_HOST, &devcfg, &handle);
	ESP_LOGI(TAG, "spi_bus_add_device=%d",ret);
	assert(ret==ESP_OK);
	*dev_handle = handle;
}

void nfc_task(void *pvParameter)
{
    // spi_master_init(PN532_MOSI, PN532_SCK, PN532_MISO);
    // device_init(&spi_device, PN532_SS);
    pn532_spi_init(&nfc, PN532_SCK, PN532_MISO, PN532_MOSI, PN532_SS);
    pn532_begin(&nfc);

    uint32_t versiondata = pn532_getFirmwareVersion(&nfc);
    if (!versiondata)
    {
        ESP_LOGI(TAG, "Didn't find PN53x board");
        while (1)
        {
            vTaskDelay(1000 / portTICK_RATE_MS);
        }
    }
    // Got ok data, print it out!
    ESP_LOGI(TAG, "Found chip PN5 %x", (versiondata >> 24) & 0xFF);
    ESP_LOGI(TAG, "Firmware ver. %d.%d", (versiondata >> 16) & 0xFF, (versiondata >> 8) & 0xFF);

    // configure board to read RFID tags
    pn532_SAMConfig(&nfc);

    ESP_LOGI(TAG, "Waiting for an ISO14443A Card ...");

    while (1)
    {
        uint8_t success;
        uint8_t uid[] = {0, 0, 0, 0, 0, 0, 0}; // Buffer to store the returned UID
        uint8_t uidLength;                     // Length of the UID (4 or 7 bytes depending on ISO14443A card type)

        // Wait for an ISO14443A type cards (Mifare, etc.).  When one is found
        // 'uid' will be populated with the UID, and uidLength will indicate
        // if the uid is 4 bytes (Mifare Classic) or 7 bytes (Mifare Ultralight)
        success = pn532_readPassiveTargetID(&nfc, PN532_MIFARE_ISO14443A, uid, &uidLength, 0);

        if (success)
        {
            // Display some basic information about the card
            ESP_LOGI(TAG, "Found an ISO14443A card");
            ESP_LOGI(TAG, "UID Length: %d bytes", uidLength);
            ESP_LOGI(TAG, "UID Value:");
            esp_log_buffer_hexdump_internal(TAG, uid, uidLength, ESP_LOG_INFO);   
            vTaskDelay(1000 / portTICK_RATE_MS);         
        }
        else
        {
            // PN532 probably timed out waiting for a card
            ESP_LOGI(TAG, "Timed out waiting for a card");
        }
    }
}

void app_main()
{
    xTaskCreate(&blink_task, "blink_task", configMINIMAL_STACK_SIZE, NULL, 5, NULL);
    xTaskCreate(&nfc_task, "nfc_task", 4096, NULL, 4, NULL);
}
