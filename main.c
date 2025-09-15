/* i2c - Simple example

 Simple I2C example that shows how to initialize I2C
 as well as reading and writing from and to registers for a sensor connected over I2C.

 The sensor used in this example is a MPU9250 inertial measurement unit.

 For other examples please check:
 https://github.com/espressif/esp-idf/tree/master/examples

 See README.md file to get detailed usage of this example.

 This example code is in the Public Domain (or CC0 licensed, at your option.)

 Unless required by applicable law or agreed to in writing, this
 software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
 CONDITIONS OF ANY KIND, either express or implied.
 */

#include <stdio.h>
#include "cJSON.h"
#include "esp_log.h"
#include "driver/i2c.h"
#include "xm125.h"
#include "driver/gpio.h"


static const char *TAG = "i2c_radar";

#define MPU9250_SENSOR_ADDR                 0x68        /*!< Slave address of the MPU9250 sensor */
#define MPU9250_WHO_AM_I_REG_ADDR           0x75        /*!< Register addresses of the "who am I" register */

#define MPU9250_PWR_MGMT_1_REG_ADDR         0x6B        /*!< Register addresses of the power managment register */
#define MPU9250_RESET_BIT                   7

#define GPIO_OUTPUT_PIN_SEL  ((1ULL<<LED_RADAR_D) | (1ULL<<LED_RADAR_P))

int led_d = 0;
int led_p = 0;

static QueueHandle_t gpio_evt_queue = NULL;
//struct config_t config;


void task_xm125_distance(void *pvParameter){
	while (1) {
				if (!write_register(LED_RADAR_D, I2C_ADDR_D,
						DISTANCE_REG_COMMAND_ADDRESS,
						DISTANCE_REG_COMMAND_ENUM_MEASURE_DISTANCE)) {
					//ERROR
					ESP_LOGE("XM125", "ERRO , measure"); //////

				}
				// Wait for measure distance to be done
				if (!wait_not_busy()) {
					//ERROR
					ESP_LOGE("XM125", "ERRO , wait"); ///////

				}
				uint32_t result;
				if (!read_register(LED_RADAR_D, I2C_ADDR_D,
						DISTANCE_REG_DISTANCE_RESULT_ADDRESS, &result)) {
					//ERROR
					ESP_LOGE("XM125", "ERRO , result"); //////

				}
				char buffer_uart[64];

				// Did we detect a peak?
				uint32_t num_distances = (result
						& DISTANCE_REG_DISTANCE_RESULT_FIELD_NUM_DISTANCES_MASK) >>
				DISTANCE_REG_DISTANCE_RESULT_FIELD_NUM_DISTANCES_POS;
				// Print peak if found
				if (num_distances > 0) {
					if(led_d == 0){
						led_d = 1;
					gpio_set_level(I2C_ADDR_D, 1);}
					//vTaskDelay(pdMS_TO_TICKS(500));
					 //uint32_t presence_distance_mm;
									// read_register(LED_RADAR_P,I2C_ADDR_P,PRESENCE_REG_INTRA_PRESENCE_SCORE_ADDRESS, &
									 	//			 presence_distance_mm);
									 //printf("score = %" PRIu32 " \n",presence_distance_mm);
					uint32_t peak_distance_mm;
					if (read_register(LED_RADAR_D, I2C_ADDR_D,
							DISTANCE_REG_PEAK0_DISTANCE_ADDRESS,
							&peak_distance_mm)) {

						snprintf(buffer_uart,              // Onde a string será salva
						         sizeof(buffer_uart),      // O tamanho máximo do buffer (para segurança)
						         "Peak distance1: %" PRIu32 " mm\r\n", // A string de formatação
								 lineariza(peak_distance_mm));

						//uart_write_bytes(UART_NUM_0,                // A porta UART que você quer usar
						  //               buffer_uart,             // O buffer com a mensagem
						  //               strlen(buffer_uart));

						if(acess_is_connected()){
							esp_ble_gatts_send_indicate(acess_spp_gatts_if(), acess_spp_conn_id(), acess_spp_handle_table(SPP_IDX_SPP_DATA_NTY_VAL),strlen(buffer_uart), (uint8_t *)buffer_uart, false);
						}
						printf("Peak distance: %" PRIu32 " mm\n", lineariza(peak_distance_mm));


						if(lineariza(peak_distance_mm) > 320){
							printf("entrou no lora\n");
							SX1276_send(&SX1276,lineariza(peak_distance_mm),1);
						}
					} else {
						//ERROR
						ESP_LOGE("XM125", "ERRO , exmaple radar");

					}
				} else {
					if(led_d == 1){
						led_d = 0;
					gpio_set_level(I2C_ADDR_D, 0);}

					//printf("No peak detected\n");
				}



	/////////////////////////////////presença


				vTaskDelay(pdMS_TO_TICKS(10));
			}
}

void task_xm125_presence(void *pvParameter){

	while(1){
	if (!write_register(LED_RADAR_P,I2C_ADDR_P,PRESENCE_REG_COMMAND_ADDRESS,
					PRESENCE_REG_COMMAND_ENUM_START_DETECTOR))
					{
					//ERROR
						ESP_LOGE("XM125_p", "ERRO ,adr");
					}
					// Wait for command be done
					if (!wait_not_busy_p())
					{
						//ERROR
						ESP_LOGE("XM125_P", "ERRO , wait");
						 }
						 // Read detector result
						 uint32_t result_p;
						 if (!read_register(LED_RADAR_P,I2C_ADDR_P,PRESENCE_REG_PRESENCE_RESULT_ADDRESS, &result_p))
						 {
						 //ERROR
							 ESP_LOGE("XM125_p", "ERRO , result");
						 }
						 // Was presence detected?
						 bool presence_detected = (result_p &
						 PRESENCE_REG_PRESENCE_RESULT_FIELD_PRESENCE_DETECTED_MASK) != 0;
						 bool presence_detected_sticky = (result_p &
						 PRESENCE_REG_PRESENCE_RESULT_FIELD_PRESENCE_DETECTED_STICKY_MASK) != 0;

						 uint32_t presence_distance_mm;
						 read_register(LED_RADAR_P,I2C_ADDR_P,PRESENCE_REG_INTRA_PRESENCE_SCORE_ADDRESS, &
						 				 presence_distance_mm);
						 //printf("score = %" PRIu32 " \n",presence_distance_mm);
						 if(detectar_obj(presence_distance_mm)){
							 if(led_p == 0){
								 led_p = 1;
							 				//gpio_set_level(I2C_ADDR_P, 1);
								 }
							 //vTaskDelay(pdMS_TO_TICKS(200));
						printf("detectou obj\n");
						 }else{
							 if(led_p == 1){
								 led_p = 0;
							 				//gpio_set_level(I2C_ADDR_P, 0);
								 }
							 //vTaskDelay(pdMS_TO_TICKS(200));
						 }
						 // Print peak if found
						/* if (presence_detected || presence_detected_sticky)
						 {
						 uint32_t presence_distance_mm;
						 if (read_register(LED_RADAR_P,I2C_ADDR_P,PRESENCE_REG_INTRA_PRESENCE_SCORE_ADDRESS, &
						 presence_distance_mm))
						 {
						 printf("Presence detected at distance: %" PRIu32 " mm\n",
						 presence_distance_mm);
						 }
						 else
						 {
						 //ERROR
							 ESP_LOGE("XM125_p", "ERRO , read addr");
						 }
						 }
						 else
						 {
						 printf("No presence detected\n");
						 }*/
						 vTaskDelay(pdMS_TO_TICKS(10));
}

}


void app_main(void) {

	

	gpio_config_t io_conf;
	io_conf.intr_type = GPIO_INTR_DISABLE;         // Desabilitar interrupções
	io_conf.mode = GPIO_MODE_OUTPUT;               // Definir como modo de saída
	io_conf.pin_bit_mask = (1ULL << LED_RADAR_D) | (1ULL << LED_RADAR_P); // Máscara de bits para os pinos
	io_conf.pull_down_en = 0;                      // Desabilitar pull-down
	io_conf.pull_up_en = 0;                        // Desabilitar pull-up
	gpio_config(&io_conf);                         // Aplicar a configuração

	ESP_ERROR_CHECK(i2c_master_init_t());
	bool verif = false;
	while(!verif)
	if (example_setup_and_measure() && example_setup_and_start()) {
		verif = true;
		ESP_LOGI("XM125", "OK , exmaplo radar");
		xTaskCreate(task_xm125_distance,              // 1. Ponteiro para a função da task
		                "Task Distance",            // 2. Nome da task (para depuração)
		                10000,                 // 3. Tamanho da pilha (stack size) em palavras
		                NULL,                 // 4. Parâmetros da task (não estamos usando)
		                1,                    // 5. Prioridade da task
		                NULL);                // 6. Handle da task (não estamos usando)
		xTaskCreate(task_xm125_presence,              // 1. Ponteiro para a função da task
		                "Task Presence",            // 2. Nome da task (para depuração)
						10000,                 // 3. Tamanho da pilha (stack size) em palavras
		                NULL,                 // 4. Parâmetros da task (não estamos usando)
		                1,                    // 5. Prioridade da task
		                NULL);                // 6. Handle da task (não estamos usando)

	}


	ESP_LOGI(TAG, "I2C de-initialized successfully");
}

