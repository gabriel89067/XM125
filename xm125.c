/*
 * xm125.c
 *
 *  Created on: 30 de jun. de 2025
 *      Author: gabri
 */

#include "xm125.h"


uint32_t values_presence[8 * NMAX_MED] = {0};
uint32_t values_p[NMAX_MED + 1] = {0};
uint32_t values[NMAX_MED + 1] = {0};
int medida_p;
int medida = 0;
uint32_t soma = 0;
uint32_t soma_p = 0;
uint32_t media = 0;
uint32_t media_p = 0;
int limite = 1000;
int margem = 20;
int margem_peak = 20;




bool read_register(int led,int addr_modo ,uint16_t reg_addr, uint32_t *reg_data)
 {
	//gpio_set_level(led, 1);
	 int status;
	 uint8_t transmit_data[REG_ADDRESS_LENGTH];
	 transmit_data[0] = (reg_addr >> 8) & 0xff;

	 transmit_data[1] = (reg_addr >> 0) & 0xff;

	 status = i2c_master_write_to_device(0, addr_modo,
	 transmit_data, REG_ADDRESS_LENGTH,
	 I2C_TIMEOUT_MS);
	 if (status != 0)
	 {
	 //gpio_set_level(led, 0);
	 return false;
	 }
	 uint8_t receive_data[REG_DATA_LENGTH];
	 status = i2c_master_read_from_device(0, addr_modo,
	 receive_data, REG_DATA_LENGTH,
	 I2C_TIMEOUT_MS);
	 if (status != 0)
	 {
	 //gpio_set_level(led, 0);
	 return false;
	 }
	 // Convert bytes to uint32_t
	 uint32_t val = receive_data[0];

	 val = val << 8;
	 val |= receive_data[1];

	 val = val << 8;
	 val |= receive_data[2];

	 val = val << 8;
	 val |= receive_data[3];

	 *reg_data = val;
	 //gpio_set_level(led, 0);
	 return true;
	 }
bool read_register1(uint16_t reg_addr, uint32_t *reg_data)
 {
	uint8_t i2c_addrr = 0x52;
	uint8_t transmit_data[REG_ADDRESS_LENGTH];
		 transmit_data[0] = (reg_addr >> 8) & 0xff;
		 ESP_LOGI("XM125_R", "transmit = %02X",transmit_data[0]);
		 transmit_data[1] = (reg_addr >> 0) & 0xff;
		 ESP_LOGI("XM125_R", "transmit = %02X",transmit_data[1]);

	i2c_cmd_handle_t cmd = NULL;
		 esp_err_t ret_w =ESP_OK;

		     uint8_t cmd_buff[I2C_LINK_RECOMMENDED_SIZE(1)] = { 0 };
		     cmd = i2c_cmd_link_create_static(cmd_buff, I2C_LINK_RECOMMENDED_SIZE(1));
		     ret_w = i2c_master_start(cmd);
		     if (ret_w != ESP_OK) {
		    	 ESP_LOGE("XM125", "ERRO1"); //////
		         goto end;
		     }
		     ret_w = i2c_master_write_byte(cmd, (i2c_addrr << 1) | I2C_MASTER_WRITE, true);
		     if (ret_w != ESP_OK) {
		    	 ESP_LOGE("XM125", "ERRO2");
		         goto end;
		     }
		     if(REG_ADDRESS_LENGTH){
		         ret_w = i2c_master_write(cmd, transmit_data, REG_ADDRESS_LENGTH, true);
		         if (ret_w != ESP_OK) {
		        	 ESP_LOGE("XM125", "ERRO3");
		             goto end;
		         }
		     }
		     ret_w = i2c_master_stop(cmd);
		     if (ret_w != ESP_OK) {
		    	 ESP_LOGE("XM125", "ERRO4");
		         goto end;
		     }
		     ret_w = i2c_master_cmd_begin(0, cmd, I2C_TIMEOUT_MS);

		 end:
		     if(cmd != NULL){
		         i2c_cmd_link_delete_static(cmd);
		     }

		     /////////////////////////////////////////
		     //ESP_ERROR_CHECK(ret_w);


		     uint8_t receive_data[REG_DATA_LENGTH];
			 esp_err_t ret_r =ESP_OK;
	         ret_r = i2c_master_read_from_device(0, i2c_addrr, receive_data, REG_DATA_LENGTH , I2C_TIMEOUT_MS);

	         //ESP_ERROR_CHECK(ret_r);
	        // Convert bytes to uint32_t
	        	 uint32_t val = receive_data[0];
	        	 ESP_LOGI("XM125_R", "transmit = %02X",receive_data[0]);
	        	 val = val << 8;
	        	 val |= receive_data[1];
	        	 ESP_LOGI("XM125_R", "transmit = %02X",receive_data[1]);
	        	 val = val << 8;
	        	 val |= receive_data[2];
	        	 ESP_LOGI("XM125_R", "transmit = %02X",receive_data[2]);
	        	 val = val << 8;
	        	 val |= receive_data[3];
	        	 ESP_LOGI("XM125_R", "transmit = %02X",receive_data[3]);
	        	 *reg_data = val;
	        	 return true;

	 }

bool write_register(int led,int addr_modo ,uint16_t reg_addr, uint32_t reg_data)
 {
//gpio_set_level(led, 1);
int status;
 uint8_t transmit_data[REG_ADDRESS_LENGTH + REG_DATA_LENGTH];
 // Convert uint16_t address to bytes
 transmit_data[0] = (reg_addr >> 8) & 0xff;

 transmit_data[1] = (reg_addr >> 0) & 0xff;

 // Convert uint32_t reg_data to bytes
 transmit_data[2] = (reg_data >> 24) & 0xff;

 transmit_data[3] = (reg_data >> 16) & 0xff;

 transmit_data[4] = (reg_data >> 8) & 0xff;

 transmit_data[5] = (reg_data >> 0) & 0xff;

 status = i2c_master_write_to_device(0, addr_modo,
 transmit_data,
 REG_ADDRESS_LENGTH + REG_DATA_LENGTH,
 I2C_TIMEOUT_MS);
 if (status != 0)
 {
	 //gpio_set_level(led, 0);
 return false;
 }
 //gpio_set_level(led, 0);
 return true;
 }

bool write_register1(uint16_t reg_addr, uint32_t reg_data){
	uint8_t i2c_addrr = 0x52;
	uint8_t transmit_data[REG_DATA_LENGTH+REG_ADDRESS_LENGTH];
	 // Convert uint16_t address to bytes
	 transmit_data[0] = (reg_addr >> 8) & 0xff;
	 ESP_LOGI("XM125", "transmit = %02X",transmit_data[0]);
	 transmit_data[1] = (reg_addr >> 0) & 0xff;
	 ESP_LOGI("XM125", "transmit = %02X",transmit_data[1]);
	 // Convert uint32_t reg_data to bytes
	 transmit_data[2] = (reg_data >> 24) & 0xff;
	 ESP_LOGI("XM125", "transmit = %02X",transmit_data[2]);
	 transmit_data[3] = (reg_data >> 16) & 0xff;
	 ESP_LOGI("XM125", "transmit = %02X",transmit_data[3]);
	 transmit_data[4] = (reg_data >> 8) & 0xff;
	 ESP_LOGI("XM125", "transmit = %02X",transmit_data[4]);
	 transmit_data[5] = (reg_data >> 0) & 0xff;
	 ESP_LOGI("XM125", "transmit = %02X",transmit_data[5]);
	 i2c_cmd_handle_t cmd = NULL;
	 esp_err_t ret =ESP_OK;

	     uint8_t cmd_buff[I2C_LINK_RECOMMENDED_SIZE(1)] = { 0 };
	     cmd = i2c_cmd_link_create_static(cmd_buff, I2C_LINK_RECOMMENDED_SIZE(1));
	     ret = i2c_master_start(cmd);
	     if (ret != ESP_OK) {
	    	 ESP_LOGE("XM125", "ERRO1"); //////
	         goto end;
	     }
	     ret = i2c_master_write_byte(cmd, (i2c_addrr << 1) | I2C_MASTER_WRITE, true);
	     if (ret != ESP_OK) {
	    	 ESP_LOGE("XM125", "ERRO2");
	         goto end;
	     }
	     if(REG_DATA_LENGTH+REG_ADDRESS_LENGTH){
	         ret = i2c_master_write(cmd, transmit_data, REG_DATA_LENGTH+REG_ADDRESS_LENGTH, true);
	         if (ret != ESP_OK) {
	        	 ESP_LOGE("XM125", "ERRO3");
	             goto end;
	         }
	     }
	     ret = i2c_master_stop(cmd);
	     if (ret != ESP_OK) {
	    	 ESP_LOGE("XM125", "ERRO4");
	         goto end;
	     }
	     ret = i2c_master_cmd_begin(0, cmd, I2C_TIMEOUT_MS);

	 end:
	     if(cmd != NULL){
	         i2c_cmd_link_delete_static(cmd);
	     }

	     /////////////////////////////////////////
	     //ESP_ERROR_CHECK(ret);
return true;
}

bool configuration_ok_p(void)
 {
 uint32_t status = 0;
 if (!read_register(LED_RADAR_P,I2C_ADDR_P,PRESENCE_REG_DETECTOR_STATUS_ADDRESS, &status))
 {
 //ERROR
 return false;
 }
 uint32_t config_ok_mask =
 PRESENCE_REG_DETECTOR_STATUS_FIELD_RSS_REGISTER_OK_MASK |
 PRESENCE_REG_DETECTOR_STATUS_FIELD_CONFIG_CREATE_OK_MASK |
 PRESENCE_REG_DETECTOR_STATUS_FIELD_SENSOR_CREATE_OK_MASK |
 PRESENCE_REG_DETECTOR_STATUS_FIELD_SENSOR_CALIBRATE_OK_MASK |
 PRESENCE_REG_DETECTOR_STATUS_FIELD_DETECTOR_CREATE_OK_MASK |
 PRESENCE_REG_DETECTOR_STATUS_FIELD_DETECTOR_BUFFER_OK_MASK |
 PRESENCE_REG_DETECTOR_STATUS_FIELD_SENSOR_BUFFER_OK_MASK |
 PRESENCE_REG_DETECTOR_STATUS_FIELD_CONFIG_APPLY_OK_MASK;
 if (status != config_ok_mask)
 {
 //ERROR
 return false;
 }
 return true;
 }

bool configuration_ok(void)
{
uint32_t status = 0;
if (!read_register(LED_RADAR_D,I2C_ADDR_D,DISTANCE_REG_DETECTOR_STATUS_ADDRESS, &status))
{
//ERROR
return false;
}
uint32_t config_ok_mask =
DISTANCE_REG_DETECTOR_STATUS_FIELD_RSS_REGISTER_OK_MASK |
DISTANCE_REG_DETECTOR_STATUS_FIELD_CONFIG_CREATE_OK_MASK |
DISTANCE_REG_DETECTOR_STATUS_FIELD_SENSOR_CREATE_OK_MASK |
DISTANCE_REG_DETECTOR_STATUS_FIELD_DETECTOR_CREATE_OK_MASK |
DISTANCE_REG_DETECTOR_STATUS_FIELD_DETECTOR_BUFFER_OK_MASK |
DISTANCE_REG_DETECTOR_STATUS_FIELD_SENSOR_BUFFER_OK_MASK |
DISTANCE_REG_DETECTOR_STATUS_FIELD_CALIBRATION_BUFFER_OK_MASK |
DISTANCE_REG_DETECTOR_STATUS_FIELD_CONFIG_APPLY_OK_MASK |
DISTANCE_REG_DETECTOR_STATUS_FIELD_SENSOR_CALIBRATE_OK_MASK |
DISTANCE_REG_DETECTOR_STATUS_FIELD_DETECTOR_CALIBRATE_OK_MASK;
if (status != config_ok_mask)
{
//ERROR
return false;
}
return true;
}
/**
* @brief Aguarde até que o detector não esteja ocupado
*
* @returns true if successful
*/
bool wait_not_busy_p(void)
{
uint32_t status = 0;
do
{
	 if (!read_register(LED_RADAR_P,I2C_ADDR_P,PRESENCE_REG_DETECTOR_STATUS_ADDRESS, &status))
	 {
	 //ERROR
	 return false;
	 }
	 } while((status & PRESENCE_REG_DETECTOR_STATUS_FIELD_BUSY_MASK) != 0);
	 return true;
	 }
bool wait_not_busy(void)
{
	 uint32_t status = 0;
	 do
	 {
	 if (!read_register(LED_RADAR_D,I2C_ADDR_D,DISTANCE_REG_DETECTOR_STATUS_ADDRESS, &status))
	 {
	 //ERROR
	 return false;
	 }
	 } while((status & DISTANCE_REG_DETECTOR_STATUS_FIELD_BUSY_MASK) != 0);
	 return true;
	 }
bool example_setup_and_start(void)
{
// Set start at 1000mm
if (!write_register(LED_RADAR_P,I2C_ADDR_P,PRESENCE_REG_START_ADDRESS, 200))
{
//ERROR
return false;
}
// Set end at 5000mm
if (!write_register(LED_RADAR_P,I2C_ADDR_P,PRESENCE_REG_END_ADDRESS, 1000))
{
//ERROR
return false;
}
// Apply configuration
if (!write_register(LED_RADAR_P,I2C_ADDR_P,
PRESENCE_REG_COMMAND_ADDRESS,
PRESENCE_REG_COMMAND_ENUM_APPLY_CONFIGURATION))
{
//ERROR
return false;
}
// Wait for the configuration to be done
if (!wait_not_busy_p())
{
//ERROR
return false;
}
// Test if configration of detector was OK
if (!configuration_ok_p())
{
//ERROR
return false;
}
// Start detector
if (!write_register(LED_RADAR_P,I2C_ADDR_P,PRESENCE_REG_COMMAND_ADDRESS,
PRESENCE_REG_COMMAND_ENUM_START_DETECTOR))
{
//ERROR
return false;
}
// Wait for command be done
if (!wait_not_busy_p())
{
	//ERROR
	 return false;
	 }
	 // Read detector result
	 uint32_t result;
	 if (!read_register(LED_RADAR_P,I2C_ADDR_P,PRESENCE_REG_PRESENCE_RESULT_ADDRESS, &result))
	 {
	 //ERROR
	 return false;
	 }
	 // Was presence detected?
	 bool presence_detected = (result &
	 PRESENCE_REG_PRESENCE_RESULT_FIELD_PRESENCE_DETECTED_MASK) != 0;
	 bool presence_detected_sticky = (result &
	 PRESENCE_REG_PRESENCE_RESULT_FIELD_PRESENCE_DETECTED_STICKY_MASK) !=
	 0;
	 // Print peak if found
	 if (presence_detected || presence_detected_sticky)
	 {
	 uint32_t presence_distance_mm;
	 if (read_register(LED_RADAR_P,I2C_ADDR_P,PRESENCE_REG_PRESENCE_DISTANCE_ADDRESS, &
	 presence_distance_mm))
	 {
	 printf("Presence detected at distance: %" PRIu32 " mm\n",
	 presence_distance_mm);
	 }
	 else
	 {
	 //ERROR
	 return false;
	 }
	 }
	 else
	 {
	 printf("No presence detected\n");
	 }
	 return true;
	 }
bool example_setup_and_measure(void)
	 {

	 if (!write_register(LED_RADAR_D,I2C_ADDR_D,DISTANCE_REG_START_ADDRESS, 200))
	 	 {
	 	 //ERROR
	 		 ESP_LOGE("XM125", "ERRO , distance_start"); //////
	 		return false;
	 	 }
	 if (!write_register(LED_RADAR_D,I2C_ADDR_D,DISTANCE_REG_END_ADDRESS, 1000))
	 	 	 {
	 	 	 //ERROR
	 	 		 ESP_LOGE("XM125", "ERRO , distance_end"); //////
	 	 		return false;
	 	 	 }
	 // Apply configuration
	 if (!write_register(LED_RADAR_D,I2C_ADDR_D,
	 DISTANCE_REG_COMMAND_ADDRESS,
	 DISTANCE_REG_COMMAND_ENUM_APPLY_CONFIG_AND_CALIBRATE))
	 {
	 //ERROR
		 ESP_LOGE("XM125", "ERRO , aplly");/////
		 return false;
	 }
	 // Wait for the configuration and calibration to be done
	// if (!wait_not_busy())
	// {
	//ERROR
		// ESP_LOGE("XM125", "ERRO , call");//////

	 //}
	 // Test if configration of detector was OK
	 if (!configuration_ok())
	 {
	 //ERROR
		 ESP_LOGE("XM125", "ERRO , conf");////////
		 return false;
	 }
	 // Measure
	 if (!write_register(LED_RADAR_D,I2C_ADDR_D,DISTANCE_REG_COMMAND_ADDRESS,
	 DISTANCE_REG_COMMAND_ENUM_MEASURE_DISTANCE))
	 {
	 //ERROR
		 ESP_LOGE("XM125", "ERRO , measure");//////
		 return false;
	 }
	 // Wait for measure distance to be done
	 if (!wait_not_busy())
	  {
	 //ERROR
		 ESP_LOGE("XM125", "ERRO , wait");///////

	  }
	  // Read detector result
	  uint32_t result;
	  if (!read_register(LED_RADAR_D,I2C_ADDR_D,DISTANCE_REG_DISTANCE_RESULT_ADDRESS, &result))
	  {
	  //ERROR
		  ESP_LOGE("XM125", "ERRO , result");//////
		  return false;
	  }
	  // Did we detect a peak?
	  uint32_t num_distances =
	  (result & DISTANCE_REG_DISTANCE_RESULT_FIELD_NUM_DISTANCES_MASK) >>
	  DISTANCE_REG_DISTANCE_RESULT_FIELD_NUM_DISTANCES_POS;
	  // Print peak if found
	  if (num_distances > 0)
	  {
	  uint32_t peak_distance_mm;
	  if (read_register(LED_RADAR_D,I2C_ADDR_D,DISTANCE_REG_PEAK0_DISTANCE_ADDRESS, &
	  peak_distance_mm))
	  {
	  printf("Peak distance: %" PRIu32 " mm\n", peak_distance_mm);
	  }
	  else
	  {
	  //ERROR
		  ESP_LOGE("XM125", "ERRO , exmaple radar");
	  return false;
	  }
	  }
	  else
	  {
	  printf("No peak detected\n");
	  }
	  ESP_LOGI("XM125", "OK , exmaple radar");
	  return true;
	  }


bool detectar_obj(uint32_t value) // retorna true ,caso em uma varedura de  8*NMAX = 80 values_presence[8 * NMAX_MED - 1] - values_presence[0] >= 800
{
  bool detectacao = false;

  if (values_presence[8 * NMAX_MED - 1] != 0)
  {
    for (int i = 0; i <= 8 * NMAX_MED - 2; i++)
    {
      values_presence[i] = values_presence[i + 1];
    }
    values_presence[8 * NMAX_MED - 1] = value;
  }
  else
  {
    for (int i = 0; i <= 8 * NMAX_MED - 1; i++)
    {
      if (values_presence[i] == 0)
      {
        values_presence[i] = value;
        detectacao = false;
        return detectacao;
      }
    }
  }

  if (((int)values_presence[8* NMAX_MED - 1] - (int)values_presence[0]) >= 800)
  {
    detectacao = true;
    medida_p = 1;
  }
  else
  {
    detectacao = false;
  }


  if ((medida_p == 1) && (medida < 750))
  {
    medida = medida + 1;
    detectacao = true;
  }
  if (medida >= 750)
  {
    medida_p = 0;
    medida = 0;
  }
  if (value > 2600)
  {
    medida = 0;
    medida_p = 1;
    detectacao = true;
  }
  return detectacao;
}

uint32_t media_dist(uint32_t value) // Calcula uma media dos NMAX_MED valores da distancia
{

  if (values[NMAX_MED] == 0)
  {
    soma = 0;

    for (uint32_t n = 1; n <= NMAX_MED; n++)
    {
      soma = soma + value;
      if (values[n] == 0)
      {

        values[n] = value;
        media = soma / n;
        n = NMAX_MED;
      }
    }
  }
  else
  {
    soma = 0;
    for (uint32_t n = 1; n <= NMAX_MED - 1; n++)
    {
      values[n] = values[n + 1];
    }
    values[NMAX_MED] = value;
    for (uint32_t n = 1; n <= NMAX_MED; n++)
    {

      soma = (soma + values[n]);
    }

    media = soma / NMAX_MED;
  }

  return media;
}

uint32_t media_peak(uint32_t value) // Calcula uma media dos NMAX_MED valores do pico de RSS
{

  if (values_p[NMAX_MED] == 0)
  {
    soma_p = 0;
    for (uint32_t n = 1; n <= NMAX_MED; n++)
    {
      soma_p = soma_p + value;
      if (values_p[n] == 0)
      {
        values_p[n] = value;
        media_p = soma_p / n;
        n = NMAX_MED;
      }
    }
  }
  else
  {
    soma_p = 0;
    for (uint32_t n = 1; n <= NMAX_MED - 1; n++)
    {
      values_p[n] = values_p[n + 1];
    }
    values_p[NMAX_MED] = value;
    for (uint32_t n = 1; n <= NMAX_MED; n++)
    {

      soma_p = (soma_p + values_p[n]);
    }
    media_p = soma_p / NMAX_MED;
  }

  return media_p;
}

uint32_t lineariza(uint32_t value) // Diminui o Ruido da variaçao da distancia em 5%
{
  uint32_t ideal = 0;

  if (((value % 10) > 0) && (value < limite) && (ideal != (uint32_t)value))
  {

    if (ideal == 0)
    {
      ideal = (uint32_t)(value - (value % 5));
    }

    if ((((int32_t)value - (int32_t)ideal) > margem))
    {
      ideal = (uint32_t)(value - (value % 5));
    }
    if (((int32_t)value - (int32_t)ideal) < (-1) * margem)
    {
      ideal = (uint32_t)(value - (value % 5));
    }
  }

  if (ideal == 0)
  {
    ideal = (uint32_t)value;
  }
  return ideal;
}

uint32_t lineariza_peak(uint32_t value) // Diminui o Ruido da variaçao da distancia em 5%
{
  uint32_t ideal = 0;

  if (((value % 10) > 0) && (ideal != (uint32_t)value))
  {

    if (ideal == 0)
    {
      ideal = (uint32_t)(value - (value % 5));
    }

    if ((((int32_t)value - (int32_t)ideal) > margem_peak))
    {
      ideal = (uint32_t)(value - (value % 5));
    }
    if (((int32_t)value - (int32_t)ideal) < (-1) * margem_peak)
    {
      ideal = (uint32_t)(value - (value % 5));
    }
  }

  if (ideal == 0)
  {
    ideal = (uint32_t)value;
  }
  return ideal;
}
