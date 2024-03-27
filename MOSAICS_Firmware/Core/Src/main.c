/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "usbd_cdc_if.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;

/* USER CODE BEGIN PV */
//The idea is to OR the WRITE_X command and the DAC_X command together to make one 8 bit packet, then shift it 8 bits to make the first
//16 bits of the data frame.

uint8_t WRITE_CODE_N              = 0b00000000;
uint8_t WRITE_CODE_ALL            = 0b10000000;
uint8_t WRITE_SPAN_N              = 0b01100000;
uint8_t WRITE_SPAN_ALL            = 0b11100000;
uint8_t WRITE_UPDATE_N            = 0b00010000;
uint8_t WRITE_UPDATE_ALL          = 0b10010000;
uint8_t WRITE_CODE_N_UPDATE_N     = 0b00110000;
uint8_t WRITE_CODE_N_UPDATE_ALL   = 0b00100000;
uint8_t WRITE_CODE_ALL_UPDATE_ALL = 0b10100000;
uint8_t WRITE_POWER_N             = 0b01000000;
uint8_t WRITE_POWER_CHIP          = 0b01010000;
uint8_t WRITE_MONITOR_MUX         = 0b10110000;
uint8_t WRITE_TOGGLE_SELECT       = 0b11000000;
uint8_t WRITE_GOLBAL_TOGGLE       = 0b11010000;
uint8_t WRITE_CONFIG_CMD          = 0b01110000;
uint8_t WRITE_NO_OP               = 0b11110000;

uint16_t DISBALE_PL = 0b0000000000000100;

uint8_t DAC_0 = 0b00000000;
uint8_t DAC_1 = 0b00000001;
uint8_t DAC_2 = 0b00000010;
uint8_t DAC_3 = 0b00000011;
uint8_t DAC_4 = 0b00000100;
uint8_t DAC_NO_OP = 0b00001111;

uint8_t dacs[5] = {0b00000000, 0b00000001, 0b00000010, 0b00000011, 0b00000100};

uint16_t SPAN_HIZ     = 0b0000000000000000;
uint16_t SPAN_003_125 = 0b0000000000000001;
uint16_t SPAN_006_250 = 0b0000000000000010;
uint16_t SPAN_012_500 = 0b0000000000000011;
uint16_t SPAN_025_000 = 0b0000000000000100;
uint16_t SPAN_050_000 = 0b0000000000000101;
uint16_t SPAN_100_000 = 0b0000000000000110;
uint16_t SPAN_200_000 = 0b0000000000000111;
uint16_t SPAN_300_000 = 0b0000000000001111;
uint16_t SPAN_V_MINUS = 0b0000000000001000;

uint16_t spans[8] = {0b0000000000000001, 0b0000000000000010, 0b0000000000000011, 0b0000000000000100, 0b0000000000000101, 0b0000000000000110, 0b0000000000000111, 0b0000000000001111};

uint16_t MUX_HIZ       = 0b0000000000000000;
uint16_t MUX_OUT0_CURR = 0b0000000000000001;
uint16_t MUX_OUT1_CURR = 0b0000000000000010;
uint16_t MUX_OUT2_CURR = 0b0000000000000011;
uint16_t MUX_OUT3_CURR = 0b0000000000000100;
uint16_t MUX_OUT4_CURR = 0b0000000000000101;
uint16_t MUX_VCC       = 0b0000000000000110;
uint16_t MUX_VREF      = 0b0000000000001000;
uint16_t MUX_VREFLO    = 0b0000000000001001;
uint16_t MUX_TEMP      = 0b0000000000001010;
uint16_t MUX_VDD0      = 0b0000000000010000;
uint16_t MUX_VDD1      = 0b0000000000010001;
uint16_t MUX_VDD2      = 0b0000000000010010;
uint16_t MUX_VDD3      = 0b0000000000010011;
uint16_t MUX_VDD4      = 0b0000000000010100;
uint16_t MUX_VPLUS     = 0b0000000000010101;
uint16_t MUX_VMINUS    = 0b0000000000010110;
uint16_t MUX_GND       = 0b0000000000010111;
uint16_t MUX_OUT0_VOLT = 0b0000000000011000;
uint16_t MUX_OUT1_VOLT = 0b0000000000011001;
uint16_t MUX_OUT2_VOLT = 0b0000000000011010;
uint16_t MUX_OUT3_VOLT = 0b0000000000011011;
uint16_t MUX_OUT4_VOLT = 0b0000000000011100;

uint16_t current_mux[5] = {0b0000000000000001, 0b0000000000000010, 0b0000000000000011, 0b0000000000000100, 0b0000000000000101};
uint16_t voltage_mux[5] = {0b0000000000011000, 0b0000000000011001, 0b0000000000011010, 0b0000000000011011, 0b0000000000011100};

uint8_t ADC_SAMPLES = 12;


double range_max[8] = {3.125, 6.25, 12.5, 25.0, 50.0, 100.0, 200.0, 300.0};

struct MOSAICS channels[50];

uint8_t shift_pos[50] = {6, 6, 6, 6, 6, 7, 7, 7, 7, 7, 5, 5, 5, 5, 5, 8, 8, 8, 8, 8, 4, 4, 4, 4, 4, 9, 9, 9, 9, 9, 3, 3, 3, 3, 3, 0, 0, 0, 0, 0, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1}; //the order that each channel appears in the SPI shift register in order from ch1 to ch50
uint8_t dac_pos[50]   = {4, 3, 2, 1, 0, 4, 3, 2, 1, 0, 4, 3, 2, 1, 0, 4, 3, 2, 1, 0, 4, 3, 2, 1, 0, 4, 3, 2, 1, 0, 4, 3, 2, 1, 0, 4, 3, 2, 1, 0, 4, 3, 2, 1, 0, 4, 3, 2, 1, 0}; //the index of the DAC on each LTC2662 that controls each channel in order from ch1 to ch50
uint8_t mux_codes[50] = {0x05, 0x05, 0x05, 0x05, 0x05, 0x07, 0x07, 0x07, 0x07, 0x07, 0x0C, 0x0C, 0x0C, 0x0C, 0x0C, 0x0D, 0x0D, 0x0D, 0x0D, 0x0D, 0x0A, 0x0A, 0x0A, 0x0A, 0x0A, 0x0B, 0x0B, 0x0B, 0x0B, 0x0B, 0x08, 0x08, 0x08, 0x08, 0x08, 0x09, 0x09, 0x09, 0x09, 0x09, 0x01, 0x01, 0x01, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00};
uint8_t mux_sel_pins[4] = {GPIO_PIN_2, GPIO_PIN_1, GPIO_PIN_0, GPIO_PIN_3};

uint8_t Buf[APP_RX_DATA_SIZE];
uint32_t Len;

uint8_t send_buf[APP_TX_DATA_SIZE];

uint16_t state = 0;

uint16_t ADC_Buffer[2];
int32_t ADC_Value;
double ADC_Convert;
double ADC_Avg;

uint8_t count = 0;
uint8_t curr_char;
uint16_t bit_stream[20];
uint16_t tmp_cmd;
uint16_t tmp_value;

uint16_t no_op_code;
uint16_t off_code;

uint8_t type[MAX_CMD_SIZE];
uint8_t command[MAX_CMD_SIZE];
uint8_t channel[MAX_CMD_SIZE];
uint8_t value[MAX_CMD_SIZE];
uint8_t channel_int;
uint16_t value_int;
double current_float;

uint8_t type_set[MAX_CMD_SIZE] = "SET";
uint8_t type_get[MAX_CMD_SIZE] = "GET";
uint8_t command_range[MAX_CMD_SIZE] = "RANGE";
uint8_t command_current[MAX_CMD_SIZE] = "CURRENT";
uint8_t command_voltage[MAX_CMD_SIZE] = "VOLTAGE";
uint8_t command_reference[MAX_CMD_SIZE] = "REFERENCE";
const uint32_t baudrate = 115200;
uint8_t USB_CONFIG[7] = {(uint8_t)baudrate>>24, (uint8_t)baudrate>>16, (uint8_t)baudrate>>8, (uint8_t)baudrate, 0x00, 0x00, 0x08};

uint8_t tmp_mux_code;

uint8_t debug = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

uint8_t Extract_Data(uint8_t *Buf, uint8_t *Data, uint32_t Start){
	uint8_t count = 0;
	uint8_t temp[MAX_CMD_SIZE];
	while ((count <= MAX_CMD_SIZE) && (Buf[Start + count] != ':')){
		temp[count] = Buf[Start + count];
		count++;
	}
	if (count > 0){
		memcpy(Data, temp, count);
		return count + Start;
	}
	return 0;
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

	for (uint8_t i = 0; i < 50; i++){ //init struct with channel data
		channels[i].range = range_max[0];
		channels[i].value = 0.0;
		channels[i].shift = shift_pos[i];
		channels[i].dac = dac_pos[i];
		channels[i].mux_code = mux_codes[i];

	}

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  MX_USB_Device_Init();
  /* USER CODE BEGIN 2 */

  vcp_init();
  //RESET ALL OUTPUTS TO 0

  tmp_cmd = 0x0000 | (WRITE_CODE_ALL_UPDATE_ALL | DAC_0);
  tmp_value = 0x0000;
  for (uint8_t i = 0; i < 10; i++){
	  bit_stream[i*2] = tmp_cmd;
	  bit_stream[(i*2) + 1] = tmp_value;
  }
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
  HAL_SPI_Transmit(&hspi1, bit_stream, 20, 100);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);


  tmp_cmd = 0x0000 | (WRITE_SPAN_ALL | DAC_0);
  tmp_value = 0x0000 | (SPAN_HIZ);
  for (uint8_t i = 0; i < 10; i++){
	  bit_stream[i*2] = tmp_cmd;
	  bit_stream[(i*2) + 1] = tmp_value;
  }
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
  HAL_SPI_Transmit(&hspi1, bit_stream, 20, 100);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

  tmp_cmd = 0x0000 | (WRITE_CONFIG_CMD | DAC_0);
  tmp_value = 0x0000 | (DISBALE_PL);
  for (uint8_t i = 0; i < 10; i++){
  	  bit_stream[i*2] = tmp_cmd;
  	  bit_stream[(i*2) + 1] = tmp_value;
    }
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi1, bit_stream, 20, 100);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);



  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  //HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_5);
	  switch (state){
	  	  case 0: //idle state
	  		  if (Recv_Data(Buf, &Len)){
	  			  memset(type, 0, sizeof type);
	  			  memset(command, 0, sizeof command);
	  			  memset(channel, 0, sizeof channel);
	  			  memset(value, 0, sizeof value);
	  			  state = 1;
	  		  }
	  		  break;
	  	  case 1: //check header
	  		  if (Buf[0] == ':'){
	  			  state = 2;
	  		  }
	  		  else {
	  			  state = 0; //ERROR
	  		  }
	  		  break;
	  	  case 2: //extract and compare first command
	  		  count = Extract_Data(Buf, type, 1);
	  		  if (count){ //SET
	  			  if (!(strcmp(type, type_set))){
	  				  //strcpy(send_buf, "SETTING RANGE\n");
	  				  state = 3;
	  			  }
	  			  else if (!(strcmp(type, type_get))){
	  				  state = 10;
	  			  }
	  			  else {
	  				  state = 0;
	  			  }

	  		  }
	  		  else {
	  			  state = 0;
	  		  }
	  		  break;
	  	  case 3:
	  		  count = Extract_Data(Buf, command, count + 1);
	  		  if (count){
	  			  if (!(strcmp(command, command_range))){ //RANGE
	  				  state = 4;
	  			  }
	  			  else if (!strcmp(command, command_current)){ //CURRENT
	  				  state = 5;
	  			  }
	  			  else {
	  				  state = 0;
	  			  }
	  		  }
	  		  else {
	  			  state = 0;
	  		  }
	  		  break;
	  	  case 4: //GET RANGE CHANNEL
	  		  count = Extract_Data(Buf, channel, count + 1);
			  if (count){
				  channel_int = atoi(channel);
				  if ((channel_int >= 1) && (channel_int <= 50)){
					  HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_5);
					  state = 7;
				  }
				  else {
					  state = 0;
				  }
			  }
			  else {
				  state = 0;
			  }
			  break;
	  	  case 5: //GET CURRENT CHANNEL
	  		  count = Extract_Data(Buf, channel, count + 1);
			  if (count){
				  channel_int = atoi(channel);
				  if ((channel_int >= 1) && (channel_int <= 50)){
					  HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_5);
					  state = 6;
				  }
				  else {
					  state = 0;
				  }
			  }
			  else {
				  state = 0;
			  }
			  break;
	  	  case 6: //SET CURRENT
	  		  count = Extract_Data(Buf, value, count + 1);
	  		  if (count){
	  			  current_float = atof(value);
	  			  if (current_float <= channels[channel_int - 1].range){
	  				  sprintf(send_buf, "CH: %d, C: %fmA\n", channel_int, current_float);
	  				  while(vcp_send_alt(send_buf, strlen(send_buf)) == -1){}
	  				  channels[channel_int - 1].value = current_float;
	  				  state = 9;
	  			  }
	  			  else {
	  				  sprintf(send_buf, "ERROR %fmA is out of range for channel %d (max is: %fmA)\n", current_float, channel_int, channels[channel_int - 1].range);
	  				  while(vcp_send_alt(send_buf, strlen(send_buf)) == -1){}
	  				  state = 0;
	  			  }

	  		  }
	  		  else {
	  			  state = 0;
	  		  }
	  		  break;
	  	  case 7: //SET RANGE
	  		  count  = Extract_Data(Buf, value, count + 1);
	  		  if (count){
	  			  value_int = atoi(value);
	  			  if ((value_int >= 1) && (value_int <= 8)){
					  sprintf(send_buf, "CH: %d, R: %fmA\n", channel_int, range_max[value_int - 1]);
					  while(vcp_send_alt(send_buf, strlen(send_buf)) == -1){}
					  channels[channel_int - 1].range = range_max[value_int - 1];
					  state = 8;
	  			  }
	  			  else {
	  				  sprintf(send_buf, "ERROR %d is an invalid range (1-8 are valid)\n", value_int);
	  				  while(vcp_send_alt(send_buf, strlen(send_buf)) == -1){};
	  				  state = 0;
	  			  }
	  		  }
	  		  else {
	  			state = 0;
	  		  }
	  		  break;
	  	  case 8://SEND Bit Stream Range
	  		  tmp_cmd = 0x0000 | (WRITE_SPAN_N | dacs[channels[channel_int - 1].dac]);
	  		  tmp_value = 0x0000 | (spans[value_int - 1]);
	  		  no_op_code = 0x0000 | (WRITE_NO_OP | DAC_4);
	  		  off_code = 0x0000;
	  		  if (debug){
	  			  sprintf(send_buf, "sending %d, %d\n", tmp_cmd, tmp_value);
	  			  while(vcp_send_alt(send_buf, strlen(send_buf)) == -1){}
	  		  }

		      for (uint8_t i = 0; i < 10; i++){
		    	  if (i == channels[channel_int - 1].shift){
		    		  bit_stream[i*2] = tmp_cmd;
		    		  bit_stream[(i*2) + 1] = tmp_value;
		    	  }
		    	  else {
		    		  bit_stream[i*2] = no_op_code;
		    		  bit_stream[(i*2) + 1] = off_code;
		    	  }
		      }
		      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
		      HAL_SPI_Transmit(&hspi1, bit_stream, 20, 100);
		      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
//
	  		  state = 0;
	  		  break;
	  	  case 9: //Send Bit Stream Current
	  		  tmp_cmd = 0x0000 | (WRITE_CODE_N_UPDATE_N | dacs[channels[channel_int - 1].dac]);
	  		  tmp_value = 0xFFFF * (current_float/channels[channel_int - 1].range);
	  		  //tmp_value = 0xFFFF;
	  		  no_op_code = 0x0000 | (WRITE_NO_OP | DAC_4);
	  		  off_code = 0x0000;
	  		  if (debug){
	  			  sprintf(send_buf, "sending %d, %d\n", tmp_cmd, tmp_value);
	  			  while(vcp_send_alt(send_buf, strlen(send_buf)) == -1){}
	  		  }

		      for (uint8_t i = 0; i < 10; i++){
		    	  if (i == channels[channel_int - 1].shift){
		    		  bit_stream[i*2] = tmp_cmd;
		    		  bit_stream[(i*2) + 1] = tmp_value;
		    	  }
		    	  else {
		    		  bit_stream[i*2] = no_op_code;
		    		  bit_stream[(i*2) + 1] = off_code;
		    	  }
		      }
		      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
		      HAL_SPI_Transmit(&hspi1, bit_stream, 20, 100);
		      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

	  		  state = 0;
	  		  break;
	  	  case 10: //GET
	  		  count = Extract_Data(Buf, command, count + 1);
	  		  if (count){
	  			  if (!(strcmp(command, command_voltage))){
	  				  state = 11;
	  			  }
	  			  else if (!(strcmp(command, command_current))){
	  				  state = 12;
	  			  }
	  			  else if (!(strcmp(command, command_reference))){
	  				  state = 13;
	  			  }
	  			  else {
	  				  state = 0;
	  			  }
	  		  }
	  		  else {
	  			  state = 0;
	  		  }
	  		  break;
	  	  case 11: //send bitstream for get voltage
	  		  count = Extract_Data(Buf, channel, count + 1);
			  if (count){
				  channel_int = atoi(channel);
				  if ((channel_int >= 1) && (channel_int <= 50)){

				  }
				  else {
					  state = 0;
					  break;
				  }
			  }
			  else {
				  state = 0;
				  break;
			  }
	  		  tmp_cmd = 0x0000 | (WRITE_MONITOR_MUX);
	  		  tmp_value = 0x0000 | (voltage_mux[channels[channel_int - 1].dac]);
	  		  no_op_code = 0x0000 | (WRITE_NO_OP | DAC_4);
	  		  off_code = 0x0000;
	  		  tmp_mux_code = channels[channel_int - 1].mux_code;
	  		  for (uint8_t i = 0; i < 4; i++){
	  			  HAL_GPIO_WritePin(GPIOA, mux_sel_pins[i], ((0x01 & tmp_mux_code >> i) ? GPIO_PIN_SET : GPIO_PIN_RESET));
	  		  }
	  		  if (debug){
	  			 sprintf(send_buf, "sending %d, %d\n", tmp_cmd, tmp_value);
			     while(vcp_send_alt(send_buf, strlen(send_buf)) == -1){}
	  		  }

			  for (uint8_t i = 0; i < 10; i++){
				  if (i == channels[channel_int - 1].shift){
					  bit_stream[i*2] = tmp_cmd;
					  bit_stream[(i*2) + 1] = tmp_value;
				  }
				  else {
					  bit_stream[i*2] = no_op_code;
					  bit_stream[(i*2) + 1] = off_code;
				  }
			  }
			  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
			  HAL_SPI_Transmit(&hspi1, bit_stream, 20, 100);
			  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
			  for (uint8_t samples = 0; samples < ADC_SAMPLES; samples++){
				  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET); // Set CNV Pin high to init conversion.
				  while(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_15) == GPIO_PIN_SET){

				  }
				  HAL_SPI_Receive(&hspi2, (uint8_t *)ADC_Buffer, 2, 1000);
				  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
				  ADC_Value = ((uint32_t)ADC_Buffer[0]) << 9 | ((uint32_t)ADC_Buffer[1]);
				  if ((ADC_Value & 0b100000000000000000) >> 17 == 1){ //the value is negative
					  ADC_Value = (ADC_Value & 0b011111111111111111) - 0b100000000000000000;
				  }
				  ADC_Convert += (ADC_Value * 0.000154);
			  }
			  ADC_Avg = ADC_Convert/ADC_SAMPLES;
//			  ADC_Avg = (1.04*ADC_Avg) + 0.0198;
			  ADC_Convert = 0;
			  sprintf(send_buf, "CH: %d, V: %f\n", channel_int, ADC_Avg);
			  while(vcp_send_alt(send_buf, strlen(send_buf)) == -1){}
			  state = 0;
	  		  break;
	  	  case 12://send bitstream for get current.
		      count = Extract_Data(Buf, channel, count + 1);
			  if (count){
				  channel_int = atoi(channel);
				  if ((channel_int >= 1) && (channel_int <= 50)){

				  }
				  else {
					  state = 0;
					  break;
				  }
			  }
			  else {
				  state = 0;
				  break;
			  }
	  		  tmp_cmd = 0x0000 | (WRITE_MONITOR_MUX);
	  		  tmp_value = 0x0000 | (current_mux[channels[channel_int - 1].dac]);
	  		  no_op_code = 0x0000 | (WRITE_NO_OP | DAC_4);
	  		  off_code = 0x0000;
	  		  state = 0;
	  		  break;
	  	  case 13:
	  		  count = Extract_Data(Buf, channel, count + 1);
			  if (count){
				  channel_int = atoi(channel);
				  if ((channel_int >= 1) && (channel_int <= 50)){

				  }
				  else {
					  state = 0;
					  break;
				  }
			  }
			  else {
				  state = 0;
				  break;
			  }
	  		  tmp_cmd = 0x0000 | (WRITE_MONITOR_MUX);
		      tmp_value = 0x0000 | (MUX_VREF);
		      no_op_code = 0x0000 | (WRITE_NO_OP | DAC_4);
		      off_code = 0x0000;
		      HAL_Delay(1);
			  sprintf(send_buf, "sending %d, %d\n", tmp_cmd, tmp_value);
			  while(vcp_send_alt(send_buf, strlen(send_buf)) == -1){}
			  for (uint8_t i = 0; i < 10; i++){
				  if (i == channels[channel_int - 1].shift){
					  bit_stream[i*2] = tmp_cmd;
					  bit_stream[(i*2) + 1] = tmp_value;
				  }
				  else {
					  bit_stream[i*2] = no_op_code;
					  bit_stream[(i*2) + 1] = off_code;
				  }
			  }
			  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
			  HAL_SPI_Transmit(&hspi1, bit_stream, 20, 100);
			  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
			  state = 0;
	  		  break;
	  }
//	  if (Recv_Data(Buf, &Len)){
//		  if(Buf[Len-1] == ':'){
//			  HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_5);
//		  }
//	  }

//	  uint16_t span_code   = 0x0000 | (WRITE_SPAN_ALL | DAC_NO_OP);
//	  uint16_t update_code = 0x0000 | (WRITE_CODE_ALL_UPDATE_ALL | DAC_NO_OP);
//	  uint16_t no_op_code = 0x0000 | (WRITE_NO_OP | DAC_4);
//	  uint16_t mux_code = 0x0000 | (WRITE_MONITOR_MUX | DAC_0);
//	  uint16_t on_code_1 = 0x7FFF;
//	  uint16_t on_code_2 = 0x6969;
//	  uint16_t off_code = 0x0000;
//
//	  uint16_t set_shift_reg_span[20];
//	  uint16_t set_shift_reg_value[20];
//	  uint16_t set_mux_code[20];
//	  for (uint8_t i = 0; i < 20; i = i + 2){
//		  set_shift_reg_span[i] = span_code;
//		  set_shift_reg_span[i+1] = SPAN_V_MINUS;
//	  }
//
//	  for (uint8_t i = 0; i < 20; i = i + 2){
//		  set_shift_reg_value[i] = no_op_code;
//		  set_shift_reg_value[i+1] = off_code;
//	  }
//
//	  for (uint8_t i = 0; i < 20; i = i + 2){
//		  set_mux_code[i] = mux_code;
//		  set_mux_code[i+1] = MUX_OUT4_CURR;
//	  }
//
//
//
//	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
//	  for (uint8_t i = 0; i < 10; i++){
//		  HAL_SPI_Transmit(&hspi1, (uint8_t*)&span_code, 1, 100);
//		  HAL_SPI_Transmit(&hspi1, (uint8_t*)&SPAN_200_000, 1, 100);
//	  }
//	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
//
//	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
//	  for (uint8_t i = 0; i < 10; i++){
//		  HAL_SPI_Transmit(&hspi1, (uint8_t*)&update_code, 1, 100);
//		  HAL_SPI_Transmit(&hspi1, (uint8_t*)&on_code_1, 1, 100);
//	  }
//	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

//	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);
//	  HAL_SPI_Transmit(&hspi1, (uint8_t*)set_shift_reg_span, 20, 100);
//	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);
//
//	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);
//	  HAL_SPI_Transmit(&hspi1, (uint8_t*)set_shift_reg_value, 20, 100);
//	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);
//
//	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);
//	  HAL_SPI_Transmit(&hspi1, (uint8_t*)set_mux_code, 20, 100);
//	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_CRSInitTypeDef pInit = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI48;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV2;
  RCC_OscInitStruct.PLL.PLLN = 25;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable the SYSCFG APB clock
  */
  __HAL_RCC_CRS_CLK_ENABLE();

  /** Configures CRS
  */
  pInit.Prescaler = RCC_CRS_SYNC_DIV1;
  pInit.Source = RCC_CRS_SYNC_SOURCE_GPIO;
  pInit.Polarity = RCC_CRS_SYNC_POLARITY_RISING;
  pInit.ReloadValue = __HAL_RCC_CRS_RELOADVALUE_CALCULATE(48000000,1);
  pInit.ErrorLimitValue = 34;
  pInit.HSI48CalibrationValue = 32;

  HAL_RCCEx_CRSConfig(&pInit);
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_16BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES_RXONLY;
  hspi2.Init.DataSize = SPI_DATASIZE_9BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 7;
  hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, MUX_SEL0_Pin|MUX_SEL1_Pin|MUX_SEL2_Pin|MUX_SEL3_Pin
                          |CS_LD_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, ADC_CNV_Pin|LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : MUX_SEL0_Pin MUX_SEL1_Pin MUX_SEL2_Pin MUX_SEL3_Pin
                           CS_LD_Pin */
  GPIO_InitStruct.Pin = MUX_SEL0_Pin|MUX_SEL1_Pin|MUX_SEL2_Pin|MUX_SEL3_Pin
                          |CS_LD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : ADC_CNV_Pin LED_Pin */
  GPIO_InitStruct.Pin = ADC_CNV_Pin|LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : ADC_BUSY_Pin */
  GPIO_InitStruct.Pin = ADC_BUSY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ADC_BUSY_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PA10 */
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF3_UCPD1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

//This function finds the information between two ":" chars given the data buffer and a start point
//Return 1 if data was found, 0 if else.


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
