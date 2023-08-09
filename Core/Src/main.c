/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "BME280_STM32.h" //bme280

#include "fatfs_sd.h"
#include <stdio.h>   //Sprintf icin gerekli(SD CARD)
#include "string.h"  //Sprintf icin gerekli(SD CARD)

#include "lwgps/lwgps.h" //gps için

#include "max30102_for_stm32_hal.h" //oksimetre

#include "moving_average.h" //filtre

#include "fonts.h"  //lcd
#include "ssd1306.h"//lcd
#include "test.h"//lcd
///////////////////////////////
#include <math.h>  // gercek zamanli alinan yol hesabi icin
///////////////////////////////
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
///////////////  Konum Hesabi(Alinan yol)    ////////////////////
#define PI 3.14159265358979323846 //pi sayısı
#define RADIO_TERRESTRE 6372797.56085 //Dünyanın yarıçapı
#define GRADOS_RADIANES PI / 180 //Radyan çevirme
/////////////////////////////////
float spo2=0;
uint8_t lcd_spo2=0; //LCD de gostermek icin

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;
I2C_HandleTypeDef hi2c3;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim5;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/////////////////////////
////////////////////////////// Alinan Yol /////////////////////////
float distance = 0.0;
float distance_old = 0.0;
float distance_filtreli=0.0;
///////////////////////////////////
float v_filtresiz = 0.0;
float v_filtreli_birinci_adim = 0.0;
float v_filtreli_ikinci_adim = 0.0;

float v_filtreli_ucuncu_adim_yeni = 0.0;
float v_filtreli_ucuncu_adim_eski = 0.0;
FilterTypeDef filterStruct;
FilterTypeDef filterStruct2;
FilterTypeDef filterStruct3;
////////////////////////////////////
float ivmeyeni = 0.0;
float ivmeeski = 0.0;

float ivmefiltered = 0.0;
////////////////////////////////////
float enlem_baslangic = 0.0;
float boylam_baslangic = 0.0;
///////////////////////////  Nabiz   ///////////////////////////////
int nabiz_bpm=0;
int nabiz_filtered=0; //surekli önceki adımdaki bpm ile ortalama alır
int nabiz_filtered2=0;
int nabiz_filtered3=0;

FilterTypeDef filterStructbpm; //Uygulanacak filtreye ait structure
FilterTypeDef filterStructbpm2;
FilterTypeDef filterStructbpm3;

uint32_t nabiz_gecen_sure = 0;
uint32_t nabiz_eski = 0;
uint32_t nabiz_yeni = 0;

////////////////////////////// Button 1//////////////////////
uint32_t previousMillis12 = 0;
uint32_t currentMillis12 = 0;
////////////////////////////// Button 2 ////////////////
uint32_t previousMillis14 = 0;
uint32_t currentMillis14 = 0;
///////////////////////////// Yazma ////////////////////
int yazma_durum = 0;  // LCD int istiyor
uint16_t nabiz_ornekleme_periyot = 0; //kullanılmıyor
uint16_t kosu_ornekleme_periyot = 0;
/////////////////////////////  Koşu Sayaci //////////////
uint32_t kosu_suresi_sayac=0;
////////////////////////////////////////////////////////
float deniz_seviyesi_sure=0;
float ort_hiz=0;
float max_hiz=0;
uint32_t max_hiza_cikma_suresi=0;
int max_hiz_nabiz=0;
int ort_nabiz=0;

float max_hiz_VS=0;
////////////////////////////Kilo-Yas////////////////////
//float kilo=100;  //LCD DE DUZGUN GOSTERMEK ICIN FLOAT TANIMLI
//float yas=0;     //

float kilo=90;  //Gecici olarak,koydum final halinde buttondan al
float yas=24;     //
////////////////////////// Button 1-2 /////////////////
uint8_t button1=0;
uint8_t button2=0;
/////////////////////////////
//// printf() function
//int __io_putchar(int ch)
//{
//  uint8_t temp = ch;
//  HAL_UART_Transmit(&huart1, &temp, 1, HAL_MAX_DELAY);
//  return ch;
//}
/////////////////////////////////////////////////////////

// Plot fonksiyonu override
void max30102_plot(uint32_t ir_sample, uint32_t red_sample)
{  //Eğer oksijen saturasyonu %100 den küçükse,değişkene ata
	if(100 *  ((float)red_sample / (float)(ir_sample))<100)
	{
    spo2 = 100 *  ((float)red_sample / (float)(ir_sample));
	}
}

//max30102_t nesnesi bildirilir

max30102_t max30102;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C2_Init(void);
static void MX_TIM5_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C3_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

////////////////////////////////////////// Alinan Yol Hesaplama Formulu
float CalcGPSDistance(float latitud1, float longitud1, float latitud2, float longitud2);
float min(float a ,float b);   // Min fonksiyonu stm kütüphanelerinde yok
/////////////////////////////////////////

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// GPS //////////////////////////////////////////////////////////
lwgps_t gps;

uint8_t rx_buffer[128];
uint8_t rx_index = 0;
uint8_t rx_data = 0;

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart == &huart2) //Eger interrupt UART2 den geldiyse
	{
		if(rx_data != '\n' && rx_index < sizeof(rx_buffer))
			{
			rx_buffer[rx_index++] = rx_data; // Buffera doldurma
			}
		else
			{
			lwgps_process(&gps, rx_buffer, rx_index+1); //Parse etme
			rx_index = 0;
			rx_data = 0;
			}
		HAL_UART_Receive_IT(&huart2, &rx_data, 1);//Yeniden
	}                                           //receive enable etme
}

// SD CARD ////////////////////////////////
int indx=0;
char yazi[200];
char dosya_adi[50];
///////////////////// LCD //////////////////////////////////////
char lcd_yazi[50];
////////////////////   BME280     //////////////////////////////
float Temperature, Pressure, Humidity;
//////////////////////   DS18B20          ////////////////////
float Temperature_DS;//Nihai ölçüm

uint8_t lcd_Temperature_DS=0; //LCD'de gösterebilmek için
uint8_t Temp_byte1, Temp_byte2;//geçici değerler
uint16_t TEMP;//geçici değerler
/////////////////////// DS18B20 MicroDelay     ///////////////////////////////
void microDelay (uint16_t delay)  //Mikrosaniye Delay Fonksiyonu
{
  __HAL_TIM_SET_COUNTER(&htim5, 0); //Counter sıfırlama
  while (__HAL_TIM_GET_COUNTER(&htim5) < delay);  //DS18B20 icin timer source(tim5)
}
// ONE WIRE PIN OUTPUT-INPUT AYARLAMA   DS18B20 ///////////////////////////////////////////
void Set_Pin_Output (GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = GPIO_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

void Set_Pin_Input (GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = GPIO_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

///////////////////////// DS18B20 Fonksiyonlar  ////////////////////////////////////////
#define DS18B20_PORT GPIOA
#define DS18B20_PIN GPIO_PIN_1

uint8_t DS18B20_Start (void)
{
	uint8_t Response = 0;
	Set_Pin_Output(DS18B20_PORT, DS18B20_PIN);   // Pin output ayarlanır
	HAL_GPIO_WritePin (DS18B20_PORT, DS18B20_PIN, 0);  // Pin low a çekilir
	microDelay(480);   // Data sheet'e uygun olarak delay

	Set_Pin_Input(DS18B20_PORT, DS18B20_PIN);    // Pin input ayarlanır
	microDelay(80);    // Data sheet'e uygun olarak delay

	if (!(HAL_GPIO_ReadPin (DS18B20_PORT, DS18B20_PIN))) Response = 1;    // pin low ise sorgusu
	else Response = -1;

	microDelay(480); // // Data sheet'e uygun olarak delay

	return Response;
}

void DS18B20_Write (uint8_t data)
{
	Set_Pin_Output(DS18B20_PORT, DS18B20_PIN);  // Pin output ayarlanır

	for (int i=0; i<8; i++)
	{

		if ((data & (1<<i))!=0)  // Eğer bit H ise
		{
			// 1 yazilir

			Set_Pin_Output(DS18B20_PORT, DS18B20_PIN);  // Pin Output ayarlanır
			HAL_GPIO_WritePin (DS18B20_PORT, DS18B20_PIN, 0);  // Pin low a çekilir
			microDelay(1);  // 1 mikrosaniye beklenir

			Set_Pin_Input(DS18B20_PORT, DS18B20_PIN);  // Pin input olarak ayarlanır
			microDelay(50);  // 50-60 mikrosaniye beklenir
		}

		else  // Bit L ise
		{
			// 0 yazılır

			Set_Pin_Output(DS18B20_PORT, DS18B20_PIN);	// Pin Output ayarlanır
			HAL_GPIO_WritePin (DS18B20_PORT, DS18B20_PIN, 0);  // Pin low'a çekilir
			microDelay(50);  // 50-60 mikrosaniye beklenir.

			Set_Pin_Input(DS18B20_PORT, DS18B20_PIN);
		}
	}
}

uint8_t DS18B20_Read (void)
{
	uint8_t value=0;

	Set_Pin_Input(DS18B20_PORT, DS18B20_PIN);

	for (int i=0;i<8;i++)
	{
		Set_Pin_Output(DS18B20_PORT, DS18B20_PIN);   // Output olarak ayarlanır

		HAL_GPIO_WritePin (DS18B20_PORT, DS18B20_PIN, 0);  // Pini lowa çekilir
		microDelay(1);  // wait for > 1us

		Set_Pin_Input(DS18B20_PORT, DS18B20_PIN);  // Pin input olarak ayarlanir
		if (HAL_GPIO_ReadPin (DS18B20_PORT, DS18B20_PIN))  // Eğer pin H ise
		{
			value |= 1<<i;  // read = 1
		}
		microDelay(50);  // 50-60 mikrosaniye beklenir.
	}
	return value;
}
///  FATFS   /////////////////////////////////////////////

FATFS fs;  // file system
FIL fil; // File
FILINFO fno;
FRESULT fresult;  // result
UINT br, bw;  // File read/write count

/**** Kapasite Icin *****/
FATFS *pfs;
DWORD fre_clust;
uint32_t total, free_space;



/////////////////////////////////////////////////////////////////////////////////////////////////////////////



/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

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
  MX_I2C2_Init();
  MX_TIM5_Init();
  MX_SPI1_Init();
  MX_FATFS_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_I2C3_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
/////////////////////  OLED EKRAN ///////////////////////////////////
SSD1306_Init();  // initialise
////// DS18B20 /////////////////////////////////
  HAL_TIM_Base_Start(&htim5);
  ///////////////////////////// BME280 ////////////////////////////////
  BME280_Config(OSRS_2, OSRS_16, OSRS_1, MODE_NORMAL, T_SB_0p5, IIR_16);
  ///////////////////////////////////////////////////////////////////
  HAL_Delay (500);
  ////////////////////////////////////////////////////////////////////////////////////////////////////
  HAL_TIM_Base_Start_IT(&htim2);
  ///  GNSS  ///////////////////////////////////////////////////////////////////////////////////////////////
  lwgps_init(&gps);
  HAL_UART_Receive_IT(&huart2, &rx_data, 1);
  ///////////////////////////////////////////////////////
  // Init
     max30102_init(&max30102, &hi2c1);

     max30102_reset(&max30102);  //Sensör sifirlanir
     max30102_clear_fifo(&max30102);

     max30102_set_fifo_config(&max30102, max30102_smp_ave_8, 1, 7); //fifo ayarları yapılır

     // Sensor ayarları
     max30102_set_led_pulse_width(&max30102, max30102_pw_16_bit);
     max30102_set_adc_resolution(&max30102, max30102_adc_2048);
     max30102_set_sampling_rate(&max30102, max30102_sr_800);
     max30102_set_led_current_1(&max30102, 6.2);
     max30102_set_led_current_2(&max30102, 6.2);


     max30102_set_mode(&max30102, max30102_spo2);// SpO2 moduna giriş
     max30102_set_a_full(&max30102, 1);//FIFO_A_FULL kesintisini etkinleştirilir

     max30102_set_die_temp_en(&max30102, 1);//Çip sıcaklığı ölçümünü etkinleştirilir
     max30102_set_die_temp_rdy(&max30102, 1);//DIE_TEMP_RDY kesmesini etkinleştirilir

     uint8_t en_reg[2] = {0};
     max30102_read(&max30102, 0x00, en_reg, 1);
//////////////////////////////////////////////////////////
////   LCD Acilis
//
//SSD1306_GotoXY (15,15);
//SSD1306_Puts ("Merhaba;", &Font_7x10, 1);
//SD1306_GotoXY (25, 30);
//SSD1306_Puts ("Data Logger'a", &Font_7x10, 1);
//SSD1306_GotoXY (15, 45);
//SSD1306_Puts ("Hosgeldiniz :)", &Font_7x10, 1);
//SSD1306_UpdateScreen(); //display
//
//HAL_Delay (2000);
//
//SSD1306_InvertDisplay(1);   // invert the display
//
//HAL_Delay(1000);
//
//SSD1306_InvertDisplay(0);  // normalize the display
//
//HAL_Delay(1000);
//
//SSD1306_Clear();
/////////////////////////////////////////////////// kilo,yas alma
//     while(button2!=1)
//     {
//
//    	 if(button1==1)
//    	 {
//    		kilo=kilo-1;
//    		button1=0;
//    	 }
//         ////
//    	 if(kilo<0)
//    	 {
//    		 kilo=100;   //Koşucularda 90 kg üstü agir kabul edilir.
//    	 }
//    	 SSD1306_GotoXY (15,15);
//    	 SSD1306_Puts ("Kilo Giriniz: kg", &Font_7x10, 1);
//    	 SSD1306_GotoXY (15, 30);
//    	 sprintf (lcd_yazi,"%.1f",kilo);
//    	 SSD1306_Puts (lcd_yazi, &Font_7x10, 1);
//    	 SSD1306_UpdateScreen(); //display
//    	 if(button2==1)   //Onay buttonu gelirse cikacak(break)
//    	 {
//    		 button2=0;
//    		 break;
//    	 }
//
//     }
//  // Program baslamadan button(exti) degiskenlerini sıfırlama
//     SSD1306_Clear();
//     ////////////////////////////////////////////////
//     HAL_Delay(1500);   //button2(onay buttonu) direk onaylayıp geçmesin
//     ///////////////////////////////////////////////
//     button1=0;
//     button2=0;
//     yazma_durum=0;
//     kosu_suresi_sayac=0;
//     //////////////////////////////////////////
//     while(button2!=1)
//          {
//
//         	 if(button1==1)
//         	 {
//         		yas++;
//         		button1=0;
//         	 }
//              ////
//         	 if(yas>100)
//         	 {
//         		 yas=0;
//         	 }
//         	 SSD1306_GotoXY (15,15);
//         	 SSD1306_Puts ("Yas Giriniz:", &Font_7x10, 1);
//         	 SSD1306_GotoXY (15, 30);
//         	 sprintf (lcd_yazi,"%.1f",yas);
//         	 SSD1306_Puts (lcd_yazi, &Font_7x10, 1);
//         	 SSD1306_UpdateScreen(); //display
//         	 if(button2==1)  // Onay buttonu gelirse cikacak(break)
//         	 {
//         		 button2=0;
//         		 break;
//         	 }
//
//          }
// Program baslamadan button(exti) degiskenlerini

       	 SSD1306_Clear();               //bir kere sıfırla
     	 HAL_Delay(1500);          //button2:   yazma_durum 3 almasın direkt diye  BU SATIRI MUTLAKA AC !!!!! Yoksa buga girer
     	 button1=0;
     	 button2=0;
     	 yazma_durum=0;
     	 kosu_ornekleme_periyot=0;
     	 kosu_suresi_sayac=0;
         //////////////////////////////////////////////
         Moving_Average_Init(&filterStructbpm); // bpm filtre degerleri temizleme
         Moving_Average_Init(&filterStructbpm2);
         Moving_Average_Init(&filterStructbpm3);
         ///////////// BME 280 ////////////////////////  Acilisda bir kerelik hesaplayacak(nem,ortam sicakligi,atmosfer bas)
         BME280_Measure();


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  /////////////////// oksimetre ////////////////
	  if (max30102_has_interrupt(&max30102))
	  	  // FIFO'yu okumak için kesme işleyicisini çalıştır
	  	  max30102_interrupt_handler(&max30102);
	  /////////////////////   DS18B20    //////////////////
	  DS18B20_Start ();
	  HAL_Delay (1);
	  DS18B20_Write (0xCC);  // skip ROM
	  DS18B20_Write (0x44);  // convert t

	  DS18B20_Start ();
	  HAL_Delay(1);
	  DS18B20_Write (0xCC);  // skip ROM
	  DS18B20_Write (0xBE);  // Read Scratch-pad

	  Temp_byte1 = DS18B20_Read();
	  Temp_byte2 = DS18B20_Read();
	  TEMP = (Temp_byte2<<8)|Temp_byte1;
	  Temperature_DS = (float)TEMP/16;   //Sporcunun vucüt sıcaklığını ölcer
	  /////////////////////////////////////////

	  if(yazma_durum==1) //Yazma baslangic adimlari
	  	  {
		  BME280_Measure(); // Ilk iki satir icin init haric,bir kez daha ölc(Ölctügü garanti olsun)

		  f_mount(&fs, "", 0);    //Sd karti Mount et
		  sprintf (dosya_adi,"%d-%d-%d--%d-%d-%d.txt",gps.hours,gps.minutes,gps.seconds,gps.date,gps.month,gps.year); //Degiskenleri cek
		  f_open(&fil, dosya_adi, FA_OPEN_ALWAYS | FA_WRITE | FA_READ); //Antreman tarihine,zamanina uygun isimle dosya olustur
		  f_lseek(&fil, f_size(&fil));  //Son satira git
		  sprintf (yazi,"Ortam_Sic(C),Nem(RH),Atmosfer_Bas(kPa),Kilo,Yas\n");
		  f_puts(yazi , &fil);  //yazdir
		  sprintf (yazi,"%.0f,%.0f,%.1f,%.0f,%.0f\n",Temperature,Humidity,Pressure/1000,kilo,yas);
		  f_puts(yazi , &fil);  //yazdir
		  sprintf (yazi,"\nSatir,Zam(ms),Enlem,Boylam,Al_Yol(m),V(m/s),Ivme(m/s^2),VS(C),Nabiz(bpm)\n");
		  f_puts(yazi , &fil);  //yazdir
		  kosu_suresi_sayac=0;       //0 dan baslasin
		  kosu_ornekleme_periyot=0;  //0 dan baslasin
		  Moving_Average_Init(&filterStruct);   //Filtre Init (Filtrelenmis degerleri sıfırlama)
		  Moving_Average_Init(&filterStruct2);  //Kayıt ikinci kere çalıştırılsa diye
		  Moving_Average_Init(&filterStruct3);
		  yazma_durum=2;  //2. Yazma durumuna gec
	  	  }
//////////////////////////////////////////////////////////////////////////////////////////////
	  if(yazma_durum==2)
	  	  {
		  //indx=0 hala burada
		  if(indx==0 && gps.latitude != 0 && gps.longitude !=0)  //gps_baslangic_ornek_alma tek seferlik baslangic konumundan ornek almasini sagliyor.
		  	  	  	  {
		  		  	  enlem_baslangic=gps.latitude;
		  		  	  boylam_baslangic=gps.longitude;
		  		  	  indx=1;  //gps
		  		  	  }
		  distance = CalcGPSDistance(gps.latitude, gps.longitude,enlem_baslangic,boylam_baslangic);  //Yazma durum 2 içinde surekli alınan yol hesabı yapacak
		  // indx=1
		  if(indx==1) // ilk satiri yazsin diye(başlangıç konumundaki değerler)
		  {
			  sprintf (yazi,"%d,%d,%.6f,%.6f,%.3f,%.3f,%.3f,%.2f,%d\n",indx,kosu_suresi_sayac,gps.latitude,gps.longitude,distance_filtreli,v_filtreli_ikinci_adim,ivmefiltered,Temperature_DS,nabiz_filtered3);
			  f_puts(yazi , &fil);  //yazdir
			  indx++;
		  }
          //indx >1
		  if(kosu_ornekleme_periyot>=400 && distance>distance_old && indx>1)  //Sadece alınan yolda ilerleme varsa kaydedecek
		  		  {

			  	  distance_filtreli=(distance+distance_filtreli)/2;

			  	  v_filtresiz=((1000*(distance-distance_old))/kosu_ornekleme_periyot); //1000 le carpmayı unutma
			  	  v_filtreli_birinci_adim=Moving_Average_Compute(v_filtresiz, &filterStruct); //Bir kere filtreleme
			  	  v_filtreli_ikinci_adim=Moving_Average_Compute(v_filtreli_birinci_adim, &filterStruct2);//İki kere filtreleme

			  	  v_filtreli_ucuncu_adim_yeni=Moving_Average_Compute(v_filtreli_ikinci_adim, &filterStruct3);//Üç kere filtreleme(ivme hesabı için,hız kaydı için değil)

			  	  ivmeyeni=((v_filtreli_ucuncu_adim_yeni - v_filtreli_ucuncu_adim_eski) * 1000)/kosu_ornekleme_periyot;//ivme hesaplama
			  	  ivmefiltered=(ivmeyeni+ivmeeski)/2;//İvme filtreleme

			  	  sprintf (yazi,"%d,%d,%.6f,%.6f,%.3f,%.3f,%.3f,%.2f,%d\n",indx,kosu_suresi_sayac,gps.latitude,gps.longitude,distance_filtreli,v_filtreli_ikinci_adim,ivmefiltered,Temperature_DS,nabiz_filtered3);
			  	  f_puts(yazi , &fil);  //yazdir
			  	  indx++;
			  	  distance_old=distance;
			  	  v_filtreli_ucuncu_adim_eski = v_filtreli_ucuncu_adim_yeni;
			  	  ivmeeski=ivmeyeni;
			  	//////////////////////////  İstatistik Kayitlar       /////////////////////////////////
			  				  	  ort_hiz=ort_hiz+v_filtreli_ikinci_adim;
			  				  	  ort_nabiz=ort_nabiz+nabiz_filtered3;

			  				  	  if(max_hiz<v_filtreli_ikinci_adim)
			  				  	  {
			  				  		max_hiz=v_filtreli_ikinci_adim;
			  				  		max_hiza_cikma_suresi=kosu_suresi_sayac;
			  				  		max_hiz_nabiz=nabiz_filtered3;
			  				  	    max_hiz_VS=Temperature_DS;
			  				  	  }

			  	 //////////////////////////////////////////////////////////
			  	  kosu_ornekleme_periyot=0;
		  		  }
	  	  }  // yazma durum 2 nin

///////////////////////////////////  Yazma Durum 3   /////////////////////////////////////////////////////////////////
	  	  if(yazma_durum == 3 || distance_filtreli >= 100 || kosu_suresi_sayac > 240000 )  // Eger durma komutu gelirse(200 metre kosulursa) ya da  4 dakika sonunda otomatik(Ram'i doldurup donmasin diye)
	  	  {                                                       //bitis icin veya koy alinan yol >= 100 metre ise yap

	  		  deniz_seviyesi_sure=(kosu_suresi_sayac*(1.03-(0.03*exp(-0.000125*gps.altitude))));
	  		  ort_hiz=ort_hiz/(indx-1);			     // indx 1 ken hız 0.
              ort_nabiz=ort_nabiz/(indx-2);      //indx 1 ken nabiz 0 değil

            if(ort_nabiz>(220-yas)*0.8)  //Nabiz degerlendirme:Yuksek
            {
            	sprintf (yazi,"\nOrtalama Nabiz Yuksek: %d \n",ort_nabiz);
            	f_puts(yazi , &fil);  //yazdir
            }

            if(ort_nabiz<=(220-yas)*0.8) // Nabiz degerlendirme:Dusuk
            {
            	sprintf (yazi,"\nOrtalama Nabiz Normal: %d \n",ort_nabiz);
            	f_puts(yazi , &fil);  //yazdir
            }

            sprintf (yazi,"\nTop_Sur(ms),Den_Sev_Sur(ms),Top_Yol(m),Ort_Hiz(m/s),Max_Hiz(m/s),Max_Hiz_Cik_Sur(ms),Max_Hiz_Nab,Max_Hiz_VS,SpO2\n");
            f_puts(yazi , &fil);  //yazdir
            sprintf (yazi,"%d,%.0f,%.3f,%.3f,%.3f,%d,%d,%.2f,%.0f\n",kosu_suresi_sayac,deniz_seviyesi_sure,distance_filtreli,ort_hiz,max_hiz,max_hiza_cikma_suresi,max_hiz_nabiz,max_hiz_VS,spo2);
            f_puts(yazi , &fil);  //yazdir




              ///////////////////////////  Kayit Bitti    ///////////////////////////////////////////////////////
	  		  f_close(&fil);    // Dosyayi kapa
	  		  f_mount(NULL, "/", 1); //sd karti unmount et
	  		  ////////////////  Değişkenleri sıfırlama //////////////////////////////////////////////////////////
	  		  indx=0;
	  		  yazma_durum=0;   // Bekleme durmumuna gec, 0 = standby


	  		  kosu_ornekleme_periyot=0;
	  		  kosu_suresi_sayac=0;

	  		  distance=0;
	  		  distance_old=0;
	  		  distance_filtreli=0;

              v_filtresiz=0;
              v_filtreli_birinci_adim=0;
              v_filtreli_ikinci_adim=0;
              v_filtreli_ucuncu_adim_yeni=0;
              v_filtreli_ucuncu_adim_eski=0;

              ivmeyeni=0;
              ivmeeski=0;
              ivmefiltered=0;

	  		  enlem_baslangic=0;
	  		  boylam_baslangic=0;

	  		  deniz_seviyesi_sure=0;
	  		  ort_hiz=0;
	  		  max_hiz=0;
	  		  max_hiza_cikma_suresi=0;
	  		  max_hiz_nabiz=0;
	  		  max_hiz_VS=0;
	  		  ort_nabiz=0;

	  		  SSD1306_Clear();  // LCD yi temizle kayit bitince
	  		  ///////////////////////////////////////
	  	  }

	  /////////////////////  LCD REAL TIME DEBUG //////////////////
	  lcd_spo2=(spo2);  //float basica duzgun gostermedi
	  lcd_Temperature_DS=Temperature_DS; //float basica duzgun gostermedi
      ////////////////////////////////////////////
	  SSD1306_GotoXY (0,0);//LCD imleç konumu değiştirme
	  sprintf (lcd_yazi,"HR:%d|Ok:%d|VS:%d ",nabiz_filtered3,lcd_spo2,lcd_Temperature_DS);
	  SSD1306_Puts (lcd_yazi, &Font_7x10, 1);//LCD Yazdırma
	  /////////////////////////////////////////
	  SSD1306_GotoXY (0,11);//LCD imleç konumu değiştirme
	  SSD1306_Puts ("Yol,Hiz,Ivme", &Font_7x10, 1);//LCD Yazdırma
	  sprintf (lcd_yazi,"%.2f,%.2f,%.2f",distance_filtreli,v_filtreli_ikinci_adim,ivmefiltered);
	  SSD1306_GotoXY (0,22);//LCD imleç konumu değiştirme
	  SSD1306_Puts (lcd_yazi, &Font_7x10, 1);//LCD Yazdırma
	  SSD1306_GotoXY (0,44);//LCD imleç konumu değiştirme
	  sprintf (lcd_yazi,"Dur:%d;Zam:%d",yazma_durum,kosu_suresi_sayac);
	  SSD1306_Puts (lcd_yazi, &Font_7x10, 1);//LCD Yazdırma
	  SSD1306_UpdateScreen(); //display


//	  SSD1306_Clear();

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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 400000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.ClockSpeed = 400000;
  hi2c3.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

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
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 100-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1000-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 100-1;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 4294967295;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(DS18B20_GPIO_Port, DS18B20_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : EXTI_Nabiz_Pin */
  GPIO_InitStruct.Pin = EXTI_Nabiz_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(EXTI_Nabiz_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : DS18B20_Pin */
  GPIO_InitStruct.Pin = DS18B20_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(DS18B20_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI_CS_Pin */
  GPIO_InitStruct.Pin = SPI_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SPI_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : button1_Pin button2_Pin */
  GPIO_InitStruct.Pin = button1_Pin|button2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : MAX30102_INT_Pin */
  GPIO_InitStruct.Pin = MAX30102_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(MAX30102_INT_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
		  if (GPIO_Pin == GPIO_PIN_13)
		  {
			nabiz_yeni=HAL_GetTick();  //Cihaz açıldığından beri geçen süre milisaniye
			nabiz_gecen_sure=nabiz_yeni-nabiz_eski; // R-R interval arasi geçen sure

			if(nabiz_gecen_sure>333)  //BPM=180 max,eğer 180 üstü bpm sinyali gelirse değerlendirilmez
			{
			nabiz_bpm=60000/nabiz_gecen_sure;  //her tikte anlık nabiz olcuyor
			}
			nabiz_eski=nabiz_yeni;//Mevcut değer eski değer olarak kaydedilir.

			nabiz_filtered=Moving_Average_Compute(nabiz_bpm, &filterStructbpm);
			nabiz_filtered2=Moving_Average_Compute(nabiz_filtered, &filterStructbpm2);
			nabiz_filtered3=Moving_Average_Compute(nabiz_filtered2, &filterStructbpm3);
		  }

	    //////////////////////////////////////////////////////////////////////////
	    currentMillis12 = HAL_GetTick();//pb12 button 1
	                                  //Button Debounce önleme
	    if (GPIO_Pin == GPIO_PIN_12 && (currentMillis12 - previousMillis12 > 10))
	    {
	  	yazma_durum = 1;  //kayit baslatma
	    previousMillis12 = currentMillis12; //yeni durum artık eskidi

	    button1=1; //Kilo,yas counter
	    }

	    ////////////////////////////////////////////////////////////////////////

	    currentMillis14 = HAL_GetTick();//pb14 button 2
	    								//Button Debounce önleme
	    if (GPIO_Pin == GPIO_PIN_14 && (currentMillis14 - previousMillis14 > 10))
	    {
	  	yazma_durum = 3;  //kayit bitirme
	    previousMillis14 = currentMillis14;//yeni durum artık eskidi

	    button2=1; //Onaylama tusu(kilo yas init)
	    }

}

//  Alinan Yol Hesaplama Haversine formulu
float CalcGPSDistance(float latitud1, float longitud1, float latitud2, float longitud2){
    double haversine;
    double temp;
    double distancia_puntos;

    latitud1  = latitud1  * GRADOS_RADIANES; //Radyana çevirme işlemleri
    longitud1 = longitud1 * GRADOS_RADIANES;
    latitud2  = latitud2  * GRADOS_RADIANES;
    longitud2 = longitud2 * GRADOS_RADIANES;

    haversine = (pow(sin((1.0 / 2) * (latitud2 - latitud1)), 2)) + ((cos(latitud1)) * (cos(latitud2)) * (pow(sin((1.0 / 2) * (longitud2 - longitud1)), 2)));
    temp = 2 * asin(min(1.0, sqrt(haversine))); //C değeri
    distancia_puntos = RADIO_TERRESTRE * temp;  //D değeri

   return distancia_puntos;
}

float min(float a ,float b) //STM32 C kütüphanelerinde min fonksiyonu yoktur
{               //Bu yüzden burada hazırlanmıştır.
	if(a > b) return b;
	return a;
}

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */
	if (htim->Instance == TIM2)
		{
			if(yazma_durum!=0) //Program basladiginda saymasin.İlk butona basildiktan sonra saysin
			{
		     kosu_suresi_sayac++;
		     kosu_ornekleme_periyot++;
			}
		}

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
