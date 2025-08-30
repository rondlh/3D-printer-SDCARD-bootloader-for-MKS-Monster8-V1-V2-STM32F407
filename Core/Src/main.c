/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

/***************************************************************************************************
* IRON 2025 smart SD card bootloader for MKS Monster8 V1/V2 3D printer motherboard (STM32F407VET6) *
****************************************************************************************************
* SD card interfacing based on Simplified Spec Physical Layer V9.10 (December 1st 2023)            *
***************************************************************************************************/

/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <stdarg.h> // For uart_printf
#include <string.h> // For memset
#include "ff.h"     // FAT File System
#include "diskio.h" // FAT File System IO driver

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define DEBUG_USART_HANDLE		huart1									// The UART to send debug info to
#define VERBOSE_MODE													// Comment out to disable sending info to USART (250K baud)
#define COMPARE_BEFORE_FLASH											// Comment out to not compare the firmware file to the flash contents (faster)
#define FIRMWARE_FILENAME		"firmware.bin"							// The firmware file to flash
#define FIRMWARE_RENAME			"firmware.cur"							// Rename the firmware after flashing

// Make sure pins don't interfere with SWD debug pins PA13 and PA14, disable when debugging
#define PROGRESS_LED_PIN		GPIO_PIN_13								// Progress LED pin, flashes during flash update
#define PROGRESS_LED_PORT		GPIOA									// Progress LED port

#define SD_DETECT_PIN			GPIO_PIN_4								// SD card detect pin, pin PC4
#define SD_DETECT_PORT			GPIOC									// SD card detect port, pin PC4
#define SD_INIT_DELAY			350U									// Delay to allow the SD card to settle

#define DFU_ON_DOUBLE_RESET												// Double reset start DFU mode, comment out to disable
#define DFU_MAGIC_KEY			0xBA5EBA11								// Magic key to jump to DFU mode
//#define DFU_MAGIC_KEY_ADDRESS	0x2001BFFC								// Store the magic key at end of SRAM2
#define DFU_MAGIC_KEY_ADDRESS	RTC_BKP_DR19							// Store the magic key at RTC backup register 19

#define FLASHWORD				4U										// 4 bytes on STM32F4
#define FILE_BUFFER_SIZE		4096UL									// Must be dividable by FLASHWORD
#define FLASH_MAX_SECTOR		12U										// Max 12 sectors on STM32F4xx (0-11)
#define FLASH_BOOTLOADER_SIZE	0x0010000UL								// Bootloader area size
#define FLASH_USER_START_SECTOR	4U										// Bootloader in sector 0-3 (4 x 16KB)
#define FLASH_USER_START_ADDR	(FLASH_BASE + FLASH_BOOTLOADER_SIZE)	// Should start on a new sector

/* STM32 DFU bootloader addresses
   STM32C0   0x1FFF0000 | STM32F030x8 0x1FFFEC00 | STM32F030xC 0x1FFFD800 | STM32F03xx 0x1FFFEC00
   STM32F05  0x1FFFEC00 | STM32F07    0x1FFFC800 | STM32F09    0x1FFFD800 | STM32F10xx 0x1FFFF000
   STM32F105 0x1FFFB000 | STM32F107   0x1FFFB000 | STM32F10XL  0x1FFFE000 | STM32F2    0x1FFF0000
   STM32F3   0x1FFFD800 | STM32F4     0x1FFF0000 | STM32F7     0x1FF00000 | STM32G0    0x1FFF0000
   STM32G4   0x1FFF0000 | STM32H503   0x0BF87000 | STM32H563   0x0BF97000 | STM32H573  0x0BF97000
   STM32H7x  0x1FF09800 | STM32H7A    0x1FF0A800 | STM32H7B    0x1FF0A000 | STM32L0    0x1FF00000
   STM32L1   0x1FF00000 | STM32L4     0x1FFF0000 | STM32L5     0x0BF90000 | STM32WBA   0x0BF88000
   STM32WBX  0x1FFF0000 | STM32WL     0x1FFF0000 | STM32U5     0x0BF90000 */

#define DFU_BOOTLOADER_ADDRESS	0x1FFF0000U	// STMF4xx address to start the DFU bootloader

/* FATFS ERROR CODES
 0 Succeeded
 1 A hard error occurred in the low level disk I/O layer
 2 Assertion failed
 3 The physical drive cannot work
 4 Could not find the file
 5 Could not find the path
 6 The path name format is invalid
 7 Access denied due to prohibited access or directory full
 8 Access denied due to prohibited access
 9 The file/directory object is invalid
10 The physical drive is write protected
11 The logical drive number is invalid
12 The volume has no work area
13 There is no valid FAT volume
14 The f_mkfs() aborted due to any problem
15 Could not get a grant to access the volume within defined period
16 The operation is rejected according to the file sharing policy
17 LFN working buffer could not be allocated
18 Number of open files > _FS_LOCK
19 Given parameter is invalid */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi3;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

FATFS    FatFs;							// FAT File System handle
FIL      fwFile;						// File handle for the firmware file
FRESULT  result;						// File operation result
uint32_t fileSize;						// Firmware file size in bytes
uint8_t  fileBuffer[FILE_BUFFER_SIZE];	// File read buffer

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI3_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_RTC_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// Calculate CRC32 of a block of data
#define CRC32_START 0xFFFFFFFF  // Start value for the CRC32 calculation
uint32_t crc32b(uint32_t crc, uint8_t *data, uint32_t size)
{
	for (int i = 0; i < size; i++)
	{
		crc = crc ^ data[i];
		for (int j = 8; j; j--)
			crc = (crc >> 1) ^ (0xEDB88320 & -(crc & 1));
	}
	return ~crc;
}

#ifdef VERBOSE_MODE // No serial output if not in VERBOSE_MODE

	// Lightweight printf, prints to uart (DEBUG_USART_HANDLE), no floats, no width control
	void uart_printf(const char * fmt, ...)
	{
		va_list va;
		va_start(va, fmt);
		char debug_msg[255]; // Message buffer
		char * buf = debug_msg;
		char c;
		unsigned int num;
		while ((c = *(fmt++)))
		{
			int width = 0;
			if (c == '%')
			{
				int base = 2;
				int s_int = 0;
			MORE_FORMAT:
				c = *(fmt++); // Skip '%', check parameter
				switch (c)
				{
					case '0'...'9': // Width indicators
				  	width = (width * 10) + c - '0';
					goto MORE_FORMAT;

					case '%': // "%%" prints "%"
				 	   *(buf++) = '%';
					break;

					case 'c': // Character
				 	   *(buf++) = va_arg(va, int);
					break;

					case 'd': // Signed integer, base 10
					case 'i': base = 10;
						s_int = va_arg(va, int);
						if (s_int < 0)
					   		num = -s_int;
						else
					  		num = s_int;
				  	goto ATOI;
					case 'x':	   // Hexadecimal, base 16
						base += 6; // 2 + 6 + 8 is base 16
					case 'u':	   // Unsigned integer, base 10
						base += 8; // 2 + 8 is base 10
					case 'b':	   // Binary, base 2
						num = va_arg(va, unsigned int);
				  	ATOI:
						char tmp[32]; // 32bit
						char *q = tmp;

						do
						{
							int rem = '0' + (num % base);
							if (rem > '9')
						  		rem += 7; // Map to 'ABCDEF'
					   		*(q++) = rem;
						} while ((num /= base));

						if (s_int < 0)
					  		*(q++) = '-';

						width -= q - tmp;
						while (width-- > 0)
					  		*(buf++) = ' ';

				   		while (tmp < q) // Reverse data order, "123" --> "321"
					   	*(buf++) = *(--q);
					break;

					case 's':  // String
					{
						const char *p = va_arg(va, const char *);
						while (*p)
							*(buf++) = *(p++);
					}
				}
			}
			else
				*(buf++) = c; // Copy literal characters
		}
		*buf = '\0'; // Terminate string

		va_end(va);

		HAL_UART_Transmit(&DEBUG_USART_HANDLE, (uint8_t *)debug_msg, buf - debug_msg, HAL_MAX_DELAY);

		#ifdef PROGRESS_LED_PIN
			HAL_GPIO_TogglePin(PROGRESS_LED_PORT, PROGRESS_LED_PIN); // Flash LED
		#endif

	}

#else

	#ifdef PROGRESS_LED_PIN
		void uart_printf(const char * fmt, ...) { HAL_GPIO_TogglePin(PROGRESS_LED_PORT, PROGRESS_LED_PIN); } // Flash LED
	#else
		void uart_printf(const char * fmt, ...) { }
	#endif

#endif

#ifdef DFU_ON_DOUBLE_RESET

    #if (DFU_MAGIC_KEY_ADDRESS < 0xFF)
  	    // Use RTC backup register
	    void set_magic_key(int value) {
	        HAL_PWR_EnableBkUpAccess();
	        HAL_RTCEx_BKUPWrite(&hrtc, DFU_MAGIC_KEY_ADDRESS, value);
	        HAL_PWR_DisableBkUpAccess();
	    }
    #else
	    // Use memory address
        #define set_magic_key(a) (*(__IO uint32_t *)DFU_MAGIC_KEY_ADDRESS = a)
    #endif

#else

	#define set_magic_key(a)

#endif

// Return value: 0=equal, 1=different, 2=error
uint32_t compareFlashToFile(void)
{
	uint32_t i = 0, j;
	uint32_t file_crc32 = ~CRC32_START; // Invert here, will be undone in crc32b
	int difference_found = 0;
	int different = 0;
    unsigned int bytesRead;
	int result = f_lseek(&fwFile, 0); // Not strictly needed

	while ((i < fileSize) && !result)
	{
		result = f_read(&fwFile, fileBuffer, FILE_BUFFER_SIZE, &bytesRead);
		file_crc32 = crc32b(~file_crc32, fileBuffer, bytesRead);
		j = 0;
		while ((j < bytesRead) && !result)
		{
			if (*(__IO char*)(FLASH_USER_START_ADDR + i + j) != fileBuffer[j])
				difference_found = 1;
			j++;
		}

		if (difference_found)
		{
			uart_printf("*");
			different = 1;
			difference_found = 0; // Reset block different status
		}
		else
			uart_printf("=");

		i += bytesRead;
	}

	if (result)
	{
		uart_printf(" Error\r\nFile read error: %d\r\n", result);
		return 2;
	}
	else
	if (different)
		uart_printf(" Different\r\nFlash contents differs, update is required\r\n");
	else
	{
		uart_printf(" Equal\r\nFlash contents is the same, update is not required\r\n");
		uart_printf("Flash CRC32: 0x%x\r\n", file_crc32);
	}

	return different; // 0=equal, 1=different, 2=file read error
}

int CopyFileToFlashMemory(void)
{
/* STM32F4xx FLASH SECTORS
	Sector  0 0x0800 0000 - 0x0800 3FFF  16KB
	Sector  1 0x0800 4000 - 0x0800 7FFF  16KB
	Sector  2 0x0800 8000 - 0x0800 BFFF  16KB
	Sector  3 0x0800 C000 - 0x0800 FFFF  16KB
	Sector  4 0x0801 0000 - 0x0801 FFFF  64KB
	Sector  5 0x0802 0000 - 0x0803 FFFF 128KB
	Sector  6 0x0804 0000 - 0x0805 FFFF 128KB
	Sector  7 0x0804 0000 - 0x0805 FFFF 128KB
	// -- 512KB -----------------------------
	Sector  8 0x0804 0000 - 0x0805 FFFF 128KB
	Sector  9 0x0804 0000 - 0x0805 FFFF 128KB
	Sector 10 0x0804 0000 - 0x0805 FFFF 128KB
	Sector 11 0x080E 0000 - 0x080F FFFF 128KB */

	uint32_t flash_sector_size = 0x10000U;

	// Erase required sectors to fit the user application
	uint32_t erasedSize = 0;
	uint32_t sector = FLASH_USER_START_SECTOR;

	HAL_FLASH_Unlock();
	FRESULT result = f_lseek(&fwFile, 0);

	while ((erasedSize < fileSize) && !result)
	{
		uart_printf("Erasing flash sector %u  Sector size: %3uKB\r\n", sector, flash_sector_size >> 10);
		FLASH_Erase_Sector(sector, FLASH_VOLTAGE_RANGE_3);

		erasedSize += flash_sector_size;
		flash_sector_size = 0x20000; // The remaining sectors are 128KB
		sector++;
	}

    uart_printf("Flashing user application to 0x0%x\r\n", FLASH_USER_START_ADDR);
	uint32_t byteCounter = 0;
	uint32_t i;
	uint32_t file_crc32 = ~CRC32_START; // Invert here, will be undone in crc32b
	unsigned int bytesRead;

	while ((byteCounter < fileSize) && !result)
	{
		result = f_read(&fwFile, fileBuffer, FILE_BUFFER_SIZE, &bytesRead);
		file_crc32 = crc32b(~file_crc32, fileBuffer, bytesRead);

		if (bytesRead < FILE_BUFFER_SIZE) // Add some "erased flash" bytes to the buffer
			memset(fileBuffer + bytesRead, 0xFF, (FILE_BUFFER_SIZE - bytesRead) % FLASHWORD);

		// Write the data to flash memory
		i = 0;
		while ((i < bytesRead) && !result)
		{
			result = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, FLASH_USER_START_ADDR + byteCounter + i, *((volatile uint32_t*)(fileBuffer + i)));
			i += FLASHWORD;
		}
		byteCounter += bytesRead;
		uart_printf("=");
	}

	HAL_FLASH_Lock();

	if (!result) // All went OK, verify flash contents
	{
		uint32_t flash_crc32 = crc32b(CRC32_START, (uint8_t*)FLASH_USER_START_ADDR, fileSize);

		if (file_crc32 != flash_crc32)
        {
			uart_printf("* Verify failed\r\n");
            result = 1; // Signal error
        }
		else
			uart_printf(" Verify OK\r\n");

		uart_printf("Flash CRC32: 0x%x\r\n", flash_crc32);
	}
	else
		uart_printf(" Failed: %u\r\n", result);

	return result;
}

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
  MX_SPI3_Init();
  MX_USART1_UART_Init();
  MX_FATFS_Init();
  MX_RTC_Init();
  /* USER CODE BEGIN 2 */

#ifdef DFU_ON_DOUBLE_RESET

#if (DFU_MAGIC_KEY_ADDRESS < 0xFF)
	if (HAL_RTCEx_BKUPRead(&hrtc, DFU_MAGIC_KEY_ADDRESS) == DFU_MAGIC_KEY) // Detect memory marker
#else
	if (*(__IO uint32_t*)DFU_MAGIC_KEY_ADDRESS == DFU_MAGIC_KEY) // Detect marker in RTC backup register
#endif
	{
		set_magic_key(0);
		MX_GPIO_Init();
		MX_USART1_UART_Init();
		uart_printf("\r\nStarting DFU mode\r\n");
		HAL_Delay(25);

		__disable_irq(); // Disable all interrupts
		HAL_SuspendTick(); // Suspend tick interrupt
        SysTick->CTRL = SysTick->LOAD = SysTick->VAL = 0;
		RCC->CIR = 0;     // Disable clock interrupts
		HAL_RCC_DeInit(); // Set the clock to the default state
		HAL_DeInit();
		// Clear Interrupt Enable Register & Interrupt Pending Register
		uint8_t cnt = (sizeof(NVIC->ICER) / sizeof(*NVIC->ICER));
		for (int i = 0; i < cnt; i++)
		{
		    NVIC->ICER[i] = 0xFFFFFFFF;
		    NVIC->ICPR[i] = 0xFFFFFFFF;
		}
		__enable_irq();

		uint32_t *vtor = (void*)DFU_BOOTLOADER_ADDRESS;
		SCB->VTOR = (uint32_t)vtor;

		// Make the jump
		asm volatile("MSR msp,%0\nbx %1" : : "r"(vtor[0]), "r"(vtor[1]));
	}

	set_magic_key(DFU_MAGIC_KEY);

	// Wait for 2nd reset while DFU marker is set
	HAL_Delay(500);

	set_magic_key(0);

#endif

 // Enable all the GPIO clocks for the configurable pin below
  SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN | RCC_AHB1ENR_GPIODEN | RCC_AHB1ENR_GPIOEEN);


  uart_printf("\r\nSD card bootloader started\r\n");

GPIO_InitTypeDef GPIO_InitStruct = { 0 };
#ifdef PROGRESS_LED_PIN
	GPIO_InitStruct.Pin = PROGRESS_LED_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(PROGRESS_LED_PORT, &GPIO_InitStruct);
#endif

#ifdef SD_DETECT_PIN
	GPIO_InitStruct.Pin = SD_DETECT_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(SD_DETECT_PORT, &GPIO_InitStruct);
#endif

#ifdef SD_DETECT_PIN
	// Detect SD card, high pin means no SD card present
	if (HAL_GPIO_ReadPin(SD_DETECT_PORT, SD_DETECT_PIN))
	{
		uart_printf("\r\nNo SD card detected\r\n");
		goto USER_APP;
	}
	uart_printf("\r\nSD card detected, mounting FAT file system\r\n");
#endif

	HAL_Delay(SD_INIT_DELAY); // Short delay to let the SD card settle

	// Mount the FAT file system
	result = f_mount(&FatFs, "", 1);
	if (result)
	{
		#ifdef SD_DETECT_PIN
			uart_printf("ERROR: Card mounting failed, not FAT/exFAT formatted? Error: %d\r\n", result);
		#else
			uart_printf("No medium mounted, status: %d\r\n", result)
		#endif

		goto USER_APP;
	}

	uart_printf(FIRMWARE_FILENAME);
	if (f_open(&fwFile, FIRMWARE_FILENAME, FA_READ))
	{
		uart_printf(" not found\r\n");
		goto USER_APP;
	}
	uart_printf(" opened successfully\r\n");

	fileSize = f_size(&fwFile);
	if (!fileSize)
	{
		uart_printf("ERROR: %s has 0 size, aborting\r\n", FIRMWARE_FILENAME);
		f_close(&fwFile); // Not strictly needed
		goto USER_APP;
	}

	// Get device flash size from memory (in KBytes)
	__IO uint16_t flashSize = *(uint32_t*)(FLASHSIZE_BASE);
	uart_printf("Total flash memory size: %uKB\r\n", flashSize);

	uint32_t freeFlash = (flashSize << 10) - FLASH_BOOTLOADER_SIZE;
	uart_printf("Free flash space: %dKB\r\n", freeFlash >> 10);

	uart_printf("Firmware file size: %uKB\r\n", fileSize >> 10);

	if (fileSize > freeFlash)
	{
		uart_printf("ERROR: Insufficient free flash space, aborting\r\n");
		f_close(&fwFile); // Not strictly needed, comment out to save some flash
		goto USER_APP;
	}

	#ifdef COMPARE_BEFORE_FLASH

		uart_printf("Comparing file to flash contents\r\n");

		result = compareFlashToFile();

		if (result > 1) // File read error
			goto USER_APP;

		if (result == 1) // Flash is different, update required
			result = CopyFileToFlashMemory();

	#else

		result = CopyFileToFlashMemory();

	#endif

	f_close(&fwFile); // Must close file before renaming

	#ifdef FIRMWARE_RENAME
		if (!result) // Only rename/delete if file was flashed successfully
		{
			f_unlink(FIRMWARE_RENAME); // Delete the old firmware (if present)

			if (f_rename(FIRMWARE_FILENAME, FIRMWARE_RENAME) != FR_OK)
            {
				uart_printf("ERROR: Failed to rename firmware file to ");
                result = 1; // Signal error
            }
			else
				uart_printf("Renaming file to ");

			uart_printf("%s\r\n", FIRMWARE_RENAME);
		}

	#endif

USER_APP:

	f_mount(NULL, "", 0); // Unmount SDCARD, not strictly needed

	if (*(__IO uint32_t*)FLASH_USER_START_ADDR != 0xFFFFFFFF) // Check if flash is empty
	{
		set_magic_key(0);
		uart_printf("Starting user application at 0x0%x\r\n", FLASH_USER_START_ADDR);
		HAL_Delay(25);

		uint32_t *vtor = (void*)FLASH_USER_START_ADDR;
		SCB->VTOR = (uint32_t)vtor;

		// Make the jump
		asm volatile("MSR msp,%0\nbx %1" : : "r"(vtor[0]), "r"(vtor[1]));
	}

	uart_printf("No user application found at 0x0%x, done!\r\n", FLASH_USER_START_ADDR);

	while (1)
	{ // Start slow LED flash
		#ifdef PROGRESS_LED_PIN
			HAL_GPIO_TogglePin(PROGRESS_LED_PORT, PROGRESS_LED_PIN);
			HAL_Delay(1500);
		#endif
	};

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
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
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

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
  huart1.Init.BaudRate = 250000;
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
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SD_CHIP_SELECT_GPIO_Port, SD_CHIP_SELECT_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : SD_CHIP_SELECT_Pin */
  GPIO_InitStruct.Pin = SD_CHIP_SELECT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(SD_CHIP_SELECT_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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
#ifdef USE_FULL_ASSERT
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
