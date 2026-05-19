/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include "ssd1306.h"
#include "ssd1306_fonts.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef enum {
	IPD_WAIT_HDR, IPD_READ_HDR, IPD_READ_PAYLOAD
} ipd_state_t;

typedef struct {
	 uint8_t id;
	 int16_t temperature_x100;  // z.B. 2345 = 23.45°C
	 int16_t humidity_x100;
	 uint32_t lastUpdate;
	 bool valid;
} slave_data_t;

typedef enum {
	UI_MENU,
	UI_MEASUREMENT,
	UI_CAL_TEMP,
	UI_CAL_HUM,
	UI_LIMIT_TEMP_MIN,
	UI_LIMIT_TEMP_MAX,
	UI_LIMIT_HUM_MIN,
	UI_LIMIT_HUM_MAX
} ui_state_t;

typedef enum {
	BUTTON_NONE,
	BUTTON_UP,
	BUTTON_DOWN,
	BUTTON_ENTER
} button_event_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define RX_BUF_SIZE 1500

// WLAN Zugangsdaten (Hotspot)
#define WIFI_SSID "Tyler"
#define WIFI_PASS "12345678"

// TCP Server-Port
#define SERVER_PORT 5000

#define IPD_HDR_MAX   64

#define MAX_SLAVES 4

#define BUTTON_UP_PORT      GPIOB
#define BUTTON_UP_PIN       GPIO_PIN_4
#define BUTTON_DOWN_PORT    GPIOB
#define BUTTON_DOWN_PIN     GPIO_PIN_5
#define BUTTON_ENTER_PORT   GPIOA
#define BUTTON_ENTER_PIN    GPIO_PIN_11
#define BUTTON_PRESSED      GPIO_PIN_SET
#define BUTTON_DEBOUNCE_MS  50

#define MENU_ITEM_COUNT 3

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

static char rxBuf[RX_BUF_SIZE];

static ipd_state_t ipd_state = IPD_WAIT_HDR;
static char ipd_hdr[IPD_HDR_MAX];
static uint16_t ipd_hdr_len = 0;
static int ipd_expect = 0;   // erwartete Nutzlast-Bytes aus Header
static uint16_t ipd_payload_i = 0;

static void ipd_reset(void) {
	ipd_state = IPD_WAIT_HDR;
	ipd_hdr_len = 0;
	ipd_expect = 0;
	ipd_payload_i = 0;
}

slave_data_t slaves[MAX_SLAVES];

static ui_state_t ui_state = UI_MENU;
static uint8_t menu_index = 0;
static bool ui_redraw_requested = true;

static int16_t temperature_offset_c = 0;
static int16_t humidity_offset_percent = 0;
static int16_t temperature_min_c = 0;
static int16_t temperature_max_c = 50;
static int16_t humidity_min_percent = 0;
static int16_t humidity_max_percent = 100;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void ESP_SendCmd(const char *cmd, uint32_t timeout) {
	uint8_t rx[RX_BUF_SIZE];
	uint16_t idx = 0;
	uint8_t ch;
	uint32_t start = HAL_GetTick();
	memset(rx, 0, RX_BUF_SIZE);
	HAL_UART_Transmit(&huart1, (uint8_t*) cmd, strlen(cmd), HAL_MAX_DELAY); // Debug-Ausgabe
	HAL_UART_Transmit(&huart2, (uint8_t*) ">>> ", 4, HAL_MAX_DELAY);
	HAL_UART_Transmit(&huart2, (uint8_t*) cmd, strlen(cmd), HAL_MAX_DELAY);
	while ((HAL_GetTick() - start) < timeout && idx < (RX_BUF_SIZE - 1)) {
		if (HAL_UART_Receive(&huart1, &ch, 1, 200) == HAL_OK) {
			rx[idx++] = ch;
		}
	}
	rx[idx] = '\0';
	HAL_UART_Transmit(&huart2, (uint8_t*) " <- ", 4, HAL_MAX_DELAY);
	HAL_UART_Transmit(&huart2, rx, strlen((char*) rx), HAL_MAX_DELAY);
	HAL_UART_Transmit(&huart2, (uint8_t*) "\r\n", 2, HAL_MAX_DELAY);
}

void ESP_Init(void) {
	HAL_UART_Transmit(&huart2,
			(uint8_t*) "\r\n--- ESP WLAN Verbinden (Server) ---\r\n", 38,
			HAL_MAX_DELAY);
	// 1) Test, Reset, Echo off
	ESP_SendCmd("AT\r\n", 2000);
	HAL_Delay(300);
	ESP_SendCmd("AT+RST\r\n", 5000);
	HAL_Delay(3000);
	ESP_SendCmd("ATE0\r\n", 1000); // Echo off
	HAL_Delay(100);
	ESP_SendCmd("AT+CIPDINFO=0\r\n", 1000); // kompakter +IPD-Header: +IPD,<len>:<payload>
	HAL_Delay(100);
	// 2) Station Mode (CWMODE=1)
	ESP_SendCmd("AT+CWMODE_DEF=1\r\n", 2000);
	HAL_Delay(200);
	// 3) Vorherige WLAN-Verbindung trennen
	ESP_SendCmd("AT+CWQAP\r\n", 2000);
	HAL_Delay(200);
	// 4) Mit Hotspot verbinden (Station)
	char joinCmd[128];
	snprintf(joinCmd, sizeof(joinCmd), "AT+CWJAP=\"%s\",\"%s\"\r\n", WIFI_SSID,
			WIFI_PASS);
	ESP_SendCmd(joinCmd, 30000);
	HAL_Delay(1000);
	// 5) IP anzeigen
	ESP_SendCmd("AT+CIFSR\r\n", 3000);
	// 6) Multi-Connection Mode aktivieren
	ESP_SendCmd("AT+CIPMUX=1\r\n", 2000);
	HAL_Delay(200);

	ESP_SendCmd("AT+CIPSERVERMAXCONN=4\r\n", 2000);
	HAL_Delay(200);
	ESP_SendCmd("AT+CIPSTO=3600\r\n", 2000);   // Timeout 1 Stunde
	HAL_Delay(200);
}

void ESP_StartServer(void) {
	char cmd[64];
	snprintf(cmd, sizeof(cmd), "AT+CIPSERVER=1,%d\r\n", SERVER_PORT);
	ESP_SendCmd(cmd, 3000);
	HAL_Delay(200);
}

static bool ipd_try_parse_len(const char *hdr, int *out_len) {
	// akzeptiere beide Formen:
	// "+IPD,<len>:" oder
	// "+IPD,<link>,<len>:"
	// Wir greifen die letzte Zahl vor ':' als Länge ab.
	const char *colon = strchr(hdr, ':');
	if (!colon)
		return false;
	// suche letzte Komma-Position vor ':'
	const char *p = colon - 1;
	while (p > hdr && *p != ',')
		p--;
	if (*p != ',')
		return false;
	int len = atoi(p + 1);
	if (len <= 0)
		return false;
	*out_len = len;
	return true;
}

void ESP_ProcessIncomingByte(uint8_t ch) {
	switch (ipd_state) {
	case IPD_WAIT_HDR:
		if (ch == '+') {
			ipd_hdr_len = 0;
			ipd_hdr[ipd_hdr_len++] = ch;
			ipd_state = IPD_READ_HDR;
		}
		break;

	case IPD_READ_HDR:
		if (ipd_hdr_len < (IPD_HDR_MAX - 1)) {
			ipd_hdr[ipd_hdr_len++] = ch;
			ipd_hdr[ipd_hdr_len] = '\0';
		} else {
			ipd_reset();
			break;
		}
		if (ch == ':') {
			if (strncmp(ipd_hdr, "+IPD", 4) != 0) {
				ipd_reset();
				break;
			}
			if (!ipd_try_parse_len(ipd_hdr, &ipd_expect)) {
				ipd_reset();
				break;
			}
			ipd_payload_i = 0;
			ipd_state = IPD_READ_PAYLOAD;
		}
		break;

	case IPD_READ_PAYLOAD:
		if (ipd_payload_i < (RX_BUF_SIZE - 1)) {
			rxBuf[ipd_payload_i++] = (char) ch;
		}
		if (--ipd_expect == 0) {
			rxBuf[(ipd_payload_i < RX_BUF_SIZE) ?
					ipd_payload_i : (RX_BUF_SIZE - 1)] = '\0';

			// CR/LF am Ende entfernen
			rxBuf[strcspn(rxBuf, "\r\n")] = 0;

			// UART Debugausgabe
			HAL_UART_Transmit(&huart2, (uint8_t*) "Empfangen: ", 11,
					HAL_MAX_DELAY);
			HAL_UART_Transmit(&huart2, (uint8_t*) rxBuf, strlen(rxBuf),
					HAL_MAX_DELAY);
			HAL_UART_Transmit(&huart2, (uint8_t*) "\r\n", 2, HAL_MAX_DELAY);

			int ID = -1;
			int t_int = 0, t_frac = 0;
			int h_int = 0, h_frac = 0;
			if (sscanf(rxBuf, "ID=%d;T=%d.%d;H=%d.%d", &ID, &t_int, &t_frac, &h_int, &h_frac) == 5) {

				// Debug ohne %f
				    char dbg[64];
				    snprintf(dbg, sizeof(dbg), "Slave %d -> %d.%02d°C H=%d.%02d%%\r\n",
				             ID, t_int, t_frac, h_int, h_frac);
				    HAL_UART_Transmit(&huart2, (uint8_t*) dbg, strlen(dbg), HAL_MAX_DELAY);

				    if (ID >= 1 && ID <= MAX_SLAVES) {
				    	slaves[ID - 1].temperature_x100 = t_int * 100 + t_frac;
				    	slaves[ID - 1].humidity_x100    = h_int * 100 + h_frac;
				        slaves[ID - 1].lastUpdate  = HAL_GetTick();
				        slaves[ID - 1].valid       = true;
				    }

			} else {
				HAL_UART_Transmit(&huart2, (uint8_t*) "Parsing failed!\r\n", 17,
						HAL_MAX_DELAY);
			}
			ipd_reset();
		}
		break;
	}
}

void ESP_CheckIncoming(void) {
	uint8_t ch;
	uint32_t last_rx_time = HAL_GetTick();
	bool got_data = false;

	// Sammle alles, was im UART-Puffer liegt (z.B. 50 ms lang)
	while ((HAL_GetTick() - last_rx_time) < 50) {
		if (HAL_UART_Receive(&huart1, &ch, 1, 5) == HAL_OK) {
			got_data = true;
			last_rx_time = HAL_GetTick();
			ESP_ProcessIncomingByte(ch);
		}
	}

	// Wenn nichts kam, einfach zurück
	if (!got_data)
		return;
}

static int16_t ClampInt16(int16_t value, int16_t min, int16_t max) {
	if (value < min) {
		return min;
	}
	if (value > max) {
		return max;
	}
	return value;
}

static int16_t RoundX100ToInt(int16_t value_x100) {
	int32_t value = value_x100;

	if (value < 0) {
		return (int16_t) -(((-value) + 50) / 100);
	}
	return (int16_t) ((value + 50) / 100);
}

static int16_t GetTemperatureC(void) {
	return RoundX100ToInt(slaves[0].temperature_x100) + temperature_offset_c;
}

static int16_t GetHumidityPercent(void) {
	int16_t humidity = RoundX100ToInt(slaves[0].humidity_x100)
			+ humidity_offset_percent;
	return ClampInt16(humidity, 0, 100);
}

static button_event_t Buttons_ReadEvent(void) {
	static button_event_t last_raw = BUTTON_NONE;
	static uint32_t last_change = 0;
	static bool pressed_latched = false;
	button_event_t raw = BUTTON_NONE;
	uint32_t now = HAL_GetTick();

	if (HAL_GPIO_ReadPin(BUTTON_ENTER_PORT, BUTTON_ENTER_PIN) == BUTTON_PRESSED) {
		raw = BUTTON_ENTER;
	} else if (HAL_GPIO_ReadPin(BUTTON_UP_PORT, BUTTON_UP_PIN) == BUTTON_PRESSED) {
		raw = BUTTON_UP;
	} else if (HAL_GPIO_ReadPin(BUTTON_DOWN_PORT, BUTTON_DOWN_PIN) == BUTTON_PRESSED) {
		raw = BUTTON_DOWN;
	}

	if (raw != last_raw) {
		last_raw = raw;
		last_change = now;
	}

	if (raw == BUTTON_NONE) {
		pressed_latched = false;
		return BUTTON_NONE;
	}

	if (pressed_latched || (now - last_change) < BUTTON_DEBOUNCE_MS) {
		return BUTTON_NONE;
	}

	pressed_latched = true;
	return raw;
}

static void UI_BackToMenu(void) {
	ui_state = UI_MENU;
	ui_redraw_requested = true;
}

static void UI_HandleButton(button_event_t event) {
	if (event == BUTTON_NONE) {
		return;
	}

	switch (ui_state) {
	case UI_MENU:
		if (event == BUTTON_UP) {
			menu_index = (menu_index == 0) ? (MENU_ITEM_COUNT - 1) : (menu_index - 1);
		} else if (event == BUTTON_DOWN) {
			menu_index = (menu_index + 1) % MENU_ITEM_COUNT;
		} else if (event == BUTTON_ENTER) {
			if (menu_index == 0) {
				ui_state = UI_MEASUREMENT;
			} else if (menu_index == 1) {
				ui_state = UI_CAL_TEMP;
			} else {
				ui_state = UI_LIMIT_TEMP_MIN;
			}
		}
		break;

	case UI_MEASUREMENT:
		if (event == BUTTON_ENTER) {
			UI_BackToMenu();
			return;
		}
		break;

	case UI_CAL_TEMP:
		if (event == BUTTON_UP) {
			temperature_offset_c = ClampInt16(temperature_offset_c + 1, -20, 20);
		} else if (event == BUTTON_DOWN) {
			temperature_offset_c = ClampInt16(temperature_offset_c - 1, -20, 20);
		} else if (event == BUTTON_ENTER) {
			ui_state = UI_CAL_HUM;
		}
		break;

	case UI_CAL_HUM:
		if (event == BUTTON_UP) {
			humidity_offset_percent = ClampInt16(humidity_offset_percent + 1, -20, 20);
		} else if (event == BUTTON_DOWN) {
			humidity_offset_percent = ClampInt16(humidity_offset_percent - 1, -20, 20);
		} else if (event == BUTTON_ENTER) {
			UI_BackToMenu();
			return;
		}
		break;

	case UI_LIMIT_TEMP_MIN:
		if (event == BUTTON_UP) {
			temperature_min_c = ClampInt16(temperature_min_c + 1, -40,
					temperature_max_c);
		} else if (event == BUTTON_DOWN) {
			temperature_min_c = ClampInt16(temperature_min_c - 1, -40,
					temperature_max_c);
		} else if (event == BUTTON_ENTER) {
			ui_state = UI_LIMIT_TEMP_MAX;
		}
		break;

	case UI_LIMIT_TEMP_MAX:
		if (event == BUTTON_UP) {
			temperature_max_c = ClampInt16(temperature_max_c + 1,
					temperature_min_c, 100);
		} else if (event == BUTTON_DOWN) {
			temperature_max_c = ClampInt16(temperature_max_c - 1,
					temperature_min_c, 100);
		} else if (event == BUTTON_ENTER) {
			ui_state = UI_LIMIT_HUM_MIN;
		}
		break;

	case UI_LIMIT_HUM_MIN:
		if (event == BUTTON_UP) {
			humidity_min_percent = ClampInt16(humidity_min_percent + 1, 0,
					humidity_max_percent);
		} else if (event == BUTTON_DOWN) {
			humidity_min_percent = ClampInt16(humidity_min_percent - 1, 0,
					humidity_max_percent);
		} else if (event == BUTTON_ENTER) {
			ui_state = UI_LIMIT_HUM_MAX;
		}
		break;

	case UI_LIMIT_HUM_MAX:
		if (event == BUTTON_UP) {
			humidity_max_percent = ClampInt16(humidity_max_percent + 1,
					humidity_min_percent, 100);
		} else if (event == BUTTON_DOWN) {
			humidity_max_percent = ClampInt16(humidity_max_percent - 1,
					humidity_min_percent, 100);
		} else if (event == BUTTON_ENTER) {
			UI_BackToMenu();
			return;
		}
		break;
	}

	ui_redraw_requested = true;
}

static void UI_DrawMenu(void) {
	ssd1306_SetCursor(0, 0);
	ssd1306_WriteString("Menu", Font_7x10, White);

	ssd1306_SetCursor(0, 16);
	ssd1306_WriteString(menu_index == 0 ? "> Messung" : "  Messung",
			Font_7x10, White);
	ssd1306_SetCursor(0, 30);
	ssd1306_WriteString(menu_index == 1 ? "> Kalibrieren" : "  Kalibrieren",
			Font_7x10, White);
	ssd1306_SetCursor(0, 44);
	ssd1306_WriteString(menu_index == 2 ? "> Grenzwerte" : "  Grenzwerte",
			Font_7x10, White);
}

static void UI_DrawMeasurement(void) {
	char line[24];
	int16_t temperature_c;
	int16_t humidity_percent;
	uint8_t warning_y = 45;

	ssd1306_SetCursor(0, 0);
	ssd1306_WriteString("Messung", Font_7x10, White);

	if (!slaves[0].valid) {
		ssd1306_SetCursor(0, 20);
		ssd1306_WriteString("Warte auf", Font_11x18, White);
		ssd1306_SetCursor(0, 40);
		ssd1306_WriteString("Daten", Font_11x18, White);
		return;
	}

	temperature_c = GetTemperatureC();
	humidity_percent = GetHumidityPercent();

	snprintf(line, sizeof(line), "T:%d C", temperature_c);
	ssd1306_SetCursor(0, 12);
	ssd1306_WriteString(line, Font_11x18, White);

	snprintf(line, sizeof(line), "H:%d %%", humidity_percent);
	ssd1306_SetCursor(0, 30);
	ssd1306_WriteString(line, Font_11x18, White);

	if (temperature_c < temperature_min_c) {
		ssd1306_SetCursor(0, warning_y);
		ssd1306_WriteString("Temp unter Min", Font_6x8, White);
		warning_y += 9;
	} else if (temperature_c > temperature_max_c) {
		ssd1306_SetCursor(0, warning_y);
		ssd1306_WriteString("Temp ueber Max", Font_6x8, White);
		warning_y += 9;
	}

	if (humidity_percent < humidity_min_percent) {
		ssd1306_SetCursor(0, warning_y);
		ssd1306_WriteString("Feuchte unter Min", Font_6x8, White);
	} else if (humidity_percent > humidity_max_percent) {
		ssd1306_SetCursor(0, warning_y);
		ssd1306_WriteString("Feuchte ueber Max", Font_6x8, White);
	}
}

static void UI_DrawCalibration(void) {
	char line[24];

	ssd1306_SetCursor(0, 0);
	ssd1306_WriteString("Kalibrieren", Font_7x10, White);

	if (ui_state == UI_CAL_TEMP) {
		ssd1306_SetCursor(0, 18);
		ssd1306_WriteString("Temp Offset", Font_7x10, White);
		snprintf(line, sizeof(line), "%+d C", temperature_offset_c);
	} else {
		ssd1306_SetCursor(0, 18);
		ssd1306_WriteString("Feuchte Offset", Font_7x10, White);
		snprintf(line, sizeof(line), "%+d %%", humidity_offset_percent);
	}

	ssd1306_SetCursor(0, 32);
	ssd1306_WriteString(line, Font_11x18, White);
	ssd1306_SetCursor(0, 56);
	ssd1306_WriteString("UP/DOWN  ENTER", Font_6x8, White);
}

static void UI_DrawLimitValue(const char *label, int16_t value,
		const char *unit) {
	char line[24];

	ssd1306_SetCursor(0, 0);
	ssd1306_WriteString("Grenzwerte", Font_7x10, White);
	ssd1306_SetCursor(0, 18);
	ssd1306_WriteString((char*) label, Font_7x10, White);

	snprintf(line, sizeof(line), "%d %s", value, unit);
	ssd1306_SetCursor(0, 32);
	ssd1306_WriteString(line, Font_11x18, White);
	ssd1306_SetCursor(0, 56);
	ssd1306_WriteString("UP/DOWN  ENTER", Font_6x8, White);
}

static void UI_Draw(void) {
	ssd1306_Fill(Black);

	switch (ui_state) {
	case UI_MENU:
		UI_DrawMenu();
		break;
	case UI_MEASUREMENT:
		UI_DrawMeasurement();
		break;
	case UI_CAL_TEMP:
	case UI_CAL_HUM:
		UI_DrawCalibration();
		break;
	case UI_LIMIT_TEMP_MIN:
		UI_DrawLimitValue("Temp Min", temperature_min_c, "C");
		break;
	case UI_LIMIT_TEMP_MAX:
		UI_DrawLimitValue("Temp Max", temperature_max_c, "C");
		break;
	case UI_LIMIT_HUM_MIN:
		UI_DrawLimitValue("Feuchte Min", humidity_min_percent, "%");
		break;
	case UI_LIMIT_HUM_MAX:
		UI_DrawLimitValue("Feuchte Max", humidity_max_percent, "%");
		break;
	}

	ssd1306_UpdateScreen();
}

void UI_Update(void) {
	static uint32_t last_refresh = 0;
	uint32_t now = HAL_GetTick();

	UI_HandleButton(Buttons_ReadEvent());

	if (!ui_redraw_requested && ui_state == UI_MEASUREMENT
			&& (now - last_refresh) >= 500) {
		ui_redraw_requested = true;
	}

	if (ui_redraw_requested) {
		UI_Draw();
		last_refresh = now;
		ui_redraw_requested = false;
	}
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
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  ssd1306_Init();
  UI_Update();

  ESP_Init();
  ESP_StartServer();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  ESP_CheckIncoming();
	  UI_Update();
	  HAL_Delay(10);

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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
  hi2c1.Init.Timing = 0x00201D2B;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
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
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
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
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin : PA11 */
  GPIO_InitStruct.Pin = GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB4 PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
