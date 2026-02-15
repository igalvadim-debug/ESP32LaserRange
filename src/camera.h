#pragma once

// ===================
// Select camera model
// ===================
//#define CAMERA_MODEL_WROVER_KIT // Has PSRAM
//#define CAMERA_MODEL_ESP_EYE // Has PSRAM
//#define CAMERA_MODEL_ESP32S3_EYE // Has PSRAM
//#define CAMERA_MODEL_M5STACK_PSRAM // Has PSRAM
//#define CAMERA_MODEL_M5STACK_V2_PSRAM // M5Camera version B Has PSRAM
//#define CAMERA_MODEL_M5STACK_WIDE // Has PSRAM
//#define CAMERA_MODEL_M5STACK_ESP32CAM // No PSRAM
//#define CAMERA_MODEL_M5STACK_UNITCAM // No PSRAM
#define CAMERA_MODEL_AI_THINKER // Has PSRAM
//#define CAMERA_MODEL_TTGO_T_JOURNAL // No PSRAM
// ** Espressif Internal Boards **
//#define CAMERA_MODEL_ESP32_CAM_BOARD
//#define CAMERA_MODEL_ESP32S2_CAM_BOARD
//#define CAMERA_MODEL_ESP32S3_CAM_LCD

#include "camera_pins.h"
#include "vec2.h"
#include <JPEGDEC.h>
#include <Preferences.h>

// ============================================
// НАСТРАИВАЕМЫЕ ПАРАМЕТРЫ (сохраняются в EEPROM)
// ============================================
struct DetectionSettings
{
	int autoExposureLevel;      // -2 to +2, рекомендуется -1
	int brightnessThreshold;    // 0-255, порог яркости для детекции (рекомендуется 240-250)
	int maxSpotSize;            // 1-200, максимальный размер яркого пятна в пикселях (рекомендуется 10-30)
	int minBrightnessChange;    // 0-100, минимальное изменение яркости между кадрами (рекомендуется 30-50)
	int minFramesBetweenHits;   // 1-100, минимальное количество кадров между попаданиями (рекомендуется 3-10)
	
	// Дефолтные значения
	DetectionSettings() :
		autoExposureLevel(-1),
		brightnessThreshold(245),
		maxSpotSize(20),
		minBrightnessChange(40),
		minFramesBetweenHits(5)
	{}
};

DetectionSettings g_settings;
framesize_t g_frameMode = FRAMESIZE_240X240;

// ============================================
// СОХРАНЕНИЕ/ЗАГРУЗКА НАСТРОЕК
// ============================================
void saveDetectionSettings()
{
	Preferences prefs;
	prefs.begin("detection", false);
	prefs.putBytes("settings", &g_settings, sizeof(g_settings));
	prefs.end();
	Serial.println("Settings saved");
}

void loadDetectionSettings()
{
	Preferences prefs;
	prefs.begin("detection", true);
	size_t len = prefs.getBytesLength("settings");
	if (len == sizeof(g_settings))
	{
		prefs.getBytes("settings", &g_settings, sizeof(g_settings));
		Serial.println("Settings loaded");
	}
	else
	{
		Serial.println("Using default settings");
	}
	prefs.end();
}

void applyExposureLevel()
{
	sensor_t* s = esp_camera_sensor_get();
	if (s) s->set_ae_level(s, g_settings.autoExposureLevel);
}

bool initCamera()
{
	camera_config_t config;

	config.ledc_channel = LEDC_CHANNEL_0;
	config.ledc_timer = LEDC_TIMER_0;
	config.pin_d0 = Y2_GPIO_NUM;
	config.pin_d1 = Y3_GPIO_NUM;
	config.pin_d2 = Y4_GPIO_NUM;
	config.pin_d3 = Y5_GPIO_NUM;
	config.pin_d4 = Y6_GPIO_NUM;
	config.pin_d5 = Y7_GPIO_NUM;
	config.pin_d6 = Y8_GPIO_NUM;
	config.pin_d7 = Y9_GPIO_NUM;
	config.pin_xclk = XCLK_GPIO_NUM;
	config.pin_pclk = PCLK_GPIO_NUM;
	config.pin_vsync = VSYNC_GPIO_NUM;
	config.pin_href = HREF_GPIO_NUM;
	config.pin_sscb_sda = SIOD_GPIO_NUM;
	config.pin_sscb_scl = SIOC_GPIO_NUM;
	config.pin_pwdn = PWDN_GPIO_NUM;
	config.pin_reset = RESET_GPIO_NUM;
	config.xclk_freq_hz = 24000000;

	config.pixel_format = PIXFORMAT_JPEG;
	config.grab_mode = CAMERA_GRAB_LATEST;
	config.fb_location = CAMERA_FB_IN_PSRAM;
	config.jpeg_quality = 4;
	config.fb_count = 3;
	config.frame_size = g_frameMode;

	esp_err_t err = esp_camera_init(&config);
	if (err != ESP_OK)
	{
		Serial.printf("Camera init failed with error 0x%x", err);
		while (true) delay(1000);
	}

	delay(200);

	sensor_t* s = esp_camera_sensor_get();
	s->set_whitebal(s, 0);
	s->set_awb_gain(s, 0);
	s->set_ae_level(s, g_settings.autoExposureLevel);
	s->set_aec2(s, 0);
	s->set_saturation(s, -2);
	s->set_special_effect(s, 2); // grayscale
	s->set_lenc(s, 1);
	s->set_wpc(s, 0);
	s->set_bpc(s, 0);
	s->set_raw_gma(s, 1);

	return true;
}

// ============================================
// УЛУЧШЕННАЯ ДЕТЕКЦИЯ С ФИЛЬТРАЦИЕЙ
// ============================================

// Буфер для хранения предыдущего кадра (для детекции изменений)
uint8_t* g_prevFrame = nullptr;
size_t g_prevFrameSize = 0;
int g_frameWidth = 0;
int g_frameHeight = 0;
int g_framesSinceLastHit = 1000; // большое значение для первого кадра

// Временные переменные для обработки
size_t g_count = 0;
size_t g_xPos = 0;
size_t g_yPos = 0;
uint8_t* g_currentFrameBuffer = nullptr;

int colbackMCU(JPEGDRAW* pDraw)
{
	uint8_t* src = (uint8_t*)pDraw->pPixels;
	
	for (int dy = 0; dy < pDraw->iHeight; dy++)
	{
		for (int dx = 0; dx < pDraw->iWidth; dx++)
		{
			int x = pDraw->x + dx;
			int y = pDraw->y + dy;
			int idx = y * g_frameWidth + x;
			
			uint8_t currentBright = *src++;
			
			// Сохраняем в текущий буфер
			if (g_currentFrameBuffer && idx < g_prevFrameSize)
			{
				g_currentFrameBuffer[idx] = currentBright;
			}
			
			// Проверяем яркость относительно порога
			if (currentBright > g_settings.brightnessThreshold)
			{
				// Если есть предыдущий кадр, проверяем изменение яркости
				if (g_prevFrame && idx < g_prevFrameSize)
				{
					uint8_t prevBright = g_prevFrame[idx];
					int change = (int)currentBright - (int)prevBright;
					
					// Детектируем только НОВЫЕ яркие пятна (вспышки)
					if (change >= g_settings.minBrightnessChange)
					{
						g_count++;
						g_xPos += x;
						g_yPos += y;
					}
				}
				else
				{
					// Для первого кадра (нет истории)
					g_count++;
					g_xPos += x;
					g_yPos += y;
				}
			}
		}
	}
	return 1; // continue
}

JPEGDEC jpeg;

bool findShot(camera_fb_t* fb, vec2& pos)
{
	g_count = 0;
	g_xPos = 0;
	g_yPos = 0;
	
	// Инициализация буферов при первом запуске
	if (!g_prevFrame)
	{
		g_frameWidth = fb->width;
		g_frameHeight = fb->height;
		g_prevFrameSize = g_frameWidth * g_frameHeight;
		g_prevFrame = (uint8_t*)heap_caps_malloc(g_prevFrameSize, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
		g_currentFrameBuffer = (uint8_t*)heap_caps_malloc(g_prevFrameSize, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
		
		if (!g_prevFrame || !g_currentFrameBuffer)
		{
			Serial.println("Failed to allocate frame buffers!");
			return false;
		}
		
		memset(g_prevFrame, 0, g_prevFrameSize);
		memset(g_currentFrameBuffer, 0, g_prevFrameSize);
	}
	
	// Декодируем JPEG и ищем яркие пятна
	jpeg.openRAM(fb->buf, fb->len, colbackMCU);
	jpeg.setPixelType(EIGHT_BIT_GRAYSCALE);
	jpeg.decode(0, 0, 0);
	jpeg.close();
	
	g_framesSinceLastHit++;
	
	// КРИТЕРИИ ДЕТЕКЦИИ ВЫСТРЕЛА:
	// 1. Есть яркие пиксели (g_count > 0)
	// 2. Размер пятна не слишком большой (защита от засветки всего кадра)
	// 3. Размер пятна не слишком маленький (защита от одиночных шумных пикселей)
	// 4. Прошло достаточно кадров с последнего попадания
	
	bool shotDetected = false;
	
	if (g_count > 0 && 
	    g_count <= g_settings.maxSpotSize && 
	    g_count >= 1 &&
	    g_framesSinceLastHit >= g_settings.minFramesBetweenHits)
	{
		pos.x = (float)g_xPos / ((float)g_count * (float)fb->width);
		pos.y = (float)g_yPos / ((float)g_count * (float)fb->height);
		
		Serial.printf("Shot detected! Pixels: %d, Pos: %.3f %.3f\n", g_count, pos.x, pos.y);
		
		shotDetected = true;
		g_framesSinceLastHit = 0;
	}
	
	// Копируем текущий кадр в предыдущий для следующей итерации
	if (g_prevFrame && g_currentFrameBuffer)
	{
		memcpy(g_prevFrame, g_currentFrameBuffer, g_prevFrameSize);
	}
	
	return shotDetected;
}
