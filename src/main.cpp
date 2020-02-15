#include <Arduino.h>
#include <BlynkSimpleEsp8266.h>
#include <Ticker.h>
#include <EEPROM.h>
#include <ESP8266mDNS.h>
#include <ESP8266WebServer.h>
#include <ESP8266HTTPUpdateServer.h>
#include <WiFiManager.h>
#include <TickerScheduler.h> // https://github.com/Toshik/TickerScheduler
#include <PubSubClient.h>

// Параметры
#define NAME_DEVICE "Temp_Sens_Vagon" // Имя устройства
#define BUTTON_SYS0_PIN 0             // Назначить кномпку меню и сброса параметров
// Настройки MQTT
#define mqtt_server "tailor.cloudmqtt.com"        // Имя сервера MQTT
#define mqtt_port 11995                           // Порт для подключения к серверу MQTT
#define mqtt_login "xnyqpfbu"                     // Логин от сервер
#define mqtt_password "q94Tbl-0-WxH"              // Пароль от сервера
#define mqtt_topic_temp "/sensors/dht/vagon/temp" // Топик температуры
/* User defines ---------------------------------------------------------*/
#define BLYNK_PRINT Serial
#define LED_SYS_PIN 13
#define BUTTON_SYS_B0_VPIN V20
#define WIFI_SIGNAL_VPIN V80                   // Пин уровня сигнала WiFi
#define INTERVAL_PRESSED_RESET_ESP 5000L       // Время срабатывания перезагрузки
#define INTERVAL_PRESSED_RESET_SETTINGS 30000L // Время срабатывания сброса
#define INTERVAL_PRESSED_SHORT 50
#define INTERVAL_SEND_DATA 30033L
#define INTERVAL_RECONNECT 60407L
#define INTERVAL_REFRESH_DATA 4065L
#define WIFI_MANAGER_TIMEOUT 180
#define EEPROM_SETTINGS_SIZE 512
#define EEPROM_START_SETTING_WM 0
#define EEPROM_SALT_WM 12661
#define LED_SYS_TOGGLE() digitalWrite(LED_SYS_PIN, !digitalRead(LED_SYS_PIN))
#define LED_SYS_ON() digitalWrite(LED_SYS_PIN, LOW)
#define LED_SYS_OFF() digitalWrite(LED_SYS_PIN, HIGH)
/* CODE END UD */

WiFiClient espClient;
PubSubClient client(espClient);
BlynkTimer timer;
Ticker tickerESP8266;
TickerScheduler ts(2); // Планировщик задач (Число задач)
//Declaration OTA WebUpdater
ESP8266WebServer httpServer(80);
ESP8266HTTPUpdateServer httpUpdater;

int startPressBtn = 0;

long lastMsg = 0; // Последнее сообщение MQTT

bool shouldSaveConfigWM = false; //flag for saving data
bool btnSystemState0 = false;
bool triggerBlynkConnect = false;
bool isFirstConnect = true; // Keep this flag not to re-sync on every reconnection

//structure for initial settings. It now takes 116 bytes
typedef struct
{
  char host[33] = NAME_DEVICE;              // 33 + '\0' = 34 bytes
  char blynkToken[33] = "";                 // 33 + '\0' = 34 bytes
  char blynkServer[33] = "blynk-cloud.com"; // 33 + '\0' = 34 bytes
  char blynkPort[6] = "8442";               // 04 + '\0' = 05 bytes
  int salt = EEPROM_SALT_WM;                // 04		 = 04 bytes
} WMSettings;                               // 111 + 1	 = 112 bytes (112 this is a score of 0)
WMSettings wmSettings;
//-----------------------------------------------------------------------------------------

/* CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
static void configModeCallback(WiFiManager *myWiFiManager);
static void saveConfigCallback(void);
static void tick(void);
static void untick(void);
static void readSystemKey(void);
static void timerRefreshData(void);
static void timerSendServer(void);
static void timerReconnect(void);
/* CODE END PFP */

// Запрос данных с MQTT
void callback(char *topic, byte *payload, unsigned int length)
{
  Serial.print("Message arrived [");
  Serial.print(topic); // отправляем в монитор порта название топика
  Serial.print("] ");
  for (int i = 0; i < length; i++)
  { // отправляем данные из топика
    Serial.print((char)payload[i]);
  }
  Serial.println();
}

// Переподключение к MQTT
void reconnect()
{
  while (!client.connected())
  { // крутимся пока не подключемся.
    // создаем случайный идентификатор клиента
    // String clientId = "ESP8266Client-";
    // clientId += String(random(0xffff), HEX);
    String clientId = NAME_DEVICE;
    // подключаемся, в client.connect передаем ID, логин и пасс
    if (client.connect(clientId.c_str(), mqtt_login, mqtt_password))
    {
      client.subscribe(mqtt_topic_temp); // подписываемся на топик, в который же пишем данные
    }
    else
    {
      // иначе ругаемся в монитор порта
    }
  }
}

void setup()
{
  Serial.begin(115200);
  pinMode(BUTTON_SYS0_PIN, INPUT_PULLUP);
  pinMode(LED_SYS_PIN, OUTPUT);

  // Read the WM settings data from EEPROM to RAM
  EEPROM.begin(EEPROM_SETTINGS_SIZE);
  EEPROM.get(EEPROM_START_SETTING_WM, wmSettings);
  EEPROM.end();

  if (wmSettings.salt != EEPROM_SALT_WM)
  {
    //Serial.println(F("Invalid wmSettings in EEPROM, trying with defaults"));
    WMSettings defaults;
    wmSettings = defaults;
  }

  tickerESP8266.attach(0.5, tick); // start ticker with 0.5 because we start in AP mode and try to connect

  WiFiManager wifiManager;
  wifiManager.setConfigPortalTimeout(WIFI_MANAGER_TIMEOUT);
  WiFiManagerParameter custom_device_name_text("<br/>Введите имя устройства<br/>или оставьте как есть<br/>");
  wifiManager.addParameter(&custom_device_name_text);
  WiFiManagerParameter custom_device_name("device-name", "Имя устройства", wmSettings.host, 33);
  wifiManager.addParameter(&custom_device_name);
  WiFiManagerParameter custom_blynk_text("<br/>Настройка Blynk<br/>");
  wifiManager.addParameter(&custom_blynk_text);
  WiFiManagerParameter custom_blynk_token("blynk-token", "Blynk токен", wmSettings.blynkToken, 33);
  wifiManager.addParameter(&custom_blynk_token);
  WiFiManagerParameter custom_blynk_server("blynk-server", "Blynk сервер", wmSettings.blynkServer, 33);
  wifiManager.addParameter(&custom_blynk_server);
  WiFiManagerParameter custom_blynk_port("blynk-port", "Blynk порт", wmSettings.blynkPort, 6);
  wifiManager.addParameter(&custom_blynk_port);
  wifiManager.setSaveConfigCallback(saveConfigCallback);
  wifiManager.setAPCallback(configModeCallback);

  if (wifiManager.autoConnect(wmSettings.host))
  {
    //Serial.println(F("Connected WiFi!"));
  }
  else
  {
    //Serial.println(F("failed to connect and hit timeout"));
  }

  untick(); // cancel the flashing LED

  // Copy the entered values to the structure
  strcpy(wmSettings.host, custom_device_name.getValue());
  strcpy(wmSettings.blynkToken, custom_blynk_token.getValue());
  strcpy(wmSettings.blynkServer, custom_blynk_server.getValue());
  strcpy(wmSettings.blynkPort, custom_blynk_port.getValue());

  if (shouldSaveConfigWM)
  {
    LED_SYS_ON();
    // Записать данные в EEPROM
    EEPROM.begin(EEPROM_SETTINGS_SIZE);
    EEPROM.put(EEPROM_START_SETTING_WM, wmSettings);
    EEPROM.end();
    LED_SYS_OFF();
  }

  // Запуск OTA WebUpdater
  MDNS.begin(wmSettings.host);
  httpUpdater.setup(&httpServer);
  httpServer.begin();
  MDNS.addService("http", "tcp", 80);
  //Serial.printf("HTTPUpdateServer ready! Open http://%s.local/update in your browser\n", wmSettings.host);

  // Настройка подключения к серверу Blynk
  Blynk.config(wmSettings.blynkToken, wmSettings.blynkServer, atoi(wmSettings.blynkPort));

  if (Blynk.connect())
  {
    // something to do if connected
  }
  else
  {
    // something to do if you failed to connect
  }

  timer.setInterval(INTERVAL_REFRESH_DATA, timerRefreshData);
  timer.setInterval(INTERVAL_SEND_DATA, timerSendServer);
  timer.setInterval(INTERVAL_RECONNECT, timerReconnect);
  client.setServer(mqtt_server, mqtt_port); // указываем адрес брокера и порт
  client.setCallback(callback);             // указываем функцию которая вызывается когда приходят данные от брокера
}

void loop()
{
  if (Blynk.connected())
  {
    Blynk.run(); // Инициализация сервера Blynk
  }
  else
  {
    if (!tickerESP8266.active())
    {
      tickerESP8266.attach(2, tick);
    }
  }

  timer.run();               // Инициализация BlynkTimer
  httpServer.handleClient(); // Инициализация OTA WebUpdater
  readSystemKey();
  ts.update(); //планировщик задач

  if (!client.connected())
  {              // проверяем подключение к брокеру
    reconnect(); // еще бы проверить подкючение к wifi...
  }
  client.loop();
}

/* BLYNK CODE BEGIN */
BLYNK_CONNECTED()
{
	untick();
	// Serial.println(F("Blynk Connected!"));
	// Serial.println(F("local ip"));
	// Serial.println(WiFi.localIP());
	char str[32];
	sprintf_P(str, PSTR("%s Online!"), wmSettings.host);
	Blynk.notify(str);
	if (isFirstConnect)
	{
		Blynk.syncAll();
		isFirstConnect = false;
	}
}

BLYNK_WRITE(BUTTON_SYS_B0_VPIN) // Example
{
	//TODO: something to do when a button is clicked in the Blynk app

	//Serial.println(F("System_0 button pressed is App!"));
}
/* BLYNK CODE END */

/* CODE BEGIN USER FUNCTION */
static void timerRefreshData(void)
{
	//TODO: here are functions for updating data from sensors, ADC, etc ...
}

static void timerSendServer(void)
{
	if (Blynk.connected())
	{
		//TODO: here are the functions that send data to the Blynk server
		Blynk.virtualWrite(WIFI_SIGNAL_VPIN, map(WiFi.RSSI(), -105, -40, 0, 100)); // Получаем уровень сигнала Wifi
	}
	else
	{
		//TODO:
	}
}

static void timerReconnect(void)
{
	if (WiFi.status() != WL_CONNECTED)
	{
		/* 		Serial.println(F("WiFi not connected"));
		if (WiFi.begin() == WL_CONNECTED)
		{
			Serial.println(F("WiFi reconnected"));
		}
		else
		{
			Serial.println(F("WiFi not reconnected"));
		} */
	}
	else // if (WiFi.status() == WL_CONNECTED)
	{
		/* 		Serial.println(F("WiFi in connected"));

		if (!Blynk.connected())
		{
			if (Blynk.connect())
			{
				Serial.println(F("Blynk reconnected"));
			}
			else
			{
				Serial.println(F("Blynk not reconnected"));
			}
		}
		else
		{
			Serial.println(F("Blynk in connected"));
		} */
	}
}

static void configModeCallback(WiFiManager *myWiFiManager)
{
	/* 	Serial.println(F("Entered config mode"));
	Serial.println(WiFi.softAPIP());
	//if you used auto generated SSID, print it
	Serial.println(myWiFiManager->getConfigPortalSSID());
	//entered config mode, make led toggle faster */
	tickerESP8266.attach(0.2, tick);
}

//callback notifying us of the need to save config
static void saveConfigCallback()
{
	//Serial.println(F("Should save config"));
	shouldSaveConfigWM = true;
}

static void tick(void)
{
	//toggle state
	LED_SYS_TOGGLE(); // set pin to the opposite state
}

static void untick(void)
{
	tickerESP8266.detach();
	LED_SYS_OFF(); //keep LED off
}

static void readSystemKey(void)
{
	if (!digitalRead(BUTTON_SYS0_PIN) && !btnSystemState0)
	{
		btnSystemState0 = true;
		startPressBtn = millis();
	}
	else if (digitalRead(BUTTON_SYS0_PIN) && btnSystemState0)
	{
		btnSystemState0 = false;
		int pressTime = millis() - startPressBtn;

		if (pressTime > INTERVAL_PRESSED_RESET_ESP && pressTime < INTERVAL_PRESSED_RESET_SETTINGS)
		{
			if (Blynk.connected())
			{
				Blynk.notify(String(wmSettings.host) + F(" reboot!"));
			}
			Blynk.disconnect();
			tickerESP8266.attach(0.1, tick);
			delay(2000);
			ESP.restart();
		}
		else if (pressTime > INTERVAL_PRESSED_RESET_SETTINGS)
		{
			if (Blynk.connected())
			{
				Blynk.notify(String(wmSettings.host) + F(" setting reset! Connected WiFi AP this device!"));
			}
			WMSettings defaults;
			wmSettings = defaults;

			LED_SYS_ON();
			// We write the default data to EEPROM
			EEPROM.begin(EEPROM_SETTINGS_SIZE);
			EEPROM.put(EEPROM_START_SETTING_WM, wmSettings);
			EEPROM.end();
			//------------------------------------------
			LED_SYS_OFF();

			delay(1000);
			WiFi.disconnect();
			delay(1000);
			ESP.restart();
		}
		else if (pressTime < INTERVAL_PRESSED_RESET_ESP && pressTime > INTERVAL_PRESSED_SHORT)
		{
			//Serial.println(F("System button_0 pressed is Device!"));
			// TODO: insert here what will happen when you press the ON / OFF button
		}
		else if (pressTime < INTERVAL_PRESSED_SHORT) // Коротное нажатие
		{
			// Serial.printf("Fixed false triggering %ims", pressTime);
			// Serial.println();
		}
	}
}
/* CODE END USER FUNCTION */