//
// Officine Giardino
// Fablab Frosinone 2014
// DEV ermanno@officinegiardino.org
// V. 0.9.5_17 - Febbraio 2016
// --> officinegiardino.org
// Release: Public Domain 
//

#include "my_id.h"								// dev_guid[]  id del dispositivo
#include <MiniTimerC.h>							// 
#include <avr/wdt.h>							// watch dog
#include <avr/pgmspace.h>
#include <EtherCard.h>							// Ethernet
#include <DHT.h>								// 
#include <ArduinoJson.h>

// Prescaler per ridurre il tempo di analogread (ARDUINO DEFAULT=128)
// http://www.microsmart.co.za/technical/2014/03/01/advanced-arduino-adc/
// Define various ADC prescaler
const unsigned char PS_16 = (1 << ADPS2);
const unsigned char PS_32 = (1 << ADPS2) | (1 << ADPS0);
const unsigned char PS_64 = (1 << ADPS2) | (1 << ADPS1);
const unsigned char PS_128 = (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);

// #define USE_SERIAL_SYS_DEBUG					// Serial debug system
// #define USE_SERIAL_ETH_DEBUG					// Serial debug for Ethernet
// #define USE_SERIAL_IP_DEBUG						// Serial debug for IP and Network
// #define USE_SERIAL_HTTP_SEND_DEBUG				// Serial debug for HTTP Send
// #define USE_SERIAL_HTTP_REPLAY_DEBUG			// Serial debug for HTTP Replay from server
// #define USE_SERIAL_DATA_DEBUG					// Serial debug for PIN data

#define POST_HTTP_DATA							// Commentare solo per debug seriale locale

const uint32_t DFT_INTERVAL = 60000;			// Intervallo di tempo tra lo stesso sensore

const uint8_t NUMBER_OF_TIMERS = 11;			// 	

MiniTimerC timerc[NUMBER_OF_TIMERS];

const uint8_t HTTP_CHECK_TIMER_ID = 0;						// Timer id 
const uint32_t HTTP_WAIT_INTERVAL = 10000;					// Timer ms for wait http to send next value
const uint8_t DUST10_CHECK_TIMER_ID = 1;					// Timer id 
const uint32_t DUST10_WAIT_INTERVAL = DFT_INTERVAL;			// Timer ms bwtween send this sensor
const uint8_t DUST01_CHECK_TIMER_ID = 2;					// Timer id
const uint32_t DUST01_WAIT_INTERVAL = DFT_INTERVAL;			// Timer ms bwtween send this sensor
const uint8_t DHT_TEMP_CHECK_TIMER_ID = 3;					// Timer id
const uint32_t DHT_TEMP_WAIT_INTERVAL = DFT_INTERVAL;		// Timer ms bwtween send this sensor 
const uint8_t DHT_WET_CHECK_TIMER_ID = 4;					// Timer id
const uint32_t DHT_WET_WAIT_INTERVAL = DFT_INTERVAL;		// TTimer ms bwtween send this sensor 
const uint8_t MQ135_CHECK_TIMER_ID = 5;						// Timer id
const uint32_t MQ135_WAIT_INTERVAL = DFT_INTERVAL;			// Timer ms bwtween send this sensor
const uint8_t MQ136_CHECK_TIMER_ID = 6;						// Timer id
const uint32_t MQ136_WAIT_INTERVAL = DFT_INTERVAL;			// Timer ms bwtween send this sensor
const uint8_t MQ138_CHECK_TIMER_ID = 7;						// Timer id
const uint32_t MQ138_WAIT_INTERVAL = DFT_INTERVAL;			// Timer ms bwtween send this sensor
const uint8_t MPX4115A_CHECK_TIMER_ID = 8;					// Timer id
const uint32_t MPX4115A_WAIT_INTERVAL = DFT_INTERVAL;		// Timer ms bwtween send this sensor
const uint8_t SYSDEBUG_CHECK_TIMER_ID = 9;					// Timer id
const uint32_t SYSDEBUG_WAIT_INTERVAL = 15000L;				// Timer ms for wait 
const uint8_t REPLAY_CHECK_TIMER_ID = 10;					// Timer id 
const uint32_t REPLAY_WAIT_INTERVAL = 120000U;				// Timer ms for wait http replay timeout reset device after timeout

// PM Variabili e Costanti Comuni
const uint16_t DUST_SAMPLE_TIME = 280;				// sampling time duration microseconds from datasheet
const uint16_t DUST_DELTA_TIME = 40;				// rising and falling time - 16 è il tempo di analogRead con #fastread abilitato
const uint16_t DUST_SLEEPING_TIME = 9680;			// sleep time from sharp datasheet

// setting DHT11/DHT22  Humidity & Temperature Sensor
// DHT11 only returns integers(e.g. 20) and does not support negative values.
// TEMP
const boolean DHT_TEMP_ENABLED = true;
const byte DHT_TEMP_VALUE_ID = 1;			// ID mysql tabella crasit_parameters
const byte DHT_TEMP_APIN = A1;					// Humidity & Temperature Sensor
const uint8_t DHT_TEMP_SAMPLE_COUNT = 8;		// Numero di letture del campione 

// setting DHT11/DHT22  Humidity & Temperature Sensor
// DHT11 only returns integers(e.g. 20) and does not support negative values.
// IGRO
const boolean DHT_WET_ENABLED = true;
const byte DHT_WET_VALUE_ID = 2;				// ID mysql tabella crasit_parameters
const byte DHT_WET_APIN = A1;					// Humidity & Temperature Sensor
const uint8_t DHT_WET_SAMPLE_COUNT = 8;		// Numero di letture del campione 

// Sharp GP2Y1010AU0F							dust sensor 0 PM 10um
// http://www.sharp-world.com/products/device/lineup/data/pdf/datasheet/gp2y1010au_appl_e.pdf
// IN seguito analizzare anche sharp GP2U05 e GP2U06
const boolean DUST10_ENABLED = true;	
const byte DUST10_VALUE_ID = 8;					// ID mysql tabella crasit_parameters
const byte DUST10_MUX_ADDRESS = 6;				// dust sensor to Arduino Analog pin
const uint8_t DUST10_SAMPLE_COUNT = 8;			// Numero di letture del campione 
const byte DUST10_IRLED = 7;					// irled driver pins of dust sensor to Arduino Digital 

// Sharp GP2Y1010AU0F							dust sensor 1 PM 1um
// http://www.sharp-world.com/products/device/lineup/data/pdf/datasheet/gp2y1010au_appl_e.pdf
// IN seguito analizzare anche sharp GP2U05 e GP2U06
const boolean DUST01_ENABLED = true;
const byte DUST01_VALUE_ID = 9;					// ID mysql tabella crasit_parameters
const byte DUST01_MUX_ADDRESS = 2;				// dust sensor to Arduino Analog pin
const uint8_t DUST01_SAMPLE_COUNT = 8;			// Numero di letture del campione 
const byte DUST01_IRLED = 6;					// irled driver pins of dust sensor to Arduino Digital 

// Prima di leggere i sensori MQ si deve essere a conoscenza di temperatura ed umidità 
// per le relative correzioni
// setting MQ-135  Semiconductor Sensor for Benzene, Alcohol, smoke and CO2
// http://davidegironi.blogspot.it/2014/01/cheap-co2-meter-using-mq135-sensor-with.html
const boolean MQ135_ENABLED = true;
const byte MQ135_VALUE_ID = 10;					// ID mysql tabella crasit_parameters
const byte MQ135_MUX_ADDRESS = 4;				// gas sensor to Arduino Analog pin
const uint8_t MQ135_SAMPLE_COUNT = 8;			// Numero di letture del campione 

// setting MQ-136  Semiconductor Sensor for SO2
// http://china-total.com/Product/meter/gas-sensor/MQ136.pdf
const boolean MQ136_ENABLED = true;
const byte MQ136_VALUE_ID = 11;					// ID mysql tabella crasit_parameters
const byte MQ136_MUX_ADDRESS = 5;				// gas sensor to Arduino Analog pin
const uint8_t MQ136_SAMPLE_COUNT = 8;			// Numero di letture del campione 

// setting MQ-138  Semiconductor Sensor for Hexane, Benzene, NH3, alcohol, smoke, CO, etc
const boolean MQ138_ENABLED = true;
const byte MQ138_VALUE_ID = 12;					// ID mysql tabella crasit_parameters
const byte MQ138_MUX_ADDRESS = 6;				// gas sensor to Arduino Analog pin
const uint8_t MQ138_SAMPLE_COUNT = 8;			// Numero di letture del campione 

// setting MPX4115A  INTEGRATED PRESSURE SENSOR 15 to 115 kPa(2.2 to 16.7 psi) 0.2 to 4.8 V Output
const boolean MPX4115A_ENABLED = true;
const byte MPX4115A_VALUE_ID = 3;				// ID mysql tabella crasit_parameters
const byte MPX4115A_MUX_ADDRESS = 7;			// barometric sensor to Arduino Analog pin
const uint8_t MPX4115_SAMPLE_COUNT = 64;				// Sample average reduces the noise

// MUX 4051 Setup
const byte MUX_BIT0 = 7;						// Low bit selector -  (D-PIN)
const byte MUX_BIT1 = 8;						// ..
const byte MUX_BIT2 = 9;						// High bit selector - (D-PIN)
// Analog Pin Input
const byte MUX_APIN = 0;						// Analog Pin 

DHT dht(DHT_TEMP_APIN, DHT11);					// classe DHT

#define sw_reset() wdt_disable();  wdt_enable(WDTO_1S); while(1) {}		// Watch Dog 

const byte NODE_ID = 0xA9;
// ethernet interface mac address, must be unique on the LAN
static byte def_mac[] = { 0x00, 0x04, 0xA3, 0x11, 0x11, NODE_ID };		// 00:04:A3 = Microchip
// the buffersize must be relatively large for DHCP to work; when
// using static setup a buffer size of 100 is sufficient;
#define ETH_BUF_SIZE 640
byte Ethernet::buffer[ETH_BUF_SIZE];

// Web
const char web_site[] PROGMEM = "smartidea.flx.it";
const char web_path[] PROGMEM = "/datac/receive";
uint32_t dev_seq = 0;													// sequenza invio dispositivo

/* struttura post
"usr_guid"	// User Guid
"dev_guid"	// Device Guid
"dev_seq"	// Device sequence
"par_sku"	// Parameter Sku
"par_val"	// Parameter Value
*/

Stash stash;					// Meccanismo Stash http://jeelabs.org/2012/04/11/ethercard-improvements/
static byte session = 255;		// id sessione classe Stash 

byte last_sensor_read = MPX4115A_VALUE_ID; 	

// Opt
// const byte ACT_LED = 3;							// Non usare pin 2,10,11,12,13 utilizzati dalla ethernet

char str_data[8];									// Array valori post parametro


void setup () {

#ifdef USE_SERIAL_SYS_DEBUG
	Serial.begin(115200);
	Serial.println(F("Start... "));
#endif

	timerc[HTTP_CHECK_TIMER_ID].ExpireIn(HTTP_WAIT_INTERVAL);
	timerc[DUST10_CHECK_TIMER_ID].ExpireIn(DUST10_WAIT_INTERVAL);
	timerc[DUST01_CHECK_TIMER_ID].ExpireIn(DUST01_WAIT_INTERVAL);
	timerc[DHT_TEMP_CHECK_TIMER_ID].ExpireIn(DHT_TEMP_WAIT_INTERVAL);
	timerc[DHT_WET_CHECK_TIMER_ID].ExpireIn(DHT_WET_WAIT_INTERVAL);
	timerc[MQ135_CHECK_TIMER_ID].ExpireIn(MQ135_WAIT_INTERVAL);
	timerc[MQ136_CHECK_TIMER_ID].ExpireIn(MQ136_WAIT_INTERVAL);
	timerc[MQ138_CHECK_TIMER_ID].ExpireIn(MQ138_WAIT_INTERVAL);
	timerc[MPX4115A_CHECK_TIMER_ID].ExpireIn(MPX4115A_WAIT_INTERVAL);
	timerc[SYSDEBUG_CHECK_TIMER_ID].ExpireIn(SYSDEBUG_WAIT_INTERVAL);
	timerc[REPLAY_CHECK_TIMER_ID].ExpireIn(REPLAY_WAIT_INTERVAL);
	
	// Velocizzaione analogRead() con prescaler 
	// http://forum.arduino.cc/index.php?topic=6549.0
	//
	// set up the ADC
	ADCSRA &= ~PS_128;  // remove bits set by Arduino library
	ADCSRA |= PS_16;    // set our own prescaler to ...

	// impostazione pin 
	pinMode(DUST10_IRLED, OUTPUT);
	digitalWrite(DUST10_IRLED, HIGH);						// DUST10 power the LED OFF

	pinMode(DUST01_IRLED, OUTPUT);
	digitalWrite(DUST01_IRLED, HIGH);						// DUST01 power the LED OFF

	pinMode(MUX_BIT0, OUTPUT);
	pinMode(MUX_BIT1, OUTPUT);
	pinMode(MUX_BIT2, OUTPUT);

	// pinMode(ACT_LED, OUTPUT);

	internet_setup();
	Stash::initMap(56);
	dht.begin();

// watchdog
wdt_disable();
wdt_enable(WDTO_8S);

}

void loop () {

#ifdef USE_SERIAL_HTTP_REPLAY_DEBUG
	StaticJsonBuffer<64>  jsonBuffer;					// 
#endif
	ether.packetLoop(ether.packetReceive());
	delay(100);
	const char* reply = ether.tcpReply(session);

#ifdef POST_HTTP_DATA
	if (reply != 0)
	{
#ifdef USE_SERIAL_HTTP_REPLAY_DEBUG
		Serial.print(F("Reply Wait: "));
		Serial.println(timerc[REPLAY_CHECK_TIMER_ID].TimeElapsed(), DEC);
#endif
		// Reset Timer Replay
		timerc[REPLAY_CHECK_TIMER_ID].Start();

#ifdef USE_SERIAL_HTTP_REPLAY_DEBUG
		// Serial.println(F(""));
		Serial.println(F("<<< reply from server "));
		// Serial.println(reply);
		// Serial.println(F(""));
		char *pos = strstr((const char*)reply, "\r\n\r\n"); // search for CRLF CRLF 
		Serial.println(pos + 4);

		JsonObject& root = jsonBuffer.parseObject(pos);
		if (!root.success())
		{
			Serial.println("parseObject() failed");
		}
		else
		{
			const char* code = root["code"];
			const char* message = root["message"];
			Serial.print(F("Code: "));
			Serial.println(code);
			Serial.print(F("message: "));
			Serial.println(message);
		}
		Serial.print(F("Session: "));
		Serial.println(session);

#endif  
	}
	else
	{
		if (timerc[REPLAY_CHECK_TIMER_ID].IsExpired()) {

#ifdef USE_SERIAL_HTTP_REPLAY_DEBUG
			Serial.println(F("Reply Timeout"));
			Serial.println(F("Reset..."));
#endif
			sw_reset();

		}
	}
#endif

boolean sensor_is_read = false;						// non esegue il post se falso (primo ciclo)

	// Invia i dati di un sensore alla volta ed aspetta fino a 
	// HTTP_WAIT_INTERVAL prima di inviare il prossimo valore
if (timerc[HTTP_CHECK_TIMER_ID].IsExpired()) 
	{
	timerc[HTTP_CHECK_TIMER_ID].Start();
	switch (last_sensor_read) 
	{
	// Lettura Temperatura
	case(MPX4115A_VALUE_ID) :
	{
		if (timerc[DHT_TEMP_CHECK_TIMER_ID].IsExpired())
		{
			timerc[DHT_TEMP_CHECK_TIMER_ID].Start();
			read_dhtxx_temp();
			sensor_is_read = true;
			last_sensor_read = DHT_TEMP_VALUE_ID;		// Imposta ultimo sensore inviato
		}
	}
	break;
	// Lettura Umidità
	case(DHT_TEMP_VALUE_ID) :
	{
		if (timerc[DHT_WET_CHECK_TIMER_ID].IsExpired())
		{
			timerc[DHT_WET_CHECK_TIMER_ID].Start();
			read_dhtxx_wet();
			sensor_is_read = true;
			last_sensor_read = DHT_WET_VALUE_ID;
		}
	}
	break;
	//Lettura PM 10um
	case(DHT_WET_VALUE_ID) :
	{
		if (timerc[DUST10_CHECK_TIMER_ID].IsExpired())
		{
			timerc[DUST10_CHECK_TIMER_ID].Start();
			read_dust10();
			sensor_is_read = true;
			last_sensor_read = DUST10_VALUE_ID;
		}
	}
	break;
	//Lettura PM 1um
	case(DUST10_VALUE_ID) :
	{
		if (timerc[DUST01_CHECK_TIMER_ID].IsExpired())
		{
			timerc[DUST01_CHECK_TIMER_ID].Start();
			read_dust01();
			sensor_is_read = true;
			last_sensor_read = DUST01_VALUE_ID;
		}
	}
	break;

	//Lettura CO2
	case(DUST01_VALUE_ID) :
	{
		if (timerc[MQ135_CHECK_TIMER_ID].IsExpired())
		{
			timerc[MQ135_CHECK_TIMER_ID].Start();
			read_mq135();
			sensor_is_read = true;
			last_sensor_read = MQ135_VALUE_ID;
		}
	}
	break;

	//Lettura SO2
	case(MQ135_VALUE_ID) :
	{
		if (timerc[MQ136_CHECK_TIMER_ID].IsExpired())
		{
			timerc[MQ136_CHECK_TIMER_ID].Start();
			read_mq136();
			sensor_is_read = true;
			last_sensor_read = MQ136_VALUE_ID;
		}
	}
	break;

	//Lettura VOC
	case(MQ136_VALUE_ID) :
	{
		if (timerc[MQ138_CHECK_TIMER_ID].IsExpired())
		{
			timerc[MQ138_CHECK_TIMER_ID].Start();
			read_mq138();
			sensor_is_read = true;
			last_sensor_read = MQ138_VALUE_ID;
		}
	}
	break;
						 //Lettura VOC
	case(MQ138_VALUE_ID) :
	{
		if (timerc[MPX4115A_CHECK_TIMER_ID].IsExpired())
		{
			timerc[MPX4115A_CHECK_TIMER_ID].Start();
			read_mpx4115a();
			sensor_is_read = true;
			last_sensor_read = MPX4115A_VALUE_ID;
		}
	}
	break;


	default:
	break;
	}
	
	if (sensor_is_read) 
	{
		post_data(last_sensor_read, str_data);		// 3) Invia
		sensor_is_read = false;
	}


	}


#ifdef USE_SERIAL_SYS_DEBUG
	if (timerc[SYSDEBUG_CHECK_TIMER_ID].IsExpired()) {
		timerc[SYSDEBUG_CHECK_TIMER_ID].Start();
		Serial.print(F("Free mem: "));
		Serial.println(free_ram());
		Serial.print(F("Ultimo Sensore Inviato: "));
		Serial.println(last_sensor_read);
	}
#endif // USE_SERIAL_SYS_DEBUG

	wdt_reset();		// reset watchdog
}

void post_data(byte parameter_value_id, char* parameter_value) {

#ifdef POST_HTTP_DATA
	dev_seq++;
#ifdef USE_SERIAL_HTTP_SEND_DEBUG
	Serial.print(F("Post Data: "));
	Serial.println(parameter_value);
#endif

	stash.cleanup();
	uint8_t sd = stash.create();

	stash.print(F("usr_guid="));
	stash.print(usr_guid);
	stash.print(F("&dev_guid="));
	stash.print(dev_guid);
	stash.print(F("&dev_seq="));
	stash.print(dev_seq);
	stash.print(F("&par_sku="));
	stash.print(parameter_value_id);
	stash.print(F("&par_val="));
	stash.println(parameter_value);
	stash.save();

	// Compose the http POST request, taking the headers below and appending
	// previously created stash in the sd holder.
	Stash::prepare(PSTR("POST http://$F/$F HTTP/1.1" "\r\n"
						"Host: $F" "\r\n"
						"Content-Length: $D" "\r\n"
						"Content-Type: application/x-www-form-urlencoded" "\r\n"
						"\r\n"
						"$H"), web_site, web_path, web_site, stash.size(), sd);
  
	// send the packet - this also releases all stash buffers once done
	// Save the session ID so we can watch for it in the main loop.
	session = ether.tcpSend();
	uint8_t freeCount = stash.freeCount();
	if (freeCount <= 3) { Stash::initMap(56); }

#ifdef USE_SERIAL_ETH_DEBUG
	Serial.print(F("Stash Ram Free: "));
	Serial.println(stash.freeCount());
	Serial.print(F("Stash Size: "));
	Serial.println(stash.size());
#endif

#ifdef USE_SERIAL_HTTP_SEND_DEBUG
	Serial.print(F("Dev Seq: "));
	Serial.println(dev_seq);
#endif

	delay(100);

#endif
}

// Temperatura
void read_dhtxx_temp() {
	uint8_t i = 0;
	boolean read_again = true;
	float dhtxx_temp = 0;
	while (read_again)
	{
		i++;
		dhtxx_temp += (float)dht.readTemperature();
		if ((i >= DHT_TEMP_SAMPLE_COUNT)) { read_again = false; }
	}
	dhtxx_temp = ((float)dhtxx_temp / (float)DHT_TEMP_SAMPLE_COUNT) + (float)DHTXX_TEMP_ZERO;
	dtostrf(dhtxx_temp, 7, 2, str_data);				// 2) Converte in stringa per HTTP POST
#ifdef USE_SERIAL_DATA_DEBUG 
	Serial.print(F("DHT_TEMP 1WIRE: "));
	Serial.println(str_data);
#endif
}

// Umidità
void read_dhtxx_wet() {
	uint8_t i = 0;
	boolean read_again = true;
	float dhtxx_wet = 0;
	while (read_again)
	{
		i++;
		dhtxx_wet += (float)dht.readHumidity();
		if ((i >= DHT_WET_SAMPLE_COUNT)) { read_again = false; }
	}
	dhtxx_wet = ((float)dhtxx_wet / (float)DHT_WET_SAMPLE_COUNT) + (float)DHTXX_WET_ZERO;
	dtostrf(dhtxx_wet, 7, 2, str_data);
#ifdef USE_SERIAL_DATA_DEBUG 
	Serial.print(F("DHT_WET 1WIRE: "));
	Serial.println(str_data);
#endif
}

// Polveri 10um
void read_dust10() {
	mux_set(DUST10_MUX_ADDRESS);
	uint8_t i = 0;
	boolean read_again = true;
	uint16_t dust10_adc_value = 0;
	float dust10_volt_calculated;
	float dust10_density;

	while (read_again) 
	{
		i++;
		// Non inserire righe di codice da quì
		digitalWrite(DUST10_IRLED, LOW);						// power on the LED
		delayMicroseconds(DUST_SAMPLE_TIME);
		dust10_adc_value += analogRead(DUST10_MUX_ADDRESS);				// read the analog value
		delayMicroseconds(DUST_DELTA_TIME);
		digitalWrite(DUST10_IRLED, HIGH);						// turn the LED off
		delayMicroseconds(DUST_SLEEPING_TIME);
		// a quì
		if ((i >= DUST10_SAMPLE_COUNT)) { read_again = false; }
	}
	// Media
	dust10_volt_calculated = ((float) (dust10_adc_value / (float) i) * 5.0) / 1024.0;
 
	// linear eqaution taken from http://www.howmuchsnow.com/arduino/airquality/
	// Chris Nafis (c) 2012
	// dust10_density = (0.17 * dust10_volt_calculated) - 0.1;
	dust10_density = (dust10_volt_calculated - DUST10_ZERO) / 5.9; // Carlotta
	dtostrf(dust10_density, 7, 2, str_data);

#ifdef USE_SERIAL_DATA_DEBUG 
	Serial.print(F("AVG PM10 adc raw Signal Value (0-1023): "));
	Serial.println(dust10_adc_value / DUST10_SAMPLE_COUNT);
 
	Serial.print(F("PM10 -- Voltage: "));
	Serial.println(dust10_volt_calculated);
 
	Serial.print(F("PM10 -- Dust Density: "));
	Serial.println(str_data	);		// unit: mg/m3
#endif
}

// Polveri 1um
void read_dust01() {
	mux_set(DUST01_MUX_ADDRESS);
	uint8_t i = 0;
	boolean read_again = true;
	uint16_t dust01_adc_value = 0;
	float dust01_volt_calculated;
	float dust01_density;

	while (read_again) 
	{
		i++;
		// Non inserire righe di codice da quì
		digitalWrite(DUST01_IRLED, LOW);						// power on the LED
		delayMicroseconds(DUST_SAMPLE_TIME);
		dust01_adc_value += analogRead(DUST01_MUX_ADDRESS);			// read the analog value
		delayMicroseconds(DUST_DELTA_TIME);
		digitalWrite(DUST01_IRLED, HIGH);					// turn the LED off
		delayMicroseconds(DUST_SLEEPING_TIME);
		// a quì
		if ((i >= DUST01_SAMPLE_COUNT)) { read_again = false; }
	}
	// 0 - 5V mapped to 0 - 1023 integer values
	// recover voltage
	dust01_volt_calculated = ((float) (dust01_adc_value / (float) i) * 5.0) / 1024.0;
	// linear eqaution taken from http://www.howmuchsnow.com/arduino/airquality/
	// Chris Nafis (c) 2012
	// dust01_density = (0.17 * dust01_volt_calculated) - 0.1;
	dust01_density = (dust01_volt_calculated - DUST01_ZERO) / 5.9; // Carlotta
	dtostrf(dust01_density, 7, 2, str_data);

#ifdef USE_SERIAL_DATA_DEBUG 
	Serial.print(F("AVG PM01 adc raw Signal Value (0-1023): "));
	Serial.println(dust01_adc_value / DUST01_SAMPLE_COUNT);
	Serial.print(F("PM01 -- Voltage: "));
	Serial.println(dust01_volt_calculated);
	Serial.print(F("PM01 -- Dust Density: "));
	Serial.println(str_data);		// unit: mg/m3
#endif
}

// CO2
void read_mq135() {
	mux_set(MQ135_MUX_ADDRESS);
	uint8_t i = 0;
	boolean read_again = true;
	uint16_t mq135_volt_measured = 0;
	float mq135_volt_calculated;
	float mq135_density;

	while (read_again)
	{
		i++;
		mq135_volt_measured += analogRead(MUX_APIN);
		if ((i >= MQ135_SAMPLE_COUNT)) { read_again = false; }
	}
	
	mq135_volt_calculated = ((float)mq135_volt_measured / float(i) * 5.0) / 1024.0;
	mq135_density = mq135_volt_calculated;			// CALCOLARE !!!

#ifdef USE_SERIAL_DATA_DEBUG 
		Serial.print(F("MQ135 raw Signal Value (0-1023): "));
		Serial.println(mq135_volt_measured);

		Serial.print(F(" -- MQ135 Voltage: "));
		Serial.println(mq135_volt_calculated);

		Serial.print(F(" -- MQ135 GAS Density: "));
		Serial.println(mq135_density);		// <-- calcolare la cocentrazione in ppm
#endif

		dtostrf(mq135_density, 7, 2, str_data);
	
}

// SO2
void read_mq136() {
	mux_set(MQ136_MUX_ADDRESS);
	uint8_t i = 0;
	boolean read_again = true;
	uint16_t mq136_volt_measured = 0;
	float mq136_volt_calculated;
	float mq136_density;

	while (read_again)
	{
		i++;
		mq136_volt_measured += analogRead(MUX_APIN);
		if ((i >= MQ136_SAMPLE_COUNT)) { read_again = false; }
	}

	mq136_volt_calculated = ((float)mq136_volt_measured / float(i) * 5.0) / 1024.0;
	mq136_density = mq136_volt_calculated;			// CALCOLARE !!!

#ifdef USE_SERIAL_DATA_DEBUG 
	Serial.print(F("MQ136 raw Signal Value (0-1023): "));
	Serial.println(mq136_volt_measured);

	Serial.print(F(" -- MQ136 Voltage: "));
	Serial.println(mq136_volt_calculated);

	Serial.print(F(" -- MQ136 GAS Density: "));
	Serial.println(mq136_density);		// <-- calcolare la cocentrazione in ppm
#endif

	dtostrf(mq136_density, 7, 2, str_data);

}

// VOC
void read_mq138() {
	mux_set(MQ138_MUX_ADDRESS);
	uint8_t i = 0;
	boolean read_again = true;
	uint16_t mq138_volt_measured = 0;
	float mq138_volt_calculated;
	float mq138_density;

	while (read_again)
	{
		i++;
		mq138_volt_measured += analogRead(MUX_APIN);
		if ((i >= MQ138_SAMPLE_COUNT)) { read_again = false; }
	}

	mq138_volt_calculated = ((float)mq138_volt_measured / float(i) * 5.0) / 1024.0;
	mq138_density = mq138_volt_calculated;			// CALCOLARE !!!

#ifdef USE_SERIAL_DATA_DEBUG 
	Serial.print(F("MQ138 raw Signal Value (0-1023): "));
	Serial.println(mq138_volt_measured);

	Serial.print(F(" -- MQ138 Voltage: "));
	Serial.println(mq138_volt_calculated);

	Serial.print(F(" -- MQ138 GAS Density: "));
	Serial.println(mq138_density);		// <-- calcolare la cocentrazione in ppm
#endif

	dtostrf(mq138_density, 7, 2, str_data);

}

void read_mpx4115a() {
	mux_set(MPX4115A_MUX_ADDRESS);
	uint8_t i = 0;
	boolean read_again = true;
	uint16_t mpx4115a_adc_value = 0;
	float mpx4115a_volt_calculated;
	float mpx4115a_hPa;

	while (read_again) {
		i++;
		mpx4115a_adc_value += analogRead(MUX_APIN);
		if ((i >= MPX4115_SAMPLE_COUNT)) { read_again = false; }
	}
	
	mpx4115a_volt_calculated = ((float)(mpx4115a_adc_value / (float)i) * 5.0) / 1023.0;
	// float pressure=((pressureValue/1024.0)+0.095)/0.000009;

	mpx4115a_hPa = ((float)(mpx4115a_adc_value / i) / 1023.0 + 0.095) / 0.009 * 10.0;

#ifdef USE_SERIAL_DATA_DEBUG 
	Serial.print(F("MPX4115A raw Signal Value (0-1023): "));
	Serial.println(mpx4115a_adc_value);

	Serial.print(F(" -- MPX4115A Voltage: "));
	Serial.println(mpx4115a_volt_calculated);

	Serial.print(F(" -- MPX4115A hPa: "));
	Serial.println(mpx4115a_hPa);
#endif

	dtostrf(mpx4115a_hPa, 7, 2, str_data);

}

// Impostazione indirizzo MUX. 
void mux_set(const byte which)
{
	// select correct MUX channel
	digitalWrite(MUX_BIT0, (which & 1) ? HIGH : LOW);  // low-order bit
	digitalWrite(MUX_BIT1, (which & 2) ? HIGH : LOW);
	digitalWrite(MUX_BIT2, (which & 4) ? HIGH : LOW);  // high-order bit

#ifdef USE_SERIAL_DATA_DEBUG 
	Serial.print(F("MUX Channel: "));
	Serial.println(which);
#endif
}

uint32_t free_ram()
{
	extern int __heap_start, *__brkval;
	int v;
	return (uint32_t)&v - (__brkval == 0 ? (uint32_t)&__heap_start : (uint32_t)__brkval);
};

void internet_setup()
{
#ifdef USE_SERIAL_ETH_DEBUG
	Serial.print(F("MAC: "));
	for (byte i = 0; i < 6; ++i) {
		Serial.print(def_mac[i], HEX);
		if (i < 5)
			Serial.print(F(":"));
	}
	Serial.println();
#endif

	if (ether.begin(sizeof Ethernet::buffer, def_mac, 10) == 0)
	{
#ifdef USE_SERIAL_ETH_DEBUG
		Serial.println(F("Failed to access Ethernet controller"));
		Serial.println(F("Reset..."));
#endif
	sw_reset();
	}

#ifdef USE_SERIAL_IP_DEBUG
	Serial.println(F("Setting up DHCP"));
#endif

	if (!ether.dhcpSetup())
	{
#ifdef USE_SERIAL_IP_DEBUG
		Serial.println(F("DHCP failed"));
		Serial.println(F("Reset..."));
#endif
	sw_reset();
	}

#ifdef USE_SERIAL_IP_DEBUG
	ether.printIp(F("My IP: "), ether.myip);
	ether.printIp(F("Netmask: "), ether.netmask);
	ether.printIp(F("GW IP: "), ether.gwip);
	ether.printIp(F("DNS IP: "), ether.dnsip);
#endif

	while (!ether.isLinkUp())
	{
		ether.packetLoop(ether.packetReceive());
	}


	if (!ether.dnsLookup(web_site))
	{
#ifdef USE_SERIAL_IP_DEBUG
		Serial.println(F("DNS failed"));
		Serial.println(F("Reset..."));
#endif
	sw_reset();
	}
	else
	{
#ifdef USE_SERIAL_IP_DEBUG
	Serial.print(F("Web Site IP: "));
	for (byte i = 0; i < 4; ++i) {
		Serial.print(ether.hisip[i], DEC);
		if (i < 3)
			Serial.print(F("."));
	}
	Serial.println();
	Serial.println(F("Connection OK"));
#endif
	}

	ether.persistTcpConnection(true);
}
