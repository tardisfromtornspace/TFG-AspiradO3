/* Simple HTTP + SSL Server Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <esp_wifi.h>
#include <esp_event.h>
#include <esp_log.h>
#include <esp_system.h>
#include <nvs_flash.h>
#include <sys/param.h>
#include "esp_netif.h"
#include "esp_eth.h"
#include "protocol_examples_common.h"

#include <esp_https_server.h>
#include "esp_tls.h"

// Del LED
#include <stdio.h>
#include "driver/gpio.h"
#include "led_strip.h"
#include "sdkconfig.h"
// Fin de includes del LED

/*Del ADC específicamente*/
#include "driver/adc.h"
#include "esp_adc_cal.h"
// Fin de la seccion ADC

// Del reseteo de fabrica
#include "esp_partition.h"
#include "esp_https_ota.h"
#include "esp_ota_ops.h"
#include "esp_log.h"

// MQTT

#include "lwip/sockets.h"
#include "lwip/dns.h"
#include "lwip/netdb.h"

#include "mqtt_client.h"

#include <stdint.h>
#include <stddef.h>
#include "esp_wifi.h"
#include "esp_system.h"
#include "esp_event.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"

// Aniadir libreria cJSON
#include "cJSON.h"

// Para I2C
#include "driver/i2c.h"

#include "driver/spi_master.h"

// Para GPS
#include "nmea_parser.h"

// Para mensajes genéricos
#include <string.h>
#include <math.h>

// Para Telegram
#include <stdlib.h>
#include <sys/time.h>
#include "lwip/apps/sntp.h"

#include "sh2lib.h"

// Para los sleep
#include <time.h>
#include "soc/soc_caps.h"
#include "esp_sleep.h"
#include "driver/rtc_io.h"
#include "soc/rtc.h"
#include "esp32/ulp.h"
#include "driver/uart.h"

// Para display
#include "ssd1306.h"
#include "font8x8_basic.h"

// Para motores
#include "driver/mcpwm.h"

/* A simple example that demonstrates how to create GET and POST
 * handlers and start an HTTPS server.
 */

static const char *TAG = "ApiradO3";

xSemaphoreHandle Semaphore; // TO-DO SEMAFORO
int enEllo = 0;

/* Motores AspiradO3 */
// WIP Arreglar conflicto con el LED
// Motor Aspirador
#define PIN_ASPIRADOR 21

// Motor interno que redirige hacia adelante o atrás
#define SERVOTIMONINTERNO_MIN_PULSEWIDTH_US (1000) // Minimum pulse width in microsecond
#define SERVOTIMONINTERNO_MAX_PULSEWIDTH_US (2000) // Maximum pulse width in microsecond
#define SERVOTIMONINTERNO_MAX_DEGREE        (45)   // Maximum angle in degree upto which servo can rotate (+-)

// Motor del timón externo que gira de -90 a 90.
#define SERVOTIMONEXTERNO_MIN_PULSEWIDTH_US (600) // Minimum pulse width in microsecond
#define SERVOTIMONEXTERNO_MAX_PULSEWIDTH_US (2600) // Maximum pulse width in microsecond
#define SERVOTIMONEXTERNO_MAX_DEGREE        (90)   // Maximum angle in degree upto which servo can rotate

#define SERVO_PULSE_TIMONINTERNO        (18)   // GPIO connects to the PWM signal line
#define SERVO_PULSE_TIMONEXTERNO        (19)   // GPIO connects to the PWM signal line


/* Sensores ozono */
// WIP
// Uso SPI?
/*
#define PIN_NUM_MISO 25
#define PIN_NUM_MOSI 23
#define PIN_NUM_CLK  19
#define PIN_NUM_CS   22
*/
/* COSAS MATEMÁTICAS Y DE COMPONENTE MISCELÁNEO*/
#define E 2.718281828459
#define RESLDATASHEET 1000000 // La resistencia del datasheet del sensor MQ131 utilizado en el módulo MikroE de detección de ozono
#define RESL 10100 // Va de 100 a 10100 ohmios

#define RESLBABOR 10630 // Lecturas reales del multimetro
#define RESLESTRIBOR 9550
#define RESLTRASFILTRO 9640

#define MQ131_DEFAULT_STABLE_CYCLE 15 //Número de ciclos estables

// TO-DO ajustar al sensor de ozono
#define VOLTREF 4.500 // En mV ya que las medidas de ADC las obtenemos en mV. Esto en teoría es vref pero a lo mejor difiere (p.ej 3.3V o 5V), por eso no he puesto VOLTREF vref, además tantos motores drenan
#define VOLTREFDATASHEET 5.000

#define CTRLZ 26 // Definir Ctrl+Z

/* GPS y GSM */
#define TIME_ZONE (+0)   // Tiempo del meridiano de Greenwich
#define YEAR_BASE (2000) // Date in GPS starts from 2000

#define UART_NUM_2_RXD_DIRECT_GPIO_NUM 16
#define UART_GPIO3_DIRECT_CHANNEL UART_NUM_0

#define NMEA_PARSER_CONFIG_CUSTOM()       \
    {                                      \
        .uart = {                          \
            .uart_port = UART_NUM_1,       \
            .rx_pin = 17,                  \
            .baud_rate = 9600,             \
            .data_bits = UART_DATA_8_BITS, \
            .parity = UART_PARITY_DISABLE, \
            .stop_bits = UART_STOP_BITS_1, \
            .event_queue_size = 16         \
        }                                  \
    }

#define SDSMSPIN "6875" // TO-DO AJUSTA PARA QUE ESTÉ EN SDKCONFIG
#define APN "mms.vodafone.net" // TO-DO AJUSTA PARA QUE ESTÉ EN SDKCONFIG

/*de https://github.com/ciruu1/SBC/blob/master/main/main.c MUCHAS GRACIAS */
#include "minmea.h"
#define UART UART_NUM_2
#define TXD_PIN 17 // Necesario para cuando tengamos el SIM800
#define RXD_PIN 3 // 16
static const int RX_BUF_SIZE = 4096;

/* LED */
#define PIN_SWITCH 2 // 35 // TO-DO TAMPOCO USES EL PIN 2 A MENOS QUE QUIERAS QUE SI EL SWITCH ESTÉ A ON NO SE PUEDA COMUNICAR CON LA FLASH PARA SUBIR PROGRAMAS POR CABLE Si aplicamos lo de la optimización, se puede hacer para despertar al procesador cuando se activa el switch del display (recomendable cambiar el switch a otro pin, sin embargo).
#define BLINK_GPIO CONFIG_BLINK_GPIO // 18 originariamente TO-DO borrar si no es necesario
#define GPIO_WAKEUP_NUM PIN_SWITCH
#define GPIO_WAKEUP_LEVEL 0
// Fin del LED

// I2C nota: hemos elegido el 4 y el 0, puede que de incompatibilidades con otros módulos fuera de ESP32
/*
     GPIO num         RTC GPIO Num
SCL: gpio4  Y gpio2   10 y 12
SDA: gpio0  y MTDO    11 y 13

EN LOS PINES NO PONGAIS DE 6 A 11 QUE ESOS SON DE LA FLASH
*/

#define I2C_MASTER_SCL_IO GPIO_NUM_4  /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO GPIO_NUM_15 /*!< GPIO number used for I2C master data  */
#define I2C_MASTER_NUM I2C_NUM_0      /*!< I2C master i2c port number, the number of i2c peripheral interfaces available will depend on the chip */
#define I2C_MASTER_FREQ_HZ 100000     /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE 0   /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0   /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS 10000

#define CO2_SENSOR_ADDR 0x5A /*!< Slave address of the CO2 sensor is 0x5A, when we add an additional binary 1 behind it transforms into 0xB5 */
#define CO2_REG_ADDR 0x5A    /*!< Register addresses of the CO2 lecture register */

#define O3_SENSOR_ADDR 0x33 /* TO-DO Si el sensor de O3 se usa por ADC esto no se necesita BORRAR EN LA VERSIÓN FINAL*/
#define O3_SENSOR_REGISTER 0x97

#define TEMPHUM_SENSOR_ADDR 0x44 /*Dir sensor tempHum*/
#define COMANDO_TEMPHUM_MSB 0x24
#define COMANDO_TEMPHUM_LSB 0x16 /*Los comandos de este sensor son de 16 bits, indican el MSB el modo y el LSB la frecuencia, así 0x24 0x16 es modo SIngle Shot con baja repetibilidad*/

#define ADC12C_AADR 0x68 /*Direccion del ADC I2C externo, 4 bits de dir (1101) + 3 de canal + bit r/w, 1101 = 0D  
The I 2C address bits (A2, A1, A0 bits) for the MCP3423 and MCP3424 are user configurable and
determined by the logic status of the two external address selection pins on the user’s application board
(Adr0 and Adr1 pins)
Así, si lo pongo Adr0 = Adr1 = LOW => la dirección es 0x68 (que se volvería 0xD0 o 0xD1 al añadir el bit de lectura/escritura)
*/
/*Bit de configuracion es por defecto 1 00 1 00 00 (o sea output no actualizado, canal de selección 1, modo conversión continuo, 12 bits, ganacia x1)*/
#define regADCI2C_paraCanal1 0x90 // 1 00 1 0000
#define regADCI2C_paraCanal2 0xB0 // 1 01 1 0000
#define regADCI2C_paraCanal3 0xD0 // 1 10 1 0000
#define regADCI2C_paraCanal4 0xF0 // 1 11 1 0000

/*Basado en https://www.electronicaembajadores.com/Datos/pdf1/sm/smci/ads1015.pdf */
#define ADCI2CADAFRUIT_AADR 0x48 /* La dirección puede ser 1001000 si a GND y 1001001 si conectado pin A0 a VCC */
#define ADCI2CADAFRUIT_CONFIGADDR 0x01 /* La dir del reg de configuracion */
#define ADCI2CADAFRUIT_CONFIGMSB 0x04 /* 0b00000100 MSB of the Config register */
#define ADCI2CADAFRUIT_CONFIGLSB 0x83 /* 0b10000011 LSB of the Config register */
#define ADCI2CADAFRUIT_CONVADDR 0X00 /* 0b00000000 registro de la conversión */
#define ADCI2CADAFRUIT_CONFIGREGMUXA0 0x42 /* Las 4 posibles salidas del ADC */
#define ADCI2CADAFRUIT_CONFIGREGMUXA1 0x52
#define ADCI2CADAFRUIT_CONFIGREGMUXA2 0x62
#define ADCI2CADAFRUIT_CONFIGREGMUXA3 0x72
/* primero hay un 0 de estado operativo, 100 es analog 0 y AInn = GND
101 es AN1
110 es AN2
111 es AN3
Luego 001 para +-4.096 V, 0 para modo continuo, 100 para 1600 SPS, luego 0 0 0 11 para otras opciones por defecto
Por lo tanto tenemos
01000010 10000011
01010010 10000011
01100010 10000011
01110010 10000011
*/

#define LUZ_SENSOR_ADDR 0x10 /*Dir sensor luminosidad*/
#define LUZ_REG_ADDR 0x04    /*Dir registro de luminosidad es 0x04*/

#define MPU9250_PWR_MGMT_1_REG_ADDR 0x6B /*!< Register addresses of the power managment register */
#define MPU9250_RESET_BIT 7

#define READ_BIT I2C_MASTER_READ /*Bits de lectura y escritura*/
#define WRITE_BIT I2C_MASTER_WRITE
#define ACK_CHECK_EN 0x1              /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS 0x0             /*!< I2C master will not check ack from slave */
#define ACK_VAL I2C_MASTER_ACK        /*!< I2C ack value */
#define NACK_VAL I2C_MASTER_LAST_NACK // I2C_MASTER_NACK    /*!< I2C nack value */

// ADC Channels
#if CONFIG_IDF_TARGET_ESP32
#define ADC4_EXAMPLE_CHAN0 ADC1_CHANNEL_5
static const char *TAG_CH[1][10] = {{"ADC2_CH5"}};
#else
#define ADC4_EXAMPLE_CHAN0 ADC1_CHANNEL_5
static const char *TAG_CH[1][10] = {{"ADC2_CH5"}};
#endif

#define PIN_ANALOG4 33

// ADC Attenuation
#define ADC_EXAMPLE_ATTEN ADC_ATTEN_DB_11

// ADC Calibration
#if CONFIG_IDF_TARGET_ESP32
#define ADC_EXAMPLE_CALI_SCHEME ESP_ADC_CAL_VAL_EFUSE_VREF
#elif CONFIG_IDF_TARGET_ESP32S2
#define ADC_EXAMPLE_CALI_SCHEME ESP_ADC_CAL_VAL_EFUSE_TP
#elif CONFIG_IDF_TARGET_ESP32C3
#define ADC_EXAMPLE_CALI_SCHEME ESP_ADC_CAL_VAL_EFUSE_TP
#elif CONFIG_IDF_TARGET_ESP32S3
#define ADC_EXAMPLE_CALI_SCHEME ESP_ADC_CAL_VAL_EFUSE_TP_FIT
#endif
// Telegram

extern const uint8_t server_rootTelegram_cert_pem_start[] asm("_binary_http2_telegram_root_cert_pem_start");
extern const uint8_t server_rootTelegram_cert_pem_end[] asm("_binary_http2_telegram_root_cert_pem_end");

/* The HTTP/2 server to connect to */
#define HTTP2_SERVER_URI "https://api.telegram.org"
/* A GET request that keeps streaming current time every second */
#define TELEGRAMTOKEN "CAMBIALO POR EL TUYO" // TO-DO NO LO SUBAS CON ESTO A LA ENTREGA!!!!
#define CHATTOKEN "-891728903"               //"CAMBIA POR EL TUYO" // TO-DO NO LO SUBAS CON ESTO A LA ENTREGA!!!!
#define UNIVERSITY "SBC22_M01"               //"UPM"
#define TOKENMQTT "YSRNEFDXnyIGhX9OaylG"
#define MQTTURI "mqtt://demo.thingsboard.io"
int ini_OFFSET = 0;
int last_msg_received = 559291163;
int extraOffset = 0;
int conexionEnProceso = 1;
int conproc = -1;

// DESCOMENTA static const char GETBOT[] = "/bot" TELEGRAMTOKEN "/";
// static const char GETUPDATES[] = "/bot" TELEGRAMTOKEN "/getUpdates?limit=5";
static const char POSTUPDATES[] = "/bot" TELEGRAMTOKEN "/sendMessage?chat_id=" CHATTOKEN;

// Sleeps
static RTC_DATA_ATTR struct timeval sleep_enter_time; // global
int contadorAdormir = 0;
int sleepEnabled = 0; // Hemos dejado el sleep inhabilitado porque al final no es esencial
int http2Caido = 0;

// LEDes, motores y algunos datos

static uint8_t s_led_state = 0; // Estado del led

static uint8_t s_switch_state = 0; // Estado del switch TO-DO borrar en final

static uint8_t s_aspirador_state = 0; // Estado del aspirador 0 - apagado, 1 - encendido
int estadoTimonInterno = 45; // Estado del timon interno, -45 posición levantada, 0 inicial o desconocido, 45 posición bajada
int estadoTimonExterno = 0; // Estado del timon externo, [-90 0) babor, 0 inicial o centrado, (0 90] estribor

int s_reset_state = 0; // tiempo hasta resetear. 0 es que no se resetea

int voltajeSolar = 0; // Voltaje generado por el panel solar del globo

int ozonoBabor = 0; // Ozono detectado por el sensor de babor
int ozonoEstribor = 0; // Ozono detectado por el sensor de estribor
int ozonoTrasFiltro = 0; // Ozono detectado tras el filtro activo de carbono

int temperaturaAtmos = 0; // Temperatura atmosferica en grados centigrados.
int humedadAtmos = 0; // Humedad relativa en %. Va de 0 a 100

/* Info GPS TO-DO ajustar*/
int gpsdateyear = 2000;
int gpsdatemonth = 1;
int gpsdateday = 1;
int gpstimhour = 1;
int gpstimminute = 0;
int gpstimsecond = 0;
double gpslatitude = 0.0;
double gpslongitude = 0.0;
double gpsaltitude = 0.0;
double gpsspeed = 0.0;
double gpscourse = 0.0;

double gpslatitudeAnt = 0.0;
double gpslongitudeAnt = 0.0;
double gpsaltitudeAnt = 0.0;
double gpsspeedAnt = 0.0;
double gpscourseAnt = 0.0;

// ADC

static int adc_raw[1][10];

static esp_adc_cal_characteristics_t adc1_chars;
static esp_adc_cal_characteristics_t adc2_chars;

// I2C

static i2c_port_t i2c_master_port = I2C_MASTER_NUM;

uint8_t data[9] = {0, 0, 0, 0, 0, 0, 0, 0, 0}; // Para sensor I2C - tomamos los 8 bytes pero solo necesitamos los 2 primeros

// LEDes

#ifdef CONFIG_BLINK_LED_RMT
static led_strip_t *pStrip_a;

static void blink_led(void)
{
    /* If the addressable LED is enabled */
    if (s_led_state)
    {
        /* Set the LED pixel using RGB from 0 (0%) to 255 (100%) for each color */
        pStrip_a->set_pixel(pStrip_a, 0, 16, 16, 16);
        /* Refresh the strip to send data */
        pStrip_a->refresh(pStrip_a, 100);
    }
    else
    {
        /* Set all LED off to clear all pixels */
        pStrip_a->clear(pStrip_a, 50);
    }
}



static void configure_led(void)
{
    ESP_LOGI(TAG, "Example configured to blink addressable LED!");
    /* LED strip initialization with the GPIO and pixels number*/
    pStrip_a = led_strip_init(CONFIG_BLINK_LED_RMT_CHANNEL, BLINK_GPIO, 1);
    /* Set all LED off to clear all pixels */
    pStrip_a->clear(pStrip_a, 50);
}

#elif CONFIG_BLINK_LED_GPIO

static void blink_led(void)
{
    /* Set the GPIO level according to the state (LOW or HIGH)*/
    gpio_set_level(BLINK_GPIO, s_led_state);
}

static void configure_led(void)
{
    ESP_LOGI(TAG, "Example configured to blink GPIO LED!");
    gpio_reset_pin(BLINK_GPIO);
    /* Set the GPIO as a push/pull output */
    gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);
}

#endif

/*Función GPS, USAMOS GPIO 5 por defecto */
/**
 * @brief GPS Event Handler
 *
 * @param event_handler_arg handler specific arguments
 * @param event_base event base, here is fixed to ESP_NMEA_EVENT
 * @param event_id event id
 * @param event_data event specific arguments
 */
static void gps_event_handler(void *event_handler_arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    gps_t *gps = NULL;
    ESP_LOGI(TAG, "GPS EVENT HANDLER SE ACTIVO");
    switch (event_id) {
    case GPS_UPDATE:
        gps = (gps_t *)event_data;
        /* print information parsed from GPS statements */
        ESP_LOGI(TAG, "%d/%d/%d %d:%d:%d => \r\n"
                 "\t\t\t\t\t\tlatitude   = %.05f°N\r\n"
                 "\t\t\t\t\t\tlongitude = %.05f°E\r\n"
                 "\t\t\t\t\t\taltitude   = %.02fm\r\n"
                 "\t\t\t\t\t\tspeed      = %fm/s",
                 gps->date.year + YEAR_BASE, gps->date.month, gps->date.day,
                 gps->tim.hour + TIME_ZONE, gps->tim.minute, gps->tim.second,
                 gps->latitude, gps->longitude, gps->altitude, gps->speed);
        gpsdateyear = gps->date.year + YEAR_BASE;
        gpsdatemonth = gps->date.month;
        gpsdateday = gps->date.day;
        gpstimhour = gps->tim.hour + TIME_ZONE;
        gpstimminute = gps->tim.minute;
        gpstimsecond = gps->tim.second;
        /*Cargamos la info anterior, por comparar*/
        gpslatitudeAnt = gpslatitude;
        gpslongitudeAnt = gpslongitude;
        gpsaltitudeAnt = gpsaltitude;
        gpsspeedAnt = gpsspeed;
        
        gpslatitude = gps->latitude;
        gpslongitude = gps->longitude;
        gpsaltitude = gps->altitude;
        gpsspeed = gps->speed;
        break;
    case GPS_UNKNOWN:
        /* print unknown statements */
        ESP_LOGW(TAG, "UART Unknown statement:%s", (char *)event_data);
        break;
    default:
        break;
    }
}

/*de https://github.com/ciruu1/SBC/blob/master/main/main.c MUCHAS GRACIAS */
void init_uart(void)
{
    const uart_config_t uart_config = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };

    // We won't use a buffer for sending data.
    uart_driver_install(UART, RX_BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_param_config(UART, &uart_config);
    uart_set_pin(UART, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}

double convert_num_fixed(double num) {
    int grados = (int) num / 100;
    double minutos = num - (grados * 100);
    num = grados + (minutos/60);
    return num;
}

static void parse(char * line) {
    
    switch(minmea_sentence_id(line, false)) {

        case MINMEA_SENTENCE_GGA: {
            //ESP_LOGI(tag, "GGA");
            struct minmea_sentence_gga frame_gga;
            if (minmea_parse_gga(&frame_gga, line)) {
                /*
                ESP_LOGI(tag, "$xxGGA: Latitude:Longitude %f:%f Time: %d:%d:%d\n",
                         minmea_tofloat(&frame_gga.latitude),
                         minmea_tofloat(&frame_gga.longitude),
                         frame_gga.time.hours,
                         frame_gga.time.minutes,
                         frame_gga.time.seconds);
                */
              
                double lat = convert_num_fixed(((double) minmea_tofloat(&frame_gga.latitude)));
                double lon = convert_num_fixed(((double) minmea_tofloat(&frame_gga.longitude)));
                if (isnan(lat))
                    lat = 0;
                if (isnan(lon))
                    lon = 0;
                ESP_LOGI(TAG, "LAT, LON: %f, %f", lat, lon);

                gpslatitudeAnt = gpslatitude;
                gpslongitudeAnt = gpslongitude;

                gpslatitude = lat;
                gpslongitude = lon;

            }
            else {
                ESP_LOGI(TAG, "$xxGGA sentence is not parsed\n");
            }
        } break;

        case MINMEA_SENTENCE_RMC: {
            struct minmea_sentence_rmc frame;
            if (minmea_parse_rmc(&frame, line)) {
                /* COMENTADO PARA VER LO DE SENSORES CON MAYOR CLARIDAD
                printf("$RMC: raw coordinates and speed: (%d/%d,%d/%d) %d/%d\n",
                        frame.latitude.value, frame.latitude.scale,
                        frame.longitude.value, frame.longitude.scale,
                        frame.speed.value, frame.speed.scale);
                printf("$RMC fixed-point coordinates and speed scaled to three decimal places: (%d,%d) %d\n",
                        minmea_rescale(&frame.latitude, 1000),
                        minmea_rescale(&frame.longitude, 1000),
                        minmea_rescale(&frame.speed, 1000));
                printf("$RMC floating point degree coordinates and speed: (%f,%f) %f\n",
                        minmea_tocoord(&frame.latitude),
                        minmea_tocoord(&frame.longitude),
                        minmea_tofloat(&frame.speed));
                */
                gpscourseAnt = gpscourse;
                gpscourse = minmea_tofloat(&frame.course);
                gpsspeedAnt = gpsspeed;
                gpsspeed = minmea_tofloat(&frame.speed);
            }
        } break;
        /* Usado solo para saber si detecto satélites */
        case MINMEA_SENTENCE_GSV: {
            struct minmea_sentence_gsv frame;
            if (minmea_parse_gsv(&frame, line)) {
                //printf("$GSV: message %d of %d\n", frame.msg_nr, frame.total_msgs);
                printf("$GSV: satellites in view: %d\n", frame.total_sats);
                //for (int i = 0; i < frame.total_sats; i++)
                //    printf("$GSV: sat nr %d, elevation: %d, azimuth: %d, snr: %d dbm\n",
                //        frame.sats[i].nr,
                //        frame.sats[i].elevation,
                //        frame.sats[i].azimuth,
                //        frame.sats[i].snr);
            }
        } break;

        default:
            //ESP_LOGI(tag, "DEFAULT");
            //ESP_LOGD(tag, "Sentence - other");
            break;
    }
}


static void parse_line(char *line)
{
    char *p;
    int i;
    for (i = 1, p = strtok(line,"\n"); p != NULL; p = strtok(NULL,"\n"), i++) {
        //printf("Output%u=%s;\n", i, p);
        //ESP_LOGI(TAG, "%u>>>>>>>>>>>>>>>> %s",i , p);
        parse(p);
    }
    //ESP_LOGI(TAG, "--------------------------------------------------------------");
}

static void rx_task(void *arg)
{
    static const char *RX_TASK_TAG = "RX_TASK";
    esp_log_level_set(RX_TASK_TAG, ESP_LOG_INFO);

    esp_gps_t *esp_gps = calloc(1, sizeof(esp_gps_t));
    esp_gps->buffer = calloc(1, RX_BUF_SIZE);
#if CONFIG_NMEA_STATEMENT_GSA
    esp_gps->all_statements |= (1 << STATEMENT_GSA);
#endif
#if CONFIG_NMEA_STATEMENT_GSV
    esp_gps->all_statements |= (1 << STATEMENT_GSV);
#endif
#if CONFIG_NMEA_STATEMENT_GGA
    esp_gps->all_statements |= (1 << STATEMENT_GGA);
#endif
#if CONFIG_NMEA_STATEMENT_RMC
    esp_gps->all_statements |= (1 << STATEMENT_RMC);
#endif
#if CONFIG_NMEA_STATEMENT_GLL
    esp_gps->all_statements |= (1 << STATEMENT_GLL);
#endif
#if CONFIG_NMEA_STATEMENT_VTG
    esp_gps->all_statements |= (1 << STATEMENT_VTG);
#endif

    ESP_LOGI(TAG, "Inicio rx del esp_gps");

    while (1) {
        int length = 0;
        ESP_ERROR_CHECK(uart_get_buffered_data_len(UART, (size_t*)&length));
        length = uart_read_bytes(UART, esp_gps->buffer, RX_BUF_SIZE, 100 / portTICK_PERIOD_MS);

        ESP_LOGI(RX_TASK_TAG, "Read %d bytes: '%s'", length, esp_gps->buffer); // TO-DO comentar luego

        /* make sure the line is a standard string TO-DO*/
        esp_gps->buffer[length] = '\0';
        /* Send new line to handle */
//        ESP_LOGI(RX_TASK_TAG, "Inicio decoding:");
        //if (gps_decode(esp_gps, length + 1) != ESP_OK) {
        //    ESP_LOGW(RX_TASK_TAG, "GPS decode line failed");
        //}

        parse_line((char *)esp_gps->buffer);
        //free(data);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}
/*Inicio GSM*/
int sendData(const char* logName, const char* data)
{
    const int len = strlen(data);
    const int txBytes = uart_write_bytes(UART, data, len);
    ESP_LOGI(logName, "Wrote %d bytes: %s", txBytes, data);
    return txBytes;
}

void send_SMS(const char* logName, const char* telefono, const char* data){
    sendData(logName, "AT+CMGF=1\r\n"); // Configuring TEXT mode
    vTaskDelay(10000 / portTICK_PERIOD_MS);
    sendData(logName, "AT+CMGS=\"+034634723664\"\r\n"); // change ZZ with country code and xxxxxxxxxxx with phone number to sms TO-DO usa tu teléfono
    vTaskDelay(10000 / portTICK_PERIOD_MS);
    sendData(logName, data);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    sendData(logName, data);
    // Enviar Ctrl+Z en una nueva línea vacía
    const char *mess = "\r\n\032";
    ESP_LOGI(logName, "Escribiendo el ctrl+z");
    sendData(logName, mess);
    vTaskDelay(10000 / portTICK_PERIOD_MS);

}

static void tx_task(void *arg)
{
    static const char *TX_TASK_TAG = "TX_TASK_GSM";
    esp_log_level_set(TX_TASK_TAG, ESP_LOG_INFO);

    /* La tarjeta a lo mejor requiere de un PIN */
    char mensajito[494];
    char mensajito2[494];
    char mensajito3[494];
    char mensajito4[494];
    char mensajito5[494];

    char finDeMensaje[] = "";
    const char *mess;
    const char *mess2;
    const char *mess3;
    const char *mess4;
    const char *mess5;

    vTaskDelay(4000 / portTICK_PERIOD_MS);

    /* Fase de comprobación y registro en la SIM */
    //sprintf(mensajito, "AT%s", "\r\n");
    //mess = strcat(mensajito, finDeMensaje);
    sendData(TX_TASK_TAG, "AT\r\n");
    vTaskDelay(4000 / portTICK_PERIOD_MS);

    sprintf(mensajito3, "AT+CPIN=%s\r\n", SDSMSPIN);
    mess3 = strcat(mensajito3, finDeMensaje);
    sendData(TX_TASK_TAG, mess3);
    vTaskDelay(4000 / portTICK_PERIOD_MS);

    //sprintf(mensajito2, "AT+CSQ%s", "\r\n");
    //mess2 = strcat(mensajito2, finDeMensaje);
    sendData(TX_TASK_TAG, "AT+CSQ\r\n");
    vTaskDelay(5000 / portTICK_PERIOD_MS);

    sendData(TX_TASK_TAG, "AT+CPIN?\r\n");
    vTaskDelay(5000 / portTICK_PERIOD_MS);

    sendData(TX_TASK_TAG, "AT+CCID\r\n");
    vTaskDelay(5000 / portTICK_PERIOD_MS);

    sendData(TX_TASK_TAG, "AT+CGREG=1\r\n");
    vTaskDelay(10000 / portTICK_PERIOD_MS);

    sendData(TX_TASK_TAG, "AT+CREG=1\r\n");
    vTaskDelay(10000 / portTICK_PERIOD_MS);

    sendData(TX_TASK_TAG, "AT+CREG?\r\n");
    vTaskDelay(10000 / portTICK_PERIOD_MS);

    sendData(TX_TASK_TAG, "AT+CGREG?\r\n");
    vTaskDelay(10000 / portTICK_PERIOD_MS);

    sendData(TX_TASK_TAG, "AT+COPS?\r\n");
    vTaskDelay(10000 / portTICK_PERIOD_MS);

    while(1){
        
        sendData(TX_TASK_TAG, "AT+CGATT?\r\n");
        vTaskDelay(500 / portTICK_PERIOD_MS);

        sendData(TX_TASK_TAG, "AT+CCID\r\n");
        vTaskDelay(500 / portTICK_PERIOD_MS);

        sendData(TX_TASK_TAG, "AT+CPIN?\r\n");
        vTaskDelay(500 / portTICK_PERIOD_MS);

        sendData(TX_TASK_TAG, "AT+COPS?\r\n");
        vTaskDelay(1000 / portTICK_PERIOD_MS);

        ESP_LOGI(TAG, "ESPERO A LLAMADA DEL SEMAFORO");
        xSemaphoreTake(Semaphore, portMAX_DELAY); // TO-DO Semaforo

        /* Etapa de conexión IP */
        sendData(TX_TASK_TAG, "AT+CIPSHUT\r\n");
        vTaskDelay(2000 / portTICK_PERIOD_MS);

        sendData(TX_TASK_TAG, "AT+CIPSTATUS\r\n"); // Estado de la conexión TO-DO coméntalos y pruébalo
        vTaskDelay(500 / portTICK_PERIOD_MS);

        sendData(TX_TASK_TAG, "AT+CIPMUX=0\r\n");
        vTaskDelay(1000 / portTICK_PERIOD_MS);


        sprintf(mensajito, "AT+CSTT=\"%s\"\r\n", APN);
        mess = strcat(mensajito, finDeMensaje);
        sendData(TX_TASK_TAG, mess); // TO-DO comprobar
        //sendData(TX_TASK_TAG, "AT+CSTT=\"mms.vodafone.net\"\r\n"); // PUEDE QUE REQUIERA DE USR Y PASS La de llamaya es \"mms.orange.es\", la de lowi es \"mms.vodafone.net\"
        vTaskDelay(5000 / portTICK_PERIOD_MS);
        sendData(TX_TASK_TAG, "AT+CIICR\r\n"); // Conexion inalámbrica
        vTaskDelay(5000 / portTICK_PERIOD_MS);
        sendData(TX_TASK_TAG, "AT+CIFSR\r\n"); // IP local
        vTaskDelay(5000 / portTICK_PERIOD_MS);

        sendData(TX_TASK_TAG, "AT+CIPSTATUS\r\n"); // Estado de la conexión TO-DO coméntalos y pruébalo
        vTaskDelay(500 / portTICK_PERIOD_MS);

        /* Etapa conexión TCP/UDP y cliente-servidor */
        sendData(TX_TASK_TAG, "AT+SAPBR=3,1,\"Contype\",\"GPRS\"\r\n");
        vTaskDelay(5000 / portTICK_PERIOD_MS);
        sendData(TX_TASK_TAG, "AT+SAPBR=3,1,\"APN\",\"mms.vodafone.net\"\r\n"); // Estado de la conexión
        vTaskDelay(5000 / portTICK_PERIOD_MS);
        sendData(TX_TASK_TAG, "AT+SAPBR=1,1\r\n");
        vTaskDelay(15000 / portTICK_PERIOD_MS);
        sendData(TX_TASK_TAG, "AT+SAPBR=2,1\r\n");
        vTaskDelay(15000 / portTICK_PERIOD_MS);

        /* Permite comunicación GPS de latitud, longitud y fecha-tiempo
        sendData(TX_TASK_TAG, "AT+CIPGSMLOC=1,1\r\n");
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        */

        sendData(TX_TASK_TAG, "AT+CIPSPRT=0\r\n");
        vTaskDelay(1000 / portTICK_PERIOD_MS);

        /* Petición HTTP */
        sendData(TX_TASK_TAG, "AT+HTTPINIT\r\n");
        vTaskDelay(1000 / portTICK_PERIOD_MS);

        sendData(TX_TASK_TAG, "AT+HTTPPARA=\"CID\",1\r\n");
        vTaskDelay(1000 / portTICK_PERIOD_MS);     

        sprintf(mensajito5, "AT+HTTPPARA=\"URL\",\"http://demo.thingsboard.io/api/v1/%s/telemetry\"\r\n", TOKENMQTT);
        mess5 = strcat(mensajito5, finDeMensaje);
        sendData(TX_TASK_TAG, mess5);
        vTaskDelay(1000 / portTICK_PERIOD_MS);

        sendData(TX_TASK_TAG, "AT+HTTPPARA=\"CONTENT\",\"application/json\"\r\n"); // CONTENT indica el Content-Type.
        vTaskDelay(5000 / portTICK_PERIOD_MS);

        /* Contenido json*/
        cJSON *root = cJSON_CreateObject();

        cJSON_AddNumberToObject(root, "ozonoBabor", ozonoBabor);                       // En p.p.m.
        cJSON_AddNumberToObject(root, "ozonoEstribor", ozonoEstribor);
        cJSON_AddNumberToObject(root, "ozonoTrasFiltro", ozonoTrasFiltro);
        cJSON_AddNumberToObject(root, "posicionDelTimonExterno", estadoTimonExterno);  // Con los valores asignados
        cJSON_AddNumberToObject(root, "posicionDelTimonInterno", estadoTimonInterno);
        cJSON_AddNumberToObject(root, "tempAtmos", temperaturaAtmos);                  // En grados Celsius
        cJSON_AddNumberToObject(root, "humedadRel", humedadAtmos);                     // En %
        cJSON_AddNumberToObject(root, "energiaSolar", voltajeSolar);                   // En voltios
        cJSON_AddNumberToObject(root, "latitudGNSS", gpslatitude);                     // Grados
        cJSON_AddNumberToObject(root, "longitudGNSS", gpslongitude);
        cJSON_AddNumberToObject(root, "altitudGNSS", gpsaltitude);
        cJSON_AddNumberToObject(root, "velocidadGNSS", gpsspeed);                      // Nudos
        cJSON_AddNumberToObject(root, "orientacionGNSS", gpscourse);                   // Grados

        const char *post_data = cJSON_PrintUnformatted(root);
        int tamanio = strlen(post_data);

        // Enviar los datos
        sprintf(mensajito4, "AT+HTTPDATA=%d,10000\r\n", tamanio);
        mess4 = strcat(mensajito4, finDeMensaje);
        sendData(TX_TASK_TAG, mess4);
        vTaskDelay(5000 / portTICK_PERIOD_MS);

        sendData(TX_TASK_TAG, post_data);
        vTaskDelay(11000 / portTICK_PERIOD_MS);

        sendData(TX_TASK_TAG, "AT+HTTPACTION=1\r\n"); // La acción HTTP es 0 = GET, 1=POST, 2=HEAD
        vTaskDelay(5000 / portTICK_PERIOD_MS);
        
        cJSON_Delete(root);

        sendData(TX_TASK_TAG, "AT+HTTPREAD\r\n"); // Leer los datos tras ejecutar
        vTaskDelay(5000 / portTICK_PERIOD_MS);

        sendData(TX_TASK_TAG, "AT+HTTPTERM\r\n"); // Terminar servicio HTTP
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        
        //send_SMS(TX_TASK_TAG, "+034634723664", "Hola Mundo\r\n");
        //vTaskDelay(10000 / portTICK_PERIOD_MS);

        enEllo = 0;
        
    }
}


/*MOTORES*/

static void blink_motorAspirador(void)
{
    /* Set the GPIO level according to the state (LOW or HIGH)*/
    gpio_set_level(PIN_ASPIRADOR, s_aspirador_state);
}

static inline uint32_t convert_servo_90_angle_to_duty_us(int angle)
{
    return (angle + SERVOTIMONINTERNO_MAX_DEGREE) * (SERVOTIMONINTERNO_MAX_PULSEWIDTH_US - SERVOTIMONINTERNO_MIN_PULSEWIDTH_US) / (2 * SERVOTIMONINTERNO_MAX_DEGREE) + SERVOTIMONINTERNO_MIN_PULSEWIDTH_US;
}

static inline uint32_t convert_servo_180_angle_to_duty_us(int angle)
{
    return (angle + SERVOTIMONEXTERNO_MAX_DEGREE) * (SERVOTIMONEXTERNO_MAX_PULSEWIDTH_US - SERVOTIMONEXTERNO_MIN_PULSEWIDTH_US) / (2 * SERVOTIMONEXTERNO_MAX_DEGREE) + SERVOTIMONEXTERNO_MIN_PULSEWIDTH_US;
}

// Motores de corriente continua, para el aspirador y posiblemente uno de los timones internos, AspiradO3
static void configure_motor_continuo(gpio_num_t pin)
{
    ESP_LOGI(TAG, "Configurando un pin para un motor");
    gpio_reset_pin(pin);
    /* Set the pin as a push/pull output */
    gpio_set_direction(pin, GPIO_MODE_OUTPUT);
}

// Switch, se usará para encender o apagar el LCD
static void configure_switch(void)
{
    ESP_LOGI(TAG, "Example configured to switch GPIO!");
    gpio_reset_pin(PIN_SWITCH);
    /* Set the GPIO 34 as a push/pull output */
    gpio_set_direction(PIN_SWITCH, GPIO_MODE_INPUT);
}

/**
 * @brief i2c master initialization
 */
static esp_err_t i2c_master_init(SSD1306_t *dev, int16_t sda, int16_t scl, int16_t reset)
{

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_DISABLE, // NO ENABLE porque las resistencias son externas
        .scl_pullup_en = GPIO_PULLUP_DISABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
        .clk_flags = 0,
    };

    ESP_ERROR_CHECK(i2c_param_config(i2c_master_port, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(i2c_master_port, I2C_MODE_MASTER, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, ESP_INTR_FLAG_LEVEL1)); // conf.mode

    if (reset >= 0)
    {
        // gpio_pad_select_gpio(reset);
        gpio_reset_pin(reset);
        gpio_set_direction(reset, GPIO_MODE_OUTPUT);
        gpio_set_level(reset, 0);
        vTaskDelay(50 / portTICK_PERIOD_MS);
        gpio_set_level(reset, 1);
    }
    dev->_address = I2CAddress;
    dev->_flip = false;
    return ESP_OK;
}

/*
 * ajustarValoresOzono
 * @brief Esta función toma la lectura de ozono y la corrige utilizando la fórmula del divisor de tensión
 * 1º Obtiene la resistencia dada por el valor de lectura obtenido, separado en getResistencia() por comodidad y claridad
 * 2º Tomando humedad y temperatura, ajusta la resistencia
 * 3º Recalcula de nuevo empleando el divisor de tensión
 * 4º Ajusta valores a la curva del datasheet de voltajes
 * 
 * returns int
 * 
*/
double getResistencia(int lecturaInicial, double resistenciaL){
    double resistencia = resistenciaL * ( VOLTREF / ((((double) lecturaInicial) / 1024.0)) - 1);
    ESP_LOGI(TAG, "Para VOLTREF / (lectInicial / 1024) me sale %lf", VOLTREF / ((((double) lecturaInicial) / 1024.0)));
    ESP_LOGI(TAG, "La resistencia me sale %lf", resistencia);
    return resistencia;
}

static int ajustarValoresOzono(int lecturaInicial, int humedad, int temperatura, double ajusteUnicoSensor, double resistenciaL, double valueR0){
    double resistencia = getResistencia(lecturaInicial, resistenciaL);

    double resistenciaAjustada = resistencia / (1 - 0.013 * (temperatura - 20) -  (humedad - 55) / 30 * (0.175 + 0.2/60 * (20 - temperatura)));
    ESP_LOGI(TAG, "La resistencia ajustada me sale %lf", resistenciaAjustada);

    // Método 1, cálculo del valor de referencia en un circuito como el indicado en el datasheet
    double correccionLectura = RESLDATASHEET / (RESLDATASHEET + RESLDATASHEET/resistenciaL * resistenciaAjustada) * VOLTREFDATASHEET; // RESLDATASHEET/resistenciaL es un valor corrector, hay 99 resistencias RESL en RESLDATASHEET.
    ESP_LOGI(TAG, "La corrección de lectura me sale %lf", (correccionLectura/1000 + ajusteUnicoSensor));
    //return (int) fmax( 0.0, fmin(1000.0, 100 * (pow(E, -(correccionLectura/1000 +ajusteUnicoSensor) + 4) -1))) ;

    // Método 2, comparación del Ractual/R0 para determinar concentraciones y luego seguimos una aproximación del ratio según datasheet
    double ratio = resistenciaAjustada / valueR0;
    ESP_LOGI(TAG, "Ratio = %lf", ratio);
    // Use this if you are monitoring low concentration of O3 (air quality project)
    return (int) fmax( 0.0, fmin(1000.0, 9.4783 * pow(ratio, 2.3348)));
    // Para altas concentraciones
    //return (int) fmax( 0.0, fmin(1000.0, 10.66435681 * pow(ratio, 2.25889394) - 10.66435681));
}

void miESPes(esp_err_t retA){
    if (retA == ESP_OK)
    {
        ESP_LOGI(TAG, "Logre llamar al modulo ADCI2C Adafruit y darle un comando");
    }
    else if (retA == ESP_ERR_TIMEOUT)
    {
        ESP_LOGW(TAG, "Bus is busy");
    }
    else if (retA == ESP_ERR_INVALID_ARG)
    {
        ESP_LOGW(TAG, "Parameter error");
    }
    else if (retA == ESP_ERR_INVALID_STATE)
    {
        ESP_LOGW(TAG, "I2C driver not installed on not in master mode");
    }
    else if (retA == ESP_FAIL)
    {
        ESP_LOGW(TAG, "Command error, slave hasn't ACK the transfer");
    }
    else
    {
        ESP_LOGW(TAG, "Read failed");
    }
//    ESP_LOGI(TAG, "My ESP-CODE is %d", retA);
}

/* ADC de adafruit*/
int ADCADAFRUIT12C_register_read(uint8_t slave_addr, uint8_t reg_to_addr, uint8_t input_addr, uint8_t config_LSB, uint8_t reg_addr, size_t len)
{

    uint8_t dato[3] = {0, 0, 0}; // En modo de 12 bits nos devuelve 3 bytes, 2 de lectura y uno de configuración

    // Primero configuramos el ADC para que nos lea la entrada que queremos
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    // Para ello primero buscamos la dirección y luego le doy el registro al que escribir
    ESP_ERROR_CHECK(i2c_master_start(cmd));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, (slave_addr << 1) | WRITE_BIT, ACK_VAL));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, reg_to_addr, ACK_CHECK_EN));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, input_addr, ACK_CHECK_EN));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, config_LSB, ACK_CHECK_EN));
    ESP_ERROR_CHECK(i2c_master_stop(cmd));

    esp_err_t retA = i2c_master_cmd_begin(i2c_master_port, cmd, pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));
    vTaskDelay(pdMS_TO_TICKS(50)); // Needs 1 ms to prepare
    miESPes(retA);
    i2c_cmd_link_delete(cmd);

    vTaskDelay(pdMS_TO_TICKS(50)); // Giving extra time so ADC works

    i2c_cmd_handle_t cmd2 = i2c_cmd_link_create();
    // Ahora le pedimos leer de reg_addr
    ESP_ERROR_CHECK(i2c_master_start(cmd2));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd2, (slave_addr << 1) | WRITE_BIT, ACK_VAL));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd2, reg_addr, ACK_CHECK_EN));
    ESP_ERROR_CHECK(i2c_master_stop(cmd2));

    esp_err_t retB = i2c_master_cmd_begin(i2c_master_port, cmd2, pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));
    vTaskDelay(pdMS_TO_TICKS(50)); // Needs 1 ms to prepare
    miESPes(retB);
    i2c_cmd_link_delete(cmd2);

    // Y nos devuelve el resultado
    i2c_cmd_handle_t cmd3 = i2c_cmd_link_create();
    ESP_ERROR_CHECK(i2c_master_start(cmd3));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd3, (slave_addr << 1) | READ_BIT, ACK_VAL));
    int i = 0;
    for (i = 0; i < len - 1; i++)
    {
        ESP_ERROR_CHECK(i2c_master_read_byte(cmd3, &dato[i], ACK_VAL));
    }
    ESP_ERROR_CHECK(i2c_master_read_byte(cmd3, &dato[len - 1], NACK_VAL));
    ESP_ERROR_CHECK(i2c_master_stop(cmd3));

    esp_err_t ret = i2c_master_cmd_begin(i2c_master_port, cmd3, pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));

    i2c_cmd_link_delete(cmd3);
    if (ret == ESP_OK)
    {
//        ESP_LOGI(TAG, "Recibi dato ozono ADC Adafruit");
        for (int i = 0; i < len; i++)
        {
//            printf("0x%02x ", dato[i]);
            if ((i + 1) % 16 == 0)
            {
                printf("\r\n");
            }
        }
        if (len % 16)
        {
            printf("\r\n");
        }
    }
    else
    {
        miESPes(ret);
    }
    
    //ESP_LOGI(TAG, "My ESP-CODE is %d", ret);

    esp_log_buffer_hex(TAG, dato, len);
    int MSBsinSigno = dato[0] % 128; // El sistema usa valores por encima de 7F para valores negativos
    int result = (int) ((MSBsinSigno * 256 + dato[1]) / 32752.0 * 4096); // Según datasheet, el MSB va primero.

    ESP_LOGI(TAG, "Lectura de ADC I2C adafruit me sale %d ", result);

    return result;
}

void miESPTempHumes(esp_err_t retA){
    if (retA == ESP_OK)
    {
        ESP_LOGI(TAG, "Logre llamar al sensor i2c tempHum y darle un comando");
    }
    else if (retA == ESP_ERR_TIMEOUT)
    {
        ESP_LOGW(TAG, "Bus is busy");
    }
    else if (retA == ESP_ERR_INVALID_ARG)
    {
        ESP_LOGW(TAG, "Parameter error");
    }
    else if (retA == ESP_ERR_INVALID_STATE)
    {
        ESP_LOGW(TAG, "I2C driver not installed on not in master mode");
    }
    else if (retA == ESP_FAIL)
    {
        ESP_LOGW(TAG, "Command error, slave TempHum hasn't ACK the transfer");
    }
    else
    {
        ESP_LOGW(TAG, "Read failed");
    }
//    ESP_LOGI(TAG, "My ESP-CODE is %d", retA);
}

static esp_err_t tempHum_register_read(uint8_t slave_addr, uint8_t reg_addrMSB, uint8_t reg_addrLSB, size_t len)
{

    uint8_t dato[6] = {1, 2, 3, 4, 5, 6}; // El primero y segundo son temperatura, el 4 y 5 humedad. el 3 y 6 son checksum

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    // Primero hacemos que el sensor nos lea el comando
    ESP_ERROR_CHECK(i2c_master_start(cmd));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, (slave_addr << 1) | WRITE_BIT, ACK_VAL));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, reg_addrMSB, ACK_CHECK_EN));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, reg_addrLSB, ACK_CHECK_EN));
    ESP_ERROR_CHECK(i2c_master_stop(cmd));

    esp_err_t retA = i2c_master_cmd_begin(i2c_master_port, cmd, pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));
    vTaskDelay(pdMS_TO_TICKS(20)); // Needs 1 ms to prepare

    i2c_cmd_link_delete(cmd);
    miESPTempHumes(retA);
//    ESP_LOGI(TAG, "My ESP-CODE is %d", retA);

    i2c_cmd_handle_t cmd3 = i2c_cmd_link_create();
    ESP_ERROR_CHECK(i2c_master_start(cmd3));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd3, (slave_addr << 1) | READ_BIT, ACK_VAL));
    int i = 0;
    for (i = 0; i < len - 1; i++)
    {
        ESP_ERROR_CHECK(i2c_master_read_byte(cmd3, &dato[i], ACK_VAL));
    }
    ESP_ERROR_CHECK(i2c_master_read_byte(cmd3, &dato[len - 1], NACK_VAL));
    ESP_ERROR_CHECK(i2c_master_stop(cmd3));

    esp_err_t ret = i2c_master_cmd_begin(i2c_master_port, cmd3, pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));

    i2c_cmd_link_delete(cmd3);
    if (ret == ESP_OK)
    {
//        ESP_LOGI(TAG, "Recibi dato tempHum");
        for (int i = 0; i < len; i++)
        {
//            printf("0x%02x ", dato[i]);
            if ((i + 1) % 16 == 0)
            {
                printf("\r\n");
            }
        }
        if (len % 16)
        {
            printf("\r\n");
        }
    }
    else
    {
        miESPTempHumes(ret);
    }
//    ESP_LOGI(TAG, "My ESP-CODE is %d", ret);

//    esp_log_buffer_hex(TAG, dato, len);
    temperaturaAtmos = -45 + (int) 175 * (dato[0] * 256 + dato[1])/((double) 65536-1); // Este sensor manda primero el MSB y luego el LSB, y eso se debe convertir a las unidades
    humedadAtmos = (int) fmax(0, fmin(100, 100 * (dato[3] * 256 + dato[4])/((double) 65536-1))); // Este sensor manda primero el MSB y luego el LSB, lo convierto a humedad relativa y ajusto al rango 0-100

//    ESP_LOGI(TAG, "Lectura de humedad me sale %d y temperatura %d", humedadAtmos, temperaturaAtmos);
    return ESP_OK;
}

// Fin I2C

// MQTT
esp_mqtt_client_config_t mqtt_cfg = {
    .uri = MQTTURI, //"mqtt://demo.thingsboard.io", // api/v1/Y0kg9ua7tm6s4vaB0X1H/telemetry" ?
    //.event_handle = mqtt_event_handler,
    .port = 1883,
    .username = TOKENMQTT, //"YSRNEFDXnyIGhX9OaylG", // token MQTT
};

esp_mqtt_client_handle_t client;

static esp_err_t mqtt_event_handler_cb(esp_mqtt_event_handle_t event)
{
    // TO-DO ver si puedo hacer que en caso de error reinicie el MQTT con el client
    switch (event->event_id)
    {
    case MQTT_EVENT_CONNECTED:
        ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
        break;
    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
        break;
    case MQTT_EVENT_SUBSCRIBED:
        ESP_LOGI(TAG, "MQTT_EVENT_SUBSCRIBED");
        break;
    case MQTT_EVENT_UNSUBSCRIBED:
        ESP_LOGI(TAG, "MQTT_EVENT_UNSUBSCRIBED");
        break;
    case MQTT_EVENT_PUBLISHED:
        ESP_LOGI(TAG, "MQTT_EVENT_PUBLISHED");
        break;
    case MQTT_EVENT_DATA:
        ESP_LOGI(TAG, "MQTT_EVENT_DATA");
        printf("TOPIC=%.*s\r\n", event->topic_len, event->topic);
        printf("DATA=%.*s\r\n", event->data_len, event->data);
        break;
    case MQTT_EVENT_ERROR:
        ESP_LOGI(TAG, "MQTT_EVENT_ERROR");
        break;
    default:
        ESP_LOGI(TAG, "Other event id:%d", event->event_id);
        break;
    }

    return ESP_OK;
}
static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id,
                               void *event_data)
{
    ESP_LOGD(TAG, "Event dispatched from event loop base=%s, event_id=%d", base, event_id);
    mqtt_event_handler_cb(event_data);
}
static int mqtt_app_start(void)
{
    // Crear json que se quiere enviar al ThingsBoard
    cJSON *root = cJSON_CreateObject();

    cJSON_AddNumberToObject(root, "ozonoBabor", ozonoBabor);                       // En p.p.m.
    cJSON_AddNumberToObject(root, "ozonoEstribor", ozonoEstribor);
    cJSON_AddNumberToObject(root, "ozonoTrasFiltro", ozonoTrasFiltro);
    cJSON_AddNumberToObject(root, "posicionDelTimonExterno", estadoTimonExterno);  // Con los valores asignados
    cJSON_AddNumberToObject(root, "posicionDelTimonInterno", estadoTimonInterno);
    cJSON_AddNumberToObject(root, "tempAtmos", temperaturaAtmos);                  // En grados Celsius
    cJSON_AddNumberToObject(root, "humedadRel", humedadAtmos);                     // En %
    cJSON_AddNumberToObject(root, "energiaSolar", voltajeSolar);                   // En voltios
    cJSON_AddNumberToObject(root, "latitudGNSS", gpslatitude);                     // Grados
    cJSON_AddNumberToObject(root, "longitudGNSS", gpslongitude);
    cJSON_AddNumberToObject(root, "altitudGNSS", gpsaltitude);
    cJSON_AddNumberToObject(root, "velocidadGNSS", gpsspeed);                      // Nudos
    cJSON_AddNumberToObject(root, "orientacionGNSS", gpscourse);                   // Grados

    char *post_data = cJSON_PrintUnformatted(root);
    // Enviar los datos
    int errorcillo = esp_mqtt_client_publish(client, "v1/devices/me/telemetry", post_data, 0, 1, 0); // En v1/devices/me/telemetry sale de la MQTT Device API Reference de ThingsBoard
    cJSON_Delete(root);
    // Free is intentional, it's client responsibility to free the result of cJSON_Print
    free(post_data);

    return errorcillo;
}

// ADC

static bool adc_calibration_init(void)
{
    esp_err_t ret;
    bool cali_enable = false;

    ret = esp_adc_cal_check_efuse(ADC_EXAMPLE_CALI_SCHEME);
    if (ret == ESP_ERR_NOT_SUPPORTED)
    {
        ESP_LOGW(TAG, "Calibration scheme not supported, skip software calibration");
    }
    else if (ret == ESP_ERR_INVALID_VERSION)
    {
        ESP_LOGW(TAG, "eFuse not burnt, skip software calibration");
    }
    else if (ret == ESP_OK)
    {
        cali_enable = true;
        esp_adc_cal_characterize(ADC_UNIT_1, ADC_EXAMPLE_ATTEN, ADC_WIDTH_BIT_DEFAULT, 0, &adc1_chars);
        // TO-DO si usamos un solo módulo adc no deberíamos comentar esto?
        esp_adc_cal_characterize(ADC_UNIT_2, ADC_EXAMPLE_ATTEN, ADC_WIDTH_BIT_DEFAULT, 0, &adc2_chars);
    }
    else
    {
        ESP_LOGE(TAG, "Invalid arg");
    }

    return cali_enable;
}
/*
 * TELEGRAM
 */
int handle_echo_response(struct sh2lib_handle *handle, const char *data, size_t len, int flags)
{
    if (len)
    {
        printf("[echo-response] %.*s\n", len, data);
    }
    if (flags == DATA_RECV_FRAME_COMPLETE)
    {
        printf("[echo-response] Frame fully received\n");
    }
    if (flags == DATA_RECV_RST_STREAM)
    {
        printf("[echo-response] Stream Closed\n");
        conexionEnProceso = 0;
    }
    //    vTaskDelete(NULL);
    return 0;
}

char *cutoff(const char *str, int from, int to)
{
    if (from >= to)
        return NULL;

    char *cut = calloc(sizeof(char), (to - from) + 1);
    char *begin = cut;
    if (!cut)
        return NULL;

    const char *fromit = str + from;
    const char *toit = str + to;
    (void)toit;
    memcpy(cut, fromit, to);
    return begin;
}
/* TO-DO Esto debería centralizarse en algún otro dispositivo, nuestra aspiradora estará aspirando y no hay que saturar el GSM o fundirnos los datos móviles */
int handle_get_response(struct sh2lib_handle *handle, const char *data, size_t len, int flags)
{
    if (len)
    {
        printf("[get-response] %.*s\n", len, data);
        // Ahora busco cosas
        if (data != NULL)
        {
            // Sacamos el json de los datos
            char auxData[len];
            sprintf(auxData, "%.*s", len, data);
            cJSON *root = cJSON_Parse(data);
            char *post_data = cJSON_Print(root);

            if (cJSON_HasObjectItem(root, "ok"))
            {
                printf("Tengo un resultado con algo\n");
                if (cJSON_HasObjectItem(root, "result"))
                {
                    printf("Tengo un resultado no vacio\n");
                    cJSON *parameters = cJSON_GetObjectItemCaseSensitive(root, "result");
                    puts("Parameters:");
                    cJSON *parameter;
                    cJSON_ArrayForEach(parameter, parameters)
                    {

                        /* Each element is an object with unknown field(s) */
                        cJSON *elem;
                        int puedo = 0;
                        cJSON_ArrayForEach(elem, parameter)
                        {

                            if (strcmp(elem->string, "update_id") == 0)
                            {
                                cJSON *update_id = cJSON_GetObjectItemCaseSensitive(parameter, "update_id");
                                char *postid_data = cJSON_Print(update_id);
                                int extraUOffset = atoi(postid_data);

                                printf("El aux del offset: %d\n", extraUOffset);
                                if (ini_OFFSET == 0 || ini_OFFSET < extraUOffset)
                                {
                                    ini_OFFSET = extraUOffset;
                                    puedo = 1;
                                }
                                else if (conproc < extraUOffset)
                                {
                                    conproc = extraUOffset;
                                    printf("valor de conproc ahora = %d", conproc);
                                }
                            }
                            // Esta es nuestra cabeza, la próxima vez que pasamos ni respondemos
                            if ((puedo == 1) && strcmp(elem->string, "message") == 0)
                            {
                                printf("Mira si hay un mensaje encontrado");
                                cJSON *unMensaje = cJSON_GetObjectItemCaseSensitive(parameter, "message");
                                if (cJSON_HasObjectItem(unMensaje, "chat"))
                                {

                                    cJSON *elChat = cJSON_GetObjectItemCaseSensitive(unMensaje, "chat");
                                    cJSON *miId = cJSON_GetObjectItemCaseSensitive(elChat, "id");
                                    if (miId != NULL)
                                    {
                                        printf("Found key '%s'\n", miId->string);
                                        char *postman_data = cJSON_Print(miId);
                                        if (strcmp(postman_data, CHATTOKEN) == 0)
                                        {
                                            printf("Estoy en el chat correcto :)");
                                            if (cJSON_HasObjectItem(unMensaje, "text"))
                                            {
                                                cJSON *elMensajeDeLectura = cJSON_GetObjectItemCaseSensitive(unMensaje, "text");
                                                char *auxMensajeC = cJSON_Print(elMensajeDeLectura); // cJSON_Print

                                                // printf("Mensaje que tengo: %s , de %d caracteres", auxMensajeC, strlen(auxMensajeC)); // Deja las 2 " y no se pueden quitar con facilidad"
                                                char *auxMensaja = cutoff(auxMensajeC, 1, strlen(auxMensajeC));
                                                // printf("Mensaje que tengo: %s , de %d caracteres", auxMensaja, strlen(auxMensaja));
                                                char *auxMensaje = cutoff(auxMensaja, 0, strlen(auxMensaja) - 1);
                                                printf("Mensaje que tengo: %s , de %d caracteres", auxMensaje, strlen(auxMensaje));
                                                // RESPUESTA SI TODO VACÍO O MAL:
                                                char auxRep[] = "No entiendo";

                                                char cmd1[] = "/saluda";
                                                // RESPUESTA: Hola Mundo
                                                char cmd1Rep[] = "Hola Mundo";

                                                char cmd2[] = "/myId";
                                                // RESPUESTA: La id
                                                // char cmd2Rep[] = <Mi Id>; que ya hicimos antes (postman_data)

                                                char cmd3[] = "/restartPlaca";
                                                // RESPUESTA: Hola Mundo
                                                char cmd3Rep[] = "Enseguida la reseteo";

                                                char cmd4[] = "/datos";
                                                // RESPUESTA: Devuelve la info del sistema que recoge
                                                char cmd4Rep[] = "El sistema detecta: ";

                                                char cmdP1[] = "/Q-Que son los sumideros de carbono";
                                                char cmdP1Rep[] = "Los sumideros de carbono son depositos naturales que absorben el carbono de la atmósfera y lo fijan.";

                                                char cmdP2[] = "/Q-Por que en la fotosintesis una planta toma CO2 y libera unicamente O2";
                                                // b. ¿Por qué en la fotosíntesis una planta toma CO2 y libera únicamente O2?, ¿qué pasa con el Carbono en ese proceso?
                                                char cmdP2Rep[] = "Los procesos fotosinteticos de las plantas emplean el CO2 y la luz para producir glucosa, el producto de desecho es el O2, mientras que el carbono se fija.";

                                                char cmdP3[] = "/Q-Que componentes son imprescindibles para que una planta crezca";
                                                // c. ¿Qué componentes son imprescindibles para que una planta crezca?
                                                char cmdP3Rep[] = "las plantas son aerobeas asi que requieren tambien de oxigeno para sobrevivir, ya que tambien respiran, aunque por el dia no se note. Ademas necesitan de agua y sales minerales para realizar sus procesos metabolicos.";

                                                char str[2048];

                                                if (strcmp(auxMensaje, cmd1) == 0)
                                                {
                                                    sprintf(str, "%s&text=%s : %s: %s", POSTUPDATES, auxMensaje, UNIVERSITY, cmd1Rep);
                                                    // sh2lib_do_post( aparentemente para casos simples se hace con un get, que curioso
                                                    sh2lib_do_get(handle, str, handle_echo_response);
                                                }
                                                else if (strcmp(auxMensaje, cmd2) == 0)
                                                {
                                                    sprintf(str, "%s&text=%s : %s: %s", POSTUPDATES, auxMensaje, UNIVERSITY, postman_data);
                                                    sh2lib_do_get(handle, str, handle_echo_response);
                                                }
                                                else if (strcmp(auxMensaje, cmd3) == 0)
                                                {
                                                    sprintf(str, "%s&text=%s : %s: %s", POSTUPDATES, auxMensaje, UNIVERSITY, cmd3Rep);
                                                    sh2lib_do_get(handle, str, handle_echo_response);
                                                    s_reset_state = 20;
                                                }
                                                /*else if (strcmp(auxMensaje, cmd4) == 0)
                                                {
                                                    sprintf(str, "%s&text=%s : %s: %s Vsolar = %d, Vhidro = %d, Tox pre-filtro = %d, Tox post-filtro = %d. SwitchDisplay = %d", POSTUPDATES, auxMensaje, UNIVERSITY, cmd4Rep, voltajeSolar, voltajeHidro, datoI2CCO2legible, datoI2CFotonlegible, s_switch_state);
                                                    sh2lib_do_get(handle, str, handle_echo_response);
                                                    s_reset_state = 20;
                                                }*/
                                                // Las preguntas
                                                else if (strcmp(auxMensaje, cmdP1) == 0)
                                                {
                                                    sprintf(str, "%s&text=%s : %s: %s", POSTUPDATES, auxMensaje, UNIVERSITY, cmdP1Rep);
                                                    sh2lib_do_get(handle, str, handle_echo_response);
                                                }
                                                else if (strcmp(auxMensaje, cmdP2) == 0)
                                                {
                                                    sprintf(str, "%s&text=%s : %s: %s", POSTUPDATES, auxMensaje, UNIVERSITY, cmdP2Rep);
                                                    sh2lib_do_get(handle, str, handle_echo_response);
                                                }
                                                else if (strcmp(auxMensaje, cmdP3) == 0)
                                                {
                                                    sprintf(str, "%s&text=%s : %s: %s", POSTUPDATES, auxMensaje, UNIVERSITY, cmdP3Rep);
                                                    sh2lib_do_get(handle, str, handle_echo_response);
                                                    s_reset_state = 20;
                                                }
                                                else
                                                {
                                                    sprintf(str, "%s&text=%s : %s: %s", POSTUPDATES, auxMensaje, UNIVERSITY, auxRep);
                                                    sh2lib_do_get(handle, str, handle_echo_response);
                                                }
                                            }
                                            else
                                                printf("Mensaje de otro tipo");
                                        }
                                        else
                                        {
                                            printf("Este no es mi chat :|");
                                        }
                                        free(postman_data);
                                    }
                                }
                                else
                                    printf("No se encuentra chat :/");
                            }
                        }
                    }
                }
                else
                    printf("Algo va mal con el resultado");
            }
            else
                printf("Algo va mal con el ok");

            cJSON_Delete(root);
            // Free is intentional, it's client responsibility to free the result of cJSON_Print
            free(post_data);
        }
    }
    if (flags == DATA_RECV_FRAME_COMPLETE)
    {
        printf("[get-response] Frame fully received\n");
    }
    if (flags == DATA_RECV_RST_STREAM)
    {
        printf("[get-response] Stream Closed\n");
        if (ini_OFFSET > conproc)
        {
            conexionEnProceso = 1;
        }
        else
            conexionEnProceso = 0;
        printf("primer get %d con ini offset = %d: Conexion en proceso = %d", conproc, ini_OFFSET, conexionEnProceso);
    }
    return 0;
}

int send_put_data(struct sh2lib_handle *handle, char *buf, size_t length, uint32_t *data_flags)
{
#define DATA_TO_SEND "Hello World"
    int copylen = strlen(DATA_TO_SEND);
    if (copylen < length)
    {
        printf("[data-prvd] Sending %d bytes\n", copylen);
        memcpy(buf, DATA_TO_SEND, copylen);
    }
    else
    {
        copylen = 0;
    }

    (*data_flags) |= NGHTTP2_DATA_FLAG_EOF;
    return copylen;
}

static void set_time(void) // Tiempo es necesario para HTTP2
{
    struct timeval tv = {
        .tv_sec = 1509449941,
    };
    struct timezone tz = {
        0, 0};
    settimeofday(&tv, &tz);

    /* Start SNTP service */
    sntp_setoperatingmode(SNTP_OPMODE_POLL);
    sntp_init();
}

static void http2_task(void *args)
{
    /* Set current time: proper system time is required for TLS based
     * certificate verification.
     */
    set_time();

    /* HTTP2: one connection multiple requests. Do the TLS/TCP connection first */
    printf("Connecting to server\n");
    struct sh2lib_config_t cfg = {
        .uri = HTTP2_SERVER_URI,
        .cacert_buf = server_rootTelegram_cert_pem_start,
        .cacert_bytes = server_rootTelegram_cert_pem_end - server_rootTelegram_cert_pem_start,
    };

    while (1)
    {
        conexionEnProceso = 1;
        struct sh2lib_handle hd;
        if (sh2lib_connect(&cfg, &hd) != 0)
        {
            printf("Failed to connect\n");
            http2Caido = 1;
            vTaskDelete(NULL); // TO-DO ver si se puede quitar o poner de forma que al cerrarse active una variable global que indique durante algún ciclo que debe reiniciarse.
            return;
        }
        printf("Connection done\n");

        int offset = ini_OFFSET; // Offset inicial

        char str[256];
        offset = ini_OFFSET;
        printf("My offset es %d", offset);
        sprintf(str, "/bot%s/getUpdates?offset=%d", TELEGRAMTOKEN, offset);
        sh2lib_do_get(&hd, str, handle_get_response);
        /* HTTP GET  */
        while (conexionEnProceso > 0) // Ahora se pide ejecutar todo lo que se ponga arriba hasta que desconecte
        {
            /* Process HTTP2 send/receive */
            if (sh2lib_execute(&hd) < 0)
            {
                printf("Error in send/receive\n");
                break;
            }
            vTaskDelay(1);
        }
        vTaskDelay(pdMS_TO_TICKS(1000));

        sh2lib_free(&hd);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    vTaskDelete(NULL);
}

/* An HTTP GET handler */
/* TO-DO AJUSTAR EN LA VERSIÓN FINAL, NO ES NECESARIO EXCEPTO PARA RESETEAR A LA PARTICIÓN DE LA OTA*/
static esp_err_t root_get_handler(httpd_req_t *req)
{
    char mensajito[494];

    char finDePagina[] = "";
    const char *mess;

    // Esto es para refrescar cada 10 segundos, es la versión válida de "<title>Servidor Especial</title><meta http-equiv='refresh' content='10'>";
    httpd_resp_set_hdr(req, "Refresh", "10");
    httpd_resp_set_type(req, "text/html");
    if (s_reset_state != 0)
    {
        sprintf(mensajito, "<h1>La ESP32 se va a resetear en %d</h1>", s_reset_state);
        mess = strcat(mensajito, finDePagina);
        // httpd_resp_send(req, resetmess, HTTPD_RESP_USE_STRLEN);
    }
    else
    {
        sprintf(mensajito, "<h1> Acceso Web http a AspiradO3 </h1><h1> (Refresh 10 segundos) </h1> <p><a href='/reset'><button style='height:50px;width:100px'>Resetear ESP32</button></a></p> <h1> V sol (mV): %d</h1> <h1> Ozono Estribor (ppm): %d</h1><h1> Ozono Babor (ppm): %d</h1><h1> Ozono tras filtros (ppm): %d</h1>", voltajeSolar, ozonoEstribor, ozonoBabor, ozonoTrasFiltro);

        mess = strcat(mensajito, finDePagina);
    }
    httpd_resp_send(req, mess, HTTPD_RESP_USE_STRLEN);

    return ESP_OK;
}

static esp_err_t buttON_get_handler(httpd_req_t *req)
{
    ESP_LOGI(TAG, "Activo el LED");
    s_led_state = true;
    blink_led();
    httpd_resp_set_hdr(req, "Location", "/");
    httpd_resp_set_status(req, "302");
    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, "Redirigiendo", HTTPD_RESP_USE_STRLEN);

    return ESP_OK;
}

static esp_err_t buttOFF_get_handler(httpd_req_t *req)
{
    ESP_LOGI(TAG, "Desactivo el LED");
    s_led_state = false;
    blink_led();
    httpd_resp_set_hdr(req, "Location", "/");
    httpd_resp_set_status(req, "302");
    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, "Redirigiendo", HTTPD_RESP_USE_STRLEN);

    return ESP_OK;
}
static esp_err_t buttReset_get_handler(httpd_req_t *req)
{
    s_reset_state = 5;
    httpd_resp_set_hdr(req, "Location", "/");
    httpd_resp_set_status(req, "302");
    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, "Redirigiendo", HTTPD_RESP_USE_STRLEN);

    return ESP_OK;
}

#if CONFIG_EXAMPLE_ENABLE_HTTPS_USER_CALLBACK
/**
 * Example callback function to get the certificate of connected clients,
 * whenever a new SSL connection is created
 *
 * Can also be used to other information like Socket FD, Connection state, etc.
 *
 * NOTE: This callback will not be able to obtain the client certificate if the
 * following config `Set minimum Certificate Verification mode to Optional` is
 * not enabled (enabled by default in this example).
 *
 * The config option is found here - Component config → ESP-TLS
 *
 */
void https_server_user_callback(esp_https_server_user_cb_arg_t *user_cb)
{
    ESP_LOGI(TAG, "Session Created!");
    ESP_LOGI(TAG, "Socket FD: %d", user_cb->tls->sockfd);

    const mbedtls_x509_crt *cert;
    const size_t buf_size = 1024;
    char *buf = calloc(buf_size, sizeof(char));
    if (buf == NULL)
    {
        ESP_LOGE(TAG, "Out of memory - Callback execution failed!");
        return;
    }

    mbedtls_x509_crt_info((char *)buf, buf_size - 1, "    ", &user_cb->tls->servercert);
    ESP_LOGI(TAG, "Server certificate info:\n%s", buf);
    memset(buf, 0x00, buf_size);

    cert = mbedtls_ssl_get_peer_cert(&user_cb->tls->ssl);
    if (cert != NULL)
    {
        mbedtls_x509_crt_info((char *)buf, buf_size - 1, "    ", cert);
        ESP_LOGI(TAG, "Peer certificate info:\n%s", buf);
    }
    else
    {
        ESP_LOGW(TAG, "Could not obtain the peer certificate!");
    }

    free(buf);
}
#endif

static const httpd_uri_t root = {
    .uri = "/",
    .method = HTTP_GET,
    .handler = root_get_handler};

static const httpd_uri_t reset = {
    .uri = "/reset",
    .method = HTTP_GET,
    .handler = buttReset_get_handler};

static const httpd_uri_t off = {
    .uri = "/off",
    .method = HTTP_GET,
    .handler = buttOFF_get_handler};
static const httpd_uri_t on = {
    .uri = "/on",
    .method = HTTP_GET,
    .handler = buttON_get_handler};

static httpd_handle_t start_webserver(void)
{
    httpd_handle_t server = NULL;

    // Start the httpd server
    ESP_LOGI(TAG, "Starting server");

    httpd_ssl_config_t conf = HTTPD_SSL_CONFIG_DEFAULT();

    extern const unsigned char cacert_pem_start[] asm("_binary_cacert_pem_start");
    extern const unsigned char cacert_pem_end[] asm("_binary_cacert_pem_end");
    conf.cacert_pem = cacert_pem_start;
    conf.cacert_len = cacert_pem_end - cacert_pem_start;

    extern const unsigned char prvtkey_pem_start[] asm("_binary_prvtkey_pem_start");
    extern const unsigned char prvtkey_pem_end[] asm("_binary_prvtkey_pem_end");
    conf.prvtkey_pem = prvtkey_pem_start;
    conf.prvtkey_len = prvtkey_pem_end - prvtkey_pem_start;

#if CONFIG_EXAMPLE_ENABLE_HTTPS_USER_CALLBACK
    conf.user_cb = https_server_user_callback;
#endif
    esp_err_t ret = httpd_ssl_start(&server, &conf);
    if (ESP_OK != ret)
    {
        ESP_LOGI(TAG, "Error starting server!");
        return NULL;
    }

    // Set URI handlers
    ESP_LOGI(TAG, "Registering URI handlers");
    httpd_register_uri_handler(server, &root); // Es importante poner los uri handlers acá para iniciarlos
    httpd_register_uri_handler(server, &reset);
    httpd_register_uri_handler(server, &off);
    httpd_register_uri_handler(server, &on);
    return server;
}

static void stop_webserver(httpd_handle_t server)
{
    // Stop the httpd server
    httpd_ssl_stop(server);
}

static void disconnect_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data)
{
    ESP_LOGI(TAG, "ME HE DESCONECTADO");
    if (enEllo == 0){
        enEllo = 1;
        xSemaphoreGive(Semaphore);
    }

    httpd_handle_t *server = (httpd_handle_t *)arg;
    if (*server)
    {
        stop_webserver(*server);
        *server = NULL;
    }
}

static void connect_handler(void *arg, esp_event_base_t event_base,
                            int32_t event_id, void *event_data)
{
    ESP_LOGI(TAG, "ME HE CONECTADO"); // TO-DO verificar
    httpd_handle_t *server = (httpd_handle_t *)arg;
    if (*server == NULL)
    {
        *server = start_webserver();
    }
}
/*
 * TELEGRAM
 */
esp_err_t _http_event_handler(esp_http_client_event_t *evt)
{
    switch (evt->event_id)
    {
    case HTTP_EVENT_ERROR:
        ESP_LOGD(TAG, "HTTP_EVENT_ERROR");
        break;
    case HTTP_EVENT_ON_CONNECTED:
        ESP_LOGD(TAG, "HTTP_EVENT_ON_CONNECTED");
        break;
    case HTTP_EVENT_HEADER_SENT:
        ESP_LOGD(TAG, "HTTP_EVENT_HEADER_SENT");
        break;
    case HTTP_EVENT_ON_HEADER:
        ESP_LOGD(TAG, "HTTP_EVENT_ON_HEADER, key=%s, value=%s", evt->header_key, evt->header_value);
        break;
    case HTTP_EVENT_ON_DATA:
        ESP_LOGD(TAG, "HTTP_EVENT_ON_DATA, len=%d", evt->data_len);
        break;
    case HTTP_EVENT_ON_FINISH:
        ESP_LOGD(TAG, "HTTP_EVENT_ON_FINISH");
        break;
    case HTTP_EVENT_DISCONNECTED:
        ESP_LOGD(TAG, "HTTP_EVENT_DISCONNECTED");
        break;
    }
    return ESP_OK;
}
// Volver a la partición de fábrica
void backtofactory()
{
    esp_partition_iterator_t pi;    // Iterator for find
    const esp_partition_t *factory; // Factory partition
    esp_err_t err;
    pi = esp_partition_find(ESP_PARTITION_TYPE_APP,            // Get partition iterator for
                            ESP_PARTITION_SUBTYPE_APP_FACTORY, // factory partition
                            "factory");
    if (pi == NULL) // Check result
    {
        ESP_LOGE(TAG, "Failed to find factory partition");
    }
    else
    {
        factory = esp_partition_get(pi);           // Get partition struct
        esp_partition_iterator_release(pi);        // Release the iterator
        err = esp_ota_set_boot_partition(factory); // Set partition for boot
        if (err != ESP_OK)                         // Check error
        {
            ESP_LOGE(TAG, "Failed to set boot partition");
        }
        else
        {
            esp_restart(); // Restart ESP
        }
    }
}

// Configuración de los 4 pines analógicos
static void configure_analog(void)
{
    ESP_LOGI(TAG, "Configuring ESP32 analog pins");

    gpio_reset_pin(PIN_ANALOG4);
    /* Set the GPIO 32 as a push/pull output */
    gpio_set_direction(PIN_ANALOG4, GPIO_MODE_INPUT);
}

/* TO-DO ESTO SE BORRA, NO TENEMOS NI DISPLAY (QUIÉN LO VA A VER A 100 M DE ALTURA, SOLO NOS VALE PARA ARRANCAR, SI ACASO ENTONCES LO AJUSTAS PARA UN MANUAL DE USUARIO)
Y EL SLEEP EN EL AIRE ES SUICIDIO */
void sleepDelDisplay(void)
{
#ifdef CONFIG_EXAMPLE_EXT1_WAKEUP
    const int ext_wakeup_pin_1 = 2;
    const uint64_t ext_wakeup_pin_1_mask = 1ULL << ext_wakeup_pin_1;

    printf("Enabling EXT1 wakeup on pin GPIO%d\n", ext_wakeup_pin_1);
    esp_sleep_enable_ext1_wakeup(ext_wakeup_pin_1_mask, ESP_EXT1_WAKEUP_ANY_HIGH);

    /* If there are no external pull-up/downs, tie wakeup pins to inactive level with internal pull-up/downs via RTC IO
     * during deepsleep. However, RTC IO relies on the RTC_PERIPH power domain. Keeping this power domain on will
     * increase some power comsumption. */
#if CONFIG_EXAMPLE_EXT1_USE_INTERNAL_PULLUPS
    esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_ON);
    rtc_gpio_pullup_dis(ext_wakeup_pin_1);
    rtc_gpio_pulldown_en(ext_wakeup_pin_1);
#endif // CONFIG_EXAMPLE_EXT1_USE_INTERNAL_PULLUPS
#endif // CONFIG_EXAMPLE_EXT1_WAKEUP
}
void deQueMeLevante(int sleep_time_ms)
{
    switch (esp_sleep_get_wakeup_cause())
    {
#if CONFIG_EXAMPLE_EXT0_WAKEUP
    case ESP_SLEEP_WAKEUP_EXT0:
    {
        printf("Wake up from ext0\n");
        break;
    }
#endif // CONFIG_EXAMPLE_EXT0_WAKEUP
#ifdef CONFIG_EXAMPLE_EXT1_WAKEUP
    case ESP_SLEEP_WAKEUP_EXT1:
    {
        uint64_t wakeup_pin_mask = esp_sleep_get_ext1_wakeup_status();
        if (wakeup_pin_mask != 0)
        {
            int pin = __builtin_ffsll(wakeup_pin_mask) - 1;
            printf("Wake up from GPIO %d\n", pin);
        }
        else
        {
            printf("Wake up from GPIO\n");
        }
        break;
    }
#endif // CONFIG_EXAMPLE_EXT1_WAKEUP
#if SOC_GPIO_SUPPORT_DEEPSLEEP_WAKEUP
    case ESP_SLEEP_WAKEUP_GPIO:
    {
        uint64_t wakeup_pin_mask = esp_sleep_get_gpio_wakeup_status();
        if (wakeup_pin_mask != 0)
        {
            int pin = __builtin_ffsll(wakeup_pin_mask) - 1;
            printf("Wake up from GPIO %d\n", pin);
        }
        else
        {
            printf("Wake up from GPIO\n");
        }
        break;
    }
#endif // SOC_GPIO_SUPPORT_DEEPSLEEP_WAKEUP
    case ESP_SLEEP_WAKEUP_TIMER:
    {
        printf("Wake up from timer. Time spent in deep sleep: %dms\n", sleep_time_ms);
        break;
    }
#ifdef CONFIG_EXAMPLE_TOUCH_WAKEUP
    case ESP_SLEEP_WAKEUP_TOUCHPAD:
    {
        printf("Wake up from touch on pad %d\n", esp_sleep_get_touchpad_wakeup_status());
        break;
    }
#endif // CONFIG_EXAMPLE_TOUCH_WAKEUP
#ifdef CONFIG_EXAMPLE_ULP_TEMPERATURE_WAKEUP
#if CONFIG_IDF_TARGET_ESP32
    case ESP_SLEEP_WAKEUP_ULP:
    {
        printf("Wake up from ULP\n");
        int16_t diff_high = (int16_t)ulp_data_read(3);
        int16_t diff_low = (int16_t)ulp_data_read(4);
        if (diff_high < 0)
        {
            printf("High temperature alarm was triggered\n");
        }
        else if (diff_low < 0)
        {
            printf("Low temperature alarm was triggered\n");
        }
        else
        {
            assert(false && "temperature has stayed within limits, but got ULP wakeup\n");
        }
        break;
    }
#endif // CONFIG_IDF_TARGET_ESP32
#endif // CONFIG_EXAMPLE_ULP_TEMPERATURE_WAKEUP
    case ESP_SLEEP_WAKEUP_UNDEFINED:
    default:
        printf("Not a deep sleep reset\n");
    }

#ifdef CONFIG_EXAMPLE_ULP_TEMPERATURE_WAKEUP
#if CONFIG_IDF_TARGET_ESP32
    if (esp_sleep_get_wakeup_cause() != ESP_SLEEP_WAKEUP_UNDEFINED)
    {
        printf("ULP did %d temperature measurements in %d ms\n", ulp_data_read(1), sleep_time_ms);
        printf("Initial T=%d, latest T=%d\n", ulp_data_read(0), ulp_data_read(2));
    }
#endif // CONFIG_IDF_TARGET_ESP32
#endif // CONFIG_EXAMPLE_ULP_TEMPERATURE_WAKEUP
}

// PROGRAMA PRINCIPAL
void app_main(void)
{
    /*
     * Información del sleep
     */
    //struct timeval now;
    //gettimeofday(&now, NULL);
    //int sleep_time_ms = (now.tv_sec - sleep_enter_time.tv_sec) * 1000 + (now.tv_usec - sleep_enter_time.tv_usec) / 1000;
    /*
     * Configurar periféricos, LED, motores y los inputs analógicos
     */
    configure_led();
    configure_motor_continuo(PIN_ASPIRADOR);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, SERVO_PULSE_TIMONINTERNO); // To drive a RC servo, one MCPWM generator is enough
    mcpwm_gpio_init(MCPWM_UNIT_1, MCPWM0A, SERVO_PULSE_TIMONEXTERNO); // For the second RC servo

    mcpwm_config_t pwm_config = {
        .frequency = 50, // frequency = 50Hz, i.e. for every servo motor time period should be 20ms
        .cmpr_a = 0,     // duty cycle of PWMxA = 0
        .counter_mode = MCPWM_UP_COUNTER,
        .duty_mode = MCPWM_DUTY_MODE_0,
    };
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);
    mcpwm_init(MCPWM_UNIT_1, MCPWM_TIMER_0, &pwm_config);

    //configure_switch();
    configure_analog();

    // iniciar I2C
    // Iniciar el display
    SSD1306_t dev;
    ESP_ERROR_CHECK(i2c_master_init(&dev, I2C_MASTER_SDA_IO, I2C_MASTER_SCL_IO, 0));
    ESP_LOGI(TAG, "I2C initialized successfully");
    // Variables auxiliares del I2C display, e inicialización extra del display
    char primeralineChar[32];
    char segundalineChar[32];
    char terceralineChar[32];
    char cuartalineChar[32];
    char quintalineChar[32];
    ssd1306_clear_screen(&dev, false); // El display a 0, no a valores basura

#if CONFIG_SSD1306_128x64
    ESP_LOGI(TAG, "Panel is 128x64");
    ssd1306_init(&dev, 128, 64);
#endif // CONFIG_SSD1306_128x64
#if CONFIG_SSD1306_128x32
    ESP_LOGI(TAG, "Panel is 128x32");
    ssd1306_init(&dev, 128, 32);
#endif // CONFIG_SSD1306_128x32

    // Iniciar ADC
#if CONFIG_IDF_TARGET_ESP32 // El WiFi usa en adc2 así que no podemos usar ese segundo módulo, mejor multiplexamos el adc1 y hay variables qur no necesitamos usar
    ESP_LOGI(TAG, "Estamos en ESP-32");
#else
    esp_err_t ret = ESP_OK;
#endif

    uint32_t voltage = 0;
    bool cali_enable = adc_calibration_init();

    // ADC1 config
    ESP_ERROR_CHECK(adc1_config_width(ADC_WIDTH_BIT_DEFAULT));
    // ADC2 config
    //original ESP_ERROR_CHECK(adc2_config_channel_atten(ADC2_EXAMPLE_CHAN0, ADC_EXAMPLE_ATTEN));
#if CONFIG_IDF_TARGET_ESP32     
    ESP_ERROR_CHECK(adc1_config_channel_atten(ADC4_EXAMPLE_CHAN0, ADC_EXAMPLE_ATTEN));
#else
    ESP_ERROR_CHECK(adc2_config_channel_atten(ADC4_EXAMPLE_CHAN0, ADC_EXAMPLE_ATTEN));
#endif


    /*
     * Informacion del chip
     */
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);
    printf("This is %s chip with %d CPU core(s), WiFi%s%s, ",
           CONFIG_IDF_TARGET,
           chip_info.cores,
           (chip_info.features & CHIP_FEATURE_BT) ? "/BT" : "",
           (chip_info.features & CHIP_FEATURE_BLE) ? "/BLE" : "");

    printf("silicon revision %d, ", chip_info.revision);

    printf("%dMB %s flash\n", spi_flash_get_chip_size() / (1024 * 1024),
           (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");

    printf("Minimum free heap size: %d bytes\n", esp_get_minimum_free_heap_size());

    static httpd_handle_t server = NULL;
    

    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    // SEMAFOROS
    Semaphore = xSemaphoreCreateBinary();

    /*
     * Registrar handlers de evento para montar el servidor cuando se conectan el Wi-Di o Ethernet, y parar cuando se desconecta.
     */

#ifdef CONFIG_EXAMPLE_CONNECT_WIFI
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &connect_handler, &server));
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, WIFI_EVENT_STA_DISCONNECTED, &disconnect_handler, &server));
#endif
#ifdef CONFIG_EXAMPLE_CONNECT_ETHERNET
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_ETH_GOT_IP, &connect_handler, &server));
    ESP_ERROR_CHECK(esp_event_handler_register(ETH_EVENT, ETHERNET_EVENT_DISCONNECTED, &disconnect_handler, &server));
#endif

    /* This helper function configures Wi-Fi or Ethernet, as selected in menuconfig.
     * Read "Establishing Wi-Fi or Ethernet Connection" section in
     * examples/protocols/README.md for more information about this function.
     */
    // TO-DO encontrar una forma de poder conectar a wi-fi sin que sea obligatorio conectar al wi-fi antes de que funcionen las cosas
    ESP_ERROR_CHECK(example_connect());

    /*
     * Establecer la conexión MQTT, crearé al cliente
     */
    client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_start(client);

    // Task HTTP2 para Telegram TO-DO descomentar???
    // xTaskCreate(&http2_task, "http2_task", (1024 * 32), NULL, 5, NULL);

    // TEST MOTOR ASPIRADOR TO-DO TO VERIFY
    s_aspirador_state = true;
    blink_motorAspirador();



    // TO-DO TESTS DE LOS OTROS DOS MOTORES

    // Task mqtt? TO-DO?
    //  xTaskCreate(mqtt_app_start, "mqtt_send_data_0", 1024 * 2, (void *)0, 10, NULL);

    // Task ADC? TO-DO?
    // xTaskCreate(adc_app_loop, "adc_receive_data_0", 1024 * 2, (void *)0, 10, NULL);

    /*GPS*/

    /* NMEA parser configuration */
/* DEL GITHUB DE OTRO COMPAÑERO*/
    init_uart();

    //Start test task
    // GPS
    xTaskCreate(rx_task, "uart_rx_task", 8192, NULL, configMAX_PRIORITIES-4, NULL); // 8192, no 1024 * 2
    // GSM
    xTaskCreate(tx_task, "uart_tx_task", 8192, NULL, configMAX_PRIORITIES-3, NULL);
/* FIN DEL GITHUB DE OTRO COMPAÑERO*/


// CALIBRACION SENSORES OZONO

    int count = 0;
    int countReadInRowBabor = 0;
    int countReadInRowEstribor = 0;
    int countReadInRowTrasFiltro = 0;
    int timeToReadConsistency = MQ131_DEFAULT_STABLE_CYCLE;
    int ozonoBaborAnt = -1;
    int ozonoEstriborAnt = -1;
    int ozonoTrasFiltroAnt = -1;
    int ozonoBaborAntAnt = -1;
    int ozonoEstriborAntAnt = -1;
    int ozonoTrasFiltroAntAnt = -1;
    int ozonoBaborC = -1;
    int ozonoEstriborC = -1;
    int ozonoTrasFiltroC  = -1;

    while(1){ // TO-DO quitar de la version final
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    /*while(countReadInRowBabor <= timeToReadConsistency && countReadInRowEstribor <= timeToReadConsistency && countReadInRowTrasFiltro <= timeToReadConsistency) {
        ESP_LOGI(TAG, "Procedo a leer ADC 0 (CALIBRACION)");
        ozonoBaborC = ADCADAFRUIT12C_register_read(ADCI2CADAFRUIT_AADR, ADCI2CADAFRUIT_CONFIGADDR, ADCI2CADAFRUIT_CONFIGREGMUXA0, ADCI2CADAFRUIT_CONFIGLSB, ADCI2CADAFRUIT_CONVADDR, 2);
        vTaskDelay(pdMS_TO_TICKS(1000));

        ESP_LOGI(TAG, "Procedo a leer ADC 1 (CALIBRACION)");
        ozonoEstriborC = ADCADAFRUIT12C_register_read(ADCI2CADAFRUIT_AADR, ADCI2CADAFRUIT_CONFIGADDR, ADCI2CADAFRUIT_CONFIGREGMUXA1, ADCI2CADAFRUIT_CONFIGLSB, ADCI2CADAFRUIT_CONVADDR, 2);
        vTaskDelay(pdMS_TO_TICKS(1000));

        ESP_LOGI(TAG, "Procedo a leer ADC 2 (CALIBRACION)");
        ozonoTrasFiltroC = ADCADAFRUIT12C_register_read(ADCI2CADAFRUIT_AADR, ADCI2CADAFRUIT_CONFIGADDR, ADCI2CADAFRUIT_CONFIGREGMUXA2, ADCI2CADAFRUIT_CONFIGLSB, ADCI2CADAFRUIT_CONVADDR, 2);

        vTaskDelay(pdMS_TO_TICKS(1000));
    
        if(countReadInRowBabor <= timeToReadConsistency && (uint32_t)ozonoBaborAnt != (uint32_t)ozonoBaborC && (uint32_t)ozonoBaborAntAnt != (uint32_t)ozonoBaborC) {
            ozonoBaborAntAnt = ozonoBaborAnt;
            ozonoBaborAnt = ozonoBaborC;
            countReadInRowBabor = 0;
            ESP_LOGI(TAG, "Reseteo inRow Babor (CALIBRACION)");
        } else {
            countReadInRowBabor++;
            ESP_LOGI(TAG, "inRow Babor = %d", countReadInRowBabor);
        }
        if(countReadInRowEstribor <= timeToReadConsistency && (uint32_t)ozonoEstriborAnt != (uint32_t)ozonoEstriborC && (uint32_t)ozonoEstriborAntAnt != (uint32_t)ozonoEstriborC) {
            ozonoEstriborAntAnt = ozonoEstriborAnt;
            ozonoEstriborAnt = ozonoEstriborC;
            countReadInRowEstribor = 0;
            ESP_LOGI(TAG, "Reseteo inRow Estribor (CALIBRACION)");
        } else {
            countReadInRowEstribor++;
            ESP_LOGI(TAG, "inRow Estribor = %d", countReadInRowEstribor);
        }
        if(countReadInRowTrasFiltro <= timeToReadConsistency && (uint32_t)ozonoTrasFiltroAnt != (uint32_t)ozonoTrasFiltroC && (uint32_t)ozonoTrasFiltroAntAnt != (uint32_t)ozonoTrasFiltroC) {
            ozonoTrasFiltroAntAnt = ozonoTrasFiltroAnt;
            ozonoTrasFiltroAnt = ozonoTrasFiltroC;
            countReadInRowTrasFiltro = 0;
            ESP_LOGI(TAG, "Reseteo inRow TrasFiltro (CALIBRACION)");
        } else {
            countReadInRowTrasFiltro++;
            ESP_LOGI(TAG, "inRow TrasFiltro = %d", countReadInRowTrasFiltro);
        }

    count++;
    }*/
    ESP_LOGI(TAG, "Calibracion COMPLETADA tras %d segundos", count);
    /* SECCION DE CALIBRACION DE SENSORES DE OZONO ON-THE-FLY */
    double R0babor = getResistencia(ozonoBaborC, RESLBABOR);
    double R0estribor = getResistencia(ozonoBaborC, RESLESTRIBOR);
    double R0trasFiltro = getResistencia(ozonoBaborC, RESLTRASFILTRO);

    double correccionSensorBabor = 0.0 ;
    double correccionSensorEstribor = 0.0;
    double correccionSensorTrasFiltro = 0.0;
    int corrInicialSensorMayor = 0;
    int corrInicialSensorMedio = 0;
    int corrInicialSensorMenor = 0;
    double multCorrEstribor = 1.0;
    double multCorrBabor = 1.0;
    double multCorrTrasFiltro = 1.0;


    /*
     * Bucle infinito TO-DO mejorar con Tasks?
     */
    while (1)
    {

        if (gpio_get_level(PIN_SWITCH)) // TO-DO AJUSTAR? BORRAR DE LA VERSION FINAL
        {
//            ESP_LOGI(TAG, "Switch display: ON");
            s_switch_state = true;
            ssd1306_contrast(&dev, 0xff);
            sprintf(primeralineChar, " O3 bab: %02d", ozonoBabor);
            sprintf(segundalineChar, " O3 est: %02d", ozonoEstribor);
            sprintf(terceralineChar, " O3 tra: %02d", ozonoTrasFiltro);
            sprintf(cuartalineChar, " V Solar: %02d", voltajeSolar);
            ssd1306_display_text(&dev, 0, primeralineChar, 32, false);
            ssd1306_display_text(&dev, 1, segundalineChar, 32, false);
            ssd1306_display_text(&dev, 2, terceralineChar, 32, false);
            ssd1306_display_text(&dev, 3, cuartalineChar, 32, false);
#if CONFIG_SSD1306_128x64
            ssd1306_display_text(&dev, 4, "                ", 32, false);
            ssd1306_display_text(&dev, 5, "     GyFhi      ", 32, false);
            ssd1306_display_text(&dev, 6, "                ", 32, false);
            ssd1306_display_text(&dev, 7, "     U.P.M.     ", 32, false);
#endif // CONFIG_SSD1306_128x32
        }
        else
        {
            s_switch_state = false;
            ssd1306_clear_screen(&dev, false);
        }
        if (s_reset_state != 0)
        {
            s_reset_state -= 1;

            char c[] = "Reseteo en ";

            char tiempo[20];

            itoa(s_reset_state, tiempo, 10);
            const char *mens;
            mens = strcat(c, tiempo);

            ESP_LOGI(TAG, "%s", mens);

            if (s_reset_state == 0)
            {
                fflush(stdout);
                backtofactory();
            }
        }

        /* FASE 1: LECTURA ADC DE PANEL SOLAR */

        ESP_LOGI(TAG, "Procedo a medir ADC ESP32");


#if CONFIG_IDF_TARGET_ESP32 // El WiFi usa en adc2 así que no podemos usar ese segundo módulo
        adc_raw[0][0] = adc1_get_raw(ADC4_EXAMPLE_CHAN0);
#else
        do
        {
            ret = adc2_get_raw(ADC4_EXAMPLE_CHAN0, ADC_WIDTH_BIT_DEFAULT, &adc_raw[0][0]);
        } while (ret == ESP_ERR_INVALID_STATE);
        ESP_ERROR_CHECK(ret);
#endif
        ESP_LOGI(TAG_CH[0][0], "raw voltaje Solar data: %d", adc_raw[0][0]);
        if (cali_enable)
        {
#if CONFIG_IDF_TARGET_ESP32
            voltage = esp_adc_cal_raw_to_voltage(adc_raw[0][0], &adc1_chars);
#else
            voltage = esp_adc_cal_raw_to_voltage(adc_raw[0][0], &adc2_chars);
#endif
            voltajeSolar = voltage;
            ESP_LOGI(TAG_CH[0][0], "cali voltaje Solar data: %d mV", voltage);
        }

        vTaskDelay(pdMS_TO_TICKS(2000)); // Delays para asegurar lecturas ADC correctas
    
        /* FASE 2: LECTURA DE HUMEDAD Y TEMPERATURA ATMOSFERICAS */
        ESP_LOGI(TAG, "Procedo a leer I2C de TempHum");
        ESP_ERROR_CHECK(tempHum_register_read(TEMPHUM_SENSOR_ADDR, COMANDO_TEMPHUM_MSB, COMANDO_TEMPHUM_LSB, 6));
        /* FASE 2.2 LECTURA OZONO I2C */
        ESP_LOGI(TAG, "Procedo a leer I2C de ADCs ozono");
        ESP_LOGI(TAG, "Procedo a leer ADC 0");
        //int ozonoAux = ADCADAFRUIT12C_register_read(ADCI2CADAFRUIT_AADR, ADCI2CADAFRUIT_CONFIGADDR, ADCI2CADAFRUIT_CONFIGREGMUXA3, ADCI2CADAFRUIT_CONFIGLSB, ADCI2CADAFRUIT_CONVADDR, 2);
        ozonoBaborC = ADCADAFRUIT12C_register_read(ADCI2CADAFRUIT_AADR, ADCI2CADAFRUIT_CONFIGADDR, ADCI2CADAFRUIT_CONFIGREGMUXA0, ADCI2CADAFRUIT_CONFIGLSB, ADCI2CADAFRUIT_CONVADDR, 2);
        ESP_LOGI(TAG, "Procedo a leer ADC 1");
        ozonoEstriborC = ADCADAFRUIT12C_register_read(ADCI2CADAFRUIT_AADR, ADCI2CADAFRUIT_CONFIGADDR, ADCI2CADAFRUIT_CONFIGREGMUXA1, ADCI2CADAFRUIT_CONFIGLSB, ADCI2CADAFRUIT_CONVADDR, 2);
        ESP_LOGI(TAG, "Procedo a leer ADC 2");
        ozonoTrasFiltroC = ADCADAFRUIT12C_register_read(ADCI2CADAFRUIT_AADR, ADCI2CADAFRUIT_CONFIGADDR, ADCI2CADAFRUIT_CONFIGREGMUXA2, ADCI2CADAFRUIT_CONFIGLSB, ADCI2CADAFRUIT_CONVADDR, 2);

        vTaskDelay(pdMS_TO_TICKS(1000)); // El I2C puede tardar hasta 11 segundos

        /* FASE 3: AJUSTE DE LECTURAS DE OZONO */

        ozonoBabor = multCorrBabor * ajustarValoresOzono(ozonoBaborC +  corrInicialSensorMayor, humedadAtmos, temperaturaAtmos, correccionSensorBabor, RESLBABOR, R0babor); // TO-DO ajuste
        ozonoEstribor =  multCorrEstribor * ajustarValoresOzono(ozonoEstriborC + corrInicialSensorMedio, humedadAtmos, temperaturaAtmos, correccionSensorEstribor, RESLESTRIBOR, R0estribor);
        ozonoTrasFiltro = multCorrTrasFiltro * ajustarValoresOzono(ozonoTrasFiltroC + corrInicialSensorMenor, humedadAtmos, temperaturaAtmos, correccionSensorTrasFiltro, RESLTRASFILTRO, R0trasFiltro);
        ESP_LOGI(TAG, "correcion O3 babor: %d", ozonoBabor );
        ESP_LOGI(TAG, "correcion O3 estribor: %d", ozonoEstribor );
        ESP_LOGI(TAG, "correcion O3 tras filtro: %d", ozonoTrasFiltro );

        /* FASE 4: CORRECIÓN DE RUMBO SEGÚN SENSORES Y GPS/GSM */
        /*TO-DO añade márgenes de tolerancia y sistema de control
        NORBERTO DECIDIÓ QUE HICIÉSEMOS A OJO */
        if (ozonoBabor == ozonoEstribor){
            ESP_LOGI(TAG, "O3B == 03E");
            estadoTimonExterno = (fmax(-85, fmin(85, (ozonoEstribor -ozonoBabor)/10 * (gpsspeed + 1.0))) - estadoTimonExterno)/2.0; //0;
        } else if (ozonoBabor > ozonoEstribor) { // A mayor velocidad y diferencia, más brusco el giro
            ESP_LOGI(TAG, "O3B > 03E");
            estadoTimonExterno = (fmax(-85, fmin(85, (ozonoEstribor -ozonoBabor)/10 * (gpsspeed + 1.0))) - estadoTimonExterno)/2.0; //-85;
        } else {
            ESP_LOGI(TAG, "O3B < 03E");
            estadoTimonExterno = (fmax(-85, fmin(85, (ozonoEstribor -ozonoBabor)/10 * (gpsspeed + 1.0))) - estadoTimonExterno)/2.0; //= 85;
        }
        /*TO-DO LOS GPS PARA TIMÓN INTERNO*/
        if (gpsspeed - gpsspeedAnt > 50 || gpsspeed > 50) {
            /*TO-DO EXPANDIR UNA VEZ TENGA EL MÓDULO CON LA INFO ADECUADA PARA AJUSTAR EL MOVIMIENTO Y LA VELOCIDAD, TOMO LO DE GPS*/
            estadoTimonInterno = -45;
        } else {
            estadoTimonInterno = 45;
        }
        ESP_LOGI(TAG, "MUEVO TIMON EXTERNO A %d e INTERNO a %d", estadoTimonExterno, estadoTimonInterno);
        
        ESP_ERROR_CHECK(mcpwm_set_duty_in_us(MCPWM_UNIT_1, MCPWM_TIMER_0, MCPWM_OPR_A, convert_servo_180_angle_to_duty_us(estadoTimonExterno)));
        vTaskDelay(pdMS_TO_TICKS(200));
        ESP_ERROR_CHECK(mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, convert_servo_90_angle_to_duty_us(estadoTimonInterno)));
        vTaskDelay(pdMS_TO_TICKS(200)); //Add delay, since it takes time for servo to rotate, generally 100ms/60degree rotation under 5V power supply

        /*
        *MQTT: los datos obtenidos los mandamos a Thingsboard
        */
        ESP_LOGI(TAG, "TRATO DE MANDAR POR WI-FI");
        int problemaWiFi = mqtt_app_start();
        if (problemaWiFi == -1 || enEllo == 1) {
            // Indico al GSM que debe enviarlo
            ESP_LOGI(TAG, "MANDO POR GSM");
        } else {
            ESP_LOGI(TAG, "ENVIO WI-FI EXITOSO");
        }
    }
}