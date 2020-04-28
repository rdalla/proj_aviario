/*********************************************************
* Por:
        Ageu Silva
        Roberto Dalla Valle Filho
        Paulo Paixão
*********************************************************/


/*inclusão de Bibliotecas*/
#include <stdio.h>
#include <String.h>
#include "_ansi.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "sdkconfig.h"


/*inclusão das Biblioteca do FreeRTOS*/
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/timers.h"

/* bibliotecas auxiliares */
#include "dht11.h"
#include "DHT22.h"

/*log*/
#include "esp_system.h"
#include "esp_log.h"
#include "esp_event_loop.h"


/*Definições*/
#define DEBUG          1
#define AQUECEDOR      2
#define UMIDIFICADOR   18

#define LIGA_AQUECEDOR()        gpio_set_level( AQUECEDOR, 1 )
#define DESLIGA_AQUECEDOR()     gpio_set_level( AQUECEDOR, 0 )

#define LIGA_UMIDIFICADOR()     gpio_set_level( UMIDIFICADOR, 1 )
#define DESLIGA_UMIDIFICADOR()  gpio_set_level( UMIDIFICADOR, 0 )

#define BUF_SIZE        (1024)

/* definições para leitura de dados sensor DHT*/
#define dhtDataPin GPIO_NUM_4
#define sampleTime 2500

/*handle do Semaforo*/
SemaphoreHandle_t xMutex = 0;


/*handle do Queue*/
QueueHandle_t xMonitor_Serial = 0;
QueueHandle_t xMonitor_Control = 0;
QueueHandle_t xControl_Serial = 0;
QueueHandle_t xSerial_Control = 0;


/* Variáveis para Armazenar o handle da Task */
TaskHandle_t xTaskComSerial;
TaskHandle_t xTaskControl;

/* timer handle */
TimerHandle_t xTimer1;

/* Protótipo das Tasks*/
void vTaskComSerial( void *pvParameter );
void vTaskControl( void *pvParameter );
void callBackTimer1( TimerHandle_t pxTimer );

/* Funções auxiliares */
void vInitHW(void);
void vSetup(void);

/* estrutura de dadOs para o sensor */
typedef struct dht {
    uint16_t temperature;
    uint16_t humidity;
    int8_t status;
} dht_t;

//static dht_t sensor1;


/* Variável Global */
static const char *TAG1 = "TASK";

static uint8_t cBufferRx_USB[BUF_SIZE]; //= (uint8_t *) malloc(BUF_SIZE);
static uint8_t cBufferTx_USB[BUF_SIZE];
static uint8_t cBufferRx_USB_Comp[BUF_SIZE];
static int len = 0;


/* Função Init Harware */
void vInitHW(void)
{
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    uart_param_config(UART_NUM_0, &uart_config);
    uart_driver_install(UART_NUM_0, BUF_SIZE * 2, 0, 0, NULL, 0);

    gpio_pad_select_gpio(AQUECEDOR );
    gpio_set_direction(AQUECEDOR , GPIO_MODE_OUTPUT);
    gpio_pad_select_gpio(UMIDIFICADOR );
    gpio_set_direction(UMIDIFICADOR , GPIO_MODE_OUTPUT);
}

/* Função app_main*/
void app_main()
{
    vSetup();

    while(true)
    {
        vTaskDelay(pdMS_TO_TICKS(10));    /* Delay de 3 segundos */
    }
}


/*Função setup - é chamada apenas uma vez na inicialização */
void vSetup(void)
{

    /* Configuração do Hardware */
    vInitHW();

    /* informações do Hardware */
    esp_log_level_set(TAG1, ESP_LOG_INFO);

    /* Cria semaforo binário */
    xMutex = xSemaphoreCreateMutex();

    /* comando necessário pois o semáforo começa em zero */
    xSemaphoreGive( xMutex );

    //criação de fila do xSerialControl -- TaskControl
  	xMonitor_Serial = xQueueCreate( 10, sizeof( int ) );
    if(xMonitor_Serial == NULL)
    {
  		ESP_LOGI(TAG1, "Erro na criação da Queue.\n");
  	}

    //criação de fila do xSerialControl -- TaskControl
  	xMonitor_Control = xQueueCreate( 10, sizeof( int ) );
    if(xMonitor_Control == NULL)
    {
  		ESP_LOGI(TAG1, "Erro na criação da Queue.\n");
  	}

    //criação de fila do xSerialControl -- TaskControl
  	xControl_Serial = xQueueCreate( 10, sizeof( int ) );
    if(xControl_Serial == NULL)
    {
  		ESP_LOGI(TAG1, "Erro na criação da Queue.\n");
  	}

    //criação de fila do xSerialControl -- TaskControl
  	xSerial_Control = xQueueCreate( 10, sizeof( int ) );
    if(xSerial_Control == NULL)
    {
  		ESP_LOGI(TAG1, "Erro na criação da Queue.\n");
  	}

    //xTaskCreate(vPiscarLED,Nome da Task,Stack Size,parametro passado para a task,Prioridade da task,handle da task);
    if( xTaskCreate( &vTaskComSerial, "Task Rec Serial", 4096, NULL, 1, xTaskComSerial )!= pdTRUE )
    {
        if( DEBUG )
        {
            ESP_LOGI( TAG1, "error - nao foi possivel alocar task_Contador.\n" );
        }
        return;
    }

    if( xTaskCreate( &vTaskControl, "TaskControl", 4096, NULL, 2, xTaskControl )!= pdTRUE )
    {
        if( DEBUG )
        {
            ESP_LOGI( TAG1, "error - nao foi possivel alocar task_Contador.\n" );
        }
        return;
    }

    /* cria e testa auto-reload timer 1 */
    xTimer1 = xTimerCreate( "Timer1", pdMS_TO_TICKS(sampleTime), pdTRUE, 0, callBackTimer1 );
    if( xTimer1 == NULL )
    {
        ESP_LOGE( "Erro","Não foi possível criar o temporizador de amostragem (xTimer1 Auto-Reload)" );
        while(1);
    }
    else
    {
        #if(DEBUG)
            ESP_LOGI( "Info","Temporizador de amostragem Criado" );
        #endif
        /* timer1 start */
        xTimerStart( xTimer1, 0 );
    }

}

/*================================================================================*/
/* timer para amostragem cíclica de dados do sensor DHT11 / DHT22 */
void callBackTimer1( TimerHandle_t pxTimer ) {

    dht_t sensorSent;

    /* atribui pino de dados de entrada do sensor */
    //DHT11_init(dhtDataPin);
    setDHTgpio( GPIO_NUM_4 );

    int ret = readDHT();
		errorHandler(ret);

    /* leitura de dados do sensor dht */
    sensorSent.temperature = (uint16_t)getTemperature();
    sensorSent.humidity = (uint16_t)getHumidity();
    //sensorSent.status = DHT11_read().status;

    if(!xQueueSend(xMonitor_Serial, &sensorSent, 10))
    {

    }

    if(!xQueueSend(xMonitor_Control, &sensorSent, 10))
    {

    }
}

/*================================================================================*/
/* Task de controle: Responsável por habilitar/desabilitar aquecedor e/ou umidificador */

void vTaskControl( void *pvParameter )
{

  dht_t sensorReceived; //struct local para receber valor da struct dht_t sensorSent
  uint8_t setPoint[2];  //variavel local para receber da fila xSerial a temperatura ou umidade

  uint8_t temp = 0;     //variavel local para receber setPoint temperatura
  uint8_t umid = 0;     //variavel local para receber setPoint umidade

  while(true)
  {

    if(!xQueueReceive(xSerial_Control, &setPoint, 10))
    {

    }
    else
    {
        switch (setPoint[0])
        {
            case 'T':
              temp = setPoint[1]; //valor convertido de string para int
              break;

            case 'U':
              umid = setPoint[1]; //valor convertido de string para int
              break;
        }
    }

    if (!xQueueReceive(xMonitor_Control, &sensorReceived, 10))
    {

    }
    else
    {
        //if (temp != 0)
        //{
            if (sensorReceived.temperature < temp)
            {
                /*desligar GPIO Aquecedor*/
                LIGA_AQUECEDOR();

                /*envia ON Aquecedor : xQueueSend para a fila xSerial */
                if(!xQueueSend(xControl_Serial, &"AON", 10))
                {

                }
            }
            else if (sensorReceived.temperature > temp)
            {
                /*ligar GPIO Aquecedor*/
                DESLIGA_AQUECEDOR();

                /*envia OFF Aquecedor : xQueueSend para a fila xSerial */
                if(!xQueueSend(xControl_Serial, &"AOFF", 10))
                {

                }
            }

        //}

        //if (umid != 0)
        //{
            if (sensorReceived.humidity < umid)
            {
                /*ligar GPIO umidificador*/
                LIGA_UMIDIFICADOR();

                /*envia ON Umidificador : xQueueSend para a fila xSerial */
                if(!xQueueSend(xControl_Serial, &"UON", 10))
                {

                }
            }
            else if (sensorReceived.humidity > umid)
            {
                /*desligar GPIO umidificador*/
                DESLIGA_UMIDIFICADOR();

                /*envia OFF Umidificador : xQueueSend para a fila xSerial */
                if(!xQueueSend(xControl_Serial, &"UOFF", 10))
                {

                }
            }
         //}
    }
  }
}

/*===========================================================================================*/
// Função que recebe os dados da serial USB e trata as informações de protocolo

void vTaskComSerial( void *pvParameter )
{

    (void) pvParameter;

    char *dadosNameTask = pcTaskGetTaskName(xTaskComSerial);
    if( DEBUG )
    {
        ESP_LOGI( TAG1, "%s \n", dadosNameTask);
    }

    uint8_t cStatusSaidas[3];

    dht_t sensorReceive;

    while (true)
    {
        len = 0;
        len = uart_read_bytes(UART_NUM_0, cBufferRx_USB, BUF_SIZE, 20 / portTICK_RATE_MS);

        if(len > 0)
        {
            strcpy((char*)cBufferRx_USB_Comp, (char*)cBufferRx_USB);

            if(strncmp((char*)cBufferRx_USB_Comp, (char*)"[SETTEMP=", strlen((char*)"[SETTEMP=")) == false)
            {

                uint8_t cSetTemp[2];

                cSetTemp[0] = 'T';
                cSetTemp[1] = ((cBufferRx_USB_Comp[9] - '0') * 10) + (cBufferRx_USB_Comp[10] - '0');

                if(!xQueueSend(xSerial_Control, &cSetTemp, 10))
                {

                }

            }

            else if(strncmp((char*)cBufferRx_USB_Comp, (char*)"[SETUMID=", strlen((char*)"[SETUMID=")) == false)
            {
                uint8_t cSetUmi[2];

                cSetUmi[0] = 'U';
                cSetUmi[1] = ((cBufferRx_USB_Comp[9] - '0') * 10) + (cBufferRx_USB_Comp[10] - '0');

                if(!xQueueSend(xSerial_Control, &cSetUmi, 10))
                {

                }

            }

            else{
                memset((char*)cBufferRx_USB_Comp, '\0', strlen((char*)cBufferRx_USB_Comp));
            }
        }


        if(!xQueueReceive(xControl_Serial, &cStatusSaidas, 10))
        {

        }
        else
        {
            if(strncmp((char*)cStatusSaidas, (char*)"AON", strlen((char*)"AON")) == false)
            {
                xSemaphoreTake( xMutex, portMAX_DELAY );
                uart_write_bytes(UART_NUM_0, (char*)"[AQUECEDOR=LIGADO]", strlen((char*)"[AQUECEDOR=LIGADO]"));
                xSemaphoreGive( xMutex );
            }
            else if(strncmp((char*)cStatusSaidas, (char*)"AOFF", strlen((char*)"AOFF")) == false)
            {
                xSemaphoreTake( xMutex, portMAX_DELAY );
                uart_write_bytes(UART_NUM_0, (char*)"[AQUECEDOR=DESLIGADO]", strlen((char*)"[AQUECEDOR=DESLIGADO]"));
                xSemaphoreGive( xMutex );
            }

            if(strncmp((char*)cStatusSaidas, (char*)"UON", strlen((char*)"UON")) == false)
            {
                xSemaphoreTake( xMutex, portMAX_DELAY );
                uart_write_bytes(UART_NUM_0, (char*)"[UMIDIFICADOR=LIGADO]", strlen((char*)"[UMIDIFICADOR=LIGADO]"));
                xSemaphoreGive( xMutex );
            }
            else if(strncmp((char*)cStatusSaidas, (char*)"UOFF", strlen((char*)"UOFF")) == false)
            {
                xSemaphoreTake( xMutex, portMAX_DELAY );
                uart_write_bytes(UART_NUM_0, (char*)"[UMIDIFICADOR=DESLIGADO]", strlen((char*)"[UMIDIFICADOR=DESLIGADO]"));
                xSemaphoreGive( xMutex );
            }
        }

        if(!xQueueReceive(xMonitor_Serial, &sensorReceive, 10))
        {

        }
        else
        {
            xSemaphoreTake( xMutex, portMAX_DELAY );
            sprintf((char*)cBufferTx_USB, "[TEMPERATURA=%d]", sensorReceive.temperature);
            uart_write_bytes(UART_NUM_0, (char*)cBufferTx_USB, strlen((char*)cBufferTx_USB));
            memset((char*)cBufferRx_USB_Comp, '\0', strlen((char*)cBufferRx_USB_Comp));

            sprintf((char*)cBufferTx_USB, "[UMIDADE=%d]", sensorReceive.humidity);
            uart_write_bytes(UART_NUM_0, (char*)cBufferTx_USB, strlen((char*)cBufferTx_USB));
            memset((char*)cBufferRx_USB_Comp, '\0', strlen((char*)cBufferRx_USB_Comp));
            xSemaphoreGive( xMutex );
        }


     }
}

/*===========================================================================================*/
