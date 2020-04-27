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

/*log*/
#include "esp_system.h"
#include "esp_log.h"
#include "esp_event_loop.h"


/*Definições*/
#define DEBUG       1
#define LED         2

#define ACENDE_LED()    gpio_set_level( LED, 1 )
#define APAGA_LED()     gpio_set_level( LED, 0 )

#define BUF_SIZE        (1024)

/* definições para leitura de dados sensor DHT*/
#define dhtDataPin GPIO_NUM_4
#define sampleTime 1000

/*handle do Semaforo*/
SemaphoreHandle_t xMutex;


/* Variáveis para Armazenar o handle da Task */
TaskHandle_t xTaskRecSerial;

/* timer handle */
TimerHandle_t xTimer1;

/* Protótipo das Tasks*/
void vTaskRecSerial( void *pvParameter );
void callBackTimer1( TimerHandle_t pxTimer );

/* Funções auxiliares */
void vInitHW(void);
void vSetup(void);

/* estrutura de dadOs para o sensor */
typedef struct dht {
    uint32_t temperature;
    uint32_t humidity;
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
    gpio_pad_select_gpio(LED );
    gpio_set_direction(LED , GPIO_MODE_OUTPUT);
}

/* Função app_main*/
void app_main()
{
    vSetup();

    while(true)
    {
        vTaskDelay(pdMS_TO_TICKS(3000));    /* Delay de 3 segundos */
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

    //criação de fila do callBackTimer1
  	xMonitor = xQueueCreate( 10, sizeof( int ) );

    if(xMonitor == NULL){
  		ESP_LOGI("Erro na criação da Queue.\n");
  	}

    //criação de fila do Serial
  	xSerial = xQueueCreate( 10, sizeof( int ) );

    if(xSerial == NULL){
  		ESP_LOGI("Erro na criação da Queue.\n");
  	}

    //xTaskCreate(vPiscarLED,Nome da Task,Stack Size,parametro passado para a task,Prioridade da task,handle da task);
    if( xTaskCreate( &vTaskRecSerial, "Task Rec Serial", 2048, NULL, 1, xTaskRecSerial )!= pdTRUE )
    {
        if( DEBUG )
        {
            ESP_LOGI( TAG1, "error - nao foi possivel alocar task_Contador.\n" );
        }
        return;
    }

    if( xTaskCreate( &vTaskControl, "TaskControl", 2048, NULL, 1, xTaskControl )!= pdTRUE )
    {
        if( DEBUG )
        {
            ESP_LOGI( TAG1, "error - nao foi possivel alocar task_Contador.\n" );
        }
        return;
    }

    /* cria e testa auto-reload timer 1 */
    xTimer1 = xTimerCreate( "Timer1", pdMS_TO_TICKS(sampleTime), pdTRUE, 0, callBackTimer1 );
    if( xTimer1 == NULL ) {
        ESP_LOGE( "Erro","Não foi possível criar o temporizador de amostragem (xTimer1 Auto-Reload)" );
        while(1);
    }
    else {
        #if(DEBUG)
            ESP_LOGI( "Info","Temporizador de amostragem Criado" );
        #endif
        /* timer1 start */
        xTimerStart( xTimer1, 0 );
    }

}

/* timer para amostragem cíclica de dados do sensor DHT11 */
void callBackTimer1( TimerHandle_t pxTimer ) {

    dht_t sensorSent;

    /* atribui pino de dados de entrada do sensor */
    DHT11_init(dhtDataPin);

    /* leitura de dados do sensor dht */
    sensorRead.temperature = DHT11_read().temperature;
    sensorRead.humidity = DHT11_read().humidity;
    sensorRead.status = DHT11_read().status;

    if(!xQueueSend(xMonitor, &sensorSent, 500)) {
      ESP_LOGI("Falha ao enviar o valor para a fila dentro de 500ms.\n");
    }

    if(!xQueueSend(xSerial, &sensorSent, 500)) {
      ESP_LOGI("Falha ao enviar o valor para a fila dentro de 500ms.\n");
    }

/*    #if(DEBUG)

        //exibe dados
        printf( "Temperatura: %d\n", sensorRead.temperature );
        printf( "Umidade: %d\n", sensorRead.humidity );
        printf("Status: %d\n", sensorRead.status);

      #endif


*/

}


/* Task de controle: Responsável por habilitar/desabilitar aquecedor e/ou umidificador */

void vTaskControl( void *pvParameter )
{

  dht_t sensorReceived; //struct local para receber valor da struct dht_t sensorSent
  uint8_t setPoint[3];  //variavel local para receber da fila xSerial a temperatura ou umidade

  uint8_t temp = 0;     //variavel local para receber setPoint temperatura
  uint8_t umid = 0;     //variavel local para receber setPoint umidade





  while(true){

    if(!xQueueReceive(xSerial, &setPoint, 200)) {
      ESP_LOGI("Falha ao receber o valor da fila dentro de 2500ms.\n");
    }

    if (!xQueueReceive(xMonitor, &sensorReceived, 2500)) {
      ESP_LOGI("Falha ao receber o valor da fila dentro de 2500ms.\n");
    }

    switch (setPoint[0]) {

      case 'T':
        temp = atoi(setPoint[1]); //valor convertido de string para int
        break;

      case 'U':
        umid = atoi(setPoint[1]); //valor convertido de string para int
        break;

    }

    if (temp != 0) {
      if (sensorReceived.temperature > temp) {
          /*ligar GPIO Umidificador*/
          //configurar GPIO
          /*envia ON Umidificador : xQueueSend para a fila xSerial */
          if(!xQueueSend(xSerial, "UON", 200)) {
            ESP_LOGI("Falha ao enviar o valor para a fila dentro de 500ms.\n");
          }

      }
      else {
        /*desligar GPIO Umidificador*/
        //configurar GPIO
        /*envia OFF Umidificador : xQueueSend para a fila xSerial */
        if(!xQueueSend(xSerial, "UOFF", 200)) {
          ESP_LOGI("Falha ao enviar o valor para a fila dentro de 500ms.\n");
        }
      }

    }

    if (umid != 0) {
      if (sensorReceived.humidity > umid) {
          /*ligar GPIO Aquecedor*/
          //configurar GPIO
          /*envia ON Aquecedor : xQueueSend para a fila xSerial */
          if(!xQueueSend(xSerial, "AON", 200)) {
            ESP_LOGI("Falha ao enviar o valor para a fila dentro de 500ms.\n");
          }

      }
      else {
        /*desligar GPIO Aquecedor*/
        //configurar GPIO
        /*envia OFF Aquecedor : xQueueSend para a fila xSerial */
        if(!xQueueSend(xSerial, "AOFF", 200)) {
          ESP_LOGI("Falha ao enviar o valor para a fila dentro de 500ms.\n");
        }

      }

    }

  }

}



/*===========================================================================================*/
// Função que recebe os dados da serial USB e trata as informações de protocolo

void vTaskRecSerial( void *pvParameter )
{

    (void) pvParameter;

    char *dadosNameTask = pcTaskGetTaskName(xTaskRecSerial);
    if( DEBUG )
    {
        ESP_LOGI( TAG1, "%s \n", dadosNameTask);
    }

    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    uart_param_config(UART_NUM_0, &uart_config);
    uart_driver_install(UART_NUM_0, BUF_SIZE * 2, 0, 0, NULL, 0);

    while (true)
    {
        len = 0;
        len = uart_read_bytes(UART_NUM_0, cBufferRx_USB, BUF_SIZE, 20 / portTICK_RATE_MS);

        if(len > 0)
        {
            strcpy((char*)cBufferRx_USB_Comp, (char*)cBufferRx_USB);

            if(strncmp((char*)cBufferRx_USB_Comp, (char*)"[SETTEMP=", strlen((char*)"[SETTEMP=")) == false)
            {

                uint8_t cSetTemp[3];

                cSetTemp[0] = 'T';
                cSetTemp[1] = cBufferRx_USB_Comp[9];
                cSetTemp[2] = cBufferRx_USB_Comp[10];

                strcpy((char*)cBufferTx_USB, (char*)cSetTemp);

                if(!xQueueSend(xSerial, &cSetTemp, 500)) {
                  ESP_LOGI("Falha ao enviar o valor para a fila dentro de 500ms.\n");
                }
                //uart_write_bytes(UART_NUM_0, (char*)cBufferTx_USB, strlen((char*)cBufferTx_USB));
                //uart_write_bytes(UART_NUM_0, (char*)"\r\n", strlen((char*)"\r\n"));


            }

            else if(strncmp((char*)cBufferRx_USB_Comp, (char*)"[SETUMID=", strlen((char*)"[SETUMID=")) == false)
            {
                uint8_t cSetUmi[3];

                cSetUmi[0] = 'U';
                cSetUmi[1] = cBufferRx_USB_Comp[9];
                cSetUmi[2] = cBufferRx_USB_Comp[10];

                strcpy((char*)cBufferTx_USB, (char*)cSetUmi);

                if(!xQueueSend(xSerial, &cSetUmi, 500)) {
                  ESP_LOGI("Falha ao enviar o valor para a fila dentro de 500ms.\n");
                }

                //uart_write_bytes(UART_NUM_0, (char*)cBufferTx_USB, strlen((char*)cBufferTx_USB));
                //uart_write_bytes(UART_NUM_0, (char*)"\r\n", strlen((char*)"\r\n"));

            }

            else{
                memset((char*)cBufferRx_USB_Comp, '\0', strlen((char*)cBufferRx_USB_Comp));
            }
        }
    }
}

/*===========================================================================================*/
