/*********************************************************
* Por: 
        Ageu Silva
        Roberto
        Paulo
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
static dht_t sensor1;


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

    //xTaskCreate(vPiscarLED,Nome da Task,Stack Size,parametro passado para a task,Prioridade da task,handle da task);
    if( xTaskCreate( &vTaskRecSerial, "Task Rec Serial", 2048, NULL, 1, xTaskRecSerial )!= pdTRUE )
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
	
    /* atribui pino de dados de entrada do sensor */
    DHT11_init(dhtDataPin);

    /* leitura de dados do sensor dht */
    sensor1.temperature = DHT11_read().temperature;
    sensor1.humidity = DHT11_read().humidity;
    sensor1.status = DHT11_read().status;

    /* exibe dados */
    #if(DEBUG)
        printf( "Temperatura: %d\n", sensor1.temperature );
        printf( "Humidade: %d\n", sensor1.humidity );
        printf("Status: %d\n", sensor1.status);
    #endif
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

                // Aqui entra a função da fila. Por favor, comente o codigo abaixo
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
                uart_write_bytes(UART_NUM_0, (char*)cBufferTx_USB, strlen((char*)cBufferTx_USB));
                uart_write_bytes(UART_NUM_0, (char*)"\r\n", strlen((char*)"\r\n"));
            }

            else{
                memset((char*)cBufferRx_USB_Comp, '\0', strlen((char*)cBufferRx_USB_Comp));
            }
        }
    }       
}

/*===========================================================================================*/