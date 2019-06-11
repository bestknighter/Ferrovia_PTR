#include <Arduino_FreeRTOS.h>

#include <task.h>
#include <semphr.h>

//#define DEBUG





#define LED_PIN_GREEN 02
#define LED_PIN_YELLW 03
#define LED_PIN_RED_1 04
#define LED_PIN_RED_2 05
#define LED_PIN_CAR 06
#define LED_PIN_PPL 07

void LED_Controller( void* pvParameters );

/* Demonstração dessa tabela. OBS.: PPL só tem Green, Red1 e Red2.
 *    CAR PPL Pin
 * G  1   0   02
 * Y  0   0   03
 * R1 0   1   04
 * R2 0   0   05
 */
byte ledStatus[3][4] = {0};
SemaphoreHandle_t mtx_ledStatus;






// Criação das tasks e a colocando no scheduler do FreeRTOS
void setup() {
  ledStatus[2][0] = LED_PIN_GREEN;
  ledStatus[2][1] = LED_PIN_YELLW;
  ledStatus[2][2] = LED_PIN_RED_1;
  ledStatus[2][3] = LED_PIN_RED_2;
  mtx_ledStatus = xSemaphoreCreateMutex(); // TODO: Tem que checar se não retornou null...
  
  xTaskCreate( LED_Controller, "LED_Controller", 192 /* STACK SIZE */, NULL, 2, NULL );

  #ifdef DEBUG
  Serial.begin( 9600 );
  #endif // DEBUG
}

// Vazio. Como usamos FreeRTOS, tudo é feito por tasks.
void loop() {}








/**
 * @brief Controla quais leds vão ficar acesos e por quanto tempo
 * 
 * Usando o fenômeno da persistência da visão, precisamos percorrer o ciclo
 * de alternar as luzes em no mínimo 60Hz.
 */
void LED_Controller(  void* pvParameters __attribute__((unused)) ) {
  /* SETUP */
  byte currentRow = 0;
  while( xSemaphoreTake( mtx_ledStatus, portMAX_DELAY ) != pdTRUE ) {}
  for( byte i = 0; i < 4; i++ ) {
    pinMode( ledStatus[2][i], OUTPUT );
  }
  pinMode( LED_PIN_CAR, OUTPUT );
  pinMode( LED_PIN_PPL, OUTPUT );
  xSemaphoreGive( mtx_ledStatus );

  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = (1000/4) / portTICK_PERIOD_MS; // 4 Hz
  
  #ifdef DEBUG
  bool highWaterMarked = false;
  #endif // DEBUG

  /* LOOP */
  while(true) {
    if( xSemaphoreTake( mtx_ledStatus, xFrequency/2 ) == pdTRUE ) {
      for( byte i = 0; i < 4; i++ ) {
        if( i == currentRow ) digitalWrite( ledStatus[2][i], HIGH );
        else digitalWrite( ledStatus[2][i], LOW );
      }
      digitalWrite( LED_PIN_CAR, ledStatus[0][currentRow] == 1 ? HIGH : LOW );
      digitalWrite( LED_PIN_PPL, ledStatus[1][currentRow] == 1 ? HIGH : LOW );
      xSemaphoreGive( mtx_ledStatus );
      
      currentRow = currentRow+1 % 4;
    }


    // Para verificação do uso da stack
    #ifdef DEBUG
    if( !highWaterMarked ) {
      Serial.print( uxTaskGetStackHighWaterMark(NULL) );
      highWaterMarked = true;
    }
    #endif // DEBUG
    
    vTaskDelayUntil( &xLastWakeTime, xFrequency );
  }
}
