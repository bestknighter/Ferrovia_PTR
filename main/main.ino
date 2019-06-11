#include <Arduino_FreeRTOS.h>
#include <task.h>

//#define DEBUG

#define LED_PIN_GREEN xx
#define LED_PIN_YELLW xx
#define LED_PIN_RED_1 xx
#define LED_PIN_RED_2 xx
#define LED_PIN_CAR xx
#define LED_PIN_PPL xx

void LED_Controller( void* pvParameters );

// Criação das tasks e a colocando no scheduler do FreeRTOS
void setup() {
  xTaskCreate( LED_Controller, "LED_Controller", 128 /* STACK SIZE */, NULL, 2, NULL );

  #ifdef DEBUG
  Serial.begin( 9600 );
  #endif // DEBUG
}

// Vazio. Como usamos FreeRTOS, tudo é feito por tasks.
void loop() {}

void LED_Controller(  void* pvParameters __attribute__((unused)) ) {
  /* SETUP */
//  pinMode( LED_PIN_GREEN, OUTPUT );
//  pinMode( LED_PIN_YELLW, OUTPUT );
//  pinMode( LED_PIN_RED_1, OUTPUT );
//  pinMode( LED_PIN_RED_2, OUTPUT );
//  pinMode( LED_PIN_CAR, OUTPUT );
//  pinMode( LED_PIN_PPL, OUTPUT );

  #ifdef DEBUG
  bool highWaterMarked = false;
  #endif // DEBUG

  /* LOOP */
  while(true) {

    // Para verificação do uso da stack
    #ifdef DEBUG
    if( !highWaterMarked ) {
      Serial.print( uxTaskGetStackHighWaterMark( NULL ) );
      highWaterMarked = true;
    }
    #endif // DEBUG
  }
}
