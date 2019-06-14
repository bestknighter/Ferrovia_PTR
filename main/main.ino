#include <Arduino_FreeRTOS.h>

#include <task.h>
#include <semphr.h>
#include <event_groups.h>

//#define DEBUG

#define E_CHANGESTATE 0x1

#define LED_PIN_GREEN 05
#define LED_PIN_YELLW 04
#define LED_PIN_RED_1 02
#define LED_PIN_RED_2 03
#define LED_PIN_CAR 06
#define LED_PIN_PPL 07
#define CAR 0
#define PPL 1
#define PIN 2
#define GREEN 0
#define YELLW 1
#define RED_1 2
#define RED_2 3

/* Demonstração dessa tabela. OBS.: PPL só tem Green, Red1 e Red2.
 *    CAR PPL Pin
 * G  1   0   02
 * Y  0   0   03
 * R1 0   1   04
 * R2 0   0   05
 */
byte ledStatus[3][4] = {0};
SemaphoreHandle_t mtx_ledStatus;
void LED_Controller( void* pvParameters );

enum lightStates {
  OFF = 0,
  GO,
  ATTENTION,
  STOP,
  E_STOP,
  ERR
};
int lightState[2];
SemaphoreHandle_t mtx_lightState;
void semaphoreStateChanger( void* pvParameters );

enum semaphoreStates {
  CARS_GO = 0,
  PEDESTRIANS_GO,
  EMERGENCY_GO,
  TRAIN_GO
};
short semaphoreDesiredState;
SemaphoreHandle_t mtx_semaphoreDesiredState;
EventGroupHandle_t eventsGroup;
void semaphoreController( void* pvParameters );



// Criação das tasks e a colocando no scheduler do FreeRTOS
void setup() {
  ledStatus[PIN][GREEN] = LED_PIN_GREEN;
  ledStatus[PIN][YELLW] = LED_PIN_YELLW;
  ledStatus[PIN][RED_1] = LED_PIN_RED_1;
  ledStatus[PIN][RED_2] = LED_PIN_RED_2;
  mtx_ledStatus = xSemaphoreCreateMutex(); // TODO: Tem que checar se não retornou null...
  mtx_lightState = xSemaphoreCreateMutex(); // TODO: Tem que checar se não retornou null...
  mtx_stateVariables = xSemaphoreCreateMutex(); // TODO: Tem que checar se não retornou null...
  eventsGroup = xEventGroupCreate(); // TODO: Tem que checar se não retornou null...
  
  xTaskCreate( LED_Controller, "LED_Controller", 192 /* STACK SIZE */, NULL, 2, NULL );
  xTaskCreate( semaphoreStateChanger, "semaphoreStateChanger", 192 /* STACK SIZE */, NULL, 2, NULL );
  xTaskCreate( semaphoreController, "semaphoreController", 192 /* STACK SIZE */, NULL, 2, NULL );

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
    pinMode( ledStatus[PIN][i], OUTPUT );
  }
  pinMode( LED_PIN_CAR, OUTPUT );
  pinMode( LED_PIN_PPL, OUTPUT );
  xSemaphoreGive( mtx_ledStatus );
  
  #ifdef DEBUG_STACK
  bool highWaterMarked = false;
  #endif // DEBUG_STACK

  /* LOOP */
  unsigned long startLoop, endLoop;
  while(true) {
    startLoop = micros();
    if( xSemaphoreTake( mtx_ledStatus, 20 ) == pdTRUE ) {
      for( byte i = 0; i < 4; i++ ) {
        if( i == currentRow ) digitalWrite( ledStatus[PIN][i], HIGH );
        else digitalWrite( ledStatus[PIN][i], LOW );
      }
      digitalWrite( LED_PIN_CAR, ledStatus[CAR][currentRow] == 1 ? LOW : HIGH );
      digitalWrite( LED_PIN_PPL, ledStatus[PPL][currentRow] == 1 ? LOW : HIGH );
      xSemaphoreGive( mtx_ledStatus );
      
      currentRow = (currentRow+1) % 4;
    }


    // Para verificação do uso da stack
    #ifdef DEBUG_STACK
    if( !highWaterMarked ) {
      Serial.print( "LED_Controller high water mark: " + uxTaskGetStackHighWaterMark(NULL) );
      highWaterMarked = true;
    }
    #endif // DEBUG_STACK

    endLoop = micros();
    const unsigned long fullDelay = 1000000/250; // 250 Hz
    delayMicroseconds( fullDelay - (endLoop-startLoop) );
  }
}

void semaphoreStateChanger( void* pvParameters __attribute__((unused)) ) {
  /* SETUP */
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = (1000/4) / portTICK_PERIOD_MS; // 4 Hz
  
  #ifdef DEBUG_STACK
  bool highWaterMarked = false;
  #endif // DEBUG_STACK

  /* LOOP */
  short showingCar = 0;
  short showingPpl = 0;
  while(true) {
    if( xSemaphoreTake( mtx_lightState, 20 ) == pdTRUE ) {
      int currentLightState = lightState[CAR];
      xSemaphoreGive( mtx_lightState );
      switch( currentLightState ) {
        case OFF:
          if( xSemaphoreTake( mtx_ledStatus, 20 ) == pdTRUE ) {
            ledStatus[CAR][GREEN] = 0;
            ledStatus[CAR][YELLW] = 0;
            ledStatus[CAR][RED_1] = 0;
            ledStatus[CAR][RED_2] = 0;
            xSemaphoreGive( mtx_ledStatus );
            showingCar = 0;
          }
          break;
        case GO:
          if( xSemaphoreTake( mtx_ledStatus, 20 ) == pdTRUE ) {
            ledStatus[CAR][GREEN] = 1;
            ledStatus[CAR][YELLW] = 0;
            ledStatus[CAR][RED_1] = 0;
            ledStatus[CAR][RED_2] = 0;
            xSemaphoreGive( mtx_ledStatus );
            showingCar = 0;
          }
          break;
        case ATTENTION:
          if( xSemaphoreTake( mtx_ledStatus, 20 ) == pdTRUE ) {
            ledStatus[CAR][GREEN] = 0;
            ledStatus[CAR][YELLW] = 1;
            ledStatus[CAR][RED_1] = 0;
            ledStatus[CAR][RED_2] = 0;
            xSemaphoreGive( mtx_ledStatus );
            showingCar = 0;
          }
          break;
        case STOP:
          if( xSemaphoreTake( mtx_ledStatus, 20 ) == pdTRUE ) {
            ledStatus[CAR][GREEN] = 0;
            ledStatus[CAR][YELLW] = 0;
            ledStatus[CAR][RED_1] = 1;
            ledStatus[CAR][RED_2] = 1;
            xSemaphoreGive( mtx_ledStatus );
            showingCar = 0;
          }
          break;
        case E_STOP:
          if( xSemaphoreTake( mtx_ledStatus, 20 ) == pdTRUE ) {
            ledStatus[CAR][GREEN] = 0;
            ledStatus[CAR][YELLW] = 0;
            ledStatus[CAR][RED_1] = showingCar <  1 ? 1 : 0; // Blinking
            ledStatus[CAR][RED_2] = showingCar >= 1 ? 1 : 0; // BLinking
            xSemaphoreGive( mtx_ledStatus );
            showingCar = (showingCar+1) % 2;
          }
          break;
        case ERR:
          if( xSemaphoreTake( mtx_ledStatus, 20 ) == pdTRUE ) {
            ledStatus[CAR][GREEN] = 1;
            ledStatus[CAR][YELLW] = 1;
            ledStatus[CAR][RED_1] = 1;
            ledStatus[CAR][RED_2] = 1;
            xSemaphoreGive( mtx_ledStatus );
            showingCar = 0;
          }
          break;
      }
    }
    
    if( xSemaphoreTake( mtx_lightState, 20) == pdTRUE ) {
      int currentLightState = lightState[PPL];
      xSemaphoreGive( mtx_lightState );
      switch( currentLightState ) {
        case OFF:
          if( xSemaphoreTake( mtx_ledStatus, 20 ) == pdTRUE ) {
            ledStatus[PPL][GREEN] = 0;
            ledStatus[PPL][RED_1] = 0;
            ledStatus[PPL][RED_2] = 0;
            xSemaphoreGive( mtx_ledStatus );
            showingPpl = 0;
          }
          break;
        case GO:
          if( xSemaphoreTake( mtx_ledStatus, 20 ) == pdTRUE ) {
            ledStatus[PPL][GREEN] = 1;
            ledStatus[PPL][RED_1] = 0;
            ledStatus[PPL][RED_2] = 0;
            xSemaphoreGive( mtx_ledStatus );
            showingPpl = 0;
          }
          break;
        case ATTENTION:
          if( xSemaphoreTake( mtx_ledStatus, 20 ) == pdTRUE ) {
            ledStatus[PPL][GREEN] = 0;
            ledStatus[PPL][RED_1] = showingPpl < 2 ? 1 : 0; // Blinking
            ledStatus[PPL][RED_2] = showingPpl < 2 ? 1 : 0; // Blinking
            xSemaphoreGive( mtx_ledStatus );
            showingPpl = (showingPpl+1) % 4;
          }
          break;
        case STOP:
          if( xSemaphoreTake( mtx_ledStatus, 20 ) == pdTRUE ) {
            ledStatus[PPL][GREEN] = 0;
            ledStatus[PPL][RED_1] = 1;
            ledStatus[PPL][RED_2] = 1;
            xSemaphoreGive( mtx_ledStatus );
            showingPpl = 0;
          }
          break;
        case E_STOP:
          if( xSemaphoreTake( mtx_ledStatus, 20 ) == pdTRUE ) {
            ledStatus[PPL][GREEN] = 0;
            ledStatus[PPL][RED_1] = showingPpl <  1 ? 1 : 0; // Blinking
            ledStatus[PPL][RED_2] = showingPpl >= 1 ? 1 : 0; // Blinking
            xSemaphoreGive( mtx_ledStatus );
            showingPpl = (showingPpl+1) % 2;
          }
          break;
        case ERR:
          if( xSemaphoreTake( mtx_ledStatus, 20 ) == pdTRUE ) {
            ledStatus[PPL][GREEN] = 1;
            ledStatus[PPL][RED_1] = 1;
            ledStatus[PPL][RED_2] = 1;
            xSemaphoreGive( mtx_ledStatus );
            showingPpl = 0;
          }
          break;
      }
    }


    // Para verificação do uso da stack
    #ifdef DEBUG_STACK
    if( !highWaterMarked ) {
      Serial.print( "semaphoreStateChanger high water mark: " + uxTaskGetStackHighWaterMark(NULL) );
      highWaterMarked = true;
    }
    #endif // DEBUG_STACK
    
    vTaskDelayUntil( &xLastWakeTime, xFrequency );
  }
}

void semaphoreController( void* pvParameters __attribute__((unused)) ) {
  /* SETUP */
  
  #ifdef DEBUG_STACK
  bool highWaterMarked = false;
  #endif // DEBUG_STACK

  while( xSemaphoreTake( mtx_lightState, portMAX_DELAY ) != pdTRUE ) {}
  lightState[CAR] = OFF;
  lightState[PPL] = OFF;
  xSemaphoreGive( mtx_lightState );
  short currentState = OFF;
  short desiredState;

  /* LOOP */
  TickType_t startingTime;
  while(true){
    if( xSemaphoreTake( mtx_semaphoreDesiredState, 20 ) == pdTRUE ) {
      desiredState = semaphoreDesiredState;
      xSemaphoreGive( mtx_semaphoreDesiredState );
      xEventGroupClearBits( eventsGroup, E_CHANGESTATE );
      if( currentState != desiredState ) {
        switch( desiredState ) {
          case CARS_GO:
            if( currentState == PEDESTRIANS_GO ) {
              while( xSemaphoreTake( mtx_lightState, portMAX_DELAY ) != pdTRUE ) {}
              lightState[CAR] = STOP;
              lightState[PPL] = ATTENTION;
              xSemaphoreGive( mtx_lightState );
              startingTime = xTaskGetTickCount();
              vTaskDelayUntil( &startingTime, 5000 / portTICK_PERIOD_MS ); // 5s de delay
            }

            if( currentState != EMERGENCY_GO ) {
              while( xSemaphoreTake( mtx_lightState, portMAX_DELAY ) != pdTRUE ) {}
              lightState[CAR] = STOP;
              lightState[PPL] = STOP;
              xSemaphoreGive( mtx_lightState );
              startingTime = xTaskGetTickCount();
              vTaskDelayUntil( &startingTime, 1000 / portTICK_PERIOD_MS ); // 1s de delay
            }
            
            while( xSemaphoreTake( mtx_lightState, portMAX_DELAY ) != pdTRUE ) {}
            lightState[CAR] = GO;
            lightState[PPL] = STOP;
            xSemaphoreGive( mtx_lightState );
            currentState = CARS_GO;
            break;
          case PEDESTRIANS_GO:
            if( currentState == CARS_GO || currentState == EMERGENCY_GO) {
              while( xSemaphoreTake( mtx_lightState, portMAX_DELAY ) != pdTRUE ) {}
              lightState[CAR] = ATTENTION;
              lightState[PPL] = STOP;
              xSemaphoreGive( mtx_lightState );
              startingTime = xTaskGetTickCount();
              vTaskDelayUntil( &startingTime, 3000 / portTICK_PERIOD_MS ); // 3s de delay
            }
            
            while( xSemaphoreTake( mtx_lightState, portMAX_DELAY ) != pdTRUE ) {}
            lightState[CAR] = STOP;
            lightState[PPL] = STOP;
            xSemaphoreGive( mtx_lightState );
            startingTime = xTaskGetTickCount();
            vTaskDelayUntil( &startingTime, 1000 / portTICK_PERIOD_MS ); // 1s de delay
            
            while( xSemaphoreTake( mtx_lightState, portMAX_DELAY ) != pdTRUE ) {}
            lightState[CAR] = STOP;
            lightState[PPL] = GO;
            xSemaphoreGive( mtx_lightState );
            currentState = PEDESTRIANS_GO;
            break;
          case EMERGENCY_GO:
            if( currentState == PEDESTRIANS_GO ) {
              while( xSemaphoreTake( mtx_lightState, portMAX_DELAY ) != pdTRUE ) {}
              lightState[CAR] = STOP;
              lightState[PPL] = ATTENTION;
              xSemaphoreGive( mtx_lightState );
              startingTime = xTaskGetTickCount();
              vTaskDelayUntil( &startingTime, 4000 / portTICK_PERIOD_MS ); // 4s de delay
            }

            if( currentState != CARS_GO ) {
              while( xSemaphoreTake( mtx_lightState, portMAX_DELAY ) != pdTRUE ) {}
              lightState[CAR] = STOP;
              lightState[PPL] = E_STOP;
              xSemaphoreGive( mtx_lightState );
              startingTime = xTaskGetTickCount();
              vTaskDelayUntil( &startingTime, 500 / portTICK_PERIOD_MS ); // 0.5s de delay
            }
            
            while( xSemaphoreTake( mtx_lightState, portMAX_DELAY ) != pdTRUE ) {}
            lightState[CAR] = GO;
            lightState[PPL] = E_STOP;
            xSemaphoreGive( mtx_lightState );
            currentState = EMERGENCY_GO;
            break;
          case TRAIN_GO:
            if( currentState == PEDESTRIANS_GO ) {
              while( xSemaphoreTake( mtx_lightState, portMAX_DELAY ) != pdTRUE ) {}
              lightState[CAR] = E_STOP;
              lightState[PPL] = ATTENTION;
              xSemaphoreGive( mtx_lightState );
              startingTime = xTaskGetTickCount();
              vTaskDelayUntil( &startingTime, 4000 / portTICK_PERIOD_MS ); // 4s de delay
            } else if( currentState == CARS_GO || currentState == EMERGENCY_GO ) {
              while( xSemaphoreTake( mtx_lightState, portMAX_DELAY ) != pdTRUE ) {}
              lightState[CAR] = ATTENTION;
              lightState[PPL] = E_STOP;
              xSemaphoreGive( mtx_lightState );
              startingTime = xTaskGetTickCount();
              vTaskDelayUntil( &startingTime, 4000 / portTICK_PERIOD_MS ); // 4s de delay
            }
            
            while( xSemaphoreTake( mtx_lightState, portMAX_DELAY ) != pdTRUE ) {}
            lightState[CAR] = E_STOP;
            lightState[PPL] = E_STOP;
            xSemaphoreGive( mtx_lightState );
            currentState = TRAIN_GO;
            break;
          default:
            while( xSemaphoreTake( mtx_lightState, portMAX_DELAY ) != pdTRUE ) {}
            lightState[CAR] = ERR;
            lightState[PPL] = ERR;
            xSemaphoreGive( mtx_lightState );
            currentState = ERR;
            break;
        }
      }
    }
    
    }
    
    // Para verificação do uso da stack
    #ifdef DEBUG_STACK
    if( !highWaterMarked ) {
      Serial.print( "semaphoreStateChanger high water mark: " + uxTaskGetStackHighWaterMark(NULL) );
      highWaterMarked = true;
    }
    #endif // DEBUG_STACK
    
    vTaskDelayUntil( &xLastWakeTime, xFrequency );
  }
}
