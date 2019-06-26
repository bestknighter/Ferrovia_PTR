#include <Arduino_FreeRTOS.h>
#include <Servo.h>

#include <task.h>
#include <semphr.h>
#include <event_groups.h>

#include "pinagem.h"
#include "constantes.h"

//#define DEBUG_STACK

#define CAR 0
#define PPL 1
#define PIN 2
#define GREEN 0
#define YELLW 1
#define RED_1 2
#define RED_2 3

Servo servo;

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
  TURNED_OFF = -1,
  CARS_GO = 0,
  PEDESTRIANS_GO,
  EMERGENCY_GO,
  TRAIN_GO
};
short semaphoreDesiredState;
short semaphoreCurrentState;
short semaphorePreviousNormalState;
SemaphoreHandle_t mtx_semaphoreStates;
EventGroupHandle_t eventsGroup;
void semaphoreController( void* pvParameters );

float trafficWeight;
bool wantToCross, emergencyVehicleIncoming, trainIncoming;
SemaphoreHandle_t mtx_stateVariables;
void sensorMonitor( void* pvParameters );

void brain( void* pvParameters );

void tester( void* pvParameters );

// Criação das tasks e a colocando no scheduler do FreeRTOS
void setup() {
  Serial.begin( 9600 );
  servo.attach( TRAIN_GATE_PIN );
  servo.write( TRN_GTE_CLSE_ANG );
  delay(1000);
  servo.write( TRN_GTE_OPEN_ANG );
  
  ledStatus[PIN][GREEN] = LED_GREEN_PIN;
  ledStatus[PIN][YELLW] = LED_YELLW_PIN;
  ledStatus[PIN][RED_1] = LED_RED_1_PIN;
  ledStatus[PIN][RED_2] = LED_RED_2_PIN;
  mtx_ledStatus = xSemaphoreCreateMutex(); // TODO: Tem que checar se não retornou null...
  mtx_lightState = xSemaphoreCreateMutex(); // TODO: Tem que checar se não retornou null...
  mtx_stateVariables = xSemaphoreCreateMutex(); // TODO: Tem que checar se não retornou null...
  mtx_semaphoreStates = xSemaphoreCreateMutex(); // TODO: Tem que checar se não retornou null...
  eventsGroup = xEventGroupCreate(); // TODO: Tem que checar se não retornou null...

  semaphoreDesiredState = TURNED_OFF;
  
  xTaskCreate( LED_Controller, "LED_Controller", 192 /* STACK SIZE */, NULL, 2, NULL );
  xTaskCreate( semaphoreStateChanger, "semaphoreStateChanger", 192 /* STACK SIZE */, NULL, 2, NULL );
  xTaskCreate( semaphoreController, "semaphoreController", 192 /* STACK SIZE */, NULL, 2, NULL );
  xTaskCreate( sensorMonitor, "sensorMonitor", 192 /* STACK SIZE */, NULL, 2, NULL );
  xTaskCreate( brain, "brain", 192 /* STACK SIZE */, NULL, 2, NULL );
//  xTaskCreate( tester, "tester", 192 /* STACK SIZE */, NULL, 2, NULL );

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
  pinMode( LED_CAR_PIN, OUTPUT );
  pinMode( LED_PPL_PIN, OUTPUT );
  xSemaphoreGive( mtx_ledStatus );
  
  #ifdef DEBUG_STACK
  bool highWaterMarked = false;
  #endif // DEBUG_STACK

  /* LOOP */
  unsigned long startLoop, endLoop;
  while(true) {
    startLoop = millis();
    if( xSemaphoreTake( mtx_ledStatus, 20 ) == pdTRUE ) {
      for( byte i = 0; i < 4; i++ ) {
        if( i == currentRow ) digitalWrite( ledStatus[PIN][i], HIGH );
        else digitalWrite( ledStatus[PIN][i], LOW );
      }
      digitalWrite( LED_CAR_PIN, ledStatus[CAR][currentRow] == 1 ? LOW : HIGH );
      digitalWrite( LED_PPL_PIN, ledStatus[PPL][currentRow] == 1 ? LOW : HIGH );
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

    endLoop = millis();
    const unsigned long fullDelay = 1000/300; // 250 Hz
    delay( fullDelay - (endLoop-startLoop) );
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

  short previousNormalState = CARS_GO;
  short currentState = TURNED_OFF;
  short desiredState;

  /* LOOP */
  TickType_t startingTime;
  while(true){
    xEventGroupWaitBits( eventsGroup, E_CHANGESTATE, pdFALSE, pdFALSE, portMAX_DELAY );
    
    while( xSemaphoreTake( mtx_semaphoreStates, portMAX_DELAY ) != pdTRUE ) {}
    desiredState = semaphoreDesiredState;
    xSemaphoreGive( mtx_semaphoreStates );
    xEventGroupClearBits( eventsGroup, E_CHANGESTATE );
    
    if( currentState != desiredState ) {
      switch( desiredState ) {
        case CARS_GO:
          if( currentState == TRAIN_GO ) {
            servo.write( TRN_GTE_OPEN_ANG );
            vTaskDelayUntil( &startingTime, 1000 / portTICK_PERIOD_MS ); // 1s de delay
          }
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
            if( currentState == TRAIN_GO ) vTaskDelayUntil( &startingTime, 1000 / portTICK_PERIOD_MS ); // 1s de delay
          }
          
          while( xSemaphoreTake( mtx_lightState, portMAX_DELAY ) != pdTRUE ) {}
          lightState[CAR] = GO;
          lightState[PPL] = STOP;
          xSemaphoreGive( mtx_lightState );
          previousNormalState = currentState;
          currentState = CARS_GO;
          break;
        case PEDESTRIANS_GO:
          if( currentState == TRAIN_GO ) {
            servo.write( TRN_GTE_OPEN_ANG );
            vTaskDelayUntil( &startingTime, 1000 / portTICK_PERIOD_MS ); // 1s de delay
          }
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
          previousNormalState = currentState;
          currentState = PEDESTRIANS_GO;
          break;
        case EMERGENCY_GO:
          if( currentState == TRAIN_GO ) {
            servo.write( TRN_GTE_OPEN_ANG );
            vTaskDelayUntil( &startingTime, 1000 / portTICK_PERIOD_MS ); // 1s de delay
          }
          if( currentState == PEDESTRIANS_GO ) {
            while( xSemaphoreTake( mtx_lightState, portMAX_DELAY ) != pdTRUE ) {}
            lightState[CAR] = STOP;
            lightState[PPL] = ATTENTION;
            xSemaphoreGive( mtx_lightState );
            startingTime = xTaskGetTickCount();
            vTaskDelayUntil( &startingTime, 5000 / portTICK_PERIOD_MS ); // 5s de delay
          }

          if( currentState != CARS_GO ) {
            while( xSemaphoreTake( mtx_lightState, portMAX_DELAY ) != pdTRUE ) {}
            lightState[CAR] = STOP;
            lightState[PPL] = E_STOP;
            xSemaphoreGive( mtx_lightState );
            startingTime = xTaskGetTickCount();
            vTaskDelayUntil( &startingTime, 1000 / portTICK_PERIOD_MS ); // 1s de delay
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
            vTaskDelayUntil( &startingTime, 5000 / portTICK_PERIOD_MS ); // 5s de delay
          }
          else if( currentState == CARS_GO || currentState == EMERGENCY_GO ) {
            while( xSemaphoreTake( mtx_lightState, portMAX_DELAY ) != pdTRUE ) {}
            lightState[CAR] = ATTENTION;
            lightState[PPL] = E_STOP;
            xSemaphoreGive( mtx_lightState );
            startingTime = xTaskGetTickCount();
            vTaskDelayUntil( &startingTime, 3000 / portTICK_PERIOD_MS ); // 3s de delay
          }
          servo.write( TRN_GTE_CLSE_ANG );
          while( xSemaphoreTake( mtx_lightState, portMAX_DELAY ) != pdTRUE ) {}
          lightState[CAR] = E_STOP;
          lightState[PPL] = E_STOP;
          xSemaphoreGive( mtx_lightState );
          vTaskDelayUntil( &startingTime, 1000 / portTICK_PERIOD_MS ); // 1s de delay
          currentState = TRAIN_GO;
          break;
        case TURNED_OFF:
          servo.write( TRN_GTE_OPEN_ANG );
          while( xSemaphoreTake( mtx_lightState, portMAX_DELAY ) != pdTRUE ) {}
          lightState[CAR] = OFF;
          lightState[PPL] = OFF;
          xSemaphoreGive( mtx_lightState );
          currentState = TURNED_OFF;
          break;
        default:
          servo.write( TRN_GTE_OPEN_ANG );
          while( xSemaphoreTake( mtx_lightState, portMAX_DELAY ) != pdTRUE ) {}
          lightState[CAR] = ERR;
          lightState[PPL] = ERR;
          xSemaphoreGive( mtx_lightState );
          currentState = ERR;
          break;
      }
      while( xSemaphoreTake( mtx_semaphoreStates, portMAX_DELAY) != pdTRUE ) {}
      semaphoreCurrentState = currentState;
      semaphorePreviousNormalState = previousNormalState;
      xSemaphoreGive( mtx_semaphoreStates );
    }
    
    // Para verificação do uso da stack
    #ifdef DEBUG_STACK
    if( !highWaterMarked ) {
      Serial.print( "semaphoreController high water mark: " + uxTaskGetStackHighWaterMark(NULL) );
      highWaterMarked = true;
    }
    #endif // DEBUG_STACK
  }
}

void sensorMonitor( void* pvParameters __attribute__((unused)) ) {
  /* SETUP */
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = (1000/30) / portTICK_PERIOD_MS; // 30 Hz

  pinMode( CRSWLK_BTN_PIN, INPUT_PULLUP );
  pinMode( EMGINC_SNS_PIN, INPUT );
  pinMode( TRNINC_SNS_PIN, INPUT_PULLUP );
  pinMode( CAR_FLOW_S_PIN, INPUT );
  
  #ifdef DEBUG_STACK
  bool highWaterMarked = false;
  #endif // DEBUG_STACK

  /* LOOP */
  while(true){
    
    while( xSemaphoreTake( mtx_stateVariables, portMAX_DELAY) != pdTRUE ) {}
    if( !wantToCross && !digitalRead( CRSWLK_BTN_PIN ) ){ // Pinagem tem logica invertida: LOW == botao pressionado
      wantToCross = true;
    }
    trafficWeight = analogRead( CAR_FLOW_S_PIN )/1023.0;
    emergencyVehicleIncoming = digitalRead( EMGINC_SNS_PIN );
    trainIncoming = !digitalRead( TRNINC_SNS_PIN );
//    Serial.print( trainIncoming );
//    Serial.print( " " );
//    Serial.print( emergencyVehicleIncoming );
//    Serial.print( " " );
//    Serial.print( trafficWeight );
//    Serial.print( " " );
//    Serial.println( wantToCross );
    xSemaphoreGive( mtx_stateVariables );

    
    // Para verificação do uso da stack
    #ifdef DEBUG_STACK
    if( !highWaterMarked ) {
      Serial.print( "sensorMonitor high water mark: " + uxTaskGetStackHighWaterMark(NULL) );
      highWaterMarked = true;
    }
    #endif // DEBUG_STACK
    
    vTaskDelayUntil( &xLastWakeTime, xFrequency );
  }
}

void brain( void* pvParameters __attribute__((unused)) ) {
  /* SETUP */
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = (1000/10) / portTICK_PERIOD_MS; // 10 Hz

  while( xSemaphoreTake( mtx_semaphoreStates, portMAX_DELAY) != pdTRUE ) {}
  semaphoreDesiredState = CARS_GO;
  xSemaphoreGive( mtx_semaphoreStates );
  xEventGroupSetBits( eventsGroup, E_CHANGESTATE );

  #ifdef DEBUG_STACK
  bool highWaterMarked = false;
  #endif // DEBUG_STACK

  /* LOOP */
  unsigned long timePplOpen = 0;
  unsigned long timeCarOpen = 0;
  unsigned long frameStart;
  unsigned long frameEnd = millis();
  unsigned long dt = 0;
  while(true){
    frameStart = frameEnd;
    frameEnd = millis();
    dt = frameEnd - frameStart;
    if( xSemaphoreTake( mtx_stateVariables, portMAX_DELAY) == pdTRUE ) {
      if( xSemaphoreTake( mtx_semaphoreStates, portMAX_DELAY) == pdTRUE ) {

        if( PEDESTRIANS_GO == semaphoreCurrentState ) {
          timePplOpen += dt;
          wantToCross = false;
        } else if ( CARS_GO == semaphoreCurrentState ) {
          timeCarOpen += dt;
        }
        
        if( trainIncoming ) {
          semaphoreDesiredState = TRAIN_GO;
          timePplOpen = 0;
          timeCarOpen = 0;
        } else if( emergencyVehicleIncoming ) {
          semaphoreDesiredState = EMERGENCY_GO;
          timePplOpen = 0;
          timeCarOpen = 0;
        } else if( semaphoreCurrentState == TRAIN_GO || semaphoreCurrentState == EMERGENCY_GO ) {
          semaphoreDesiredState = semaphorePreviousNormalState;
          timePplOpen = 0;
          timeCarOpen = 0;
        } else if( wantToCross ) {
          if( timeCarOpen > (1-trafficWeight)*MIN_PPL_WAITTIME + trafficWeight*MAX_PPL_WAITTIME ) semaphoreDesiredState = PEDESTRIANS_GO;
        } else if( timePplOpen > PPL_CROSS_TIME ) {
          semaphoreDesiredState = CARS_GO;
        }

        if( semaphoreCurrentState != semaphoreDesiredState ) xEventGroupSetBits( eventsGroup, E_CHANGESTATE );
        xSemaphoreGive( mtx_semaphoreStates );
      }
      xSemaphoreGive( mtx_stateVariables );
    }
    
    
    // Para verificação do uso da stack
    #ifdef DEBUG_STACK
    if( !highWaterMarked ) {
      Serial.print( "sensorMonitor high water mark: " + uxTaskGetStackHighWaterMark(NULL) );
      highWaterMarked = true;
    }
    #endif // DEBUG_STACK
    
    vTaskDelayUntil( &xLastWakeTime, xFrequency );
  }
}

void tester( void* pvParameters __attribute__((unused)) ) {
  TickType_t lastAwakeTime = xTaskGetTickCount();
  while(true) {
    int mode = Serial.read() - '0';
    if( mode >= -1 ){
      while( xSemaphoreTake( mtx_semaphoreStates, portMAX_DELAY) != pdTRUE ) {}
      semaphoreDesiredState = mode;
      xSemaphoreGive( mtx_semaphoreStates );
      xEventGroupSetBits( eventsGroup, E_CHANGESTATE );
    }
    vTaskDelayUntil( &lastAwakeTime, 500 / portTICK_PERIOD_MS ); // 0.5s de delay
  }
}
