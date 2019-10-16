/* 
 *  Program z uzyciem nRF24L01+ pod uC Atmega 328 (Arduino Uno Rev3)
 *  Przesylanie danych z urzadzenia nadrzednego do odbiornika.
 *  Biblioteka: TMRh20/RF24
 *  Urzadzdenie: Master
 *  Autor: Khevenin
 */

/* Libraries */

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

/* Directives and Macros */
#define UART_SPEED_48 4800
#define UART_SPEED_96 9600
#define UART_SPEED_288 28800
#define UART_SPEED_115 115200

#define TX_PIN_LED 6
#define RX_PIN_LED 5

/* Variables */

/* Arrays */
uint8_t TxBuffer[32];
uint8_t RxBuffer[32];

uint8_t TxAddresses[5] = {0x0A, 0x0A, 0x0A, 0x0A, 0x01};
uint8_t RxAddresses[5] = {0x0B, 0x0B, 0x0B, 0x0B, 0x02};

/* Prototypes */

/* Init */
RF24 radio(7, 8);    //Create an RF24 object - set CE - pin 7, CN - pin 8

void setup() {
  /* Serial port init */
  Serial.begin(UART_SPEED_96, SERIAL_8E1);      //UART 8 bits with EVEN mode - że bit parzystości
  Serial.println("\nRemote application start\nUART init OK\n");

  /* Pin init */
  pinMode(TX_PIN_LED, OUTPUT);
  pinMode(RX_PIN_LED, OUTPUT);
  Serial.println("\nTX, RX LED pins OK\n");
  /* add blink function in this place */
  

}

void loop() {
  

}


/* Functions */
