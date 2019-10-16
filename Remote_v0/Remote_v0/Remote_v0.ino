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


/* Variables */

/* Arrays */

/* Prototypes */

/* Init */
RF24 radio(7, 8 );    //Create an RF24 object - set CE - pin 7, CN - pin 8

void setup() {
  

}

void loop() {
  

}
