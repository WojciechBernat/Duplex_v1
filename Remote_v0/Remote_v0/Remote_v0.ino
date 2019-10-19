/*
    Program z uzyciem nRF24L01+ pod uC Atmega 328 (Arduino Uno Rev3)
    Przesylanie danych z urzadzenia nadrzednego do odbiornika.
    Biblioteka: TMRh20/RF24
    Urzadzdenie: Master
    Autor: Khevenin
*/

/* Libraries */

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

/* Directives and Macros */
#define PIPE_ADDRESS_SIZE  5    //Only 5 byte!
#define BUFFER_SIZE        32   //

#define UART_SPEED_48 4800
#define UART_SPEED_96 9600
#define UART_SPEED_288 28800
#define UART_SPEED_115 115200

#define TX_PIN_LED 6
#define RX_PIN_LED 5

/* Variables */
String TxLedName = "TX LED";
String RxLedName = "RX LED";
String TxBufName = "TxBuffer";
String RxBufName = "RxBuffer";

uint16_t defaultBlinkTime = 0xFF;  // value of blink time - 255ms ;

uint8_t RTxChannel = 64;              // 0 - 124
uint8_t TxPowerLevel = RF24_PA_MIN;   //RF24_PA_MIN - -18dBm, RF24_PA_LOW - -12dBm, RF24_PA_HIGH - -6dBm, RF24_PA_MAX - 0dBm
//RF24_250KBPS for 250kbs, RF24_1MBPS for 1Mbps, or RF24_2MBPS for 2Mbps


/* Arrays */
uint8_t TxBuffer[BUFFER_SIZE];
uint8_t RxBuffer[BUFFER_SIZE];

uint8_t TxAddresses[PIPE_ADDRESS_SIZE] = {0x0A, 0x0A, 0x0A, 0x0A, 0x01};
uint8_t RxAddresses[PIPE_ADDRESS_SIZE] = {0x0B, 0x0B, 0x0B, 0x0B, 0x02};

/* Prototypes */
boolean Blink(uint8_t ledPin = 0, uint16_t blinkTime = defaultBlinkTime);                       //Funkcja z wartosciami domyslnymi
boolean doubleBlink(uint8_t ledPin_1, uint8_t ledPin_2, uint16_t blinkTime = defaultBlinkTime);
void pinsInitPrint(String pinNum, String Name);
void pinsInit( uint8_t LedPin_1, uint8_t LedPin_2, String Name_1, String Name_2 );

void bufferReset(uint8_t *buf, uint8_t bufSize);
void bufferResetPrint( String bufferName);
void cleanBuffers(uint8_t *buf_1, uint8_t bufSize_1, uint8_t *buf_2, uint8_t bufSize_2, String bufferName_1, String bufferName_2 );

void addressPrint(uint8_t *buf, uint8_t bufSize);
void channelPrint(uint8_t channel);
void powerLevelPrint( rf24_pa_dbm_e power );
void dataratePrint( rf24_datarate_e rate );
void radioInit(uint8_t *TxADDR, uint8_t RxADDR, uint8_t channel,  rf24_pa_dbm_e power, rf24_datarate_e rate  );

/* Init */
RF24 radio(7, 8);    //Create an RF24 object - set CE - pin 7, CN - pin 8

void setup() {
  /* Serial port init */
  Serial.begin(UART_SPEED_96, SERIAL_8E1);      //UART 8 bits with EVEN mode - że bit parzystości
  Serial.println("\nRemote application start\nUART init OK\n");

  /* Pin init */
  pinsInit( TX_PIN_LED, RX_PIN_LED, TxLedName, RxLedName );
  /* Buffer reset - clean cells of arrays */
  cleanBuffers(TxBuffer, sizeof(TxBuffer), RxBuffer, sizeof(RxBuffer), TxBufName, RxBufName);

  /* Radio init */
  radio.begin();
  radio.openWritingPipe( TxAddresses );         //TX pipe address
  addressPrint(TxAddresses,PIPE_ADDRESS_SIZE );
  radio.openReadingPipe( 1, RxAddresses );      //RX pipe address
  addressPrint(RxAddresses,PIPE_ADDRESS_SIZE );
  
  radio.setChannel(RTxChannel);                 //Set TX/RX channel
  channelPrint(RTxChannel);

  radio.setPALevel(RF24_PA_MIN) ;              //Set TX output power
  powerLevelPrint(RF24_PA_MIN);

  radio.setDataRate(RF24_250KBPS);            //Set TX speed
  dataratePrint(RF24_250KBPS);


}

void loop() {


}


/* Functions */

boolean Blink(uint8_t ledPin, uint16_t blinkTime) {
  if ((ledPin < 0) || (ledPin > 13)) {
    return false;
  }
  if ( blinkTime > 10000) {   //jezeli wieksze do 10 sek
    blinkTime = 2000;
  }
  if ( digitalRead(ledPin) == HIGH) {
    digitalWrite(ledPin, LOW);
    delay(blinkTime);
    digitalWrite(ledPin, HIGH);
    return true;
  }
  else {
    digitalWrite(ledPin, HIGH);
    delay(blinkTime);
    digitalWrite(ledPin, LOW);
    return true;
  }
}

boolean doubleBlink(uint8_t ledPin_1, uint8_t ledPin_2, uint16_t blinkTime) {
  if ( ((ledPin_1 < 0) || (ledPin_1 > 13)) || ((ledPin_2 < 0) || (ledPin_2 > 13)) ) {
    return false;
  }
  if ( blinkTime > 10000) {   //jezeli wieksze do 10 sek
    blinkTime = 2000;
  }

  if ( digitalRead(ledPin_1) == HIGH || digitalRead(ledPin_2) == HIGH ) {       // high high
    digitalWrite(ledPin_1, LOW);  digitalWrite(ledPin_2, LOW);
    delay(blinkTime);
    digitalWrite(ledPin_1, HIGH); digitalWrite(ledPin_2, HIGH);
    return true;

  } else if ( digitalRead(ledPin_1) == LOW || digitalRead(ledPin_2) == LOW ) {    //low low
    digitalWrite(ledPin_1, HIGH); digitalWrite(ledPin_2, HIGH);
    delay(blinkTime);
    digitalWrite(ledPin_1, LOW);  digitalWrite(ledPin_2, LOW);
    return true;

  } else if ( digitalRead(ledPin_1) == LOW || digitalRead(ledPin_2) == HIGH) {    //low high
    digitalWrite(ledPin_1, HIGH); digitalWrite(ledPin_2, LOW);
    delay(blinkTime);
    digitalWrite(ledPin_1, LOW);  digitalWrite(ledPin_2, HIGH);
    return true;

  } else if ( digitalRead(ledPin_1) == HIGH || digitalRead(ledPin_2) == LOW ) {   //high low
    digitalWrite(ledPin_1, LOW); digitalWrite(ledPin_2, HIGH);
    delay(blinkTime);
    digitalWrite(ledPin_1, HIGH);  digitalWrite(ledPin_2, LOW);
    return true;
  } else {
    return false;
  }
}

void pinsInitPrint(String pinNum, String Name) {
  Serial.println("\nCorrect pin " +  pinNum  + " INIT - " + Name + "\n");
}

void pinsInit( uint8_t LedPin_1, uint8_t LedPin_2, String Name_1, String Name_2 ) {
  pinMode(LedPin_1, OUTPUT);
  pinMode(LedPin_2, OUTPUT);
  pinsInitPrint(String(LedPin_1), Name_1);   //toDebug
  pinsInitPrint(String(LedPin_2), Name_2);   //toDebug
  doubleBlink(LedPin_1, LedPin_2);           //toDebug
}


void bufferReset(uint8_t *buf, uint8_t bufSize) {            //Funkcja resetowanai bufora
  for (uint8_t i = 0; i < bufSize; i++) {
    buf[i] = 0;
  }
}

void bufferResetPrint( String bufferName) {
  Serial.println("\nCorrect " + bufferName + " RESET\n");
}

void cleanBuffers(uint8_t *buf_1, uint8_t bufSize_1, uint8_t *buf_2, uint8_t bufSize_2, String bufferName_1, String bufferName_2 ) {
  bufferReset(buf_1, bufSize_1);
  bufferResetPrint( bufferName_1);
  bufferReset(buf_2, bufSize_2);
  bufferResetPrint( bufferName_2);
}


void addressPrint(uint8_t *buf, uint8_t bufSize) {
  Serial.print("\nSet TxAddress Pipeline: ");
  for(int i = 0; i < bufSize; i++ ) {
    Serial.print("\t " + (String(buf[i])) + "\t" );
  }
}

void channelPrint(uint8_t channel) {
  Serial.println("\nSet RX & TX channel: " + (String(channel)));
}

void powerLevelPrint( rf24_pa_dbm_e power ) {

  if (power == 0) {
    Serial.println("\nSet TX power: -18dBm");
  }
  else if (power == 1) {
    Serial.println("\nSet TX power: -12dBm");
  }
  else if ( power == 2) {
    Serial.println("\nSet TX power: -6dBm");
  }
  else if ( power == 3) {
    Serial.println("\nSet TX power: 0dBm");
  }
}

void dataratePrint( rf24_datarate_e rate ) {
  if (rate == 0) {
    Serial.println("\nSet data rate: 1Mbps");
  }
  else if (rate== 1) {
    Serial.println("\nSet data rate: 2Mbps");
  }
  else if (rate == 2) {
    Serial.println("\nSet data rate: 250kbps");
  }
}



void radioInit(uint8_t *TxADDR, uint8_t RxADDR, uint8_t channel,  rf24_pa_dbm_e power, rf24_datarate_e rate  ) {


}
