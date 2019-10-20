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
#define MEAS_SIZE          3    //number of cells in ADCmeas

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
String ADCBufName = "ADCmeas";

uint16_t defaultBlinkTime = 0xFF;  // value of blink time - 255ms ;

uint8_t         RTxChannel = 64;              // 0 - 124
rf24_pa_dbm_e   TxPowerLevel = RF24_PA_MIN;   //RF24_PA_MIN - -18dBm, RF24_PA_LOW - -12dBm, RF24_PA_HIGH - -6dBm, RF24_PA_MAX - 0dBm
rf24_datarate_e TxDataRate = RF24_250KBPS;     //RF24_250KBPS for 250kbs, RF24_1MBPS for 1Mbps, or RF24_2MBPS for 2Mbps

uint8_t pinX = 0;   //number of pin to measure X axis 
uint8_t pinY = 1;   // - || - Y axis
uint8_t pinS = 2;   // - || - switch state
uint8_t posX = 0;   //X axis value in ADCmeas array
uint8_t posY = 1;   //Y - || -
uint8_t posS = 2;   //switch state - || - 

/* Arrays */
uint8_t ADCmeas[MEAS_SIZE] = {0x00, 0x00};

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

void TxAddressPrint(uint8_t *buf, uint8_t bufSize);   //added after merge
void RxAddressPrint(uint8_t *buf, uint8_t bufSize);
void TransmitedDataPrint(uint8_t *buf, uint8_t bufSize);
void ReceivedDataPrint(uint8_t *buf, uint8_t bufSize);

//void addressPrint(uint8_t *buf, uint8_t bufSize);
void channelPrint(uint8_t channel);
void powerLevelPrint( rf24_pa_dbm_e power );
void dataratePrint( rf24_datarate_e rate );
void radioInit(uint8_t *TxADDR, uint8_t *RxADDR, uint8_t addrSize, uint8_t channel, rf24_pa_dbm_e power, rf24_datarate_e rate );  //zastanowic sie nad inicjalziacja

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
//  radioInit(TxAddresses, RxAddresses, PIPE_ADDRESS_SIZE, RTxChannel, TxPowerLevel, TxDataRate );
  radio.begin();
  radio.openWritingPipe(TxAddresses);   //added to debug
//  radio.openReadingPipe(1,RxAddresses);
  radio.setPALevel(RF24_PA_MIN);
  Serial.println("Radio INIT done");
  
}

void loop() {
  delay(500);
  /* ADC A0 measurement and Data proccessing */
   ADCmeas[posX] = map(analogRead(pinX), 0, 1023, 0, 255);    //measure value of voltage on joystic for X axis
   ADCmeas[posY] = map(analogRead(pinY), 0, 1023, 0, 255);    // - || - for Y axis
   ADCmeas[posS] = map(analogRead(pinS), 0, 1023, 0, 255);    // - || - for switch state
   Serial.println("\nMeasured joystic position: \nX axis: " + (String(ADCmeas[posX])) + "\nY axis: " + (String(ADCmeas[posY])) + "\nSwitch: " + (String(ADCmeas[posS])));
   Serial.println("\nTX Buffer content: \n");
   for(uint8_t j = 0; j < sizeof(ADCmeas) ; j++) {
     TxBuffer[j] = ADCmeas[j];
     Serial.println("\t" + (String(TxBuffer[j])) );
   }
   bufferReset(ADCmeas, sizeof(ADCmeas));
   bufferResetPrint(ADCBufName);
   
  /* Transmit */
  radio.stopListening();
  delay(100);
   /* Start sending */
  digitalWrite(TX_PIN_LED, HIGH);     //TX led set
  boolean state = radio.write(TxBuffer, BUFFER_SIZE);
  Serial.println("\n---------" + String(state) + "-------\n");
  if(radio.write(TxBuffer, BUFFER_SIZE))  {
      TransmitedDataPrint(TxBuffer,BUFFER_SIZE);
  }

  digitalWrite(TX_PIN_LED, LOW);
  
  /* Receive */
//  radio.startListening();
//  if(radio.available()) {
//    digitalWrite(RX_PIN_LED, HIGH);
//    while(radio.available()) {
//      radio.read(RxBuffer, sizeof(RxBuffer));
//    }
//    digitalWrite(RX_PIN_LED, LOW);
//  }
  

  
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


void TxAddressPrint(uint8_t *buf, uint8_t bufSize) {
  Serial.print("\nSet TxAddress Pipeline: ");
  for(int i = 0; i < bufSize; i++ ) {
    Serial.print("\t " + (String(buf[i])) + " " );
  }
}

void RxAddressPrint(uint8_t *buf, uint8_t bufSize) {
  Serial.print("\nSet RxAddress Pipeline: ");
  for(int i = 0; i < bufSize; i++ ) {
    Serial.print("\t " + (String(buf[i])) + " " );
  }
}

void TransmitedDataPrint(uint8_t *buf, uint8_t bufSize) {
  Serial.print("\nTransmited bytes from TX Buffer: ");
  for(int i = 0; i < bufSize; i++ ) {
    Serial.print("\t " + (String(buf[i])) + " " );
  }
}

void ReceivedDataPrint(uint8_t *buf, uint8_t bufSize) {
  Serial.print("\nReceived bytes from RX Buffer: ");
  for(int i = 0; i < bufSize; i++ ) {
    Serial.print("\t " + (String(buf[i])) + " " );
  }
}



//void addressPrint(uint8_t *buf, uint8_t bufSize) {
//  Serial.print("\nSet TxAddress Pipeline: ");
//  for(int i = 0; i < bufSize; i++ ) {
//    Serial.print("\t " + (String(buf[i])) + "\t" );
//  }
//}

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


void radioInit(uint8_t *TxADDR, uint8_t *RxADDR, uint8_t addrSize, uint8_t channel,  rf24_pa_dbm_e power, rf24_datarate_e rate  ) {
  radio.begin();
  radio.openWritingPipe( TxADDR);         //TX pipe address
  TxAddressPrint(TxADDR, addrSize );
  radio.openReadingPipe( 1, RxADDR);      //RX pipe address
  RxAddressPrint(RxADDR,addrSize);
  
//  radio.setChannel(channel);                 //Set TX/RX channel
//  channelPrint(channel);
//
//  radio.setPALevel(power) ;              //Set TX output power
//  powerLevelPrint(power);
//
//  radio.setDataRate(rate);            //Set TX speed
//  dataratePrint(rate);
}
