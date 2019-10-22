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

boolean         ReceiveState = false;            //state of received bytes
boolean         ACKEnable    = true;
uint8_t         RTxChannel   = 125;              // 0 - 124
rf24_pa_dbm_e   TxPowerLevel = RF24_PA_MIN;      //RF24_PA_MIN - -18dBm, RF24_PA_LOW - -12dBm, RF24_PA_HIGH - -6dBm, RF24_PA_MAX - 0dBm
rf24_datarate_e TxDataRate = RF24_250KBPS;       //RF24_250KBPS for 250kbs, RF24_1MBPS for 1Mbps, or RF24_2MBPS for 2Mbps

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

void bufferPrint(uint8_t *buf, uint8_t bufSize);
void setBufferPrint(uint8_t *buf, uint8_t bufSize, String bufferName);
void DataPrint(uint8_t *buf, uint8_t bufSize, String bufferName);

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
  radio.openWritingPipe(TxAddresses);     //TX pipeline address
  radio.openReadingPipe(1,RxAddresses);   //RX pipeline address
  setBufferPrint(TxAddresses, BUFFER_SIZE,TxBufName);
  setBufferPrint(RxAddresses, BUFFER_SIZE,RxBufName);
  
  radio.setPALevel(RF24_PA_MIN);          //PA power level
  Serial.println("\n PA level " + (String(radio.getPALevel())));
  
  radio.setDataRate(TxDataRate);          //Data Rate
  Serial.println("\n Data rate " + (String(radio.getDataRate())));
  
  radio.setChannel(RTxChannel);            //Channel 
  Serial.println("\n Channel" + (String(radio.getChannel())));
  
  radio.setAutoAck(ACKEnable);                    //ACK
  Serial.println("Radio INIT done");
  
}

void loop() {
  /* ADC A0 measurement and Data proccessing */
   ADCmeas[posX] = map(analogRead(pinX), 0, 1023, 0, 255);    //measure value of voltage on joystic for X axis
   ADCmeas[posY] = map(analogRead(pinY), 0, 1023, 0, 255);    // - || - for Y axis
   ADCmeas[posS] = map(analogRead(pinS), 0, 1023, 0, 255);    // - || - for switch state

   /* Copy ADCmeas on TxBuffer */
   for(uint8_t j = 0; j < sizeof(ADCmeas) ; j++) {
     TxBuffer[j] = ADCmeas[j];
   }
   bufferReset(ADCmeas, sizeof(ADCmeas));
   
  /* Transmit */
  radio.stopListening();
  delay(1000);
   /* Start sending */
  digitalWrite(TX_PIN_LED, HIGH);     //TX led set
  if(radio.write(TxBuffer, BUFFER_SIZE))  {
      DataPrint(TxBuffer, BUFFER_SIZE, TxBufName);
  }
  digitalWrite(TX_PIN_LED, LOW);
  
  /* Receive */
  radio.startListening();
 if(radio.available()) {
    digitalWrite(RX_PIN_LED, HIGH);
    while(radio.available()) {
     radio.read(RxBuffer, sizeof(RxBuffer));
    }
    digitalWrite(RX_PIN_LED, LOW);
  }
}
/* End of loop() */


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

/* Clean arrays functions */
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

/* Print content arrays functions */
void bufferPrint(uint8_t *buf, uint8_t bufSize) {
    for(int i = 0; i < bufSize; i++ ) {
    Serial.print("\t " + (String(buf[i])) + " " );
  }
}

void setBufferPrint(uint8_t *buf, uint8_t bufSize, String bufferName) {
  Serial.print("\nSet " + bufferName + " Pipeline: ");
  bufferPrint(buf, bufSize);
}


void DataPrint(uint8_t *buf, uint8_t bufSize, String bufferName) {
  if(bufferName == "TxBuffer")  {
    Serial.print("\nTransmited bytes from TX Buffer: ");
    bufferPrint(buf, bufSize);
  }
  if(bufferName == "RxBuffer")  {
    Serial.print("\nReceived bytes from RX Buffer: ");
    bufferPrint(buf, bufSize);
  }
}
