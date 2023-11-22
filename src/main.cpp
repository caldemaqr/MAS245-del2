#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Arduino.h>
#include <FlexCAN_T4.h>
#include <SPI.h>
#include <Wire.h>
#include <string.h>




//Kommentarer er lagt til i scriptet.
//En del kodelinjer er kommentert ut da samme script er blitt brukt i flere oppgaver


//namespace carrier og oled er skrivd av foreleser. Dette definerner pin-porter og parametere for skjermen
namespace carrier
{
  namespace pin
  {
    constexpr uint8_t oledDcPower{6};
    constexpr uint8_t oledCs{10};
    constexpr uint8_t oledReset{5};
  }

  namespace oled
  {
    constexpr uint8_t screenWidth{128}; // OLED display width in pixels
    constexpr uint8_t screenHeight{64}; // OLED display height in pixels
  }
}


//Også skrevet av foreleser. Import av flexCAN biblo, bruker kun Can0.
namespace {
  CAN_message_t msg;

  FlexCAN_T4<CAN0, RX_SIZE_256, TX_SIZE_16> can0;
  

  Adafruit_SSD1306 display( carrier::oled::screenWidth,
                            carrier::oled::screenHeight,
                            &SPI,
                            carrier::pin::oledDcPower,
                            carrier::pin::oledReset,
                            carrier::pin::oledCs);
}



//Av foreleser. Bygger sendcan
struct Message {
    uint8_t sequenceNumber;
    float temperature;
};

void sendCan();
void sendCan(const Message& message);

//void setup er lagd av faglærer
void setup() {
  Serial.begin(9600);
  can0.begin();
  //Setter frekvens til 250kHz
  can0.setBaudRate(250000);
 
   if( !display.begin(SSD1306_SWITCHCAPVCC) )
  {
    Serial.println(F("ERROR: display.begin(SSD1306_SWITCHCAPVCC) failed."));
  }


  display.clearDisplay();
  display.display();
  delay(200);
}

 // Lager en void for å midtstille teksen og hente ut plassering

 void displayCenteredText(Adafruit_SSD1306 &display, const String &text, int y) {
  int16_t x1, y1;
  uint16_t w, h;

  display.getTextBounds(text, 0, 0, &x1, &y1, &w, &h);
  int16_t x = (display.width() - w) / 2;
  display.setCursor(x, y);
  display.print(text);
} 

 #define imu  0.00322580645 //konverteringskonstant for IMU 
 

 void sendCan()
 {
  msg.len = 1;
  msg.id = 0x021;
  msg.buf[0] = (((imu*analogRead(A7)-1.65)/0.36)*98.1); //Konverterer imu-målinger til m/s^2
  can0.write(msg);
 }


 //motatt og siste id brukes for å telle antall meldinger motatt, og på siste ID melding kom på.
 int mottatt = 0;
 uint32_t sisteID = 0;

 int number = analogRead(A7); //Lagrer IMU-målingane til en integer
 int digit1, digit2, digit3, digit4;

  void sendCanIMU(){

   msg.len = 3;
   msg.id = 0x022;
   msg.buf[0] = digit2;
   msg.buf[1] = digit3;
   msg.buf[2] = digit4;
   can0.write(msg);
 }

 void loop() 
 {

  display.clearDisplay();
   
   //Denne første delen er brukt i oppgave 4. 

   //Printe IMU-målinger til terminal
   Serial.print(((imu*analogRead(A7)-1.65)/0.36)*9.81);
   Serial.print(" m/s^2  ");
   Serial.print(analogRead(7));
   Serial.println(" IMU: ");

    int number = analogRead(A7);
    digit4 = number % 10;     
    number = number / 10;
    digit3 = number % 10;       
    number = number / 10;
    digit2 = number % 10;       
    number = number / 10;
   

   //Det er lagt inn 2 delay på 500 ms, dette tilsvarer en fast rate på 1Hz. sikkert litt tungvind måte å gjøre det på
   sendCanIMU();
   delay(500);
   
   Serial.print(digit2);
   Serial.print(" ");
   Serial.print(digit3);
   Serial.print(" ");
   Serial.print(digit4);
   Serial.print(" ");
   delay(500);

   
   //Lager can message som frame. Tidligere gjort som msg
   CAN_message_t frame;
   
   if (can0.read(frame)) {
    if(frame.id == 0x246){
    
   
    //printer i terminalen
    Serial.print("ID: 0x");
    Serial.print(frame.id, HEX);
    Serial.print("  Data Length: ");
    Serial.print(frame.len);
    Serial.print("  Data: ");
    for (int i = 0; i < frame.len; i++) {
      Serial.print(frame.buf[i], HEX);
      Serial.print(" ");
    }
    Serial.println();
     
    //Teller en opp på antall meldinger motatt. 
    mottatt++;
    //Lagrer ID i egen variabel for å kunne vise på display
    sisteID = frame.id;

    }delay(100);

  }

  //Initialiserer tekststørrelse, farge og så clearer skjermen.
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.clearDisplay(); 
  
 
  String textLine1 = "MAS 245 - Gruppe 7";
  String textLine2 = "CAN-statistikk";
  String textLine3 = "Antall mottatt: " + String(mottatt);
  String textLine4 = "Mottok sist ID:" + String(sisteID, HEX);
  String textLine5 = "IMU z: " + String(((imu*analogRead(A7)-1.65)/0.36)*9.81) + String(" m/s^2");
  

  //Can-busmeldinga blir brukt i oppgave 2 og 3
  //sendCan();

  
  //Lager ramme på display av avrunda hjørner.
  int cornerRadius = 8;
  display.drawRoundRect(0, 0, display.width(), display.height(), cornerRadius, SSD1306_WHITE);

  // Senterer første tekstlinje og tegner bue under med bruk av en for-løkke.
  displayCenteredText(display, textLine1, 4);    
  for (int16_t x = 0; x < 128; ++x)
  {
    display.drawPixel(x, 14 + (int16_t)(4*std::sin((PI*x)/128)), 1);
  }


  // Textline 3 og 4 er det ikke plass til i oppgave 4. Derfor kommenteres de ut.
  // Textline 3 og 4 blei brukt i oppgave 2 og 3
  
  display.setCursor(2,20);
  display.print(textLine2);
  display.setCursor(2, 29);
  display.println("--------------"); 
  display.setCursor(2, 45);    
  //display.println(textLine3);  
  //display.setCursor(2, 35);
  //display.println(textLine4);
  display.println(textLine5);
  display.setCursor(2, 55);
  display.println("--------------"); 
  display.display(); 

  int buffer0 = 0;
  int buffer1 = 0;
  int buffer2 = 0;
  int imu1 = 0;
  float imu2 = 0;


  // Hvis canbus-melding kommer på id 22 kjøres denne if-statementen.
  if(frame.id == 0x022){
   
   // Lagrer meldingene som egne variabler
    buffer0 = frame.buf[0];
    buffer1 = frame.buf[1];
    buffer2 = frame.buf[2];
   
   //Samler rådata i en variabler
   imu1 = buffer0 *100 + buffer1 *10 + buffer2;

   //Konverterer rådata til m/s^2
   imu2 = (((imu*imu1-1.65)/0.36)*9.81);


   //Printer til terminal for å dobbeltsjekke at konvertering er riktig
   Serial.println( imu2 );
   Serial.print("  ");

   //Sender konverterte IMU-målinger på ny ID (23)
   frame.buf[0] = imu2;
   frame.id = 0x023;

  can0.write(frame);
  };


 // Oppdaterer display
  display.display();
}