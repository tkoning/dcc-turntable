
/*
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  Arduino DCC Turntable control
  Original DCC sketch : Arduino DCC solenoid Control  Author: Ruud Boer - January 2015
  Adapted for turntable control with LDT protocol Dick Koning feb 2017, echter de nummering van de sporen is conform RocRail

  The DCC signal is optically separated and fed to pin 2 (=Interrupt 0). Schematics: www.mynabay.com
  Many thanks to www.mynabay.com for publishing their DCC monitor and -decoder code.

  Basis idee  geef korte puls op het locking relais, de draaischijf gaat draaien
  als de draaischijf 1 positie gedraait heeft, stopt de stroom door de motor automatisch
  dit genereert een overgang van laag naar hoog op de motor sense pin
  Gebruik dit event om te bepalen hoeveeel afslagen er gedraaid is en genereer eventueel een nieuwe puls

  versie DCC_draai_rocrail3
    :inbouwen DCC stuurelementen, eenvoudige koppeling tussen DCC en draaischijf gemaakt
    :onderstening van commando'ga naar track nr , + 1 track, -1 track, 180 graden draai

  In Rocrail met Xpressnet interface liggen de adressen 4 lager dan de standaard LDT adressen dus basis is 221 ipv 225
  Dit moet worden aangepast met de offset constante
  De Rocrail commando's zijn geinverteerd (instellenin in Rocrail zelf)
  Rocrail genereert 2 commando's een draairichting en een afslagnummer  De draairichting is overbodig en wordt niet gebruikt

  TODO break statement tijdens daai
  TODO calibratie algoritme


  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
*/

#define aan 0               // tbv relais ingangen zijn geinverteerd        
#define uit 1
#define test 1              // Indien (test) = 1 worden de positie medlingen uitgeprint

#define richting1  3        // Arduino Digital I/O pin number  tbv relais sturing
#define richting2  4        // 2 single relais altijd gelijkop bewegen bepalen de draairichting
#define lock       5        // relais wat de interne "lock" van de draaischijf aanstuurt 
#define brug1      6        // ompoolrelais tbv polariteit van de draaibrug
#define brug2      7        // ompoolrelais tbv polariteit draaibrug

#define In_1      8         // Arduino Digital I/O pin number  tbv schakelaars
#define In_2      9
#define In_3      10
#define In_4      11
#define cSense    12        // curent sense motoraandrijving

#include <DCC_Decoder.h>    // Mynabay DCC library
#define kDCC_INTERRUPT 0


// Globale variabelen

int teller = 0;                   // teller is het aantal afslagen dat de scijf moet draaien
int track = 0;                    // actuele afslag waar de draaischijf zich bevindt
int target = 0;                   // doel waarnaar naar toe gedraaid moet worden
bool brugrelais = 0;              // ompoolrelais draaibrug  Normaal=0 ( NC aansluitingen Geinverteerd =1 C aansluitingen

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// FILL IN
const byte maxaccessories = 6;    //The number of "LDT command" adressen you want to control with this Arduino
const byte offset = 4;            // Offset tov originele LDT adressen  Bij gebruik Xpressnet/ multimouse offset = 4 anders offset = 0

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

typedef struct
{
  int               address;          // Address to respond to
  byte              output;           // State of accessory: 1=on, 0=off   rood /groen in wissel commando
  int               track;            // turntable tracknumber
  int               track2;           // turntable track number
  boolean           finished;         // finished : 0=busy 1= ready for next command
  boolean           activate;         // start verwerking commando
  unsigned long     offMilli;         // for internal use
  unsigned long     durationMilli;    // for internal use  "busy periode"

}
DCCAccessoryAddress;
DCCAccessoryAddress accessory[maxaccessories];

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// DCC packet handler
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void BasicAccDecoderPacket_Handler(int address, boolean activate, byte data)
{
  // Convert NMRA packet address format to human address
  address -= 1;
  address *= 4;
  address += 1;
  address += (data & 0x06) >> 1;

  boolean enable = (data & 0x01) ? 1 : 0;

  for (int i = 0; i < maxaccessories; i++)
  {
    if ( address == accessory[i].address )
    {
      accessory[i].activate = 1;
      if ( enable ) {
        accessory[i].output = 1;
      }
      else {
        accessory[i].output = 0;
      }
    }
  }
} // END BasicAccDecoderPacket_Handler


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Initialization: COPY - PASTE the structure as many times as you have commands and fill in the values you want.
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void ConfigureDecoderFunctions() // The amount of accessories must be same as in line 52 above!
{
  accessory[0].address = 226 - offset; // basisadres LDT =225, eerste commando 226     Voor Multimaus + rocrail  adres -4 invoeren
  accessory[0].track = 0;       // output 0 adres
  accessory[0].track2 = 0;      // output 1 adres    LDT : 226 1 draai 180 graden
  accessory[0].finished = 1;    // Do not change this value
  accessory[0].activate = 0;    // Do not change this value
  accessory[0].output = 0;      // Do not change this value
  accessory[0].durationMilli = 250;

  accessory[1].address = 227 - offset;
  accessory[1].track = 0;       // output 0 adres    LDT : 227 0 draai 1 positie CW
  accessory[1].track2 = 0;      // output 1 adres    LDT : 226 1 draai 1 positie CCW
  accessory[1].finished = 1;    // Do not change this value
  accessory[1].activate = 0;    // Do not change this value
  accessory[1].output = 0;      // Do not change this value
  accessory[1].durationMilli = 250;

  accessory[2].address = 229 - offset;
  accessory[2].track = 12;      // output 0 adres    LDT : 229 ga naar spoor 1
  accessory[2].track2 = 5;      // output 1 adres    LDT : 229 ga naar spoor 2
  accessory[2].finished = 1;    // Do not change this value
  accessory[2].activate = 0;    // Do not change this value
  accessory[2].output = 0;      // Do not change this value
  accessory[2].durationMilli = 250;

  accessory[3].address = 230 - offset;
  accessory[3].track = 3;       // output 0 adres    LDT : 230 ga naar spoor 3
  accessory[3].track2 = 0;      // output 1 adres    LDT : 230 ga naar spoor 4
  accessory[3].finished = 1;    // Do not change this value
  accessory[3].activate = 0;    // Do not change this value
  accessory[3].output = 0;      // Do not change this value
  accessory[3].durationMilli = 250;

  accessory[4].address = 231 - offset;
  accessory[4].track = 45;      // output 0 adres    LDT : 231 ga naar spoor 5
  accessory[4].track2 = 24;     // output 1 adres    LDT : 231 ga naar spoor 6
  accessory[4].finished = 1;    // Do not change this value
  accessory[4].activate = 0;    // Do not change this value
  accessory[4].output = 0;      // Do not change this value
  accessory[4].durationMilli = 250;

  accessory[5].address = 225 - offset; // basisadres LDT =225, eerste commando is programeerstand hier gebruikt als reset, toekomst calibratie oid
  accessory[5].track = 0;       // output 0 adres    reset track en target naar 0  moet nog iets in om dat ogv sensor te doen
  accessory[5].track2 = 0;      // output 1 adres    reset track en target naar 0  moet nog iets in om dat ogv sensor te doen
  accessory[5].finished = 1;    // Do not change this value
  accessory[5].activate = 0;    // Do not change this value
  accessory[5].output = 0;      // Do not change this value
  accessory[5].durationMilli = 250;

} // END ConfigureDecoderFunctions



//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void setup()
{
  DCC.SetBasicAccessoryDecoderPacketHandler(BasicAccDecoderPacket_Handler, true);
  ConfigureDecoderFunctions();
  DCC.SetupDecoder( 0x00, 0x00, kDCC_INTERRUPT );


  //-------( Initialize Pins so relays are inactive at reset)----
  digitalWrite(richting1, uit);
  digitalWrite(richting2, uit);
  digitalWrite(lock, uit);
  digitalWrite(brug1, uit);
  digitalWrite(brug2, uit);
  

  //---( THEN set pins as outputs )----
  pinMode(richting1, OUTPUT);
  pinMode(richting2, OUTPUT);
  pinMode(lock, OUTPUT);
  pinMode(brug1, OUTPUT);
  pinMode(brug2, OUTPUT);
  delay(1000); //Check that all relays are inactive at Reset

  pinMode(In_1, INPUT_PULLUP);
  pinMode(In_2, INPUT_PULLUP);
  pinMode(In_3, INPUT_PULLUP);
  pinMode(In_4, INPUT_PULLUP);
  pinMode(cSense, INPUT);
  pinMode(2, INPUT_PULLUP); //Interrupt 0 with internal pull up resistor (can get rid of external 10k)

  pinMode(13, OUTPUT);
  digitalWrite(13, LOW); //turn off Arduino led at startup

  Serial.begin(38400);

  target = 0;
  track = 0;
  teller = 0;

} //--(end setup )---


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Main loop
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void loop()
{
  // target gewenste afslag
  // track actuele afslag
  // teller aantal slagen
  // clockwise(1) : clockwise  clockwise(0) : counter clockwise

  leesDCC();                // genereert een gewenst afslag nummer
  pushButton();             // genereert een gewenst afslag nummer

  if (target > track)
  {
    teller = target - track;
    if (teller <= 24) {
      clockwise(0);
      if (test) melding();
      while (teller > 0) {   // niet definitief gebruiken, blokkeert alles
        draai(0);
      }
    }
    else
    {
      teller = 48 - teller ;
      clockwise(1);
      if (test) melding();
      while (teller > 0) {   // niet definitief gebruiken, blokkeert alles
        draai(1);
      }
    }
  }

  if (target < track)
  {
    teller = track - target;

    if (teller <= 24) {
      clockwise(1);
      if (test) melding();
      while (teller > 0) {   // niet definitief gebruiken, blokkeert alles
        draai(1);
      }
    }
    else
    {
      teller = 48 - teller ;
      clockwise(0);
      if (test) melding();
      while (teller > 0) {   // niet definitief gebruiken, blokkeert alles
        draai(0);
      }
    }
  }
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Aansturing van de draaischijf
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void draai(bool richting)
{
  static bool pulsBusy = false;            // pulsBusy is de variable die aangeeft dat het interne draaischijf locking relais geactiveerd is
  static unsigned long pulsTimer = 0;      // pulsTimer is de teller die meeloopt met de duur van de relais puls
  static unsigned long pulsDuur = 2000;    // pulsDuur geeft aan hoelang de puls default duurt  Moet iets korter zijn dan de tijd die nodig is voor 1 stap

  // teller is het aantal afslagen dat de schijf moet draaien    Globale variable
  // richting is de variabele die aangeeft welke kant de draai opgaat  Procedure Parameter

  if (!pulsBusy) {
    pulsTimer = millis() + pulsDuur;
    pulsBusy = true;
    digitalWrite(lock, aan);
    delay(10);  // probeersel geef het relais even tijd
  }
  else {
    if ( millis() > pulsTimer)
    {
      digitalWrite(lock, uit);  // de default pulsperiode is voorbij

      if (digitalRead(cSense)) {
        //cSense is hoog : er loopt geen stroom door de motor   Hier moet je mischien nog iets van een debounce in zetten
        teller = teller - 1;
        pulsBusy = false;
        if (richting) {        // richting 1=clockwise   0=counterclockwise
          track = track - 1;
          if (track < 0) track = track + 48;
          if (test) melding();
          if (track == 13 || track == 37) {
            brugrelais = !brugrelais; // statement voor ompolen railrelais.  Kies de sporen ogv de layout
            brug(brugrelais);
          }
        }
        else
        {
          track = track + 1;
          if (track >= 48) track = track - 48;
          if (test) melding();
          if (track == 13 || track == 37) {
            brugrelais = !brugrelais; // statement voor ompolen railrelais  Kies de sporen ogv de layout
            brug(brugrelais);
          }
        }
      }
    }
  }

  if (!digitalRead(cSense)) {
    digitalWrite(13, HIGH);
  }
  else {
    digitalWrite(13, LOW);
  }
}

//--(end draairoutine )---



//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// DCCLoop
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void leesDCC()
{
  static int addr = 0;          // Static variable assignment alleen bij startup. Waarde persisteert als de functie verlaten wordt

  DCC.loop(); // Loop DCC library



  if (accessory[addr].finished && accessory[addr].activate)    //event wordt 1 maal getriggerd daarna verplichte pauze
  {
    accessory[addr].finished = 0;
    accessory[addr].offMilli = millis() + accessory[addr].durationMilli;

    if (accessory[addr].output == 1)
    {
      switch (accessory[addr].address) {
        case (225-offset):   //calibratie draaischijf
          track = 0;
          target = 0;
          brugrelais=0;
          break;
        case (226-offset):   // halve draai  LDT 226
          target = track + 24;
          if (target >= 48) target = target - 48;
          break;
        case (227-offset):  // 1 step clockwise LDT 227
          target = track - 1;
          if (target < 0) target = target + 48;
          break;

        default:
          target = (accessory[addr].track2);
          break;
      }
      if (test) {
        Serial.println (accessory[addr].address);
        Serial.print ("Ga naar target track (output = 1 'groen' )   ");
        Serial.println(target);
      }
    }
    else // output=0
    {
      switch (accessory[addr].address) {
        case (225-offset):   // calibratie draaischijf
          target = 0;
          track = 0;
          brugrelais=0;
          break;
        case (226-offset):   // halve draai  LDT 226
          target = track + 24;
          if (target >= 48) target = target - 48;
          break;

        case (227-offset):  // 1 step  counter clockwise
          target = track + 1 ;
          if (target >= 48) target = target - 48;
          break;

        default:
          target = (accessory[addr].track);
          break;
      }
      if (test) {
        Serial.println(accessory[addr].address);
        Serial.print ("Ga naar target track (output = 0 'rood')   ");
        Serial.println(target);
      }
    }
  }

  if ((!accessory[addr].finished) && (millis() > accessory[addr].offMilli))    // blokkade want commando herhaalt zich zelf
  {
    accessory[addr].finished = 1;
    accessory[addr].activate = 0;
  }
  if ( ++addr >= maxaccessories ) addr = 0;   // Bump to next address to activate
}




void clockwise(bool on)
{
  if (on) {
    digitalWrite(richting1, aan);
    digitalWrite(richting2, aan);
    if (test) Serial.println("Clockwise ");
  }
  else {
    digitalWrite(richting1, uit);
    digitalWrite(richting2, uit);
    if (test) Serial.println("Counter Clockwise");
  }
  delay(10);
}

void pushButton()
{
  // invoer dmv druk toetsen  0 = ingedrukt

  if (!digitalRead(In_1)) {
    target = track + 2;
    if (target >= 48) target = target - 48;
  }
  if (!digitalRead(In_2)) {
    target = track + 4;
    if (target >= 48) target = target - 48;
  }
  if (!digitalRead(In_3)) {
    target = track - 2;
    if (target < 0) target = target + 48;
  }
  if (!digitalRead(In_4)) {
    target = track - 4;
    if (target < 0) target = target + 48;
  }
}

void brug(bool on)
{
  if (on) {
    digitalWrite(brug1, aan);
    digitalWrite(brug2, aan);
    if (test) Serial.println("Brug geinverteerd ");
  }
  else {
    digitalWrite(brug1, uit);
    digitalWrite(brug2, uit);
    if (test) Serial.println("Brug normaal");
  }
  delay(10);
}

void melding()
{
  Serial.print("Teller :"); Serial.print(teller); Serial.print("    Target : "); Serial.print(target); Serial.print("    Track : "); Serial.println(track);
}

