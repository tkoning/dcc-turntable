

# dcc-turntable
DCC control of a fleischmann turntable

Digital control of a fleischmann turntable can be achieved with an arduino nano
Turntable is controlled by means of an arduino relay board ( 5 single relays neccessary).
No modification of the original turntable is needed.
The schematic used to connect the turntable to the relay board can be found at
http://usuaris.tinet.cat/fmco/turntable_en.html

DCC input is by means of the standard mynabay optocoupler circuit.
Manual Control is by means of a rotary encoder and an LCD display.

For indexing of the fleischmann turntable a power sensing circuit is used.
A seperate relais can be used to "flip" the polarity of the turntable brige.

code is annotated in dutch.

the turntable can be seen  in action on youtube.

https://www.youtube.com/watch?v=R1SMiSs2mZU

https://www.youtube.com/watch?v=gE1vPq5jKME

https://www.youtube.com/watch?v=S2jX1PM-Gi4


Schematic of the decoder is in the repository.

New software version 6 : position of bridge is retained at arduino shutdown by setting eeprom parameter in code

erratum : relay's must de mounted on the bottomside of the printed circuit board due to reversed polarity of the coil
          increase resitor r6 (dcc optocoupler circuit to 4,7 k)

