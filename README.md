

# dcc-turntable
DCC control of a fleischmann turntable

Digital control of a fleischmann turntable can be achieved with an arduino nano

No modification of the original turntable is needed.
The turntable is controlled by four relays, 

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

erratum : relay's must de mounted on the bottomside of the printed circuit board due to a design mistake regarding polarity of the relay coil.  Increase resitor r6 (dcc optocoupler circuit to 4,7 k)

I have noticed the fleischmann turntable electric engine to be unreliable. Especialy when cold the engine can cause short circuits
A new software version (vs10) cuts the power after a 3 second period
For this to work the positive power line has to be connected via the spare connecors of the aux1 relais
