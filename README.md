# vBus2IP
One-way vBus to json over IP using an Arduino Ethernet

### Why
Because I wanted to see what my solar collector is doing when I'm away, or am simply too lazy to walk up to where the controller is mounted.

I did it in json in order to make it easy to put it together with some bootstrap or something and make a nice status page.

### Hardware
- [Arduino Ethernet](https://www.arduino.cc/en/Main/ArduinoBoardEthernet), or Arduino with Ethernet shield, or compatible
- vBus2TTL(https://circuits.io/circuits/2085475-vbus-to-ttl-converter) or similar. See comments in [Autodesk Circuits](https://circuits.io/)

I am using Arduino Ethernet with PoE. It has more than enough power for the meager needs of the vBus to TTL level converter; that is the reason I left a 3-pin header in the circuit design. Pin 2 goes to the rx of the serial port of the Arduino, and the other 2 to +5V and GND.

Note that the rx of the serial port is pin 0 on the Arduino Ethernet. This means that you need to disconnect the vBus2TTL pin from pin 0 when uploading the sketch; otherwise you won't be able to do it. If you use a Mega, it has more serial ports and you can use a different one, for example Serial1.

### Software
This sketch, vBusIP.ino.

I have tested it with a Roth BW/H Komfort, which is a rebranded Resol Deltasol M.

This is based off https://github.com/bbqkees/vbus-arduino-domoticz which provided the basis for this sketch, as well as a great deal of hints.

Also take a look at the library version at https://github.com/FatBeard/vbus-arduino-library

### Documentation
I got all the information I needed from the document **RESOL VBus Protocol Specification** dated 11.10.2007, which I found somewhere (can't recall exactly where). The document came in a PDF file titled **VBus_Protokol_en_20071218.pdf**.

Many a source and the circuit schematic came from [Piamble blog](https://piamble.wordpress.com/2014/06/17/home-energy-centre-using-raspberry-pi-and-nook-simple-touch/)

Other sources are hinted at in the sketch.

That is all the info I have for now. Feel free to ping me if you believe I could clarify matters here and there.

### Testing
Not very exciting, to be honest. curl is your friend:
```
curl http://10.0.0.1:8080/Estado
```
Replace 10.0.0.1 with the IP address you have given to your Arduino, and if everything is connected you're good to go.

### Contributing
Contributors are welcome. The code needs some cleanup and perhaps it could be expanded to understand other vBus speaking controllers.
