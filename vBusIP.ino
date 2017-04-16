
#include <SPI.h>
#include <Ethernet.h>
#include <avr/wdt.h>

// #define DEPURAR 1
// #define DEPURARMUCHO 1

// 18 y 19 son Serial1, para vBus (en plataforma Mega)
// 0 y 1 en Arduino Uno, Ethernet, etc

// Cosas de leer vBus
const unsigned long timerInterval = 2000;
const unsigned long longitudTampon = 120;
// Settings for the VBus decoding
// #define Sync  0xAA  // Synchronisation bytes
// #define ResolAddress 0x3271  //   ConergyDT5 (0x3271) Original default Address of the controller
const int SENSORNOTCONNECTED = 8888; // Sometimes this might be 888 instead.

float irradiacion = -666;
float temperaturas [] = { -666, -666, -666, -666, -666, -666, -666, -666 };
short int reles[] = { -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 };
const unsigned short int ultimaTemperatura = sizeof(temperaturas) / sizeof(temperaturas[0]) - 1;
const unsigned short int ultimoRele = sizeof(reles) / sizeof(reles[0]) - 1;

// Preparamos Ethernet:
byte MAC[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
byte IP[] = { 10, 0, 0, 0 };
byte PASARELA[] = { 10, 0, 0, 1 };
byte MASCARA[] = { 255, 255, 255, 0 };
byte DNS[] = { 10, 0, 0, 1 };

EthernetServer servidor = EthernetServer(8080);

void setup() {
  delay( 50 );   // Dar 50 ms al circuito de arranque del Wiznet W5100
  Serial.begin(9600);  // vBus

  Ethernet.begin(MAC, IP, DNS, PASARELA, MASCARA);
  servidor.begin();
#ifdef DEPURAR
  Serial.print("Servidor corriendo en ");
  Serial.println(Ethernet.localIP());
#endif
  wdt_enable(WDTO_8S);
}

void loop() {
  wdt_reset();     // Seguimos vivos
  leerVBus();
  Servidor();      // Ver si nos han mandado algun comando y ejecutarlo
}

void Servidor() {
  EthernetClient cliente = servidor.available();
  if (cliente) {
#ifdef DEPURAR
    Serial.print(millis());
    Serial.println(" Tenemos cliente HTTP");
#endif
    // an http request ends with a blank line
    boolean lineaEnBlanco = true;
    char linea[65] = "";
    char *cursorLinea = linea;
    int  cursor = 0;
    const char delimitadores[] = " ?=&";


    while (cliente.connected()) {
      if (cliente.available()) {
        char c = cliente.read();
#ifdef DEPURAR
        //        Serial.write(c);
#endif
        if (cursor < 63) {
          linea[cursor] = c;
          cursor++;
          linea[cursor] = 0;
        } else { // demasiados caracteres
          cliente.stop();
        }
        if (c == '\n' && lineaEnBlanco) { // Hemos recibido dos \n, asi que ya podemos ver que tenemos
#ifdef DEPURAR
          Serial.print("Tenemos linea: \"");
          Serial.print(linea);
          Serial.println("\"");
#endif
          char *elemento;
          elemento = strsep(&cursorLinea, delimitadores);
          /*          if ( elemento == NULL ) {
                      cliente.stop();
                      return;
                    }
          */
#ifdef DEPURAR
          Serial.print("Tenemos un primer elemento: \"");
          Serial.print(elemento);
          Serial.println("\"");
#endif

          if (strcmp(elemento, "GET") == 0) { // Vamos bien, tenemos un comando GET; podemos empezar a responder
            cliente.println("HTTP/1.1 200 OK");
            cliente.println("Content-Type: application/json");
            cliente.println("cache-control: private, max-age=0, no-cache");
            cliente.println("Access-Control-Allow-Origin: *");
            cliente.println("Connection: close");  // the connection will be closed after completion of the response
            cliente.println();
            elemento = strsep(&cursorLinea, delimitadores);
#ifdef DEPURAR
            Serial.print("Tenemos un segundo elemento: \"");
            Serial.print(elemento);
            Serial.println("\"");
#endif
            if (strcmp(elemento, "/Estado") == 0) {
#ifdef DEPURAR
              Serial.print(millis());
              Serial.println(" Ejecutando /Estado");
#endif
              cliente.print("{\"status\": \"success\", \"irradiacion\": \"");
              cliente.print(irradiacion);
              cliente.print("\",\"temperaturas\": [\"");
              for (unsigned short i = 0; i <= ultimaTemperatura; i++) {
                cliente.print(temperaturas[i]);
                cliente.print("\",\"");
              }
              cliente.print("\"], \"reles\": [\"");
              for (unsigned short i = 0; i <= ultimoRele; i++) {
                cliente.print(reles[i]);
                cliente.print("\",\"");
              }
              cliente.println("\"] }");
            } else { // Comando o URL no reconocidos
              cliente.println("{\"status\": \"fail\"}");
            }
            cliente.stop();
            if (c == '\n') {
              // you're starting a new line
              lineaEnBlanco = true;
            } else if (c != '\r') {
              // you've gotten a character on the current line
              lineaEnBlanco = false;
            }
          }
          // free(trocitosDeLinea);
        }
      }
    }
  }
}


float CalcTemp(int Byte1, int Byte2) {
  int v;

  v = Byte1 << 8 | Byte2; //bit shift 8 to left, bitwise OR

  if (Byte1 == 0x00)
    v = v & 0xFF;
  else if (Byte1 == 0xFF)
    v = v - 0x10000;

  if (v == SENSORNOTCONNECTED)
    v = 0;

  return (float)((float) v * 0.1);
}

void  InjectSeptet(unsigned char *Buffer, int Offset, unsigned char Septet, int Length) {
  for (unsigned int i = 0; i < Length; i++) {
    if (Septet & (1 << i)) {
      Buffer [Offset + i] |= 0x80;
    }
  }
}

boolean leerVBus () {
  const char sync1 = 0xAA;
  const int FLength = 6; // Framelength
  const int FOffset = 10; // Offset start of Frames
  const int FSeptet = 4; // Septet byte in Frame
  int F;
  char c;
  bool start = true;
  bool stop = false;
  bool timeout = false;
  unsigned char Buffer[longitudTampon];
  volatile unsigned char Bufferlength = 0;
  static unsigned long ultimaLecturaVBus = millis();
  unsigned int Destination_address;
  unsigned int Source_address;
  unsigned char ProtocolVersion;
  unsigned int Command;
  unsigned char Framecnt;
  unsigned char Septet;
  unsigned char Checksum;
  char PumpSpeed1, PumpSpeed2;  //  in %
  char Relay1, Relay2;  //  in %
  char RelaisMask, ErrorMask;
  char Scheme;
  char OptionPostPulse, OptionThermostat, OptionHQM;
  char MixerOpen, MixerClosed; // in  %
  char SystemNotification;

  uint16_t SystemTime;
  uint16_t OperatingHoursRelais1, OperatingHoursRelais2;
  uint32_t HeatQuantity;
  uint16_t Version;


  while ((!stop) and (!timeout))  {
    if (Serial.available()) {
      c = Serial.read();
      if (c == sync1) {
#ifdef DEPURAR
        Serial.println("Sincronizado");
#endif
        if (start) {
          start = false;
          Bufferlength = 0;
        } else {
          if (Bufferlength < 20) {
            ultimaLecturaVBus =  millis();
            Bufferlength = 0;
          } else
            stop = true;
        }
      }
#ifdef DEPURARMUCHO
      Serial.println(c, HEX);
#endif
      if ((!start) and (!stop)) {
        Buffer[Bufferlength] = c;
        Bufferlength++;
        ultimaLecturaVBus =  millis();
      }
    }
    timeout = millis() - ultimaLecturaVBus > timerInterval;
  }

  ultimaLecturaVBus = 0;

  if (!timeout) {
    Destination_address = Buffer[2] << 8;
    Destination_address |= Buffer[1];
    Source_address = Buffer[4] << 8;
    Source_address |= Buffer[3];
    ProtocolVersion = (Buffer[5] >> 4) + (Buffer[5] & (1 << 15));

    Command = Buffer[7] << 8;
    Command |= Buffer[6];
    Framecnt = Buffer[8];
    Checksum = Buffer[9];  // Checksum is OK
    byte Verificacion = 0;
    for ( int i = 0; i <= 8; i++ )
      Verificacion += Buffer[i];
    Verificacion = ~ Verificacion;
#ifdef DEPURAR
    Serial.println("---------------");
    Serial.print("Destino: 0x");
    Serial.println(Destination_address, HEX);
    Serial.print("Origen:  0x");
    Serial.println(Source_address, HEX);
    Serial.print("Version: ");
    Serial.println(ProtocolVersion);
    Serial.print("Comando: 0x");
    Serial.println(Command, HEX);
    Serial.print("Longitud:");
    Serial.println(Framecnt);
    Serial.print("Checksum (recibido, calculado): 0x");
    Serial.print(Checksum, HEX);
    Serial.print(", 0x");
    Serial.println(Verificacion, HEX);
#endif
    // Solo interesan comando==0x100 (paquete de datos)
    // La longitud correcta es 10 bytes de cabecera mas 6 bytes/frame
    // and (Bufferlength == 10 + Framecnt * 6) a tomar por culo la correccion

    if ((Command == 0x0100) ) { // Paquete de datos. Esto nos interesa.
#ifdef DEPURAR
      Serial.println("---------------");
      Serial.print("Decodificando para:");
      Serial.println(Source_address, HEX);
#endif
      if (Source_address == 0x3271) {
        // Frame info for the Resol ConergyDT5
        // check VBusprotocol specification for other products

        // This library is made for the ConergyDT5 (0x3271)

        //Offset  Size    Mask    Name                    Factor  Unit
        //0       2               Temperature sensor 1    0.1     &#65533;C
        //2       2               Temperature sensor 2    0.1     &#65533;C
        //4       2               Temperature sensor 3    0.1     &#65533;C
        //6       2               Temperature sensor 4    0.1     &#65533;C
        //8       1               Pump speed pump         1       1
        //9       1               Pump speed pump 2       1
        //10      1               Relay mask              1
        //11      1               Error mask              1
        //12      2               System time             1
        //14      1               Scheme                  1
        //15      1       1       Option PostPulse        1
        //15      1       2       Option thermostat       1
        //15      1       4       Option HQM              1
        //16      2               Operating hours relay 1 1
        //18      2               Operating hours relay 2 1
        //20      2               Heat quantity           1       Wh
        //22      2               Heat quantity           1000    Wh
        //24      2               Heat quantity           1000000 Wh
        //26      2               Version 0.01
        //
        // Each frame has 6 bytes
        // byte 1 to 4 are data bytes -> MSB of each bytes
        // byte 5 is a septet and contains MSB of bytes 1 to 4
        // byte 6 is a checksum
        //
        //*******************  Frame 1  *******************

        F = FOffset;
        Septet = Buffer[F + FSeptet];
        InjectSeptet(Buffer, F, Septet, 4);

        // 'collector1' Temperatur Sensor 1, 15 bits, factor 0.1 in C
        temperaturas[0] = CalcTemp(Buffer[F + 1], Buffer[F]);
        // 'store1' Temperature sensor 2, 15 bits, factor 0.1 in C
        temperaturas[1] = CalcTemp(Buffer[F + 3], Buffer[F + 2]);

        //*******************  Frame 2  *******************
        F = FOffset + FLength;
        Septet = Buffer[F + FSeptet];
        InjectSeptet(Buffer, F, Septet, 4);

        temperaturas[2] = CalcTemp(Buffer[F + 1], Buffer[F]);
        temperaturas[3] = CalcTemp(Buffer[F + 3], Buffer[F + 2]);

        //*******************  Frame 3  *******************
        F = FOffset + FLength * 2;
        Septet = Buffer[F + FSeptet];
        InjectSeptet(Buffer, F, Septet, 4);

        PumpSpeed1 = (Buffer[F] & 0X7F);
        PumpSpeed2 = (Buffer[F + 1] & 0X7F);
        RelaisMask = Buffer[F + 2];
        ErrorMask  = Buffer[F + 3];

        //*******************  Frame 4  *******************
        F = FOffset + FLength * 3;
        Septet = Buffer[F + FSeptet];
        InjectSeptet(Buffer, F, Septet, 4);

        SystemTime = Buffer[F + 1] << 8 | Buffer[F];
        Scheme    =  Buffer[F + 2];

        OptionPostPulse  = (Buffer[F + 3] & 0x01);
        OptionThermostat = ((Buffer[F + 3] & 0x02) >> 1);
        OptionHQM  = ((Buffer[F + 3] & 0x04) >> 2);

        //*******************  Frame 5  *******************
        F = FOffset + FLength * 4;
        Septet = Buffer[F + FSeptet];
        InjectSeptet(Buffer, F, Septet, 4);

        OperatingHoursRelais1 = Buffer[F + 1] << 8 | Buffer[F];
        OperatingHoursRelais2 = Buffer[F + 3] << 8 | Buffer[F + 2];

        //*******************  Frame 6  *******************
        F = FOffset + FLength * 5;
        Septet = Buffer[F + FSeptet];
        InjectSeptet(Buffer, F, Septet, 4);

        HeatQuantity = (Buffer[F + 1] << 8 | Buffer[F]) + (Buffer[F + 3] << 8 | Buffer[F + 2]) * 1000;

        //*******************  Frame 7  *******************
        F = FOffset + FLength * 6;
        Septet = Buffer[F + FSeptet];
        InjectSeptet(Buffer, F, Septet, 4);

        HeatQuantity = HeatQuantity + (Buffer[F + 1] << 8 | Buffer[F]) * 1000000;
        Version = Buffer[F + 3] << 8 | Buffer[F + 2];

        ///******************* End of frames ****************

      }// end 0x3271 Conenergy DT5
      else if (Source_address == 0x5611) {
#ifdef DEPURAR
        Serial.println("Now decoding for 0x5611");
        Serial.println("---------------");
#endif
        // Frame info for the Resol Deltatherm FK and Oranier Aquacontrol III
        // check VBusprotocol specification for other products

        //

        //Offset  Size    Mask    Name                    Factor  Unit
        // Frame 1
        //0       2               Temperature sensor 1    0.1     &#65533;C
        //2       2               Temperature sensor 2    0.1     &#65533;C
        // Frame 2
        //4       2               Temperature sensor 3    0.1     &#65533;C
        //6       2               Temperature sensor 4    0.1     &#65533;C
        // Frame 3
        //8       1               Relay 1                 1       %
        //9       1               Relay 2                 1       %
        //10      1               Mixer open              1       %
        //11      1               Mixer closed            1       %
        // Frame 4
        //12      4               System date             1
        // Frame 5
        //16      2               System time             1
        //18      1               System notification     1
        //
        // Each frame has 6 bytes
        // byte 1 to 4 are data bytes -> MSB of each bytes
        // byte 5 is a septet and contains MSB of bytes 1 to 4
        // byte 6 is a checksum
        //
        //*******************  Frame 1  *******************

        F = FOffset;

        Septet = Buffer[F + FSeptet];
        InjectSeptet(Buffer, F, Septet, 4);

        temperaturas[0] = CalcTemp(Buffer[F + 1], Buffer[F]);
        temperaturas[1] = CalcTemp(Buffer[F + 3], Buffer[F + 2]);
        Serial.println( Buffer[F], HEX);
        Serial.println( Buffer[F + 1], HEX);
        Serial.println( Buffer[F + 2], HEX);
        Serial.println( Buffer[F + 3], HEX);

        //*******************  Frame 2  *******************
        F = FOffset + FLength;

        Septet = Buffer[F + FSeptet];
        InjectSeptet(Buffer, F, Septet, 4);

        temperaturas[3] = CalcTemp(Buffer[F + 1], Buffer[F]);
        temperaturas[4] = CalcTemp(Buffer[F + 3], Buffer[F + 2]);
        Serial.println( Buffer[F], HEX);
        Serial.println( Buffer[F + 1], HEX);
        Serial.println( Buffer[F + 2], HEX);
        Serial.println( Buffer[F + 3], HEX);
        //*******************  Frame 3  *******************
        F = FOffset + FLength * 2;

        Septet = Buffer[F + FSeptet];
        InjectSeptet(Buffer, F, Septet, 4);

        // Some of the values are 7 bit instead of 8.
        // Adding '& 0x7F' means you are only interested in the first 7 bits.
        // 0x7F = 0b1111111.
        // See: http://stackoverflow.com/questions/9552063/c-language-bitwise-trick
        Relay1 = (Buffer[F] & 0X7F);
        Relay2 = (Buffer[F + 1] & 0X7F);
        MixerOpen = (Buffer[F + 2] & 0X7F);
        MixerClosed  = (Buffer[F + 3] & 0X7F);
        //*******************  Frame 4  *******************
        F = FOffset + FLength * 3;

        Septet = Buffer[F + FSeptet];
        InjectSeptet(Buffer, F, Septet, 4);

        // System date is not needed for Domoticz

        //*******************  Frame 5  *******************
        F = FOffset + FLength * 4;

        Septet = Buffer[F + FSeptet];
        InjectSeptet(Buffer, F, Septet, 4);

        // System time is not needed for Domoticz

        // Status codes System Notification according to Resol:
        //0: no error / warning
        //1: S1 defect
        //2: S2 defect
        //3: S3 defect
        //4: VFD defect
        //5: Flow rate?
        //6: ΔT too high
        //7: Low water level

        SystemNotification = Buffer[F + 2];

        ///******************* End of frames ****************

      } //End 0x5611 Resol DeltaTherm FK

      /* Add your own controller ID and code in the if statement below and uncomment
         Segun https://github.com/openhab/openhab1-addons/wiki/Resol-VBUS-Binding la Resol DeltaSol M es 7311
        else if (Source_address ==0x????){
        }
      */

      else if (Source_address == 0x7311) { // Deltasol M, alias Roth B/W Komfort
        // 6 frames de temperaturas, 12 sensores
        for ( unsigned int i = 0; i <= 5; i++) {
          F = FOffset + i * FLength;
          Septet = Buffer[F + FSeptet];
          InjectSeptet(Buffer, F, Septet, 4);
          temperaturas[i * 2] = CalcTemp(Buffer[F + 1], Buffer[F]);
          temperaturas[i * 2 + 1] = CalcTemp(Buffer[F + 3], Buffer[F + 2]);
        }
        // Frame 7: Irradiación y no usado
        F = FOffset + 6 * FLength;
        irradiacion = CalcTemp(Buffer[F + 1], Buffer[F]); // No muy ortodoxo, pero hasta tener algo mejor...
        // Frame 8: Contador de pulsos 1
        // Frame 9: Contador de pulsos 2
        // Frame 10: Errores de sensor: sin sensor / corto
        // Frame 11: Sensores
        // Frame 12: Velocidad relés 1-4
        // Frame 13: Velocidad relés 5-8
        // Frame 14: Velocidad relés 9-12
        for ( unsigned int i = 0; i <= 2; i++) {
          F = FOffset + (i + 11) * FLength;
          for ( unsigned int j = 0; j <= 3 ; j++ ) { // cuatro reles por cada frame
            reles[ 4*i + j] = Buffer[F];
          }
        }
        // Frame 15: No usado / relés
        // Frame 16: Errores / Avisos
        // Frame 17: Versión, revisión / hora
#ifdef DEPURAR
        Serial.println("Valores recabados");
        Serial.print("Temperaturas: ");
        for (unsigned short i = 0; i <= ultimaTemperatura; i++) {
          Serial.print(temperaturas[i]);
          Serial.print(" ");
        }
        Serial.println("");
        Serial.print("Reles: ");
        for (unsigned short i = 0; i <= ultimoRele; i++) {
          Serial.print(reles[i]);
          Serial.print(" ");
        }
        Serial.println("");
        Serial.print("Irradiacion: ");
        Serial.println(irradiacion);
#endif
      } // 0x7311 Deltasol M

    } // end if command 0x0100
  } // end !quit

  return timeout;
}

