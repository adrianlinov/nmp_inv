
// **** Código utilizado para el Nodo de Medición de Parámetros ****
//
// Referencias:
// https://circuitdigest.com/microcontroller-projects/raspberry-pi-with-lora-peer-to-peer-communication-with-arduino 
// Dependencias
#include <DallasTemperature.h>
#include <OneWire.h>
#include <BH1750.h>
#include <RH_RF95.h>
#include <ArduinoJson.h>

// Reiniciar el Arduino por software
void(* resetArduino)(void) = 0;

// Sensor de Temperatura (Agua y Aire)
struct TemperatureSensor
{

  OneWire ourWire;
  DallasTemperature dallasSensor;
  uint8_t powerPin = 5;

  // Inicializar el sensor con un pin de lectura de datos
  // y un pin de energía
  void begin(uint8_t pin, uint8_t powerPinT)
  {
    powerPin = powerPinT;
    pinMode(powerPin, OUTPUT);
    digitalWrite(powerPin, HIGH);
    ourWire.begin(pin);
    dallasSensor.setOneWire(&ourWire);
    digitalWrite(powerPin, LOW);
  }

  // Obtener el valor del sensor
  float getValue()
  { 
    // Prende el sensor
    digitalWrite(powerPin, HIGH);
    delay(250);
    // Lee el valor
    dallasSensor.requestTemperatures();
    float temp = dallasSensor.getTempCByIndex(0);
    // Apaga el sensor
    digitalWrite(powerPin, LOW);
    return temp;
  }
};

// Paquete de datos enviados por LoRa
struct Payload
{
  // Memoria reservada para el paquete
  char value[100] = "";

  // Obtener dígito validador para verificar integridad luego
  void addCheckSum()
  {
    int checksum = 0;
    int SizeOfArray = strlen(value);

    // Iterar por posiciones y sumar su valor en char
    for (int x = 0; x < SizeOfArray; x++) {
      checksum += value[x];
    }

    // Concatenar el paquete con el dígito validador
    strcat(value, "-");
    char aux[5];
    sprintf(aux, "%04d", checksum);
    strcat(value, aux);
  }

  // Serializar JSON
  void serialize(StaticJsonDocument<100> doc)
  {
    memset(value, 0, 100);
    serializeJson(doc, value);
  }
};

// Sensor de pH
struct PHSensor
{
  uint8_t pin;
  
  // Compensacion asignada al calibrar con agua destilada
  float compensation = 1.38;

  // Inicializar sensor
  void begin(uint8_t newPin)
  {
    pin = newPin;
    pinMode(8, OUTPUT);
    digitalWrite(8, LOW);
    
  }
  // Obtener valor del sensor
  float getValue()
  { 
    digitalWrite(8, HIGH);
    float po = 0;
    for (int c = 0; c < 2000; c++)
    {
      //po = po + ((analogRead(pin) / 1023.0) * 14.0) + compensation;
      po = po + ((analogRead(pin) / ((analogRead(A1) * 5.0) / 3.3)) * 14.0) + compensation;
    }
    po = po / 2000;
    digitalWrite(8, LOW);
    return po;
  }
};
// Sensor de Luz
struct LuxSensor
{
  BH1750 luxSensor;
  float compensation = 0.0;

  // Inicializar sensor
  void begin()
  {
    pinMode(7, OUTPUT);
    digitalWrite(7, HIGH);  
    Wire.begin();
    luxSensor.begin();
    digitalWrite(7, LOW); 
  }

  // Obtener valor del sensor
  int getValue()
  { 
    // Prender sensor
    digitalWrite(7, HIGH);
    delay(500);
    // Leer datos
    int temp = luxSensor.readLightLevel() - compensation;
    // Apagar sensor
    digitalWrite(7, LOW);
    return temp;
  }

 
};

// Emisor LoRa
struct LoraTransiver
{

  RH_RF95 *RF95;
  // Inicializar Modulo
  void begin(uint8_t RFM95_INT, uint8_t RFM95_RST, uint8_t RFM95_CS, float RF95_FREQ)
  {
    RH_RF95 rf95(RFM95_CS, RFM95_INT);
    RF95 = &rf95;
    
    // Reiniciar modulo
    pinMode(RFM95_RST, OUTPUT);
    digitalWrite(RFM95_RST, LOW);
    delay(10);
    digitalWrite(RFM95_RST, HIGH);
    delay(10);

    // Esperar a que el modulo se inicialice
    while (!rf95.init())
    {
      while (1)
        ;
    }

    //Establecer frecuencia
    if (!rf95.setFrequency(RF95_FREQ))
    {
      while (1)
        ;
    }
    rf95.setTxPower(15); // Establecer poder de transmisión
  }

  // Enviar paquete
  void sendPayload(Payload payload)
  {
    (*RF95).send((uint8_t *)payload.value, strlen(payload.value));
  }
};

// Sensor de Oxigeno
struct DOSesor
{
  // Inicializar sensor
  void begin()
  {
    Serial.begin(9600);
    // Dormir el sensor para ahorrar energía
    sleep();
  }
  void sleep()

  {
    // Envía comando SLEEP para dormir
    delay(1500);
    Serial.write("SLEEP");
    Serial.write('\r');
    // Espera confirmación
    for (int i = 0; i <= 50000; i++)
    {
      if (Serial.read() == '\r')
      {
        break;
      }
    }
  }

  // Despierta el sensor
  void awake()
  {
    Serial.write("K");
    Serial.write('\r');
    delay(1500);
    for (int i = 0; i <= 50000; i++)
    {
      if (Serial.read() == '\r')
      {
        break;
      }
    }
  }

  // Obtener datos del sensor
  float getValue()
  { 
    // Despertar el sensor
    awake();

    // Envía comando de lectura
    Serial.write('R');
    Serial.write('\r');

    char buffer[10];
    int cont = 0;
    // Lee datos
    for (int i = 0; i <= 50000; i++)
    {
      if (Serial.available() > 0)
      {
        char res = Serial.read();
        // Serial.write(res);
        buffer[cont] = res;
        if (res == '\r')
        {
          break;
        }
        cont = cont + 1;
      }
    }
    // Duerme al sensor
    sleep();
    // Verifica si hubo error de lectura y reinicia el Arduino en caso lo hubiere
    if (atof(buffer) == 0 || atof(buffer) == 0.0) {
      resetArduino();
      }
    return atof(buffer);
  }

};
// Declarar variables
LoraTransiver lT;
TemperatureSensor tempW;
TemperatureSensor tempA;
PHSensor phS;
LuxSensor luxS;
DOSesor doS;

// Establecer pines de salida
void setupDigitalPins()
{
  pinMode(6, OUTPUT);
  pinMode(7, OUTPUT);
  pinMode(8, OUTPUT);
  pinMode(A1, OUTPUT);
  pinMode(A0, OUTPUT);
  pinMode(A2, OUTPUT);
  pinMode(A3, OUTPUT);
}

// Inicializar sensores
void setup()
{
  lT.begin(2, 9, 10, 434.0);
  tempW.begin(3, 5);
  tempA.begin(4, 6);
  phS.begin(A7);
  doS.begin();
  luxS.begin();
  setupDigitalPins();
}

// Tiempo de espera
void sleep(int segundos)
{
  for (int i = 0; i < segundos; i++)
  {
    delay(1000);
  }
}

void loop()
{
  
  StaticJsonDocument<100> doc;
  Payload payload;

  // Esperar 10 segundos
  sleep(10);

  // PASO1: MEDIR PARÁMETROS
  doc["WATER_TEMP"] = tempW.getValue();

  // PASO2: SERIALIZAR PAYLOAD
  payload.serialize(doc);

  // PASO3: HACER CHECKSUM
  payload.addCheckSum();

  // PASO4: ENVIAR PAYLOAD
  lT.sendPayload(payload);

  // PASO5: LIMPIAR JSON
  doc.clear();

  // PASO1: MEDIR PARÁMETROS
  doc["AMB_LIGHT"] = luxS.getValue();
   doc["AIR_TEMP"] = tempA.getValue();

  // PASO2: CONCATENAR PAYLOAD
  payload.serialize(doc);

  // PASO3: HACER CHECKSUM
  payload.addCheckSum();

  // PASO4: ENVIAR PAYLOAD
  lT.sendPayload(payload);

  // PASO5: LIMPIAR JSON
  doc.clear();

  doc["WATER_PH"] = phS.getValue();
  doc["WATER_O2"] = doS.getValue();

  // PASO2: CONCATENAR PAYLOAD
  payload.serialize(doc);

  // PASO3: HACER CHECKSUM
  payload.addCheckSum();

  // PASO4: ENVIAR PAYLOAD
  lT.sendPayload(payload);

  // PASO5: LIMPIAR JSON
  doc.clear();
}
