//Implementaicon inicial del codigo final hecho por Tomas con la idea que los demas puedan llenar los otros componentes

/*lineas pendientes (ToDo):(lineas pueden no ser exactas)
129 TODO: guardar altura inicial del robot para uso en las fases futuras
139 TODO: Verficiar las lecturas
176 TODO: Actualizar altura
192 TODO: Timer para transmitir datos LORA
195 TODO: Timer para grabar en SD
275 TODO: Crear una equacion que tome la distancia de gps actual del robot y la meta. Si la distancia es menor a "tolerancia_de_meta" (variable) entoces entrar a etapa 5
310 TODO: mover las llantas a velocidad 100% para lograr que el paracaidas y el robot no se enreden con forme se despega el paragaidas
314 TODO: Activar el servo para despegar el paracaidas
376 OPCIONAL case 5 ver cual es el default 
  */
//################ Iniciar librerias ###################//

// * Librerias de LORA
#include <SPI.h>
#include <RH_RF95.h>
#include <cmath>

// * Valores para el LoRa
#define RFM95_CS 10 
#define RFM95_RST 9
#define RFM95_INT 2
#define RF95_FREQ 915.0

// * Librerias de GPS
  #include <TinyGPS++.h>
  #include <SoftwareSerial.h>


// * Librerias de brujula
  #include <Wire.h>
  #include <LSM303D.h>
  //LSM303D compass;

/*  int16_t accel[3];  // we'll store the raw acceleration values here
  int16_t magnet[3];  // raw magnetometer values stored here
  float realAccel[3];  // calculated acceleration values here
  float heading_actual, titleHeading;
*/
//// Libreria de servo

  #include <Servo.h>
  Servo servo_paracaidas;
  
//################# Variables Globales ######################  ///
// Todas las variables que se van a ocupar atravez de la mision

//////#/#/#/#/  Parametros de mision () /#/#/#/#/#/#
float rango_altura_activacion = 100; //(En metros) A que altura sobre la altura inicial quieres que el robot detecte que esta subiendo 
int fase_de_inicio = 0; // La fase en la que va a iniciar el robot
float tolerancia_de_meta = 10; // Radio de tolerancia en metros de que tan cerca consideramos que el robot ya llego a su meta (necesario cuando el GPS no tiene buena resolucion)
bool DEBUG = true; //Determinar si queremos saber todos las salidas
float gps_latitud_meta = 0; // latitud meta
float gps_longitud_meta = 0; // longitud meta
float dif_long_meta = 0 //valor absoluto de la resta entre la distancia de longitud actual y de la meta
float dif_lat_meta = 0 //valor absoluto de la resta entre la distancia de latitud actual y de la meta

///// Parametros del robot (navegacion, control, actuacion)
float kp = 1.5; // factor proporcional del control del motor (que tan agresivos queremos acercarnos a el heading correcto)
int pos_servo_cerrado = 20; // posicion en la que el servo esta cerrado
int pos_servo_abierto = 180; // posicion en la que el servo esta abierto para soltar el paracaidas

///// Pines del robot
int pin_motor_izquierda = 6; // Pin conectado al mosfet de la izquierda (controla el motor, actuar con salida pwm analog output)
int pin_motor_derecha = 5; // Pin conectado al mosfet de la derecha (controla el motor, actuar con salida pwm analog output)

///// Estado del robot (no modificar directamente estos variables)
int fase_actual = 0; // fase actual del robot
float velocidad_vertical_actual = 0; // metros por segundo
float altura_actual = 0; // metros
float heading_actual = 0; // grados sobrel el horizonte 
//float gps_latitud_actual = 0; // latitud
//float gps_longitud_actual = 0; // longitud
float altura_inicial = 0; // Una captura de la altura en metros del robot al inicio (Idealmente que sea cerca al piso)


///// variables helpers (variables que ayudan!)
bool timer_activado_fase0 = false;
unsigned long tiempo_fase0 = 0;


////// variables de prueba (temporales para prueba)
// heading deseado (Esta variable se consigue haciendo el calculo de orientacion de nuestra meta al robot)
float goal_heading = 20;

// ? Definicion de la interfaz sensor
class Sensor{
  public:
    virtual float getData() = 0;
}

// ? Definicion del sensor GPS
class GPS : public Sensor {
public:
  SoftwareSerial& gpsSerial;
  TinyGPSPlus& gps;
  float& gps_latitud_actual;
  float& gps_longitud_actual;

  GPS(SoftwareSerial& serial, TinyGPSPlus& gps, float& latitud, float& longitud)
    : gpsSerial(serial), gps(gps), gps_latitud_actual(latitud), gps_longitud_actual(longitud) {
      gpsSerial.begin(9600);
    }

  // * Obtener datos del GPS
  void getData(LoRa& lora) override {
    gps_latitud_actual = gps.location.lat();
    gps_longitud_actual = gps.location.lng();
    lora.sendData("GPS", String(gps_latitud_actual).c_str()); // * Mandar los datos al LoRa
  }

};

SoftwareSerial gpsSerial(10, 11); // RX = 10, TX = 11
TinyGPSPlus gps; // Declaramos el GPS
float gps_latitud_actual = 0.0; // ! No modificar directamente
float gps_longitud_actual = 0.0; // ! No modificar directamente


// ? Inicialización de la brujula

class Compass : public Sensor {
public:
  LSM303D& lsm303d;
  float heading_actual;

  Compass(LSM303D& compass)
    : lsm303d(compass), heading_actual(0.0) {
      int16_t accel[3];
      int16_t magnet[3];
      float realAccel[3];
      float titleHeading;

      char rtn = 0;
      rtn = lsm303d.initI2C();
      if (rtn != 0) {
        Serial.println("\r\nLSM303D is not found");
        while (1);
      } else {
        Serial.println("\r\nLSM303D is found");
      }

      lsm303d.getAccel(accel);
      while (!lsm303d.isMagReady());
      lsm303d.getMag(magnet);

      for (int i = 0; i < 3; i++) {
        realAccel[i] = accel[i] / pow(2, 15) * ACCELE_SCALE;
      }
      titleHeading = lsm303d.getTiltHeading(magnet, realAccel);
  }

  void getData() override {
    heading_actual = lsm303d.heading();
    lora.sendData("Compass", String(heading_actual).c_str()); // * Mandar los datos al LoRa
  }

  // * Mandar los datos al LoRa
  void sendData(LoRa& lora) override {
    char sensorName[] = "Compass";
    char radiopacket[50];
    sprintf(radiopacket, "%s: Heading %.2f", sensorName, heading_actual);

    lora.sendData(radiopacket);
  }
};

LSM303D compass;

class LoRa {
private:
  RH_RF95& rf95;

public:
  LoRa(RH_RF95& radio)
    : rf95(radio) {
      pinMode(RFM95_RST, OUTPUT);
      delay(10);
      digitalWrite(RFM95_RST, HIGH);

      while (!Serial);
      Serial.begin(9600);

      digitalWrite(RFM95_RST, LOW);
      delay(10);
      digitalWrite(RFM95_RST, HIGH);

      while (!rf95.init()) {
        Serial.println("LoRa radio init failed");
        while (1);
      }
      Serial.println("LoRa radio init OK!");

      if (!rf95.setFrequency(RF95_FREQ)) {
        Serial.println("setFrequency failed");
        while (1);
      }

      Serial.print("Set Freq to: "); Serial.println(RF95_FREQ);

      rf95.setTxPower(23, false);
    }

  void sendData(const char* radiopacket) {
    Serial.print("Sending "); Serial.println(radiopacket);

    rf95.send((uint8_t *)radiopacket, strlen(radiopacket));
    rf95.waitPacketSent();

    Serial.println("Packet sent");
  }
};

// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);


void setup() {
  
///// Setup: Iniciar sensores y comunicacion
// * iniciar comm i2c para sensores
  Wire.begin();
// ! Iniciar comunicaciones seriales
  // ? Comunicacion serial del Compass
  Serial.begin(9600);  // Start serial port
  // ? Comunicacion serial del GPS
  gpsSerial.begin(9600);

///// Setup: Iniciar outputs (motores y servos)
//Definir los outputs para el motor
  pinMode(pin_motor_derecha, OUTPUT);  // definir output para motor de la derecha
  pinMode(pin_motor_izquierda, OUTPUT);  // definir output para motor de la derecha

//Definir los outputs para el servo
  servo_paracaidas.attach(4);
  servo_paracaidas.write(0);
////// Setup: Actualizar los estados y lectura de sensor

  delay(5000); // dormir 5 segundos para permitir que los sensores se estabilicen 

fase_actual = fase_de_inicio; // declarar la fase en la que vamos a iniciar (con el parametro de mision fase_de_inicio) 

/// TODO: guardar altura inicial del robot para uso en las fases futuras
  altura_inicial = 0;

  velocidad_vertical_actual = ((accel * tiempo_caida) + 0);
  altura_actual = gps.altitude.meters();
  heading_actual = Lsm303d.getHeading(magnet); 
  gps_latitud_actual = gps.location.lat();
  gps_longitud_actual = gps.location.lng();

if(DEBUG){
  /// TODO: Verficiar las lecturas
  Serial.println("-------------Lectura inicial------------");
  Serial.print("altura_inicial: ");
  Serial.print(altura_inicial);
  Serial.print("velocidad_vertical_actual: ");
  Serial.print(velocidad_vertical_actual);
  Serial.print("heading_actual: ");
  Serial.print(heading_actual);
  Serial.print("gps_latitud_actual: ");
  Serial.print(gps_latitud_actual);
  Serial.print("gps_longitud_actual: ");
  Serial.print(gps_longitud_actual);
}

// ! Iniciar timers de los sensores, no se usarian
unsigned long timer_barometro = millis(); // en millis
unsigned long timer_gps = millis();  // en millis
unsigned long timer_brujula = millis();  // en millis
unsigned long timer_lora = millis(); //en millis

// Iniciar timer lectura de los sensores
unsigned long previousMillis = 0;
const unsigned long interval = 5000;

// * Iniciar sensores
GPS gpsSensor(gpsSerial, gps, gps_latitud_actual, gps_longitud_actual); // ? Declaracion del GPS
Compass compassSensor(compass); // ? Declaracion del Compass
LoRa loraModule(rf95);

}

void loop() {
/// REGLAS
// NO PONER NINGUN DELAY()
// SOLO MODIFICAR VARIABLE AFUERA DE LA SECCION "condiciones"

///////// Leer todos los sensores

// * Lectura de todos los sensores
unsigned long currentMillis = millis();

if (currentMillis - previousMillis >= interval) {
    // Actualiza el tiempo del último ciclo
    previousMillis = currentMillis;
    gpsSensor.getData(loraModule);
    if(fase_actual == 4){
      compassSensor.getData(loraModule);
    }
}
// ! timer de barometro
if((millis()-timer_barometro) > periodo_barometro){
  /// TODO: Actualizar altura 
  altura_pasada = altura_actual;
  altura_actual = ;//No utilizamos el altimetro por falta de conecciones I2C
                   /// <----- leer el sensor y que te de la altura actual 

  velocidad_vertical_actual = ((altura_actual - altura_pasada) / (millis()-timer_barometro)) * 1000 // Velocidad vertical en metros por segudo
  timer_barometro = millis();
}

// Timer para grabar en SD
/// TODO:



//////////Condiciones///////////
switch (fase_actual) {
  case 0:
    //fase_actual 0
    if(altura_inicial + rango_altura_activacion < altura_actual){ //Si llegamos a esta condicion significa que estamos arriba de 100 metros y queremos estar en esta condicion por 10 segundos
     
      if(timer_activado_fase0){// si el timer ya esta activado vamos ver cuanto tiempo falta
        if(millis() - tiempo_fase0 > 10000 ){
          fase_actual = 1;
        }
      }
      else{
        tiempo_fase0 = millis();
        timer_activado_fase0 = true;
      }
    }
    else{
      timer_activado_fase0 = false
    }

    break;
  case 1:
    //fase_actual 1
    if(velocidad_vertical_actual < velocidad_descenso_activacion){ //Si llegamos a esta condicion significa que tenemos una velocidad negativa AKA estamos bajando 
     
      if(timer_activado_fase1){// si el timer ya esta activado vamos ver cuanto tiempo falta
        if(millis() - tiempo_fase1 > 10000 ){
          fase_actual = 2;
        }
      }
      else{
        tiempo_fase1 = millis();
        timer_activado_fase1 = true;
      }
    }
    else{
      timer_activado_fase1 = false
    }

    break;
  case 2:
    //fase_actual 2
    if(velocidad_vertical_actual < velocidad_aterrizage_tolerancia && velocidad_vertical_actual > -velocidad_aterrizage_tolerancia){ //Si llegamos a esta condicion significa que tenemos una velocidad negativa AKA estamos bajando 
     
      if(timer_activado_fase2){// si el timer ya esta activado vamos ver cuanto tiempo falta
        if(millis() - tiempo_fase2 > 10000 ){
          fase_actual = 3;
        }
      }
      else{
        tiempo_fase1 = millis();
        timer_activado_fase2 = true;
      }
    }
    else{
      timer_activado_fase2 = false
    }
    break;
  case 3:
    //fase_actual 3
    // esperar 3 segundos
    if(timer_activado_fase3){// si el timer ya esta activado vamos ver cuanto tiempo falta
        if(millis() - tiempo_fase3 > 3000 ){
          fase_actual = 4;
        }
      }
    else{
        tiempo_fase1 = millis();
        timer_activado_fase3 = true;
      }

    break;
  case 4:
    //fase_actual 4
    /// TODO: Crear una ecuacion que tome la distancia de gps actual del robot y la meta. Si la distancia es menor a "tolerancia_de_meta" (variable) entoces entrar a etapa 5
    //gps_latitud_actual = gps.location.lat();
    //gps_longitud_actual = gps.location.lng();
    //float gps_latitud_meta = 0; // latitud meta
    //float gps_longitud_meta = 0; // longitud meta
    dif_long_meta=abs(gps_longitud_actual-gps_longitud_meta);
    if(dif_long_meta<tolerancia_de_meta)
    {
      fase_actual = 5;
    }
    
    break;
  case 5:
    //fase_actual 5
    break;



  default:
    // if nothing else matches, do the default
    // default is optional
    break;
}

////////// Acciones/////////////
switch (fase_actual) {
 
  case 0:
    //fase_actual 1
    break;
  case 1:
    //fase_actual 1
    break;
  case 2:
    //fase_actual 2
    break;
  case 3:
    //fase_actual 3
    /// TODO: mover las llantas a velocidad 100% para lograr que el paracaidas y el robot no se enreden con forme se despega el paragaidas 

    
    /// TODO: Activar el servo para despegar el paracaidas
    analogWrite(motor_pin_izquierda, 255);
    analogWrite(motor_pin_derecha, 255);
    servo.write(90);
    break;
  case 4:
    //fase_actual 4
   
    float error;
    // Se ocupa un delay de 100 millis pero en la implementacion real se va a cambiar por un non-blocking delay
    delay(100);
    
    Serial.print("    angle full: ");     // Display 16 bit angle with decimal place
    Serial.print(heading_actual, DEC);
    Serial.print("    angle desired: ");    
    Serial.print(goal_heading, DEC);

    //calcular la differencia entre angulo deseado y actual
    error = (goal_heading - heading_actual);
   
    if (error > 180){
        error = error -2*180;
    }
    else if( error < -180){
        error = error +2*180;
    }
    else{
    error = error;
    }

    Serial.print("    error: ");    
    Serial.println(error, DEC);

    int output;
    //tomar el error y corregir
    if(error > 0){
     output = error * kp;
     if(output > 255){
       output = 255;
     }
     
    analogWrite(motor_pin_derecha, 0  );
    analogWrite(motor_pin_izquierda, 0 + output);
    }
    else if(error == 0){
    analogWrite(motor_pin_derecha, 0 );
    analogWrite(motor_pin_izquierda, 0);
    }
    else{
      output = abs(error) *kp;
           if(output > 255){
       output = 255;
     }
    analogWrite(motor_pin_derecha, 0 + output);
    analogWrite(motor_pin_izquierda, 0);
    }
    break;
  case 5:
    //fase_actual 5
    break;

  default:
    // if nothing else matches, do the default
    // default is optional
    break;
  }
}