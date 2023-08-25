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

//Librerias de LORA
#include <SPI.h>
#include <RH_RF95.h>
#include <cmath>

  //modificar a valores necesarios
#define RFM95_CS 10 
#define RFM95_RST 9
#define RFM95_INT 2

  // Change to 434.0 or other frequency, must match RX's freq!
#define RF95_FREQ 915.0

  // Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);

///// Librerias de GPS
  #include <TinyGPS++.h>
  #include <SoftwareSerial.h>
  SoftwareSerial gpsSerial(10,11); //RX = 10; TX = 11
  TinyGPSPlus gps; // Declaramos el GPS
///// Librerias de brujula
  #include <Wire.h>
  #include <LSM303D.h>
  LSM303D compass;

  int16_t accel[3];  // we'll store the raw acceleration values here
  int16_t magnet[3];  // raw magnetometer values stored here
  float realAccel[3];  // calculated acceleration values here
  float heading_actual, titleHeading;

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
float gps_latitud_actual = 0; // latitud
float gps_longitud_actual = 0; // longitud
float altura_inicial = 0; // Una captura de la altura en metros del robot al inicio (Idealmente que sea cerca al piso)


///// variables helpers (variables que ayudan!)
bool timer_activado_fase0 = false;
unsigned long tiempo_fase0 = 0;


////// variables de prueba (temporales para prueba)
// heading deseado (Esta variable se consigue haciendo el calculo de orientacion de nuestra meta al robot)
float goal_heading = 20;

void setup() {
  
///// Setup: Iniciar sensores y comunicacion
// iniciar comm i2c para sensores
  Wire.begin();
//iniciar comm serial
  Serial.begin(9600);  // Start serial port
//iniciar gps
  gpsSerial.begin(9600);
//iniciar brujula
  char rtn = 0;
  rtn = Lsm303d.initI2C();
    //rtn = Lsm303d.initSPI(SPI_CS);
    if (rtn != 0) { // Initialize the LSM303, using a SCALE full-scale range
        Serial.println("\r\nLSM303D is not found");
        while (1);
    } else {
        Serial.println("\r\nLSM303D is found");
    }
    Lsm303d.getAccel(accel);
    while (!Lsm303d.isMagReady()); // wait for the magnetometer readings to be ready
    Lsm303d.getMag(magnet);  // get the magnetometer values, store them in mag

    for (int i = 0; i < 3; i++) {
        realAccel[i] = accel[i] / pow(2, 15) * ACCELE_SCALE;  // calculate real acceleration values, in units of g
    }
    titleHeading = Lsm303d.getTiltHeading(magnet, realAccel);

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
  altura_inicial = 0

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

////// Iniciar timers de los sensores
unsigned long timer_barometro = millis(); // en millis
unsigned long timer_gps = millis();  // en millis
unsigned long timer_brujula = millis();  // en millis
unsigned long timer_lora = millis(); //en millis

}

void loop() {
/// REGLAS
// NO PONER NINGUN DELAY()
// SOLO MODIFICAR VARIABLE AFUERA DE LA SECCION "condiciones"

///////// Leer todos los sensores

// timer de gps
if((millis()-timer_gps) > periodo_gps){
  gps_latitud_actual = gps.location.lat();
  gps_longitud_actual = gps.location.lng();
  timer_gps = millis();
}

// timer de barometro
if((millis()-timer_barometro) > periodo_barometro){
  /// TODO: Actualizar altura 
  altura_pasada = altura_actual;
  altura_actual = ;//No utilizamos el altimetro por falta de conecciones I2C
                   /// <----- leer el sensor y que te de la altura actual 

  velocidad_vertical_actual = ((altura_actual - altura_pasada) / (millis()-timer_barometro)) * 1000 // Velocidad vertical en metros por segudo
  timer_barometro = millis();
}

// timer de brujula
if(fase_actual == 4){ // No es necesario leer la brujula hasta la etapa 4
  if((millis()-timer_brujula) > periodo_brujula){
    heading_actual = compass.heading();
    timer_brujula = millis();
  }
}
// Timer para transmitir datos LORA
/// TODO:
if((millis()-timer_lora) > periodo_lora){
  gps_latitud_actual = gps.location.lat();
  gps_longitud_actual = gps.location.lng();
  timer_gps = millis();
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