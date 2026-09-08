#include <ESP32Encoder.h>
#include <PID_v1.h>
#include <ESP32Servo.h>
/// Placa ESP32 NodeMCU-32s
// ============================
// CONFIGURACIÓN DE PINES
// ============================
#define MD1_DIRA 4      // Motor 1 dirección A
#define MD1_DIRB 16     // Motor 1 dirección B
#define MD1_PWM  5      // Motor 1 PWM
#define E1A 19          // Encoder 1 canal A
#define E1B 21          // Encoder 1 canal B

#define MD2_DIRA 17     // Motor 2 dirección A
#define MD2_DIRB 18     // Motor 2 dirección B
#define MD2_PWM  27     // Motor 2 PWM
#define E2A 22          // Encoder 2 canal A
#define E2B 23          // Encoder 2 canal B

#define SERVO_SSS 26    // Servo SSS
#define SERVO_SIS 25    // Servo SIS
#define SE_PI 36        // Potenciómetro PI (rho)
#define SE_PU 34        // Potenciómetro PU (theta)

long posicionEncoder1 = 0;
long posicionEncoder2 = 0;

// ============================
// OBJETOS Y VARIABLES
// ============================
Servo sss, sis;  // Objetos para servomotores
ESP32Encoder encoderMotor1, encoderMotor2;  // Encoders para motores

// Variables PID para control de posición
double posicionActual1 = 0, salidaPID1 = 0, posicionDeseada1 = 0;
double posicionActual2 = 0, salidaPID2 = 0, posicionDeseada2 = 0;
PID controladorPID1(&posicionActual1, &salidaPID1, &posicionDeseada1, 40.0, 5.0, 0.0, DIRECT);
PID controladorPID2(&posicionActual2, &salidaPID2, &posicionDeseada2, 40.0, 5.0, 0.0, DIRECT);

// Control PI para compensación del servo SIS
double r_input = 0, r_setpoint = 0, r_output = 90;
double Kp_sis = 5, Ki_sis = 0.5;
PID controladorPISIS(&r_input, &r_output, &r_setpoint, Kp_sis, Ki_sis, 0.0, DIRECT);

// Variables para filtro EMA (Exponential Moving Average)
#define ALPHA 0.5        // Factor de suavizado (0.1-0.5)
float filtered_PU = 0, filtered_PI = 0;
bool primera_lectura = true;

// Variables de comunicación con Python
bool enviarDatos = false;      // Controla envío de datos CSV
unsigned long tiempoInicio = 0; // Para timestamp relativo

// Constantes geométricas del brazo robótico
int L1=44, L2=44, L3=93, L4=93;
float betha = radians(13.32); //13.32 ANTERIOR
float Pr[] = {-L1*cos(betha), -L1*sin(betha)};
float Pl[] = {L2*cos(betha), -L2*sin(betha)};

// Tiempos de muestreo
const unsigned long tiempoMuestreo = 10;  // 10ms = 100Hz
const unsigned long tiempoMuestreo2 = 50;  // 50ms = 20Hz
unsigned long tiempoAnterior = 0;

// Relación de transmisión y resolución de encoders
const double relacionTransmision = 298.0 * (27.0 / 21.0);
const int cuentasPorVuelta = 6;
const double cuentasEjeSalida = relacionTransmision * cuentasPorVuelta;

// ============================
// SETUP - Configuración inicial
// ============================
void setup() {
  Serial.begin(115200);    // Inicia comunicación serial
  Serial.setTimeout(10);   // Timeout corto para lectura
  
  // Configurar pines de motores como salidas
  pinMode(MD1_DIRA, OUTPUT); 
  pinMode(MD1_DIRB, OUTPUT); 
  pinMode(MD1_PWM, OUTPUT);
  pinMode(MD2_DIRA, OUTPUT); 
  pinMode(MD2_DIRB, OUTPUT); 
  pinMode(MD2_PWM, OUTPUT);
  
  // Configurar encoders
  encoderMotor1.attachHalfQuad(E1A, E1B);
  encoderMotor2.attachHalfQuad(E2A, E2B);
  encoderMotor1.clearCount();  // Reiniciar contadores
  encoderMotor2.clearCount();
  
  // Configurar controladores PID
  controladorPID1.SetMode(AUTOMATIC);
  controladorPID1.SetOutputLimits(-255, 255);
  controladorPID1.SetSampleTime(tiempoMuestreo);
  
  controladorPID2.SetMode(AUTOMATIC);
  controladorPID2.SetOutputLimits(-255, 255);
  controladorPID2.SetSampleTime(tiempoMuestreo);
  
  // Configurar control PI para servo SIS
  controladorPISIS.SetMode(AUTOMATIC);
  controladorPISIS.SetSampleTime(tiempoMuestreo);
  controladorPISIS.SetOutputLimits(-30, 30); //rango de movimiento del servo inferior
  
  // Configurar servomotores
  sss.attach(SERVO_SSS);
  sis.attach(SERVO_SIS);
  sss.write(90);  // Posición inicial
  sis.write(90);
  
  // Señal de inicio para Python
  Serial.println("ARDUINO_READY");
}

// ============================
// FUNCIÓN: Mover motor
// ============================
void moverMotor(int motor, double velocidad) {
  int dira, dirb, pwmPin;
  
  // Seleccionar pines según motor
  if (motor == 1) { 
    dira = MD1_DIRA; dirb = MD1_DIRB; pwmPin = MD1_PWM; 
  } else { 
    dira = MD2_DIRA; dirb = MD2_DIRB; pwmPin = MD2_PWM; 
  }
  
  // Control de dirección según signo de velocidad
  if (velocidad > 0) { 
    digitalWrite(dira, HIGH); 
    digitalWrite(dirb, LOW); 
  } else if (velocidad < 0) { 
    digitalWrite(dira, LOW); 
    digitalWrite(dirb, HIGH); 
  } else { 
    digitalWrite(dira, LOW); 
    digitalWrite(dirb, LOW); 
  }
  
  // Aplicar PWM (valor absoluto)
  analogWrite(pwmPin, abs((int)velocidad));
}

// ============================
// FUNCIÓN: Leer sensores con filtro EMA
// ============================
void leerSensores(float &rho, float &rho_real, float &theta, float &theta_real) {
  // Leer valores RAW de potenciómetros
  
  float raw_PU = analogRead(SE_PU) - 68.0;  // Theta con comp -6° = Compensación_en_binario = (Compensación_en_grados × 4095) / 360
  float raw_PI = analogRead(SE_PI) - 68.0;  // Rho con comp -6° = Compensación_en_binario = (Compensación_en_grados × 4095) / 360
  
  // Convertir RAW a grados (sin filtro)
  rho_real = (raw_PI * 360.0) / 4095.0;      // Rho real compensado
  theta_real = (raw_PU * 360.0) / 4095.0;    // Theta real compensado
  
  // Aplicar filtro EMA (suavizado exponencial)
  if (primera_lectura) {
    // Primera lectura: inicializar filtros
    filtered_PU = raw_PU;
    filtered_PI = raw_PI;
    primera_lectura = false;
  } else {
    // Aplicar fórmula EMA: nuevo_valor = ALPHA*actual + (1-ALPHA)*anterior
    filtered_PU = ALPHA * raw_PU + (1 - ALPHA) * filtered_PU;
    filtered_PI = ALPHA * raw_PI + (1 - ALPHA) * filtered_PI;
  }
  
  // Convertir valores filtrados a grados
  float val_PU_filt = (filtered_PU * 360.0) / 4095.0;
  float val_PI_filt = (filtered_PI * 360.0) / 4095.0;
  
  theta=val_PU_filt;
  rho=val_PI_filt;


  // Aplicar compensaciones de calibración a valores filtrados
  //theta = 90.0 - fabs(val_PU_filt - 134.0);  // Theta filtrado con compensación
  //rho = val_PI_filt - 99.0;                  // Rho filtrado con compensación
}

// ============================
// FUNCIÓN: Procesar comandos de Python
// ============================
void procesarComandos() {
  if (Serial.available()) {
    String comando = Serial.readStringUntil('\n');
    comando.trim();
    
    // Comando para iniciar/parar envío de datos
    if (comando == "START") {
      enviarDatos = true;
      tiempoInicio = millis();  // Resetear timestamp
      Serial.println("ACK_START");
    }
    else if (comando == "STOP") {
      enviarDatos = false;
      Serial.println("ACK_STOP");
    }
    
    // Comandos de configuración de motores
    else if (comando.startsWith("M1:")) {
      posicionDeseada1 = comando.substring(3).toFloat();
      Serial.print("ACK_M1:");
      Serial.println(posicionDeseada1);
    }
    else if (comando.startsWith("M2:")) {
      posicionDeseada2 = comando.substring(3).toFloat();
      Serial.print("ACK_M2:");
      Serial.println(posicionDeseada2);
    }
    
    // Comandos para servomotores
    else if (comando.startsWith("SSS:")) {
      int angulo = comando.substring(4).toInt();
      sss.write(angulo - 10);  // Compensación de 10 grados
      Serial.print("ACK_SSS:");
      Serial.println(angulo);
    }
    else if (comando.startsWith("SIS:")) {
      int angulo = comando.substring(4).toInt();
      sis.write(angulo);
      Serial.print("ACK_SIS:");
      Serial.println(angulo);
    }
    
    // Solicitar header del CSV
    else if (comando == "HEADER") {
      Serial.println("Timestamp,Pos1,PWM1,Pos2,PWM2,SSS,SIS,rho,rho_real,theta,theta_real,gamma,diametro");
    }
  }
}

// ============================
// FUNCIÓN: Actualizar control PID
// ============================
void actualizarControl() {
  // Leer posiciones actuales de encoders y convertir a grados
  posicionEncoder1 = encoderMotor1.getCount();
  posicionActual1 = (double)posicionEncoder1 * (360.0 / cuentasEjeSalida);
  
  posicionEncoder2 = encoderMotor2.getCount();
  posicionActual2 = (double)posicionEncoder2 * (360.0 / cuentasEjeSalida);
  
  // Calcular salidas PID
  controladorPID1.Compute();
  controladorPID2.Compute();
  
  // Aplicar señales a motores
  moverMotor(1, salidaPID1);
  moverMotor(2, salidaPID2);
}

// ============================
// FUNCIÓN: Calcular r y diámetro del brazo
// ============================
void calcularRDiametro(float &r, float &diametro, float rho_filt, float theta_filt) {
  // L5: distancia entre puntos de unión
  float L5 = sqrt((L4*L4) + (L3*L3) - (2.0 * L4 * L3 * cos(radians(rho_filt))));
  
  // Ángulo tau usando ley de senos
  float tau = asin((sin(radians(rho_filt)) * L4) / L5);
  
  // Ángulo r = theta - tau (convertido a grados)
  r = theta_filt - degrees(tau);
  
  // Coordenadas del punto Pg
  float Pg[2] = { L5 * sin(radians(r)), -L5 * cos(radians(r)) };
  
  // Longitudes de los lados del triángulo
  float L6 = sqrt(pow(Pr[0] - Pg[0], 2) + pow(Pr[1] - Pg[1], 2));
  float L7 = sqrt(pow(Pl[0] - Pr[0], 2) + pow(Pl[1] - Pr[1], 2));
  float L8 = sqrt(pow(Pg[0] - Pl[0], 2) + pow(Pg[1] - Pl[1], 2));
  
  // Área del triángulo usando fórmula de coordenadas
  float A = fabs((Pr[0]*(Pg[1]-Pl[1]) + Pg[0]*(Pl[1]-Pr[1]) + Pl[0]*(Pr[1]-Pg[1])) / 2.0);
  
  // Radio del círculo circunscrito (fórmula geométrica)
  float Rc = ((L6 * L7 * L8) / (4.0 * A)) - 26;  // Restar offset constante
  
  // Diámetro = 2 * radio
  diametro = 2 * Rc;
}

// ============================
// FUNCIÓN: Enviar datos en formato CSV
// ============================
void enviarDatosCSV() {
  if (!enviarDatos) return;  // Solo enviar si está activado
  
  unsigned long tiempo = millis() - tiempoInicio;  // Timestamp relativo
  float rho, rho_real, theta, theta_real, r, diametro;
  
  // Leer sensores (valores reales y filtrados)
  leerSensores(rho, rho_real, theta, theta_real);
  
  // Calcular r y diámetro usando valores FILTRADOS
  calcularRDiametro(r, diametro, rho, theta);
  
  // Control PI para servo SIS usando valor r filtrado
  r_input = r;
  r_setpoint = 0;
  controladorPISIS.Compute();
  sis.write(sss.read() - (float)r_output);  // Compensación
  
  // Enviar línea CSV con todos los datos
  Serial.print(tiempo);                     // Timestamp relativo
  Serial.print(",");
  Serial.print(posicionActual1, 2);         // Posición motor 1
  Serial.print(",");
  Serial.print(salidaPID1, 2);              // PWM motor 1
  Serial.print(",");
  Serial.print(posicionActual2, 2);         // Posición motor 2
  Serial.print(",");
  Serial.print(salidaPID2, 2);              // PWM motor 2
  Serial.print(",");
  Serial.print(sss.read());                 // Ángulo servo SSS
  Serial.print(",");
  Serial.print(sis.read());                 // Ángulo servo SIS
  Serial.print(",");
  Serial.print(rho, 2);                     // Rho FILTRADO (para control)
  Serial.print(",");
  Serial.print(rho_real, 2);                // Rho REAL (para análisis)
  Serial.print(",");
  Serial.print(theta, 2);                   // Theta FILTRADO (para control)
  Serial.print(",");
  Serial.print(theta_real, 2);              // Theta REAL (para análisis)
  Serial.print(",");
  Serial.print(r, 2);                       // Ángulo r (calculado con filtrados)
  Serial.print(",");
  Serial.println(diametro, 2);              // Diámetro (calculado con filtrados)
}

// ============================
// LOOP - Bucle principal
// ============================
void loop() {
  unsigned long t = millis();
  
  // Ejecutar cada tiempoMuestreo (50ms)
  if (t - tiempoAnterior >= tiempoMuestreo2) {
    tiempoAnterior = t;
    
    // 1. Procesar comandos desde Python
    procesarComandos();
    
    // 2. Actualizar control PID de motores
    actualizarControl();
    
    // 3. Enviar datos en formato CSV
    enviarDatosCSV();
  }
}
