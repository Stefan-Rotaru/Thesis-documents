// ----- Do not modify -----
#define USB_BAUDRATE         115200
#define RS485_BAUDRATE       2000000
#define RESOLUTION            12
#define RS485_ENC0            0x54
#define RS485_ENC1            0x58
#define RS485_POS             0x00
#define RS485_TURNS           0x01
#define RS485_ZERO            0x02
#define RS485_RESET           0x03
#define RS485_T_RE            8
#define RS485_T_DE            8
#define RS485_T_DI            18
#define RS485_T_RO            19
#define RS485_T_TX            0
#define RS485_T_RX            1 
#define ZERO_BUTTON           13
// -------------------------

#include <Servo.h>
#include <math.h>
// Pin d'attache sur l'arduino des servos-moteurs
#define SERVO_L_PIN 11
#define SERVO_R_PIN 10
Servo servoL, servoR;
float phi_prev;

// Structure reprenant toute les valeurs nécessaires pour le PID
struct control_values {
  float control;
  float prev_error;
  float integral;
};

// Fonction de return de la structure
control_values create_struct(){
  control_values result;
  result.control = 0;
  result.prev_error = 0;
  result.integral = 0;
  return result;
}

control_values control_val = create_struct();

// Fonction PID
void PID_controller(float setpoint, float measured, float Kp, float Ki, float Kd, float dt)
{
  float error = setpoint - measured;

  float P = Kp * error;
  float D = Kd * (error - control_val.prev_error) / dt;
  control_val.integral += Ki * error * dt;
  float U = P + control_val.integral + D;

  // Anti-windup
  if (U >= 20 * M_PI/180){ 
    control_val.integral -= Ki * error * dt;
    U = 20 * M_PI/180;
  }
  else if (U <= -20 * M_PI/180){
    control_val.integral -= Ki * error * dt;
    U = -20 * M_PI/180;
  }
  control_val.control = U;
  control_val.prev_error = error;
}

// Delais entre chaque boucle : dt
float time = 0;
float dt = 0.02;

void setup()
{ 
  // ----- Ne pas modifier -----
  setup_AMT();
  // ---------------------------

  // Setup en remetteant les ailerons droits et en attandant 2 secondes pour que l'avion se restabilise.
  float phi_prev = 0;
  servoL.attach(SERVO_L_PIN);
  servoL.write(101);
  servoR.attach(SERVO_R_PIN);
  servoR.write(94);
  delay(2000);
}

void loop()
{
  // ----- Ne pas modifier -----
  float angle_of_roll = 0;
  uint8_t addresses[1] = {RS485_ENC0};
  if(digitalRead(ZERO_BUTTON) == LOW)
  {
    for(int encoder = 0; encoder < sizeof(addresses); ++encoder)
    {
      Serial.println("Zeroing...");
      sendCommandRS485(addresses[encoder] | RS485_ZERO);
      delay(100); 
      Serial.println("Zero done. Resetting...");
      sendCommandRS485(addresses[encoder] | RS485_RESET);
      delay(500); 
    }
    while(digitalRead(ZERO_BUTTON) == LOW)
    {
      delay(100);
    }
  }
  for(int encoder = 0; encoder < sizeof(addresses); ++encoder)
  {
    while (Serial1.available()) Serial1.read();
    sendCommandRS485(addresses[encoder] | RS485_POS);
    delayMicroseconds(400);
    int bytes_received = Serial1.available();
    if (bytes_received == 2)
    {
      uint16_t currentPosition = Serial1.read(); 
      currentPosition |= Serial1.read() << 8;   
      if (verifyChecksumRS485(currentPosition))
      {
        currentPosition &= 0x3FFF;
        if (RESOLUTION == 12)
        {
          currentPosition = currentPosition >> 2;
        }

        // Lecture de l'angle et transformation en radians
        const uint16_t HALF_TURN      = 4096 / 2;
        int16_t signedCounts = currentPosition;      
        if (signedCounts >= HALF_TURN)              
          signedCounts -= 4096;          
        angle_of_roll = -(360.0f * signedCounts) / 4096;
      }
      else
      {
        Serial.print("Encoder #");
        Serial.print(encoder, DEC);
        Serial.println(" error: Invalid checksum.");
      }
    }
    else
    {
      Serial.print("Encoder #");
      Serial.print(encoder, DEC);
      Serial.print(" error: Expected to receive 2 bytes. Actually received ");
      Serial.print(bytes_received, DEC);
      Serial.println(" bytes.");
    }
    while (Serial1.available()) Serial1.read();
  }
  // --------------------

  // ------ Partie modifiable ------
  float setpoint = 0; // Setpoint en radians

  // Ici on peut mettre differentes position de roulis en fonction du temps. Depend du test qu'on veut réaliser.
  // if (time < 20){
  //   setpoint = 0;
  // }
  if (time >= 13){
    setpoint = 10* M_PI/180;
  }
  // if (time >= 16){
  //   setpoint = 0* M_PI/180;
  // }
  // if (time >= 19){
  //   setpoint = 20* M_PI/180;
  // }
  // if (time >= 22){
  //   setpoint = 0* M_PI/180;
  // }

  // ----- PID -----
  // Coefficients  PID
  float Kp = 1.1;
  float Ki = 1.2;
  float Kd = 0.7;
  PID_controller(setpoint, angle_of_roll * M_PI / 180, Kp, Ki, Kd, dt);

  // Transformer l'angle de control en un angle de servomoteur
  float delta_s_R = control_val.control * 180/M_PI * 1200/95 + 1470;
  float delta_s_L = control_val.control * 180/M_PI * 1200/95 + 1580; 

  // Commande au servo-moteur
  servoL.writeMicroseconds(delta_s_L);
  servoR.writeMicroseconds(delta_s_R);

  // Récuperation des données
  Serial.print(angle_of_roll);
  Serial.print(" ,");
  Serial.print(control_val.control * 180/M_PI);
  double p_dot = ((angle_of_roll - phi_prev)) / dt;
  Serial.print(" ,");
  Serial.println(p_dot);
  phi_prev = angle_of_roll;

  // Boucle temporelle
  time += dt;
  delay(dt * 1000);
}


// ---- Ne pas modifier -----
void setup_AMT()
{
  pinMode(RS485_T_RE, OUTPUT);
  pinMode(RS485_T_DE, OUTPUT);
  pinMode(RS485_T_RO, INPUT_PULLUP);
  pinMode(ZERO_BUTTON, INPUT_PULLUP); 
  Serial.begin(USB_BAUDRATE);
  Serial1.begin(RS485_BAUDRATE);
}

bool verifyChecksumRS485(uint16_t message)
{
  uint16_t checksum = 0x3;
  for(int i = 0; i < 14; i += 2)
  {
    checksum ^= (message >> i) & 0x3;
  }
  return checksum == (message >> 14);
}

void sendCommandRS485(uint8_t command)
{
  PORTH |= 0b01100000; 
  delayMicroseconds(10); 
  Serial1.write(command); 
  while (!(UCSR1A & _BV(TXC1))); 
  PORTH &= 0b10011111; 
}