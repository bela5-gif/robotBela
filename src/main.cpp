#include <Arduino.h>
#include <SPI.h>
#include <MFRC522.h>
#include <PID_v1.h>

// --- Függvénydeklarációk ---
void moveForward(int motorleft, int motorright );
void stopMotors();
void turnRight();
bool compareUID(byte *uid, byte *ref);
void countLeft();
void countRight();
void tavolsagmeres();
void tolat();
void pidmozgas(double kozep, int maxS);

// RFID
#define RST_PIN A4
#define SS_PIN 10
MFRC522 mfrc522(SS_PIN, RST_PIN);

// L298N motorvezérlés
#define ENA 5
#define IN1 2
#define IN2 3
#define ENB 6
#define IN3 4
#define IN4 7

// IR szenzorok (analóg)
#define IR1 A0
#define IR2 A1
#define IR3 A2
#define IR4 A3

// Enkóderek
#define LEFT_ENCODER_PIN 8
#define RIGHT_ENCODER_PIN 9

volatile int leftPulses = 0;
volatile int rightPulses = 0;

unsigned long lastMeasure = 0;
int pulsesPerRev = 20;
float wheelCircumference = 20.0;

bool started = false;

double tavolsagok[4];
bool kozel = false;
int maxS = 200;

int pidmode = 2;
double setpoint = 0; // PID célérték
double input, output;
double Pid_P = 20, Pid_I = 0.1, Pid_D = 5; // PID tényezők
PID pid(&input, &output, &setpoint, Pid_P, Pid_I, Pid_D, DIRECT);

// RFID UID-k
byte startUID[4]  = {0x04, 0x77, 0x7E, 0x8A};
byte rightUID[4]  = {0x04, 0x77, 0x7D, 0x8A};
byte stopUID[4]   = {0x04, 0x77, 0x7C, 0x8A};

void setup() {
  Serial.begin(9600);
  SPI.begin();
  mfrc522.PCD_Init();

  pinMode(ENA, OUTPUT); pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT); pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);

  pinMode(LEFT_ENCODER_PIN, INPUT_PULLUP);
  pinMode(RIGHT_ENCODER_PIN, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(LEFT_ENCODER_PIN), countLeft, RISING);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENCODER_PIN), countRight, RISING);

  stopMotors();
  Serial.println("Robot készen áll.");
}

void loop() {
  int ir1 = analogRead(IR1);
  int ir2 = analogRead(IR2);
  int ir3 = analogRead(IR3);
  int ir4 = analogRead(IR4);

  if (mfrc522.PICC_IsNewCardPresent() && mfrc522.PICC_ReadCardSerial()) {
    byte* uid = mfrc522.uid.uidByte;

    if (compareUID(uid, startUID)) {
      Serial.println("START kártya olvasva.");
      started = true;
    } 
    else if (compareUID(uid, rightUID) && started) {
      Serial.println("JOBBRA fordulás");
      turnRight();
    } 
    else if (compareUID(uid, stopUID) && started) {
      Serial.println("STOP kártya olvasva – megállás.");
      started = false;
      stopMotors();
    }

    mfrc522.PICC_HaltA();
    delay(1000);
  }

  if (started) {
    while (true)
    {
      tavolsagmeres();
    }
    
    /*if (ir1 > 700 || ir2 > 700 || ir3 > 700 || ir4 > 700) {
      Serial.println("Fal érzékelve → jobbra fordulás");
      stopMotors();
      delay(300);
      turnRight();
    } else {
      moveForward();
    }*/
    
  }

  if (millis() - lastMeasure >= 1000) {
    float revsLeft = leftPulses / float(pulsesPerRev);
    float revsRight = rightPulses / float(pulsesPerRev);

    float distLeft = revsLeft * wheelCircumference;
    float distRight = revsRight * wheelCircumference;

    Serial.print("Bal sebesség: "); Serial.print(distLeft); Serial.print(" mm/s | ");
    Serial.print("Jobb sebesség: "); Serial.println(distRight);

    leftPulses = 0;
    rightPulses = 0;
    lastMeasure = millis();
  }
}

void moveForward(int motorleft, int motorright) {
  if (motorleft >= 0) {
    digitalWrite(IN1, HIGH); 
    digitalWrite(IN2, LOW);
  } else {
    digitalWrite(IN1, LOW); 
    digitalWrite(IN2, HIGH);
    motorleft = -motorleft;
  }
  if (motorright >= 0) {
    digitalWrite(IN3, HIGH); 
    digitalWrite(IN4, LOW);
  } else {
    digitalWrite(IN3, LOW); 
    digitalWrite(IN4, HIGH);
    motorright = -motorright;
  }
  analogWrite(ENA, motorleft); 
  analogWrite(ENB, motorright);
  /*analogWrite(ENA, motorleft); analogWrite(ENB, motorright);
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);*/
}

void stopMotors() {
  delay(500);
  digitalWrite(IN1, LOW); 
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); 
  digitalWrite(IN4, LOW);
  analogWrite(ENA, 0); 
  analogWrite(ENB, 0);
}

void turnRight() {
  /*analogWrite(ENA, 100); analogWrite(ENB, 100);
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);*/
  delay(700);
  moveForward(50, -50);
}

void tolat() {
  digitalWrite(IN1, LOW); 
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW); 
  digitalWrite(IN4, HIGH);
  delay(1000);
  analogWrite(ENA, -150); 
  analogWrite(ENB, -150);
}

void pidmozgas(double kozep, int maxS){
  input = kozep;
  pid.Compute();
  int motorleft = constrain(maxS - output, -50, 255);
  int motorright = constrain(maxS + output, -50, 255);
  moveForward(motorleft, motorright);
}

bool compareUID(byte *uid, byte *ref) {
  for (byte i = 0; i < 4; i++) {
    if (uid[i] != ref[i]) return false;
  }
  return true;
}

void countLeft() {
  leftPulses++;
}

void countRight() {
  rightPulses++;
}

double tavolsag(int sensor){
  int sensorertek = analogRead(sensor);
  double feszultseg = sensorertek * (5.0 / 1023.0);
  if (feszultseg == 0){
    return 0;
  }
  double tavolsagCM = (4 / (feszultseg - 0.042)) - 0.42;
  if (tavolsagCM < 2){
    return 0;
  }
  if (tavolsagCM > 45.0){
    return 45;
  }
  return tavolsagCM;
}

void tavolsagmeres() {
  tavolsagok[0] = tavolsag(IR1);
  tavolsagok[1] = tavolsag(IR2);
  tavolsagok[2] = tavolsag(IR3);
  tavolsagok[3] = tavolsag(IR4);
  if (tavolsagok[0] < 8 ) {
    Serial.println("Fal érzékelve!");
    tolat();
  }
  else if(tavolsagok[1] < 8){
    delay(200);
    stopMotors();
  }
  else {
    pidmozgas(tavolsagok[0], 200);
  }
}


