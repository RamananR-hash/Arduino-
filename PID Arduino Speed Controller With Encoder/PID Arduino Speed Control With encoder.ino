# include<SoftwareSerial.h>
SoftwareSerial SERIAL_SET(10, 11); //RX TX

const int MOTOR_PIN = 5; //1st motor

# define ENC_IN 3

# define led 13

float elapsedTime, time, timePrev;//pid I/O




///////////////////DIGITAL/////////////////////////
int stopval = 124;
int FeedBack_RPM = 0;
int Set_RPM = 0;
float Theta,  RPM_d;
float Theta_prev = 0;
int dt;
unsigned long t;
unsigned long t_prev = 0;
volatile unsigned long count = 0;
unsigned long count_prev = 0;
////////////////////PID_INITILIZATION//////////////////////
float PID, pwmLeft, error, previous_error;
float pid_p = 0;
float pid_i = 0;
float pid_d = 0;
///////////////////////////////////////////////
/////////////////PID CONSTANTS/////////////////
double kp = 0.80; //0.80;// 1.90;//1.825; //3.55
double ki = 0.95; //0.95;//0.85;// 0.595; //0.003
double kd = 1.32; //1.32;//1.40;//1.0055; //2.05
///////////////////////////////////////////////
volatile long encoderValue = 0;
long previousMillis = 0;
long currentMillis = 0;



void ISR_EncoderA() {
  bool PinA = digitalRead(ENC_IN);
  if (PinA == HIGH) {
    encoderValue++;

  }

}

void setup() {
  pinMode(ENC_IN, INPUT_PULLUP);
  pinMode(MOTOR_PIN, OUTPUT);
  pinMode(led,OUTPUT);
  analogWrite(MOTOR_PIN, stopval);
  attachInterrupt(digitalPinToInterrupt(ENC_IN), ISR_EncoderA, RISING);
  /////////////////////////Serial_initize////////////////
  Serial.begin(9600);
  SERIAL_SET.begin(9600);
  digitalWrite(led,LOW);
  ///////////////////////////////////////////////////////

  /////////////////////////TIMER_INIT////////////////////////////////////
  cli();
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = 0;
  OCR1A = 12499; //Prescaler = 64
  TCCR1B |= (1 << WGM12);
  TCCR1B |= (1 << CS11 | 1 << CS10);
  TIMSK1 |= (1 << OCIE1A);
  sei();
  /////////////////////////////////////////////////////////////////////////
}

void loop() {

  while (SERIAL_SET.available() == 0) {}

  Set_RPM = SERIAL_SET.parseInt();


  //Serial.println(Set_RPM);
  if (count > count_prev) {//encoder
    digitalWrite(led,HIGH);
    t = millis();
    Theta = (encoderValue /  9031.0);
    dt = (t - t_prev);
    FeedBack_RPM = (Theta - Theta_prev) / (dt / 1000.0) * 60;
    error = (Set_RPM - FeedBack_RPM );
    //PID Propertional
    pid_p = kp * error;
    //PID integral
    if (-3<error< 3)
    {
      pid_i = pid_i + (ki * error);
    }
    //PID Derivative
    pid_d = kd * ((error - previous_error) / dt); // timeoriginal);
    PID = pid_p + pid_i + pid_d;
    Serial.print("Set_RPM");
    Serial.print("\t");
    Serial.print(Set_RPM);
    Serial.print("\t");
    Serial.print("FeedBack_RPM");
    Serial.print("\t");
    Serial.println(FeedBack_RPM);
    /*Serial.print("\t");
      Serial.print("Error");
      Serial.print("\t");
      Serial.print(error);
      Serial.print("\t");
      Serial.print("PID");
      Serial.print("\t");
      Serial.print(PID);
      Serial.print("\t");
      Serial.print("Pwm Value");
      Serial.print("\t");
      Serial.print(pwmLeft);
      Serial.print("\t");
      Serial.println("PWM");*/
    if (PID > 0 && PID < 180) //there is only one & i changed to &&
    {
      pwmLeft = map(PID, 0, 180, 139, 255);
      analogWrite(MOTOR_PIN, pwmLeft);
    }
    else {
      if (PID > 180) {
        analogWrite(MOTOR_PIN, 255);
      }
      else {
        analogWrite(MOTOR_PIN, 139);
      }
    }
    previous_error = error; //Remember to store the previous error.
    Theta_prev = Theta;
    count_prev = count;
    t_prev = t;
  }
}

ISR(TIMER1_COMPA_vect) {
  count++;
}
