int motor1Pin = 3;
int motor2Pin = 4;
int motor3Pin = 7;
int motor4Pin = 8;
int Pulse = 5;

int encoderPin = 2;
int encoderPos = 0;

char order;
int s;
void setup() {
  // set all the other pins you're using as outputs:
  Serial.begin(9600);
  pinMode(motor1Pin, OUTPUT);
  pinMode(motor2Pin, OUTPUT);
  pinMode(motor3Pin, OUTPUT);
  pinMode(motor4Pin, OUTPUT);
  pinMode(Pulse, OUTPUT);
  pinMode (encoderPin,INPUT);
  attachInterrupt(1, CountA, CHANGE);
}
void loop()
{
if (Serial.available() > 0) 
   {
               // read the incoming byte:
      order = Serial.read();
      switch(order)
      {
       case 'f':
       {
          forward();
          break; 
       }
       case 'c':
       {
         rotate_clock();
         break;
         
       }
       case 'a':
       {
         rotate_anticlock();
         break;
       }    
       case 's':
       {
          Halt();
          break; 
       }  
   }
   delay(10);
   Serial.write(order);
   }
   
   //encoderPos = encoderPos/2;
   //Serial.print (encoderPos); 
   //Serial.print ('\n'); 
}

void rotate_clock()
{
   //analogWrite(Pulse, a);
   digitalWrite(Pulse, HIGH);
   digitalWrite(motor1Pin, LOW);    
   digitalWrite(motor2Pin, LOW);
   digitalWrite(motor3Pin, HIGH);   
   digitalWrite(motor4Pin, HIGH);
}
void rotate_anticlock()
{
  //analogWrite(Pulse, a);
   digitalWrite(Pulse, HIGH);
   digitalWrite(motor1Pin, HIGH);   
   digitalWrite(motor2Pin, HIGH);
   digitalWrite(motor3Pin, LOW);   
   digitalWrite(motor4Pin, LOW);
}
void forward()
{
  //analogWrite(Pulse, a);
  digitalWrite(Pulse, HIGH);
  digitalWrite(motor1Pin, HIGH);  
  digitalWrite(motor2Pin, LOW);
  digitalWrite(motor3Pin, HIGH); 
  digitalWrite(motor4Pin, LOW);
}
void Halt()
{
  digitalWrite(Pulse, LOW);
}
void CountA()
{
   encoderPos++;  
}
