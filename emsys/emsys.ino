int motorSpeed = 200;
void setup()
{
  // put your setup code here, to run once:
  pinMode(8, OUTPUT); // Forward Right P
  pinMode(9, OUTPUT); // Forward Right N

  pinMode(11, OUTPUT); // Forward Left P
  pinMode(12, OUTPUT); // Forward Left N

  //  pinMode(5, OUTPUT); //Back Right P
  //  pinMode(6, OUTPUT); //Back Right N

  // pinMode(2, OUTPUT); //Back Right P
  // pinMode(3, OUTPUT); //Back Right N
  Serial.begin(9600);
  Serial.setTimeout(1);
}

void loop()
{
  // put your main code here, to run repeatedly:
  if (Serial.available())
  {
    int traffic_state = Serial.parseInt();
    Serial.println(traffic_state);
    if (traffic_state == 1)
    {
      forward();
      delay(200);
      // traffic_state=0;
    }
    else
    {
      stopm();
    }
  }
}

void forward()
{
  analogWrite(11, motorSpeed);
  analogWrite(12, 0);
  analogWrite(9, motorSpeed);
  analogWrite(8, 0);

  // analogWrite(5, 170); analogWrite(6, 0);
  //  analogWrite(11, 100); analogWrite(12, 0);
}

void reverse()
{
  analogWrite(5, 0);
  analogWrite(6, 100);
  analogWrite(9, 0);
  analogWrite(10, 100);

  // analogWrite(2, 0); analogWrite(3, 100);
  // analogWrite(11, 0); analogWrite(12, 100);
}

void stopm()
{
  analogWrite(11, 0);
  analogWrite(12, 0);
  analogWrite(8, 0);
  analogWrite(9, 0);

  // analogWrite(2, 0); analogWrite(3, 0);
  //  analogWrite(11, 0); analogWrite(12, 0);
}