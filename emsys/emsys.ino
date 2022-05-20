void setup() {
  // put your setup code here, to run once:
  pinMode(2, OUTPUT); //Forward Right P
  pinMode(3, OUTPUT); //Forward Right N

  pinMode(5, OUTPUT); //Forward Left P
  pinMode(6, OUTPUT); //Forward Left N

  pinMode(9, OUTPUT); //Back Right P
  pinMode(10, OUTPUT); //Back Right N

  pinMode(11, OUTPUT); //Back Right P
  pinMode(12, OUTPUT); //Back Right N
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  if (Serial.available()){
    int traffic_state = Serial.parseInt();
    if(traffic_state==1){
      forward();
    }
    else{
      stopm();
    }
  }

}

void forward() {
  analogWrite(5, 100); analogWrite(6, 0);
  analogWrite(9, 200); analogWrite(10, 0);

  analogWrite(2, 100); analogWrite(3, 0);
  analogWrite(11, 200); analogWrite(12, 0);
}

void reverse() {
  analogWrite(5, 0); analogWrite(6, 100);
  analogWrite(9, 0); analogWrite(10, 100);

  analogWrite(2, 0); analogWrite(3, 100);
  analogWrite(11, 0); analogWrite(12, 100);
}

void stopm(){
  analogWrite(5, 0); analogWrite(6, 0);
  analogWrite(9, 0); analogWrite(10, 0);

  analogWrite(2, 0); analogWrite(3, 0);
  analogWrite(11, 0); analogWrite(12, 0);
}
