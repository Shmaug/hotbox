#define m1 9
#define m2 10
#define minThro 136 // full reverse
#define midThro 191 // neutral
#define maxThro 243 // full forward
#define revDif 55 // mid - min
#define forDif 52 // max - mid

void b(int thr, int m){ // go backward at (thr)% throttle
  float f=(float)thr/100;
  analogWrite(m,midThro-(revDif*f));
}
void f(int thr, int m){ // go forward at (thr)% throttle
  float f=(float)thr/100;
  analogWrite(m,midThro+(forDif*f));
}
void n(int m){
  analogWrite(m,midThro);
}
void setup() {
  Serial.begin(9600);
  pinMode(m1,OUTPUT);
  pinMode(m2,OUTPUT);
  
  pinMode(13,OUTPUT);
  
  // arm
  digitalWrite(13,HIGH);
  n(m1);
  n(m2);
  delay(5000);
  digitalWrite(13,LOW);
}

void loop() {
  n(m1);
  n(m2);
  int del = random(1000,2000);
  Serial.print("1:\t");
  Serial.println(del);
  delay(del);
  
  int thr1=random(0,100);
  int thr2=random(0,100);
  if (random(-10,10)>0){
    f(m1,thr1);
    Serial.print("f1:\t");
    Serial.println(thr1);
  }else{
    b(m1,thr1);
    Serial.print("b1:\t");
    Serial.println(thr1);
  }
  if (random(-10,10)>0){
    f(m2,thr2);
    Serial.print("f2:\t");
    Serial.println(thr2);
  }else{
    b(m2,thr2);
    Serial.print("b2:\t");
    Serial.println(thr2);
  }
  
  digitalWrite(13,HIGH);
  del = random(1000,3000);
  Serial.print("2:\t");
  Serial.println(del);
  delay(del);
  digitalWrite(13,LOW);
}
