#define m1 9
#define m2 10
#define minThro 136 // full reverse
#define midThro 191 // neutral
#define maxThro 243 // full forward
#define revDif 55 // mid - min
#define forDif 52 // max - mid
int cM;
int lastDir;

void sm(int m){
  cM=m;
}
void b(int thr){ // go backward at (thr)% throttle
  lastDir=-1;
  
  float f=(float)thr/100;
  analogWrite(cM,midThro-(revDif*f));
}
void f(int thr){ // go forward at (thr)% throttle
  lastDir=1;
  
  float f=(float)thr/100;
  analogWrite(cM,midThro-(forDif*f));
}
void n(){
  analogWrite(cM,midThro);
}
void s(){ // brake and stop
  if (lastDir > 0){
    b(50);
    delay(500);
    n();
    
  }else{
    f(50);
    delay(500);
    n();
  }
}

void setup() {
  Serial.begin(9600);
  pinMode(m1,OUTPUT);
  pinMode(m2,OUTPUT);
  
  pinMode(13,OUTPUT);
  
  sm(m1);
}

void loop() {
  digitalWrite(13,HIGH);
  f(75);
  delay(1000);
  digitalWrite(13,LOW);
  
  s();
  delay(1000);
  
  digitalWrite(13,HIGH);
  b(75);
  delay(1000);
  digitalWrite(13,LOW);
  
  s();
  delay(1000);
}
