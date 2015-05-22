#define m1 9;
#define m2 10;
#define minThro=136; // full reverse
#define midThro 191; // neutral
#define maxThro 243; // full forward
#define revDif 55; // mid - min
#define forDif 52; // max - mid
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
void s(int del){ // brake and stop
  
}

void w(int v){
  
}

void setup() {
  Serial.begin(9600);
  pinMode(m1,OUTPUT);
  pinMode(m2,OUTPUT);
  
  pinMode(13,OUTPUT);
  
  sm(m1);
}

void loop() {
  
}
