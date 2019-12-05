long randNumber;
int calis=false;
int r=false;
int s=0;
void setup() {
  Serial.begin(4800);
  delay(2000);
  for ( int p = 1; p <= 20; p++){
  Serial.println("LED="+String(p)+","+String(2));
  delay(50);
}
delay(200);
  for ( int k = 1; k <= 20; k++){
  Serial.println("LED="+String(k)+","+String(3));
  delay(50);
}
delay(500);
Serial.println("Başlatmak İçin RUN yazın !");
}
void loop() {
  tiki();
  String s1 = Serial.readString();
  if (s1.indexOf("RUN")>=0){
    calis=true;

    Serial.println("zaman basladi");
    }
}
void tiki()
{
  String s1 = Serial.readString();
  if (calis==true){
  randNumber=random(1,21);
  Serial.println("LED="+String(randNumber)+","+String(2));
  calis=false;
  }
  if (s1.indexOf("B="+String(randNumber))>=0){
  Serial.println("LED="+String(randNumber)+","+String(3));
  calis=true;
  s++;
    }
    if (s1.indexOf("STOP")>=0){
      Serial.println("zaman doldu");
      delay(1000);
      Serial.print("Toplam Skor=");Serial.print("   ");Serial.println(s);
      r=false;
      delay(1000);
      Serial.println("Tekrar Başlatmak İçin RUN yazın !");
    }
  }
