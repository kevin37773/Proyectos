//Planta de temperatura con control Fuzzy
//Librerias para sensor de temperatura y interrupcion
#include <DHT.h>
#include <TimerOne.h>

#define DHTYPE DHT11
#define DHTPIN 9

DHT dht(DHTPIN,DHTYPE);

//#define sensor_temp1 A1  //LM35
//#define sensor_temp2 A2 //LM35
//#define sensor_corr A2  //circuito
#define salida_voltaje 3  //pwm a transistor TIP31
#define led_advertencia 13 //led indicador de alta temperatura

//VARIABLES GLOBALES
int cont=0;
float T1,H1=0.0;
float w=0;
float e[2]={0,0};
float u[2]={0,0};
int kU = sizeof(u)/sizeof(float)-1;
int kE = sizeof(e)/sizeof(float)-1;
float universo_e = 70, universo_de = 0.5, universo_u = 2.5;
float der;
float CF_u;
int setPoint;

//tiempo de muestreo
int Ts = 8;

bool interfaz_externa = true;

void Muestreo(void){
   actualizar_datos(u,kU);
   actualizar_datos(e,kE);
   //calculo del error
   e[kE] = w - T1;
   //limitamos el error para que este dentro del universo
   if(e[kE]<-1*universo_e+0.1)
      e[kE]=-1*universo_e+0.1;
   if(e[kE]>universo_e-0.1)
      e[kE]=universo_e-0.1;
   //calculo de la derivada del error 
   der=(e[kE]-e[kE-1])/Ts;
   //limitamos la derivada del error para que este dentro del universo
   if(der<-1*universo_de+0.01)
      der=-1*universo_de+0.01;
   if(der>universo_de-0.01)
      der=universo_de-0.01;
   //calculo de la señal de control Fuzzy
   CF_u=control_fuzzy(e[kE],der,universo_e,universo_de,universo_u);
   //integramos la señal de control
   u[kU]=CF_u*4+u[kU-1];  //**************************** AJUSTAR
   //limitamos la señal de salida
   if(u[kU]>=100.0)
      u[kU]=100.0;
   if(u[kU]<=0.0)
      u[kU]=0.0;
   H1 = u[kU];
   //aplicamos la señal de control en el PWM
   analogWrite(salida_voltaje,map(H1,0,100,0,255));
}

void setup(){
  //configuracion de pines de entrada y salida
  dht.begin();
  Timer1.initialize(8000000); //interrupcion cada 8 segundos
  Timer1.attachInterrupt(Muestreo);
  pinMode(led_advertencia,OUTPUT);
  digitalWrite(led_advertencia,LOW);

  //configuracion del puerto serial para transmision y recepcion de datos
  Serial.begin(9600);
  analogWrite(salida_voltaje,0);
}

void loop(){
  String dato,degC;
  int i,ini,fin;
  T1 = dht.readTemperature();
  if(T1>30.0)
    digitalWrite(led_advertencia,HIGH);
  else
    digitalWrite(led_advertencia,LOW);
  //recepcion de datos por puerto serial
  if(Serial.available()){
    dato=Serial.readString();
    for(i=0;i<10;i++){
      if(dato[i]=='S'){
        ini=i+1;
        i=10;
      }
    }
    for(i=ini;i<10;i++){
      if(dato[i]=='$'){
        fin=i;
        i=10;
      }
    }
    degC = dato.substring(ini,fin); //la señal escalon
    w = degC.toDouble();
    //analogWrite(salida_voltaje,map(H1,0,100,0,255));
  }
  if(interfaz_externa){
    Serial.print("I");
    Serial.print(T1);
    Serial.print("F");
    Serial.print("C");
    Serial.print(H1);
    Serial.print("R");
  }
  else{
    Serial.println("temperatura:(C),potencia(%),setPoint");
    Serial.print(T1);
    Serial.print(",");
    Serial.print(H1);
    Serial.print(",");
    Serial.println(w);
  }
  delay(1000);
}

//FUNCIONES Y PROCEDIMIENTOS:
//FUNCION 01: ACTUALIZAR DATOS 
void actualizar_datos(float v[], int k){
  int i;
  for(i=1;i<=k;i++){
    v[i-1]=v[i];    
  }
}

//FUNCION 02: TIPO MENBRESIA TRAPEZOIDAL
float trapezoidal(float x, float a, float b, float c, float d){
  float y;
  y=max(min(min((x-a)/(b-a),(d-x)/(d-c)),1),0);
  return(y);
}

//FUNCION 03: TIPO MENBRESIA TRIANGULAR
float triangular(float x, float a, float b, float c){
  float y;
  y=max(min((x-a)/(b-a),(c-x)/(c-b)),0);
  return(y);
}

//FUNCION 04: CONTROLADOR FUZZY
float control_fuzzy(float e,float de,float uni_e,float uni_de,float uni_u){
  float eNG,eNP,eZ,ePP,ePG; //valores linguisticos de entrada 01: variable ERROR
  float edNG,edNP,edZ,edPP,edPG; //valores linguisticos de entrada 02: variable derivada de Error
  float uNG=0,uNP=0,uZ=0,uPP=0,uPG=0;//salida parcial
  float vNG,vNP,vZ,vPP,vPG; //valores linguisticos de salida 01: variable Voltaje-PWM

  float universo[3]; //vector de 3 posiciones para rango de variable de salida
  float R[25];  //variable vector para REGLAS DIFUSAS
  float maxU=0; //maximos de todas las salidas
  
  float sumfx=0,sumUX=0;  //variables auxiliares para calculo de centroide
  float cg; //variable defuzificada
  
  int i; //variable contador
  float k;

  universo[0]=-1*uni_u;       //limite inferior -2.5
  universo[1]=(uni_u*2)/200;  //saltos de 0.05
  universo[2]=1*uni_u;        //limite superior 2.5

  //01 PROCESO DE FUSIFICACION .........................................................................................................................
  eNG=triangular(e,-1*uni_e,-1*0.66*uni_e,-1*0.33*uni_e);
  eNP=triangular(e,-1*0.66*uni_e,-1*0.33*uni_e,0);
  eZ =triangular(e,-1*0.33*uni_e,0,0.33*uni_e);
  ePP=triangular(e,0,0.33*uni_e,0.66*uni_e);
  ePG=triangular(e,0.33*uni_e,0.66*uni_e,1*uni_e);

  edNG=triangular(de,-1*uni_de,-1*0.66*uni_de,-1*0.33*uni_de);
  edNP=triangular(de,-1*0.66*uni_de,-1*0.33*uni_de,0);
  edZ =triangular(de,-1*0.33*uni_de,0,0.33*uni_de);
  edPP=triangular(de,0,0.33*uni_de,0.66*uni_de);
  edPG=triangular(de,0.33*uni_de,0.66*uni_de,uni_de);

  //02 PROCESO DE INFERENCIA ...........................................................................................................................
  //Reglas difusas
  R[0]=min(eNG,edNG);
  R[1]=min(eNG,edNP);
  R[2]=min(eNG,edZ);
  R[3]=min(eNG,edPP);
  R[4]=min(eNG,edPG);
  R[5]=min(eNP,edNG);
  R[6]=min(eNP,edNP);
  R[7]=min(eNP,edZ);
  R[8]=min(eNP,edPP);
  R[9]=min(eNP,edPG);
  R[10]=min(eZ,edNG);
  R[11]=min(eZ,edNP);
  R[12]=min(eZ,edZ);
  R[13]=min(eZ,edPP);
  R[14]=min(eZ,edPG);
  R[15]=min(ePP,edNG);
  R[16]=min(ePP,edNP);
  R[17]=min(ePP,edZ);
  R[18]=min(ePP,edPP);
  R[19]=min(ePP,edPG);
  R[20]=min(ePG,edNG);
  R[21]=min(ePG,edNP);
  R[22]=min(ePG,edZ);
  R[23]=min(ePG,edPP);
  R[24]=min(ePG,edPG);

  //reglas salida negativo grande
  uNG = max(uNG,R[0]);
  uNG = max(uNG,R[1]);
  uNG = max(uNG,R[2]);
  uNG = max(uNG,R[5]);
  uNG = max(uNG,R[10]);

  //Reglas salida negativo pequeño
  uNP = max(uNP,R[3]);
  uNP = max(uNP,R[6]);
  uNP = max(uNP,R[7]);
  uNP = max(uNP,R[11]);
  uNP = max(uNP,R[15]);

  //Reglas salida cero
  uZ = max(uZ,R[4]);
  uZ = max(uZ,R[8]);
  uZ = max(uZ,R[12]);
  uZ = max(uZ,R[16]);
  uZ = max(uZ,R[21]);

  //Reglas salida positivo pequeño
  uPP = max(uPP,R[9]);
  uPP = max(uPP,R[13]);
  uPP = max(uPP,R[17]);
  uPP = max(uPP,R[18]);
  uPP = max(uPP,R[21]);

  //Reglas salida positivo grande
  uPG = max(uPG,R[14]);
  uPG = max(uPG,R[19]);
  uPG = max(uPG,R[22]);
  uPG = max(uPG,R[23]);
  uPG = max(uPG,R[24]);

  //03 PROCESO DE DEFUSIFICACION metodo-centroide .......................................................................................................
  //salida del sistema difuso
  k=universo[0];  //-5
  while(k<=universo[2]) //5
  {
    vNG=triangular(k,-1*uni_u,-1*0.66*uni_u,-1*0.33*uni_u);
    vNP=triangular(k,-1*0.66*uni_u,-1*0.33*uni_u,0);
    vZ =triangular(k,-1*0.33*uni_u,0,0.33*uni_u);
    vPP=triangular(k,0,0.33*uni_u,0.66*uni_u);
    vPG=triangular(k,0.33*uni_u,0.66*uni_u,uni_u);

    //limitar las salidas con los puntos maximos de agregacion
    if(vNG>=uNG)
      vNG=uNG;
    if(vNP>=uNP)
      vNP=uNP;
    if(vZ>=uZ)
      vZ=uZ;
    if(vPP>=uPP)
      vPP=uPP;
    if(vPG>=uPG)
      vPG=uPG;

    //agrupamos los valores maximos hallados de las 5 funciones de menbresia
    maxU = max(maxU,vNG);
    maxU = max(maxU,vNP);
    maxU = max(maxU,vZ);
    maxU = max(maxU,vPP);
    maxU = max(maxU,vPG);

    //guardamos en el vector Fx
    sumUX = sumUX + maxU*k;
    sumfx = sumfx + maxU;

    k=k+universo[1];
    maxU=0;
  }
  cg=sumUX/sumfx;
  return(cg);     
}

void parpadeo(){
  digitalWrite(led_advertencia,HIGH);
  delay(300);
  digitalWrite(led_advertencia,LOW);
  delay(300);
}
