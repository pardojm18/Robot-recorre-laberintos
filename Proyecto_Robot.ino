class Robot{
  public:
      bool  sensorDelanteroIZQ();
      bool  sensorDelanteroDER();
      void  correccionDelantera();
      bool  sensorTrasero();
      int   analizarCelda();  //Adelante 1, Derecha 2, Izquierda 3, Vuelta 4
      void  movAdelante();
      void  movDerecha();
      void  movIzquierda();
      void  movVuelta();
      void  correccion();
      void  parar();
      int   numCeldasRecorridas() {return numCeldasRecorridas_;}
      int   numGirosDerecha()    {return numGirosDerecha_;}
      int   numGirosIzquierda() {return numGirosIzquierda_;}
      int   numVueltas()  {return numVueltas_;}
      void  meta();
   private:
      //Datos de interes sobre el recorrido
      int   numCeldasRecorridas_ = 0;
      int   numGirosDerecha_ = 0;
      int   numGirosIzquierda_ = 0;
      int   numVueltas_ = 0;
      //Constante de velocidad
      int   velocidad = 255;
      //Funciones internas
      bool  paredFre();
      bool  paredIzq();
      bool  paredDer();
      double  GetDistance(float v);
      //Pines
      //Motores
      const int mDerA = 5;
      const int mDerB = 6;
      const int mIzqA = 10;
      const int mIzqB = 9;
      //Sensores Luminicos
      const int CNY_Pin_DelanteroDER = A0;
      const int CNY_Pin_DelanteroIZQ = A1;
      const int CNY_Pin_Trasero = A5;
      const int NEGRO =750;
      //Sensores distancia
      const int pinSeDelantero = 8;
      const int pinSeIzquierda = A3;
      const int pinSeDerecha = A4;
      //Bluetooth
      const int RXBT = 0;
      const int TXBT = 1;
      //LEDS
      const int led1 = 12;
      const int led2 = 13;
      const int led3 = 11;
};

bool corregir = true;


//Calcular distancia sensores laterales
double Robot::GetDistance(float v){
  if(v<0.5 || v>2.7){
    return 0;
  }  
  return (12/(v-0.1))-0.42;
}

//Comprueba si hay pared, true = hay pared, false = espacio libre.
bool Robot::paredFre(){

  float distancia;
  unsigned long time_bounce;
  
  pinMode(pinSeDelantero, OUTPUT);
  digitalWrite(pinSeDelantero, LOW);
  delayMicroseconds(5);

  digitalWrite(pinSeDelantero, HIGH);
  delayMicroseconds(10);
  pinMode(pinSeDelantero,INPUT);
  digitalWrite(pinSeDelantero, LOW);

  time_bounce = pulseIn(pinSeDelantero, HIGH);

  distancia = 0.017 * time_bounce; //Formula distancia
  
  return distancia < 20;
}

//Detector pared derecha

bool Robot::paredDer(){
  float ResolutionADC = 0.00488;
  double Value_Pin = analogRead(pinSeDerecha);
  double Voltage = Value_Pin*ResolutionADC;
  double distancia = GetDistance(Voltage);

  return distancia < 15 && distancia > 5; 
}

//Detector pared izquierda
bool Robot::paredIzq(){
  float ResolutionADC = 0.00488;
  double Value_Pin = analogRead(pinSeIzquierda);
  double Voltage = Value_Pin*ResolutionADC;
  double distancia = GetDistance(Voltage);

  return distancia < 15 && distancia > 5;
}

//Detector de ambos sensores delanteros
bool Robot::sensorDelanteroIZQ(){
  int ValorSensorDelanteroIZQ = analogRead(CNY_Pin_DelanteroIZQ);
  return ValorSensorDelanteroIZQ > NEGRO + 100;
}

bool Robot::sensorDelanteroDER(){
  int ValorSensorDelanteroDER = analogRead(CNY_Pin_DelanteroDER);
  return ValorSensorDelanteroDER > NEGRO;
}

//Detector de sensores trasero
bool Robot::sensorTrasero(){
  int  ValorSensorTrasero = analogRead(CNY_Pin_Trasero);

  if (ValorSensorTrasero > NEGRO)
    numCeldasRecorridas_++;

  return ValorSensorTrasero > NEGRO;
}

//Analiza celda en función de las paredes.
int   Robot::analizarCelda(){
    if(paredDer()){
      if(paredFre()){
        if(paredIzq()){
          return 4;
        }else{
          return 3;
        }
      }else{
        return 1;
      }
    }else{
      return 2;
    }
}

//----------------------
//------MOVIMIENTOS-----
//----------------------
void  Robot::movAdelante(){
    analogWrite(mDerA, 150);
    analogWrite(mDerB, 0);
    analogWrite(mIzqA, 0);
    analogWrite(mIzqB, 150);
}

void Robot::movDerecha(){
  
  analogWrite(mDerA, 0);
  analogWrite(mDerB, 0);
  analogWrite(mIzqA, 0);
  analogWrite(mIzqB, 255);
  delay(1250);
  corregir = true;

  numGirosDerecha_++;

  //correccionDelantera();
}

void Robot::movIzquierda(){  
  analogWrite(mDerA, 255);
  analogWrite(mDerB, 0);
  analogWrite(mIzqA, 0);
  analogWrite(mIzqB, 0);
  delay(1250);
  corregir = true;

  numGirosIzquierda_++;
  
  //correccionDelantera();
}

void Robot::movVuelta(){

  analogWrite(mDerA, 150);
  analogWrite(mDerB, 0);
  analogWrite(mIzqA, 0);
  analogWrite(mIzqB, 150);

  delay(300);
  
  analogWrite(mDerA, velocidad/4);
  analogWrite(mDerB, 0);
  analogWrite(mIzqA, velocidad/4);
  analogWrite(mIzqB, 0);
  delay(3200);

  analogWrite(mDerA, 150);
  analogWrite(mDerB, 0);
  analogWrite(mIzqA, 0);
  analogWrite(mIzqB, 150);

  
  numVueltas_++;
}

void Robot::parar(){
  analogWrite(mDerA, 0);
  analogWrite(mDerB, 0);
  analogWrite(mIzqA, 0);
  analogWrite(mIzqB, 0);
}

void Robot::meta(){
  if(sensorDelanteroDER() && sensorTrasero()){
    pinMode(led1, OUTPUT);
    pinMode(led2, OUTPUT);
    pinMode(led3, OUTPUT);
  }
  while(true){
    digitalWrite(led1, HIGH);
    delay(100);
    digitalWrite(led1, LOW);
    delay(100);
    digitalWrite(led2, HIGH);
    delay(100);
    digitalWrite(led2, LOW);
    delay(100);
    digitalWrite(led3, HIGH);
    delay(100);
    digitalWrite(led3, LOW);
    delay(100);
  }
}

void Robot::correccionDelantera(){

  if(sensorDelanteroDER()){
    while(!sensorDelanteroIZQ()){
      analogWrite(mDerA, 0);
      analogWrite(mDerB, 0);
      analogWrite(mIzqA, 0);
      analogWrite(mIzqB, 100);
    }    
    corregir = false;
  }
  if(sensorDelanteroIZQ()){
    while(!sensorDelanteroDER()){
      analogWrite(mDerA, 100);
      analogWrite(mDerB, 0);
      analogWrite(mIzqA, 0);
      analogWrite(mIzqB, 0);
    }
    corregir = false;
  }else{
    analogWrite(mDerA, 150);
    analogWrite(mDerB, 0);
    analogWrite(mIzqA, 0);
    analogWrite(mIzqB, 150);
  }
  
}

void setup() {
  Serial1.begin(9600);
}

bool final = false;

void loop() {
  Robot robot;
  int movimiento;
  int empezar = 0;

  delay(5000);

  Serial1.println("Hola, soy Leni. El robot que va a recorrer este laberinto.");

  /*while(empezar != 288)
    empezar = Serial1.read();*/

  
  
  do{

      robot.movAdelante();
      delay(100);

      corregir = true;
      
      while(!robot.sensorTrasero()){
        if(corregir)
          robot.correccionDelantera();
        else
          robot.movAdelante();
      }

      corregir = true;
      
      robot.parar();

      if(robot.sensorDelanteroIZQ() and robot.sensorDelanteroDER())
        final = true;
      else{
        movimiento = robot.analizarCelda();

        switch(movimiento){
          case 1:
              Serial1.println("Leni ha detectado una pared a su derecha, por lo tanto sigue adelante");
              robot.movAdelante();
              delay(200);
              break;
          case 2:
              Serial1.println("Leni no ha detectado una pared a la derecha, por tanto sigue su movimiento por defecto, girar a la derecha");
              robot.movDerecha();   
              break;
          case 3:
              Serial1.println("Leni ha detectado paredes tanto a su derecha, como en frente por tanto gira a la izquierda");
              robot.movIzquierda(); 
              break;
          case 4:
              Serial1.println("Leni se ha encontrado acorralado por todos lados, por tanto decide dar la vuelta sobre si mismo, para volver sobre sus pasos.");
              robot.movVuelta();   
              break;
        }
      }

      Serial1.println(robot.numCeldasRecorridas());
            
  } while(!final);
    
  Serial1.println("Leni ha terminado el recorrido del laberinto y estos son los resultados: ");

  Serial1.print("Número de celdas recorridas: ");
  Serial1.println(robot.numCeldasRecorridas());

  Serial1.print("Número de giros a la derecha: ");
  Serial1.println(robot.numGirosDerecha());

  Serial1.print("Número de giros a la izquierda: ");
  Serial1.println(robot.numGirosIzquierda());

  Serial1.print("Número de vueltas dadas: ");
  Serial1.println(robot.numVueltas());

  robot.meta();
  delay(50000);  
  
}
