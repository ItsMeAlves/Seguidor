/* Socorro.ino
 * Autor - Gabriel Alves de Lima, Ruamberg Vasconcelos, Arthur Diego
 * Algoritmo de seguidor de linha- Último caso
 * Configuração utilizada para o arduino MEGA
 * Última alteração:
 * Alteração no cálculo do erro
 */

//CONSTANTES DE SAÍDA DE DEBUG
#define DEBUG0 1 //DEBUG DOS SENSORES
#define DEBUG1 0 //DEBUG DOS MOTORES
#define LIMIAR_SERIAL 1000 // Para nao imprimir na serial a cada ciclo 

//CONSTANTES DE CONTROLE
#define KP 14.2
#define KI 0.001//0.0001
#define KD 5//2
#define SET_POINT 0
#define INITIAL_SPEED 35 
#define MAX_SPEED 82
#define MIN_SPEED INITIAL_SPEED
#define MAX_REVERSE MAX_SPEED
#define MAX_ERROR MAX_SPEED/KP
#define EE A12
#define ED A11

//CONSTANTES DE AUXÍLIO 
#define CORRECTION(a,b,c) a+b+c
#define NUM_SENSORS 7
#define CENTRAL NUM_SENSORS/2
#define LIMITE 500

//COR DA LINHA
#define BRANCA true
#define PRETA false

//CONSTANTES DOS MOTORES
#define ME1 9
#define ME2 8
#define MD2 10
#define MD1 11

/******* CONFIGURAÇÃO PARA OS SENSORES *********/

	//Valores que representam os sensores em ordem nas portas do arduino
	const int line[NUM_SENSORS] = {A3, A2, A1, A0, A6, A7, A8};
	//Valores atribuídos para cara sensor em ordem (o peso de cada um)
	//Para esquema 1, -9, -7, -5, -3, -1, 1, 3, 5, 7, 9; 
	//Para esquema 2, 0, 10, 20, 30, 40, 50, 60, 70, 80, 90;
	//int error[NUM_SENSORS] = {-57, -45, -34, -24, -14, 0, 14, 24, 34, 45, 57};
	const int error[NUM_SENSORS] = {-6, -3, -1, 0, 1, 3, 6};
	bool lineColor = BRANCA;
	int blackLimit = 0, inValue = 0;

/******** CONFIGURAÇÃO PARA OS MOTORES **********/
	
	int cont = 0; // Variavel temporaria para contar para a impressao da velocidade do motor
	bool debugSerial = false;

/******** CONFIGURAÇÃO PARA O CONTROLE **********/
	
	int err = 0; //Variável para armazenar o valor do erro
	int iErr = 0, prevErr = 0; //Respectivamente, erro integral e erro anterior
	//Valores a serem testados
	int baseSpeed = INITIAL_SPEED;    //Velocidade base do robô
	int countFinal = 0;
	unsigned long timeFlag = 0; //Sinal de tempo para aceleração
	unsigned long T00 = 0;
	unsigned long T01 = 0;
	long T0;
	int contAccel = 0;
	bool flagAccel = false; 
	int contador1 = 0; 
	bool flagDeParada = false;

void setup() {
	//delay(1000);
	Serial.begin(9600);

	//Definição das portas dos sensores como input e dos motores como output
	sensorsSetup();//faz o ajuste da leitura dos sensores
    T00 = millis();
    T01 = T00;
    //Definição dos motores como saída
	pinMode(ME1, OUTPUT);
	pinMode(ME2, OUTPUT); 
	pinMode(MD1, OUTPUT);
	pinMode(MD2, OUTPUT);
}

void loop() {
	checkFinal();
	
	//Cálculo do erro: ponto atual subtraído do ponto de desejo
    err = getCurrentPoint() - SET_POINT;
	
	//Ajuste Da Velocidade Base ******************** temporario *************************

	//flagAccel = contador(&contAccel , 50);

    if(debugSerial){
        Serial.print("Erro = ");
        Serial.println(err);
        Serial.print("ED: ");
        Serial.println(analogRead(ED));
        Serial.print("EE: ");
        Serial.println(analogRead(EE));
    } 

	// ******************************************** temporario ***************************
    accel(1, 50);
        
	//Motor na velocidade base (baseSpeed) +- CORRECTION(p,i,d)
    if(err >= 0 ) {
        leftMotorControl((baseSpeed + (KP * err + KI * iErr + KD * (err - prevErr))));
       	rightMotorControl((baseSpeed - 1.3*(KP * err + KI * iErr + KD * (err - prevErr))));
    }
    else {
        leftMotorControl((baseSpeed + 1.3*(KP * err + KI * iErr + KD * (err - prevErr))));
       	rightMotorControl((baseSpeed - (KP * err + KI * iErr + KD * (err - prevErr))));
    }
	//Atualiza o erro anterior e soma o erro no erro integral
	prevErr = err;
	iErr += err;

	//Previne windup (soma excessiva potencialmente perigosa do erro integral)
	preventWindup();

	debugSerial = contador(&cont , LIMIAR_SERIAL);
	
	//Imprime tempo na saída serial
	//Serial.print("Tempo: ");
	//Serial.println(millis() - T0);
}

//Previne windup (soma excessiva potencialmente perigosa do erro integral)
void preventWindup() {
	if(iErr >= MAX_SPEED)
	{
		iErr = iErr/3;
	}
	else if(err == 0) {
		iErr = 0;
	}
}

//Cálculo do ponto atual do robô.
//Pela média dos pesos dos sensores na linha pela quantidade de sensores na linha
int getCurrentPoint() {
	int i = 0;
	char mark = 0;
	int numerador = 0; //numerador: somatório dos pesos de cada sensor
	int denominador = 0; //denominador: quantidade de sensores na linha
	for(i = 0; i < NUM_SENSORS; i++) {
		//Vê se o sensor está fora da linha
		if(debugSerial){
			Serial.print(i);
			Serial.print(" = ");
		}
		if(getline(line[i])){
			if(i == CENTRAL) {
				mark = 1;
			}
			denominador++;
			numerador += error[i];
		}
	} 
	if(denominador == 0) {
		//denominador = 1;
        return prevErr;
	}
	if(mark == 1 && denominador > 1) {
		denominador -= 1;
	}

	//Cálculo da média
	return numerador/denominador;
}

bool getline(int analogPort){
	int readValue = analogRead(analogPort);

	//readValue = 1023 - readValue;

	if(debugSerial)Serial.println(readValue);

	if( (lineColor == PRETA  && readValue > blackLimit) || 
		(lineColor == BRANCA && readValue < blackLimit)    ){
		if(debugSerial){
			Serial.println("linha");
			Serial.println(blackLimit);
		}
		return 1;
	}
	else
		return 0;
}

//(Des)aceleração com em unidade v/ms²
void accel(int accelFactor, int timeInterval) {
	if(millis() - timeFlag >= timeInterval) {
		baseSpeed += accelFactor;
		if(baseSpeed + err > MAX_SPEED - err) {
			baseSpeed = 60 - abs(err)*3;
		}
		timeFlag = millis();
	}
}


//Ativa o motor direito com intensidade 'speed'
void rightMotorControl(float speed) {
	float newSpeedRightMotor = speed;

	if(speed > MAX_SPEED) {
		newSpeedRightMotor = MAX_SPEED;
	} else if (speed < -MAX_REVERSE) {
		newSpeedRightMotor = -MAX_REVERSE;
	}

 	if(speed >= 0) {
		analogWrite(MD1, newSpeedRightMotor);
		digitalWrite(MD2, LOW);
		if(debugSerial){
			Serial.print("[");
			Serial.print(newSpeedRightMotor);
			Serial.println("]\n");
		}
	}
	else {
		analogWrite(MD2, -newSpeedRightMotor);
		digitalWrite(MD1, LOW);
		if(debugSerial){
			Serial.print("[-");
			Serial.print(newSpeedRightMotor);
			Serial.println("]\n");
		}
	}
}

//Ativa o motor esquerdo com intensidade 'speed'
void leftMotorControl(float speed) {
	float newSpeedLeftMotor = speed;

	if(speed > MAX_SPEED) {
		newSpeedLeftMotor = MAX_SPEED;
	} else if(speed < -MAX_REVERSE) {
		newSpeedLeftMotor = -MAX_REVERSE;
	}

	if(speed >= 0) {
		analogWrite(ME1, newSpeedLeftMotor);
		digitalWrite(ME2, LOW);
		if(debugSerial){
			Serial.print("\n[");
			Serial.print(newSpeedLeftMotor);
			Serial.print("].");
		}
	}
	else {
		analogWrite(ME2, -newSpeedLeftMotor);
		digitalWrite(ME1, LOW);
		if(debugSerial){
			Serial.print("\n[-");
			Serial.print(newSpeedLeftMotor);
			Serial.print("].");
		}
	}
}

void sensorsSetup(){
	int centralPin = (NUM_SENSORS -1)/2; 
    int outValue = 0;
    int aux = 0;

	for(int i = 0; i < NUM_SENSORS; i++) {
		pinMode(line[i], INPUT);
                
        if(line[i] != centralPin){
            outValue += analogRead(line[i]);
        }
	}

	aux = min(analogRead(line[centralPin]), analogRead(line[centralPin + 1]));
    inValue = min(analogRead(line[centralPin - 1]), aux);
    outValue /= (NUM_SENSORS - 1);

    if(inValue < outValue)
    	lineColor = BRANCA;
    else 
    	lineColor = PRETA;

  	blackLimit = ((inValue + outValue)/2)-100;

  	//if(DEBUG0){
	//  	Serial.print("blackLimit = ");
	//  	Serial.println(blackLimit);
   	//}

}


//Checa a linha de chegada, verificando os estados dos sensores das extremidades
void checkFinal() {
	//Lembrar do retao
	if(millis() - T00 > 5000) {
		if(getline(line[ED]) && isCentral()) {
			flagDeParada = true;	
			digitalWrite(13, HIGH);		
		}
		else {
			digitalWrite(13, LOW);
		}

		if(flagDeParada) {
			contador1 += 1;
			if(isRight() || isLeft()) {
				flagDeParada = false;
				contador1 = 0;
			}
		}
		else {
			contador1 = 0;
		}

		if(contador1 >= LIMITE) {
				//FIM DO PROGRAMA
				leftMotorControl(0);
				rightMotorControl(0);
				digitalWrite(13, HIGH);
				while(true) {}
		}
	}

	/*if(getline(line[ED]) && isCentral() && countState >= 3000) {
		countingLine += 1;
		digitalWrite(13, HIGH);
	}
	else {

		if(countingLine > 100){

			stopRobot();

		} else {
			countingLine = 0;
			digitalWrite(13, LOW);
		}

	}*/

}

bool isCentral() {
	return getline(line[NUM_SENSORS]) || getline(line[NUM_SENSORS + 1]) || getline(line[NUM_SENSORS - 1]);
}
bool isLeft() {
	bool isItLeft = false;
	for(int i = 0; i < CENTRAL - 1; i++) {
		if(getline(line[i])) {
			isItLeft = true;
			break;
		}
	}
	return isItLeft;
}

bool isRight() {
	bool isItRight = false;
	for(int i = CENTRAL + 2; i < NUM_SENSORS; i++) {
		if(getline(line[i])) {
			isItRight = true;
			break;
		}
	}
	return isItRight;
}

bool contador(int* ptrCont , int limiar){
	if(*ptrCont < limiar){
		(*ptrCont)++;
		return false;
	} 
	else {
		*ptrCont = 0;
		return true;
	}
}
