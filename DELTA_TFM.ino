
#include "MeAuriga.h"

//The ASCII buffer for receiving from the serial:
#define MAX_CMD_SIZE 96
#define BUFSIZE 4


extern const char errormagic[] PROGMEM;
extern const char echomagic[] PROGMEM;
///////////////////////////////////////////////////////////////////////////////////////////////////
/////////// cadenas ///////////////////////////////////////////////////////////////////////////////
#define MSG_ERR_CHECKSUM_MISMATCH           "checksum mismatch, Last Line: "
#define MSG_ERR_LINE_NO                     "Line Number is not Last Line Number+1, Last Line: "
#define MSG_RESEND                          "Resend: "
#define MSG_OK                              "ok"
#define MSG_Enqueueing                      "enqueueing \""
#define MSG_KILLED                          "PARADA DE EMERG."
///////////////////////////////////////////////////////////////////////////////////////////////////
/////////// fin cadenas ///////////////////////////////////////////////////////////////////////////////



static long gcode_N, gcode_LastN, Stopped_gcode_LastN = 0;
static int commands_in_queue = 0;
static char command_queue[BUFSIZE][MAX_CMD_SIZE];
static int serial_count = 0;
static char serial_char;
const char* queued_commands_P = NULL; /* Puntero a la línea actual en la secuencia activa de comandos, o NULL cuando ninguno */
static int cmd_queue_index_w = 0;

// pines del los motores paso a paso
int dirPin1 = mePort[PORT_1].s1;//the direction pin connect to Base Board PORT1 SLOT1
int stpPin1 = mePort[PORT_1].s2;//the Step pin connect to Base Board PORT1 SLOT2
int dirPin2 = mePort[PORT_2].s1;//the direction pin connect to Base Board PORT2 SLOT1
int stpPin2 = mePort[PORT_2].s2;//the Step pin connect to Base Board PORT2 SLOT2
int dirPin3 = mePort[PORT_3].s1;//the direction pin connect to Base Board PORT3 SLOT1
int stpPin3 = mePort[PORT_3].s2;//the Step pin connect to Base Board PORT3 SLOT2

float xPos = 0;  // posición del eje x
float yPos = 0; // posición del eje y
float zPos = -190; // posición del eje z. Negativo es arriba, positivo es abajo. Esto es así por razones históricas; los robots delta generalmente se instalan en el techo

float posT1Act = -1;
float posT2Act = -1;
float posT3Act = -1;

float posT1Dest = -1;
float posT2Dest = -1;
float posT3Dest = -1;



/// no se para que son
int result;
int data = 0;
int serialTeller=0;

#define SERIAL_ERROR_START serialprintPGM(errormagic)
#define MYSERIAL Serial
#define SERIAL_CHAR(x) MYSERIAL.write(x)
#define SERIAL_EOL SERIAL_CHAR('\n')

#define SERIAL_PROTOCOLLN(x) do{ MYSERIAL.print(x); SERIAL_EOL; }while(0)
#define SERIAL_PROTOCOLPGM(x) serialprintPGM(PSTR(x))

#define SERIAL_ERRORLN(x) SERIAL_PROTOCOLLN(x)
#define SERIAL_ECHO_START serialprintPGM(echomagic)


#define SERIAL_CHAR(x) MYSERIAL.write(x)
#define SERIAL_EOL SERIAL_CHAR('\n')

#define SERIAL_PROTOCOLCHAR(x) SERIAL_CHAR(x)
#define SERIAL_PROTOCOL(x) MYSERIAL.print(x)
#define SERIAL_PROTOCOL_F(x,y) MYSERIAL.print(x,y)
#define SERIAL_PROTOCOLPGM(x) serialprintPGM(PSTR(x))
#define SERIAL_PROTOCOLLN(x) do{ MYSERIAL.print(x); SERIAL_EOL; }while(0)
#define SERIAL_PROTOCOLLNPGM(x) do{ serialprintPGM(PSTR(x)); SERIAL_EOL; }while(0)


#define SERIAL_ERROR_START serialprintPGM(errormagic)
#define SERIAL_ERROR(x) SERIAL_PROTOCOL(x)
#define SERIAL_ERRORPGM(x) SERIAL_PROTOCOLPGM(x)
#define SERIAL_ERRORLN(x) SERIAL_PROTOCOLLN(x)
#define SERIAL_ERRORLNPGM(x) SERIAL_PROTOCOLLNPGM(x)

#define SERIAL_ECHO_START serialprintPGM(echomagic)
#define SERIAL_ECHO(x) SERIAL_PROTOCOL(x)
#define SERIAL_ECHOPGM(x) SERIAL_PROTOCOLPGM(x)
#define SERIAL_ECHOLN(x) SERIAL_PROTOCOLLN(x)
#define SERIAL_ECHOLNPGM(x) SERIAL_PROTOCOLLNPGM(x)

#define SERIAL_ECHOPAIR(name,value) do{ serial_echopair_P(PSTR(name),(value)); }while(0)



//Cosas que escribir en serie desde la memoria del programa. Ahorra 400 a 2k de RAM.
inline void serialprintPGM(const char* str) {
  char ch;
  while ((ch = pgm_read_byte(str))) {
    MYSERIAL.write(ch);
    str++;
  }
}

void ok_to_send() {
  SERIAL_PROTOCOLPGM(MSG_OK);
  SERIAL_EOL;
}

void FlushSerialRequestResend() {
  MYSERIAL.flush();
  SERIAL_PROTOCOLPGM(MSG_RESEND);
  SERIAL_PROTOCOLLN(gcode_LastN + 1);
  ok_to_send();
}

void gcode_line_error(const char* err, bool doFlush = true) {
  SERIAL_ERROR_START;
  serialprintPGM(err);
  SERIAL_ERRORLN(gcode_LastN);
  //Serial.println(gcode_N);
  if (doFlush) FlushSerialRequestResend();
  serial_count = 0;
}


/**
* Copiar un comando directamente en el búfer del mandato principal, desde la RAM.
 *
 * Devuelve false si no agrega ningún comando
 */
bool enqueuecommand(const char* cmd) {
  if (*cmd == ';' || commands_in_queue >= BUFSIZE) return false;

  // This is dangerous if a mixing of serial and this happens
  char* command = command_queue[cmd_queue_index_w];
  strcpy(command, cmd);
  SERIAL_ECHO_START;
  SERIAL_ECHOPGM(MSG_Enqueueing);
  SERIAL_ECHO(command);
  SERIAL_ECHOLNPGM("\"");
  cmd_queue_index_w = (cmd_queue_index_w + 1) % BUFSIZE;
  commands_in_queue++;
  return true;
}

/**
 * Inyectar el siguiente comando de la cola de comandos, cuando sea posible
 * Devuelve false solo si no hay ningún comando pendiente
 */
static bool drain_queued_commands_P() {
  if (!queued_commands_P) return false;

  // Obtener los 30 caracteres siguientes de la secuencia de gcodes a ejecutar
  char cmd[30];
  strncpy_P(cmd, queued_commands_P, sizeof(cmd) - 1);
  cmd[sizeof(cmd) - 1] = '\0';

  // Busca el final de la línea, o el final de la secuencia
  size_t i = 0;
  char c;
  while ((c = cmd[i]) && c != '\n') i++; //Encontrar el final de este comando de código
  cmd[i] = '\0';
  if (enqueuecommand(cmd)) {      // buffer no estaba lleno (de lo contrario volveremos a intentarlo más tarde)
    if (c)
      queued_commands_P += i + 1; // Pasar al siguiente comando
    else
      queued_commands_P = NULL;   // No tendrá más órdenes en la secuencia
  }
  return true;
}

/**
 * Agrega a la cola de comandos circular el siguiente comando de:
- La cola de comandos de inyección (queued_commands_P)
- La entrada en serie activa (usualmente USB)
 */
void get_command() {
  if (drain_queued_commands_P()) return; // se le da prioridad a los comandos que no vienen los del puerto serie
  //
  // se enbucla mientras entran los caracteres por el puerto serie y la cola no está llena
  //
  while (commands_in_queue < BUFSIZE && MYSERIAL.available() > 0) {
    serial_char = MYSERIAL.read();
    //
    // Si el caracter termina la línea, o la línea está llena ..
    //
    if (serial_char == '\n' || serial_char == '\r' || serial_count >= MAX_CMD_SIZE - 1) {
      if (!serial_count) return; // las lineas vacias se saltan

      char* command = command_queue[cmd_queue_index_w];
      command[serial_count] = 0; 

      while (*command == ' ') command++; // omite los espacios

      // si el comando es el de paro, lo procesa ya!
      if (strcmp(command, "M112") == 0) kill(PSTR(MSG_KILLED));

      cmd_queue_index_w = (cmd_queue_index_w + 1) % BUFSIZE;
      commands_in_queue += 1;

      serial_count = 0; // limpia el buffer
    } else { // no es una nueva linea
      command_queue[cmd_queue_index_w][serial_count++] = serial_char;
    }
  }

}


















 // geometría del robot
 const float e = 90.0;      // lado del triángulo del efector
 const float f = 200.0;     // lado del triangulo fijo
 const float re = 200.0;     // longitud del paralelogramo de la articulacion
 const float rf = 75.0;      // longitud de la articulación superior 

 
 // constantes trigonométricas
 const float sqrt3 = sqrt(3.0);
 const float pi = 3.141592653;    // PI
 const float sin120 = sqrt3/2.0;   
 const float cos120 = -0.5;        
 const float tan60 = sqrt3;
 const float sin30 = 0.5;
 const float tan30 = 1/sqrt3;
 
 // cinemática directa: (posT1, posT2, posT3) -> (x0, y0, z0)
 // salida: 0=OK, -1=posición inexistente
 int delta_cinematicaDirecta(float posT1, float posT2, float posT3, float &x0, float &y0, float &z0) {
     int status = -1;
   // TODO cambiar esto, ya que está pensado para angulos de servo y yo nenesito posición en vertical de las torres
  /* float dtr = pi/(float)180.0;
 
     theta1 *= dtr;
     theta2 *= dtr;
     theta3 *= dtr;
 
     float y1 = -(t + rf*cos(theta1));
     float z1 = -rf*sin(theta1);
 
     float y2 = (t + rf*cos(theta2))*sin30;
     float x2 = y2*tan60;
     float z2 = -rf*sin(theta2);
 
     float y3 = (t + rf*cos(theta3))*sin30;
     float x3 = -y3*tan60;
     float z3 = -rf*sin(theta3);
 
     float dnm = (y2-y1)*x3-(y3-y1)*x2;
 
     float w1 = y1*y1 + z1*z1;
     float w2 = x2*x2 + y2*y2 + z2*z2;
     float w3 = x3*x3 + y3*y3 + z3*z3;
     
     // x = (a1*z + b1)/dnm
     float a1 = (z2-z1)*(y3-y1)-(z3-z1)*(y2-y1);
     float b1 = -((w2-w1)*(y3-y1)-(w3-w1)*(y2-y1))/2.0;
 
     // y = (a2*z + b2)/dnm;
     float a2 = -(z2-z1)*x3+(z3-z1)*x2;
     float b2 = ((w2-w1)*x3 - (w3-w1)*x2)/2.0;
 
     // a*z^2 + b*z + c = 0
     float a = a1*a1 + a2*a2 + dnm*dnm;
     float b = 2*(a1*b1 + a2*(b2-y1*dnm) - z1*dnm*dnm);
     float c = (b2-y1*dnm)*(b2-y1*dnm) + b1*b1 + dnm*dnm*(z1*z1 - re*re);
  
     // discriminant
     float d = b*b - (float)4.0*a*c;
     if (d < 0) return -1; // non-existing point
 
     z0 = -(float)0.5*(b+sqrt(d))/a;
     x0 = (a1*z0 + b1)/dnm;
     y0 = (a2*z0 + b2)/dnm;*/
     return status;
 }
 
 // cinemática inversa: (x0, y0, z0) -> (posT1, posT2, posT3)
 // returned status: 0=OK, -1=non-existing position
 int delta_cinematicaInversa(float x0, float y0, float z0, float &posT1, float &posT2, float &posT3) {
   int status = -1;
    /* theta1 = theta2 = theta3 = 0;
     int status = delta_calcAngleYZ(x0, y0, z0, theta1);
     if (status == 0) status = delta_calcAngleYZ(x0*cos120 + y0*sin120, y0*cos120-x0*sin120, z0, theta2);  // rotate coords to +120 deg
     if (status == 0) status = delta_calcAngleYZ(x0*cos120 - y0*sin120, y0*cos120+x0*sin120, z0, theta3);  // rotate coords to -120 deg */
     return status;
 }




///////////////////////////////////////////////////////////////////////////////////////////////
// funciones auxiliares
///////////////////////////////////////////////////////////////////////////////////////////////

void stepT1(boolean dir,int steps)
{
  digitalWrite(dirPin1,dir);
  delay(50);
  for(int i=0;i<steps;i++)
  {
    digitalWrite(stpPin1, HIGH);
    delayMicroseconds(800);
    digitalWrite(stpPin1, LOW);
    delayMicroseconds(800); 
  }
}

void stepT2(boolean dir,int steps)
{
  digitalWrite(dirPin2,dir);
  delay(50);
  for(int i=0;i<steps;i++)
  {
    digitalWrite(stpPin2, HIGH);
    delayMicroseconds(800);
    digitalWrite(stpPin2, LOW);
    delayMicroseconds(800); 
  }
}

void stepT3(boolean dir,int steps)
{
  digitalWrite(dirPin3,dir);
  delay(50);
  for(int i=0;i<steps;i++)
  {
    digitalWrite(stpPin3, HIGH);
    delayMicroseconds(800);
    digitalWrite(stpPin3, LOW);
    delayMicroseconds(800); 
  }
}

///////////////////////////////////////////////////////////////////////////////////////////////
// inicialización
///////////////////////////////////////////////////////////////////////////////////////////////
void setup()
{
  // inicializa los motores
  pinMode(dirPin1, OUTPUT);
  pinMode(stpPin1, OUTPUT);
  pinMode(dirPin2, OUTPUT);
  pinMode(stpPin2, OUTPUT);
  pinMode(dirPin3, OUTPUT);
  pinMode(stpPin3, OUTPUT);
  
  // inicializa la comunicacion
  Serial.begin(115200);
}


///////////////////////////////////////////////////////////////////////////////////////////////
// fbucle principal
///////////////////////////////////////////////////////////////////////////////////////////////
void loop(){
  if (commands_in_queue < BUFSIZE - 1) get_command();

  if (commands_in_queue) {

      process_next_command();
      commands_in_queue--;
      cmd_queue_index_r = (cmd_queue_index_r + 1) % BUFSIZE;
  }    
  checkHitEndstops();
  idle();
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  char xxPos[4];  // x position taken from processing
  char yyPos[4];  // y position taken from processing
  char zzPos[4];  // z position taken from processing
  char tempo;     // temp variable

  if (Serial.available() > 12) {
    tempo=Serial.read();  //read from serial
    int conta=0;
    while (tempo != 'X')  // wait for the X position
    { 
      tempo=Serial.read();
       
      }
  
    if (Serial.available() >= 11) {
  
      xxPos[0]=Serial.read();  //read X byte
      xxPos[1]=Serial.read();  //read X byte
      xxPos[2]=Serial.read();  //read X byte
      xxPos[3]=Serial.read();  //read X byte
      
      xPos=atoi(xxPos); //transfor bytes in integer
    
    
     tempo=Serial.read();  //read Y char
     
      yyPos[0]=Serial.read(); //read Y byte
      yyPos[1]=Serial.read(); //read Y byte
      yyPos[2]=Serial.read(); //read Y byte
      yyPos[3]=Serial.read(); //read Y byte
     
      yPos=atoi(yyPos); //transform bytes in integer
    
      tempo=Serial.read(); //read Z Char
     
      zzPos[0]=Serial.read(); //read Z byte
      zzPos[1]=Serial.read(); //read Z byte
      zzPos[2]=Serial.read(); //read Z byte
      zzPos[3]=Serial.read(); //read Z byte
     
      zPos=atoi(zzPos); //transform bytes in integer
    
    }
  } 

  result = delta_cinematicaInversa(xPos, yPos, zPos, posT1Dest, posT2Dest, posT3Dest);
 //step(1,200);//run 200 step
  
  
  
  
  /*
  
   H = DELTA_DIAGONAL_ROD      // La hipotenusa de todos los triángulos delta es una longitud de brazo
  Dx = Tx - Ex                // X diferencia entre el carro y el centro efector
  Dy = Ty - Ey                // Diferencia de Y entre el carro y el centro efector
  Dte = sqrt(Dx^2 + Dy^2)     // Distancia del carro XY al centro del efector (A)
  Dte -= EFFECTOR_RADIUS      // Longitud hasta el borde, no el centro
  // H*sin(acos(A/H))
  Ø = acos(Dte/H)             // Get the angle Ø from A/H
  O = H * sin(Ø)              // Opposite side is solved, O=H*(O/H)
  Zt = O + z                  // Add the carriage Z position
  Zt += effector_thickness/2  // Now the effector will touch the bed when z=0.
  Zt += nozzle_length         // Now the nozzle touches the bed at z=0. Done!
  
  */
  
}



