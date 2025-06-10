/******************************************************************************
 RETO
*******************************************************************************/
#include <Arduino_FreeRTOS.h>
#include "queue.h"
#include "semphr.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "src/MeSingleLineFollower.h"
#include "src/MeCollisionSensor.h"
#include "src/MeBarrierSensor.h"
#include "src/MeNewRGBLed.h"
#include <MeMegaPi.h>
#include <math.h>

//declaraciones de la tasa de comunicación serial
#define F_CPU 16000000
#define USART_BAUDRATE 57600
#define UBRR_VALUE (((F_CPU / (USART_BAUDRATE * 16UL))) - 1)

//variables constantes para calcular las velocidades de las llantas
#define distancia_ruedas_cm 13.5  //l (en cm) distancia de ruedas
#define Vn 20               //velocidad normal, derecho(cm/s)

int interrupcion = 0; //variable para el manejo de la interrupción
char RX_Byte;
unsigned char mybuffer[50];
double Vl, Vr; //velocidades en cada rueda izqueirda Vl, derecha Vr

///Codigo de los motores utilizados en los ejemplos///
MeBarrierSensor barrier_62(62);
MeMegaPiDCMotor motor_1(1); //1A
MeMegaPiDCMotor motor_9(9); //1B
MeMegaPiDCMotor motor_2(2); //2A
MeMegaPiDCMotor motor_10(10); //2B
double angle_rad = PI/180.0;
double angle_deg = 180.0/PI;

///funcion que calcula la velocidad de las ruedas en cm/s
void calcularVelocidades(double Kp, double angulo_en_grados, double* Vl, double* Vr) {

    //angulo a radianes
    double angulo_rad = angulo_en_grados * M_PI / 180.0; //M_PI = PI (constante)
    
    //velocidad angular
    double omega = Kp * angulo_rad;
    
    //velocidad para las ruedad
    *Vl = Vn + (omega * distancia_ruedas_cm / 2.0); //direcciones corregidas
    *Vr = Vn - (omega * distancia_ruedas_cm/ 2.0); //direcciones corregidas
    //*Vl = Vn - (omega * distancia_ruedas_cm / 2.0); //formula original
    //*Vr = Vn + (omega * distancia_ruedas_cm / 2.0); //formula original
}
//funcion que transforma la velodicad en cm/s a porcentaje de pwm
double transformarVelocidades(double velocidad_cm_s) {
    const double velocidad_maxima_cm_s = 22.2425;  //velocidad
    const double pwm_base = 60.0;                 //porcentaje de PWM que da esa velocidad

    //regla de 3 
    double pwm = (velocidad_cm_s * pwm_base) / velocidad_maxima_cm_s;

    // Limitar entre 0 y 100
    if (pwm > 100.0) pwm = 100.0;
    if (pwm < 0.0) pwm = 0.0;

    return pwm;
}
//funcion que transforma velocidad en pwm a un entero de entre 0 y 255
int transformarPWM(double velocidad_pwm) {
    const double velocidad_maxima_pwm = 100;  //velocidad
    const int numero_base = 255;                 //entero que da ese pwm

    //regla de 3 
    int entero = (velocidad_pwm * numero_base) / velocidad_maxima_pwm ;

    if (entero > 255) entero = 255;
    if (entero < 0) entero = 0;

    return entero;
}
///Codigo de los motores utilizados en los ejemplos///
void motor_foward_left_run(int16_t speed)
{
   motor_10.run(-speed);
}

void motor_foward_right_run(int16_t speed)
{
  motor_1.run(speed);
}

void motor_back_left_run(int16_t speed)
{
  motor_2.run(-speed);
}

void motor_back_right_run(int16_t speed)
{
  motor_9.run(speed);
}

void move_control(int16_t vx, int16_t vy, int16_t vw)
{
  int16_t foward_left_speed;
  int16_t foward_right_speed;
  int16_t back_left_speed;
  int16_t back_right_speed;

  foward_left_speed = vy + vx + vw;
  foward_right_speed = vy - vx - vw;
  back_left_speed = vy - vx + vw;
  back_right_speed = vy + vx - vw;

  motor_foward_left_run(foward_left_speed);
  motor_foward_right_run(foward_right_speed);
  motor_back_left_run(back_left_speed);
  motor_back_right_run(back_right_speed);
}

//handle para la queue que almacena los angulos
QueueHandle_t myQueue;

//semaphore handle para manejar la interrupción PK0
SemaphoreHandle_t interruptSemaphore;

TaskHandle_t vTaskHandleUART; //Handle de la tarea que controla motores

//UART and pin configuration
void setup()
{
  xTaskCreate(vHandlerTaskUART,"MOTOR HANDLER TASK",100, NULL, 1, &vTaskHandleUART);
  xTaskCreate(vHandlerTaskSTOP,"PCINT16 HANDLER TASK",100,NULL,1,NULL);
  xTaskCreate(vREADTask,"READ DATA",100,NULL,2,NULL);
 
  //creación del semáforo binario
  interruptSemaphore = xSemaphoreCreateBinary();
  
  //si el semáforo es creado, inicializa interrupción PCINT16
  if(interruptSemaphore != NULL)
  {
    //se hace PK0 (pin 62) entrada
    DDRK &= ~(1 << PK0);
    //Resistencia pull up en PK0
    PORTK &= ~(1 << PK0);
    //se habilita interrupción por cambio de estado en PORTK
    PCICR |= (1 << PCIE2);
    //se habilita interrupción PCINT16
    PCMSK2 |= (1 << PCINT16);
    sei();
  }
  //creación de la queue
  myQueue = xQueueCreate(3, sizeof(char));

  //revisa si la queue ha sido creada
  if(myQueue != NULL) 
  {    
    //configuración UART
    UBRR0H = (uint8_t)(UBRR_VALUE >> 8);
    UBRR0L = (uint8_t)UBRR_VALUE;
    //UCSR0C |= (1 << UCSZ00) | (1 << UCSZ01); // 8-bit data
    UCSR0C = 0x06;       // Set frame format: 8data, 1stop bit 
    UCSR0B |= (1 << RXEN0) | (1 << TXEN0) | (0 << RXCIE0);   // TX and RX enables and RX interrupt enabled      
    UCSR0A = 0x00; // Limpia banderas

    sprintf(mybuffer, "1\n");//Envia un "1" para iniciar la raspberry
    USART_Transmit_String((unsigned char *)mybuffer);
  }
}

//Tarea que se encarga de leer los caracteres de la cola y controla los motores
void vHandlerTaskUART(void * pvParameters)
{
  
   char data1, data2, data3;
  BaseType_t qStatus1, qStatus2, qStatus3;
  int entero_L, entero_R;
  double kp;

  while(1)
  {
    //revisa el tamaño de la cola, si es 3, avanza
    unsigned int itemsInQueue = uxQueueMessagesWaiting(myQueue);
    if (itemsInQueue == 3)
    {
      qStatus1 = xQueueReceive(myQueue, &data1, portMAX_DELAY);
        
      if(qStatus1 == pdPASS)
      {
        sprintf(mybuffer, "Dato 1 leido %c \n", data1);
        USART_Transmit_String((unsigned char *)mybuffer);
      }
      qStatus2 = xQueueReceive(myQueue, &data2, portMAX_DELAY);
      if (qStatus2 == pdPASS)
      {
        sprintf(mybuffer, "Dato 2 leido %c \n", data2);
        USART_Transmit_String((unsigned char *)mybuffer);
      }
        qStatus3 = xQueueReceive(myQueue, &data3, portMAX_DELAY);
        if (qStatus3 == pdPASS)
        {
          sprintf(mybuffer, "Dato 3 leido %c \n", data3);
          USART_Transmit_String((unsigned char *)mybuffer);
          
          //Almaceno los caracteres en una variable y lo convierte a entero
          char value[4];
          sprintf(value, "%c%c%c", data1, data2, data3);
          int grad = atoi(value);
          sprintf(mybuffer, "Entrando a la funcion, grados: %d \n", grad);
          USART_Transmit_String((unsigned char *)mybuffer);

          //Obtitiene el valor absoluto del angulo y dependiendo de este, el valor de kp cambia
          int grados = abs(grad);
          if (grados  >= 60)
          {
            kp = 1.8;
          }else if((grados < 60) && (grados >= 40))
          {
            kp = 1.6;
          }else if ((grados < 40) && (grados >= 20))
          {
            kp = 1.4;
          }else
          {
            kp = 0.1;
            }
          //velocidades en cm/s llamando a la función
          calcularVelocidades(kp,grad, &Vl, &Vr);
          
          //convertir cm/s a porcentaje de pwm
          double pwm_L = transformarVelocidades(Vl);
          double pwm_R = transformarVelocidades(Vr);
           
          //convertir pwm  a entero entre 0 y 255
          entero_L = transformarPWM(pwm_L);
          entero_R = transformarPWM(pwm_R);
      
          //muestra la velocidad
          sprintf(mybuffer,"velocidad rueda izquierda (Vl): %d \n", entero_L);
          USART_Transmit_String((unsigned char *)mybuffer);
          sprintf(mybuffer,"velocidad rueda derecha (VR): %d \n", entero_R);
          USART_Transmit_String((unsigned char *)mybuffer);
          
          sprintf(mybuffer, "Modificando PWM\n");
          USART_Transmit_String((unsigned char *)mybuffer);
          
          if (grad == 0)  //Si el angulo = 0, los motores se mantienen constantes
          {
            motor_1.run(100);
            motor_9.run(100);
            motor_2.run(-100);
            motor_10.run(-100);
          }
          else //Si engulo!=0, los motores se modifican
          {
            motor_1.run(entero_R);
            motor_9.run(100);
            motor_2.run(-100);
            motor_10.run(-entero_L);
            }
          
           sprintf(mybuffer, "1\n");
          USART_Transmit_String((unsigned char *)mybuffer);
          }
      }
    vTaskDelay(1);
  }
}

//Handler Task, Tarea que detiene los motores al haber una interrupción en PK0
void vHandlerTaskSTOP(void *pvParameters)
{
  while(1)
  {
    //espera por siempre al semáforo
    if (xSemaphoreTakeFromISR(interruptSemaphore, pdTRUE) == pdPASS) 
    {
      if (interrupcion == 1)
      {
        sprintf(mybuffer, "-------------------0\n");
        USART_Transmit_String((unsigned char *)mybuffer);
        motor_1.run(0);
        motor_9.run(0);
        motor_2.run(0);
        motor_10.run(0);
        vTaskSuspend(vTaskHandleUART); //Suspende la tarea que controla los motores
      }else
      {
        sprintf(mybuffer, "1\n");
        USART_Transmit_String((unsigned char *)mybuffer);
        vTaskResume(vTaskHandleUART); //Reanuda la tarea que controla los motores
        }
    }
  //vTaskDelay(1);
  }
}

//Tarea que leee los datos del UART y los almacena en la cola
void vREADTask(void *pvParameters)
{  
    BaseType_t qStatus;
    while(1)
    {  
          //revisa si hay nuevo dato RX en UART
      if((UCSR0A & (1 << RXC0)) != 0)
      {
        //lee dato recibido
        RX_Byte = UDR0;
        sprintf(mybuffer, "Recibido: %c\n", RX_Byte);
        USART_Transmit_String((unsigned char *)mybuffer);
        
        //Envia dato a la cola
        qStatus = xQueueSendToBack(myQueue, &RX_Byte, portMAX_DELAY);
        if(qStatus == pdPASS)
        {
          sprintf(mybuffer, "Dato enviado a la queue: %c\n", RX_Byte);
          USART_Transmit_String((unsigned char *)mybuffer);
        }
        else
        {
          sprintf(mybuffer, "Dato no enviado a la queue\n");
          USART_Transmit_String((unsigned char *)mybuffer);
        }
      }
     vTaskDelay(1);
    }
}

//loop vacio
void loop()
{}

//PCINT2 ISR, da el semáforo desde ISR
ISR(PCINT2_vect)
{  
  sprintf(mybuffer, "ISR\n");
  USART_Transmit_String((unsigned char *)mybuffer);
  if (!(PINK & (1 << PK0))) 
  {
    interrupcion = 1;
    } else 
    {
    interrupcion = 0;
    }
    xSemaphoreGiveFromISR(interruptSemaphore, pdTRUE);
}

/////funciones de transmisión del UART///////
void USART_Transmit(unsigned char data)
{
    //wait for empty transmit buffer
    while(!(UCSR0A & (1 << UDRE0)));
  
    //put data into buffer, send data
    UDR0 = data;  
}

void USART_Transmit_String(unsigned char * pdata)
{
    unsigned char i;
    //calculate string length
    unsigned char len = strlen(pdata);

    //transmit byte for byte
    for(i=0; i < len; i++)
    {
        //wait for empty transmit buffer
        while(!(UCSR0A & (1 << UDRE0)));
        //put data into buffer, send data
        UDR0 = pdata[i];
    }
}
