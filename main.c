#pragma config FOSC = HS        // Oscillator Selection bits (HS oscillator)
#pragma config WDTE = OFF       // Watchdog Timer Enable bit (WDT disabled)
#pragma config PWRTE = OFF      // Power-up Timer Enable bit (PWRT disabled)
#pragma config BOREN = OFF      // Brown-out Reset Enable bit (BOR disabled)
#pragma config LVP = OFF        // Low-Voltage (Single-Supply) In-Circuit Serial Programming Enable bit (RB3 is digital I/O, HV on MCLR must be used for programming)
#pragma config CPD = OFF        // Data EEPROM Memory Code Protection bit (Data EEPROM code protection off)
#pragma config WRT = OFF        // Flash Program Memory Write Enable bits (Write protection off; all program memory may be written to by EECON control)
#pragma config CP = OFF         // Flash Program Memory Code Protection bit (Code protection off)

#include <xc.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "adc.h"
#include "PWM.h"
#include "lcd.h"
#define _XTAL_FREQ 4000000
#define S1 RB0
#define S2 RB1
#define S3 RB2
#define S4 RB3
int temperatura = 0;
int temperaturaReferencia = 0;
int menu;
char string[32];
int setPoint = 4000;
int setPointReferencia = 0;
int erro;
int erroAnterior;
int kp = 50;
int ki = 2;
int kd = 0.001;
int tempo = 0.1;
int polo = 10;
int kpReferencia = 0;
int kiReferencia = 0;
int kdReferencia = 0;
int PIDReferencia = 0;
int proporcional;
int integral;
int derivativo;
int PID;

void controlarValores(){
    static char S1Anterior;
    static char S1Atual;
    static char S2Anterior;
    static char S2Atual;
    static char S3Anterior;
    static char S3Atual;
    static char S4Anterior;
    static char S4Atual;    
    
    S1Atual = S1;

    if((S1Atual)&&(!S1Anterior)){
        if (menu == 1){
            setPoint += 100;
        }
        
        if (menu == 2){
            kp += 10;
        }
        
        if (menu == 3){
            ki += 1;
        }
        
        if (menu == 4){
            kd += 1;
        }
    }

    S1Anterior = S1Atual;
    S2Atual = S2;

    if((S2Atual)&&(!S2Anterior)){
        if (menu == 1){
            setPoint -= 100;
        }
        
        if (menu == 2){
            kp -= 10;
        }
        
        if (menu == 3){
            ki -= 1;
        }
        
        if (menu == 4){
            kd -= 1;
        }
    }

    S2Anterior = S2Atual;
    S3Atual = S3;

    if((S3Atual)&&(!S3Anterior)){
    }

    S3Anterior = S3Atual;
    
    S4Atual = S4;
    if((S4Atual)&&(!S4Anterior)){
        menu += 1;
        
        if (menu == 5){
            menu = 1;
        }
    }
    S4Anterior = S4Atual;    
}

int controleMaximoMinimo(int valor){
    if (valor < 0){
        valor = 0;
    }
    
    if (valor > 1023){
        valor = 1023;
    }
    
    return valor;
}

void imprimirValoresLcd(){
    if (menu == 1){
        lcd_cmd(L1_digito8);
    }
    
    if (menu == 2){
        lcd_cmd(L2_digito8);
    }
    
    if (menu == 3){
        lcd_cmd(L1_digito10+6);
    }
    
    if (menu == 4){
        lcd_cmd(L2_digito10+6);
    }
    
    if (setPoint != setPointReferencia){
         int milhar  = setPoint/1000;
         int centena = (setPoint%1000)/100;

         lcd_cmd(L1_digito4);
         lcd_str("  C  ");
         lcd_cmd(L1_digito4);
         sprintf(string, "%d", milhar);
         lcd_str(string);
         sprintf(string, "%d", centena);
         lcd_str(string);            

         setPointReferencia = setPoint;
     }
    
    if (kp != kpReferencia){
        lcd_cmd(L2_digito4);
        lcd_str("     ");
        lcd_cmd(L2_digito4);
        sprintf(string, "%d", kp);
        lcd_str(string);
        kpReferencia = kp;
    }
    
    if (ki != kiReferencia){
        lcd_cmd(L1_digito10+2);
        lcd_str("     ");
        lcd_cmd(L1_digito10+2);
        sprintf(string, "%d", ki);
        lcd_str(string);
        kiReferencia = ki;
    }
    /*
    if (kd != kdReferencia){
        lcd_cmd(L2_digito10+2);
        lcd_str("     ");
        lcd_cmd(L2_digito10+2);
        sprintf(string, "%d", kd);
        lcd_str(string);
        kdReferencia = kd;
    }
    */
    if (PID != PIDReferencia){
        lcd_cmd(L2_digito10+2);
        lcd_str("     ");
        lcd_cmd(L2_digito10+2);
        sprintf(string, "%d", PID);
        lcd_str(string);
        PIDReferencia = PID;
    }
}

void realizarCalculo()
{
    temperatura             = (ADC_Read(0)*10/8 - 150);
    erro                    = (setPoint/10) - temperatura;
    proporcional            = kp * erro;
    integral               += (erro * ki) * 100E-3;
    derivativo             += ((temperaturaReferencia - temperatura) * kd) / 100E-3;
    temperaturaReferencia   = temperatura;
    
    PID = (proporcional + integral + derivativo);
    PID = controleMaximoMinimo(PID);
    
    PWM1_Duty(PID, 4000);
}

void realizarCalculo2()
{
    temperatura             = (ADC_Read(0)*10/8 - 150);
    erro                    = (setPoint/10) - temperatura;
    proporcional            = kp * erro;
    integral               += (tempo * ki * erro);
    derivativo             += ((polo * kd * (erro - erroAnterior)) / (1 + polo * tempo));  
    erroAnterior            = erro;
    
    PID = (proporcional + integral + derivativo);
    PID = controleMaximoMinimo(PID);    
    
    PWM1_Duty(PID, 4000);
}

void controlarCooler()
{
    int cooler  = (unsigned int)ADC_Read(1);
    cooler      = controleMaximoMinimo(cooler);
    PWM2_Duty(cooler, 4000);
}

void main(void) {
    TRISA = 0xFF;
    TRISB = 0x0F;
    TRISC = 0x00;
    TRISD = 0x00;
    TRISE = 0x00;
    ADC_Init();
    PWM1_Start();
    PWM2_Start();
    lcd_init();
    lcd_cmd(L1_digito1);
    lcd_str("TP:  C  KI:");
    lcd_cmd(L2_digito1);
    lcd_str("KP:     KD:");
    
    while(1){
        controlarValores();
        kp = controleMaximoMinimo(kp);
        ki = controleMaximoMinimo(ki);
        kd = controleMaximoMinimo(kd);   
        
        realizarCalculo();
        controlarCooler();
        imprimirValoresLcd();
    }
    
    return;
}