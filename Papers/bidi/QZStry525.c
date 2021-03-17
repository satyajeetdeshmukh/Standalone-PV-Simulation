#include "F28x_Project.h"
#include "math.h"
#include "complex.h"
#define EPWM2_TIMER_TBPRD 2500 // 10kHz PWM Period register
#define PI 3.14159265358979323846
void Setup_ADC_conf(void);
void InitEPwm2Example(void);
void InitEPwm6Example(void);
void Setup_ePWM7(void);
void Setup_ePWM8(void);
__interrupt void epwm2_isr(void);
unsigned int i=0,m=0,k=0,b[401];
float Vpv=0;
float Ipv=0;
float P=0;
float P_old=0;
float inc=-1;
float V_max=37;
float V_min=34.0;
float Vref_pv=35.2;
float DeltaV=0.01;
float verror=0;
float v_adc=0;
float v_adc1=0;
float i_adc=0;
float i_adc1=0;
float kp_vpv = 0.0015;
float ki_vpv = 0.1e-6;
float D = 0.1;
float D_pv_limit = 0.1;
float t_s = 50e-6;
int PWM7_CMP = 0;
float a=0;
float c=1;
float MI=0;
float M1=0;
float M2=0;
float vac1=0;
float vac2=0;
float vac=0;
float io1=0;
float io2=0;
float io=0;
float V_out=0.0;
float  I_ac=0.0, M=0.0,  Vd=0.0, Vq=0.0, I_ac_ref=0.0, Iac_er_n1=0.0, Iac_error=0.0,Iac_error1=0.0;
float Vac_pk1=0, Vac_ref=0.0, V_alpha_r=0.0, V_beta_r=0.0, Vd_r=0.0, Vq_r=0.0, V_alpha=0.0, I_alpha_r=0.0, I_beta_r=0.0, I_ac2=0.0,I_ac1=0.0;
float V_beta = 0.0,  Vd_er1=0.0, Id_p=0.0, Id_i=0.0, Id_i1=0.0, Id_ref=0.0, Vq_er1=0.0, Iq_p=0.0, Iq_i=0.0, Iq_i1=0.0,Iq_ref=0.0;
float kp_vd=0.009, ki_vd=0.001, Vac_pk=30, kp_iac=0.09, ki_iac=0.01;
//int PWM6_CMP = 0;

// Main

void main(void)
{
 InitSysCtrl();

/*Here I generate sin look up table for sin values, I have 200 sample for one period 50Hz sin wave.Also I multiple buy -1 of negative values of sin wave.

Sin(w*t)=sin(2*pi*f*t)=sin(2*pi*50*(20ms/100)*sample_number)

Sample number=0,1,2,3,4,……………,200

f=50Hz

m=sample number*/

for(m=0;m<400;m++){
a=sin(PI*0.005*m);
c=cos(PI*0.005*m);
b[m]=1250*a + 1250;
}
b[400]=0;

EALLOW;

GpioCtrlRegs.GPAPUD.bit.GPIO2 = 1; // Disable pull-up on GPIO2 (EPWM2A)
GpioCtrlRegs.GPAMUX1.bit.GPIO2 = 1; // Configure GPIO2 as EPWM2A
GpioCtrlRegs.GPAPUD.bit.GPIO2 = 1; // Disable pull-up on GPIO2 (EPWM2A)
GpioCtrlRegs.GPAMUX1.bit.GPIO3 = 1; // ePWM2B active
GpioCtrlRegs.GPAPUD.bit.GPIO10 = 1; // Disable pull-up on GPIO10 (EPWM6A)
GpioCtrlRegs.GPAPUD.bit.GPIO11 = 1; // Disable pull-up on GPIO11 (EPWM6B)
GpioCtrlRegs.GPAMUX1.bit.GPIO11 = 1; // ePWM6B active
GpioCtrlRegs.GPAMUX1.bit.GPIO10 = 1; // ePWM6A active
GpioCtrlRegs.GPAMUX1.bit.GPIO12 = 1; // ePWM7A active
GpioCtrlRegs.GPAPUD.bit.GPIO12 = 1; // Disable pull-up on GPIO12 (EPWM7A)
GpioCtrlRegs.GPAMUX1.bit.GPIO14 = 1; // ePWM8A active
GpioCtrlRegs.GPAPUD.bit.GPIO14 = 1; // Disable pull-up on GPIO14 (EPWM8A)


EDIS;
DINT;
InitPieCtrl();
IER = 0x0000;
IFR = 0x0000;
InitPieVectTable();
EALLOW;
PieVectTable.EPWM2_INT = &epwm2_isr; //function for epwm2 interrupt
EDIS;
EALLOW;
CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 0;
EDIS;
InitEPwm2Example();
InitEPwm6Example();
Setup_ePWM7();
Setup_ePWM8();
Setup_ADC_conf();
EALLOW;
CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 1;
EDIS;
IER |= M_INT3;
PieCtrlRegs.PIEIER3.bit.INTx2 = 1;
CpuSysRegs.PCLKCR2.bit.EPWM2=1;
EINT; // Enable Global interrupt INTM
ERTM; // Enable Global realtime interrupt DBGM

for(;;)
{
asm (" NOP");
}
}
void Setup_ePWM8(void)
{
        EPwm8Regs.TBCTL.bit.CLKDIV =  0;    // CLKDIV = 1
        EPwm8Regs.TBCTL.bit.HSPCLKDIV = 1;  // HSPCLKDIV = 2
        EPwm8Regs.TBCTL.bit.CTRMODE = 2;    // up count mode
        EPwm8Regs.TBPRD = 2500;            // 10KHz - PWM signal
        EPwm8Regs.ETSEL.all = 0;
        EPwm8Regs.ETSEL.bit.SOCAEN = 1;     //Enable SOC on A group
        EPwm8Regs.ETSEL.bit.SOCASEL = 2;    //Select SOC from CPMA on counter
        EPwm8Regs.ETPS.bit.SOCAPRD = 1;     //Generate pulse on 1st event
}
void Setup_ADC_conf(void)
{

        EALLOW;
        AdcaRegs.ADCCTL2.bit.PRESCALE = 6;          // set ADCCLK divider to /4
        //  AdcSetMode(ADC_ADCB, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);
        AdcaRegs.ADCCTL2.bit.RESOLUTION = 0;        // 12-bit resolution
        AdcaRegs.ADCCTL2.bit.SIGNALMODE = 0;        // single-ended channel conversions (12-bit mode only)
        AdcaRegs.ADCCTL1.bit.INTPULSEPOS = 1;       // Set pulse positions to late
        AdcaRegs.ADCCTL1.bit.ADCPWDNZ = 1;          // power up the ADC

        AdcbRegs.ADCCTL2.bit.PRESCALE = 6;          // set ADCCLK divider to /4
        //  AdcSetMode(ADC_ADCB, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);
        AdcbRegs.ADCCTL2.bit.RESOLUTION = 0;        // 12-bit resolution
        AdcbRegs.ADCCTL2.bit.SIGNALMODE = 0;        // single-ended channel conversions (12-bit mode only)
        AdcbRegs.ADCCTL1.bit.INTPULSEPOS = 1;       // Set pulse positions to late
        AdcbRegs.ADCCTL1.bit.ADCPWDNZ = 1;          // power up the ADC
        DELAY_US(100);                             // delay for 1ms to allow ADC time to power up
        EDIS;

        EALLOW;
        AdcaRegs.ADCSOC1CTL.bit.CHSEL = 4;          // SOC1 will convert pin A4 (69)          ipv
        AdcaRegs.ADCSOC1CTL.bit.ACQPS = 14;         // sample window is 14 SYSCLK cycles
        AdcaRegs.ADCSOC1CTL.bit.TRIGSEL = 19;        // trigger on ePWM2 SOCA
        AdcaRegs.ADCINTSEL1N2.bit.INT1SEL = 1;      // end of SOC1   will set INT1 flag
        AdcaRegs.ADCINTSEL1N2.bit.INT1E = 1;        // enable INT1 flag
        AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;      // make sure INT1 flag is cleared

        AdcbRegs.ADCSOC1CTL.bit.CHSEL = 3;          // SOC1 will convert pin B3  (68)         vpv
        AdcbRegs.ADCSOC1CTL.bit.ACQPS = 14;         // sample window is 14 SYSCLK cycles
        AdcbRegs.ADCSOC1CTL.bit.TRIGSEL = 19;        // trigger on ePWM2 SOCA
        AdcbRegs.ADCINTSEL1N2.bit.INT1SEL = 1;      // end of SOC1   will set INT1 flag
        AdcbRegs.ADCINTSEL1N2.bit.INT1E = 1;        // enable INT1 flag
        AdcbRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;      // make sure INT1 flag is cleared

        AdcaRegs.ADCSOC0CTL.bit.CHSEL = 2;          // SOC0 will convert pin A2  (64)         vac
        AdcaRegs.ADCSOC0CTL.bit.ACQPS = 14;         // sample window is 14 SYSCLK cycles
        AdcaRegs.ADCSOC0CTL.bit.TRIGSEL = 19;        // trigger on ePWM2 SOCA
        AdcaRegs.ADCINTSEL1N2.bit.INT1SEL = 0;      // end of SOC0   will set INT1 flag
        AdcaRegs.ADCINTSEL1N2.bit.INT1E = 1;        // enable INT1 flag
        AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;      // make sure INT1 flag is cleared

        AdcbRegs.ADCSOC0CTL.bit.CHSEL = 5;          // SOC0 will convert pin B5 (66)           i0
        AdcbRegs.ADCSOC0CTL.bit.ACQPS = 14;         // sample window is 14 SYSCLK cycles
        AdcbRegs.ADCSOC0CTL.bit.TRIGSEL = 19;        // trigger on ePWM2 SOCA
        AdcbRegs.ADCINTSEL1N2.bit.INT1SEL = 0;      // end of SOC1   will set INT1 flag
        AdcbRegs.ADCINTSEL1N2.bit.INT1E = 1;        // enable INT1 flag
        AdcbRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;      // make sure INT1 flag is cleared
        EDIS;
    }

__interrupt void epwm2_isr(void)
{
       //mppt
       AdcbRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;      // Clear INT1 flag
       v_adc =AdcbResultRegs.ADCRESULT1;
       v_adc1 = ((v_adc*3)/4095);
       //Vpv = 35.4;
       Vpv = v_adc1*303.3030;
       AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;      // Clear INT1 flag
       i_adc =AdcaResultRegs.ADCRESULT1;
       i_adc1 = ((i_adc*3)/4095);
       //Ipv =7.5;
       Ipv = i_adc1*10;

       P = Vpv*Ipv;
       if(P<P_old)
       {
          inc=-inc;
       }
       Vref_pv=Vref_pv+inc*DeltaV;
       if (Vref_pv>V_max)
       {
       Vref_pv=V_max;
       }
       if (Vref_pv<V_min)
       {
       Vref_pv=V_min;
       }
       verror = Vpv - Vref_pv;
       P_old=P;
       D = D + kp_vpv*verror + ki_vpv*verror*t_s;
       if (D > 0.255)
       {
           D_pv_limit = 0.255;
           D=0.255;
       }
       else if (D < 0.245)
       {
           D_pv_limit = 0.245;
           D=0.245;
       }
       else
       {
           D_pv_limit = D;
       }
       PWM7_CMP  = D_pv_limit*1250;
       EPwm7Regs.CMPA.bit.CMPA = PWM7_CMP;
       //PieCtrlRegs.PIEACK.all = 1;     // acknowledge PIE group 1 to enable further interrupts

       // Return from interrupt
       AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;      // Clear INT1 flag
       //  control logic
       vac1 =AdcaResultRegs.ADCRESULT0;
       vac2 = ((vac1*3)/4095)-1;
       vac = vac2*229;
       V_out = vac;
       //V_out = 62*a;
       AdcbRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;      // Clear INT1 flag
       io1 =AdcbResultRegs.ADCRESULT0;
       io2 = ((io1*3)/4095)-1.5;
       io = io2*5.43;
       I_ac = io;
// soft start
        if(Vac_pk<=Vac_pk1)
            {           Vac_pk1=Vac_pk;             }
        else
            {           Vac_pk1=Vac_pk1+0.0005;             }
        Vac_ref = Vac_pk1*a;
// I_ac current limiter to avoid transient current sensing
         if (I_ac>= 3)
                         {I_ac = 3;}
              else if (I_ac<= -3)
                            {I_ac =-3;}
               else
                              {I_ac = I_ac;}

//dq control

            V_alpha_r = Vac_ref;
            V_beta_r = 0.0;

            Vd_r = c*V_alpha_r + a*V_beta_r;
            Vq_r = -a*V_alpha_r + c*V_beta_r;

            V_alpha =V_out; // V_beta = 0.0 initialized section

            Vd = c*V_alpha + a*V_beta;
            Vq = -a*V_alpha + c*V_beta;

            Vd_er1 = Vd_r - Vd;

            Id_p=kp_vd*Vd_er1;

            Id_i=Id_i1+(ki_vd*Vd_er1);


            if(Id_i>5.0)
                {Id_i=3.0;Id_i1=3.0;}
            else if(Id_i<-5.0)
                {Id_i=-3.0;Id_i1=-3.0;}
            else
                {Id_i=Id_i;}
            Id_i1=Id_i;

            Id_ref=Id_p+Id_i;

            Vq_er1 = Vq_r - Vq;

            Iq_p=kp_vd*Vq_er1;

            Iq_i=Iq_i1+(ki_vd*Vq_er1);

            if(Iq_i>3.0)
                {Iq_i=3.0;}
            else if(Iq_i<-3.0)
                {Iq_i=-3.0;}
            else
                {Iq_i=Iq_i;}
            Iq_i1=Iq_i;
            Iq_ref=Iq_p+Iq_i;

            I_alpha_r = c*Id_ref - a*Iq_ref;
    //      I_beta_r = sin*Id_ref + cos*Iq_ref;
            I_ac_ref = I_alpha_r;
            if(I_ac_ref>3.0)
                {I_ac_ref=3.0;}
            else if(I_ac_ref<-3.0)
                {I_ac_ref=-3.0;}
            else
            {I_ac_ref=I_ac_ref;}

            Iac_er_n1= I_ac_ref-I_ac;

            Iac_error=Iac_error1+(ki_iac*Iac_er_n1);

            if(Iac_error>0.95)
                {   Iac_error=0.95;}
            else if(Iac_error<-0.95)
                {   Iac_error=-0.95;}
            else
                {Iac_error=Iac_error;}
            Iac_error1=Iac_error;
            MI = (kp_iac*Iac_er_n1)+Iac_error;
                        if(MI>0.80)
                            {   MI=0.80;}
                        else if(MI<-0.80)
                           {   MI=-0.80;}
                        else
                            {MI=MI;}


/*Here my code assign a new sin value into compare reg. but negative side of sin wave pwm logic is invertered.*/
    a=sin(PI*0.005*i);
    c=cos(PI*0.005*i);
    M=MI*a;
    M1  = (M)*(1250) + (1250);
    M2  = (-M)*(1250)+ (1250);
    if (a>=0){
    EPwm2Regs.AQCTLA.bit.CAU = AQ_CLEAR; // Set PWM2A on event A, up
    EPwm2Regs.AQCTLA.bit.CAD =AQ_SET; // Clear PWM2A on event B, down
    EPwm2Regs.CMPA.bit.CMPA =M1;
    }
    else {
    EPwm2Regs.AQCTLA.bit.CAU =AQ_SET; // Set PWM2A on event A, up
    EPwm2Regs.AQCTLA.bit.CAD = AQ_CLEAR; // Clear PWM2A on event B, down
    EPwm2Regs.CMPA.bit.CMPA =M2;
    }
    i++;
    if (i==401){
    i=0;
    }
    if (a>=0){
    EPwm6Regs.AQCTLA.bit.CAU = AQ_CLEAR; // Set PWM6A on event A, up
    EPwm6Regs.AQCTLA.bit.CAD =AQ_SET; // Clear PWM6A on event B, down
    EPwm6Regs.CMPA.bit.CMPA =M2;
    }
    else {
    EPwm6Regs.AQCTLA.bit.CAU =AQ_SET; // Set PWM6A on event A, up
    EPwm6Regs.AQCTLA.bit.CAD = AQ_CLEAR; // Clear PWM6A on event B, down
    EPwm6Regs.CMPA.bit.CMPA =M1;
    }
    i++;
    if (i==401){
    i=0;
    }
    EPwm2Regs.ETCLR.bit.INT = 1;
    EPwm6Regs.ETCLR.bit.INT = 1;
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;
}
void InitEPwm2Example()
{
EPwm2Regs.ETSEL.bit.INTSEL = ET_CTR_ZERO; // hem zero hemde TBPRD zamaninda intrrupt aliyo.
EPwm2Regs.ETSEL.bit.INTEN = 1; // Enable INT
EPwm2Regs.ETPS.bit.INTPRD = ET_1ST; // Generate INT on 1st event
EPwm2Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN; // Count up and down
EPwm2Regs.TBCTL.bit.PHSEN = TB_ENABLE; // Disable phase loading
EPwm2Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1; // Clock ratio to SYSCLKOUT
EPwm2Regs.TBCTL.bit.CLKDIV = TB_DIV2;
EPwm2Regs.CMPCTL.bit.SHDWAMODE = CC_IMMEDIATE;
EPwm2Regs.CMPA.bit.CMPA =0;
EPwm2Regs.TBPRD = EPWM2_TIMER_TBPRD; // Set timer period 801 TBCLKs
EPwm2Regs.TBPHS.bit.TBPHS = 0x0000; // Phase is 0
EPwm2Regs.TBCTR = 0x0000; // Clear counter
EPwm2Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;
EPwm2Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;
EPwm2Regs.DBCTL.bit.IN_MODE = DBA_ALL;
EPwm2Regs.DBRED.bit.DBRED = 5;
EPwm2Regs.DBFED.bit.DBFED = 5;
EPwm2Regs.TBCTL.bit.SYNCOSEL = 1;   // generate a syncout if CTR = 0
}
void InitEPwm6Example()
{
EPwm6Regs.ETSEL.bit.INTSEL = ET_CTR_ZERO; // hem zero hemde TBPRD zamaninda intrrupt aliyo.
EPwm6Regs.ETSEL.bit.INTEN = 1; // Enable INT
EPwm6Regs.ETPS.bit.INTPRD = ET_1ST; // Generate INT on 1st event
EPwm6Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN; // Count up and down
EPwm6Regs.TBCTL.bit.PHSEN = TB_DISABLE; // Disable phase loading
EPwm6Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1; // Clock ratio to SYSCLKOUT
EPwm6Regs.TBCTL.bit.CLKDIV = TB_DIV2;
EPwm6Regs.CMPCTL.bit.SHDWAMODE = CC_IMMEDIATE;
EPwm6Regs.CMPA.bit.CMPA =0;
EPwm6Regs.TBPRD = EPWM2_TIMER_TBPRD; // Set timer period 801 TBCLKs
EPwm6Regs.TBPHS.bit.TBPHS = 0x0000; // Phase is 0
EPwm6Regs.TBCTR = 0x0000; // Clear counter
EPwm6Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;
EPwm6Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;
EPwm6Regs.DBCTL.bit.IN_MODE = DBA_ALL;
EPwm6Regs.DBRED.bit.DBRED = 5;
EPwm6Regs.DBFED.bit.DBFED = 5;
EPwm6Regs.TBCTL.bit.SYNCOSEL = 1;   // generate a syncout if CTR = 0
}


void Setup_ePWM7(void)
  {

                      EPwm7Regs.TBPRD = 1250;                         // Set timer period
                      EPwm7Regs.TBPHS.bit.TBPHS = 0x0000;            // Phase is 0
                      EPwm7Regs.TBCTR = 0x0000;                       // Clear counter
                      // Setup TBCLK
                      EPwm7Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN; // Count up
                      EPwm7Regs.TBCTL.bit.PHSEN = TB_DISABLE;        // Disable phase loading
                      EPwm7Regs.TBCTL.bit.HSPCLKDIV = TB_DIV2;       // Clock ratio to SYSCLKOUT
                      EPwm7Regs.TBCTL.bit.CLKDIV = TB_DIV2;          // Slow so we can observe on the scope

                      // Setup compare
                      EPwm7Regs.CMPA.bit.CMPA = 0;
                      EPwm7Regs.CMPCTL.bit.SHDWAMODE = 0x0;        // enable shadow mode
                      EPwm7Regs.CMPCTL.bit.LOADAMODE = 0x2;        // Change CMPA for CTR = 0 or  CTR = PRD

                      // Set actions Qualifiers
                      EPwm7Regs.AQCTLA.bit.CAU = AQ_CLEAR;              // Set PWM7A on Zero
                      EPwm7Regs.AQCTLA.bit.CAD = AQ_SET;
                      EPwm7Regs.AQCTLB.bit.CAU = AQ_SET;            // Set PWM7B on Zero
                      EPwm7Regs.AQCTLB.bit.CAD = AQ_CLEAR;
                      //EPwm7Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;
                      //EPwm7Regs.DBCTL.bit.POLSEL = DB_ACTV_LOC;
                     // EPwm7Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;
                      //EPwm7Regs.DBCTL.bit.IN_MODE = DBA_ALL;
                      //EPwm7Regs.DBRED.bit.DBRED = 65;
                      //EPwm7Regs.DBFED.bit.DBFED = 65;
                      //EPwm7Regs.TBCTL.bit.SYNCOSEL = 1;   // generate a syncout if CTR = 0
                      //EPwm7_DB_Direction = DB_UP;
                      // Interrupt where we will change the deadband
                      //EPwm7Regs.ETSEL.bit.INTSEL = ET_CTR_ZERO; // Select INT on Zero event
                      //EPwm7Regs.ETSEL.bit.INTEN = 1; // Enable INT
                     // EPwm7Regs.ETPS.bit.INTPRD = ET_3RD; // Generate INT on 3rd event




}





