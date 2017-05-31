/**
 *****************************************************************************
   @example  Thermocouple_to_PWM.c
   @brief    This file expects a Thermocouple to be connected differentially to AIN2/AIN3 (J3 on the CN0319-EB1Z Demo board).
   - The RTD connected to AIN0/AIN1 will be used for Cold Junction compensation.
   - This file will measure the thermocouple/RTD inputs, convert this to an overall temperature.
   - This temperature is sent to the 4-20mA interface which is controlled by the PWM filtered output driving an external NPN transistor.
   - ADC0 is not used, therefore this example is directly  applicable to ADuCM361.

   - Thermocouple look-up tables assumes Type T thermocouple.
   - Temperature range is -200C to 350C.
   - Calibration options are included to calibrate the PWM output. Modify the define calibratePWM below to review the different options
   
   Baud rate of UART interface is 19200

   @version  V0.1
   @author   ADI
   @date     April 2013 

All files for ADuCM360/361 provided by ADI, including this file, are
provided  as is without warranty of any kind, either expressed or implied.
The user assumes any and all risk from the use of this code.
It is the responsibility of the person integrating this code into an application
to ensure that the resulting application performs as required and is safe.

**/


#include <stdio.h>
#include <string.h>
#include <ADuCM360.h>
#include "FlashEraseWrite.h"
#include "TempCalc.h"

#include <..\common\AdcLib.h>
#include <..\common\IexcLib.h>
#include <..\common\DacLib.h>
#include <..\common\UrtLib.h>
#include <..\common\ClkLib.h>
#include <..\common\WdtLib.h>
#include <..\common\PwmLib.h>
#include <..\common\DioLib.h>



void ADC1INIT(void);                // Init ADC1
void ADCERROR(void);
void UARTINIT (void);               // Enables UART
void DisUART(void);                 // Disable UART
void SendString(void);              // Transmit string using UART
void IEXCINIT(void);                // Setup Excitation Current sources
void DACINIT(void);                 // Init DAC for 1.2v Reference output
void PWMsetting(void);              
void delay(long int);								// Simple delay function
void SendResultToUART(void);        // Send measurement results to UART - in ASCII String format
void SendErrorToUART(void);         // Send Error String to UART to indicate PGA overrange occured
void Ioutfonction(float);           // Convert Final temperature value to 4mA-20mA output current
void CalibratePWM(void);            // Routine for calibrating PWM output.
void ADC1RTDCfg(void);              // RTD ADC1 settings
void ADC1ThermocoupleCfg(void);     // Thermocouple ADC1 settings

#define calibratePWM	2            // Set to 0 if you don't want to calibrate and just use default values
				     // Set to 1 if you want to calibrate externally and save to flash
				     // Set to 2 if you want to use the stored value from flash																	  // set to 2 if you want to load previously saved values from flash
																	 
#define THERMOCOUPLE 	0	     // Used for switching ADC0 to Thermocouple channel
#define RTD 		1	     // Used for switching ADC0 to RTD channel
#define DEFAULT20mA (594)           // PWM value that nominally gives 20mA at 10v
#define DEFAULT4mA (2422)           // PWM value that nominally gives 4mA at 10v
#define SAMPLENO  0x8


struct PWM_CAL
{   	
  unsigned long ul4mA_PWMCODE;    // PWM output code that generates 4mA 			   	
  unsigned long ul20mA_PWMCODE;   // PWM output code that generates 20mA	  
};

struct PWM_CAL PWM_Calibration;		    // Create instance for this parts calibration values
struct PWM_CAL *ptr_PWM_Calibration = &PWM_Calibration;		// Create pointer to lowest member of the structure.


volatile unsigned char bSendResultToUART = 0;   // Flag used to indicate ADC1 result ready to send to UART	
volatile unsigned char ucComRx = 0;
unsigned char ucCalComplete = 0;      // Flag used in PWM calibration routine

	
//Calculation varibales
float fVRTD = 0.0 ;                      // RTD voltage, 
float fRrtd = 0.0;                       // resistance of the RTD
float fTRTD = 0.0;                       // RTD temperature
float fVolts = 0.0;	                     // ADC to voltage constant
float fVThermocouple = 0.0;						   // thermoucouple voltage
float fColdJVolt = 0.0;		               // cold junction equivalent thermocouple voltage
float fFinalVoltage = 0.0;						   // fFinalVoltage = thermocouple voltage + cold j voltage
float fTThermocouple = 0.0;					     // thermoucouple temperature
float fFinalTemp = 0.0;								   // Final temperature including cold j compensation
//PWM variables
volatile float  Iout = 0.0;
volatile int PWMval = 6500;
volatile float CurrentInPWM = 0;	
int Error = 0;
volatile unsigned int uiTime2 = 0;        // Used for reading return value from PWMTime function
float minPwmVal = 0.0;
float maxPwmVal = 0.0;
float Stepsize = 0.0;
//ADC variables
volatile long ulADC1DATThermocouple[SAMPLENO];	// Variable that ADC1DAT is read into when sampling TC
volatile long ulADC1DATRtd[SAMPLENO];           // Variable that ADC0DAT is read into when sampling RTD
volatile unsigned char ucADCERR = 0;      // Used to indicate an ADC error
unsigned char ucADCInput;							// Used to indicate what channel the ADC1 is sampling
unsigned char ucSampleNo = 0;					// Used to keep track of when to switch channels
unsigned char ucCounter = 0;
// UART-based external variables
unsigned char ucTxBufferEmpty  = 0;       // Used to indicate that the UART Tx buffer is empty
unsigned char szTemp[64] = "";            // Used to store string before printing to UART
unsigned char nLen = 0;
unsigned char i = 0;
unsigned char ucWaitForUart = 0;          // Used by calibration routines to wait for user input
//flash
unsigned int uiFEESTA;
volatile unsigned char ucFlashCmdStatus = 0;
volatile unsigned char ucWaitForCmdToComplete = 0;
unsigned long ulAbortAddress = 0;
unsigned long ulSelectPage;						    // Used to store address of which page to erage
unsigned int uiArraySize = 0;					// size of array to be written to flash

int main (void)
{
  WdtCfg(T3CON_PRE_DIV1,T3CON_IRQ_EN,T3CON_PD_DIS); // Disable Watchdog timer resets
   //Disable clock to unused peripherals
   ClkDis(CLKDIS_DISSPI0CLK|CLKDIS_DISSPI1CLK|CLKDIS_DISI2CCLK|CLKDIS_DISDMACLK|CLKDIS_DIST0CLK|CLKDIS_DIST1CLK|CLKDIS_DISUARTCLK); // Only enable clock to used blocks
   ClkCfg(CLK_CD3,CLK_HF,CLKSYSDIV_DIV2EN,CLK_UCLKCG);
   ClkSel(CLK_CD7,CLK_CD7,CLK_CD7,CLK_CD0);     // Select CD0 for UART System clock and CD0 for PWM

   DioCfg(pADI_GP1,0x4000);                    // Setup P1.7 as PWM output
   PwmInit(UCLK_2,PWMCON0_PWMIEN_EN,PWMCON0_SYNC_DIS,PWMCON1_TRIPEN_DIS); //UCLK/2 to PWM, Enable IRQs
  
   ADC1INIT();                         // Setup ADC1
   IEXCINIT();                         // Setup Excitation current source
   DACINIT();                          // Setup DAC out at 1.2v for external reference
	
   uiTime2 = PwmTime(PWM4_5,0x3FFF,0x3FFF,PWMval);	  // 240Hz freq.      
   ucADCInput = THERMOCOUPLE;			                                //	Indicate that ADC1 is sampling thermocouple
	
   NVIC_EnableIRQ(PWM_PAIR2_IRQn);	    // Enable pair 2 IRQ 
   NVIC_EnableIRQ(FLASH_IRQn);          //Enable Flash interrupt 
   NVIC_EnableIRQ(ADC1_IRQn);           // Enable ADC1 interrupt
	  
   if (uiTime2 != 1) Error = 1;      // Error with PWM pair 2	
	  
   PwmGo(PWMCON0_ENABLE_EN,PWMCON0_MOD_DIS);      // Enable PWM outputs
   PWMsetting();
	 
   fVolts	= (1.2 / 268435456);	 
   Stepsize = (float)((maxPwmVal-minPwmVal)/16);    
   AdcGo(pADI_ADC1,ADCMDE_ADCMD_CONT); 
	 
   while (1)
   {
     if(bSendResultToUART == 1)	
     {
       fVRTD = 0.0;	  
       fVThermocouple = 0.0;		  		
      
       for (ucCounter = 0; ucCounter < SAMPLENO; ucCounter++)		
       {                            
         fVRTD += (((float)ulADC1DATRtd[ucCounter]) / 268435456);                // RTD voltage	in terms of reference voltage				 
         fVThermocouple += (((float)ulADC1DATThermocouple[ucCounter])*fVolts);   // Thermocouple voltage    
       }
			
      fVThermocouple = fVThermocouple/SAMPLENO;					                  // Get the average of the results			
      fVRTD = fVRTD/SAMPLENO;							
      fRrtd = fVRTD * 5600;											                          // RTD resistance			
      fTRTD =	CalculateRTDTemp(fRrtd);							                      // RTD temperature			
      fColdJVolt = CalculateColdJVoltage(fTRTD);				                  // get an equvalent thermocouple voltage			
      fFinalVoltage = fVThermocouple + fColdJVolt;			
      //fTThermocouple = CalculateThermoCoupleTemp(fVThermocouple);		
      fFinalTemp = CalculateThermoCoupleTemp(fFinalVoltage);	    
      Ioutfonction(fFinalTemp);                     //PWM out
       
      if (calibratePWM == 1)                           // Calibrate PWM and use these values  
     {
       SendResultToUART();                      // Send results to UART - enable uart first
       ADCERROR();                             // will generate an error if no thermocouple connected 	
    }
     
  bSendResultToUART = 0;
      			  
     }
   }
}

// Setup ADC1 to measure the RTD input
void ADC1RTDCfg(void)
{
	AdcPin(pADI_ADC1,ADCCON_ADCCN_AIN1,ADCCON_ADCCP_AIN0);                    // AIN0/AIN1 input channels
	AdcRng(pADI_ADC1,ADCCON_ADCREF_EXTREF,ADCMDE_PGA_G32,ADCCON_ADCCODE_INT); // External reference, G32 used
}
// Setup ADC1 to measure the Thermocouple input
void ADC1ThermocoupleCfg(void)
{
	AdcPin(pADI_ADC1,ADCCON_ADCCN_AIN3,ADCCON_ADCCP_AIN2);                    // Select AIn2/AIN3 as ADC inputs
	AdcRng(pADI_ADC1,ADCCON_ADCREF_INTREF,ADCMDE_PGA_G32,ADCCON_ADCCODE_INT); // Internal reference, Gain=32
}

void UARTINIT (void)
{
	 ClkDis(CLKDIS_DISSPI0CLK|CLKDIS_DISSPI1CLK|CLKDIS_DISI2CCLK|CLKDIS_DISDMACLK|CLKDIS_DIST0CLK|CLKDIS_DIST1CLK);
   ClkCfg(CLK_CD3,CLK_HF,CLKSYSDIV_DIV2EN,CLK_UCLKCG);
   ClkSel(CLK_CD7,CLK_CD7,CLK_CD0,CLK_CD0); 
   UrtCfg(pADI_UART,B9600*2,COMLCR_WLS_8BITS,0);   // setup baud rate for 19200, 8-bits
   UrtMod(pADI_UART,COMMCR_DTR,0);  			         // Setup modem bits
   UrtIntCfg(pADI_UART,COMIEN_ERBFI|COMIEN_ETBEI|COMIEN_ELSI|COMIEN_EDSSI|COMIEN_EDMAT|COMIEN_EDMAR);  // Setup UART IRQ sources
   DioPul(pADI_GP0,0xFF);								           // Enable pullup on P0.7/0.6
   DioCfg(pADI_GP0,0x3C);	
}

void DisUART (void)
{
	NVIC_DisableIRQ(UART_IRQn);
  pADI_CLKCTL->CLKDIS = 0x16F;
  ClkSel(CLK_CD7,CLK_CD7,CLK_CD7,CLK_CD0);
}

void SendString (void)
{
   for ( i = 0 ; i < nLen ; i++ )	// loop to send ADC0 result
   {
	  ucTxBufferEmpty = 0;
    UrtTx(pADI_UART,szTemp[i]);
    while (ucTxBufferEmpty == 0){}
   }
} 

void SendResultToUART(void)
{
    sprintf ( (char*)szTemp, "RTD Resistance: %fOhms \r\n",fRrtd );                          
    nLen = strlen((char*)szTemp);
    if (nLen <64)
            SendString();
    
    sprintf ( (char*)szTemp, "RTD Temperature: %fC \r\n",fTRTD );                         
    nLen = strlen((char*)szTemp);
    if (nLen <64)
            SendString();
    
    sprintf ( (char*)szTemp, "Cold Junction Voltage: %fmV \r\n",(fColdJVolt*1000) );// Send the Result to the UART                          
    nLen = strlen((char*)szTemp);
    if (nLen <64)
            SendString();
    
    sprintf ( (char*)szTemp, "Thermoucouple Voltage: %fmV \r\n",(fVThermocouple*1000) );// Send the Result to the UART                          
    nLen = strlen((char*)szTemp);
    if (nLen <64)
            SendString();

    sprintf ( (char*)szTemp, "Final Temperature: %fC \r\n",fFinalTemp );// Send the Result to the UART                          
    nLen = strlen((char*)szTemp);
    if (nLen <64)
            SendString();
}

void SendErrorToUART(void)
{
   sprintf ( (char*)szTemp, "ADC Overvoltage error on ADC1 PGA  \r\n");// Send error message to UART
   nLen = strlen((char*)szTemp);
   if (nLen <64)
      SendString();
}

void ADCERROR(void)
{
	 if (ucADCERR != 0)
		 {
		   if (ucADCERR == 1)
		   		sprintf ( (char*)szTemp, "ADC error on ADC0  \r\n");// Send error message to UART  
			 if (ucADCERR == 2)
		   		sprintf ( (char*)szTemp, "ADC error on ADC1  \r\n");// Send error message to UART
		   if ((ucADCERR == 1) | (ucADCERR == 2)) 
				 {	
					 nLen = strlen((char*)szTemp);
			  	 if (nLen <64)
		 			 SendString();
	       }
			ucADCERR = 0;
     }
}

void ADC1INIT(void)
{
	 AdcGo(pADI_ADC1,ADCMDE_ADCMD_IDLE);
	
	 AdcBias(pADI_ADC1,ADCCFG_PINSEL_AIN7,ADC_BIAS_X1,0);              //vbias ain7 buffers on
   AdcMski(pADI_ADC1,ADCMSKI_RDY,1);                                 // Enable ADC ready interrupt source		
   AdcFlt(pADI_ADC1,95,13,FLT_NORMAL|ADCFLT_NOTCH2|ADCFLT_CHOP|ADCFLT_RAVG2);    // ADC filter set for 5Hz update rate with chop on enabled
   AdcRng(pADI_ADC1,ADCCON_ADCREF_INTREF,ADCMDE_PGA_G32,ADCCON_ADCCODE_INT);  // Internal reference selected, Gain of 32, Signed integer output
   AdcBuf(pADI_ADC1,ADCCFG_EXTBUF_VREFPN,ADC_BUF_ON);                //External reference buffers on
   AdcPin(pADI_ADC1,ADCCON_ADCCN_AIN3,ADCCON_ADCCP_AIN2);             
}

void DACINIT(void)
{
   // Configure DAC output for 0-1.2V output range, Normal 12-bit mode and immediate update.
   DacCfg(DACCON_CLR_Off,DACCON_RNG_IntVref,DACCON_CLK_HCLK,DACCON_MDE_12bit);	  
   DacWr(0,0xFFF0000);                 // Output value 1.2V
	
}

void IEXCINIT(void)
{
   IexcCfg(IEXCCON_PD_off,IEXCCON_REFSEL_Int,IEXCCON_IPSEL1_Off,IEXCCON_IPSEL0_AIN6); //Setup IEXC for AIN6
	 IexcDat(IEXCDAT_IDAT_200uA,IDAT0Dis);         // Set output for 200uA
	
}

void Ioutfonction(float T)
{     
	    // Calculate the current and duty cycle needed  
      Iout = (T + 200)*0.029091;   	                    // -200degC : 4mA :: 16mA/550degC=0.029091
	    Iout = Iout + 4;
	    CurrentInPWM = -(Iout-20)*Stepsize;      
      PWMval = minPwmVal+CurrentInPWM;                
}

void CalibratePWM(void)
{
	
  volatile unsigned char ucEraseSuccess = 0;
	unsigned long *pWrite;
	
    // PWM Go Through Inverting Op Amp
    // Calibrate 20mA first
  minPwmVal = DEFAULT20mA;
	PWMval = minPwmVal;	
	ucCalComplete = 0;
	sprintf ( (char*)szTemp, "PWM Calibration Routine - calibrate to 20mA \r\n");                         
	nLen = strlen((char*)szTemp);
 	if (nLen <64)
		 	SendString();
	sprintf ( (char*)szTemp, "Press 1 to increase Output current - Press return when ready \r\n");                         
	nLen = strlen((char*)szTemp);
 	if (nLen <64)
		 	SendString();
	sprintf ( (char*)szTemp, "Press 0 to Decrease Output current - Press return when ready \r\n");                         
	nLen = strlen((char*)szTemp);
 	if (nLen <64)
		 	SendString();
	sprintf ( (char*)szTemp, "Press return when Complete - Press return when ready \r\n\n\n");                         
	nLen = strlen((char*)szTemp);
 	if (nLen <64)
		 	SendString();
	while (ucCalComplete == 0)
	{
		if (ucComRx == 0x31)                                    // Character "1" received, so increase output current
		{

			minPwmVal--;                                        // Current output increases when PWM duty cycle decreases 
			PWMval = minPwmVal;
			ucComRx = 0;
		
		}
		if (ucComRx == 0x30)                                    // Character "0" received, so Decrease output current
		{
			minPwmVal++;
      PWMval = minPwmVal;			                            // Current output decreases when PWM duty cycle increases
			ucComRx = 0;
		
		}

}
	
  // Calibrate 4mA 
	maxPwmVal = DEFAULT4mA;
  PWMval = maxPwmVal;  
  ucCalComplete = 0;
	sprintf ( (char*)szTemp, "PWM Calibration Routine - calibrate to 4mA \r\n");                         
	nLen = strlen((char*)szTemp);
 	if (nLen <64)
		 	SendString();
	sprintf ( (char*)szTemp, "Press 1 to increase Output current - Press return when ready \r\n");                         
	nLen = strlen((char*)szTemp);
 	if (nLen <64)
		 	SendString();
	sprintf ( (char*)szTemp, "Press 0 to Decrease Output current - Press return when ready \r\n");                         
	nLen = strlen((char*)szTemp);
 	if (nLen <64)
		 	SendString();
	sprintf ( (char*)szTemp, "Press return when Complete - Press return when ready \r\n");                         
	nLen = strlen((char*)szTemp);
 	if (nLen <64)
		 	SendString();
	while (ucCalComplete == 0)
	{
		if (ucComRx == 0x31)                                    // Character "1" received, so increase output current
		{

			maxPwmVal--;                                        // Current output increases when PWM duty cycle decreases 
			PWMval = maxPwmVal;
			ucComRx = 0;
	
		}
		if (ucComRx == 0x30)                                    // Character "0" received, so Decrease output current
		{
			maxPwmVal++;
			PWMval = maxPwmVal;                                  // Current output decreases when PWM duty cycle increases
			ucComRx = 0;
		
		}
	 
  }
 // Write the calibration values for the PWM to Flash page 0x1F000	
	ulSelectPage = 0x1F000;
	ucEraseSuccess = ErasePage(ulSelectPage);                     // Erase flash page used for calibrating the PWM
	pWrite = (unsigned long *)ptr_PWM_Calibration;
	ptr_PWM_Calibration->	ul4mA_PWMCODE = maxPwmVal;
  ptr_PWM_Calibration->	ul20mA_PWMCODE = minPwmVal;
  
	uiArraySize = sizeof(PWM_Calibration);
	WriteToFlash(pWrite,ulSelectPage,uiArraySize);
		
}

void delay (long int length)
{
	while (length >0)

    	length--;
}

void PWMsetting(void)
{
  if (calibratePWM == 0)                            // Use default values  
  {   
    minPwmVal = DEFAULT20mA;		                  // Use Default PWM output to generate 20mA output     
    maxPwmVal = DEFAULT4mA;                       // Use Default PWM output to generate 4mA output
  }
	
  if (calibratePWM == 1)                           // Calibrate PWM and use these values  
  {	   
    UARTINIT();                         // Init UART to 19200	
    NVIC_EnableIRQ(UART_IRQn);          // UART interrupt sources  
    CalibratePWM();	 
    //DisUART();                          // leave enable if results need to be sent via UART  
  }
  if (calibratePWM == 2)
  {	
    maxPwmVal = *( unsigned long *)0x0001F000;     // Use Pre-stored 4mA PWM calibration value
    minPwmVal = *( unsigned long *)0x0001F004;    // Use Pre-stored 20mA PWM calibration value
  }
}

void ADC1_Int_Handler ()
{
  volatile unsigned int uiADCSTA = 0;
  volatile long ulADC1DAT = 0;

  uiADCSTA = AdcSta(pADI_ADC1);
   if ((uiADCSTA & 0x10) == 0x10)			// Check for an error condition
   		ucADCERR = 2;
	ulADC1DAT = AdcRd(pADI_ADC1);
	if( bSendResultToUART == 0 )
	{
		if(ucSampleNo < SAMPLENO)
			{
			if (ucADCInput == THERMOCOUPLE){
				ulADC1DATThermocouple[ucSampleNo] = ulADC1DAT;}
			else{   
				ulADC1DATRtd[ucSampleNo] = ulADC1DAT;}
			  ucSampleNo++;	
	  	}
	  	else
			{
	   	ucSampleNo = 0;
			if (ucADCInput == THERMOCOUPLE){
				ADC1RTDCfg();
				ucADCInput = RTD;}
			else{
				ADC1ThermocoupleCfg();
				ucADCInput = THERMOCOUPLE;
				bSendResultToUART = 1;}
		  }
	 }
   
}

void UART_Int_Handler ()
{
  volatile unsigned char ucCOMSTA0 = 0;
	volatile unsigned char ucCOMIID0 = 0;
	
	ucCOMSTA0 = UrtLinSta(pADI_UART);			// Read Line Status register
	ucCOMIID0 = UrtIntSta(pADI_UART);			// Read UART Interrupt ID register
	if ((ucCOMIID0 & 0x2) == 0x2)	  			// Transmit buffer empty
	{
	  ucTxBufferEmpty = 1;
	}
	if ((ucCOMIID0 & 0x4) == 0x4)	  			// Receive byte
	{
		ucComRx	= UrtRx(pADI_UART);
		ucWaitForUart = 0;
		if (ucComRx == 0xD)                 // "Carriage return" detected
			ucCalComplete = 1;
	}
} 

void PWM2_Int_Handler()
{
 
 PwmClrInt(PWMCLRI_PWM2);
 PwmLoad(PWMCON0_LCOMP_EN);
 uiTime2 = PwmTime(PWM4_5,0x3FFF,0x3FFF,PWMval);
 if (uiTime2 != 1) Error = 1;//		Error with PWM pair 2	

}

void Flsh_Int_Handler ()
{
	uiFEESTA = 0;
	uiFEESTA = pADI_FEE->FEESTA;

	if ((uiFEESTA & 0x30) == 0x00)	// Command completed Successfully
	{
	   ucFlashCmdStatus = 0;			// Command passed
	}
	if ((uiFEESTA & 0x30) == 0x10)	// Error: Attempted erase of protected location
	{
	   ucFlashCmdStatus = 1;			// Command failed - protection error
	}
	if ((uiFEESTA & 0x30) == 0x20)	// Error: Sign error or, Erase error
	{
	   	   ucFlashCmdStatus = 2;	// Command failed - Sign/Erase error
	}
	if ((uiFEESTA & 0x30) == 0x30)	// Error: Command aborted.
	{
		 ulAbortAddress = pADI_FEE->FEEADRAH;
		 ulAbortAddress = (ulAbortAddress << 16);
		 ulAbortAddress |= pADI_FEE->FEEADRAL;
		 ucFlashCmdStatus = 3;			// Command failed - Command aborted before complete
	}
	if ((uiFEESTA & 0x8) == 0x8)		// Write Complete
	{
		 
	}
	if ((uiFEESTA & 0x4) == 0x4)		// Command Complete
	{
		 
	}
	ucWaitForCmdToComplete = 0;
}   

