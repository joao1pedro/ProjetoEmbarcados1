#include "hw_types.h"
#include "soc_AM335x.h"
#include "interrupt.h"
#include "gpio_v2.h"
#include "consoleUtils.h"
#include "uart_irda_cir.h"
#include "beaglebone.h"
#include "dmtimer.h"

/*****************************************************************************
**                INTERNAL MACRO DEFINITIONS
*****************************************************************************/
#define LOW														0
#define HIGH													1			

/*Definição pinos GPIO 1*/
#define PINO1_0                                                 0
#define PINO1_1                                                 1
#define PINO1_2                                                 2
#define PINO1_3                                                 3
#define PINO1_4                                                 4
#define PINO1_5                                                 5
#define PINO1_6                                                 6
#define PINO1_7                                                 7
#define PINO1_12                                                12 
#define PINO1_14                                                14
#define PINO1_16                                                16
#define PINO1_17                                                17

/* Values denoting the Interrupt Line number to be used. */
#define GPIO_INTC_LINE_1                                        (0x0)
#define GPIO_INTC_LINE_2                                        (0x1)

/*
** Values used to enable/disable interrupt generation due to level
** detection on an input GPIO pin.
*/
#define GPIO_INTC_TYPE_NO_LEVEL                                 (0x01)
#define GPIO_INTC_TYPE_LEVEL_LOW                                (0x04)
#define GPIO_INTC_TYPE_LEVEL_HIGH                               (0x08)
#define GPIO_INTC_TYPE_BOTH_LEVEL                               (0x0C)

/*
** Values used to enable/disable interrupt generation due to edge
** detection on an input GPIO pin.
*/
#define GPIO_INTC_TYPE_NO_EDGE                                  (0x80)
#define GPIO_INTC_TYPE_RISE_EDGE                                (0x10)
#define GPIO_INTC_TYPE_FALL_EDGE                                (0x20)
#define GPIO_INTC_TYPE_BOTH_EDGE                                (0x30)

#define TIMER_INITIAL_COUNT             (0xFF000000u)
#define TIMER_RLD_COUNT                 (0xFF000000u)
#define T_1MS_COUNT                     (0x5DC0u) 
#define OVERFLOW                        (0xFFFFFFFFu)
#define TIMER_1MS_COUNT                 (0x5DC0u) 

/*****************************************************************************
**                INTERNAL FUNCTION PROTOTYPES
*****************************************************************************/
static void initGame(void);
static void carrosBlink(void);
static void seqL(void);
static void seqR(void);
static void gameOver(void);
static void startLeds(void);
static void moveDireita(void);
static void moveEsquerda(void);
static void initLed(unsigned int, unsigned int,unsigned int);
static void initButton(unsigned int, unsigned int,unsigned int);
static void setupInterruptButton(void);
static void initSerial(void);
static void	gpioAintcConf(void);
static void gpioIsr(void);
static int	gpioPinIntConf(unsigned int, unsigned int, unsigned int);
static void gpioPinIntEnable(unsigned int, unsigned int, unsigned int); 
static void gpioIntTypeSet(unsigned int, unsigned int, unsigned int);
static void DMTimerSetUp(void);
static void Delay(volatile unsigned int);
static volatile unsigned int flagIsr;
//volatile unsigned int mSec = 1000;
char nome[25];
unsigned int score = 0;
/* 
 * ===  FUNCTION  ======================================================================
 *         Name:  main
 *  Deion:  
 * =====================================================================================
 */
int main(void){
	flagIsr = 0;
    
    /* Enable DMTimer clk */
    DMTimer2ModuleClkConfig();

    initSerial();
    /* Enable IRQ in CPSR */
	IntMasterIRQEnable();
  	
    /* Enable GPIO1 CLOCK */
    GPIOModuleClkConfig(GPIO1);
    /* Enable GPIO2 CLOCK */
    //GPIOModuleClkConfig(GPIO2);

    //faixa esquerda
    initLed(SOC_GPIO_1_REGS, 1, PINO1_0);
    initLed(SOC_GPIO_1_REGS, 1, PINO1_1);
    initLed(SOC_GPIO_1_REGS, 1, PINO1_2);
    initLed(SOC_GPIO_1_REGS, 1, PINO1_3);
    //faixa direita
    initLed(SOC_GPIO_1_REGS, 1, PINO1_4);
    initLed(SOC_GPIO_1_REGS, 1, PINO1_5);
    initLed(SOC_GPIO_1_REGS, 1, PINO1_6);
    initLed(SOC_GPIO_1_REGS, 1, PINO1_7);

    //inicia leds do carrinho
    initLed(SOC_GPIO_1_REGS, 1, PINO1_12); //carrinho inicio a esquerda
    initLed(SOC_GPIO_1_REGS, 1, PINO1_14); //carrinho direita

    //inicia botoes para mover o carrinho
    initButton(SOC_GPIO_1_REGS, 1, PINO1_16); //move esquerda
    initButton(SOC_GPIO_1_REGS, 1, PINO1_17); //move direita
        
    // ENABLE PIN TO INTERRUPT   	
	gpioAintcConf();
    
    /* Perform the necessary configurations for DMTimer */
    DMTimerSetUp();

    /* Register DMTimer2 interrupts on to AINTC */
    //DMTimerAintcConfigure();

    /* Enable the DMTimer interrupts */
    //DMTimerIntEnable(SOC_DMTIMER_2_REGS, DMTIMER_INT_OVF_EN_FLAG);

    initGame();

    setupInterruptButton();
    
    /*
    while(flagIsr){
        flag = 0;
        initGame();
    }
    */
	return(0);
} /* ----------  end of function main  ---------- */

/*FUNCTION*-------------------------------------------------------
*
* A function which is used to initialize GPIO like LED.
*END*-----------------------------------------------------------*/
static void initLed(unsigned int baseAdd, unsigned int module, unsigned int pin){
    
    	/* Selecting GPIO pin for use. */
    	GPIOPinMuxSetup(module, pin);
    
    	/* Setting the GPIO pin as an output pin. */
    	GPIODirModeSet(baseAdd, pin, GPIO_DIR_OUTPUT);
}

/*FUNCTION*-------------------------------------------------------
*
* A function which is used to initialize GPIO like BUTTON.
*END*-----------------------------------------------------------*/
static void initButton(unsigned int baseAdd, unsigned int module, unsigned int pin){
    
    	/* Selecting GPIO pin for use. */
    	GPIOPinMuxSetup(module, pin);
    
    	/* Setting the GPIO pin as an output pin. */
    	GPIODirModeSet(baseAdd, pin, GPIO_DIR_INPUT);
}

static void setupInterruptButton(){
    gpioPinIntConf(SOC_GPIO_1_REGS, GPIO_INTC_LINE_1, PINO1_16);
    gpioPinIntConf(SOC_GPIO_1_REGS, GPIO_INTC_LINE_1, PINO1_17);
   	gpioIntTypeSet(SOC_GPIO_1_REGS, PINO1_16, GPIO_INTC_TYPE_RISE_EDGE);
   	gpioIntTypeSet(SOC_GPIO_1_REGS, PINO1_17, GPIO_INTC_TYPE_RISE_EDGE);
}
/*FUNCTION*------------------------------------------------------
 *
 * A function which is used to initialize UART.
 *END*---------------------------------------------------------*/
 static void initSerial(){
    // initialize console for communication with the host machine
    ConsoleUtilsInit();
    
    // selecti the console type based on compile time check
    ConsoleUtilsSetType(CONSOLE_UART);
 }
/* inicia a partida */
static void initGame(){
    score = 0;
    ConsoleUtilsPrintf("\n\r##############################\n\r");
    ConsoleUtilsPrintf("\r##### TRAFFIC BLOCK GAME  #####\n\r");
    ConsoleUtilsPrintf("\r##############################\n\r");
	ConsoleUtilsPrintf("Digite o nome do jogador: \n");
    ConsoleUtilsScanf("%s", nome);
    startLeds();
}
/* Move o carro(led que o jogador controla) p/ a esquerda */
static void moveEsquerda(){
    GPIOPinWrite(SOC_GPIO_1_REGS, PINO1_12, HIGH); //1 a esquerda
    GPIOPinWrite(SOC_GPIO_1_REGS, PINO1_14, LOW); // 0 a direita
    ConsoleUtilsPrintf("Moveu pra esquerda \n");
}
/* Move o carro(led que o jogador controla) p/ a direita */
static void  moveDireita(){
    GPIOPinWrite(SOC_GPIO_1_REGS, PINO1_12, LOW); //0 a esqueda
    GPIOPinWrite(SOC_GPIO_1_REGS, PINO1_14, HIGH); //1 a direita
    ConsoleUtilsPrintf("Moveu pra direita \n");
}
/* Inicia os leds */
static void startLeds(){
    GPIOPinWrite(SOC_GPIO_1_REGS, PINO1_0, LOW);
    GPIOPinWrite(SOC_GPIO_1_REGS, PINO1_1, LOW);
    GPIOPinWrite(SOC_GPIO_1_REGS, PINO1_2, LOW);
    GPIOPinWrite(SOC_GPIO_1_REGS, PINO1_3, LOW);
    GPIOPinWrite(SOC_GPIO_1_REGS, PINO1_4, LOW);
    GPIOPinWrite(SOC_GPIO_1_REGS, PINO1_5, LOW);
    GPIOPinWrite(SOC_GPIO_1_REGS, PINO1_6, LOW);
    GPIOPinWrite(SOC_GPIO_1_REGS, PINO1_7, LOW);
    Delay(1000); //1 segundo para começar a acender os leds
    //chama função p/acender aleatoriamente
    carrosBlink();
}
/* Faz carros da faixa a esquerda irem em direção
 * ao carro do jogador */
static void seqL(){
    /* sequencia a esquerda */
    GPIOPinWrite(SOC_GPIO_1_REGS, PINO1_0, HIGH);
    Delay(1000);
    GPIOPinWrite(SOC_GPIO_1_REGS, PINO1_0, LOW);
    GPIOPinWrite(SOC_GPIO_1_REGS, PINO1_1, HIGH);
    Delay(1000);
    GPIOPinWrite(SOC_GPIO_1_REGS, PINO1_1, LOW);
    GPIOPinWrite(SOC_GPIO_1_REGS, PINO1_2, HIGH);
    Delay(1000);
    GPIOPinWrite(SOC_GPIO_1_REGS, PINO1_2, LOW);
    GPIOPinWrite(SOC_GPIO_1_REGS, PINO1_3, HIGH);
    Delay(1000);
    GPIOPinWrite(SOC_GPIO_1_REGS, PINO1_3, LOW);
    if(GPIOPinRead(SOC_GPIO_1_REGS, PINO1_12)) //batida
        gameOver();
    GPIOPinWrite(SOC_GPIO_1_REGS, PINO1_12, HIGH);
    score++;
    Delay(1000);
    GPIOPinWrite(SOC_GPIO_1_REGS, PINO1_12, LOW);
}
/* Faz os carros da faixa a direita irem em direção
 * ao carro do jogador */
static void seqR(){
    /* sequencia a direita*/
    GPIOPinWrite(SOC_GPIO_1_REGS, PINO1_4, HIGH);
    Delay(1000);
    GPIOPinWrite(SOC_GPIO_1_REGS, PINO1_4, LOW);
    GPIOPinWrite(SOC_GPIO_1_REGS, PINO1_5, HIGH);
    Delay(1000);
    GPIOPinWrite(SOC_GPIO_1_REGS, PINO1_5, LOW);
    GPIOPinWrite(SOC_GPIO_1_REGS, PINO1_6, HIGH);
    Delay(1000);
    GPIOPinWrite(SOC_GPIO_1_REGS, PINO1_6, LOW);
    GPIOPinWrite(SOC_GPIO_1_REGS, PINO1_7, HIGH);
    Delay(1000);
    GPIOPinWrite(SOC_GPIO_1_REGS, PINO1_7, LOW);
    if(GPIOPinRead(SOC_GPIO_1_REGS, PINO1_14)) //batida
        gameOver();
    GPIOPinWrite(SOC_GPIO_1_REGS, PINO1_14, HIGH);
    score++;
    Delay(1000);
    GPIOPinWrite(SOC_GPIO_1_REGS, PINO1_14, LOW);
}
/* Mostrar placar do jogador e reinicia o jogo após 10 segundos
 */
static void gameOver(void){
    ConsoleUtilsPrintf("\n\r##############################\n\r");
    ConsoleUtilsPrintf("\r##### GAME OVER  #####\n\r");
    ConsoleUtilsPrintf("\r##############################\n\r");
    ConsoleUtilsPrintf("\nJogador: %s", nome);
    ConsoleUtilsPrintf("\nScore: %d", score);
    Delay(10000);
    initGame();
}
/* Faz um padrão de leds acenderem nas faixas
 * para ir em direção ao jogador */
static void carrosBlink(void){
    int sequencia = 0;
    if(sequencia == 0){
        seqL();
        Delay(2000);
        seqR();
        Delay(1000);
        seqL();
        Delay(1000);
        seqR();
        Delay(1000);
        seqL();
        Delay(500);
        sequencia++;
    }
    else if(sequencia == 1){
        seqL();
        Delay(1000);
        seqL();
        Delay(1000);
        seqR();
        Delay(2000);
        seqL();
        Delay(500);
        sequencia++;
    }
    else if(sequencia == 2){
        seqR();
        Delay(1000);
        seqL();
        Delay(2000);
        seqL();
        Delay(500);
        seqR();
        Delay(700);
        sequencia++;
    }
    else if(sequencia == 3){
        seqL();
        Delay(1000);
        seqL();
        Delay(500);
        seqR();
        Delay(1500);
        seqL();
        Delay(800);
        sequencia++;
    }
    else if(sequencia == 4){
        seqR();
        Delay(2000);
        seqL();
        Delay(700);
        seqL();
        Delay(200);
        seqR();
        Delay(1000);
        sequencia++;
    }
    else if(sequencia == 5){
        seqL();
        Delay(800);
        seqL();
        Delay(300);
        seqR();
        Delay(900);
        seqR();
        Delay(1000);
        sequencia++;
    }
    else if(sequencia == 6){
        seqL();
        Delay(850);
        seqR();
        Delay(300);
        seqR();
        Delay(250);
        seqR();
        Delay(500);
        sequencia++;
    }
    else if(sequencia == 7){
        seqL();
        Delay(850);
        seqR();
        Delay(300);
        seqR();
        Delay(250);
        seqR();
        Delay(500);
        sequencia++;
    }
    else if(sequencia == 8){
        seqL();
        Delay(600);
        seqL();
        Delay(300);
        seqR();
        Delay(900);
        seqL();
        Delay(800);
        sequencia++;
    }
    else{
        seqR();
        Delay(800);
        seqR();
        Delay(1000);
        seqL();
        Delay(1000);
        seqR();
        Delay(1000);
        sequencia = 1;
    }
}
/*FUNCTION*-------------------------------------------------------
*
* Function Name : gpioAintcconfigure
* Comments      : Do the necessary gpio configurations on to AINTC.
*END*-----------------------------------------------------------*/
static void gpioAintcConf(void){

    /* Initialize the ARM interrupt control */
    IntAINTCInit();
 
    /* Registering gpioIsr */
    IntRegister(SYS_INT_GPIOINT1A, gpioIsr);
    
    /* Set the priority */
    IntPrioritySet(SYS_INT_GPIOINT1A, 0, AINTC_HOSTINT_ROUTE_IRQ);
    
    /* Enable the system interrupt */
    IntSystemEnable(SYS_INT_GPIOINT1A);
   
}

/*FUNCTION*-------------------------------------------------------
*
* Function Name : gpioIsr
* Comments      : DMTimer interrupt service routine. This will 
* send a character to serial.
*END*-----------------------------------------------------------*/    
static void gpioIsr(void){
	//flagIsr = 1;
    if(GPIOPinRead(SOC_GPIO_1_REGS, PINO1_16))
        moveEsquerda();
    else if(GPIOPinRead(SOC_GPIO_1_REGS, PINO1_17))
        moveDireita();
    /*	Clear wake interrupt	*/
	//HWREG(SOC_GPIO_1_REGS + 0x3C) = 0x1000;
	//HWREG(SOC_GPIO_1_REGS + 0x34) = 0x1000;
	//ConsoleUtilsPrintf("\nAPERTOU\n");
	//HWREG(SOC_GPIO_1_REGS + 0x2C) = 0x10000;
	HWREG(SOC_GPIO_1_REGS + 0x2C) = 0xFFFFFFFF;
}

/*FUNCTION*-------------------------------------------------------
*
* Function Name : gpioPinIntConfig
* Comments      :
*END*-----------------------------------------------------------*/
static int gpioPinIntConf(unsigned int baseAdd, unsigned int intLine,
                  unsigned int pinNumber){

    	/* Setting interrupt GPIO pin. */
    	gpioPinIntEnable(baseAdd, intLine, pinNumber);
    
    	return(0);
}

/*FUNCTION*-------------------------------------------------------
*
* Function Name : GPIOPinIntEnable
* Comments      : This API enables the configured interrupt event on a specified input
* GPIO pin to trigger an interrupt request.
*
* \param  baseAdd     The memory address of the GPIO instance being used
* \param  intLine     This specifies the interrupt request line on which the
*                     interrupt request due to the transitions on a specified
*                     pin be propagated
* \param  pinNumber   The number of the pin in the GPIO instance
*
* 'intLine' can take one of the following two values:
* - GPIO_INT_LINE_1 - interrupt request be propagated over interrupt line 1\n
* - GPIO_INT_LINE_2 - interrupt request be propagated over interrupt line 2\n
* 
* 'pinNumber' can take one of the following values:
* (0 <= pinNumber <= 31)\n
*
* \return None
*
*END*-----------------------------------------------------------*/
static void gpioPinIntEnable(unsigned int baseAdd,
                      unsigned int intLine,
                      unsigned int pinNumber){
    if(GPIO_INTC_LINE_1 == intLine){
        HWREG(baseAdd + GPIO_IRQSTATUS_SET(0)) = (1 << pinNumber);
    }else{
        HWREG(baseAdd + GPIO_IRQSTATUS_SET(1)) = (1 << pinNumber);
    }
}

/*FUNCTION*-------------------------------------------------------
*
* Function Name : gpioAintcconfigure
* Comments      : This API configures the event type for a specified 
* input GPIO pin. Whenever the selected event occurs on that GPIO pin 
* and if interrupt generation is enabled for that pin, the GPIO module 
* will send an interrupt to CPU.
*
* \param  baseAdd    The memory address of the GPIO instance being used
* \param  pinNumber  The number of the pin in the GPIO instance
* \param  eventType  This specifies the event type on whose detection,
*                    the GPIO module will send an interrupt to CPU,
*                    provided interrupt generation for that pin is enabled.
*
* 'pinNumber' can take one of the following values:
* (0 <= pinNumber <= 31)\n
*
* 'eventType' can take one of the following values:
* - GPIO_INT_TYPE_NO_LEVEL - no interrupt request on occurence of either a
*   logic LOW or a logic HIGH on the input GPIO pin\n
* - GPIO_INT_TYPE_LEVEL_LOW - interrupt request on occurence of a LOW level
*   (logic 0) on the input GPIO pin\n
* - GPIO_INT_TYPE_LEVEL_HIGH - interrupt request on occurence of a HIGH level
*   (logic 1) on the input GPIO pin\n
* - GPIO_INT_TYPE_BOTH_LEVEL - interrupt request on the occurence of both the
*   LOW level and HIGH level on the input GPIO pin\n
* - GPIO_INT_TYPE_NO_EDGE -  no interrupt request on either rising or
*   falling edges on the pin\n
* - GPIO_INT_TYPE_RISE_EDGE - interrupt request on occurence of a rising edge
*   on the input GPIO pin\n
* - GPIO_INT_TYPE_FALL_EDGE - interrupt request on occurence of a falling edge
*   on the input GPIO pin\n
* - GPIO_INT_TYPE_BOTH_EDGE - interrupt request on occurence of both a rising
*   and a falling edge on the pin\n
*
* \return  None
*
* \note  A typical use case of this API is explained below:
* 
*        If it is initially required that interrupt should be generated on a
*        LOW level only, then this API can be called with
*        'GPIO_INT_TYPE_LEVEL_LOW' as the parameter.
*        At a later point of time, if logic HIGH level only should be made as
*        the interrupt generating event, then this API has to be first called
*        with 'GPIO_INT_TYPE_NO_LEVEL' as the parameter and later with
*        'GPIO_INT_TYPE_LEVEL_HIGH' as the parameter. Doing this ensures that
*        logic LOW level trigger for interrupts is disabled.
*END*-----------------------------------------------------------*/
static void gpioIntTypeSet(unsigned int baseAdd,
                    unsigned int pinNumber,
                    unsigned int eventType){
    eventType &= 0xFF;

    switch(eventType)
    {

        case GPIO_INT_TYPE_NO_LEVEL:

            /* Disabling logic LOW level detect interrupt generation. */
            HWREG(baseAdd + GPIO_LEVELDETECT(0)) &= ~(1 << pinNumber);

            /* Disabling logic HIGH level detect interrupt generation. */
            HWREG(baseAdd + GPIO_LEVELDETECT(1)) &= ~(1 << pinNumber);

        break;

        case GPIO_INT_TYPE_LEVEL_LOW:

            /* Enabling logic LOW level detect interrupt geenration. */
            HWREG(baseAdd + GPIO_LEVELDETECT(0)) |= (1 << pinNumber);

            /* Disabling logic HIGH level detect interrupt generation. */
            HWREG(baseAdd + GPIO_LEVELDETECT(1)) &= ~(1 << pinNumber);

            /* Disabling rising edge detect interrupt generation. */
            HWREG(baseAdd + GPIO_RISINGDETECT) &= ~(1 << pinNumber);

            /* Disabling falling edge detect interrupt generation. */
            HWREG(baseAdd + GPIO_FALLINGDETECT) &= ~(1 << pinNumber);

        break;

        case GPIO_INT_TYPE_LEVEL_HIGH:

            /* Disabling logic LOW level detect interrupt generation. */
            HWREG(baseAdd + GPIO_LEVELDETECT(0)) &= ~(1 << pinNumber);

            /* Enabling logic HIGH level detect interrupt generation. */
            HWREG(baseAdd + GPIO_LEVELDETECT(1)) |= (1 << pinNumber);

            /* Disabling rising edge detect interrupt generation. */
            HWREG(baseAdd + GPIO_RISINGDETECT) &= ~(1 << pinNumber);

            /* Disabling falling edge detect interrupt generation. */
            HWREG(baseAdd + GPIO_FALLINGDETECT) &= ~(1 << pinNumber);
        
        break;

        case GPIO_INT_TYPE_BOTH_LEVEL:
            
            /* Enabling logic LOW level detect interrupt geenration. */
            HWREG(baseAdd + GPIO_LEVELDETECT(0)) |= (1 << pinNumber);

            /* Enabling logic HIGH level detect interrupt generation. */
            HWREG(baseAdd + GPIO_LEVELDETECT(1)) |= (1 << pinNumber);

            /* Disabling rising edge detect interrupt generation. */
            HWREG(baseAdd + GPIO_RISINGDETECT) &= ~(1 << pinNumber);

            /* Disabling falling edge detect interrupt generation. */
            HWREG(baseAdd + GPIO_FALLINGDETECT) &= ~(1 << pinNumber);
            
        break;

        case GPIO_INT_TYPE_NO_EDGE:
            
            /* Disabling rising edge detect interrupt generation. */
            HWREG(baseAdd + GPIO_RISINGDETECT) &= ~(1 << pinNumber);

            /* Disabling falling edge detect interrupt generation. */
            HWREG(baseAdd + GPIO_FALLINGDETECT) &= ~(1 << pinNumber);

        break;

        case GPIO_INT_TYPE_RISE_EDGE:

            /* Enabling rising edge detect interrupt generation. */
            HWREG(baseAdd + GPIO_RISINGDETECT) |= (1 << pinNumber);

            /* Disabling falling edge detect interrupt generation. */
            HWREG(baseAdd + GPIO_FALLINGDETECT) &= ~(1 << pinNumber);

            /* Disabling logic LOW level detect interrupt generation. */
            HWREG(baseAdd + GPIO_LEVELDETECT(0)) &= ~(1 << pinNumber);

            /* Disabling logic HIGH level detect interrupt generation. */
            HWREG(baseAdd + GPIO_LEVELDETECT(1)) &= ~(1 << pinNumber);

        break;

        case GPIO_INT_TYPE_FALL_EDGE:

            /* Disabling rising edge detect interrupt generation. */
            HWREG(baseAdd + GPIO_RISINGDETECT) &= ~(1 << pinNumber);

            /* Enabling falling edge detect interrupt generation. */
            HWREG(baseAdd + GPIO_FALLINGDETECT) |= (1 << pinNumber);

            /* Disabling logic LOW level detect interrupt generation. */
            HWREG(baseAdd + GPIO_LEVELDETECT(0)) &= ~(1 << pinNumber);

            /* Disabling logic HIGH level detect interrupt generation. */
            HWREG(baseAdd + GPIO_LEVELDETECT(1)) &= ~(1 << pinNumber);

        break;

        case GPIO_INT_TYPE_BOTH_EDGE:

            /* Enabling rising edge detect interrupt generation. */
            HWREG(baseAdd + GPIO_RISINGDETECT) |= (1 << pinNumber);

            /* Enabling falling edge detect interrupt generation. */
            HWREG(baseAdd + GPIO_FALLINGDETECT) |= (1 << pinNumber);

            /* Disabling logic LOW level detect interrupt generation. */
            HWREG(baseAdd + GPIO_LEVELDETECT(0)) &= ~(1 << pinNumber);

            /* Disabling logic HIGH level detect interrupt generation. */
            HWREG(baseAdd + GPIO_LEVELDETECT(1)) &= ~(1 << pinNumber);

        break;

        default:
        break;
    }
}

static void DMTimerSetUp(void){
    DMTimerReset(SOC_DMTIMER_2_REGS);
    DMTimerModeConfigure(SOC_DMTIMER_2_REGS, DMTIMER_AUTORLD_NOCMP_ENABLE);
}
static void Delay(volatile unsigned int mSec){
   while(mSec != 0){
        DMTimerCounterSet(SOC_DMTIMER_2_REGS, 0);
        DMTimerEnable(SOC_DMTIMER_2_REGS);
        while(DMTimerCounterGet(SOC_DMTIMER_2_REGS) < TIMER_1MS_COUNT);
        DMTimerDisable(SOC_DMTIMER_2_REGS);
        mSec--;
    }
}
