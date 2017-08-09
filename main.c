/* Title: SASB Single Axis FLoator Test Code
 * Author: Tyler Larson
 * Purpose: Through use of a PID controller a current sensor and a displacement sensor, the code controls both axis of the top bearings of the flywheel.
 */

/*
 * Includes
 */

#include "F28x_Project.h"   //Seems like a nice header file, includes a lot of things so I don't have to

/*
 * Defines
 */

//Sets the size of the array for the Integral and Derivative previous values
#define prev_size   64  //Size of the Memory array in the PID control

//Sets the number of Sysclk cycles that the ADC samples for
#define sample_cycles (20 - 1)  //Note* only change the first number, the 1 accounts for the minimum

//Current Bias current for the SADB to help stabilize
#define current_bias .5   //Set arbitrarily as a middle point in the range of currents we anticipate to use

//Current sensor conversion scale       *Calibrated as of 2017/07/20
#define current_scale (10.168070782 * 3.3 / 4096)//Conversion scale for current sensor *(Amps/Volt)

//Current sensor conversion offset      *Calibrated as of 2017/07/20
#define current_offset -13.45198481 //Conversion offset for current sensor *(Amps)

//Displacement sensor conversion scale
#define disp_scale = .9540446743 * 3.3 / 4096   //Conversion scale for displacement sensor *(Millimeters/Volt)

//Displacement sensor conversion offset
#define disp_offset = 0  //Conversion offset for displacement sensor *(Millieters)

//Displacement sensor sample period
#define x1_dt 0.1 //*(MilliSeconds)

//Displacement sensor target
#define x1_target 1   //Target float level *(Millimeters)

//PID constants
#define kp_init .5 //Default to 1 for Initial tuning
#define ki_init 0  //Default to 0 for initial tuning
#define kd_init 0  //Default to 0 for initial tuning

//State Transition States Enumeration
typedef enum SPI_state {Idle_state,
                        Sampling_state,
                        Start_state,
                        Null_state,
                        Data_state,
                        Error_state,
                        Warning_state,
                        CRC_state,
                        Ignore_state
}SPI_state;


/*
 * Structures
 */

/*! Position sensor structure */
typedef struct position_sensor_struct{
    int *sample_loc;        /*!< Sample value pointer */
    float sample;           /*!< Sample value, *(mm) */
    float target;           /*!< Target Displacement value *(mm) */
    float error;            /*!< Error for PID controls *(mm) */
    float scale;            /*!< Sensor specific scale value */
    float offset;           /*!< Sensor specific offset value */
    float dt;               /*!< Time period for PID control *(ms) */
    float kp;               /*!< Proportional Constant */
    float p;                /*!< Proportional calculated value */
    float ki;               /*!< Integral Constant */
    float i;                /*!< Integral calculated value */
    float kd;               /*!< Derivative Constant */
    float d;                /*!< Derivative calculated value */
    float pid_out;          /*!< Output of the PID control */
    int prev_place;         /*!< Current location in the array of previous values */
    int prev_last;          /*!< Last location in the array, used for Derivative term */
    float prev[prev_size];  /*!< Array of previous values, used for calculating the Derivative and Integral terms */
}position;

/*! Current Sensor and Coil Control Structure */
typedef struct current_coil_struct{
    int *sample_loc;        /*!< Sample value pointer */
    float sample;           /*!< Sample value, (*Amps) */
    float target;           /*!< Target Displacement value, *(Amps) */
    float error;            /*!< Error to determine more or less current, *(Amps)   **May change later for PWM and/or a PID loop */
    float scale;            /*!< Scaling constant to convert sensor reading to current, *(Amps/Volt) */
    float offset;           /*!< Offset constant to convert sensor reading to current, *(Amps) */
    float x_influence;      /*!< X-axis influence of the coil */
    float y_influence;      /*!< Y-axis influence of the coil */
    float bias;             /*!< Individual bias current of the coil **May use in Gauss/De-Gauss process in the future */
    int gpio_offset;        /*!< Offset in the register to the correct GPIO pin */
}current;

/*
 * Function Prototypes
 */
    //*Note, at some point this should be moved to a header file

void SetupPosition(position *sensor);
void SetupCoil(current *coil);
void Position_PID_Cntrl(position *sensor);
void current_target(current *coil,position *x_disp_sensor,position *y_disp_sensor);
void Bang_Bang_Cntrl(current *coil);
void InitADCPart1();
void InitADCPart2();
void InitEPwm();
void PIEMap();
void EPwmStart();
int Check_CRC(uint32_t CRC_check, int CRC_value);
interrupt void adca1_isr();
interrupt void adcb2_isr();
interrupt void epwm3_isr();

/*
 * Globals
 */

//Definition of the X1 instance of the position sensor structure
position X1;    //Variable used to store the PID math variables for the x axis displacement sensor
position Y1;    //Variable used to store the PID math variables for the y axis displacement sensor
position X2;
position Y2;

//Definition of the current coil structure
current C1;     //Variable used to store the current control variables for coil 1
current C2;     //Variable used to store the current control variables for coil 2
current C3;     //Variable used to store the current control variables for coil 3
current C4;     //Variable used to store the current control variables for coil 4

//Displacement sensor reading
int x1_sample;  //Global for x1 displacement sensor reading *(Digitized)
int y1_sample;  //Global for y1 displacement sensor reading *(Digitized)
int x2_sample;
int y2_sample;

//Cutoff value for X displacement
    //*Used for Debug
int x1_cutoff = 0;  //Debuging purposes, clean out later

//Displacement sensor update flag
    /* Update name to account for more than just x being updated */
int x1_update;  //Flag for new displacement sensor readings



//Displacement to current ratio
    //*Make this a define
    //*First add it into the displacement struct
    //*Should this change in SADB/DADB?
float disp_to_current = 2.2;  //Scales the PID output to current *(Meters/Amp)

//Current sensors readings
int c1_sample;  //Global for coil 1 current sensor reading *(Digitized)
int c2_sample;  //Global for coil 2 current sensor reading *(Digitized)
int c3_sample;  //Global for coil 3 current sensor reading *(Digitized)
int c4_sample;  //Global for coil 4 current sensor reading *(Digitized)

//Current sensor update flag
int c_update;  //Flag for new current sensor readings

//Maximum Current
float current_max = 10;  //Max current for operation *(Amps)

//Minimum Current
float current_min = -10; //Min current for operation *(Amps)

//Rotor diameter deviation variables
float xmax = 0;
float xmin = 3;
float xdelta;
float ymax = 0;
float ymin = 3;
float ydelta;

//SPI State Variables
SPI_state c_state = Idle_state; //Variable for the current state
SPI_state n_state = Idle_state;  //Variable for the next state

//SPI Storage Variables
uint32_t CRC_check;
float Data_value;
bool Error_flag = 0;
int Warning_flag = 0;
int CRC_value = 0;
bool CRC_result = 0;
int Data_count = 0;
int CRC_count = 0;
bool miso;
float Rotation_percent;
float Max_value = 0x3ffff;

/*
 * Main
 */

void main(void){
    //Main Initialization function for the 77D
    InitSysCtrl();

    //Setup the GPIO pins
    InitGpio();

    //Setup output pins
    EALLOW;
    
    //Pin 89, Used to clock how long the displacement if statement in the main loop takes
    GpioCtrlRegs.GPBPUD.bit.GPIO40 = 0;     //Leaves pull up resistor on pin
    GpioDataRegs.GPBSET.bit.GPIO40 = 1;     //Sets the pin high
    GpioCtrlRegs.GPBGMUX1.bit.GPIO40 = 0;   //Sets the pins to default mux
    GpioCtrlRegs.GPBDIR.bit.GPIO40 = 1;     //Sets the pin to output
    GpioDataRegs.GPBCLEAR.bit.GPIO40 = 1;   //Sets the pin low

    //MISO Pin for Rotational Encoder
    GpioCtrlRegs.GPBPUD.bit.GPIO41 = 0;
    GpioCtrlRegs.GPBGMUX1.bit.GPIO41 = 0;
    GpioCtrlRegs.GPBDIR.bit.GPIO41 = 0;

    //Current State Output Pins
    GpioCtrlRegs.GPBDIR.all = (uint32_t)0x7 << 16;    //GPIO 48,49 and 50
    GpioDataRegs.GPACLEAR.all = (uint32_t)0x7 << 16;

    //PWM_H Pins
        //Pin 86, PWM_H for C1
    GpioCtrlRegs.GPBPUD.bit.GPIO39 = 0;     //Leaves the pull up resistor on the pin
    GpioDataRegs.GPBSET.bit.GPIO39 = 1;     //Sets the pin high
    GpioCtrlRegs.GPBGMUX1.bit.GPIO39 = 0;   //Sets the pin to default mux
    GpioCtrlRegs.GPBDIR.bit.GPIO39 = 1;     //Sets the pin to output
    GpioDataRegs.GPBCLEAR.bit.GPIO39 = 1;   //Sets the pin low
        //Pin 88, PWM_H for C2
    GpioCtrlRegs.GPBGMUX1.bit.GPIO34 = 0;   //Sets the pin to default mux
    GpioCtrlRegs.GPBDIR.bit.GPIO34 = 1;     //Sets the pin to output
    GpioDataRegs.GPBCLEAR.bit.GPIO34 = 1;   //Sets the pin to low
        //Pin 90, PWM_H for C3
    GpioCtrlRegs.GPBDIR.bit.GPIO44 = 1;     //Sets the pin to output
    GpioDataRegs.GPBCLEAR.bit.GPIO44 = 1;   //Sets the pin to low
        //Pin 92, PWM_H for C4
    GpioCtrlRegs.GPBDIR.bit.GPIO45 = 1;     //Sets the pin to output
    GpioDataRegs.GPBCLEAR.bit.GPIO45 = 1;   //Sets the pin to low
    
    EDIS;

    //Enable output of ePWM signals to Pins
    InitEPwm1Gpio();
    InitEPwm2Gpio();
    InitEPwm3Gpio();

    //Setup X1 variable
    SetupPosition(&X1);
    SetupPosition(&Y1);
    SetupPosition(&X2);
    SetupPosition(&Y2);
    
    //Sets the location of the global for each sample
    X1.sample_loc = &x1_sample;
    Y1.sample_loc = &y1_sample;
    X2.sample_loc = &x2_sample;
    Y2.sample_loc = &y2_sample;

    //Sets the target for X1 and Y1
    X1.target = 1.6;
    Y1.target = 1.5;

    //Sets the correct offset and scaling factor for each displacement sensor
    X1.scale = 0.00076433121;
    X1.offset = 0;
    Y1.scale = 0.00077299665;
    Y1.offset = 0;
    X2.scale = 0.000768639508;
    X2.offset = 0;
    Y2.scale = 0.000807102502;
    Y2.offset = 0;

    //Setup C1 variable
    SetupCoil(&C1);
    SetupCoil(&C2);
    SetupCoil(&C3);
    SetupCoil(&C4);
    
    //Sets the location of the global for each sample
    C1.sample_loc = &c1_sample;
    C2.sample_loc = &c2_sample;
    C3.sample_loc = &c3_sample;
    C4.sample_loc = &c4_sample;
    
    //Displacement sensor influences for each coil
    C1.x_influence = -1;
    C2.x_influence = 1;
    C3.y_influence = 1;
    C4.y_influence = -1;

    //GPIO offsets
        //These are found by mapping between the physical pins and their GPIO address in the GPIO registers
    C1.gpio_offset = 2;
    C2.gpio_offset = 7;
    C3.gpio_offset = 12;
    C4.gpio_offset = 13;

    //Disable interrupts? Followed the example of process from the control suite example code
    DINT;

    //Initialize PIE control registers
    InitPieCtrl();

    //Disable CPU interrupts
    IER=0x0000;

    //Clear CPU Flags
    IFR=0x0000;

    //Initialize the PIE vector Table
    InitPieVectTable();

    //Setup PIE Table Mappings
    PIEMap();

    //ADC setup Part 1
    InitADCPart1();

    //ePWM setup function
    InitEPwm();

    //ADC setup Part 2
    InitADCPart2();

    //Enable The Interrupts
    IER |=(M_INT1 | M_INT3 | M_INT10); //Enable Groups 1,3 and 10

    //Enable Interrupts in PIE
    PieCtrlRegs.PIEIER1.bit.INTx1 = 1;  //Enable ADCA1 interrupt
    PieCtrlRegs.PIEIER3.bit.INTx1 = 1;  //Enable ePWM1 interrupt
    PieCtrlRegs.PIEIER3.bit.INTx2 = 1;  //Enable ePWM2 interrupt
    PieCtrlRegs.PIEIER3.bit.INTx3 = 1;
    PieCtrlRegs.PIEIER10.bit.INTx6 = 1; //Enable ADCB2 interrupt

    //Enable interrupts
    EINT;

    //Enable Real Time Management?
    ERTM;

    //Start ePWM
    EPwmStart();
    //Loop
    for(;;){

        //Current control if statement
        if(c_update){
            c_update = 0;
            //Bang_Bang_Cntrl();

        }

        //If statement to facilitate the PID calculations,
            //*Figure out if the current control should still be here
        if(x1_update){
            x1_update = 0 ;
            GpioDataRegs.GPBTOGGLE.bit.GPIO40 = 1;  //Used to clock how long this if statement takes
            Position_PID_Cntrl(&X1);    //PID control for X1 sensor
            Position_PID_Cntrl(&Y1);    //PID control for Y1 sensor
            Position_PID_Cntrl(&X2);    //PID control for X2 sensor
            Position_PID_Cntrl(&Y2);    //PID control for Y2 sensor
            current_target(&C1, &X1, &Y1);  //Displacement to Current target for coil 1
            current_target(&C2, &X1, &Y1);  //Displacement to Current target for coil 2
            current_target(&C3, &X1, &Y1);  //Displacement to Current target for coil 3
            current_target(&C4, &X1, &Y1);  //Displacement to Current target for coil 4
            Bang_Bang_Cntrl(&C1);   //Current control function for coil 1
            Bang_Bang_Cntrl(&C2);   //Current control function for coil 2
            Bang_Bang_Cntrl(&C3);   //Current control function for coil 3
            Bang_Bang_Cntrl(&C4);   //Current control function for coil 4
            GpioDataRegs.GPBTOGGLE.bit.GPIO40 = 1;  //*OLD* Takes 1.0226 micro seconds
            if(((X1.sample - X2.sample) > xmax) || ((X1.sample - X2.sample) < xmin)){
                if((X1.sample - X2.sample) > xmax){
                    xmax = X1.sample - X2.sample;
                }
                else{
                    xmin = X1.sample - X2.sample;
                }
                xdelta = xmax - xmin;
            }
            if(((Y1.sample - Y2.sample) > ymax) || ((Y1.sample - Y2.sample) < ymin)){
                if((Y1.sample - Y2.sample) > ymax){
                    ymax = Y1.sample - Y2.sample;
                }
                else{
                    ymin = Y1.sample - Y2.sample;
                }
                ydelta = ymax - ymin;
            }
        }
    }
}

/*
 * Setup function for displacement structures
 */

void SetupPosition(position *sensor){
    //Sets up the initial state for the displacement structure variables
    int i;      //Used as a simple loop counter
    sensor->target = x1_target;         //Feeds the position structure the location of sample global
    sensor->dt = x1_dt;                 //Feeds in the specific sample period for the sensor
    sensor->kp = kp_init;               //Sets default proportional constant
    sensor->ki = ki_init;               //Sets default integral constant
    sensor->kd = kd_init;               //Sets default derivative constant
    sensor->prev_place = 0;             //Starts the PID progression at 0
    sensor->prev_last = prev_size - 1;  //Starts the last pointer to the end of the array
    for(i=0;i<prev_size;i++){   
        sensor->prev[i]=0;              //Step by step through the PID memory array and zero it
    }
}

/*
 * Setup function for coil structure
 */

void SetupCoil(current *coil){
    //Sets up the initial state for the coil structure variables
    coil->scale = current_scale;    //Sets the current conversion scale
    coil->offset = current_offset;  //Sets the current conversion offset
    coil->x_influence = 0;          //Sets the current x value scaling constant to default of zero
    coil->y_influence = 0;          //Sets the current y value scaling constant to default of zero
    coil->bias = current_bias;      //Sets the current bias to the default define
}

/*
 * Position PID Control Function
 */

void Position_PID_Cntrl(position *sensor){
    //Read sample into structure
    sensor->sample = (*(sensor->sample_loc) * sensor->scale) + sensor->offset;    //*(Meters)

    //Initial error calculation between target and sample values
    sensor->error = sensor->sample - sensor->target;

    //Proportional term calculation
    sensor->p = sensor->error * sensor->kp;

    //Integral term calculation
        //This was fun to code, pretty much I subtract out the last value of the array at the same time I add in the new one!
    sensor->i = sensor->i + ((sensor->error - sensor->prev[sensor->prev_place]) * sensor->dt * sensor->ki);

    //Derivative term calculation
    sensor->d = (sensor->error - sensor->prev[sensor->prev_last]) * sensor->kd / sensor->dt;    

    //PID output
    sensor->pid_out = sensor->p + sensor->i + sensor->d;    //*(mm?)

    //clean up operations to set the array for the next round of PID calculations
    sensor->prev[sensor->prev_place] = sensor->error;   //Places the current rounds error into the memory array
    sensor->prev_last = sensor->prev_place;     //Progresses the placement variable for the "last" location to the current place
    sensor->prev_place = (prev_size - 1) & (sensor->prev_place + 1);    //Progresses the current location variable while using the bitwise math to limit and loop the variable
}

/*
 * Current Target Function
 */

void current_target(current *coil, position *x_disp_sensor, position *y_disp_sensor){
    float temp;
    /* In the next line is where the future addition of the X Y influence will be accounted for */
    temp = coil->bias + disp_to_current * ((coil->x_influence * x_disp_sensor->pid_out) + (coil->y_influence * y_disp_sensor->pid_out)); //*(Amps)
    if(temp > current_max || temp < current_min){
        if(temp > current_max){
            temp = current_max;     //Limits the target current to the max value
        }
        else{
            temp = current_min;     //Limits the target current to the min value
        }
    }
    //Sets the target for the coil
    coil->target = temp; //*(Amps)
}

/*
 * Current Control Function
 */

void Bang_Bang_Cntrl(current *coil){
    //Conversion of ADC current reading into Amps
    coil->sample = (*(coil->sample_loc) * coil->scale) + coil->offset;  //*(Amps)

    //Initial error calculation between target and sampled current
    coil->error = coil->sample - coil->target;

    //Run P_H pin high or low depending on error
    if(coil->error < 0){
        GpioDataRegs.GPBSET.all |= 1 << coil->gpio_offset;     //This is Tied to the P_H pin for one of the Pololu's
    }
    else{
        GpioDataRegs.GPBCLEAR.all |= 1 << coil->gpio_offset;   //This is Tied to the P_H pin for one of the Pololu's
    }
}

/*
 * ADC Initialization Functions
 */

void InitADCPart1(){
    //Enable editing of EALLOW registers
    EALLOW;

    //Enable The CLk signal for ADC A and B
    CpuSysRegs.PCLKCR13.bit.ADC_A = 1;
    CpuSysRegs.PCLKCR13.bit.ADC_B = 1;

    //Initial setup function for ADCs, calibrates them to factory state
        /***Find this function in the F2837xD_ADC.c file***/
    AdcSetMode(ADC_ADCA, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);
    AdcSetMode(ADC_ADCB, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);

    //Initialization Settings for ADC A
    AdcaRegs.ADCCTL1.bit.INTPULSEPOS = 1;   //Sets Interrupt pulse to trigger after conversion
    AdcaRegs.ADCCTL2.bit.SIGNALMODE = 0;    //Single Ended, not Differential
    AdcaRegs.ADCCTL2.bit.RESOLUTION = 0;    //Sets resolution to 12 bits, 1=16 bits
    AdcaRegs.ADCCTL2.bit.PRESCALE = 8;      //Sets the Prescale of the ADC, 6=1/4   *Be careful with value, if it is to low, issues with sample value will occur

    //Initialization Settings for ADC B
    AdcbRegs.ADCCTL1.bit.INTPULSEPOS = 1;   //Sets Interrupt pulse to trigger after conversion
    AdcbRegs.ADCCTL2.bit.SIGNALMODE = 0;    //Single Ended, not Differential
    AdcbRegs.ADCCTL2.bit.RESOLUTION = 0;    //Sets resolution to 12 bits, 1=16 bits
    AdcbRegs.ADCCTL2.bit.PRESCALE = 8;      //Sets the Prescale of the ADC, 6=1/4   *Be careful with value, if it is to low, issues with sample value will occur

    AdcaRegs.ADCCTL1.bit.ADCPWDNZ = 1;      //Powers up all of the analog circuitry
    AdcbRegs.ADCCTL1.bit.ADCPWDNZ = 1;      //Powers up all of the analog circuitry


    //Delay for 1 ms to allow ADCs to start up
    DELAY_US(1000);     //Required warmup time

    //Disable editing of EALLOW registers
    EDIS;
}

void InitADCPart2(){
    EALLOW;

    //Configuration Settings for ADCA
    AdcaRegs.ADCSOC0CTL.bit.TRIGSEL = 5;    //Set the Trigger for SOC0, 5=ePWM1
    AdcaRegs.ADCSOC0CTL.bit.CHSEL = 0;      //Set the channel for SOC0 to convert, 0=ADCAIN0
    AdcaRegs.ADCSOC0CTL.bit.ACQPS = 19;     //Set the sample window size for SOC0, 19=20 SysClkCycles
    AdcaRegs.ADCSOC1CTL.bit.TRIGSEL = 5;    //Set the Trigger for SOC1, 5=ePWM1
    AdcaRegs.ADCSOC1CTL.bit.CHSEL = 1;      //Set the channel for SOC1 to convert, 0=ADCAIN1
    AdcaRegs.ADCSOC1CTL.bit.ACQPS = 19;     //Set the sample window size for SOC1, 19=20 SysClkCycles
    AdcaRegs.ADCSOC2CTL.bit.TRIGSEL = 5;    //Set the Trigger for SOC2, 5=ePWM1
    AdcaRegs.ADCSOC2CTL.bit.CHSEL = 2;      //Set the channel for SOC2 to convert, 0=ADCAIN2
    AdcaRegs.ADCSOC2CTL.bit.ACQPS = 19;     //Set the sample window size for SOC2, 19=20 SysClkCycles
    AdcaRegs.ADCSOC3CTL.bit.TRIGSEL = 5;    //Set the Trigger for SOC3, 5=ePWM1
    AdcaRegs.ADCSOC3CTL.bit.CHSEL = 3;      //Set the channel for SOC3 to convert, 0=ADCAIN3
    AdcaRegs.ADCSOC3CTL.bit.ACQPS = 19;     //Set the sample window size for SOC3, 19=20 SysClkCycles
    AdcaRegs.ADCINTSEL1N2.bit.INT1SEL = 3;  //Set EOCx to trigger ADCINT1, 0=EOC0
    AdcaRegs.ADCINTSEL1N2.bit.INT1E = 1;    //Enable/Disable ADCINT1
    AdcaRegs.ADCINTSEL1N2.bit.INT1CONT = 0; //Enable/Disable Continuous Mode
    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;  //Clears ADCINT1 Flag

    //Configuration Settings for ADCB
    AdcbRegs.ADCSOC0CTL.bit.TRIGSEL = 8;    //Set the Trigger for SOC0, 8=ePWM2
    AdcbRegs.ADCSOC0CTL.bit.CHSEL = 0;      //Set the channel for SOC0 to convert, 0=ADCBIN0
    AdcbRegs.ADCSOC0CTL.bit.ACQPS = 19;     //Set the sample window size for SOC0, 19=20 SysClkCycles
    AdcbRegs.ADCSOC1CTL.bit.TRIGSEL = 8;    //Set the Trigger for SOC1, 8=ePWM2
    AdcbRegs.ADCSOC1CTL.bit.CHSEL = 1;      //Set the channel for SOC1 to convert, 1=ADCBIN1
    AdcbRegs.ADCSOC1CTL.bit.ACQPS = 19;     //Set the sample window size for SOC1, 19=20 SysClkCycles
    AdcbRegs.ADCSOC2CTL.bit.TRIGSEL = 8;    //Set the Trigger for SOC2, 8=ePWM2
    AdcbRegs.ADCSOC2CTL.bit.CHSEL = 2;      //Set the channel for SOC2 to convert, 0=ADCBIN0
    AdcbRegs.ADCSOC2CTL.bit.ACQPS = 19;     //Set the sample window size for SOC2, 19=20 SysClkCycles
    AdcbRegs.ADCSOC3CTL.bit.TRIGSEL = 8;    //Set the Trigger for SOC3, 8=ePWM2
    AdcbRegs.ADCSOC3CTL.bit.CHSEL = 3;      //Set the channel for SOC3 to convert, 1=ADCBIN1
    AdcbRegs.ADCSOC3CTL.bit.ACQPS = 19;     //Set the sample window size for SOC3, 19=20 SysClkCycles
    AdcbRegs.ADCINTSEL1N2.bit.INT2SEL = 3;  //Set EOCx to trigger ADCINT2, 1=EOC1
    AdcbRegs.ADCINTSEL1N2.bit.INT2E = 1;    //Enable/Disable ADCINT2
    AdcbRegs.ADCINTSEL1N2.bit.INT2CONT = 0; //Enable/Disable Continuous Mode
    AdcbRegs.ADCINTFLGCLR.bit.ADCINT2 = 1;  //Clears ADCINT2 Flag

    EDIS;
}

/*
 * ePWM Initialization Function
 */

void InitEPwm(){
    EALLOW;
    CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 0;   //Disable the Time Base Clk

    //Enable Clk for PWM
    CpuSysRegs.PCLKCR2.bit.EPWM1 = 1;   //Enable ePWM1 CLK
    CpuSysRegs.PCLKCR2.bit.EPWM2 = 1;   //Enable ePWM2 CLK
    CpuSysRegs.PCLKCR2.bit.EPWM3 = 1;   //Enable ePWM2 CLK

    //Time Base Controls for ePWM1
    EPwm1Regs.TBCTL.bit.PHSEN = 0;      //Disable loading Counter register from Phase register
    EPwm1Regs.TBCTL.bit.PRDLD = 0;      //A shadow register is used to write to the Period register
    EPwm1Regs.TBCTL.bit.HSPCLKDIV = 0;  //High speed Time Base Prescale, 0="/1"
    EPwm1Regs.TBCTL.bit.CLKDIV = 0;     //Time Base Clock Prescale, 0="/1"
    EPwm1Regs.TBCTL.bit.CTRMODE = 3;    //Set counter mode, 3=freeze counter

    //Set ePWM1 counter to 0
    EPwm1Regs.TBCTR = 0;    //Resets ePWM1 counter register

    //Set ePWM1 period
    EPwm1Regs.TBPRD = 1249;     //Results in a period of 1/80 kHz

    //Disable ePWM1 phase sync
    EPwm1Regs.TBPHS.bit.TBPHS = 0;

    //Enable ePWM1 event Triggering selection
    EPwm1Regs.ETSEL.bit.SOCAEN = 0;     //Disable the SOCA conversion until ready
    EPwm1Regs.ETSEL.bit.SOCASEL = 1;    //Select SOCA to be triggered by ePWM1

    //Enable pulse on first event
    EPwm1Regs.ETPS.bit.SOCAPRD = 1;

    //Set the Compare A register
    EPwm1Regs.CMPA.bit.CMPA = 625;      //Sets Compare A to a 50% duty cycle, arbitrary for now

    //Action qualifier control
    EPwm1Regs.AQCTLA.bit.CAU = 2;       //Action at A going up, 2=set_high
    EPwm1Regs.AQCTLA.bit.ZRO = 1;       //Action at zero, 1=clear


    //Time Base Controls for ePWM2
    EPwm2Regs.TBCTL.bit.PHSEN = 0;      //Disable loading Counter register from Phase register
    EPwm2Regs.TBCTL.bit.PRDLD = 0;      //A shadow register is used to write to the Period register
    EPwm2Regs.TBCTL.bit.HSPCLKDIV = 0;  //High speed Time Base Prescale, 0="/1"
    EPwm2Regs.TBCTL.bit.CLKDIV = 0;     //Time Base Clock Prescale, 0="/1"
    EPwm2Regs.TBCTL.bit.CTRMODE = 3;    //Set counter mode, 3=freeze counter

    //Set ePWM2 counter to 0
    EPwm2Regs.TBCTR = 0;    //Resets ePWM2 counter register

    //Set ePWM2 period
    EPwm2Regs.TBPRD = 9999;     //Results in a period of 1/10 kHz 

    //Disable ePWM2 phase sync
    EPwm2Regs.TBPHS.bit.TBPHS = 0;

    //Enable ePWM2 event Triggering selection
    EPwm2Regs.ETSEL.bit.SOCBEN = 0;     //Disable the SOCB conversion until ready
    EPwm2Regs.ETSEL.bit.SOCBSEL = 1;    //Select SOCB to be triggered by ePWM2

    //Enable pulse on first event
    EPwm2Regs.ETPS.bit.SOCBPRD = 1;

    //Set the Compare A register
    EPwm2Regs.CMPA.bit.CMPA = 5000;     //Sets Compare A to a 50% duty cycle, arbitrary for now

    //Action qualifier control
    EPwm2Regs.AQCTLA.bit.CAU = 2;       //Action at A going up, 2=set_high
    EPwm2Regs.AQCTLA.bit.ZRO = 1;       //Action at zero, 1=clear

    //Time Base Controls for ePWM3
    EPwm3Regs.TBCTL.bit.PHSEN = 0;      //Disable loading Counter register from Phase register
    EPwm3Regs.TBCTL.bit.PRDLD = 0;      //A shadow register is used to write to the Period register
    EPwm3Regs.TBCTL.bit.HSPCLKDIV = 0;  //High speed Time Base Prescale, 0="/1"
    EPwm3Regs.TBCTL.bit.CLKDIV = 0;     //Time Base Clock Prescale, 0="/1"
    EPwm3Regs.TBCTL.bit.CTRMODE = 3;    //Set counter mode, 3=freeze counter

    //Set ePWM3 counter to 0
    EPwm3Regs.TBCTR = 0;    //Resets ePWM3 counter register

    //Set ePWM3 period
    EPwm3Regs.TBPRD = 99;     //Results in a period of 1/1 MHz 

    //Disable ePWM3 phase sync
    EPwm3Regs.TBPHS.bit.TBPHS = 0;

    //Set the Compare A register
    EPwm3Regs.CMPA.bit.CMPA = 49;     //Sets Compare A to a 50% duty cycle, arbitrary for now

    //Action qualifier control
    EPwm3Regs.AQCTLA.bit.CAU = 2;       //Action at A going up, 2=set_high
    EPwm3Regs.AQCTLA.bit.PRD = 1;       //Action at period, 1=clear
    EPwm3Regs.AQCTLA.bit.CBU = 0;
    EPwm3Regs.AQCTLA.bit.ZRO = 0;

    //Interrupt setup
    EPwm3Regs.ETSEL.bit.INTSEL = 4;     //Sets interrupt to trigger on zero
    EPwm3Regs.ETSEL.bit.INTEN = 1;      //Enable Interrupt
    EPwm3Regs.ETPS.bit.INTPRD = 1;      //Generate an interrupt on every event

    //CPU clock enable
    CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 1;   //Renable the Time Base Clk
    EDIS;
}

/*
 * PIE Mapping Function
 */

void PIEMap(){
    EALLOW;

    //PIE mappings for ADC interrupts
    PieVectTable.ADCA1_INT=&adca1_isr;
    PieVectTable.ADCB2_INT=&adcb2_isr;

    //PIE mappings for ePWM interrupts
    //PieVectTable.EPWM1_INT = &epwm1_isr;
    //PieVectTable.EPWM2_INT = &epwm2_isr;
    PieVectTable.EPWM3_INT = &epwm3_isr;

    EDIS;
}

/*
 * ePWM Startup Function
 */

void EPwmStart(){
    EALLOW;

    //Enable SOCs
    EPwm1Regs.ETSEL.bit.SOCAEN = 1; 
    EPwm2Regs.ETSEL.bit.SOCBEN = 1;

    //Unfreeze ePWM to up count mode
    EPwm1Regs.TBCTL.bit.CTRMODE = 0;
    EPwm2Regs.TBCTL.bit.CTRMODE = 0;
    EPwm3Regs.TBCTL.bit.CTRMODE = 0;

    EDIS;
}

/*
 * CRC calculation function
 */

int Check_CRC(uint32_t data, int crc){
    int buffer = 6;
    uint32_t crc_poly = 0x00000043;
    int i = 20 - 1;
    data = data << buffer;
    for(i=19; i > 0 ; i=i-1){
        if(data & ((uint32_t)1<<(i+6))){
            data ^= crc_poly << i;
        }
    }
    if((int)(0x3f & (~(data))) && crc){
        return 0;
    }
    else{
        return 1;
    }
}

/*
 * ADC1 ISR
 */

interrupt void adca1_isr(){


    //Write the sample to the global variable
    c1_sample = AdcaResultRegs.ADCRESULT0;  //Reads the result register of SOC0
    c2_sample = AdcaResultRegs.ADCRESULT1;  //Reads the result register of SOC1
    c3_sample = AdcaResultRegs.ADCRESULT2;  //Reads the result register of SOC2
    c4_sample = AdcaResultRegs.ADCRESULT3;  //Reads the result register of SOC3
    //Set the update flag
    c_update = 1;  //Triggers the if statement in the main loop for Current Control

    //Clear the interrupt flag
    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1=1;    //Clears ADCA interrupt 1

    //Acknowledge the interrupt
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;     //Acknowledges the interrupt in the PIE table for ADCA interrupt 1
}

/*
 * ADC2 ISR
 */

interrupt void adcb2_isr(){
    //Write the sample to the global variable
    x1_sample = AdcbResultRegs.ADCRESULT0;  //Reads the result register of SOC0
    y1_sample = AdcbResultRegs.ADCRESULT1;  //Reads the result register of SOC1
    x2_sample = AdcbResultRegs.ADCRESULT2;  //Reads the result register of SOC2
    y2_sample = AdcbResultRegs.ADCRESULT3;  //Reads the result register of SOC3

    //Set the update flag
    x1_update = 1;  //Triggers the if statement in the main loop for PID operations

    EPwm3Regs.TBCTL.bit.CTRMODE = 0;


    //Clear the interrupt flag
    AdcbRegs.ADCINTFLGCLR.bit.ADCINT2=1;    //Clears ADCB interrupt 2

    //Acknowledge the interrupt
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP10;    //Acknowledges the interrupt in the PIE table for ADCB interrupt 2
}

/*
 * ePWM3 ISR
 */

interrupt void epwm3_isr(){
    //Progress the state to the next state
    c_state = n_state;

    GpioDataRegs.GPBTOGGLE.all = (uint32_t)c_state << 16;

    //Read the miso pin
    miso = GpioDataRegs.GPBDAT.bit.GPIO41;

    switch(c_state){
        case Idle_state:
            if(miso == 0){
                n_state = Sampling_state;
            }
            break;
        case Sampling_state:
            if(miso == 1){
                n_state = Start_state;
            }
            break;
        case Start_state:
            if(miso == 0){
                n_state = Null_state;
            }
            break;
        case Null_state:
            n_state = Data_state;
            CRC_check = (CRC_check <<1) + miso;
            break;
        case Data_state:
            if(Data_count < 17){
                CRC_check = (CRC_check << 1) + miso;
                Data_count++;
            }
            else{
                CRC_check = (CRC_check << 1) + miso;
                n_state = Error_state;
                if(miso == 0){
                    Error_flag = 1;
                }
                else{
                    Error_flag = 0;
                }
                Data_count = 0;
            }
            break;
        case Error_state:
            CRC_check = (CRC_check << 1) + miso;
            n_state = Warning_state;
            Warning_flag += !miso;
            break;
        case Warning_state:
            CRC_value = (CRC_value << 1) + miso;
            n_state = CRC_state;
            break;
        case CRC_state:
            if(CRC_count < 4){
                CRC_value = (CRC_value << 1) + miso;
                CRC_count++;
            }
            else{
                n_state = Idle_state;
                EPwm3Regs.TBCTL.bit.CTRMODE = 3;
                CRC_value = (CRC_value << 1) + miso;
                CRC_result = Check_CRC(CRC_check, CRC_value);
                if(!(CRC_result || Error_flag)){
                    Data_value = (CRC_check >> 2) & 0x0003ffff;
                }
                else
                {
                    Data_value = 0x4000;
                }
                Rotation_percent = __divf32(Data_value,Max_value);
                Rotation_percent = Rotation_percent * 360;
            }
            break;
        case Ignore_state:
            if(miso == 1){
                n_state = Idle_state;
                EALLOW;
                EPwm3Regs.TBCTL.bit.CTRMODE = 3;
                EDIS;
            }
            break;
        }
    GpioDataRegs.GPBCLEAR.all = (uint32_t)0x7 << 16;

    //Clear the interrupt flag
    EPwm3Regs.ETCLR.bit.INT = 1;

    //Acknowledge the interrupt
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;
}
