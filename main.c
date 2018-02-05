 /* Title: SLFBM LSFESS Test Code
 * Author: Tyler Larson
 * Purpose: This code is intended to run the bottom half of the low speed flywheel
 */

/*
 * Includes
 */

#include "F28x_Project.h"   //Seems like a nice header file, includes a lot of things so I don't have to

/*
 * Defines
 */

//Sets the number of coils that the machine has
#define coil_count 24 //There are 24 coils in the FRRM that we are controling

//Sets the size of the array for the Integral and Derivative previous values
#define prev_size   32  //Size of the Memory array in the PID control

//Sets the number of Sysclk cycles that the ADC samples for
#define sample_cycles (20 - 1)  //Note* only change the first number, the 1 accounts for the minimum

//Current Bias current for the SADB to help stabilize
#define current_bias 0   //Set arbitrarily as a middle point in the range of currents we anticipate to use

//Current sensor conversion scale       *Calibrated as of 2017/07/20
#define current_scale (10.168070782 * 3.3 / 4096)//Conversion scale for current sensor *(Amps/Volt)

//Current sensor conversion offset      *Calibrated as of 2017/07/20
#define current_offset -13.45198481 //Conversion offset for current sensor *(Amps)

//Displacement sensor conversion scale
#define disp_scale = .9540446743 * 3.3 / 4096   //Conversion scale for displacement sensor *(Millimeters/Volt)

//Displacement sensor conversion offset
#define disp_offset = 0  //Conversion offset for displacement sensor *(Millieters)

//Displacement sensor sample period
#define x1_dt 0.2 //*(MilliSeconds)

//Displacement sensor target
#define x1_target 1   //Target float level *(Millimeters)

//PID constants
#define kp_init .1 //Default to 1 for Initial tuning
#define ki_init 0  //Default to 0 for initial tuning
#define kd_init 0  //Default to 0 for initial tuning

/*
 * Enumerations for Readability
 */

//Gpio Register Enumerations
typedef enum Gpio_Regs{
    Gpio_A,
    Gpio_B,
    Gpio_C,
    Gpio_D,
    Gpio_E,
    Gpio_F
}Gpio_Regs;

//State Transition States Enumeration
typedef enum SPI_state {
    Idle_state,
    Sampling_state,
    Start_state,
    Null_state,
    Data_state,
    Error_state,
    Warning_state,
    CRC_state,
    Ignore_state
}SPI_state;

enum coil_num{
    C1,
    C2,
    C3,
    C4,
    C5,
    C6,
    C7,
    C8,
    C9,
    C10,
    C11,
    C12,
    C13,
    C14,
    C15,
    C16,
    C17,
    C18,
    C19,
    C20,
    C21,
    C22,
    C23,
    C24
};

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
    //float bearing_cpos;     /*!< Coil positive demand, *(Amps) */
    //float bearing_cneg;     /*!< Coil negative demand, *(Amps) */
    float error;            /*!< Error to determine more or less current, *(Amps)   **May change later for PWM and/or a PID loop */
    float scale;            /*!< Scaling constant to convert sensor reading to current, *(Amps/Volt) */
    float offset;           /*!< Offset constant to convert sensor reading to current, *(Amps) */
    //float x_influence;      /*!< X-axis influence of the coil */
    //float y_influence;      /*!< Y-axis influence of the coil */
    //float bias;             /*!< Individual bias current of the coil **May use in Gauss/De-Gauss process in the future */
    //int gpio_offset;        /*!< Offset in the register to the correct GPIO pin */
    uint16_t gpio_pwm_h;         /*!< Gpio value for the PWM_H pin */
    uint16_t gpio_pwm_l;         /*!< Gpio value for the PWM_L pin */
    uint16_t gpio_dir;           /*!< Gpio value for the Dir pin */
    //int neighbor_coil;      /*!< The coil locatation of the neighboring coil */
}current;

/*
 * Function Prototypes
 */
    //*Note, at some point this should be moved to a header file

void Coil_Pin_Assignments(current *Coils);
void Coil_Gpio_Init(current *Coils,uint32_t *gpio_mask);
void SetupPosition(position *sensor);
void SetupCoil(current *coil);
void Position_PID_Cntrl(position *sensor);
void current_target(current *Coils[],position *X,position *Y);
void Bearing_Current_Target(current *Coils, position *X, position *Y);
void Current_Calc_Bearing(current *Coil_Array[], position *X, position *Y);
void Bang_Bang_Cntrl(current *Coils);
void InitADCPart1();
void InitADCPart2();
void InitEPwm();
void PIEMap();
void EPwmStart();
int Check_CRC(uint32_t CRC_check, int CRC_value);
interrupt void adca1_isr();
interrupt void adcb2_isr();
interrupt void adcc1_isr();
interrupt void adcd1_isr();
interrupt void epwm3_isr();

/*
 * Globals
 */

//Definition of the X1 instance of the position sensor structure
position X1;    //Variable used to store the PID math variables for the x axis displacement sensor
position Y1;    //Variable used to store the PID math variables for the y axis displacement sensor

//Definition of the current coil structure
current Coils[coil_count];  //Define all the coils at once

//Bias Current Value
float field_coil_bias = .1;

//Displacement sensor reading
int x1_sample;  //Global for x1 displacement sensor reading *(Digitized)
int y1_sample;  //Global for y1 displacement sensor reading *(Digitized)

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
int c_samples[coil_count];

//GPIO Masks Array
uint32_t gpio_mask[6];  //Init variable array for GPIO Register masks


//Current sensor update flag
int ADCA_update = 0;  //Flag for new current sensor readings
int ADCB_update = 0;  //Flag for new current sensor readings
int ADCC_update = 0;  //Flag for new current sensor readings
int ADCD_update = 0;  //Flag for new current sensor readings

//Maximum Current
float current_max = 10;  //Max current for operation *(Amps)

//Minimum Current
float current_min = -10; //Min current for operation *(Amps)

float map_array[2][coil_count] = {
    /*X values*/{0,0,0,0,0,0,1,1,0,0,1,1,0,0,0,0,0,0,1,1,0,0,1,1},
    /*Y values*/{1,1,0,0,1,1,0,0,0,0,0,0,1,1,0,0,1,1,0,0,0,0,0,0}
};

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
    /*Someday make this a struct */
uint32_t CRC_check;
float Data_value;
bool Error_flag = 0;
int Warning_flag = 0;
int CRC_value = 0;
bool CRC_result = 0;
int Data_count = 0;
int CRC_count = 0;
bool miso;
uint32_t Rotation_reading;
uint32_t Hw_offset;
float Rotation_correct;
float Rotation_percent;
float Rotation_degrees;
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
    GpioCtrlRegs.GPAPUD.bit.GPIO8 = 0;     //Leaves pull up resistor on pin
    GpioDataRegs.GPASET.bit.GPIO8 = 1;     //Sets the pin high
    GpioCtrlRegs.GPAGMUX1.bit.GPIO8 = 0;   //Sets the pins to default mux
    GpioCtrlRegs.GPADIR.bit.GPIO8 = 1;     //Sets the pin to output
    GpioDataRegs.GPACLEAR.bit.GPIO8 = 1;   //Sets the pin low


    //MISO Pin for Rotational Encoder
    GpioCtrlRegs.GPAPUD.bit.GPIO9 = 0;
    GpioCtrlRegs.GPAGMUX1.bit.GPIO9 = 0;
    GpioCtrlRegs.GPADIR.bit.GPIO9 = 0;

//    //Current State Output Pins
//    GpioCtrlRegs.GPADIR.all = (uint32_t)0x7 << 16;    //GPIO 48,49 and 50
//    GpioDataRegs.GPACLEAR.all = (uint32_t)0x7 << 16;

    //Sets the PWM_H, PWM_L and Dir GPIO values
        /* This is where one would go to see/change the pinouts on the board */
    Coil_Pin_Assignments(Coils);

    //Enables and sets to low PWM_H, PWM_L and Dir for each coil
    Coil_Gpio_Init(Coils, gpio_mask);

    EDIS;

    //Enable output of ePWM signals to Pins
    InitEPwm1Gpio();
    InitEPwm2Gpio();
    InitEPwm3Gpio();

    //Setup X1 variable
    SetupPosition(&X1);
    SetupPosition(&Y1);
    
    //Sets the location of the global for each sample
    X1.sample_loc = &x1_sample;
    Y1.sample_loc = &y1_sample;

    //Sets the target for X1 and Y1
    X1.target = 1.6;
    Y1.target = 1.5;

    //Sets the correct offset and scaling factor for each displacement sensor
    X1.scale = 0.00076433121;
    X1.offset = 0;
    Y1.scale = 0.00077299665;
    Y1.offset = 0;
    // X2.scale = 0.000768639508;
    // X2.offset = 0;
    // Y2.scale = 0.000807102502;
    // Y2.offset = 0;

    int i;

    //Setup Coils
    for (i = 0; i < coil_count; ++i)
    {
        SetupCoil(&Coils[i]);
        //Sets the location of the global for each sample
        Coils[i].sample_loc = &(c_samples[i]);
    }   

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
    PieCtrlRegs.PIEIER1.bit.INTx3 = 1;  //Enable ADCC1 interrupt
    PieCtrlRegs.PIEIER1.bit.INTx6 = 1;  //Enable ADCD1 interrupt
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

//        //Current control if statement
//        if(c_update){
//            c_update = 0;
//            //Bang_Bang_Cntrl; //Commented out for a single point of stablilization
//        }

        //If statement to facilitate the PID calculations,
            //*Figure out if the current control should still be here
        if(ADCA_update&&ADCB_update&&ADCC_update&&ADCD_update/*x1_update*/){
            x1_update = 0 ;
            ADCA_update = 0;
            ADCB_update = 0;
            ADCC_update = 0;
            ADCD_update = 0;
            GpioDataRegs.GPATOGGLE.bit.GPIO8 = 1;  //Used to clock how long this if statement takes
            Position_PID_Cntrl(&X1);    //PID control for X1 sensor
            Position_PID_Cntrl(&Y1);    //PID control for Y1 sensor
            // Position_PID_Cntrl(&X2);    //PID control for X2 sensor
            // Position_PID_Cntrl(&Y2);    //PID control for Y2 sensor
            
            Bearing_Current_Target(Coils,&X1,&Y1); //Displacement to Current target for coil 24
                        
//            Bang_Bang_Cntrl(&C1);   //Current control function for coil 1
//            Bang_Bang_Cntrl(&(Coils[1]));   //Current control function for coil 2
//            Bang_Bang_Cntrl(&C3);   //Current control function for coil 3
//            Bang_Bang_Cntrl(&C4);   //Current control function for coil 4
//            Bang_Bang_Cntrl(&C5);   //Current control function for coil 5
//            Bang_Bang_Cntrl(&C6);   //Current control function for coil 6
//            Bang_Bang_Cntrl(&C7);   //Current control function for coil 7
//            Bang_Bang_Cntrl(&C8);   //Current control function for coil 8
//            Bang_Bang_Cntrl(&C9);   //Current control function for coil 9
//            Bang_Bang_Cntrl(&C10);   //Current control function for coil 10
//            Bang_Bang_Cntrl(&C11);   //Current control function for coil 11
//            Bang_Bang_Cntrl(&C12);   //Current control function for coil 12
//            Bang_Bang_Cntrl(&C13);   //Current control function for coil 13
//            Bang_Bang_Cntrl(&C14);   //Current control function for coil 14
//            Bang_Bang_Cntrl(&C15);   //Current control function for coil 15
//            Bang_Bang_Cntrl(&C16);   //Current control function for coil 16
//            Bang_Bang_Cntrl(&C17);   //Current control function for coil 17
//            Bang_Bang_Cntrl(&C18);   //Current control function for coil 18
//            Bang_Bang_Cntrl(&C19);   //Current control function for coil 19
//            Bang_Bang_Cntrl(&C20);   //Current control function for coil 20
//            Bang_Bang_Cntrl(&C21);   //Current control function for coil 21
//            Bang_Bang_Cntrl(&C22);   //Current control function for coil 22
//            Bang_Bang_Cntrl(&C23);   //Current control function for coil 23
//            Bang_Bang_Cntrl(&C24);   //Current control function for coil 24
            Bang_Bang_Cntrl(Coils);    //Current control function for the coils

            GpioDataRegs.GPATOGGLE.bit.GPIO8 = 1;  //*OLD* Takes 1.0226 micro seconds
        }
    }
}

/*
 * Function to assing the PWM_H, PWM_L and Dir GPIO values
 */

void Coil_Pin_Assignments(current *Coils){
    Coils[C1].gpio_pwm_h = 82;      //C1 PWM_H, GPIO , Pin
    Coils[C1].gpio_pwm_l = 84;      //C1 PWM_L, GPIO , Pin
    Coils[C1].gpio_dir = 86;        //C1 Dir, GPIO , Pin
    Coils[C2].gpio_pwm_h = 76;      //C2 PWM_H, GPIO , Pin
    Coils[C2].gpio_pwm_l = 78;      //C2 PWM_L, GPIO , Pin
    Coils[C2].gpio_dir = 80;        //C2 Dir, GPIO , Pin
    Coils[C3].gpio_pwm_h = 70;      //C3 PWM_H, GPIO , Pin
    Coils[C3].gpio_pwm_l = 72;      //C3 PWM_L, GPIO , Pin
    Coils[C3].gpio_dir = 74;        //C3 Dir, GPIO , Pin
    Coils[C4].gpio_pwm_h = 64;      //C4 PWM_H, GPIO , Pin
    Coils[C4].gpio_pwm_l = 66;      //C4 PWM_L, GPIO , Pin
    Coils[C4].gpio_dir = 68;        //C4 Dir, GPIO , Pin
    Coils[C5].gpio_pwm_h = 27;      //C5 PWM_H, GPIO , Pin
    Coils[C5].gpio_pwm_l = 32;      //C5 PWM_L, GPIO , Pin
    Coils[C5].gpio_dir = 33;        //C5 Dir, GPIO , Pin
    Coils[C6].gpio_pwm_h = 24;      //C6 PWM_H, GPIO , Pin
    Coils[C6].gpio_pwm_l = 25;      //C6 PWM_L, GPIO , Pin
    Coils[C6].gpio_dir = 26;        //C6 Dir, GPIO , Pin
    Coils[C7].gpio_pwm_h = 17;      //C7 PWM_H, GPIO , Pin
    Coils[C7].gpio_pwm_l = 18;      //C7 PWM_L, GPIO , Pin
    Coils[C7].gpio_dir = 19;        //C7 Dir, GPIO , Pin
    Coils[C8].gpio_pwm_h = 10;      //C8 PWM_H, GPIO , Pin
    Coils[C8].gpio_pwm_l = 11;      //C8 PWM_L, GPIO , Pin
    Coils[C8].gpio_dir = 16;        //C8 Dir, GPIO , Pin
    Coils[C9].gpio_pwm_h = 77;      //C9 PWM_H, GPIO , Pin
    Coils[C9].gpio_pwm_l = 75;      //C9 PWM_L, GPIO , Pin
    Coils[C9].gpio_dir = 73;        //C9 Dir, GPIO , Pin
    Coils[C10].gpio_pwm_h = 83;     //C10 PWM_H, GPIO , Pin
    Coils[C10].gpio_pwm_l = 81;     //C10 PWM_L, GPIO , Pin
    Coils[C10].gpio_dir = 79;       //C10 Dir, GPIO , Pin
    Coils[C11].gpio_pwm_h = 89;     //C11 PWM_H, GPIO , Pin
    Coils[C11].gpio_pwm_l = 87;     //C11 PWM_L, GPIO , Pin
    Coils[C11].gpio_dir = 85;       //C11 Dir, GPIO , Pin
    Coils[C12].gpio_pwm_h = 133;    //C12 PWM_H, GPIO , Pin
    Coils[C12].gpio_pwm_l = 93;     //C12 PWM_L, GPIO , Pin
    Coils[C12].gpio_dir = 91;       //C12 Dir, GPIO , Pin
    Coils[C13].gpio_pwm_h = 20;     //C13 PWM_H, GPIO , Pin
    Coils[C13].gpio_pwm_l = 15;     //C13 PWM_L, GPIO , Pin
    Coils[C13].gpio_dir = 14;       //C13 Dir, GPIO , Pin
    Coils[C14].gpio_pwm_h = 31;     //C14 PWM_H, GPIO , Pin
    Coils[C14].gpio_pwm_l = 30;     //C14 PWM_L, GPIO , Pin
    Coils[C14].gpio_dir = 23;       //C14 Dir, GPIO , Pin
    Coils[C15].gpio_pwm_h = 44;     //C15 PWM_H, GPIO , Pin
    Coils[C15].gpio_pwm_l = 39;     //C15 PWM_L, GPIO , Pin
    Coils[C15].gpio_dir = 34;       //C15 Dir, GPIO , Pin
    Coils[C16].gpio_pwm_h = 55;     //C16 PWM_H, GPIO , Pin
    Coils[C16].gpio_pwm_l = 54;     //C16 PWM_L, GPIO , Pin
    Coils[C16].gpio_dir = 45;       //C16 Dir, GPIO , Pin
    Coils[C17].gpio_pwm_h = 58;     //C17 PWM_H, GPIO , Pin
    Coils[C17].gpio_pwm_l = 57;     //C17 PWM_L, GPIO , Pin
    Coils[C17].gpio_dir = 56;       //C17 Dir, GPIO , Pin
    Coils[C18].gpio_pwm_h = 38;     //C18 PWM_H, GPIO , Pin
    Coils[C18].gpio_pwm_l = 36;     //C18 PWM_L, GPIO , Pin
    Coils[C18].gpio_dir = 59;       //C18 Dir, GPIO , Pin
    Coils[C19].gpio_pwm_h = 65;     //C19 PWM_H, GPIO , Pin
    Coils[C19].gpio_pwm_l = 63;     //C19 PWM_L, GPIO , Pin
    Coils[C19].gpio_dir = 61;       //C19 Dir, GPIO , Pin
    Coils[C20].gpio_pwm_h = 71;     //C20 PWM_H, GPIO , Pin
    Coils[C20].gpio_pwm_l = 69;     //C20 PWM_L, GPIO , Pin
    Coils[C20].gpio_dir = 67;       //C20 Dir, GPIO , Pin
    Coils[C21].gpio_pwm_h = 37;     //C21 PWM_H, GPIO , Pin
    Coils[C21].gpio_pwm_l = 60;     //C21 PWM_L, GPIO , Pin
    Coils[C21].gpio_dir = 62;       //C21 Dir, GPIO , Pin
    Coils[C22].gpio_pwm_h = 52;     //C22 PWM_H, GPIO , Pin
    Coils[C22].gpio_pwm_l = 53;     //C22 PWM_L, GPIO , Pin
    Coils[C22].gpio_dir = 35;       //C22 Dir, GPIO , Pin
    Coils[C23].gpio_pwm_h = 49;     //C23 PWM_H, GPIO , Pin
    Coils[C23].gpio_pwm_l = 50;     //C23 PWM_L, GPIO , Pin
    Coils[C23].gpio_dir = 51;       //C23 Dir, GPIO , Pin
    Coils[C24].gpio_pwm_h = 40;     //C24 PWM_H, GPIO , Pin
    Coils[C24].gpio_pwm_l = 41;     //C24 PWM_L, GPIO , Pin
    Coils[C24].gpio_dir = 48;       //C24 Dir, GPIO , Pin
}

/*
 * Initialization functions for the GPIO pins for the coils
 */

void Coil_Gpio_Init(current *Coils,uint32_t *gpio_mask){
    int i;    
    for(i = 0; i < coil_count; i++){
        //Switch for PWM_H Pin
        switch((Gpio_Regs)((Coils[i].gpio_pwm_h >> 5) & 0x0007)){
            case Gpio_A:
                gpio_mask[Gpio_A] |= (uint32_t)1 << (Coils[i].gpio_pwm_h & 0x001F);
                break;
            case Gpio_B:
                gpio_mask[Gpio_B] |= (uint32_t)1 << (Coils[i].gpio_pwm_h & 0x001F);
                break;
            case Gpio_C:
                gpio_mask[Gpio_C] |= (uint32_t)1 << (Coils[i].gpio_pwm_h & 0x001F);
                break;
            case Gpio_D:
                gpio_mask[Gpio_D] |= (uint32_t)1 << (Coils[i].gpio_pwm_h & 0x001F);
                break;
            case Gpio_E:
                gpio_mask[Gpio_E] |= (uint32_t)1 << (Coils[i].gpio_pwm_h & 0x001F);
                break;
            case Gpio_F:
                gpio_mask[Gpio_F] |= (uint32_t)1 << (Coils[i].gpio_pwm_h & 0x001F);
                break;
        }
        //Switch for PWM_L Pin
        switch((Gpio_Regs)((Coils[i].gpio_pwm_l >> 5) & 0x0007)){
            case Gpio_A:
                gpio_mask[Gpio_A] |= (uint32_t)1 << (Coils[i].gpio_pwm_l & 0x001F);
                break;
            case Gpio_B:
                gpio_mask[Gpio_B] |= (uint32_t)1 << (Coils[i].gpio_pwm_l & 0x001F);
                break;
            case Gpio_C:
                gpio_mask[Gpio_C] |= (uint32_t)1 << (Coils[i].gpio_pwm_l & 0x001F);
                break;
            case Gpio_D:
                gpio_mask[Gpio_D] |= (uint32_t)1 << (Coils[i].gpio_pwm_l & 0x001F);
                break;
            case Gpio_E:
                gpio_mask[Gpio_E] |= (uint32_t)1 << (Coils[i].gpio_pwm_l & 0x001F);
                break;
            case Gpio_F:
                gpio_mask[Gpio_F] |= (uint32_t)1 << (Coils[i].gpio_pwm_l & 0x001F);
                break;
        }
        //Switch for Dir Pin
        switch((Gpio_Regs)((Coils[i].gpio_dir >> 5) & 0x0007)){
            case Gpio_A:
                gpio_mask[Gpio_A] |= (uint32_t)1 << (Coils[i].gpio_dir & 0x001F);
                break;
            case Gpio_B:
                gpio_mask[Gpio_B] |= (uint32_t)1 << (Coils[i].gpio_dir & 0x001F);
                break;
            case Gpio_C:
                gpio_mask[Gpio_C] |= (uint32_t)1 << (Coils[i].gpio_dir & 0x001F);
                break;
            case Gpio_D:
                gpio_mask[Gpio_D] |= (uint32_t)1 << (Coils[i].gpio_dir & 0x001F);
                break;
            case Gpio_E:
                gpio_mask[Gpio_E] |= (uint32_t)1 << (Coils[i].gpio_dir & 0x001F);
                break;
            case Gpio_F:
                gpio_mask[Gpio_F] |= (uint32_t)1 << (Coils[i].gpio_dir & 0x001F);
                break;
        }
        //EALLOW;
        //Set the Direction on each of the PWM_H, PWM_L and Dir pins
        GpioCtrlRegs.GPADIR.all |= gpio_mask[Gpio_A];     //Sets the selected GPIO A pins to output
        GpioCtrlRegs.GPBDIR.all |= gpio_mask[Gpio_B];     //Sets the selected GPIO B pins to output
        GpioCtrlRegs.GPCDIR.all |= gpio_mask[Gpio_C];     //Sets the selected GPIO C pins to output
        GpioCtrlRegs.GPDDIR.all |= gpio_mask[Gpio_D];     //Sets the selected GPIO D pins to output
        GpioCtrlRegs.GPEDIR.all |= gpio_mask[Gpio_E];     //Sets the selected GPIO E pins to output
        GpioCtrlRegs.GPFDIR.all |= gpio_mask[Gpio_F];     //Sets the selected GPIO F pins to output

        //Sets the Output low on each of the PWM_H, PWM_L and Dir pins
        GpioDataRegs.GPACLEAR.all = gpio_mask[Gpio_A];     //Sets the selected GPIO A pins low
        GpioDataRegs.GPBCLEAR.all = gpio_mask[Gpio_B];     //Sets the selected GPIO B pins low
        GpioDataRegs.GPCCLEAR.all = gpio_mask[Gpio_C];     //Sets the selected GPIO C pins low
        GpioDataRegs.GPDCLEAR.all = gpio_mask[Gpio_D];     //Sets the selected GPIO D pins low
        GpioDataRegs.GPECLEAR.all = gpio_mask[Gpio_E];     //Sets the selected GPIO E pins low
        GpioDataRegs.GPFCLEAR.all = gpio_mask[Gpio_F];     //Sets the selected GPIO F pins low
        //EDIS;
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
    //coil->x_influence = 0;          //Sets the current x value scaling constant to default of zero
    //coil->y_influence = 0;          //Sets the current y value scaling constant to default of zero
    //coil->bias = current_bias;      //Sets the current bias to the default define
}

/*
 * Position PID Control Function
 */

void Position_PID_Cntrl(position *sensor){
    //Read sample into structure
    int temp_div;
    sensor->sample = (*(sensor->sample_loc) * sensor->scale) + sensor->offset;    //*(Meters)

    //Initial error calculation between target and sample values
    sensor->error = sensor->sample - sensor->target;

    //Proportional term calculation
    sensor->p = sensor->error * sensor->kp;

    //Integral term calculation
        //This was fun to code, pretty much I subtract out the last value of the array at the same time I add in the new one!
    sensor->i = sensor->i + ((sensor->error - sensor->prev[sensor->prev_place]) * sensor->dt * sensor->ki);

    //Derivative term calculation
    temp_div = __divf32(sensor->kd,sensor->dt);
    sensor->d = (sensor->error - sensor->prev[sensor->prev_last]) * temp_div;

    //PID output
    sensor->pid_out = sensor->p + sensor->i + sensor->d;    //*(mm?)

    //clean up operations to set the array for the next round of PID calculations
    sensor->prev[sensor->prev_place] = sensor->error;   //Places the current rounds error into the memory array
    sensor->prev_last = sensor->prev_place;     //Progresses the placement variable for the "last" location to the current place
    sensor->prev_place = (prev_size - 1) & (sensor->prev_place + 1);    //Progresses the current location variable while using the bitwise math to limit and loop the variable
}

///*
// * Current Targetting
// */
//
//void current_target(current *Coils[], position *X, position *Y){
//    float temp;
//    int i;
//    for(i = 0;i < coil_count;i++){
//        /* In the next line is where the future addition of the X Y influence will be accounted for */
//        temp = (Coils[i]->bias * field_coil_bias) + (disp_to_current * ((Coils[i]->x_influence * (X->pid_out)) + ((Coils[i]->x_influence) * (Y->pid_out)))); //*(Amps)
//        if(temp > current_max || temp < current_min){
//            if(temp > current_max){
//                temp = current_max;     //Limits the target current to the max value
//            }
//            else{
//                temp = current_min;     //Limits the target current to the min value
//            }
//        }
//        //Sets the target for the coil
//        Coils[i]->target = temp; //*(Amps)
//    }
//
//}

/*
 * Lower Bearing Current Targeting
 */

void Bearing_Current_Target(current *Coils, position *X, position *Y){
    float X_wheel_demand = disp_to_current * (X->pid_out * __cospuf32(1 - Rotation_percent) + (Y->pid_out * __sinpuf32(1 - Rotation_percent)));    //X/Ystator to Xrotor translation
    float Y_wheel_demand = disp_to_current * (Y->pid_out * __cospuf32(1 - Rotation_percent) - (X->pid_out * __sinpuf32(1 - Rotation_percent)));    //X/Ystator to Yrotor translation
    float theta_coil_offset = 7.5;  //Offset dependant on coil locations and stator geometries
    float theta_coil_correct = Rotation_degrees + theta_coil_offset;    //Adds the coil location offset to the current rotation
    float theta_percent = __divf32(theta_coil_correct, 360);    //Per unit rotation
    int coil_offset = (int)(theta_percent * 24);    //Finite coil location offset
    int i = 0;  //Iteration Variable for the loop
    int temp_coil;  //Holder for theoretical coil location
    int Coil_demand_gain = 1;   //Gain for all of the coil bearing demands
    for(i = 0;i < coil_count; i++){ //Loop through all 24 coils

        //Circular offset calculation
        temp_coil = (i + coil_offset) % coil_count; //Circular offset calculation

        //Multiply the influences by the respective PID outputs to determine current target
        Coils[i].target = Coil_demand_gain * ((X_wheel_demand * (map_array[0][temp_coil])) + (Y_wheel_demand * (map_array[1][temp_coil])));
    }
}


///*
// * Lower Bearing Current demand calculations !!!Not Used!!!
// */
//
//void Current_Calc_Bearing(current *Coil_Array[], position *X, position *Y){
//    float X_wheel = disp_to_current * ((X->pid_out * __cospuf32(1 - Rotation_percent)) + (Y->pid_out * __sinpuf32(1 - Rotation_percent)));    //X/Ystator to Xrotor translation
//    float Y_wheel = disp_to_current * ((Y->pid_out * __cospuf32(1 - Rotation_percent)) - (X->pid_out * __sinpuf32(1 - Rotation_percent)));    //X/Ystator to Yrotor translation
//    float Bearing_Current   //Temporary variable for Bearing current
//    int i = 0;
//    for(i = 0;i < coil_count;i++){  //Runs current targetting for all coils
//        uint32_t current_location = (Rotation_correct + Coil_Array[i]->location) & 0x0003ffff;
//        switch((current_location >> 16) & 0x0003){
//        case 0:
//            if(((current_location >> 14) & 0x0003) == 0){
//                Bearing_Current = __divf32(current_location,0x00003FFF);
//            }
//            else{
//                if(((current_location >> 14) & 0x0003) == 3){
//                    Bearing_Current = 1 - (__divf32(current_location,0x00003FFF));
//                }
//                else{
//                    Bearing_Current = 1;
//                }
//            }
//            Coil_Array[i]->bearing_cpos = Bias + (Bearing_Current * Y_wheel);
//            Coil_Array[Coil_Array[i]->neighbor_coil]->bearing_cneg = Bearing_Current;
//            break;
//        case 1:
//            if(((current_location >> 14) & 0x0003) == 0){
//                Bearing_Current = __divf32(current_location,0x00003FFF);
//            }
//            else{
//                if(Ramp_down){
//                    Bearing_Current = 1 - (__divf32(current_location,0x00003FFF));
//                }
//                else{
//                    Bearing_Current = 1;
//                }
//            }
//            Coil_Array[i]->bearing_cpos = -(Bias + (Bearing_Current * X_wheel));
//            Coil_Array[Coil_Array[i]->neighbor_coil]->bearing_cneg = Bearing_Current;
//            break;
//        case 2:
//            if(((current_location >> 14) & 0x0003) == 0){
//                Bearing_Current = __divf32(current_location,0x00003FFF);
//            }
//            else{
//                if(((current_location >> 14) & 0x0003) == 3){
//                    Bearing_Current = 1 - (__divf32(current_location,0x00003FFF));
//                }
//                else{
//                    Bearing_Current = 1;
//                }
//            }
//            Coil_Array[i]->bearing_cpos = Bias + (Bearing_Current * (-Y_wheel));
//            Coil_Array[Coil_Array[i]->neighbor_coil]->bearing_cneg = Bearing_Current;
//            break;
//        case 3:
//            if(((current_location >> 14) & 0x0003) == 0){
//                Bearing_Current = __divf32(current_location,0x00003FFF);
//            }
//            else{
//                if(Ramp_down){
//                    Bearing_Current = 1 - (__divf32(current_location,0x00003FFF));
//                }
//                else{
//                    Bearing_Current = 1;
//                }
//            }
//            Coil_Array[i]->bearing_cpos = -(Bias + (Bearing_Current * (-X_wheel)));
//            Coil_Array[Coil_Array[i]->neighbor_coil]->bearing_cneg = Bearing_Current;
//            break;
//        }
//    }
//}


/*
 * Current Control Function
 */

void Bang_Bang_Cntrl(current *Coils){
    int temp_pwm_h = 0;
    int temp_dir = 0;
    int temp_pwm_l = 1;
    uint32_t GPA_temp;
    uint32_t GPB_temp;
    uint32_t GPC_temp;
    uint32_t GPD_temp;
    uint32_t GPE_temp;
    uint32_t GPF_temp;
    int i;
    for(i = 0; i < coil_count; i++){    
        //Conversion of ADC current reading into Amps
        Coils[i].sample = (*(Coils[i].sample_loc) * Coils[i].scale) + Coils[i].offset;  //*(Amps)
            /* Add in the rotation demands in the line below */
        //Coils[i].target = Coils[i].bearing_cpos - Coils[i].bearing_cneg;    //Summation of the demand for each part of the Coils

        //Initial error calculation between target and sampled current
        Coils[i].error = Coils[i].sample - Coils[i].target;

        //Run P_H pin high or low depending on error
        if(Coils[i].error < 0){
            temp_dir = 0;
            temp_pwm_h = 1;
        }
        if(Coils[i].error > 0){
            temp_dir = 1;
            temp_pwm_h = 1;
        }
        else{
            temp_pwm_h = 0;
        }
        switch((Gpio_Regs)((Coils[i].gpio_pwm_h >> 5) & 0x0007)){
            case Gpio_A:
                GPA_temp |= (uint32_t)temp_pwm_h << (Coils[i].gpio_pwm_h & 0x001F);
                break;
            case Gpio_B:
                GPB_temp |= (uint32_t)temp_pwm_h << (Coils[i].gpio_pwm_h & 0x001F);
                break;
            case Gpio_C:
                GPC_temp |= (uint32_t)temp_pwm_h << (Coils[i].gpio_pwm_h & 0x001F);
                break;
            case Gpio_D:
                GPD_temp |= (uint32_t)temp_pwm_h << (Coils[i].gpio_pwm_h & 0x001F);
                break;
            case Gpio_E:
                GPE_temp |= (uint32_t)temp_pwm_h << (Coils[i].gpio_pwm_h & 0x001F);
                break;
            case Gpio_F:
                GPF_temp |= (uint32_t)temp_pwm_h << (Coils[i].gpio_pwm_h & 0x001F);
                break;
        }
        switch((Gpio_Regs)((Coils[i].gpio_dir >> 5) & 0x0007)){
            case Gpio_A:
                GPA_temp |= (uint32_t)temp_dir << (Coils[i].gpio_dir & 0x001F);
                break;
            case Gpio_B:
                GPB_temp |= (uint32_t)temp_dir << (Coils[i].gpio_dir & 0x001F);
                break;
            case Gpio_C:
                GPC_temp |= (uint32_t)temp_dir << (Coils[i].gpio_dir & 0x001F);
                break;
            case Gpio_D:
                GPD_temp |= (uint32_t)temp_dir << (Coils[i].gpio_dir & 0x001F);
                break;
            case Gpio_E:
                GPE_temp |= (uint32_t)temp_dir << (Coils[i].gpio_dir & 0x001F);
                break;
            case Gpio_F:
                GPF_temp |= (uint32_t)temp_dir << (Coils[i].gpio_dir & 0x001F);
                break;
        }
        switch((Gpio_Regs)((Coils[i].gpio_pwm_l >> 5) & 0x0007)){
            case Gpio_A:
                GPA_temp |= (uint32_t)temp_pwm_l << (Coils[i].gpio_pwm_l & 0x001F);
                break;
            case Gpio_B:
                GPB_temp |= (uint32_t)temp_pwm_l << (Coils[i].gpio_pwm_l & 0x001F);
                break;
            case Gpio_C:
                GPC_temp |= (uint32_t)temp_pwm_l << (Coils[i].gpio_pwm_l & 0x001F);
                break;
            case Gpio_D:
                GPD_temp |= (uint32_t)temp_pwm_l << (Coils[i].gpio_pwm_l & 0x001F);
                break;
            case Gpio_E:
                GPE_temp |= (uint32_t)temp_pwm_l << (Coils[i].gpio_pwm_l & 0x001F);
                break;
            case Gpio_F:
                GPF_temp |= (uint32_t)temp_pwm_l << (Coils[i].gpio_pwm_l & 0x001F);
                break;
        }
    }
    //Sets the Output low on each of th PWM_H, PWM_L and Dir pins
        GpioDataRegs.GPACLEAR.all = gpio_mask[Gpio_A] & (~GPA_temp);     //Sets the selected GPIO A pins low
        GpioDataRegs.GPBCLEAR.all = gpio_mask[Gpio_B] & (~GPB_temp);     //Sets the selected GPIO B pins low
        GpioDataRegs.GPCCLEAR.all = gpio_mask[Gpio_C] & (~GPC_temp);     //Sets the selected GPIO C pins low
        GpioDataRegs.GPDCLEAR.all = gpio_mask[Gpio_D] & (~GPD_temp);     //Sets the selected GPIO D pins low
        GpioDataRegs.GPECLEAR.all = gpio_mask[Gpio_E] & (~GPE_temp);     //Sets the selected GPIO E pins low
        GpioDataRegs.GPFCLEAR.all = gpio_mask[Gpio_F] & (~GPF_temp);     //Sets the selected GPIO F pins low

        //Sets the Output low on each of th PWM_H, PWM_L and Dir pins
        GpioDataRegs.GPASET.all = gpio_mask[Gpio_A] & GPA_temp;     //Sets the selected GPIO A pins low
        GpioDataRegs.GPBSET.all = gpio_mask[Gpio_B] & GPB_temp;     //Sets the selected GPIO B pins low
        GpioDataRegs.GPCSET.all = gpio_mask[Gpio_C] & GPC_temp;     //Sets the selected GPIO C pins low
        GpioDataRegs.GPDSET.all = gpio_mask[Gpio_D] & GPD_temp;     //Sets the selected GPIO D pins low
        GpioDataRegs.GPESET.all = gpio_mask[Gpio_E] & GPE_temp;     //Sets the selected GPIO E pins low
        GpioDataRegs.GPFSET.all = gpio_mask[Gpio_F] & GPF_temp;     //Sets the selected GPIO F pins low

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
    CpuSysRegs.PCLKCR13.bit.ADC_C = 1;
    CpuSysRegs.PCLKCR13.bit.ADC_D = 1;

    //Initial setup function for ADCs, calibrates them to factory state
        /***Find this function in the F2837xD_ADC.c file***/
    AdcSetMode(ADC_ADCA, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);
    AdcSetMode(ADC_ADCB, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);
    AdcSetMode(ADC_ADCC, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);
    AdcSetMode(ADC_ADCD, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);

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

    //Initialization Settings for ADC B
    AdccRegs.ADCCTL1.bit.INTPULSEPOS = 1;   //Sets Interrupt pulse to trigger after conversion
    AdccRegs.ADCCTL2.bit.SIGNALMODE = 0;    //Single Ended, not Differential
    AdccRegs.ADCCTL2.bit.RESOLUTION = 0;    //Sets resolution to 12 bits, 1=16 bits
    AdccRegs.ADCCTL2.bit.PRESCALE = 8;      //Sets the Prescale of the ADC, 6=1/4   *Be careful with value, if it is to low, issues with sample value will occur

    //Initialization Settings for ADC B
    AdcdRegs.ADCCTL1.bit.INTPULSEPOS = 1;   //Sets Interrupt pulse to trigger after conversion
    AdcdRegs.ADCCTL2.bit.SIGNALMODE = 0;    //Single Ended, not Differential
    AdcdRegs.ADCCTL2.bit.RESOLUTION = 0;    //Sets resolution to 12 bits, 1=16 bits
    AdcdRegs.ADCCTL2.bit.PRESCALE = 8;      //Sets the Prescale of the ADC, 6=1/4   *Be careful with value, if it is to low, issues with sample value will occur


    AdcaRegs.ADCCTL1.bit.ADCPWDNZ = 1;      //Powers up all of the analog circuitry
    AdcbRegs.ADCCTL1.bit.ADCPWDNZ = 1;      //Powers up all of the analog circuitry
    AdccRegs.ADCCTL1.bit.ADCPWDNZ = 1;      //Powers up all of the analog circuitry
    AdcdRegs.ADCCTL1.bit.ADCPWDNZ = 1;      //Powers up all of the analog circuitry


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
    AdcaRegs.ADCSOC4CTL.bit.TRIGSEL = 5;    //Set the Trigger for SOC3, 5=ePWM1
    AdcaRegs.ADCSOC4CTL.bit.CHSEL = 4;      //Set the channel for SOC3 to convert, 0=ADCAIN3
    AdcaRegs.ADCSOC4CTL.bit.ACQPS = 19;     //Set the sample window size for SOC3, 19=20 SysClkCycles
    AdcaRegs.ADCSOC5CTL.bit.TRIGSEL = 5;    //Set the Trigger for SOC3, 5=ePWM1
    AdcaRegs.ADCSOC5CTL.bit.CHSEL = 5;      //Set the channel for SOC3 to convert, 0=ADCAIN3
    AdcaRegs.ADCSOC5CTL.bit.ACQPS = 19;     //Set the sample window size for SOC3, 19=20 SysClkCycles
    AdcaRegs.ADCINTSEL1N2.bit.INT1SEL = 5;  //Set EOCx to trigger ADCINT1, 0=EOC0
    AdcaRegs.ADCINTSEL1N2.bit.INT1E = 1;    //Enable/Disable ADCINT1
    AdcaRegs.ADCINTSEL1N2.bit.INT1CONT = 0; //Enable/Disable Continuous Mode
    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;  //Clears ADCAINT1 Flag

    //Configuration Settings for ADCB
    AdcbRegs.ADCSOC0CTL.bit.TRIGSEL = 5;    //Set the Trigger for SOC0, 5=ePWM2
    AdcbRegs.ADCSOC0CTL.bit.CHSEL = 0;      //Set the channel for SOC0 to convert, 0=ADCBIN0
    AdcbRegs.ADCSOC0CTL.bit.ACQPS = 19;     //Set the sample window size for SOC0, 19=20 SysClkCycles
    AdcbRegs.ADCSOC1CTL.bit.TRIGSEL = 5;    //Set the Trigger for SOC1, 5=ePWM2
    AdcbRegs.ADCSOC1CTL.bit.CHSEL = 1;      //Set the channel for SOC1 to convert, 1=ADCBIN1
    AdcbRegs.ADCSOC1CTL.bit.ACQPS = 19;     //Set the sample window size for SOC1, 19=20 SysClkCycles
    AdcbRegs.ADCSOC2CTL.bit.TRIGSEL = 5;    //Set the Trigger for SOC2, 5=ePWM2
    AdcbRegs.ADCSOC2CTL.bit.CHSEL = 2;      //Set the channel for SOC2 to convert, 0=ADCBIN0
    AdcbRegs.ADCSOC2CTL.bit.ACQPS = 19;     //Set the sample window size for SOC2, 19=20 SysClkCycles
    AdcbRegs.ADCSOC3CTL.bit.TRIGSEL = 5;    //Set the Trigger for SOC3, 5=ePWM2
    AdcbRegs.ADCSOC3CTL.bit.CHSEL = 3;      //Set the channel for SOC3 to convert, 1=ADCBIN1
    AdcbRegs.ADCSOC3CTL.bit.ACQPS = 19;     //Set the sample window size for SOC3, 19=20 SysClkCycles
    AdcbRegs.ADCSOC4CTL.bit.TRIGSEL = 5;    //Set the Trigger for SOC3, 5=ePWM2
    AdcbRegs.ADCSOC4CTL.bit.CHSEL = 4;      //Set the channel for SOC3 to convert, 1=ADCBIN1
    AdcbRegs.ADCSOC4CTL.bit.ACQPS = 19;     //Set the sample window size for SOC3, 19=20 SysClkCycles
    AdcbRegs.ADCSOC5CTL.bit.TRIGSEL = 5;    //Set the Trigger for SOC3, 5=ePWM2
    AdcbRegs.ADCSOC5CTL.bit.CHSEL = 5;      //Set the channel for SOC3 to convert, 1=ADCBIN1
    AdcbRegs.ADCSOC5CTL.bit.ACQPS = 19;     //Set the sample window size for SOC3, 19=20 SysClkCycles
    AdcbRegs.ADCINTSEL1N2.bit.INT2SEL = 5;  //Set EOCx to trigger ADCBINT2, 1=EOC1
    AdcbRegs.ADCINTSEL1N2.bit.INT2E = 1;    //Enable/Disable ADCBINT2
    AdcbRegs.ADCINTSEL1N2.bit.INT2CONT = 0; //Enable/Disable Continuous Mode
    AdcbRegs.ADCINTFLGCLR.bit.ADCINT2 = 1;  //Clears ADCBINT2 Flag

    //Configuration Settings for ADCC
    AdccRegs.ADCSOC0CTL.bit.TRIGSEL = 5;    //Set the Trigger for SOC0, 5=ePWM2
    AdccRegs.ADCSOC0CTL.bit.CHSEL = 14;      //Set the channel for SOC0 to convert, 0=ADCBIN0
    AdccRegs.ADCSOC0CTL.bit.ACQPS = 19;     //Set the sample window size for SOC0, 19=20 SysClkCycles
    AdccRegs.ADCSOC1CTL.bit.TRIGSEL = 5;    //Set the Trigger for SOC1, 5=ePWM2
    AdccRegs.ADCSOC1CTL.bit.CHSEL = 15;      //Set the channel for SOC1 to convert, 1=ADCBIN1
    AdccRegs.ADCSOC1CTL.bit.ACQPS = 19;     //Set the sample window size for SOC1, 19=20 SysClkCycles
    AdccRegs.ADCSOC2CTL.bit.TRIGSEL = 5;    //Set the Trigger for SOC2, 5=ePWM2
    AdccRegs.ADCSOC2CTL.bit.CHSEL = 2;      //Set the channel for SOC2 to convert, 0=ADCBIN0
    AdccRegs.ADCSOC2CTL.bit.ACQPS = 19;     //Set the sample window size for SOC2, 19=20 SysClkCycles
    AdccRegs.ADCSOC3CTL.bit.TRIGSEL = 5;    //Set the Trigger for SOC3, 5=ePWM2
    AdccRegs.ADCSOC3CTL.bit.CHSEL = 3;      //Set the channel for SOC3 to convert, 1=ADCBIN1
    AdccRegs.ADCSOC3CTL.bit.ACQPS = 19;     //Set the sample window size for SOC3, 19=20 SysClkCycles
    AdccRegs.ADCSOC4CTL.bit.TRIGSEL = 5;    //Set the Trigger for SOC3, 5=ePWM2
    AdccRegs.ADCSOC4CTL.bit.CHSEL = 4;      //Set the channel for SOC3 to convert, 1=ADCBIN1
    AdccRegs.ADCSOC4CTL.bit.ACQPS = 19;     //Set the sample window size for SOC3, 19=20 SysClkCycles
    AdccRegs.ADCSOC5CTL.bit.TRIGSEL = 5;    //Set the Trigger for SOC3, 5=ePWM2
    AdccRegs.ADCSOC5CTL.bit.CHSEL = 5;      //Set the channel for SOC3 to convert, 1=ADCBIN1
    AdccRegs.ADCSOC5CTL.bit.ACQPS = 19;     //Set the sample window size for SOC3, 19=20 SysClkCycles
    AdccRegs.ADCINTSEL1N2.bit.INT1SEL = 5;  //Set EOCx to trigger ADCINT1, 1=EOC1
    AdccRegs.ADCINTSEL1N2.bit.INT1E = 1;    //Enable/Disable ADCINT1
    AdccRegs.ADCINTSEL1N2.bit.INT1CONT = 0; //Enable/Disable Continuous Mode
    AdccRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;  //Clears ADCINT1 Flag

    //Configuration Settings for ADCD
    AdcdRegs.ADCSOC0CTL.bit.TRIGSEL = 5;    //Set the Trigger for SOC0, 5=ePWM2
    AdcdRegs.ADCSOC0CTL.bit.CHSEL = 0;      //Set the channel for SOC0 to convert, 0=ADCBIN0
    AdcdRegs.ADCSOC0CTL.bit.ACQPS = 19;     //Set the sample window size for SOC0, 19=20 SysClkCycles
    AdcdRegs.ADCSOC1CTL.bit.TRIGSEL = 5;    //Set the Trigger for SOC1, 5=ePWM2
    AdcdRegs.ADCSOC1CTL.bit.CHSEL = 1;      //Set the channel for SOC1 to convert, 1=ADCBIN1
    AdcdRegs.ADCSOC1CTL.bit.ACQPS = 19;     //Set the sample window size for SOC1, 19=20 SysClkCycles
    AdcdRegs.ADCSOC2CTL.bit.TRIGSEL = 5;    //Set the Trigger for SOC2, 5=ePWM2
    AdcdRegs.ADCSOC2CTL.bit.CHSEL = 2;      //Set the channel for SOC2 to convert, 0=ADCBIN0
    AdcdRegs.ADCSOC2CTL.bit.ACQPS = 19;     //Set the sample window size for SOC2, 19=20 SysClkCycles
    AdcdRegs.ADCSOC3CTL.bit.TRIGSEL = 5;    //Set the Trigger for SOC3, 5=ePWM2
    AdcdRegs.ADCSOC3CTL.bit.CHSEL = 3;      //Set the channel for SOC3 to convert, 1=ADCBIN1
    AdcdRegs.ADCSOC3CTL.bit.ACQPS = 19;     //Set the sample window size for SOC3, 19=20 SysClkCycles
    AdcdRegs.ADCSOC4CTL.bit.TRIGSEL = 5;    //Set the Trigger for SOC3, 5=ePWM2
    AdcdRegs.ADCSOC4CTL.bit.CHSEL = 4;      //Set the channel for SOC3 to convert, 1=ADCBIN1
    AdcdRegs.ADCSOC4CTL.bit.ACQPS = 19;     //Set the sample window size for SOC3, 19=20 SysClkCycles
    AdcdRegs.ADCSOC5CTL.bit.TRIGSEL = 5;    //Set the Trigger for SOC3, 5=ePWM2
    AdcdRegs.ADCSOC5CTL.bit.CHSEL = 5;      //Set the channel for SOC3 to convert, 1=ADCBIN1
    AdcdRegs.ADCSOC5CTL.bit.ACQPS = 19;     //Set the sample window size for SOC3, 19=20 SysClkCycles
    AdcdRegs.ADCINTSEL1N2.bit.INT1SEL = 5;  //Set EOCx to trigger ADCDINT1, 1=EOC1
    AdcdRegs.ADCINTSEL1N2.bit.INT1E = 1;    //Enable/Disable ADCDINT1 
    AdcdRegs.ADCINTSEL1N2.bit.INT1CONT = 0; //Enable/Disable Continuous Mode
    AdcdRegs.ADCINTFLGCLR.bit.ADCINT1  = 1;  //Clears ADCDINT1  Flag

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
    EPwm1Regs.TBPRD = 19999;     //Results in a period of 1/5 kHz

    //Disable ePWM1 phase sync
    EPwm1Regs.TBPHS.bit.TBPHS = 0;

    //Enable ePWM1 event Triggering selection
    EPwm1Regs.ETSEL.bit.SOCAEN = 0;     //Disable the SOCA conversion until ready
    EPwm1Regs.ETSEL.bit.SOCASEL = 1;    //Select SOCA to be triggered by ePWM1

    //Enable pulse on first event
    EPwm1Regs.ETPS.bit.SOCAPRD = 1;

    //Set the Compare A register
    EPwm1Regs.CMPA.bit.CMPA = 10000;      //Sets Compare A to a 50% duty cycle, arbitrary for now

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
    EPwm2Regs.TBPRD = 19999;     //Results in a period of 1/5 kHz

    //Disable ePWM2 phase sync
    EPwm2Regs.TBPHS.bit.TBPHS = 0;

    //Enable ePWM2 event Triggering selection
    EPwm2Regs.ETSEL.bit.SOCBEN = 0;     //Disable the SOCB conversion until ready
    EPwm2Regs.ETSEL.bit.SOCBSEL = 1;    //Select SOCB to be triggered by ePWM2

    //Enable pulse on first event
    EPwm2Regs.ETPS.bit.SOCBPRD = 1;

    //Set the Compare A register
    EPwm2Regs.CMPA.bit.CMPA = 10000;     //Sets Compare A to a 50% duty cycle, arbitrary for now

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
    PieVectTable.ADCC1_INT=&adcc1_isr;
    PieVectTable.ADCD1_INT=&adcd1_isr;

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
 * ADC A1 ISR
 */

interrupt void adca1_isr(){


    //Write the sample to the global variable
    c_samples[C5] = AdcaResultRegs.ADCRESULT0;  //Reads the result register of SOC0 for coil 5
    c_samples[C6] = AdcaResultRegs.ADCRESULT1;  //Reads the result register of SOC1 for coil 6
    c_samples[C7] = AdcaResultRegs.ADCRESULT2;  //Reads the result register of SOC2 for coil 7
    c_samples[C19] = AdcaResultRegs.ADCRESULT2;  //Reads the result register of SOC2 for coil 19
    c_samples[C8] = AdcaResultRegs.ADCRESULT3;  //Reads the result register of SOC3 for coil 8
    c_samples[C21] = AdcaResultRegs.ADCRESULT4;  //Reads the result register of SOC3 for coil 21
    c_samples[C22] = AdcaResultRegs.ADCRESULT5;  //Reads the result register of SOC3 for coil 22

    //Set the update flag
    ADCA_update = 1;  //Triggers the if statement in the main loop for Current Control

    //Clear the interrupt flag
    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1=1;    //Clears ADCA interrupt 1

    //Acknowledge the interrupt
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;     //Acknowledges the interrupt in the PIE table for ADCA interrupt 1
}

/*
 * ADC B2 ISR
 */

interrupt void adcb2_isr(){
    //Write the sample to the global variable
    /*c_samples[C13]*/ x1_sample = AdcbResultRegs.ADCRESULT0;  //Reads the result register of SOC0 for coil 13
    c_samples[C14] = AdcbResultRegs.ADCRESULT1;  //Reads the result register of SOC1 for coil 14
    c_samples[C15] = AdcbResultRegs.ADCRESULT2;  //Reads the result register of SOC2 for coil 15
    c_samples[C16] = AdcbResultRegs.ADCRESULT3;  //Reads the result register of SOC3 for coil 16
    c_samples[C17] = AdcbResultRegs.ADCRESULT4;  //Reads the result register of SOC4 for coil 17
    c_samples[C18] = AdcbResultRegs.ADCRESULT5;  //Reads the result register of SOC5 for coil 18

    //Set the update flag
    ADCB_update = 1;  //Triggers the if statement in the main loop for PID operations

    EPwm3Regs.TBCTL.bit.CTRMODE = 0;


    //Clear the interrupt flag
    AdcbRegs.ADCINTFLGCLR.bit.ADCINT2=1;    //Clears ADCB interrupt 2

    //Acknowledge the interrupt
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP10;    //Acknowledges the interrupt in the PIE table for ADCB interrupt 2
}

/*
 * ADC C1 ISR
 */

interrupt void adcc1_isr(){
    //Write the sample to the global variable
    c_samples[C23] = AdccResultRegs.ADCRESULT0;  //Reads the result register of SOC0 for coil 23
    c_samples[C24] = AdccResultRegs.ADCRESULT1;  //Reads the result register of SOC1 for coil 24
    c_samples[C1] = AdccResultRegs.ADCRESULT2;  //Reads the result register of SOC2 for coil 1
    c_samples[C13] = AdccResultRegs.ADCRESULT2;  //Reads the result register of SOC2 for coil 13
    c_samples[C2] = AdccResultRegs.ADCRESULT3;  //Reads the result register of SOC3 for coil 2
    c_samples[C3] = AdccResultRegs.ADCRESULT4;  //Reads the result register of SOC4 for coil 3
    c_samples[C4] = AdccResultRegs.ADCRESULT5;  //Reads the result register of SOC5 for coil 4

    //Set the update flag
    ADCC_update = 1;  //Triggers the if statement in the main loop for PID operations

    EPwm3Regs.TBCTL.bit.CTRMODE = 0;


    //Clear the interrupt flag
    AdccRegs.ADCINTFLGCLR.bit.ADCINT1=1;    //Clears ADCB interrupt 2

    //Acknowledge the interrupt
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;    //Acknowledges the interrupt in the PIE table for ADCB interrupt 2
}

/*
 * ADC D1 ISR
 */

interrupt void adcd1_isr(){
    //Write the sample to the global variable
    /*c_samples[C19]*/ y1_sample = AdcdResultRegs.ADCRESULT0;  //Reads the result register of SOC0 for coil 19
    c_samples[C20] = AdcdResultRegs.ADCRESULT1;  //Reads the result register of SOC1 for coil 20
    c_samples[C9] = AdcdResultRegs.ADCRESULT2;  //Reads the result register of SOC2 for coil 9
    c_samples[C10] = AdcdResultRegs.ADCRESULT3;  //Reads the result register of SOC3 for coil 10
    c_samples[C11] = AdcdResultRegs.ADCRESULT4;  //Reads the result register of SOC4 for coil 11
    c_samples[C12] = AdcdResultRegs.ADCRESULT5;  //Reads the result register of SOC5 for coil 12

    //Set the update flag
    ADCD_update = 1;  //Triggers the if statement in the main loop for PID operations

    EPwm3Regs.TBCTL.bit.CTRMODE = 0;


    //Clear the interrupt flag
    AdcdRegs.ADCINTFLGCLR.bit.ADCINT1=1;    //Clears ADCB interrupt 2

    //Acknowledge the interrupt
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;    //Acknowledges the interrupt in the PIE table for ADCB interrupt 2
}

/*
 * ePWM3 ISR
 * Note: FSM for the BISS comunication with the optical encoder. GPIO9 is the MISO pin and GPIO4 is CLK.
 */


interrupt void epwm3_isr(){
    //Progress the state to the next state
    c_state = n_state;

    //GpioDataRegs.GPBTOGGLE.all = (uint32_t)c_state << 16;

    //Read the miso pin
    miso = GpioDataRegs.GPADAT.bit.GPIO9;

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
                Rotation_reading = Data_value;
                Rotation_correct = (Rotation_reading + Hw_offset) & 0x0003ffff;
                Rotation_percent = __divf32(Rotation_correct,Max_value);
                Rotation_degrees = Rotation_percent * 360;
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
