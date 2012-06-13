/**************************************************************************************/
/***************                  Motor Pin order                  ********************/
/**************************************************************************************/
// since we are uing the PWM generation in a direct way, the pin order is just to inizialie the right pins 
// its not possible to change a PWM output pin just by changing the order
#if defined(PROMINI)
  uint8_t PWM_PIN[8] = {9,10,11,3,6,5,A2,12};   //for a quad+: rear,right,left,front
#endif
#if defined(PROMICRO)
  #if !defined(HWPWM6)
    #if !defined(TEENSY20)
      uint8_t PWM_PIN[8] = {9,10,5,6,4,A2,SW_PWM_P3,SW_PWM_P4};   //for a quad+: rear,right,left,front
    #else
      uint8_t PWM_PIN[8] = {14,15,9,12,22,18,16,17};   //for a quad+: rear,right,left,front
    #endif
  #else
    #if !defined(TEENSY20)
      uint8_t PWM_PIN[8] = {9,10,5,6,11,13,SW_PWM_P3,SW_PWM_P4};   //for a quad+: rear,right,left,front
    #else
      uint8_t PWM_PIN[8] = {14,15,9,12,4,10,16,17};   //for a quad+: rear,right,left,front
    #endif
  #endif
#endif
#if defined(MEGA)
  uint8_t PWM_PIN[8] = {3,5,6,2,7,8,9,10};      //for a quad+: rear,right,left,front   //+ for y6: 7:under right  8:under left
#endif

/**************************************************************************************/
/***************         Software PWM & Servo variables            ********************/
/**************************************************************************************/
#if defined(PROMINI) || (defined(PROMICRO) && defined(HWPWM6))
  #if defined(SERVO)
    #if defined(AIRPLANE)|| defined(HELICOPTER)
      // To prevent motor to start at reset. atomicServo[7]=5 or 249 if reversed servo
      volatile uint8_t atomicServo[8] = {125,125,125,125,125,125,125,5}; 
    #else
      volatile uint8_t atomicServo[8] = {125,125,125,125,125,125,125,125};
    #endif
  #endif
  #if (NUMBER_MOTOR > 4)
    //for HEX Y6 and HEX6/HEX6X flat for promini
    volatile uint8_t atomicPWM_PIN5_lowState;
    volatile uint8_t atomicPWM_PIN5_highState;
    volatile uint8_t atomicPWM_PIN6_lowState;
    volatile uint8_t atomicPWM_PIN6_highState;
  #endif
  #if (NUMBER_MOTOR > 6)
    //for OCTO on promini
    volatile uint8_t atomicPWM_PINA2_lowState;
    volatile uint8_t atomicPWM_PINA2_highState;
    volatile uint8_t atomicPWM_PIN12_lowState;
    volatile uint8_t atomicPWM_PIN12_highState;
  #endif
#else
  #if defined(SERVO)
    #if defined(AIRPLANE)|| defined(HELICOPTER)
      // To prevent motor to start at reset. atomicServo[7]=5 or 249 if reversed servo
      volatile uint8_t atomicServo[8] = {8000,8000,8000,8000,8000,8000,8000,8000}; 
    #else
      volatile uint16_t atomicServo[8] = {8000,8000,8000,8000,8000,8000,8000,320};
    #endif
  #endif
  #if (NUMBER_MOTOR > 4)
    //for HEX Y6 and HEX6/HEX6X and for Promicro
    volatile uint16_t atomicPWM_PIN5_lowState;
    volatile uint16_t atomicPWM_PIN5_highState;
    volatile uint16_t atomicPWM_PIN6_lowState;
    volatile uint16_t atomicPWM_PIN6_highState;
  #endif
  #if (NUMBER_MOTOR > 6)
    //for OCTO on Promicro
    volatile uint16_t atomicPWM_PINA2_lowState;
    volatile uint16_t atomicPWM_PINA2_highState;
    volatile uint16_t atomicPWM_PIN12_lowState;
    volatile uint16_t atomicPWM_PIN12_highState;
  #endif
#endif

/**************************************************************************************/
/***************   Writes the Servos values to the needed format   ********************/
/**************************************************************************************/
void writeServos() {
  #if defined(SERVO)
    #if defined(PRI_SERVO_FROM)    // write primary servos
      for(uint8_t i = (PRI_SERVO_FROM-1); i < PRI_SERVO_TO; i++){
        #if defined(PROMINI) || (defined(PROMICRO) && defined(HWPWM6))
          atomicServo[i] = (servo[i]-1000)>>2;
        #else
          atomicServo[i] = (servo[i]-1000)<<4;
        #endif
      }
    #endif
    #if defined(SEC_SERVO_FROM)   // write secundary servos
      #if defined(SERVO_TILT) && defined(MMSERVOGIMBAL)
        // Moving Average Servo Gimbal by Magnetron1
        static int16_t mediaMobileServoGimbalADC[3][MMSERVOGIMBALVECTORLENGHT];
        static int32_t mediaMobileServoGimbalADCSum[3];
        static uint8_t mediaMobileServoGimbalIDX;
        uint8_t axis;

        mediaMobileServoGimbalIDX = ++mediaMobileServoGimbalIDX % MMSERVOGIMBALVECTORLENGHT;
        for (axis=(SEC_SERVO_FROM-1); axis < SEC_SERVO_TO; axis++) {
          mediaMobileServoGimbalADCSum[axis] -= mediaMobileServoGimbalADC[axis][mediaMobileServoGimbalIDX];
          mediaMobileServoGimbalADC[axis][mediaMobileServoGimbalIDX] = servo[axis];
          mediaMobileServoGimbalADCSum[axis] += mediaMobileServoGimbalADC[axis][mediaMobileServoGimbalIDX];
          #if defined(PROMINI) || (defined(PROMICRO) && defined(HWPWM6))
            atomicServo[axis] = (mediaMobileServoGimbalADCSum[axis] / MMSERVOGIMBALVECTORLENGHT - 1000)>>2;
          #else
            atomicServo[axis] = (mediaMobileServoGimbalADCSum[axis] / MMSERVOGIMBALVECTORLENGHT - 1000)<<4;
          #endif
        }
      #else
        for(uint8_t i = (SEC_SERVO_FROM-1); i < SEC_SERVO_TO; i++){
          #if defined(PROMINI) || (defined(PROMICRO) && defined(HWPWM6))
            atomicServo[i] = (servo[i]-1000)>>2;
          #else
            atomicServo[i] = (servo[i]-1000)<<4;
          #endif
        }
      #endif
    #endif
  #endif
}

/**************************************************************************************/
/************  Writes the Motors values to the PWM compare register  ******************/
/**************************************************************************************/
void writeMotors() { // [1000;2000] => [125;250]

/****************  Specific PWM Timers & Registers for the MEGA's   *******************/
  #if defined(MEGA)// [1000:2000] => [8000:16000] for timer 3 & 4 for mega
    #if (NUMBER_MOTOR > 0) 
      #ifndef EXT_MOTOR_RANGE 
        OCR3C = motor[0]<<3; //  pin 3
      #else
        OCR3C = ((motor[0]<<4) - 16000) + 128;
      #endif
    #endif
    #if (NUMBER_MOTOR > 1)
      #ifndef EXT_MOTOR_RANGE 
        OCR3A = motor[1]<<3; //  pin 5
      #else
        OCR3A = ((motor[1]<<4) - 16000) + 128;
      #endif
    #endif
    #if (NUMBER_MOTOR > 2)
      #ifndef EXT_MOTOR_RANGE 
        OCR4A = motor[2]<<3; //  pin 6
      #else
        OCR4A = ((motor[2]<<4) - 16000) + 128;
      #endif
    #endif
    #if (NUMBER_MOTOR > 3)
      #ifndef EXT_MOTOR_RANGE 
        OCR3B = motor[3]<<3; //  pin 2
      #else
        OCR3B = ((motor[3]<<4) - 16000) + 128;
      #endif
    #endif
    #if (NUMBER_MOTOR > 4)
      #ifndef EXT_MOTOR_RANGE 
        OCR4B = motor[4]<<3; //  pin 7
        OCR4C = motor[5]<<3; //  pin 8
      #else
        OCR4B = ((motor[4]<<4) - 16000) + 128;
        OCR4C = ((motor[5]<<4) - 16000) + 128;
      #endif
    #endif
    #if (NUMBER_MOTOR > 6)
      #ifndef EXT_MOTOR_RANGE 
        OCR2B = motor[6]>>3; //  pin 9
        OCR2A = motor[7]>>3; //  pin 10
      #else
        OCR2B = ((motor[6]>>2) - 250) + 2);
        OCR2A = ((motor[7]>>2) - 250) + 2);
      #endif
    #endif
  #endif
  
/******** Specific PWM Timers & Registers for the atmega32u4 (Promicro)   ************/
  #if defined(PROMICRO)
    #if (NUMBER_MOTOR > 0) // Timer 1 A & B [1000:2000] => [8000:16000]
      OCR1A = motor[0]<<3; //  pin 9
    #endif
    #if (NUMBER_MOTOR > 1)
      OCR1B = motor[1]<<3; //  pin 10
    #endif
    #if (NUMBER_MOTOR > 2) // Timer 4 A & D [1000:2000] => [1000:2000]
      #if !defined(HWPWM6)
        // to write values > 255 to timer 4 A/B we need to split the bytes
        TC4H = (2047-motor[2])>>8; OCR4A = ((2047-motor[2])&0xFF); //  pin 5
      #else
        OCR3A = motor[2]<<3; //  pin 5
      #endif
    #endif
    #if (NUMBER_MOTOR > 3)
      TC4H = motor[3]>>8; OCR4D = (motor[3]&0xFF); //  pin 6
    #endif    
    #if (NUMBER_MOTOR > 4)
      #if !defined(HWPWM6)
        #if (NUMBER_MOTOR == 6) && !defined(SERVO)
          atomicPWM_PIN5_highState = motor[4]<<3;
          atomicPWM_PIN5_lowState = 16383-atomicPWM_PIN5_highState;
          atomicPWM_PIN6_highState = motor[5]<<3;
          atomicPWM_PIN6_lowState = 16383-atomicPWM_PIN6_highState;      
        #else
          atomicPWM_PIN5_highState = ((motor[4]-1000)<<4)+320;
          atomicPWM_PIN5_lowState = 15743-atomicPWM_PIN5_highState;
          atomicPWM_PIN6_highState = ((motor[5]-1000)<<4)+320;
          atomicPWM_PIN6_lowState = 15743-atomicPWM_PIN6_highState;        
        #endif
      #else
        OCR1C = motor[4]<<3; //  pin 11
        TC4H = motor[5]>>8; OCR4A = (motor[5]&0xFF); //  pin 13    
      #endif
    #endif
    #if (NUMBER_MOTOR > 6)
      #if !defined(HWPWM6)
        atomicPWM_PINA2_highState = ((motor[6]-1000)<<4)+320;
        atomicPWM_PINA2_lowState = 15743-atomicPWM_PINA2_highState;
        atomicPWM_PIN12_highState = ((motor[7]-1000)<<4)+320;
        atomicPWM_PIN12_lowState = 15743-atomicPWM_PIN12_highState;
      #else
        atomicPWM_PINA2_highState = ((motor[6]-1000)>>2)+5;
        atomicPWM_PINA2_lowState = 245-atomicPWM_PINA2_highState;
        atomicPWM_PIN12_highState = ((motor[7]-1000)>>2)+5;
        atomicPWM_PIN12_lowState = 245-atomicPWM_PIN12_highState;     
      #endif
    #endif
  #endif
  
/********  Specific PWM Timers & Registers for the atmega328P (Promini)   ************/
  #if defined(PROMINI)
    #if (NUMBER_MOTOR > 0)
      #ifndef EXT_MOTOR_RANGE 
        OCR1A = motor[0]>>3; //  pin 9
      #else
        OCR1A = ((motor[0]>>2) - 250) + 2;
      #endif
    #endif
    #if (NUMBER_MOTOR > 1)
      #ifndef EXT_MOTOR_RANGE 
        OCR1B = motor[1]>>3; //  pin 10
      #else
        OCR1B = ((motor[1]>>2) - 250) + 2;
      #endif
    #endif
    #if (NUMBER_MOTOR > 2)
      #ifndef EXT_MOTOR_RANGE
        OCR2A = motor[2]>>3; //  pin 11
      #else
        OCR2A = ((motor[2]>>2) - 250) + 2;
      #endif
    #endif
    #if (NUMBER_MOTOR > 3)
      #ifndef EXT_MOTOR_RANGE
        OCR2B = motor[3]>>3; //  pin 3
      #else
        OCR2B = ((motor[3]>>2) - 250) + 2;
      #endif
    #endif
    #if (NUMBER_MOTOR > 4)
      #if (NUMBER_MOTOR == 6) && !defined(SERVO)
        #ifndef EXT_MOTOR_RANGE 
          atomicPWM_PIN6_highState = motor[4]>>3;
          atomicPWM_PIN5_highState = motor[5]>>3;
        #else
          atomicPWM_PIN6_highState = ((motor[4]>>2) - 250) + 2;
          atomicPWM_PIN5_highState = ((motor[5]>>2) - 250) + 2;       
        #endif
        atomicPWM_PIN6_lowState  = 255-atomicPWM_PIN6_highState;
        atomicPWM_PIN5_lowState  = 255-atomicPWM_PIN5_highState; 
      #else //note: EXT_MOTOR_RANGE not possible here
        atomicPWM_PIN6_highState = ((motor[4]-1000)>>2)+5;
        atomicPWM_PIN6_lowState  = 245-atomicPWM_PIN6_highState;
        atomicPWM_PIN5_highState = ((motor[5]-1000)>>2)+5;
        atomicPWM_PIN5_lowState  = 245-atomicPWM_PIN5_highState;
      #endif
    #endif
    #if (NUMBER_MOTOR > 6) //note: EXT_MOTOR_RANGE not possible here
      atomicPWM_PINA2_highState = ((motor[6]-1000)>>2)+5;
      atomicPWM_PINA2_lowState  = 245-atomicPWM_PINA2_highState;
      atomicPWM_PIN12_highState = ((motor[7]-1000)>>2)+5;
      atomicPWM_PIN12_lowState  = 245-atomicPWM_PIN12_highState;
    #endif
  #endif
}

/**************************************************************************************/
/************          Writes the mincommand to all Motors           ******************/
/**************************************************************************************/
void writeAllMotors(int16_t mc) {   // Sends commands to all motors
  for (uint8_t i =0;i<NUMBER_MOTOR;i++) {
    motor[i]=mc;
  }
  writeMotors();
}

/**************************************************************************************/
/************        Initialize the PWM Timers and Registers         ******************/
/**************************************************************************************/
void initOutput() {
  
/****************            mark all PWM pins as Output             ******************/
  for(uint8_t i=0;i<NUMBER_MOTOR;i++) {
    pinMode(PWM_PIN[i],OUTPUT);
  }
    
/****************  Specific PWM Timers & Registers for the MEGA's    ******************/
  #if defined(MEGA)
    #if (NUMBER_MOTOR > 0)
      // init 16bit timer 3
      TCCR3A |= (1<<WGM31); // phase correct mode
      TCCR3A &= ~(1<<WGM30);
      TCCR3B |= (1<<WGM33);
      TCCR3B &= ~(1<<CS31); // no prescaler
      ICR3   |= 0x3FFF; // TOP to 16383;      
      
      TCCR3A |= _BV(COM3C1); // connect pin 3 to timer 3 channel C
    #endif
    #if (NUMBER_MOTOR > 1)
      TCCR3A |= _BV(COM3A1); // connect pin 5 to timer 3 channel A
    #endif
    #if (NUMBER_MOTOR > 2)
      // init 16bit timer 4
      TCCR4A |= (1<<WGM41); // phase correct mode
      TCCR4A &= ~(1<<WGM40);
      TCCR4B |= (1<<WGM43);
      TCCR4B &= ~(1<<CS41); // no prescaler
      ICR4   |= 0x3FFF; // TOP to 16383;    
      
      TCCR4A |= _BV(COM4A1); // connect pin 6 to timer 4 channel A
    #endif
    #if (NUMBER_MOTOR > 3)
      TCCR3A |= _BV(COM3B1); // connect pin 2 to timer 3 channel B
    #endif
    #if (NUMBER_MOTOR > 4)
      TCCR4A |= _BV(COM4B1); // connect pin 7 to timer 4 channel B
      TCCR4A |= _BV(COM4C1); // connect pin 8 to timer 4 channel C
    #endif
    #if (NUMBER_MOTOR > 6)
      // timer 2 is a 8bit timer so we cant change its range 
      TCCR2A |= _BV(COM2B1); // connect pin 9 to timer 2 channel B
      TCCR2A |= _BV(COM2A1); // connect pin 10 to timer 2 channel A
    #endif
  #endif
  
/******** Specific PWM Timers & Registers for the atmega32u4 (Promicro)   ************/
  #if defined(PROMICRO)
    #if (NUMBER_MOTOR > 0)
      TCCR1A |= (1<<WGM11); // phase correct mode & no prescaler
      TCCR1A &= ~(1<<WGM10);
      TCCR1B &= ~(1<<WGM12) &  ~(1<<CS11) & ~(1<<CS12);
      TCCR1B |= (1<<WGM13) | (1<<CS10); 
      ICR1   |= 0x3FFF; // TOP to 16383;     
      TCCR1A |= _BV(COM1A1); // connect pin 9 to timer 1 channel A
    #endif
    #if (NUMBER_MOTOR > 1)
      TCCR1A |= _BV(COM1B1); // connect pin 10 to timer 1 channel B
    #endif
    #if (NUMBER_MOTOR > 2)
      #if !defined(HWPWM6) // timer 4A
        TCCR4E |= (1<<ENHC4); // enhanced pwm mode
        TCCR4B &= ~(1<<CS41); TCCR4B |= (1<<CS42)|(1<<CS40); // prescaler to 16
        TCCR4D |= (1<<WGM40); TC4H = 0x3; OCR4C = 0xFF; // phase and frequency correct mode & top to 1023 but with enhanced pwm mode we have 2047
        TCCR4A |= (1<<COM4A0)|(1<<PWM4A); // connect pin 5 to timer 4 channel A 
      #else // timer 3A
        TCCR3A |= (1<<WGM31); // phase correct mode & no prescaler
        TCCR3A &= ~(1<<WGM30);
        TCCR3B &= ~(1<<WGM32) &  ~(1<<CS31) & ~(1<<CS32);
        TCCR3B |= (1<<WGM33) | (1<<CS30); 
        ICR3   |= 0x3FFF; // TOP to 16383;     
        TCCR3A |= _BV(COM3A1); // connect pin 5 to timer 3 channel A    
      #endif 
    #endif
    #if (NUMBER_MOTOR > 3)
      #if defined(HWPWM6) 
        TCCR4E |= (1<<ENHC4); // enhanced pwm mode
        TCCR4B &= ~(1<<CS41); TCCR4B |= (1<<CS42)|(1<<CS40); // prescaler to 16
        TCCR4D |= (1<<WGM40); TC4H = 0x3; OCR4C = 0xFF; // phase and frequency correct mode & top to 1023 but with enhanced pwm mode we have 2047
      #endif
      TCCR4C |= (1<<COM4D1)|(1<<PWM4D); // connect pin 6 to timer 4 channel D
    #endif
    #if (NUMBER_MOTOR > 4)
      #if defined(HWPWM6) 
        TCCR1A |= _BV(COM1C1); // connect pin 11 to timer 1 channel C
        TCCR4A |= (1<<COM4A1)|(1<<PWM4A); // connect pin 13 to timer 4 channel A 
      #else
        initializeSoftPWM();
      #endif
    #endif
    #if (NUMBER_MOTOR > 6)
      #if defined(HWPWM6) 
        initializeSoftPWM();
      #endif
    #endif
  #endif
  
 /********  Specific PWM Timers & Registers for the atmega328P (Promini)   ************/
  #if defined(PROMINI)
    #if (NUMBER_MOTOR > 0)
      TCCR1A |= _BV(COM1A1); // connect pin 9 to timer 1 channel A
    #endif
    #if (NUMBER_MOTOR > 1)
      TCCR1A |= _BV(COM1B1); // connect pin 10 to timer 1 channel B
    #endif
    #if (NUMBER_MOTOR > 2)
      TCCR2A |= _BV(COM2A1); // connect pin 11 to timer 2 channel A
    #endif
    #if (NUMBER_MOTOR > 3)
      TCCR2A |= _BV(COM2B1); // connect pin 3 to timer 2 channel B
    #endif
    #if (NUMBER_MOTOR > 5)  // PIN 5 & 6 or A0 & A1
      initializeSoftPWM();
      #if defined(A0_A1_PIN_HEX) || (NUMBER_MOTOR > 6)
        pinMode(5,INPUT);pinMode(6,INPUT);     // we reactivate the INPUT affectation for these two PINs
        pinMode(A0,OUTPUT);pinMode(A1,OUTPUT);
      #endif
    #endif
  #endif
  
  writeAllMotors(MINCOMMAND);
  delay(300);
  #if defined(SERVO)
    initializeServo();
  #endif
}


#if defined(SERVO)
/**************************************************************************************/
/************                Initialize the PWM Servos               ******************/
/**************************************************************************************/
void initializeServo() {
  #if (PRI_SERVO_FROM == 1) || (SEC_SERVO_FROM == 1)
    SERVO_1_PINMODE;
  #endif
  #if (PRI_SERVO_FROM <= 2 && PRI_SERVO_TO >= 2) || (SEC_SERVO_FROM <= 2 && SEC_SERVO_TO >= 2) 
    SERVO_2_PINMODE;
  #endif
  #if (PRI_SERVO_FROM <= 3 && PRI_SERVO_TO >= 3) || (SEC_SERVO_FROM <= 3 && SEC_SERVO_TO >= 3) 
    SERVO_3_PINMODE;
  #endif 
  #if (PRI_SERVO_FROM <= 4 && PRI_SERVO_TO >= 4) || (SEC_SERVO_FROM <= 4 && SEC_SERVO_TO >= 4) 
    SERVO_4_PINMODE;
  #endif 
  #if (PRI_SERVO_FROM <= 5 && PRI_SERVO_TO >= 5) || (SEC_SERVO_FROM <= 5 && SEC_SERVO_TO >= 5) 
    SERVO_5_PINMODE;
  #endif 
  #if (PRI_SERVO_FROM <= 6 && PRI_SERVO_TO >= 6) || (SEC_SERVO_FROM <= 6 && SEC_SERVO_TO >= 6) 
    SERVO_6_PINMODE;
  #endif 
  #if (PRI_SERVO_FROM <= 7 && PRI_SERVO_TO >= 7) || (SEC_SERVO_FROM <= 7 && SEC_SERVO_TO >= 7) 
    SERVO_7_PINMODE;
  #endif 
  #if (PRI_SERVO_FROM <= 8 && PRI_SERVO_TO >= 8) || (SEC_SERVO_FROM <= 8 && SEC_SERVO_TO >= 8) 
    SERVO_8_PINMODE;
  #endif
  
  #if defined(PROMINI) || (defined(PROMICRO) && defined(HWPWM6)) // uses timer 0 Comperator A (8 bit)
    TCCR0A = 0; // normal counting mode
    TIMSK0 |= (1<<OCIE0A); // Enable CTC interrupt
    #define SERVO_ISR TIMER0_COMPA_vect
    #define SERVO_CHANNEL OCR0A
    #define SERVO_1K_US 250
  #endif
  #if (defined(PROMICRO) && !defined(HWPWM6)) // uses timer 3 Comperator A (11 bit)
    TCCR3A &= ~(1<<WGM30) & ~(1<<WGM31); //normal counting & no prescaler
    TCCR3B &= ~(1<<WGM32) & ~(1<<CS31) & ~(1<<CS32) & ~(1<<WGM33);
    TCCR3B |= (1<<CS30);   
    TIMSK3 |= (1<<OCIE3A); // Enable CTC interrupt 
    #define SERVO_ISR TIMER3_COMPA_vect
    #define SERVO_CHANNEL OCR3A
    #define SERVO_1K_US 16000 
  #endif
  #if defined(MEGA) // uses timer 5 Comperator A (11 bit)
    TCCR5A &= ~(1<<WGM50) & ~(1<<WGM51); //normal counting & no prescaler
    TCCR5B &= ~(1<<WGM52) & ~(1<<CS51) & ~(1<<CS52) & ~(1<<WGM53);
    TCCR5B |= (1<<CS50);   
    TIMSK5 |= (1<<OCIE5A); // Enable CTC interrupt  
    #define SERVO_ISR TIMER5_COMPA_vect
    #define SERVO_CHANNEL OCR5A
    #define SERVO_1K_US 16000 
  #endif
}

/**************************************************************************************/
/************              Servo software PWM generation             ******************/
/**************************************************************************************/

// prescaler is set by default to 64 on Timer0
// Duemilanove : 16MHz / 64 => 4 us
// 256 steps = 1 counter cycle = 1024 us

// for servo 2-8
// its almost the same as for servo 1
#define SERVO_PULSE(PIN_HIGH,ACT_STATE,SERVO_NUM,LAST_PIN_LOW) \
  }else if(state == ACT_STATE){                                \
    LAST_PIN_LOW;                                              \
    PIN_HIGH;                                                  \
    SERVO_CHANNEL+=SERVO_1K_US;                                \
    state++;                                                   \
  }else if(state == ACT_STATE+1){                              \
    SERVO_CHANNEL+=atomicServo[SERVO_NUM];                     \
    state++;                                                   \

ISR(SERVO_ISR) {
  static uint8_t state = 0; // indicates the current state of the chain
  if(state == 0){
    SERVO_1_HIGH; // set servo 1's pin high 
    SERVO_CHANNEL+=SERVO_1K_US; // wait 1000us
    state++; // count up the state
  }else if(state==1){
    SERVO_CHANNEL+=atomicServo[SERVO_1_ARR_POS]; // load the servo's value (0-1000us)
    state++; // count up the state
  #if defined(SERVO_2_HIGH)
    SERVO_PULSE(SERVO_2_HIGH,2,SERVO_2_ARR_POS,SERVO_1_LOW); // the same here
  #endif
  #if defined(SERVO_3_HIGH)
    SERVO_PULSE(SERVO_3_HIGH,4,SERVO_3_ARR_POS,SERVO_2_LOW);
  #endif
  #if defined(SERVO_4_HIGH)
    SERVO_PULSE(SERVO_4_HIGH,6,SERVO_4_ARR_POS,SERVO_3_LOW);
  #endif
  #if defined(SERVO_5_HIGH)
    SERVO_PULSE(SERVO_5_HIGH,8,SERVO_5_ARR_POS,SERVO_4_LOW);
  #endif
  #if defined(SERVO_6_HIGH)
    SERVO_PULSE(SERVO_6_HIGH,10,SERVO_6_ARR_POS,SERVO_5_LOW);
  #endif
  #if defined(SERVO_7_HIGH)
    SERVO_PULSE(SERVO_7_HIGH,12,SERVO_7_ARR_POS,SERVO_6_LOW);
  #endif
  #if defined(SERVO_8_HIGH)
    SERVO_PULSE(SERVO_8_HIGH,14,SERVO_8_ARR_POS,SERVO_7_LOW);
  #endif
  }else{
    LAST_LOW;
    #if defined(SERVO_RFR_300HZ)
      #if defined(SERVO_3_HIGH)  // if there are 3 or more servos we dont need to slow it down
        SERVO_CHANNEL+=(SERVO_1K_US>>3); // 0 would be better but it causes bad jitter
        state=0; 
      #else // if there are less then 3 servos we need to slow it to not go over 300Hz (the highest working refresh rate for the digital servos for what i know..)
        SERVO_CHANNEL+=SERVO_1K_US;
        if(state<4){
          state+=2;
        }else{
          state=0;
        }
      #endif
    #endif
    #if defined(SERVO_RFR_160HZ)
      #if defined(SERVO_4_HIGH)  // if there are 4 or more servos we dont need to slow it down
        SERVO_CHANNEL+=(SERVO_1K_US>>3); // 0 would be better but it causes bad jitter
        state=0; 
      #else // if there are less then 4 servos we need to slow it to not go over ~170Hz (the highest working refresh rate for analog servos)
        SERVO_CHANNEL+=SERVO_1K_US;
        if(state<8){
          state+=2;
        }else{
          state=0;
        }
      #endif
    #endif   
    #if defined(SERVO_RFR_50HZ) // to have ~ 50Hz for all servos
      SERVO_CHANNEL+=SERVO_1K_US;
      if(state<30){
        state+=2;
      }else{
        state=0;
      }     
    #endif
  }
} 
#endif

/**************************************************************************************/
/************             Motor software PWM generation              ******************/
/**************************************************************************************/
// SW PWM is only used if there are not enough HW PWM pins (for exampe hexa on a promini)

#if (NUMBER_MOTOR > 4) && (defined(PROMINI) || defined(PROMICRO))

/****************    Pre define the used ISR's and Timerchannels     ******************/
  #if !defined(PROMICRO)
    #define SOFT_PWM_ISR1 TIMER0_COMPB_vect
    #define SOFT_PWM_ISR2 TIMER0_COMPA_vect
    #define SOFT_PWM_CHANNEL1 OCR0B
    #define SOFT_PWM_CHANNEL2 OCR0A 
  #elif !defined(HWPWM6)
    #define SOFT_PWM_ISR1 TIMER3_COMPB_vect
    #define SOFT_PWM_ISR2 TIMER3_COMPC_vect
    #define SOFT_PWM_CHANNEL1 OCR3B
    #define SOFT_PWM_CHANNEL2 OCR3C 
  #else
    #define SOFT_PWM_ISR2 TIMER0_COMPB_vect  
    #define SOFT_PWM_CHANNEL2 OCR0B 
  #endif
  
/****************         Initialize Timers and PWM Channels         ******************/
  void initializeSoftPWM() {
    #if !defined(PROMICRO)
      TCCR0A = 0; // normal counting mode
      #if (NUMBER_MOTOR > 4) && !defined(HWPWM6) 
        TIMSK0 |= (1<<OCIE0B); // Enable CTC interrupt  
      #endif
      #if (NUMBER_MOTOR > 6) || ((NUMBER_MOTOR == 6) && !defined(SERVO))

        TIMSK0 |= (1<<OCIE0A);
      #endif
    #else
      #if !defined(HWPWM6)
        TCCR3A &= ~(1<<WGM30) & ~(1<<WGM31); //normal counting & no prescaler
        TCCR3B &= ~(1<<WGM32) & ~(1<<CS31) & ~(1<<CS32) & ~(1<<WGM33);
        TCCR3B |= (1<<CS30);   
        TIMSK3 |= (1<<OCIE3B); // Enable CTC interrupt  
        #if (NUMBER_MOTOR > 6) || ((NUMBER_MOTOR == 6) && !defined(SERVO))
          TIMSK3 |= (1<<OCIE3C);
        #endif   
      #else
        TCCR0A = 0; // normal counting mode
        TIMSK0 |= (1<<OCIE0B); // Enable CTC interrupt 
      #endif
    #endif
  }
  
/****************               Motor SW PWM ISR's                 ******************/
  // hexa with old but sometimes better SW PWM method
  // for setups without servos
  #if (NUMBER_MOTOR == 6) && (!defined(SERVO) && !defined(HWPWM6))
    ISR(SOFT_PWM_ISR1) { 
      static uint8_t state = 0;
      if(state == 0){
        SOFT_PWM_1_PIN_HIGH;
        SOFT_PWM_CHANNEL1 += atomicPWM_PIN5_highState;
        state = 1;
      }else if(state == 1){
        SOFT_PWM_CHANNEL1 += atomicPWM_PIN5_highState;
        state = 2;
      }else if(state == 2){
        SOFT_PWM_1_PIN_LOW;
        SOFT_PWM_CHANNEL1 += atomicPWM_PIN5_lowState;
        state = 3;  
      }else if(state == 3){
        SOFT_PWM_CHANNEL1 += atomicPWM_PIN5_lowState;
        state = 0;   
      }
    }
    ISR(SOFT_PWM_ISR2) { 
      static uint8_t state = 0;
      if(state == 0){
        SOFT_PWM_2_PIN_HIGH;
        SOFT_PWM_CHANNEL2 += atomicPWM_PIN6_highState;
        state = 1;
      }else if(state == 1){
        SOFT_PWM_CHANNEL2 += atomicPWM_PIN6_highState;
        state = 2;
      }else if(state == 2){
        SOFT_PWM_2_PIN_LOW;
        SOFT_PWM_CHANNEL2 += atomicPWM_PIN6_lowState;
        state = 3;  
      }else if(state == 3){
        SOFT_PWM_CHANNEL2 += atomicPWM_PIN6_lowState;
        state = 0;   
      }
    }
  #else
    #if (NUMBER_MOTOR > 4) && !defined(HWPWM6)
      // HEXA with just OCR0B 
      ISR(SOFT_PWM_ISR1) { 
        static uint8_t state = 0;
        if(state == 0){
          SOFT_PWM_1_PIN_HIGH;
          SOFT_PWM_CHANNEL1 += atomicPWM_PIN5_highState;
          state = 1;
        }else if(state == 1){
          SOFT_PWM_2_PIN_LOW;
          SOFT_PWM_CHANNEL1 += atomicPWM_PIN6_lowState;
          state = 2;
        }else if(state == 2){
          SOFT_PWM_2_PIN_HIGH;
          SOFT_PWM_CHANNEL1 += atomicPWM_PIN6_highState;
          state = 3;  
        }else if(state == 3){
          SOFT_PWM_1_PIN_LOW;
          SOFT_PWM_CHANNEL1 += atomicPWM_PIN5_lowState;
          state = 0;   
        }
      } 
    #endif
    //the same with digital PIN A2 & 12 OCR0A counter for OCTO
    #if (NUMBER_MOTOR > 6)
      ISR(SOFT_PWM_ISR2) {
        static uint8_t state = 0;
        if(state == 0){
          SOFT_PWM_3_PIN_HIGH;
          SOFT_PWM_CHANNEL2 += atomicPWM_PINA2_highState;
          state = 1;
        }else if(state == 1){
          SOFT_PWM_4_PIN_LOW;
          SOFT_PWM_CHANNEL2 += atomicPWM_PIN12_lowState;
          state = 2;
        }else if(state == 2){
          SOFT_PWM_4_PIN_HIGH;
          SOFT_PWM_CHANNEL2 += atomicPWM_PIN12_highState;
          state = 3;  
        }else if(state == 3){
          SOFT_PWM_3_PIN_LOW;
          SOFT_PWM_CHANNEL2 += atomicPWM_PINA2_lowState;
          state = 0;   
        }
      }
    #endif
  #endif
#endif


/**************************************************************************************/
/********** Mixes the Computed stabilize values to the Motors & Servos  ***************/
/**************************************************************************************/
void mixTable() {
  int16_t maxMotor;
  uint8_t i;

  #define PIDMIX(X,Y,Z) rcCommand[THROTTLE] + axisPID[ROLL]*X + axisPID[PITCH]*Y + YAW_DIRECTION * axisPID[YAW]*Z

  #if NUMBER_MOTOR > 3
    //prevent "yaw jump" during yaw correction
    axisPID[YAW] = constrain(axisPID[YAW],-100-abs(rcCommand[YAW]),+100+abs(rcCommand[YAW]));
  #endif
  /****************                   main Mix Table                ******************/
  #ifdef BI
    motor[0] = PIDMIX(+1, 0, 0); //LEFT
    motor[1] = PIDMIX(-1, 0, 0); //RIGHT        
    servo[4]  = constrain(1500 + YAW_DIRECTION * (axisPID[YAW] + axisPID[PITCH]), 1020, 2000); //LEFT
    servo[5]  = constrain(1500 + YAW_DIRECTION * (axisPID[YAW] - axisPID[PITCH]), 1020, 2000); //RIGHT
  #endif
  #ifdef TRI
    motor[0] = PIDMIX( 0,+4/3, 0); //REAR
    motor[1] = PIDMIX(-1,-2/3, 0); //RIGHT
    motor[2] = PIDMIX(+1,-2/3, 0); //LEFT
    servo[5] = constrain(conf.tri_yaw_middle + YAW_DIRECTION * axisPID[YAW], TRI_YAW_CONSTRAINT_MIN, TRI_YAW_CONSTRAINT_MAX); //REAR
  #endif
  #ifdef QUADP
    motor[0] = PIDMIX( 0,+1,-1); //REAR
    motor[1] = PIDMIX(-1, 0,+1); //RIGHT
    motor[2] = PIDMIX(+1, 0,+1); //LEFT
    motor[3] = PIDMIX( 0,-1,-1); //FRONT
  #endif
  #ifdef QUADX
    motor[0] = PIDMIX(-1,+1,-1); //REAR_R
    motor[1] = PIDMIX(-1,-1,+1); //FRONT_R
    motor[2] = PIDMIX(+1,+1,+1); //REAR_L
    motor[3] = PIDMIX(+1,-1,-1); //FRONT_L
  #endif
  #ifdef Y4
    motor[0] = PIDMIX(+0,+1,-1);   //REAR_1 CW
    motor[1] = PIDMIX(-1,-1, 0); //FRONT_R CCW
    motor[2] = PIDMIX(+0,+1,+1);   //REAR_2 CCW
    motor[3] = PIDMIX(+1,-1, 0); //FRONT_L CW
  #endif
  #ifdef Y6
    motor[0] = PIDMIX(+0,+4/3,+1); //REAR
    motor[1] = PIDMIX(-1,-2/3,-1); //RIGHT
    motor[2] = PIDMIX(+1,-2/3,-1); //LEFT
    motor[3] = PIDMIX(+0,+4/3,-1); //UNDER_REAR
    motor[4] = PIDMIX(-1,-2/3,+1); //UNDER_RIGHT
    motor[5] = PIDMIX(+1,-2/3,+1); //UNDER_LEFT    
  #endif
  #ifdef HEX6
    motor[0] = PIDMIX(-1/2,+1/2,+1); //REAR_R
    motor[1] = PIDMIX(-1/2,-1/2,-1); //FRONT_R
    motor[2] = PIDMIX(+1/2,+1/2,+1); //REAR_L
    motor[3] = PIDMIX(+1/2,-1/2,-1); //FRONT_L
    motor[4] = PIDMIX(+0  ,-1  ,+1); //FRONT
    motor[5] = PIDMIX(+0  ,+1  ,-1); //REAR
  #endif
  #ifdef HEX6X
    motor[0] = PIDMIX(-1/2,+1/2,+1); //REAR_R
    motor[1] = PIDMIX(-1/2,-1/2,+1); //FRONT_R
    motor[2] = PIDMIX(+1/2,+1/2,-1); //REAR_L
    motor[3] = PIDMIX(+1/2,-1/2,-1); //FRONT_L
    motor[4] = PIDMIX(-1  ,+0  ,-1); //RIGHT
    motor[5] = PIDMIX(+1  ,+0  ,+1); //LEFT
  #endif
  #ifdef OCTOX8
    motor[0] = PIDMIX(-1,+1,-1); //REAR_R
    motor[1] = PIDMIX(-1,-1,+1); //FRONT_R
    motor[2] = PIDMIX(+1,+1,+1); //REAR_L
    motor[3] = PIDMIX(+1,-1,-1); //FRONT_L
    motor[4] = PIDMIX(-1,+1,+1); //UNDER_REAR_R
    motor[5] = PIDMIX(-1,-1,-1); //UNDER_FRONT_R
    motor[6] = PIDMIX(+1,+1,-1); //UNDER_REAR_L
    motor[7] = PIDMIX(+1,-1,+1); //UNDER_FRONT_L
  #endif
  #ifdef OCTOFLATP
    motor[0] = PIDMIX(+7/10,-7/10,+1); //FRONT_L
    motor[1] = PIDMIX(-7/10,-7/10,+1); //FRONT_R
    motor[2] = PIDMIX(-7/10,+7/10,+1); //REAR_R
    motor[3] = PIDMIX(+7/10,+7/10,+1); //REAR_L
    motor[4] = PIDMIX(+0   ,-1   ,-1); //FRONT
    motor[5] = PIDMIX(-1   ,+0   ,-1); //RIGHT
    motor[6] = PIDMIX(+0   ,+1   ,-1); //REAR
    motor[7] = PIDMIX(+1   ,+0   ,-1); //LEFT 
  #endif
  #ifdef OCTOFLATX
    motor[0] = PIDMIX(+1  ,-1/2,+1); //MIDFRONT_L
    motor[1] = PIDMIX(-1/2,-1  ,+1); //FRONT_R
    motor[2] = PIDMIX(-1  ,+1/2,+1); //MIDREAR_R
    motor[3] = PIDMIX(+1/2,+1  ,+1); //REAR_L
    motor[4] = PIDMIX(+1/2,-1  ,-1); //FRONT_L
    motor[5] = PIDMIX(-1  ,-1/2,-1); //MIDFRONT_R
    motor[6] = PIDMIX(-1/2,+1  ,-1); //REAR_R
    motor[7] = PIDMIX(+1  ,+1/2,-1); //MIDREAR_L 
  #endif
  #ifdef VTAIL4
    motor[0] = PIDMIX(+0,+1, -1/2); //REAR_R 
    motor[1] = PIDMIX(-1, -1, +0);  //FRONT_R 
    motor[2] = PIDMIX(+0,+1, +1/2); //REAR_L 
    motor[3] = PIDMIX(+1, -1, -0);  //FRONT_L
  #endif

  /****************                Cam stabilize Sevos             ******************/
  #if defined(SERVO_TILT)
    #if defined(A0_A1_PIN_HEX) && (NUMBER_MOTOR == 6) && defined(PROMINI)
      #define S_PITCH servo[2]
      #define S_ROLL  servo[3]
    #else
      #define S_PITCH servo[0]
      #define S_ROLL  servo[1]
    #endif
    S_PITCH = TILT_PITCH_MIDDLE + rcData[AUX3]-1500;
    S_ROLL  = TILT_ROLL_MIDDLE  + rcData[AUX4]-1500;
    if (rcOptions[BOXCAMSTAB]) {
      S_PITCH += TILT_PITCH_PROP * angle[PITCH] /16 ;
      S_ROLL  += TILT_ROLL_PROP  * angle[ROLL]  /16 ;
    }
    S_PITCH = constrain(S_PITCH, TILT_PITCH_MIN, TILT_PITCH_MAX);
    S_ROLL  = constrain(S_ROLL , TILT_ROLL_MIN, TILT_ROLL_MAX  );   
  #endif
  
  
 /************************************************************************************************************/ 
 // Bledi Experimentals
 /************************************************************************************************************/ 
   #ifdef SERVO_MIX_TILT 
   // Simple CameraGimbal By Bledy http://youtu.be/zKGr6iR54vM
    if (rcOptions[BOXCAMSTAB]) {
      servo[0] = constrain(TILT_PITCH_MIDDLE - (-TILT_ROLL_PROP) * angle[PITCH] /16 - TILT_ROLL_PROP * angle[ROLL] /16 , TILT_PITCH_MIN, TILT_PITCH_MAX);
      servo[1] = constrain(TILT_ROLL_MIDDLE + (-TILT_ROLL_PROP) * angle[PITCH] /16 - TILT_ROLL_PROP * angle[ROLL] /16 , TILT_ROLL_MIN, TILT_ROLL_MAX);
    } else {
        // to use it with A0_A1_PIN_HEX
      #if defined(A0_A1_PIN_HEX) && (NUMBER_MOTOR == 6) && defined(PROMINI)
        servo[2] = constrain(TILT_PITCH_MIDDLE  + rcData[AUX3]-1500 , TILT_PITCH_MIN, TILT_PITCH_MAX);
        servo[3] = constrain(TILT_ROLL_MIDDLE   + rcData[AUX4]-1500,  TILT_ROLL_MIN, TILT_ROLL_MAX);     
      #else
        servo[0] = constrain(TILT_PITCH_MIDDLE  + rcData[AUX3]-1500 , TILT_PITCH_MIN, TILT_PITCH_MAX);
        servo[1] = constrain(TILT_ROLL_MIDDLE   + rcData[AUX4]-1500,  TILT_ROLL_MIN, TILT_ROLL_MAX);
      #endif
    }
  #endif  
 /************************************************************************************************************/ 
 // End of Bledi Experimentals
 /************************************************************************************************************/ 
  
  #ifdef GIMBAL
    servo[0] = constrain(TILT_PITCH_MIDDLE + TILT_PITCH_PROP * angle[PITCH] /16 + rcCommand[PITCH], TILT_PITCH_MIN, TILT_PITCH_MAX);
    servo[1] = constrain(TILT_ROLL_MIDDLE + TILT_ROLL_PROP   * angle[ROLL]  /16 + rcCommand[ROLL], TILT_ROLL_MIN, TILT_ROLL_MAX);
  #endif
  #if defined(FLYING_WING)
    motor[0] = rcCommand[THROTTLE];
    if (flag.PASSTHRU_MODE) {// do not use sensors for correction, simple 2 channel mixing
       servo[0]  = PITCH_DIRECTION_L * (rcData[PITCH]-MIDRC) + ROLL_DIRECTION_L * (rcData[ROLL]-MIDRC);
       servo[1]  = PITCH_DIRECTION_R * (rcData[PITCH]-MIDRC) + ROLL_DIRECTION_R * (rcData[ROLL]-MIDRC);
    } else { // use sensors to correct (gyro only or gyro+acc according to aux1/aux2 configuration
       servo[0]  = PITCH_DIRECTION_L * axisPID[PITCH]        + ROLL_DIRECTION_L * axisPID[ROLL];
       servo[1]  = PITCH_DIRECTION_R * axisPID[PITCH]        + ROLL_DIRECTION_R * axisPID[ROLL];
    }
    servo[0]  = constrain(servo[0] + conf.wing_left_mid , WING_LEFT_MIN,  WING_LEFT_MAX );
    servo[1]  = constrain(servo[1] + conf.wing_right_mid, WING_RIGHT_MIN, WING_RIGHT_MAX);
  #endif
  
  /************************************************************************************************************/
  #if defined(AIRPLANE)
    // Common parts for Plane and Heli
    static int16_t   servoMid[8];                        // Midpoint on servo
    static uint8_t   servoTravel[8] = SERVO_RATES;       // Rates in 0-100% 
    static int8_t    servoReverse[8] = SERVO_DIRECTION ; // Inverted servos
    static int16_t   servoLimit[8][2]; // Holds servoLimit data

    /***************************
     * servo endpoints Airplane. 
     ***************************/
  #define SERVO_MIN 1020           // limit servo travel range must be inside [1020;2000]
  #define SERVO_MAX 2000           // limit servo travel range must be inside [1020;2000]
    for(i=0; i<8; i++){  //  Set rates with 0 - 100%. 
      servoMid[i]     =MIDRC + conf.servoTrim[i];
      servoLimit[i][0]=servoMid[i]-((servoMid[i]-SERVO_MIN)   *(servoTravel[i]*0.01));
      servoLimit[i][1]=servoMid[i]+((SERVO_MAX - servoMid[i]) *(servoTravel[i]*0.01));  
    }

    // servo[7] is programmed with safty features to avoid motorstarts when ardu reset..  
    // All other servos go to center at reset..  Half throttle can be dangerus    
    // Only use servo[7] as motorcontrol if motor is used in the setup            */
    if (!flag.ARMED){ 
      servo[7] =  MINCOMMAND; // Kill throttle when disarmed
    } else {   
      servo[7] =  rcData[THROTTLE];
    }

    // Flapperon Controll
    int16_t flapperons[2]={0,0};    
    #if  defined(FLAPPERONS) && defined(FLAPPERON_EP)
      int8_t flapinv[2] = FLAPPERON_INVERT; 
      static int16_t F_Endpoint[2] = FLAPPERON_EP;
      int16_t flap =MIDRC-constrain(rcData[FLAPPERONS],F_Endpoint[1],F_Endpoint[0]);
      static int16_t slowFlaps= flap;
      #if defined(FLAPSPEED)
        if (slowFlaps < flap ){slowFlaps+=FLAPSPEED;}else if(slowFlaps > flap){slowFlaps-=FLAPSPEED;}
      #else
        slowFlaps = flap;
      #endif      
      flap = MIDRC-(constrain(MIDRC-slowFlaps,F_Endpoint[0],F_Endpoint[1]));      
    for(i=0; i<2; i++){flapperons[i] = flap * flapinv[i] ;}
    #endif
    
    // Traditional Flaps on A2
    #if defined(FLAPS)  && defined(FLAP_EP)
      static int16_t lF_Endpoint[2] = FLAP_EP;
      int16_t lFlap = MIDRC-constrain(rcData[FLAPS],lF_Endpoint[0],lF_Endpoint[1]);
      static int16_t slow_LFlaps= lFlap;
      #if defined(FLAPSPEED)
        if (slow_LFlaps < lFlap ){slow_LFlaps+=FLAPSPEED;}else if(slow_LFlaps > lFlap){slow_LFlaps-=FLAPSPEED;}
      #else
        slow_LFlaps = lFlap;
      #endif
      servo[2]    =  servoMid[2]+(slow_LFlaps *servoReverse[2]);
    #endif

    if(flag.PASSTHRU_MODE){   // Direct passthru from RX 
      servo[3]  = servoMid[3]+((rcCommand[ROLL] + flapperons[0]) *servoReverse[3]);     //   Wing 1
      servo[4]  = servoMid[4]+((rcCommand[ROLL] + flapperons[1]) *servoReverse[4]);     //   Wing 2
      servo[5]  = servoMid[5]+(rcCommand[YAW]                    *servoReverse[5]);     //   Rudder
      servo[6]  = servoMid[6]+(rcCommand[PITCH]                  *servoReverse[6]);     //   Elevator 
    }else{
      // Assisted modes (gyro only or gyro+acc according to AUX configuration in Gui
      servo[3]  =(servoMid[3] + ((axisPID[ROLL] + flapperons[0]) *servoReverse[3]));   //   Wing 1 
      servo[4]  =(servoMid[4] + ((axisPID[ROLL] + flapperons[1]) *servoReverse[4]));   //   Wing 2
      servo[5]  =(servoMid[5] + (axisPID[YAW]                    *servoReverse[5]));   //   Rudder
      servo[6]  =(servoMid[6] + (axisPID[PITCH]                  *servoReverse[6]));   //   Elevator
    } 
    // ServoRates
    for(i=3;i<8;i++){
      servo[i]  = map(servo[i], SERVO_MIN, SERVO_MAX,servoLimit[i][0],servoLimit[i][1]);
      servo[i]  = constrain( servo[i], SERVO_MIN, SERVO_MAX);
    }

  #endif

  /************************************************************************************************************/
  #ifdef HELICOPTER 
  // Common controlls for Helicopters 
    int16_t heliRoll,heliNick;
    int16_t collRange[3] = COLLECTIVE_RANGE;
    static int16_t   collective;	
    static int16_t   servoEndpiont[8][2];
    static int16_t   servoHigh[8] = SERVO_ENDPOINT_HIGH; // HIGHpoint on servo
    static int16_t   servoLow[8]  = SERVO_ENDPOINT_LOW ; // LOWpoint on servo

  /***************************
   * servo settings Heli. 
   ***************************/
    for(i=0; i<8; i++){  //  Set rates using endpoints. 
      servoEndpiont[i][0] = servoLow[i];  //Min
      servoEndpiont[i][1] = servoHigh[i]; //Max   
    }

  // Limit Collective range up/down    
    int16_t collect = rcData[COLLECTIVE_PITCH]-collRange[1];
    if   (collect>0) { 
      collective = collect * (collRange[2]*0.01); 
    } else{
      collective = collect * (collRange[0]*0.01); 
    } 

    if(flag.PASSTHRU_MODE){ // Use Rcdata Without sensors
      heliRoll=  rcCommand[ROLL] ;
      heliNick=  rcCommand[PITCH];
    } else{ // Assisted modes
      heliRoll= axisPID[ROLL];
      heliNick= axisPID[PITCH];
    }  

  // Limit Maximum Rates for Heli
    int16_t cRange[2] = CONTROLL_RANGE;
    heliRoll*=cRange[0]*0.01;
    heliNick*=cRange[1]*0.01;
	
  #define HeliXPIDMIX(Z,Y,X) collRange[1]+collective*Z + heliNick*Y +  heliRoll*X

  // Yaw is common for Heli 90 & 120
    uint16_t yawControll =  YAW_CENTER + (axisPID[YAW]*YAW_DIRECTION) + conf.servoTrim[5];

  /* Throttle & YAW
  ********************
  Handeled in common functions for Heli */
    if (!flag.ARMED){ 
      servo[7] = 900; // Kill throttle when disarmed
      if (YAWMOTOR){servo[5] =  MINCOMMAND;} else {servo[5] =  yawControll; } // Kill YAWMOTOR when disarmed
    }else {   
      servo[7]  = rcData[THROTTLE]; //   50hz ESC or servo
      if (YAWMOTOR && rcData[THROTTLE] < MINTHROTTLE){servo[5] =  MINCOMMAND;}
      else{ servo[5] =  yawControll; }     // YawSero	 
    }


  //              ( Collective, Pitch/Nick, Roll ) Change sign to invert
  /************************************************************************************************************/
  #ifdef HELI_120_CCPM 
    static int8_t nickMix[3] =SERVO_NICK;
    static int8_t leftMix[3] =SERVO_LEFT;
    static int8_t rightMix[3]=SERVO_RIGHT;

    servo[3]  =  HeliXPIDMIX( (nickMix[0]*0.1) , nickMix[1]*0.1, nickMix[2]*0.1) +conf.servoTrim[3] ;   //    NICK  servo
    servo[4]  =  HeliXPIDMIX( (leftMix[0]*0.1) , leftMix[1]*0.1, leftMix[2]*0.1) +conf.servoTrim[4] ;   //    LEFT servo
    servo[6]  =  HeliXPIDMIX( (rightMix[0]*0.1),rightMix[1]*0.1,rightMix[2]*0.1) +conf.servoTrim[6] ;   //    RIGHT  servo

  #endif

  /************************************************************************************************************/
  #ifdef HELI_90_DEG   
    static int8_t servoDir[3]=SERVO_DIRECTIONS;   
    servo[3]  = HeliXPIDMIX( +0, servoDir[1], -0)+conf.servoTrim[3] ;      //     NICK  servo
    servo[4]  = HeliXPIDMIX( +0, +0, servoDir[2])+conf.servoTrim[4] ;      //     ROLL servo
    servo[6]  = HeliXPIDMIX( servoDir[0], +0, +0)+conf.servoTrim[6] ;      //     COLLECTIVE  servo
  #endif    

    for(i=3;i<8;i++){
      servo[i]  = constrain( servo[i], servoEndpiont[i][0], servoEndpiont[i][1] ); 
    }

  #endif
  
// End of PatrikE Experimentals
/************************************************************************************************************/
 
  /****************                    Cam trigger Sevo                ******************/
  #if defined(CAMTRIG)
    static uint8_t camCycle = 0;
    static uint8_t camState = 0;
    static uint32_t camTime = 0;
    if (camCycle==1) {
      if (camState == 0) {
        servo[2] = CAM_SERVO_HIGH;
        camState = 1;
        camTime = millis();
      } else if (camState == 1) {
       if ( (millis() - camTime) > CAM_TIME_HIGH ) {
         servo[2] = CAM_SERVO_LOW;
         camState = 2;
         camTime = millis();
       }
      } else { //camState ==2
       if ( (millis() - camTime) > CAM_TIME_LOW ) {
         camState = 0;
         camCycle = 0;
       }
      }
    }
    if (rcOptions[BOXCAMTRIG]) camCycle=1;
  #endif
  
  /****************                Filter the Motors values                ******************/
  maxMotor=motor[0];
  for(i=1;i< NUMBER_MOTOR;i++)
    if (motor[i]>maxMotor) maxMotor=motor[i];
  for (i = 0; i < NUMBER_MOTOR; i++) {
    if (maxMotor > MAXTHROTTLE) // this is a way to still have good gyro corrections if at least one motor reaches its max.
      motor[i] -= maxMotor - MAXTHROTTLE;
    motor[i] = constrain(motor[i], MINTHROTTLE, MAXTHROTTLE);    
    if ((rcData[THROTTLE]) < MINCHECK)
      #ifndef MOTOR_STOP
        motor[i] = MINTHROTTLE;
      #else
        motor[i] = MINCOMMAND;
      #endif
    if (!flag.ARMED)
      motor[i] = MINCOMMAND;
  }
  /****************                      Powermeter Log                    ******************/
  #if (LOG_VALUES == 2) || defined(POWERMETER_SOFT)
    uint32_t amp;
    /* true cubic function; when divided by vbat_max=126 (12.6V) for 3 cell battery this gives maximum value of ~ 500 */

    static uint16_t amperes[64] =   {   0,  2,  6, 15, 30, 52, 82,123,
                                     175,240,320,415,528,659,811,984,
                                     1181,1402,1648,1923,2226,2559,2924,3322,
                                     3755,4224,4730,5276,5861,6489,7160,7875,
                                     8637 ,9446 ,10304,11213,12173,13187,14256,15381,
                                     16564,17805,19108,20472,21900,23392,24951,26578,
                                     28274,30041,31879,33792,35779,37843,39984,42205,
                                     44507,46890,49358,51910,54549,57276,60093,63000};
  
    if (vbat) { // by all means - must avoid division by zero 
      for (i =0;i<NUMBER_MOTOR;i++) {
        amp = amperes[ ((motor[i] - 1000)>>4) ] / vbat; // range mapped from [1000:2000] => [0:1000]; then break that up into 64 ranges; lookup amp
  	  #if (LOG_VALUES == 2)
           pMeter[i]+= amp; // sum up over time the mapped ESC input 
        #endif
        #if defined(POWERMETER_SOFT)
           pMeter[PMOTOR_SUM]+= amp; // total sum over all motors
        #endif
      }
    }
  #endif
}
