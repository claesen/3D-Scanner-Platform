/* Controller code for LiDAR 3D-Scanner platform. 
 * Part of thesis at Chalmers University of Technology
 * @Author Rasmus Claesén
 */
 
/* Lidar Lite v3 by Garmin & Adafruit
     Motorshield v2.3 packages */
#include <LIDARLite.h>
#include <Adafruit_MotorShield.h>
#include <Servo.h>

/* Grab hardware references */
Adafruit_MotorShield MS;
Servo servo;    
Adafruit_DCMotor *DCMotor;
LIDARLite LIDAR;

<<<<<<< HEAD
/* Survey settings */
int c_cycle = 0;
int t_cycles = 3;

/* Speed settings */
enum Speed {low_speed = 120, medium_speed = 200, high_speed = 255};
enum Speed speed = medium_speed;

/* State keeping */
=======
// State keeper
>>>>>>> parent of 63559ba... Arduino Controller update
enum State {STATE_STOPPED, STATE_RUNNING};
enum State state = STATE_STOPPED;

/* The three measurements for
     spherical reperesentation */
float phi;
float theta;
int rho;

int n_laps = 0;
int total_laps_to_record = 2;

/* Pitch field of view and
     angular resultion */
int MIN_PITCH = 42;
int MAX_PITCH = 123;
int INC_PITCH = 3;

/* Motor position & revulation trackers
     phi == pitch
     theta == yaw */    
float pitchAngle = MIN_PITCH;
float yawAngle = 0;
bool MEASURE = false;
bool newRevulation = false;
int needForSpeed = 160;

/* Pins, flags and counter for hardware interrupt encoder counter
     Routine from https://www.instructables.com/id/Improved-Arduino-Rotary-Encoder-Reading/ */
static int pinA = 2;
static int pinB = 3;
volatile int aFlag = 0;
volatile byte bFlag = 0; 
volatile byte encoderCount = 0;
volatile byte oldEncoderCount = 0;
volatile byte reading = 0;

/* Serial IDs */
String newLap = "n";
String done = "d";

/* Calculate angle per count of the encoder
     (0.84 degrees per count) */
float dcGearRatio = 42.875;
float gearRatio = 15.0/50.0;
float DPC = ( 360.0 / ( dcGearRatio * 3.0 ) ) * gearRatio;

/* RPM Measurements (Part of experimental function) */
unsigned long prev_rpm_time = 0;
int rpm = 0;
int TARGET_RPM = 22;
int TARGET_RPM_COUNTER = 0;
int MSPEED = 120;
int MSPEED_FINAL = 0;
int RPM_LIST[ 10 ];

// Function prottypes
void PinA();
void PinB();
float deg2rad();
void angularStuff();
void stop_f();
void start_f();
void start_interrupts();
void stop_interrupts();

void setup() {
<<<<<<< HEAD
    
=======
>>>>>>> parent of 63559ba... Arduino Controller update
    Serial.begin(115200);
    
    /*    Configuration of pins and interrupts for encoder routine */
    pinMode( pinA, INPUT_PULLUP ); 
    pinMode( pinB, INPUT_PULLUP );

    /*    Configure motor attachments */
    MS = Adafruit_MotorShield( );
    servo.attach( 10 );
    DCMotor = MS.getMotor( 4 );

    /*    Start and configure LiDAR
    0 = default balanced
    See data sheet for more configs */
    LIDAR.begin( 0, true );
    LIDAR.configure( 0 );

    /*    Start the motorshield */
    MS.begin( );
}

void start_interrupts() {
    
    attachInterrupt( digitalPinToInterrupt(pinA), PinA, RISING );
    attachInterrupt( digitalPinToInterrupt(pinB), PinB, RISING );
}

void stop_interrupts() {
    
    detachInterrupt( digitalPinToInterrupt(pinA) );
    detachInterrupt( digitalPinToInterrupt(pinB) );
}

<<<<<<< HEAD
void start_f() {
    
=======
void InitSpeed() {
    while(TARGET_RPM_COUNTER < 10) {
        if ( ( rpm < TARGET_RPM ) && ( MSPEED < 255 ) ) {
            MSPEED++; 
        }
        else if ( rpm > TARGET_RPM ) {
            MSPEED--; 
        }
        DCMotor->setSpeed( MSPEED );
        delay( 100 );
        
        /* Reglersystem? */
        if ( ( rpm >= TARGET_RPM - 1 ) && ( rpm <= TARGET_RPM + 1 ) ) {
            RPM_LIST[TARGET_RPM_COUNTER] = MSPEED;
            TARGET_RPM_COUNTER++; 
        } 
    }
    
    for (int i=0; i < 10; i++) {
        MSPEED_FINAL += RPM_LIST[i]; 
    }
    MSPEED_FINAL = (int) ( MSPEED_FINAL / 10 );
    DCMotor->setSpeed(MSPEED_FINAL); 
}

void start_f(){
>>>>>>> parent of 63559ba... Arduino Controller update
    String num;
    char c;

    c_cycle = 0;
    delay(20);
<<<<<<< HEAD
    
    while ((c = Serial.read()) != -1) {     
=======
    while ((c = Serial.read()) != -1) {
>>>>>>> parent of 63559ba... Arduino Controller update
        num += String(c);
   }

<<<<<<< HEAD
    t_cycles = num.toInt();

    if (state != STATE_RUNNING) {
        DCMotor->setSpeed( 0 ); // MIN 100. MAX 255
        DCMotor->run( FORWARD );  // turn it on going forward
        
        for (int i = 0; i <= 10; i++) {
            DCMotor->setSpeed( 100 + (i * (speed - 100)) / 10 );
=======
    ret = num.toInt();
    total_laps_to_record = ret;
    if(ret != 0){
        n_laps = ret;
    }

    if (state != STATE_RUNNING){
        DCMotor->setSpeed( 0 ); // MIN 100. MAX 255
        DCMotor->run( FORWARD );  // turn it on going forward
        for (int i = 0; i <= 10; i++) {
            DCMotor->setSpeed( 100+(i*(255-100))/10 ); // MIN 100. MAX 255
>>>>>>> parent of 63559ba... Arduino Controller update
            delay(100);
        }
        DCMotor->setSpeed( 255 ); // MIN 100. MAX 255
    }
    DCMotor->setSpeed(needForSpeed);

    state = STATE_RUNNING;
    
    aFlag = 0;
    bFlag = 0; 
    encoderCount = 0;
    oldEncoderCount = 0;
    reading = 0;

    prev_rpm_time = 0;
    rpm = 0;
    TARGET_RPM = 22;
    TARGET_RPM_COUNTER = 0;
    MSPEED = 120;
    MSPEED_FINAL = 0;
    TARGET_RPM_COUNTER = 0;
    memset(RPM_LIST, 0, sizeof(RPM_LIST));

 //   InitSpeed(); ( Experimental )
       
    pitchAngle = MIN_PITCH;
    servo.write( pitchAngle );
    delay( 200 ); 
    start_interrupts();
}

<<<<<<< HEAD
void stop_f() {
    
    stop_interrupts();
    state = STATE_STOPPED;
    
    for (int i = 10; i >= 0; i--) {
        DCMotor->setSpeed( 100+(i*(speed-100))/10 );
=======
void stop_f(){
    stop_interrupts();
    state = STATE_STOPPED;
    for (int i = 10; i >= 0; i--) {
        DCMotor->setSpeed( 100+(i*(255-100))/10 );
>>>>>>> parent of 63559ba... Arduino Controller update
        delay(100);
    }
    DCMotor->setSpeed( 0 );
    DCMotor->run( RELEASE );  // turn it on going forward

    servo.write( MIN_PITCH );
    delay( 200 ); 
}

<<<<<<< HEAD
float deg2rad(float in_degree) {

    return in_degree * PI / 180;
}

void PinA() {
    
=======
void PinA(){
>>>>>>> parent of 63559ba... Arduino Controller update
    cli(); //stop interrupts happening before we read pin values
    reading = PIND & 0xC; // read all eight pin values then strip away all but pinA and pinB's values
    
    if(reading == B00001100 && aFlag) { //check that we have both pins at detent (HIGH) and that we are expecting detent on this pin's rising edge
        encoderCount --; //decrement the encoder's position count
        angularStuff();
        bFlag = 0; //reset flags for the next turn
        aFlag = 0; //reset flags for the next turn
    } 
        
    else if (reading == B00000100) bFlag = 1; //signal that we're expecting pinB to signal the transition to detent from free rotation
    sei(); //restart interrupts 
}

<<<<<<< HEAD
void PinB() {
    
=======
void PinB(){
>>>>>>> parent of 63559ba... Arduino Controller update
    cli(); //stop interrupts happening before we read pin values
    reading = PIND & 0xC; //read all eight pin values then strip away all but pinA and pinB's values
    
    if (reading == B00001100 && bFlag) { //check that we have both pins at detent (HIGH) and that we are expecting detent on this pin's rising edge
        encoderCount ++; //increment the encoder's position count
        bFlag = 0; //reset flags for the next turn
        aFlag = 0; //reset flags for the next turn 
    }
        
    else if (reading == B00001000) aFlag = 1; //signal that we're expecting pinA to signal the transition to detent from free rotation
    sei(); //restart interrupts 
}

void rpmCount() {
<<<<<<< HEAD
    
=======
>>>>>>> parent of 63559ba... Arduino Controller update
    rpm = 60000 / ( millis( ) - prev_rpm_time ) ;
    prev_rpm_time = millis( );
}


/*    Routine to update angular stuff */
<<<<<<< HEAD
void angularStuff() {
         
=======
void angularStuff() {     
>>>>>>> parent of 63559ba... Arduino Controller update
    if ( yawAngle + DPC > 360.0 ) {
        newRevulation = true; 
    }
     
    yawAngle += DPC; 
<<<<<<< HEAD
    MEASURE = true;
=======
>>>>>>> parent of 63559ba... Arduino Controller update
}


void RevulationUpdate() {
<<<<<<< HEAD
    
=======
    rpmCount( );
>>>>>>> parent of 63559ba... Arduino Controller update
    yawAngle = yawAngle - 360.0;
    pitchAngle = pitchAngle + INC_PITCH;

    if ( pitchAngle + INC_PITCH > MAX_PITCH ) {
<<<<<<< HEAD
        c_cycle++;
        pitchAngle = MIN_PITCH + (c_cycle % t_cycles);
 
        if (c_cycle == t_cycles) {
            Serial.println( done );
            stop_f();
            
        } else {
            Serial.println( newLap );
        } delay( 10 ); 
    }
            
    servo.write( pitchAngle ); 
    rpmCount( );
    delay( 20 ); 
}

void measure() {
    
    // Magic + 9 degrees angle addition
    phi = deg2rad( (pitchAngle + 9) );
    theta = deg2rad( yawAngle );
    rho = LIDAR.distance( );
    
    Serial.println( String(phi) + " " + String(theta) + " " + String(rho) );
    delay(10);
}

void getStatus() {
    
=======
        pitchAngle = MIN_PITCH + round(3*(total_laps_to_record-n_laps)/total_laps_to_record+0.1);
        
        n_laps--;
        if (n_laps == 0){
            Serial.println( done );
            stop_f();
        } else {
            Serial.println( newLap );
        }
        delay( 20 ); 
    }
            
    servo.write( pitchAngle ); 
    delay( 50 ); 
}

void measure() {
    phi = (float) ( pitchAngle + 9) * PI/180;
    theta = (float) yawAngle * PI/180;
    r = LIDAR.distance( );

    delay(10); 
}

void setSpeed(){
>>>>>>> parent of 63559ba... Arduino Controller update
    Serial.println( "Not implemented" );
    return;
}

<<<<<<< HEAD
void setSpeed() {
    Serial.println( "Not implemented" );
    return;
}

void loop() {
    
    if ( state != STATE_STOPPED ) {
        if (MEASURE) {
            measure();  
            MEASURE = false;
        } 
        
=======
void setResulution(){
    Serial.println( "Not implemented" );
        
    return;
}

void getStatus(){
    Serial.println( "Not implemented" );
        
    return;
}

void loop() {
    switch(Serial.read()){
        case -1:    // No data
        case '\0':  // escape characters
        case '\n':
        case '\r':
            break;
        case 'r':
            start_f();
            break;
        case 's':
            stop_f();
            break;
        case 'v':
            setSpeed();
            break;
        case 'u':
            setResulution();
            break;
        case 'g':
            getStatus();
            break;
        default:
            Serial.println( "Unknown command" );
            delay( 20 ); 
            break;
    }

    if (state != STATE_STOPPED) {
        Serial.println( String(phi) + " " + String(theta) + " " + String(r) );
        measure( );
>>>>>>> parent of 63559ba... Arduino Controller update
        if ( newRevulation ) {
            RevulationUpdate( );
            newRevulation = false; 
        }
    }
    
    switch(Serial.read()) {
      case -1:    // No data
      case '\0':  // escape characters
      case '\n':
      case '\r':
          break;
      case 'r':
          start_f();
          break;
      case 's':
          stop_f();
          break;
      case 'g':
          getStatus();
          break;
      default:
          Serial.println( "Unknown command" );
          delay( 20 ); 
          break;
    }
}
