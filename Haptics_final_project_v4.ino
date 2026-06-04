int tMin = -100;
int tMax =  100;
float tCenter = 0;
int currentMode = 0;  // 0=visual, 1=vibrotactile, 2=kinesthetic
int loopCount = 0;
int loopCount2 = 0;
float tableAngle = 0;
int tableAngleTicks = 0;

int vibroPin = 4;
float vibroPWM = 0.0;
float vRaw = 0.0;
float dx = 0.0;
float dt = 0.0;
int dxTicks = 0;

volatile long encoderPosition = 0;
volatile int lastEncoded = 0;

const int encoderPinA = 2;
const int encoderPinB = 3;
const byte ledPin = 13;

int torque = 0;
int oldTorque = 0;

int seeking_torque_quantized = 80; // originally 50, changed to 80 per Lab 4 Task 1 instructions (JK 5/19)
bool direction = 1;   // 1 = clockwise, 0 = counterclockwise

float read_encoderPosition = 0;

// full quadrature encoder logic
void IRAM_ATTR updateEncoder() {
  int MSB = digitalRead(encoderPinA);
  int LSB = digitalRead(encoderPinB);

  int encoded = (MSB << 1) | LSB;
  int before_after = (lastEncoded << 2) | encoded;

  // compare before and after digital read and increment depending on direction of change
  // more efficient to look up values than do a bunch of conditionals
  if (before_after == 0b1101 || before_after == 0b0100 || before_after == 0b0010 || before_after == 0b1011) encoderPosition++;
  if (before_after == 0b1110 || before_after == 0b0111 || before_after == 0b0001 || before_after == 0b1000) encoderPosition--;

  lastEncoded = encoded;
}

// detent specifications and support functions
const int detentSpacing = 150;
const int detentRange = 369;
const float detentWidth = 20;
float detentK = 2.5;

int detentPositions[2 * (detentRange / detentSpacing) + 1];
int numDetents = 0;

int nearestDetent(int currentPosition) {
    int nearest = detentPositions[0];
    int minDist = INT_MAX;
    for (int i = 0; i < numDetents; i++) {
        int dist = abs(currentPosition - detentPositions[i]);
        if (dist < minDist) {
            minDist = dist;
            nearest = detentPositions[i];
        }
    }
    return nearest;
}

bool isNearDetent(int currentPosition) {
    return abs(currentPosition - nearestDetent(currentPosition)) <= detentWidth;
}


//----------------------------------------------------------------------------
void setup() {
  // initialize detent locations

  for (int pos = 0; pos <= detentRange; pos += detentSpacing) {
      detentPositions[numDetents++] = pos;
      if (pos != 0) {
          detentPositions[numDetents++] = -pos;
      }
  }


  Serial.begin(115200);
  // 2nd serial monotor for R3 connection
  Serial1.begin(115200, SERIAL_8N1, D6, D7); 
  Serial1.setTimeout(0);

  // define arduino input/output pin modes
  pinMode(encoderPinA, INPUT_PULLUP);
  pinMode(encoderPinB, INPUT_PULLUP);
  pinMode(ledPin, OUTPUT);

  pinMode(9, OUTPUT);
  ledcAttachPin(9, 0);  // super-annoying, analogWrite doesn't work correctly in setup(), had to switch to the ESP32 native version
  ledcSetup(0, 5000, 8);
  pinMode(10, OUTPUT);
  pinMode(11, OUTPUT);  
  
  // final prodject button and potentiometer pins
  pinMode(A7, INPUT); // Set analog pin A7 as input
  pinMode(D5, INPUT);   // Pin D6 as output
  pinMode(LED_BUILTIN, OUTPUT); 

  attachInterrupt(digitalPinToInterrupt(encoderPinA), updateEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderPinB), updateEncoder, CHANGE);
  // Serial.println("Encoder Initializing:");

  digitalWrite(ledPin, LOW);  // LED Off
  digitalWrite(10, LOW);
  digitalWrite(11, LOW);
  ledcWrite(0, 0);
  // Serial.println("Wait before initialization, 10s");
  delay(1000); // originally 1000, changed to 5000 per Lab 4 Task 1 instructions (JK 5/19)

  
  // lab 3 paddle initialization code
  bool found_end = false;
  bool started_moving = false;

  float found_end_delay = 50; // originally 500
  float old_position = encoderPosition;
  // Serial.println("Initializing, finding negative end...");

  // start motor moving slowly
  digitalWrite(10, LOW);
  digitalWrite(11, HIGH);   // want to go negative direction
  ledcWrite(0, constrain(seeking_torque_quantized, 0, 255));

  while (!started_moving){
    ledcWrite(0, constrain(seeking_torque_quantized, 0, 255));
    if (encoderPosition != old_position) {
      started_moving = true;
    } 
      // Serial.print("waiting for slewing to begin, encoder at: ");Serial.println(encoderPosition);

      // Serial.print(", ");Serial.print("0"); // Jamie's prints to log values before paddle moves
      // Serial.print(", ");Serial.print(encoderPosition);
      // Serial.print(", ");Serial.println("0");
      delay(10);
  }  // simple loop to not run the loop below till the paddle starts moving.

  float last_move_time = millis();
  while (!found_end){
    ledcWrite(0, constrain(seeking_torque_quantized, 0, 255));
    if (encoderPosition != old_position) {
      dxTicks = encoderPosition - old_position;
      old_position = encoderPosition;
      dt = millis()-last_move_time;
      last_move_time = millis();
      dx = 0.1983*dxTicks;  // in mm
      vRaw = dx / dt;          // in mm/s

    } else if ((millis() - last_move_time) > found_end_delay) {
      found_end = true;
      // Serial.println("end found");
    }
    // Serial.print("looking for negative end...encoder at: ");Serial.println(encoderPosition);
          // write to serial monitor (use faster baud rates! e.g. 115200)

    // Serial.print(millis());    // Jamie's prints for logging values during homing
    // Serial.print(", ");Serial.print(seeking_torque_quantized);
    // Serial.print(", ");Serial.print(encoderPosition);
    // Serial.print(", ");Serial.println(vRaw);
    
  }


  int W = 739;               // measured full paddle travel range in ticks
  encoderPosition = -W/2;    // truncate if W odd, probably okay
  ledcWrite(0, 0);   // STOP
  digitalWrite(10, LOW);
  digitalWrite(11, LOW);   // reset dir

  // Serial.print("end found, setting to ");Serial.println(-W/2);
  delay(250);

  // back off from end for confirmation and to start user on-center
  int start_offset = W/2;
  digitalWrite(10, HIGH);
  digitalWrite(11, LOW);
  while (encoderPosition < (-W/2 + start_offset)){
    ledcWrite(0, seeking_torque_quantized);
    //Serial.print("backing off...encoder at: ");Serial.print((encoderPosition < (-W/2 + start_offset)));Serial.println(encoderPosition);
    }

  ledcWrite(0, 0);   // STOP
  digitalWrite(10, LOW);
  digitalWrite(11, LOW);   // reset dir

  // Serial.println("Move haptic paddle to center to begin");
  // digitalWrite(ledPin, HIGH);  // turn on LED and wait
  // int centerWidth = 10;
  // while (abs(encoderPosition) > centerWidth ){
  //   delay(1);
  //   }
  // // digitalWrite(ledPin, LOW);  // turn on LED and wait

}
//----------------------------------------------------------------------------


// define additional paramters

float K = 0.0;   // initialize K
float K_Strenght = 1.0;  // value to default back to
float K_teleop = 2.0;    // 4 works, but a bit stiff

float vibroGain = 0.8;   //0.05
float vibroMag = 0;
int springLoc = 0;  // center point_of spring

float ticksToDegrees = 90.0/739.0;

int  mu = 19; //18; // in units of digitized 8-bit torque
float B = 0.02;
float alpha = 0.5; // Exponential Moving Average Weight

float last_time = micros();
int last_encoderPosition = encoderPosition;
float compTorque = 0;
float vLast = 0;

float loopFreq = 100;
float loopDuration = 1/loopFreq;  // in seconds
int lastLoopExit = micros();

bool lastBotton1State = 0;

//----------------------------------------------------------------------------
void loop() {
  // stalling loop to set the loop frequency  
  while ( (micros() - lastLoopExit)/1000000.0 < loopDuration);
  float loopLengthms = (micros() - lastLoopExit)/1000;
  lastLoopExit = micros();
  // end stalling loop

  // Operation Mode toggle logic, and LED indication
  int button1 = digitalRead(D5); 
  if (button1 == HIGH) {         

    if (lastBotton1State != HIGH){
      if (currentMode == 1) { 
        currentMode = 0;
        digitalWrite(LED_BUILTIN, LOW);}
      else {
        currentMode = 1;
        digitalWrite(LED_BUILTIN, HIGH);
        }
    }
  } 
  lastBotton1State = button1;


  read_encoderPosition = encoderPosition;     // single postion read per loop

  if (currentMode == 0){
    // configure detent location
    springLoc = nearestDetent(read_encoderPosition);
    int detentDist = (abs(read_encoderPosition - springLoc));

    // configure spring strengths
    if (isNearDetent(read_encoderPosition)){
      if ( detentDist < (detentWidth/2.0) ) {
        K = 2.0* detentK;         //  try double spring strength in the inner half of the detent
      }
      else {
        K = detentK;
      }
    } else {
      K = 0;
    }

    // configure vibration
    int tableDist = abs(tableAngleTicks-read_encoderPosition);
    // if ( detentDist > (detentWidth/2.0) ){
    if ( tableDist > detentWidth/2.0 ) {
      // vibroMag = 1.0*detentDist;  // TODO, change to be from confirmation of location
        vibroMag = 1.0*(tableDist);
      } else {
        vibroMag = 0;
      }


  } else if (currentMode == 1){
    K = K_teleop;
    // tCenter set by serial read code
    vibroMag = 0;
    springLoc = 1.0*tableAngleTicks;

  } else {  // failure state only
    vibroMag = 0;
    K = 0;
  }

  // Virtual Spring
  float spring_torque = -(read_encoderPosition-springLoc)*K;

  // measure velocity
  int new_time = micros();
  dt = (new_time - last_time)/1000000.0;
  last_time = new_time;

  dxTicks = read_encoderPosition - last_encoderPosition;
  dx = 0.1983*dxTicks;  // in mm
  vRaw = dx / dt;          // in mm/s
  last_encoderPosition = read_encoderPosition;

  // Exponential SMOOTHING ON VELOCITY
  float v = vRaw*alpha + (1-alpha)*vLast;
  vLast = v;

  // Friction Compensation
  if (v == 0){
    compTorque = 0;
  } 
  else {
    compTorque = mu * v/abs(v);
  }

  // Virtual Damping
  float dampTorque = -B*v;

  // sum up torque components
  float netTorque = spring_torque + compTorque + dampTorque;

  // logic to get motor direction from torque sign
  if (netTorque < 0){
    direction = -1;
    digitalWrite(10, LOW);
    digitalWrite(11, HIGH);   // reset dir
    }
  else if (netTorque > 0) {
    direction = 1;
    digitalWrite(10, HIGH);
    digitalWrite(11, LOW);   // reset dir
    }
  else {
    direction = 0;
    digitalWrite(10, LOW);
    digitalWrite(11, LOW);   // reset dir
    }

  // output to motor
  float torqueMag = abs(netTorque);
  ledcWrite(0, constrain(torqueMag, 0, 255));
  vibroPWM = vibroMag*vibroGain; // scale the calculated torque value by the appropriate gain for the vibrotactile actuator
  analogWrite(vibroPin, vibroPWM); // write calculated value as a PWM to the vibrotactile actuator

  // inside loop(), after your haptic math
  loopCount++;
  if (loopCount >= loopFreq/2) {  // 1000Hz / 20 = 50Hz
    Serial.println(read_encoderPosition);
    loopCount = 0;
  }


    // Serial conneciton read block
  if (Serial1.available() > 0) {
    tableAngle = Serial1.parseFloat(); 
    tableAngleTicks = int(tableAngle/ticksToDegrees);
    
    // Clean out any trailing newline characters instantly without blocking
    while(Serial1.available() > 0 && (Serial1.peek() == '\n' || Serial1.peek() == '\r')) {
      Serial1.read();
    }
  }

  //  Serial conneciton write block
  loopCount2++;
  if (loopCount2 >= 5) { // send every 5th cycle to avoid overwhelming the interface TODO, improve to send changes sooner
    float angleSetpoint = read_encoderPosition * ticksToDegrees;
    Serial1.println(angleSetpoint);

    // also print locally when sending to monitor
    Serial.print(tableAngle);Serial.print("|");Serial.println(angleSetpoint); // Optional: comment out if your USB serial monitor gets flooded

    loopCount2 = 0; // Reset your counter
  }

}

