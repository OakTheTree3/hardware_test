const int ENC_A = 3;
const int ENC_B = 4;
const int PWM = 9;
const int DIR = 10;

char command;
char cha;
char argv[16];
int arg = 0;
int index = 0;
int toggler = 0;
volatile bool pidStatus = false;
volatile bool turn_on = false;

const float COUNT_PER_REV = 20.0;
volatile long totalPulse = 0;
volatile long prevTime = 0;
volatile double motorSpeed = 0; //in RPM

volatile double targetSpeed;
const double K_P = 0.1;
volatile double speedInput = 0;


void resetCommand() {
  memset(argv, 0, sizeof(argv));
  arg = 0;
  toggler = 0;
  command = '\0';
  index = 0;
}


void runCommand() {
  arg = atoi(argv);

  switch(command) {
    case 'a':
      pidStatus = false;
      if (arg < 0) {
        digitalWrite(DIR, LOW);
      } else {
        digitalWrite(DIR, HIGH);
      }

      if (abs(arg) > 255) {
        Serial.println("Invalid Speed Command");
      } else {
        analogWrite(PWM, abs(arg));
      }
      break;

    case 'e':
      Serial.println(motorSpeed, 6);
      turn_on = false;
      break;

    case 'm':
      targetSpeed = arg;
      pidStatus = true;
      break;

    case 'p':
      turn_on = true;
      break;

    default:
      Serial.println("invalid Command");
      break;
  }
  Serial.print("Recieved! \r");
}


void countPulses() {
  if (digitalRead(ENC_B) > 0) {
    totalPulse++;
  } else {
    totalPulse--;
  }
}


void calcSpeed() {
  double numOfRev = totalPulse / COUNT_PER_REV;
  motorSpeed = numOfRev * 60;
  totalPulse = totalPulse - numOfRev * COUNT_PER_REV;
}

int pController(double currSpeed, double dt) {
  // if (abs(targetSpeed) <= 150) {
  //   speedInput = 0;
  //   pidStatus = false;
  //   return 0;
  // }

  double currError = targetSpeed - currSpeed;
  return K_P * currError;
}


void setup() {
  Serial.begin(115200);

  pinMode(ENC_A, INPUT_PULLUP);
  pinMode(ENC_B, INPUT_PULLUP);
  pinMode(PWM, OUTPUT);
  pinMode(DIR, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(ENC_A), countPulses, RISING);

  Serial.println("Ready!");
}


void loop() {
  while (Serial.available()) {
    cha = Serial.read();

    if (cha == '\r') {
      if (toggler == 1) argv[index] = '\0';
      runCommand();
      resetCommand();
    } else if (cha == ' ') {
      toggler = 1;
    } else {
      if (toggler == 0) {
        command = cha;
      } else if (toggler == 1) {
        argv[index] = cha;
        index++;
      }
    }
  }

  if (turn_on) {
    // Serial.print(digitalRead(ENC_A));
    // Serial.print(" ");
    // Serial.print(digitalRead(ENC_B));
    // Serial.print(" ");
    // Serial.print(totalPulse);
    // Serial.print(" ");
    Serial.println(motorSpeed);
  }
    
  unsigned long currTime = millis();
  unsigned long diffTime = currTime - prevTime;

  if (diffTime > 1000) {
    calcSpeed();
    prevTime = currTime;

    if (pidStatus) {
      speedInput += pController(motorSpeed, diffTime);

      if (speedInput < 0) {
        digitalWrite(DIR, LOW);
      } else {
        digitalWrite(DIR, HIGH);
      }

      if (abs(speedInput) > 255) {
        analogWrite(PWM, 255);
      } else {
        analogWrite(PWM, abs(speedInput));
      }
    }
  }
}
