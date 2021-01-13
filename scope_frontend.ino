int val = 0, avg = 0;
long interval = 0, curr = 0, prev = 0;
char input[8];

void setup(){
  pinMode(13, OUTPUT);
  pinMode(A0, INPUT);
  analogReadResolution(10);
  Serial.begin(9600);
  delay(500);
  GetInterval();
}

void GetInterval() {
  Serial.flush();
  digitalWrite(13, HIGH);
  Serial.print("Please input interval (in us) between measurements...\n");
  interval = 0;
  while (not(interval)) {
      if (Serial.available()) {
          Serial.readBytesUntil("\n", input, 8);
          if (atol(input) > 0) {
            interval = atol(input);
            Serial.printf("Message received: %03ld\n", interval);
            avg = (int)pow(2, (float)((int)(log(interval)/(log(2.5))))- 1.0);
            analogReadAveraging(avg);
//            Serial.printf("Averaging: %d", avg);
            digitalWrite(13, LOW);
            delay(1000);
          }
      }
  }
}

void loop() {
  curr = micros();
  val = analogRead(A0);
  Serial.printf("%04d,%lu\n", val, curr - prev);
    if (Serial.available()) {
      Serial.readBytesUntil("\n", input, 8);
      if (atoi(input) == -6) {
        GetInterval();
        return;
      }
  }
  prev = curr;
  while (curr - prev < interval) {
    curr = micros();
  }
}
