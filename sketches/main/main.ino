    #include <NewPing.h>
     
    #define MAX_DISTANCE 800
     
    NewPing sonar(29, 28, MAX_DISTANCE);
    NewPing sonar2(27, 26, MAX_DISTANCE);
     
    void setup() {
      Serial.begin(9600);
    }
     
    void loop() {
      delay(300);
      Serial.println("============");
      Serial.print("1 Ping: ");
      Serial.print(sonar.convert_cm(sonar.ping_median(5)));
      Serial.println("cm");

      delay(100);

      Serial.print("2 Ping: ");
      Serial.print(sonar2.convert_cm(sonar2.ping_median(5)));
      Serial.println("cm");
    }
}
