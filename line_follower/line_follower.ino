// Include the necessary packages
#include "Arduino_Alvik.h"

// Instanstiate the alvik object
Arduino_Alvik alvik;

// Variables for line following
int line_sensors[3];
float error = 0;
float control = 0;
float kp = 50.0;
// List containing colors of RGB
int rgb_colors[3];
// String read from Serial communication from Raspberry Pi
String readString;

// Setting Alvik up for Serial communication
void setup() {
  Serial.begin(115200); // Establish Serial port communication via USB-C
  while(!Serial){
    ; // wait for USB serial to come out
  }
  alvik.begin();
  alvik.left_led.set_color(0,0,1);
  alvik.right_led.set_color(0,0,1);

  while(!alvik.get_touch_ok()){
    delay(50);
  }
}


void loop() {
  while (!alvik.get_touch_cancel()){
    
    // Turns light below Alvik off (otherwise, blows the colour values)
    alvik.set_illuminator(false);

    // Retrives the IR sensor values and stores them in line_sensors list
    alvik.get_line_sensors(line_sensors[0], line_sensors[1], line_sensors[2]);
    alvik.get_color_raw(rgb_colors[0],rgb_colors[1],rgb_colors[2]); // returns RGB values into the list rgb_colors

    // Error for the line following is calculated
    error = calculate_center(line_sensors[0], line_sensors[1], line_sensors[2]);
    control = error * kp;

    // For Serial Monitoring of the colours below the Alvik
    Serial.print(rgb_colors[0]);
    Serial.print("\t");
    Serial.print(rgb_colors[1]);
    Serial.print("\t");
    Serial.print(rgb_colors[2]);
    Serial.print("\n");

    if ((rgb_colors[0] == 4) && (rgb_colors[1] == 4) && (rgb_colors[2] == 4 || rgb_colors[2] == 3)){ // If the triple (R,G,B) is any of these values, strong indication that yellow is below the Alvik
      alvik.left_led.set_color(1,0,0); // brake lights
      alvik.right_led.set_color(1,0,0);
      alvik.brake();
      delay(2000);
      alvik.rotate(14);
      delay(2000);
      alvik.set_wheels_speed(10,10);
      delay(2000);
      alvik.brake();
      delay(3000);
      
      // Alvik communicates via Serial to the Pi to take a photo and calculate the distance of the ping pong ball (if any)
      Serial.println("photo");
      
      // Alvik remains stopped while the Serial bus is empty
      while(Serial.available() <= 0){
        alvik.brake();
      }

      // Alvik reads from Serial if Pi is sending information over and reads the encoded message
      while(Serial.available()){
        if (Serial.available() > 0 ){
          char c = Serial.read();
          readString += c;
        }
      }

      int distance = readString.toInt(); // mm
      int rpm = 10; // rpm
      int time = 60000*distance/(rpm*3.14159*3.4); // milliseconds
      Serial.println(time); // prints the time needed in milliseconds in Serial Monitor
      alvik.set_wheels_speed(rpm,rpm); // Wheel speed set for Alvik to approach object
      delay(time); // Delays the exact time needed to travel the distance required
      alvik.brake(); // Alvik stops for 3 seconds to show it has ended this operation
      delay(3000);
    }

    // Deviation to the right
    else if (control > 0.2){
      alvik.left_led.set_color(1,0,0);
      alvik.right_led.set_color(0,0,0);
    }
    else{
      // Deviation to the left
      if (control < -0.2){
        alvik.left_led.set_color(0,0,0);
        alvik.right_led.set_color(1,0,0);
      }
      // On course
      else{
        alvik.left_led.set_color(0,1,0);
        alvik.right_led.set_color(0,1,0);
      }
    }
    
    // Speed adjusted so that the Alvik follows the line
    alvik.set_wheels_speed(30-control, 30+control);
    delay(100);
  }
  
  while (!alvik.get_touch_ok()){
    alvik.left_led.set_color(0,0,1);
    alvik.right_led.set_color(0,0,1);
    alvik.brake();
    delay(100);
  }
}

// Calculates the center of the IR sensors
float calculate_center(const int left, const int center, const int right){
  float centroid = 0.0; 
  float sum_weight = left + center + right;
  float sum_values = left + center * 2 + right * 3;
  if (sum_weight!=0.0){                                                         // divide by zero protection
    centroid=sum_values/sum_weight;
    centroid=-centroid+2.0;                                                     // so it is right on robot axis Y
  }
  return centroid;
}
