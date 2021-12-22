#include <ros.h>
#include <Servo.h>
#include <std_msgs/Float32.h>
ros::NodeHandle nh;
std_msgs::Float32 Distance;
ros::Publisher chatter("/echo",  &Distance);   // Remove????

// DC Motor
#define DIRA  4
#define PWMA  5
float vel = 0;

// Servo Motor
Servo servo;
const int ServoPin = 9;   // 
float deg = 128;   // initial deg

// Echo Sensor
#define TRIG 7   //TRIG PIN (Sending echo)
#define ECHO 6   // ECHO PIN (Receiving echo)

void goMotor(bool Dir, int Speed){
  digitalWrite(DIRA, Dir);
  analogWrite(PWMA, Speed);
}
//
////DC Motor Command Subscriber
void DCcallback( const std_msgs::Float32& DCcmd){
  vel=DCcmd.data;
  goMotor( (vel>=0)?HIGH:LOW, int(min(abs(vel), 255)) );
  delay(1);
}
ros::Subscriber<std_msgs::Float32> sub1("pub_dc", &DCcallback);
////Servo Motor Command Subscriber
void Servocallback( const std_msgs::Float32& Servocmd){
  deg=Servocmd.data;
  servo.write(int(deg));
  delay(1);
}
ros::Subscriber<std_msgs::Float32> sub2("pub_servo", &Servocallback);

void setup() {
  nh.initNode();
  nh.advertise(chatter);
  nh.subscribe(sub1);
  nh.subscribe(sub2);

  servo.attach(ServoPin);
  servo.write(130);
  
  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT);
  pinMode(DIRA, OUTPUT);
  pinMode(PWMA, OUTPUT);
  goMotor(0,0);
  delay(1000);
}

void loop() {

  long duration, distance;

  digitalWrite(TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);

  duration = pulseIn (ECHO, HIGH);   // measure and save time echo comes back

  // 34000 * time/1000000/2
  // change distance unit to cm
  distance = duration * 17 / 1000; 

  Distance.data = distance;
  chatter.publish(&Distance);
  nh.spinOnce();

  // check distance data through PC monitor cmd
  Serial.println(duration );   // show time how long it took echo comes back
  Serial.print("\nDIstance : ");
  Serial.print(distance);   // show distance between the sensor and the obstacle
  Serial.println(" Cm");

  delay(1000);   // repeat for every 1 second
}
