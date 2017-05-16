
#include <ros.h>
#include <uchile_turning_base/platoMov.h>
#include <Servo.h>

ros::NodeHandle  nh;

Servo myservo;

void messageCb( const uchile_turning_base::platoMov& msg)
  {
    myservo.attach(9);
    myservo.write(89);
    delay((msg.revolution_time*msg.angle)/360);
    myservo.detach();
  }

ros::Subscriber<uchile_turning_base::platoMov> sub("toggle_servo", &messageCb );

void setup()
{ 
  nh.initNode();
  nh.subscribe(sub);
}

void loop()
{  
  nh.spinOnce();
  delay(1);
}

