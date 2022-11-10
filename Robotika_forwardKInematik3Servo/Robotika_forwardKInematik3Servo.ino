#include <Servo.h>

String myString;
char c;
int Index1,Index2,Index3,Index4,Index5;
String secondValue, thirdValue, fourthValue, firstValue;


 
Servo myservo[3];
int servo_pins[3] = {9, 10, 11}; //pin servo // array
int theta1, theta2, theta3;

double rad_theta1, rad_theta2, rad_theta3;
float result_x, result_y, result_z;


void setup()
{
  Serial.begin(9600);
  while (!Serial);
  Serial.println("-------------------------");
  Serial.println("memuat sistem....");
  delay(500);
  Serial.println("sistem berhasil dimuat");
  Serial.println("-------------------------");
  Serial.println("proses kalibrasi servo...");
  for (int i=0; i<3; i++) { // increment
    myservo[i].attach(servo_pins[i]);
    myservo[i].write(0);
    delay(1000);
  }


Serial.println("servo sudah dikalibrasi");
Serial.println("-------------------------");
Serial.println("Command input aktif, masukan perintah");
Serial.println("-------------------------");
Serial.println("Masukan nilai sudut = [0 0 0] ");
Serial.println("sudut1 sudut2 sudut3  || x  y z ");
}

// fungsi digunakan untuk pemetaan jika input berasal dari potensiometer
//int degree(int x) {
//  int val = map(x, 0, 1023, 0, 90);
//  return val;
//}

void loop() 
{ 
    while (Serial.available()>0){
    delay(10);
    c = Serial.read();
    myString += c;
  }
  if (myString.length()>0){
    Index1 = myString.indexOf('[');
    Index2 = myString.indexOf(' ', Index1+1);
    Index3 = myString.indexOf(' ', Index2+1);
    Index4 = myString.indexOf(' ', Index3+1);
    Index5 = myString.indexOf(']', Index4+1);
   
    secondValue = myString.substring(Index1+1, Index2);
    thirdValue = myString.substring(Index2+1, Index3);
    fourthValue = myString.substring(Index3+1, Index4);
    firstValue = myString.substring(Index4+1, Index5);

    myString = "";
  }
  delay(1000);
  
  theta1 = secondValue.toFloat();
  theta2 = thirdValue.toFloat();
  theta3 = fourthValue.toFloat();
  
  forwardKinematic();
  
  myservo[0].write(theta1);
  myservo[1].write(theta2);
  myservo[2].write(theta3);
  delay(15);

  Serial.print("[ ");
  Serial.print(theta1);
  Serial.print("  ");
  Serial.print(theta2);
  Serial.print("  ");
  Serial.print(theta3);
  Serial.print(" ]");
  Serial.print("  --->  ");
  Serial.print("[ ");
  Serial.print(result_x);
  Serial.print("  ");
  Serial.print(result_y);
  Serial.print("  ");
  Serial.print(result_z);
  Serial.println(" ]");
}

void forwardKinematic() {
  rad_theta1 = theta1 * M_PI / 180;
  rad_theta2 = theta2 * M_PI / 180;
  rad_theta3 = theta3 * M_PI / 180;

  // --------------DH - Parameter -----------------------
  // link  |  Rz(theta) | Tz(d)   | Tx(a)     | Rx(alpha)
  // ----------------------------------------------------
  //   1   |  theta1    | L1 = 95 | 0         | 90       
  //   2   |  theta2    | 0       | L2 = 93.5 | 0      
  //   3   |  theta3    | 0       | L3 = 175  | 0       
  // ----------------------------------------------------

  float L1 = 95;    //d
  float L2 = 93.5; // a1
  float L3 = 175; // a2

  result_x = L3 * (cos(rad_theta2) * cos(rad_theta3) - sin(rad_theta2) * sin(rad_theta3)) * cos(rad_theta1) + L2 * cos(rad_theta2) * cos(rad_theta1);
  result_y = L3 * (cos(rad_theta2) * cos(rad_theta3) - sin(rad_theta2) * sin(rad_theta3)) * sin(rad_theta1) + L2 * cos(rad_theta2) * sin(rad_theta1);
  result_z = L3 * (sin(rad_theta2) * cos(rad_theta3) + cos(rad_theta2) * sin(rad_theta3)) + (L2 * sin(rad_theta2)) + L1;
}
