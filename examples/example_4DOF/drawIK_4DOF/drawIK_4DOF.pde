import processing.serial.*;

Serial myPort;  // Create object from Serial class
String val;     // Data received from the serial port

float maxX = -1000;
float maxY = -1000;

void setup()
{
  // On Windows machines, Serial.list()[0] generally opens COM1.
  // Open whatever port is the one you're using.
  String portName = Serial.list()[1]; //change the 0 to a 1 or 2 etc. to match your port
  myPort = new Serial(this, portName, 9600);
  
  size(350, 350);
}

void draw()
{
  if ( myPort.available() > 0) 
  {  // If data is available,
    val = myPort.readStringUntil('\n');         // read it and store it in val
    if (val != null) {
      
      String[] list = split(val, '\t');

      if (list.length == 12) {
        float angS = float(list[0])*(PI/180);
        float angE = float(list[1])*(PI/180);
        float angW = float(list[2])*(PI/180);
        
        float shoulderX = int(float(list[3]))+width/4;
        float shoulderY = height-int(float(list[4]))-height/10;
        float elbowX = int(float(list[5]))+width/4;
        float elbowY = height-int(float(list[6]))-height/10;
        float wristX = int(float(list[7]))+width/4;
        float wristY = height-int(float(list[8]))-height/10;
        float endX = int(float(list[9]))+width/4;
        float endY = height-int(float(list[10]))-height/10;
        float baseAngle = float(list[11]);
        
        if (endX > maxX) maxX = endX;
        if (endY > maxY) maxY = endY;

        background(204);
        stroke(255,0,0);
        noFill();
        line(50+shoulderX, shoulderY-100, 150+shoulderX, shoulderY-100);
        stroke(0,0,0);
        line(shoulderX, shoulderY, elbowX, elbowY);
        line(elbowX, elbowY, wristX, wristY);
        line(wristX, wristY, endX, endY);
        fill(150);
        ellipse(shoulderX, shoulderY, 10, 10);
        fill(255);
        ellipse(elbowX, elbowY, 10, 10);
        ellipse(wristX, wristY, 10, 10);
        fill(150,255,150);
        ellipse(endX, endY, 10, 10);
        fill(255);
        
        float offsetX = 180;
        float offsetY = 280;
        stroke(255,0,0);
        line(shoulderX+offsetX+50*0.5, shoulderY-offsetY-0*0.5, shoulderX+offsetX+150*0.5, shoulderY-offsetY-0*0.5);
        line(shoulderX+offsetX+150*0.5, shoulderY-offsetY-0*0.5, shoulderX+offsetX+150*0.5, shoulderY-offsetY-50*0.5);
        line(shoulderX+offsetX+50*0.5, shoulderY-offsetY-0*0.5, shoulderX+offsetX+50*0.5, shoulderY-offsetY-50*0.5);
        line(shoulderX+offsetX+50*0.5, shoulderY-offsetY-50*0.5, shoulderX+offsetX+150*0.5, shoulderY-offsetY-50*0.5);
        stroke(0,0,0);
        fill(255);
        line(shoulderX+offsetX, shoulderY-offsetY, shoulderX+offsetX+0.5*(elbowX-shoulderX)*cos(baseAngle), shoulderY-offsetY-0.5*(elbowX-shoulderX)*sin(baseAngle));
        line(shoulderX+offsetX+0.5*(elbowX-shoulderX)*cos(baseAngle), shoulderY-offsetY-0.5*(elbowX-shoulderX)*sin(baseAngle), shoulderX+offsetX+0.5*(wristX-shoulderX)*cos(baseAngle), shoulderY-offsetY-0.5*(wristX-shoulderX)*sin(baseAngle));
        line(shoulderX+offsetX+0.5*(wristX-shoulderX)*cos(baseAngle), shoulderY-offsetY-0.5*(wristX-shoulderX)*sin(baseAngle), shoulderX+offsetX+0.5*(endX-shoulderX)*cos(baseAngle), shoulderY-offsetY-0.5*(endX-shoulderX)*sin(baseAngle));
        fill(150);
        ellipse(shoulderX+offsetX, shoulderY-offsetY, 10, 10);
        fill(255);
        ellipse(shoulderX+offsetX+0.5*(elbowX-shoulderX)*cos(baseAngle), shoulderY-offsetY-0.5*(elbowX-shoulderX)*sin(baseAngle), 10, 10);
        ellipse(shoulderX+offsetX+0.5*(wristX-shoulderX)*cos(baseAngle), shoulderY-offsetY-0.5*(wristX-shoulderX)*sin(baseAngle), 10, 10);
        fill(150,255,150);
        ellipse(shoulderX+offsetX+0.5*(endX-shoulderX)*cos(baseAngle), shoulderY-offsetY-0.5*(endX-shoulderX)*sin(baseAngle), 10, 10);
        fill(255);
        
        println(val); //print it out in the console
        
        //println(sqrt(pow(wristX-wristX,2)+pow(wristY-elbowY,2)));
      }
      
    }
  } 
}