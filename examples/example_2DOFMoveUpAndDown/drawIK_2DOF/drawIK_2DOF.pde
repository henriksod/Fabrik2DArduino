import processing.serial.*;

Serial myPort;  // Create object from Serial class
String val;     // Data received from the serial port

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

      if (list.length == 8) {
        int shoulderX = int(float(list[2]))+width/4;
        int shoulderY = height-int(float(list[3]))-height/4;
        int elbowX = int(float(list[4]))+width/4;
        int elbowY = height-int(float(list[5]))-height/4;
        int endX = int(float(list[6]))+width/4;
        int endY = height-int(float(list[7]))-height/4;

        background(204);
        stroke(255,0,0);
        line(200+shoulderX, shoulderY-10, 200+shoulderX, shoulderY-100);
        stroke(0,0,0);
        line(shoulderX, shoulderY, elbowX, elbowY);
        line(elbowX, elbowY, endX, endY);
        fill(150);
        ellipse(shoulderX, shoulderY, 10, 10);
        fill(255);
        ellipse(elbowX, elbowY, 10, 10);
        fill(150,255,150);
        ellipse(endX, endY, 10, 10);
        
        println(val); //print it out in the console
      }
      
    }
  } 
}