
import processing.serial.*;
import java.nio.ByteBuffer;
import java.nio.ShortBuffer;
import java.nio.ByteOrder;

Serial serial;

final int GRAPH_DETAIL = 64;

float pitch, roll, yaw;
float accelX, accelY, accelZ;
float pitchSp, rollSp, yawSp;
float gyroX, gyroY, gyroZ;
float rc[] = new float[6];
float va, vb, vc, vd = 0; // motor velocities
float arduinoDeltaTime = 1;

class Graph {
  private int m_index;
  private float m_data[];
  private int m_size;

  public Graph(int size) {
    m_index = 0; 
    m_size = size;
    m_data = new float[size];
  }

  public void appendData(float data) {
    if (m_index > 0) {
      //if (data != m_data[m_index-1]) {
        m_data[m_index++] = data;
      //}
    } else m_data[m_index++] = data; 

    if (m_index >= GRAPH_DETAIL) {
      m_index = GRAPH_DETAIL-1;
      for (int i = 1; i < m_data.length; i++) {
        m_data[i-1] = m_data[i];
      }
    }
  }


  public void draw(float startX, float startY, float spacing, float amp) {
    for (int i = 0; i < m_size-1; i++) {
      float val1 = -m_data[i];
      float val2 = -m_data[i+1];

      line(startX+i*spacing, startY+val1*amp, 
      startX+(i+1)*spacing, startY+val2*amp);
    }
  }
};

Graph pitchGraph = new Graph(GRAPH_DETAIL);
Graph rollGraph = new Graph(GRAPH_DETAIL);
Graph yawGraph = new Graph(GRAPH_DETAIL);

Graph pitchSpGraph = new Graph(GRAPH_DETAIL);
Graph rollSpGraph = new Graph(GRAPH_DETAIL);
Graph yawSpGraph = new Graph(GRAPH_DETAIL);

Graph rc3Graph = new Graph(GRAPH_DETAIL);

void setup() {
  size(800, 600);
  fill(255);
  rectMode(CENTER);
  frameRate(60);

  PFont font;
  font = createFont("Arial", 20);
  textFont(font);
  textAlign(LEFT, CENTER);
}


void draw() {

  waitForSerialConnection();

  if (serial != null) {
    while(readSerialData() == true){
      
    }
    //readSerialData();
   // while(){
      
 //   }
  } else {
  }

  background(128);
  stroke(255);


  updateGraphs();
  drawOrientationGraphs();
  drawTransmitter(300, 300);
  drawMotorVelocities(400, 400);
  
  pitchSpGraph.appendData(pitchSp);
  rollSpGraph.appendData(rollSp);
  yawSpGraph.appendData(yawSp);
  
  rc3Graph.appendData(rc[1]);
  rc3Graph.draw(400, 100, 200.0f/GRAPH_DETAIL,50);

  noFill();
  stroke(0);
  textAlign(LEFT,CENTER);
  for (int i = 0; i < 6; i++) {
    text("RC(" + (i+1) + "): " + rc[i], 600, 50+i*20);
  }
  
}

void drawMotorVelocities(float x, float y) {
  ellipseMode(CENTER);


  float ax = x;
  float ay = y;

  float bx = x;
  float by = y+100;

  float cx = x+100;
  float cy = y+100;

  float dx = x+100;
  float dy = y;

  // A
  textAlign(CENTER, CENTER);
  fill(((va-30f)/110.0f)*255, 0, 0);
  ellipse(ax, ay, 30, 30);
  fill(255, 255, 255);
  text("A", ax, ay-3);
  textAlign(RIGHT, CENTER);
  text(""+va, ax-20, ay-3);

  // B
  textAlign(CENTER, CENTER);
  fill(((vb-30f)/110.0f)*255, 0, 0);
  ellipse(bx, by, 30, 30);
  fill(255, 255, 255);
  text("B", bx, by-3);
  
  textAlign(RIGHT, CENTER);
  text(""+vb, bx-20, by-3);

  // C
  textAlign(CENTER, CENTER);
  fill(((vc-30f)/110.0f)*255, 0, 0);
  ellipse(cx, cy, 30, 30);
  fill(255, 255, 255);
  text("C", cx, cy-3);
  textAlign(LEFT, CENTER);
  text(""+vc, cx+20, cy-3);

  // D
  textAlign(CENTER, CENTER);
  fill(((vd-30f)/110.0f)*255f, 0, 0);
  ellipse(dx, dy, 30, 30);
  fill(255, 255, 255);
  text("D", dx, dy-3);
  textAlign(LEFT, CENTER);
  text(""+vd, dx+20, dy-3);

  line(cx-10, cy-10, ax+10, ay+10);
  line(bx+10, by-10, dx-10, dy+10);
}


void drawTransmitter(float x, float y) {
  fill(200, 200, 200);
  rect(x, y, 100, 100);
  rect(x+150, y, 100, 100);

  noFill();
  strokeWeight(2);
  stroke(0, 0, 0);
  rect(x, y, 100, 100);
  rect(x+150, y, 100, 100);
  rect(x+(rc[3]-0.5f)*100.0f, y-(rc[2]-0.5f)*100.0f, 10, 10);
  rect(x+150+(rc[0]-0.5f)*100.0f, y-(rc[1]-0.5f)*100.0f, 10, 10);
}

void drawOrientationGraphs() {
  textAlign(LEFT,CENTER);
  strokeWeight(2);
  stroke(0);
  drawSquare(50, 50, 200, 100);
  drawSquare(50, 150, 200, 100);
  drawSquare(50, 250, 200, 100);

  strokeWeight(1);

  stroke(255, 0, 0);
  fill(255, 0, 0);
  text("Pitch:" + pitch, 50, 50);
  pitchGraph.draw(50, 100, 200.0f/GRAPH_DETAIL, 7);
  pitchSpGraph.draw(50, 100, 200.0f/GRAPH_DETAIL, 7);

  stroke(0, 255, 0);
  fill(0, 255, 0);
  text("Roll:" + roll, 50, 70);
  rollGraph.draw(50, 200, 200.0f/GRAPH_DETAIL, 7);
  rollSpGraph.draw(50, 200, 200.0f/GRAPH_DETAIL, 7);

  stroke(0, 0, 255);
  fill(0, 0, 255);
  text("Yaw:" + yaw, 50, 90);
  yawGraph.draw(50, 300, 200.0f/GRAPH_DETAIL, 7);
  yawSpGraph.draw(50, 300, 200.0f/GRAPH_DETAIL, 7);
}

void drawSquare(float x, float y, float width, float height) {
  line(x, y, x+width, y);
  line(x, y, x, y+height);
  line(x+width, y, x+width, y+height);
  line(x, y+height, x+width, y+height);
}

void waitForSerialConnection() {
  if (Serial.list()[0].equals("/dev/ttyS0") || serial == null) {
    while (true) {
      String serialStr = Serial.list()[0];
      if (serialStr.equals("/dev/ttyS0") == false ) {
        delay(500);
        serial = new Serial(this, serialStr, 115200); 
        println("Connected to serial at port: " + serialStr);
        break;
      }
    }
  }
}

void updateGraphs() {

  pitchGraph.appendData(pitch);
  rollGraph.appendData(roll);
  yawGraph.appendData(yaw);
}

void drawData(float startX, float startY, float space, float amp, float data[]) {
}

boolean readSerialData() {
  final int DATA_COUNT = 26;
  
  int data[] = new int[DATA_COUNT];

    if (serialRead16(data, DATA_COUNT) == true) {
        
              if(data[0] == 1 && data[1] == 2 && data[2] == 3){
                gyroX = data[3]/1000.0f;
                gyroY = data[4]/1000.0f;
                gyroZ = data[5]/1000.0f;
          
                accelX = data[6]/1000.0f;
                accelY = data[7]/1000.0f;
                accelZ = data[8]/1000.0f;
          
                pitch = data[9]/1000.0f;
                roll = data[10]/1000.0f;
                yaw = data[11]/1000.0f;
          
          
                for (int i = 0; i < 6; i++) {
                  rc[i] = data[12+i]/1000.0f;
                }
          
                va = data[18]/100.0f;
                vb = data[19]/100.0f;
                vc = data[20]/100.0f;
                vd = data[21]/100.0f;
                
                pitchSp = data[22]/100.0f;
                rollSp = data[23]/100.0f;
                yawSp = data[24]/100.0f;
                
                arduinoDeltaTime = data[25]/10000.0f;
                
                return true;
           }
    }
    return false;
    
}

float maxArray(float data[]) {
  float max = 0;
  for (int i = 0; i < data.length; i++) {
    if (data[i] > max) max = data[i];
  } 

  return max;
}

float minArray(float data[]) {
  float min = 0;
  for (int i = 0; i < data.length; i++) {
    if (data[i] < min) min = data[i];
  } 

  return min;
}

// reads array of integers from Arduino serial, returns true of read succesfully
boolean serialRead16(int data[], int size)
{
  byte[] bytes = new byte[2]; 
  if (serial.available() >= 2*size) {

    for (int i = 0; i < size; i++) {
      data[i] = serialRead16();
    }
  } else return false;

  return true;
}

// reads single integer from Arduino serial
short serialRead16()
{
  short data = 0;
  byte[] bytes = new byte[2]; 
  if (serial.available() >= 2) {

    serial.readBytes(bytes);
    ByteBuffer buffer = ByteBuffer.allocate(2);
    buffer.order(ByteOrder.LITTLE_ENDIAN);
    buffer.put(bytes[0]);
    buffer.put(bytes[1]);
    data = buffer.getShort(0);
  }

  return data;
}

