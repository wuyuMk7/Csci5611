float floor = 400;
float grav = 9.8; 
float radius = 20;

float k=150, kV=.1, restLen=100, mass=1;
float[] posX, posY, accX, accY, velX, velY;
int numV = 5, pushBall = 2;

void setup() {
  size(1200, 800, P3D);
  surface.setTitle("Ball on Spring!");
  
  posX = new float[numV]; posY = new float[numV];
  accX = new float[numV]; accY = new float[numV];
  velX = new float[numV]; velY = new float[numV];
  velX[0] = 0; velY[0] = 0;
  for (int i = 0;i < numV; ++i) {
    posX[i] = 600;
    posY[i] = 100 * (i+1);
  }
}

void update(float dt){
  for (int q = 0; q < 10; q++){ //10 substeps
    float[] springForce = new float[numV];
    springForce[0] = 0;
    for (int i = 0; i < numV; i++) {
      accX[i] = 0; 
      accY[i] = 0;
    } //Reset acceleration
    for (int i = 1; i <= numV-1; i++){ 
      float xLen = posX[i] - posX[i-1], 
            yLen = posY[i] - posY[i-1],
            leng = sqrt(sq(xLen) + sq(yLen));
      springForce[i] = (k / restLen) * (leng - restLen); 
    }
    
    for (int i = 1; i <= numV-1; i++){ //Interior verts    
      float sForceX = 0, sForceY = 0;
      if (i == numV - 1) {
        float xLen = posX[i] - posX[i-1], 
              yLen = posY[i] - posY[i-1],
              leng = sqrt(sq(xLen) + sq(yLen));
        sForceX = 0.5 * (-springForce[i]) * (xLen / leng);
        sForceY = 0.5 * (-springForce[i]) * (yLen / leng);
      } else {
        float xLenUp = posX[i] - posX[i-1],
              yLenUp = posY[i] - posY[i-1],
              lengUp = sqrt(sq(xLenUp) + sq(yLenUp)),
              xLenDown = posX[i+1] - posX[i],
              yLenDown = posY[i+1] - posY[i],
              lengDown = sqrt(sq(xLenDown) + sq(yLenDown));
        sForceX = 0.5 * (-springForce[i]) * (xLenUp / lengUp) + 
                  0.5 * (springForce[i+1]) * (xLenDown / lengDown);
        sForceY = 0.5 * (-springForce[i]) * (yLenUp/ lengUp) + 
                  0.5 * (springForce[i+1]) * (yLenDown / lengDown);
      }  
      
      float accX = (sForceX - kV * (velX[i] - velX[i-1])) / mass, 
            accY = (sForceY - kV * (velY[i] - velY[i-1])) / mass + grav;
    
      velX[i] += accX * dt;
      velY[i] += accY * dt;
      posX[i] += velX[i] * dt; 
      posY[i] += velY[i] * dt;
    }
  }
}

void keyPressed() {
  if (keyCode == RIGHT) {
    velX[pushBall] += 80;
  }
  if (keyCode == LEFT) {
    velX[pushBall] -= 80;
  }
}

void draw() {
  background(255,255,255);
  for (int i = 0; i < 10; i++){
    update(1/(10.0*frameRate));
  }

  fill(0,0,0);
  stroke(5);

  for (int i = 0;i < numV-1; ++i) {
    line(posX[i], posY[i], posX[i+1], posY[i+1]);  
    push();
    translate(posX[i+1], posY[i+1]);
    noStroke();
    fill(0,200,10);
    sphere(radius);
    pop();
  }
}


