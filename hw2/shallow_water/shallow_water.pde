Camera camera;
float grav = 9.8; 

int nx = 60;
float dx = 2, totalLen = nx * dx, sim_dt = 0.002, damp = 0.05,
      left = 200, right = left + int(nx * dx), poolWidth = 100;
float[] heightVec, momentumVec, heightMVec, momentumMVec;

void setup() {
  size(800, 600, P3D);
  camera = new Camera();
  
  heightVec = new float[nx];
  momentumVec = new float[nx];
  heightMVec = new float[nx];
  momentumMVec = new float[nx];
  
  for (int i = 0;i < nx; ++i) {
      if (i < nx / 2 - 20)
        heightVec[i] = 15 * random(1.0);
      else if (i >= nx / 2 - 20 && i < nx / 2 + 20)
        heightVec[i] = 5;
      else
        heightVec[i] = 10 * random(1.0);
        
      momentumVec[i] = 0;
      heightMVec[i] = 0;
      momentumMVec[i] = 0;
  }
  
  computeWaves(sim_dt);

  /*
  heightVec[0] = heightVec[1];
  heightVec[nx - 1] = heightVec[nx - 2];
  momentumVec[0] = -momentumVec[1];
  momentumVec[nx - 1] = -momentumVec[nx-2];
  */
  /*
  heightVec[0] = heightVec[nx - 2];
  heightVec[nx - 1] = heightVec[1];
  momentumVec[0] = momentumVec[nx - 2];
  momentumVec[nx - 1] = momentumVec[1];
  */
}


void update(float dt){
  for ( int i = 0;i < int(dt / sim_dt); ++i)
    computeWaves(sim_dt);
}

void draw() {
  background(255,255,255);
  camera.Update( 1.0/frameRate );
  pointLight(200, 200, 200, 250, -150, 50);
  
  pushMatrix();
  noFill();
  stroke(0, 0, 0);
  strokeWeight(2);
  beginShape(LINES);
  vertex(left, 50, 0);
  vertex(left, -20, 0);
  vertex(left, -20, 0);
  vertex(left + totalLen - dx, -20, 0);
  vertex(left + totalLen - dx, -20, 0);
  vertex(left + totalLen - dx, 50, 0);
  
  vertex(left + totalLen - dx, -20, -100);
  vertex(left + totalLen - dx, 50, -100);
  vertex(left + totalLen - dx, -20, 0);
  vertex(left + totalLen - dx, -20, -100);
  
  vertex(left + totalLen - dx, -20, -100);
  vertex(left, -20, -100);
  vertex(left, -20, -100);
  vertex(left, 50, -100);
  
  vertex(left, -20, -100);
  vertex(left, -20, 0);
  
  endShape();
  popMatrix();
  
  for (int i = 0; i < nx - 1; ++i) {
    float x1 = left + i * dx, x2 = left + i * dx + dx;
    /*
    float[] v1 = new float[]{x2-x1, heightVec[i+1]-heightVec[i], 0},
            v2 = new float[]{0, 0, -1};
    float[] n = new float[3];
    n[0] = v1[1]*v2[2] - v1[2]*v2[1];
    n[1] = v1[2]*v2[0] - v1[0]*v2[2];
    n[2] = v1[0]*v2[1] - v1[1]*v2[0];
    */
    
    fill(128, 128, 255);
    //stroke(128, 128, 255);
    noStroke();
    beginShape(QUADS);
    //normal(n[0], n[1], n[2]);
    vertex(x1, heightVec[i], 0);
    vertex(x2, heightVec[i+1], 0);
    vertex(x2, heightVec[i+1], -poolWidth);
    vertex(x1, heightVec[i], -poolWidth);
    
    vertex(x1, 50, 0);
    vertex(x1, heightVec[i], 0);
    vertex(x2, heightVec[i+1], 0);
    vertex(x2, 50, 0);
    endShape(CLOSE);
  }
  
  for (int i = 0; i < 1; i++){
    update(0.05);
  }
  
  surface.setTitle("1D shallow water, FPS: " + str(round(frameRate)));
}

void computeWaves(float dt) 
{
  for (int i = 0;i < nx-1; ++i) {
    heightMVec[i] = (heightVec[i] + heightVec[i+1])/2.0 
                    - (dt / 2.0) * (momentumVec[i+1] - momentumVec[i])/dx;
    momentumMVec[i] = (momentumVec[i] + momentumVec[i+1]) / 2.0
                      - (dt / 2.0) * (
                        sq(momentumVec[i+1]) / heightVec[i+1] + 
                        0.5 * grav * sq(heightVec[i+1]) -
                        sq(momentumVec[i]) / heightVec[i] - 
                        0.5 * grav * sq(heightVec[i])
                      ) / dx;
  }
  
  for(int i = 0;i < nx-2; ++i) {
    heightVec[i+1] -= dt * (momentumMVec[i+1] - momentumMVec[i]) / dx;
    momentumVec[i+1] -= dt * (
      damp * momentumVec[i+1] + 
      sq(momentumMVec[i+1]) / heightMVec[i+1] + 
      0.5 * grav * sq(heightMVec[i+1]) -
      sq(momentumMVec[i]) / heightMVec[i] - 
      0.5 * grav * sq(heightMVec[i])
    ) / dx;
  }
  
  heightVec[0] = heightVec[nx - 2];
  heightVec[nx - 1] = heightVec[1];
  momentumVec[0] = momentumVec[nx - 2];
  momentumVec[nx - 1] = momentumVec[1];
}

void keyPressed()
{
  camera.HandleKeyPressed();
}

void keyReleased()
{
  camera.HandleKeyReleased();
}
