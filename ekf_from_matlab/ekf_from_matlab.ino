#include <Wire.h>
#include <Adafruit_BNO055.h>
#include <ArduinoEigen.h>
#include <Adafruit_BMP280.h>
#include <TinyGPS++.h>
#include <SD.h>
#include <Servo.h>
#include "EKF_rocket.h"
#include "rt_nonfinite.h"

// PINS

// Voltage divider pwm pin
#define V_PIN 20

//buzzer pin
#define BUZZER_PIN 11

// Servos
#define SERVO_X 9 
#define SERVO_Y 10

#define FIRST_PYRO_PIN 14
#define SECOND_PYRO_PIN 15

// State Machine
int rocket_FSM_state = 0;

// EKF lib
EKF_rocket ekf;

double omB[3] = {0,0,0};
double acB[3] = {0,0,0};
double mB[3] = {0,0,0};


double x0[22] = {0,0,0,0, 0,0,0, 0,0,0, 0,0,0, 0,0,0, 0,0,0, 0,0,0 };
double init_process_cov = 0.000000001;

double euler[3];
double pos[3];
double vel[3];

// Timing variables
unsigned long t = 0; // millis() [ms]
unsigned long t_old = 0;
#define DELAY_MS (8.6) //116Hz

// XBEE
#define XbeeSerial Serial7
bool send_telemetry = true;

// SD card
bool log_data = true;

// Servos
Servo servo_x;
Servo servo_y;

// Setup BNO055 (IMU)
Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x28);
imu::Vector<3> omegaB_imu,accB_imu,magB_imu;

Eigen::Matrix<double,3,1> omegaB;
Eigen::Matrix<double,3,1> accB;
Eigen::Matrix<double,3,1> magB;
Eigen::Matrix<double,3,1> magB_old;
Eigen::Matrix<double,3,3> Aaccb;
Eigen::Matrix<double,3,3> Amagb;
Eigen::Matrix<double,3,1> accBbias;
Eigen::Matrix<double,3,1> magBbias;
Eigen::Matrix<double,3,1> gyroBias;

// GPS (Adafruit Ultimate GPS)
TinyGPSPlus gps;
#define gpsSerial Serial2
double gps_age_old = -1;
double gps_age = 0.0;
double lla[3];
double lla0[3];
double gps_vel_2d[2];
double gps_local[3];

// Barometer (BMP280)
Adafruit_BMP280 bmp; // use I2C interface
double baro_z0 = 0.0;
double baro_z = 0.0;
double baro_z_old = 0.0;
double pressure_sea_level = 1013.25; // hPa

void setup(void)
{
  Serial.begin(115200);

  // Start up SD card and Xbee
  start_sdcard_and_radio();

  // Setup data-link to IMU GPS and baro 
  start_up_sensors();

  // Setup voltage divider, buzzer and pyro pins
  setup_pins();

  // Setup servo PWM
  setup_servos();

  // Calibrates all sensors and estimates initial position and orientation
  calibrate_for_state_estimation(2000,true,true); // inputs: (int)number of iterations, (bool) wait for gps (and estimates initial GPS position and altitude), (bool) estimate referecence barometer altitude 
}


void loop(void)
{  
  // read IMU barometer and GPS and perform sensor fusion
  run_ekf(); 

  // get position and orientation data from EKF  
  ekf.get_eulerZYX(euler);
  ekf.get_posNED(pos);
  ekf.get_velNED(vel);

  
  printdata(); // Print data to serial line

  
  // Log data to SD card
  if(log_data){
    log_data_to_sdcard();
  }

  if(send_telemetry){
    send_telemetry_xbee();
  }

  // Smart delay function that takes into account computational time listens to the GPS serial line
  delay_fn(); 
}


/// FUNCTIONS


void printdata(){
  double rad2deg = 180.0/3.141592;
  Serial.print("eul: ");Serial.print(euler[0]*rad2deg,5); Serial.print(" "); Serial.print(euler[1]*rad2deg,5); Serial.print(" ");  Serial.print(euler[2]*rad2deg,5); Serial.print(" ");
  Serial.print("pos: ");Serial.print(pos[0]); Serial.print(" "); Serial.print(pos[1]); Serial.print(" ");  Serial.print(pos[2]);Serial.print(" ");
  Serial.print("lat/long: ");Serial.print(lla[0]); Serial.print(" "); Serial.print(lla[1]); Serial.print(" ");Serial.print("gps_pos: ");Serial.print(gps_local[0]); Serial.print(" "); Serial.print(gps_local[1]);Serial.print(" "); Serial.print(gps_local[2]); Serial.print(" ");;
  Serial.print("baro: ");Serial.print(baro_z);Serial.print(" ");

  Serial.println(" ");

}

Eigen::Matrix<double,3,1> calibrate_imu(int calculation_steps,bool avg_gps,bool avg_baro){
  imu::Vector<3> gyroBias_imu;
  imu::Vector<3> omegaB_imu;

  Eigen::Matrix<double,3,1> accB_eigen;
  Eigen::Matrix<double,3,1> magB_eigen;

  Eigen::Matrix<double,3,1> accB_eigen_cal;
  Eigen::Matrix<double,3,1> magB_eigen_cal;

  Eigen::Matrix<double,3,1> euler_meas;
  Eigen::Matrix<double,3,1> euler_avg;euler_avg << 0,0,0;

  Eigen::Matrix<double,3,1> magB_avg; magB_avg << 0,0,0;

  double z_baro_avg = 0.0;
  int baro_count = 0;

  double lat_avg = 0.0;
  double long_avg = 0.0;
  double alt_avg = 0.0;

  int gps_count = 0;
  int gps_count_alt = 0;

  Serial.print("Estimating initial state and bias... \n");
  for (int i = 1; i <= calculation_steps; i++) {

    // Average gyro measurements to get constant gyro bias
      omegaB_imu = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
      gyroBias_imu = gyroBias_imu + omegaB_imu;

        // Get acceleration and magnetometer measurements to estimate mag vec and initial orientation
        accB_imu = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
        magB_imu = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);

          // Convert to eigen variables and calibrate
          accB_eigen << accB_imu[0],accB_imu[1],accB_imu[2];
          magB_eigen << magB_imu[0],magB_imu[1],magB_imu[2];

            accB_eigen_cal = Aaccb*(accB_eigen-accBbias);
            magB_eigen_cal = Amagb*(magB_eigen-magBbias);

             // convert to NED frame
            accB_eigen_cal(1) = -accB_eigen_cal(1);
            accB_eigen_cal(2) = -accB_eigen_cal(2);

            magB_eigen_cal(0) = -magB_eigen_cal(0);
            magB_eigen_cal(2) = -magB_eigen_cal(2);

            // get attitude estimate in ZYX euler angles and average it

            double eul[3];
            //euler_meas = euler_ZYX_from_mag_acc(accB_eigen_cal,magB_eigen_cal);

            acB[0] = accB_eigen_cal(0);
            acB[1] = accB_eigen_cal(1);
            acB[2] = accB_eigen_cal(2);

            mB[0] = magB_eigen_cal(0);
            mB[1] = magB_eigen_cal(1);
            mB[2] = magB_eigen_cal(2);
            
            ekf.eulerZYX_from_acc_mag(acB, mB, eul);
            euler_meas << eul[0],eul[1],eul[2];

            //Serial.print("initial att: ");Serial.print(eul[0]); Serial.print(" "); Serial.print(eul[1]); Serial.print(" ");  Serial.print(eul[2]); Serial.println(" ");

            euler_avg = euler_avg + euler_meas;

            // average magnetic vector estimate in bodyframe
            magB_avg = magB_avg + magB_eigen_cal;


            if(avg_gps){
                if(gps.location.isValid()){
                   gps_age = gps.location.age();
                   if((gps_age<=gps_age_old)||(gps_age_old==-1)){
                     gps_age_old=gps_age;
                     lat_avg = gps.location.lat() + lat_avg;
                     long_avg = gps.location.lng()+long_avg;
                     gps_count = gps_count + 1;

                    if(gps.altitude.isValid()){
                      alt_avg = gps.altitude.meters()+alt_avg;
                      gps_count_alt = gps_count_alt + 1;
                     }
                   }
                }
            }

            if(avg_baro){
              z_baro_avg = z_baro_avg + bmp.readAltitude(pressure_sea_level);
              baro_count = baro_count  + 1;
            }

            
      delay(DELAY_MS);
  }

  gyroBias_imu = gyroBias_imu/(double)(calculation_steps);
  euler_avg = euler_avg/(double)(calculation_steps);
  magB_avg = magB_avg/(double)(calculation_steps);

  if(avg_gps){
    lat_avg = lat_avg/(double)gps_count;
    long_avg = long_avg/(double)gps_count;
    alt_avg = alt_avg/(double)gps_count_alt;  

    lla0[0] =lat_avg; 
    lla0[1] =long_avg; 
    lla0[2] =alt_avg; 

  }

  if(avg_baro){
    z_baro_avg = z_baro_avg/(double)baro_count; 
    baro_z0 =z_baro_avg;
  }


  double att[4];
  double eulZYX[3];
  eulZYX[0] = euler_avg(0);
  eulZYX[1] = euler_avg(1);
  eulZYX[2] = euler_avg(2);

  ekf.eulZYX2quat(eulZYX, att);

  x0[0] = att[0];
  x0[1] = att[1];
  x0[2] = att[2];
  x0[3] = att[3];

  double magB_avg_ar[3];
  magB_avg_ar[0] = magB_avg(0);
  magB_avg_ar[1] = magB_avg(1);
  magB_avg_ar[2] = magB_avg(2);

  ekf.init(x0,init_process_cov);
  ekf.set_mag_vec_from_body_avg(magB_avg_ar);

  if(log_data){
    log_init_state_to_sdcard(eulZYX,magB_avg_ar);
  }

  
  // return gyro bias
  Eigen::Matrix<double,3,1> gyroBias_eigen;gyroBias_eigen << gyroBias_imu[0],gyroBias_imu[1],gyroBias_imu[2];
  Serial.print("Gyro bias: ");Serial.print(gyroBias_eigen(0),5); Serial.print(" "); Serial.print(gyroBias_eigen(1),5); Serial.print(" ");  Serial.print(gyroBias_eigen(2),5); Serial.println(" ");
  return gyroBias_eigen;
}



void read_imu(){
  // read raw data from sensor
  omegaB_imu = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  accB_imu = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  magB_imu = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);

  omegaB << omegaB_imu[0],omegaB_imu[1],omegaB_imu[2];
  accB << accB_imu[0],accB_imu[1],accB_imu[2];
  magB << magB_imu[0],magB_imu[1],magB_imu[2];

  // remove bias
  omegaB = (omegaB - gyroBias)*(3.141592/180.0);
  accB = Aaccb*(accB-accBbias);
  magB = Amagb*(magB-magBbias);

 // convert to NED frame
 accB(1) = -accB(1);
 accB(2) = -accB(2);

 omegaB(0) = -omegaB(0);
 omegaB(2) = -omegaB(2);

 magB(0) = -magB(0);
 magB(2) = -magB(2);

 omB[0] = omegaB(0);
 omB[1] = omegaB(1);
 omB[2] = omegaB(2);

 acB[0] = accB(0);
 acB[1] = accB(1);
 acB[2] = accB(2);

 mB[0] = magB(0);
 mB[1] = magB(1);
 mB[2] = magB(2);
}

void calibrate_for_state_estimation(int num_steps,bool wait4gps,bool avg_baro){

  // Set calibration matrices
  Amagb << 0.9733,0.0155,0.0070, 0.0155,0.9676,-0.0039, 0.0070,-0.0039,1.0621;
  magBbias << -28.7506,15.4834,16.8240;
  Aaccb << 1,0,0,0,1,0,0,0,1;
  accBbias << 0,0,0;

  if(wait4gps){

      Serial.print("Waiting for gps fix...\n");
    do{
    delay_fn();
  }while(gps.location.isValid()==false);

  Serial.print("Got gps fix...\n");
    
  }else{
    Serial.print("Not waiting for gps fix...\n");
  }

  // Calculate gyro bias, magnetic vector and initial orientation
  gyroBias = calibrate_imu(num_steps,wait4gps,avg_baro);

  Serial.print("Setup done\n");
  
}


void start_sdcard_and_radio(){
  //Sd stuff

  if(log_data){
      if (!SD.begin(BUILTIN_SDCARD)) {
    Serial.println("SD not present, turning off logs");
    log_data=false;
  }
  }

  if(send_telemetry){
    //XBee serial
    XbeeSerial.begin(115200);
  }


}

void send_telemetry_xbee(){
      XbeeSerial.print(millis());XbeeSerial.print(",");
      XbeeSerial.print(rocket_FSM_state);XbeeSerial.print(",");

    // Log IMU data
      XbeeSerial.print(omB[0],5);XbeeSerial.print(",");
      XbeeSerial.print(omB[1],5);XbeeSerial.print(",");
      XbeeSerial.print(omB[2],5);XbeeSerial.print(",");

      XbeeSerial.print(acB[0],5);XbeeSerial.print(",");
      XbeeSerial.print(acB[1],5);XbeeSerial.print(",");
      XbeeSerial.print(acB[2],5);XbeeSerial.print(",");

      XbeeSerial.print(mB[0],3);XbeeSerial.print(",");
      XbeeSerial.print(mB[1],3);XbeeSerial.print(",");
      XbeeSerial.print(mB[2],3);XbeeSerial.print(",");

      // Log Barometer

      XbeeSerial.print(baro_z);XbeeSerial.print(",");
      
      // Log GPS

      XbeeSerial.print(lla[0],10);XbeeSerial.print(",");
      XbeeSerial.print(lla[1],10);XbeeSerial.print(",");
      XbeeSerial.print(lla[2],4);XbeeSerial.print(",");

      XbeeSerial.print(gps_vel_2d[0],3);XbeeSerial.print(",");
      XbeeSerial.print(gps_vel_2d[1],3);XbeeSerial.print(",");

      // Log EKF state
      XbeeSerial.print(pos[0]);XbeeSerial.print(",");
      XbeeSerial.print(pos[1]);XbeeSerial.print(",");
      XbeeSerial.print(pos[2]);XbeeSerial.print(",");

      XbeeSerial.print(vel[0]);XbeeSerial.print(",");
      XbeeSerial.print(vel[1]);XbeeSerial.print(",");
      XbeeSerial.print(vel[2]);XbeeSerial.print(",");

      XbeeSerial.print(euler[0]);XbeeSerial.print(",");
      XbeeSerial.print(euler[1]);XbeeSerial.print(",");
      XbeeSerial.print(euler[2]);XbeeSerial.print(",");
}

void log_data_to_sdcard(){
      File dataFile = SD.open("state_log.txt", FILE_WRITE);
  if (dataFile) {

    // Log time and finite state machine state
      dataFile.print(millis());dataFile.print(",");
      dataFile.print(rocket_FSM_state);dataFile.print(",");

    // Log IMU data
      dataFile.print(omB[0],5);dataFile.print(",");
      dataFile.print(omB[1],5);dataFile.print(",");
      dataFile.print(omB[2],5);dataFile.print(",");

      dataFile.print(acB[0],5);dataFile.print(",");
      dataFile.print(acB[1],5);dataFile.print(",");
      dataFile.print(acB[2],5);dataFile.print(",");

      dataFile.print(mB[0],3);dataFile.print(",");
      dataFile.print(mB[1],3);dataFile.print(",");
      dataFile.print(mB[2],3);dataFile.print(",");

      // Log Barometer

      dataFile.print(baro_z);dataFile.print(",");
      
      // Log GPS

      dataFile.print(lla[0],10);dataFile.print(",");
      dataFile.print(lla[1],10);dataFile.print(",");
      dataFile.print(lla[2],4);dataFile.print(",");

      dataFile.print(gps_vel_2d[0],3);dataFile.print(",");
      dataFile.print(gps_vel_2d[1],3);dataFile.print(",");

      // Log EKF state
      dataFile.print(pos[0]);dataFile.print(",");
      dataFile.print(pos[1]);dataFile.print(",");
      dataFile.print(pos[2]);dataFile.print(",");

      dataFile.print(vel[0]);dataFile.print(",");
      dataFile.print(vel[1]);dataFile.print(",");
      dataFile.print(vel[2]);dataFile.print(",");

      dataFile.print(euler[0]);dataFile.print(",");
      dataFile.print(euler[1]);dataFile.print(",");
      dataFile.print(euler[2]);dataFile.print(",");
                  
      dataFile.println("");
      dataFile.close();
  } else {
    Serial.println("error opening datalog.txt");
  }
}

void log_init_state_to_sdcard(double init_eulZYX[3],double mag_b[3]){
        File dataFile = SD.open("init_state_ekf.txt", FILE_WRITE);
  if (dataFile) {

    // Log IMU data
      dataFile.print(init_eulZYX[0],5);dataFile.print(",");
      dataFile.print(init_eulZYX[1],5);dataFile.print(",");
      dataFile.print(init_eulZYX[2],5);dataFile.print(",");

      dataFile.print(mag_b[0],5);dataFile.print(",");
      dataFile.print(mag_b[1],5);dataFile.print(",");
      dataFile.print(mag_b[2],5);dataFile.print(",");

      // Log Barometer

      dataFile.print(baro_z0);dataFile.print(",");

      // Log GPS

      dataFile.print(lla0[0],10);dataFile.print(",");
      dataFile.print(lla0[1],10);dataFile.print(",");
      dataFile.print(lla0[2],4);dataFile.print(",");
      
      dataFile.println("");
      dataFile.close();
  } else {
    Serial.println("error opening initial_state.txt");
  }
}

void start_up_sensors(){
  // GPS
  gpsSerial.begin(9600);
  gpsSerial.println("$PMTK251,38400*27");  //set baud rate to 38400
  delay(1000);
  gpsSerial.end();
  gpsSerial.begin(38400);
  gpsSerial.println("$PMTK220,100*2F"); // 10 Hz update rate

  // BMP280 setup
  if (!bmp.begin()) {
    Serial.println(F("Could not find a valid BMP280 sensor!"));
    while (1) delay(10);
  }
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X8,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X2,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */

  delay(100);
  baro_z0 = bmp.readAltitude(pressure_sea_level);

  // BNO055 stuff
  if(!bno.begin())
  {
    Serial.print("no BNO055 detected");
    while(1);
  }
  delay(1000);
  bno.setExtCrystalUse(true);
  bno.setMode(Adafruit_BNO055::OPERATION_MODE_AMG); // disable sensor fusion
  delay(200); // set sensors configuration using the modified library (sends twice bc sometimes it does not take it)
  bno.setGyroConfig116Hz();
  delay(100);
  bno.setAccelConfig16G();
  bno.setGyroConfig116Hz();
  bno.setMagConfig30Hz();
}

void setup_pins(){
    //Pyro stuff
  pinMode(FIRST_PYRO_PIN, OUTPUT);
  pinMode(SECOND_PYRO_PIN, OUTPUT);

  //Buzzer
  pinMode(BUZZER_PIN, OUTPUT);

  //voltage divider (batery power)
  pinMode(V_PIN, INPUT);
}

void setup_servos(){
    //Servo pwm
  servo_x.attach(SERVO_X);
  servo_y.attach(SERVO_Y);
}

void run_ekf(){
    // read sensors
    read_imu();

    // integrate state forwards
    ekf.predict_step(acB, omB, (double)(DELAY_MS)*0.001);

    // correct with magnetometer
    if(magB!=magB_old){
      ekf.update_step_mag(mB);
    }
    magB_old = magB;

    // correct with barometer
    baro_z = baro_z0 - bmp.readAltitude(pressure_sea_level);
    if(baro_z!=baro_z_old){
      ekf.update_step_baro(baro_z);
    }
    baro_z_old = baro_z;

    while (gpsSerial.available()){
      gps.encode(gpsSerial.read());
    }

    // correct with GPS position
   if(gps.location.isValid()){
    gps_age = gps.location.age();

    if((gps_age<=gps_age_old)||(gps_age_old==-1)){

      if(gps_age_old==-1){
        lla0[0] = lla[0];
        lla0[1] = lla[1];
        lla0[2] = lla[2];
      }
      
      gps_age_old=gps_age;
      lla[0] = gps.location.lat();
      lla[1] = gps.location.lng();
      lla[2] = gps.altitude.meters();
      
      ekf.get_gps_local_pos(lla, lla0, gps_local);
      ekf.update_step_gps_pos(lla,lla0);
    }
  }

  if(gps.course.isValid()&&gps.speed.isValid()){
      ekf.gps_vel_2d_from_vel_course( gps.speed.kmph(), gps.course.deg(), gps_vel_2d);
      ekf.update_step_gps_vel_2d(gps_vel_2d);
  }
}

void delay_fn(){
  t = millis();
  if(t_old-t < DELAY_MS){
      do{
    while (gpsSerial.available()){
      gps.encode(gpsSerial.read());
    }
    }while(millis() - t < DELAY_MS-(t_old-t));
  t_old = millis();
  }

}
