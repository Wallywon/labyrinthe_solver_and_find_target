
import java.util.ArrayList;
import java.util.List;

import com.cyberbotics.webots.controller.Camera;
import com.cyberbotics.webots.controller.CameraRecognitionObject;
import com.cyberbotics.webots.controller.DistanceSensor;
import com.cyberbotics.webots.controller.LED;
import com.cyberbotics.webots.controller.Motor;
import com.cyberbotics.webots.controller.PositionSensor;
import com.cyberbotics.webots.controller.Robot;

public class AutonomyTwo extends Robot {
	private int timeStep;
	private DistanceSensor[] distanceSensor;	
	private Motor leftMotor;
	private Motor rightMotor;
	private PositionSensor leftMotorSensor;
	private PositionSensor rightMotorSensor;
	private double encoder_unit=159.23;
	private Odometry odometry;	
	private Camera camera;
	private LED[] leds;
	
	
	public AutonomyTwo() {
		timeStep = 64;  // set the control time step
		
		odometry = new Odometry(); // to compute a relative position (if needed)

		// Sensors initialization 
		// IR distance sensors
		distanceSensor = new DistanceSensor[8];
		String[] sensorNames = {
				"ps0", "ps1", "ps2", "ps3",
				"ps4", "ps5", "ps6", "ps7"
		};

		for (int i = 0; i < 8; i++) {
			distanceSensor[i] = this.getDistanceSensor(sensorNames[i]);
			distanceSensor[i].enable(timeStep);
		}

		// Camera
		camera=this.getCamera("camera");
		camera.enable(timeStep);
		camera.recognitionEnable(timeStep);

		// Actuators initialization
		// Motors
		leftMotor = this.getMotor("left wheel motor");
		rightMotor = this.getMotor("right wheel motor");
		leftMotor.setPosition(Double.POSITIVE_INFINITY);
		rightMotor.setPosition(Double.POSITIVE_INFINITY);
		leftMotor.setVelocity(0.0);
		rightMotor.setVelocity(0.0);
		
		// Motor sensors : to compute relative position
		leftMotorSensor = this.getPositionSensor("left wheel sensor");
		rightMotorSensor = this.getPositionSensor("right wheel sensor");
		leftMotorSensor.enable(timeStep);
		rightMotorSensor.enable(timeStep);

		// LEDS
		leds = new LED[10];
		String[] ledsNames = {
				"led0", "led1", "led2", "led3",
				"led4", "led5", "led6", "led7",
				"led8", "led9"
		};
		for (int i = 0; i < 10; i++) {
			leds[i] = this.getLED(ledsNames[i]);
		}
	}

	

	/**
	 * The main method of the robot behaviour
	 */	
	public void run() {
		// if necessary - allow the computation of the relative position of the robot
		initLocalisation(); 
		
		// main control loop: perform simulation steps of timeStep milliseconds
		// and leave the loop when the simulation is over
		//init nombre angle
		double[] psValues = {0, 0, 0, 0, 0, 0, 0, 0};
		int angle=0;
		int state=0;//0 = avance 1=SUIS le mur a gauche 2 = tourne a droite
		while (step(timeStep) != -1) {
			//vérifie sensor value
			
			psValues=readDistanceSensorValues();
			
			if(angle==0){
      			    if(psValues[0]<80 && psValues[7]<80 && psValues[6]<80 && psValues[1]<80){
          			       move(20,20);
          			       if(state==0) {
          			    	   state=2;
          			       }
      			    }
      			    
      			    else{
      			    	move(20.0,-20.0);
            			if(state==2 && psValues[5]>130) {
            			    angle=angle-1;
            			    state=0;
            			    System.out.println(angle);
            			}
          			    }
			}
			else{
  			
			 if(psValues[5]<100){
            			    move(-20.0,10.0);
            			    if(state==1 && psValues[5]>95){
            			    state=0;
            			    angle=angle+1;
            			    System.out.println(angle);
            			    }
      			   }else if(psValues[0]<80 && psValues[7]<80 && psValues[6]<80 && psValues[1]<80){
          			       move(20,20);
          			       if(state==0) {
          			    	   state=1;
          			       }
      			    }else {
        			      move(20.0,-20.0);
        			      if(state==1 && psValues[0]<80 && psValues[7]<80 && psValues[6]<80 && psValues[1]<80){
        			      state=0;
        			      angle=angle-1;
        			      System.out.println(angle);
        			      }
      			    }
      			
      			    
			
			
			
			}
			//si je ne capte pas de mur à gauche et que je suis à 0 alors je vais tout droit
			
			//si je ne capte pas de mur à gauche et que je suis pas à 0 alors je vais à gauche 
			
			//si je suis face a un mur je vais à droite 
			
			//si je vois l'objectif je vais vers lui 
				//si je rencontre un obstacle ET que je vois l'objectif éviter légèrement gauche/eviter droite 
		}
	}
	

	/**
	 * Initialisation of the computation of the relative position of the robot
	 */
	private void initLocalisation() {		
		step(timeStep);
		odometry.track_start_pos(encoder_unit * leftMotorSensor.getValue(), encoder_unit * rightMotorSensor.getValue());
		odometry.setX(0);
		odometry.setY(0);
		odometry.setTheta(Math.PI/2);		
	}
	
	
	/**
	 * To call to compute in real time its own relative position
	 * @param print : true if the relative position must be printed in the console 
	 */
	protected void localise(boolean print) {
		odometry.track_step_pos(encoder_unit * leftMotorSensor.getValue(), encoder_unit * rightMotorSensor.getValue());
		if(print) {
			Double[] pos=getPosition();
			System.out.println("Position : "+pos[0]+","+pos[1]);
		}
	}

	
	/**
	 * Get the computed relative position of the robot (x;y)
	 * The starting point is always (0;0)
	 * @return
	 */
	protected Double[] getPosition() {
		return new Double[] {odometry.getX(),odometry.getY()};
	}
	
	/**
	 * Move towards a position (xObj;yObj) within the relative coordinate system of the robot
	 * @param xObj
	 * @param yObj
	 * @param left power of the left motor
	 * @param right power of the right motor
	 */
	@SuppressWarnings("unused")
	private void moveTowards(Double xObj, Double yObj, double left, double right) {
		double theta_goal=0;
		double eps=0.05;
		boolean error=false;
		if(Math.abs(odometry.getX() - xObj) < eps) {
			if(Math.abs(odometry.getY() - yObj) < eps) {
				error=true;
				System.err.println("Erreur de localisation : ("+odometry.getX()+","+odometry.getY()+") --> ("+xObj+","+yObj+")");
			}
			else if(odometry.getY()>yObj) {
				theta_goal=3*Math.PI/2;
			}
			else {
				theta_goal=Math.PI/2;
			}

		}
		else if(odometry.getX() > xObj) {
			if(Math.abs(odometry.getY() - yObj) < eps) {
				theta_goal=Math.PI;
			}
			else if(odometry.getY()>yObj) {
				theta_goal=(5*Math.PI/4);
			}
			else {
				theta_goal=3*Math.PI/4;
			}
		}
		else {
			if(Math.abs(odometry.getY() - yObj) < eps) {
				theta_goal=0;
			}
			else if(odometry.getY()>yObj) {
				theta_goal=(7*Math.PI/4);
			}
			else {
				theta_goal=Math.PI/4;
			}
		}

		if(!error) {
			double mon_theta=odometry.getTheta();

			if(mon_theta > 3 * Math.PI / 2 && theta_goal < Math.PI / 2)
				theta_goal += 2 * Math.PI;
			else if(mon_theta < Math.PI / 2 && theta_goal > 3 * Math.PI / 2)
				mon_theta += 2 * Math.PI;

			if(Math.abs(mon_theta - theta_goal) > eps) {
				if(mon_theta - theta_goal > theta_goal - mon_theta) {				
					move(left,right/2);
				}
				else {								
					move(left/2,right);
				}
			}
			else {
				move(left,right);
			}
		}
	}

	
	/**
	 * 
	 * @return a double array with values for each IR sensor 
	 * Each value is between approx. [67 ; 750 (very close - contact)]
	 * (see https://cyberbotics.com/doc/guide/epuck)
	 */
	protected double[] readDistanceSensorValues() {
		// read sensors outputs
		double[] psValues = {0, 0, 0, 0, 0, 0, 0, 0};
		for (int i = 0; i < 8 ; i++)
			psValues[i] = distanceSensor[i].getValue();

		return psValues;
	}

	/**
	 * Set powers to left and right motors
	 * @param left : a value between [-100;100]%
	 * @param right : a value between [-100;100]%
	 */
	protected void move(double left, double right) {
		double max=6.2;
		getMotor("left wheel motor").setVelocity(left * max / 100);
		getMotor("right wheel motor").setVelocity(right * max / 100);
	}
	
	/**
	 * Switch on / off a LED according to its num ([0;9])
	 * @param num
	 * @param on : true if the LED is to be switched on, 
	 * or false if the LED is to be switched off
	 */
	protected void setLED(int num, boolean on) {
		if(num < 10) {
			leds[num].set(on ? 1 : 0);
		}
	}

	/**
	 * 
	 * @return an empty list if nothing is detected by the camera, 
	 * a list of CameraRecognitionObject otherwise (see https://cyberbotics.com/doc/reference/camera#camera-recognition-object)
	 */
	protected List<CameraRecognitionObject> cameraDetection() {
		ArrayList<CameraRecognitionObject> detected=new ArrayList<>();
		int nb=camera.getRecognitionNumberOfObjects();
		if(nb >0) {
			CameraRecognitionObject[] objects=camera.getRecognitionObjects();
			for(int i=0;i<objects.length;i++) {
				detected.add(objects[i]);
			}
		}
		return detected;
	}

	/**
	 * Look in a List of camera detected objects if the target is one of them 
	 * @param detected: a List of camera detected objects
	 * @return the target (a specific CameraRecognitionObject) or null
	 */
	protected CameraRecognitionObject targetDetected(List<CameraRecognitionObject> detected) {
		for(CameraRecognitionObject ob:detected) {
			if(ob.getModel().compareTo("cible") == 0)
				return ob;
		}
		return null;		
	}

	/**
	 * Look in a List of camera detected objects if other robots are recognized 
	 * @param detected: a List of camera detected objects
	 * @return a List of CameraRecognitionObject representing the other robots
	 */
	protected List<CameraRecognitionObject> otherRobotsDetected(List<CameraRecognitionObject> detected) {
		ArrayList<CameraRecognitionObject> robots=new ArrayList<>();
		for(CameraRecognitionObject ob:detected) {
			if(ob.getModel().compareTo("e-puck") == 0)
				robots.add(ob);
		}
		return robots;		
	}


	public static void main(String[] args) {
		AutonomyTwo controller = new AutonomyTwo();
		controller.run();
	}
	
	/**
	 * Do NOT modify
	 * Private class providing tools to compute a relative position for the robot
	 */
	private class Odometry{
		private double wheel_distance;
		private double wheel_conversion_left;
		private double wheel_conversion_right;
		private double pos_left_prev;
		private double pos_right_prev;
		private double x;
		private double y;
		private double theta;

		private double increments_per_tour = 1000.0;   // from e-puck.org
		private double axis_wheel_ratio = 1.4134;      // from e-puck.org
		private double wheel_diameter_left = 0.0416;   // from e-puck.org
		private double wheel_diameter_right = 0.0416;  // from e-puck.org
		private double scaling_factor = 0.976;         // default is 1

		public Odometry() {
			// TODO Auto-generated constructor stub
		}

		public int track_start_pos(double pos_left, double pos_right) {
			x=0;
			y=0;
			theta =0;

			pos_left_prev=pos_left;
			pos_right_prev=pos_right;

			wheel_distance = axis_wheel_ratio * scaling_factor * (wheel_diameter_left + wheel_diameter_right) / 2;
			wheel_conversion_left = wheel_diameter_left * scaling_factor * Math.PI / increments_per_tour;
			wheel_conversion_right = wheel_diameter_right * scaling_factor * Math.PI / increments_per_tour;

			return 1;
		}

		public void track_step_pos(double pos_left, double pos_right) {
			double delta_pos_left, delta_pos_right;
			double delta_left, delta_right, delta_theta, theta2;
			double delta_x, delta_y;

			delta_pos_left = pos_left - pos_left_prev;
			delta_pos_right = pos_right - pos_right_prev;
			delta_left = delta_pos_left * wheel_conversion_left;
			delta_right = delta_pos_right * wheel_conversion_right;
			delta_theta = (delta_right - delta_left) / wheel_distance;
			theta2 = theta + delta_theta * 0.5;
			delta_x = (delta_left + delta_right) * 0.5 * Math.cos(theta2);
			delta_y = (delta_left + delta_right) * 0.5 * Math.sin(theta2);

			x += delta_x;
			y += delta_y;
			theta += delta_theta;

			if(theta < 0)
				theta +=2 * Math.PI;
			if(theta > 2 * Math.PI)
				theta -=2 * Math.PI;

			pos_left_prev = pos_left;
			pos_right_prev = pos_right;
		}

		public double getX() {
			return x;
		}

		public void setX(double x) {
			this.x = x;
		}

		public double getY() {
			return y;
		}

		public void setY(double y) {
			this.y = y;
		}

		public double getTheta() {
			return theta;
		}

		public void setTheta(double theta) {
			this.theta = theta;
		}



	}
	
}
