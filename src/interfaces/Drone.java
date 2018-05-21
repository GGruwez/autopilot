package interfaces;

import java.util.ArrayList;

class Drone {

  	
  	private double SIMULATION_PERIOD = 0.01;


  	private boolean ascending = false;
 	private float refHeight = 0;
  	private boolean cruising = true;
  	private boolean descending = false;
  	private boolean turnLeft = false;
  	private boolean turnRight = false;
  	private boolean takeoff = false;
  	private boolean prevtakeoff = false;
  	private boolean landing = false;
  	private boolean taxi = false;
  	private boolean homing = false;
    private float turningDistance = 450;
    private boolean masterOfAirport = false;
    private AutopilotImplementation autopilot;
  	
  	private Airport airport = null;
  	private int gate = 0;
  	private int pointingToRunway = 0;
  	private Vector position = Vector.NULL;
  	private Job job;
  	private Path path;

//  	//testing

//    private Job job = null;
//    private PathImplementation path = null;
	private int reachedTargets = 0;
  	private Vector finalTarget;
	
	
  	public Drone(Airport airport, int gate, int pointingToRunway) {
  		this.airport = airport;
  		this.gate = gate;
  		this.pointingToRunway = pointingToRunway;
  	}
  	
  	public Airport getAirport() {
  		return this.airport;
  	}
  	
  	public int getGate() {
  		return this.gate;
  	}
  	
  	public int getPointingToRunway() {
  		return this.pointingToRunway;
  	}
  	
  	public void setPosition(float x, float y, float z) {
  		this.position = new Vector(x,y,z);
  	}
  	
  	public Vector getPosition() {
  		return this.position;
  	}
  	
  	public Job getJob() {
  		return this.job;
  	}
  	
    public AutopilotOutputsImplementation calculate(AutopilotInputs input, float[] targetVector, int nbColumns, int nbRows, AutopilotImplementation autopilot) {
    	
    	this.autopilot = autopilot;
    	this.job = autopilot.getCurrentJob();
    	Log.println(autopilot.getJobs().indexOf(job));
    	this.path = job.getPath();
    	
    		if (path == null) {
				return new AutopilotOutputsImplementation(0, 0, 0, 0, 0, 0, 0, 0);

    		}

    	Log.println("nb of autopilot jobs: "+ autopilot.getJobs().size());

		float offset = autopilot.getModule().airportWidth/2;
		if (job.getGateTo() == 0){
			offset = -offset;
		}

		Vector nzAxis = new Vector(0,0,-1);
		Vector Airportaxis = new Vector(job.getAirportTo().getCenterToRunway0X(),0,job.getAirportTo().getCenterToRunway0Z());
		float bias = nzAxis.angleBetween(Airportaxis);
		if (airport.getCenterToRunway0X() < 0){
			bias -= Math.PI;
		}


        finalTarget = new Vector((float) (job.getAirportTo().getCenterX() +  offset * Math.cos(bias)) , 1.22f,(float) (job.getAirportTo().getCenterZ() + offset * Math.sin(bias)));
		refHeight = getCurrentPath().getY()[1];
        this.setPosition(input.getX(), input.getY(), input.getZ());
    	
    	 PreviousInputs prev = autopilot.getPreviousInput();
         float dt = -prev.getElapsedTime()+input.getElapsedTime();
         Vector velocityWorld;
         Vector velocityDrone = new Vector(0, 0, -32);
         AutopilotConfig config = autopilot.getConfig();
         Vector positionDrone = new Vector(input.getX(), input.getY(), input.getZ());
         
         velocityWorld = new Vector((input.getX()-prev.getX())/dt,(input.getY()-prev.getY())/dt,(input.getZ()-prev.getZ())/dt);
         velocityDrone = velocityWorld.inverseTransform(prev.getHeading(),prev.getPitch(),prev.getRoll());
         Vector angularVelocity = (new Vector((input.getPitch()-prev.getPitch())/dt,(input.getHeading()-prev.getHeading())/dt,(input.getRoll()-prev.getRoll())/dt));

         Vector nextTarget;
         
         if (reachedTargets >= path.getX().length){
        	 nextTarget = finalTarget;
         }
         else{
         nextTarget = new Vector(path.getX()[reachedTargets],path.getY()[reachedTargets],path.getZ()[reachedTargets]);
         }
         
         if (nextTarget.calculateDistance(new Vector(input.getX(),nextTarget.getY(), input.getZ())) < 30){ //SET TO 5
        	 reachedTargets += 1;
         }
         
         //nextTarget.printVector("next target: "); 
         
    	 float targetHeading = (float) Math.atan2(-(nextTarget.getX() - input.getX()),-(nextTarget.getZ()-input.getZ()));
     	 float currentHeading = (float) (input.getHeading());
     	 float ref = -(targetHeading - currentHeading);

     	 if (input.getY() < refHeight && reachedTargets == 0){
        	 setTakeoff();
         }
         //// START FINAL APROACH
         else if (nextTarget.equals(finalTarget)){

        	 Vector toTarget = finalTarget.subtract(positionDrone);
        	 if (Math.abs(toTarget.getX()) > turningDistance) {
        		 float Ycentercircle = finalTarget.getY()- toTarget.getY()/2;
        		 float Xcentercircle = finalTarget.getX()- toTarget.getX()/2;
        		 for (float i = 0; i < toTarget.getX(); i+= toTarget.getX()/30) {
        			 float x = i;
        		 }
        	 }
        		 
        	 
        	 
        	 
         	 
         	 
         	 if (input.getY() < 1.5 && velocityDrone.getZ() > -15){
         		 setTaxi();
         	 }
         	 else if (Math.abs(ref) < 0.03f || Math.abs(ref) > Math.PI*2 + 0.03f || landing){
         		 setLanding();
         	 }
         	 else if ((ref > 0.f && ref  < Math.PI) || (ref < -0f && ref < -Math.PI)) {
         		//turnright
         		//Log.println("right");
         		if (Math.abs(ref) > 0.03f && checkIfReachable(input, nextTarget)) {//Math.abs(ref) > 0.1f && 
         			setTurnRight();
         			
         		}else{
         			setCruising(refHeight);
         		}
      		
         	}
         	else if ((ref >= 0.f && ref >= Math.PI) || (ref <= 0f && ref >= -Math.PI)) {
         		//turnleft
         		//Log.println("left");
         		if ( Math.abs(ref) > 0.03f && checkIfReachable(input, nextTarget)) {//Math.abs(ref) > 0.1f&&
         			setTurnLeft();
      		
         		}else{
         			setCruising(refHeight);
         		}
         	}
         	 
         }
         //// END FINAL APROACH
         
         //// START HOMING at 150m
         else if (targetVector != null && nextTarget.calculateDistance(new Vector(input.getX(),input.getY(), input.getZ())) < 200 && Math.abs(ref) < 0.1) {
        	setHoming();
         }
         //// END HOMING
         else if ((ref > 0.f && ref  < Math.PI) || (ref < -0f && ref < -Math.PI)) {
     		//turnright
     		//Log.println("right");
     		if (Math.abs(ref) > 0.1f) { //&& checkIfReachable(input, nextTarget)) {//Math.abs(ref) > 0.1f && 
     			setTurnRight();
     		
         	}else if (input.getY() - nextTarget.getY() > 10 && Math.abs(input.getRoll()) < 0.1 ){
         		
         			//setDescending();
         		
         	}else if (input.getY() - nextTarget.getY() < -10 && Math.abs(input.getRoll()) < 0.1 ){
         			//setAscending();
         		
         	}else{
         		setCruising(nextTarget.getY());
         	}
     		
     	}
     	else if ((ref > 0.f && ref > Math.PI) || (ref < 0f && ref > -Math.PI)) {
     		//turnleft
     		//Log.println("left");
     		if ( Math.abs(ref) > 0.1f ) {//&& checkIfReachable(input, nextTarget)) {//Math.abs(ref) > 0.1f&&
     			setTurnLeft();
     		
         	}else if (input.getY() - nextTarget.getY() > 10 && Math.abs(input.getRoll()) < 0.1  ){
         			//setDescending();
         		
         	}else if (input.getY() - nextTarget.getY() < -10 && Math.abs(input.getRoll()) < 0.1 ){
         			//setAscending();
         	
         	}else{
         		setCruising(nextTarget.getY());
         	}
     	}

     	else{
     		setCruising(nextTarget.getY());
     	}
         
        
         
         //TESTING
//         if (!takeoff && input.getElapsedTime() <25)
//         setCruising(nextTarget.getY());
//         else if (input.getElapsedTime() >25)
//         setLanding();
         //
   
         int temp;
		 if (reachedTargets < path.getY().length) temp = reachedTargets; else temp =  reachedTargets-1;
         refHeight = path.getY()[0];


		 AutopilotOutputsImplementation output = null;
		 if (cruising) {
			 output = cruising(input, velocityDrone, velocityWorld, config);
		 }
		 else if (homing) {
			 output = homing(input, velocityDrone,velocityWorld, config, targetVector, nextTarget);
		 }
		 else if (landing) {
			 output = landing(input, velocityDrone, velocityWorld, config);
		 }
		 else if (ascending) {
			 output = ascending(input, velocityDrone, velocityWorld, config);
		 }
		 else if (descending) {
			 output = descending(input, velocityDrone, velocityWorld, config);
		 }
		 else if (takeoff) {
			 output = takeoff(input, velocityDrone, velocityWorld, config);
		 }else if (taxi) {
			 output = taxi(input, velocityDrone, velocityWorld, config, autopilot);
		 }
		 if (turnLeft) {
			 output = turnLeft(input, velocityDrone, velocityWorld, config,output);
		 }
		 if (turnRight) {
			 output = turnRight(input, velocityDrone, velocityWorld, config,output);
		 }
		 
		 // Reservatie opstijgbaan
		 if (takeoff && getAirport().takeOffTaken() && ! masterOfAirport) {
			 output = new AutopilotOutputsImplementation(0, 0, 0, 0, 0, 0, 0, 0);
		 }
		 else if (takeoff && (! getAirport().takeOffTaken() || masterOfAirport)) {
			 this.getAirport().setTakeOffTaken(true);
			 this.masterOfAirport = true;
		 }
		 else {
			 this.getAirport().setTakeOffTaken(false);
			 this.masterOfAirport = false;
		 }
		 
		 // Reservatie landingsbaan
		 if (takeoff) {
			 if (this.getJob().getAirportTo().hasDroneAt(this.getJob().getGateTo())
					 && this.getJob().getAirportTo().getDroneAt(this.getJob().getGateTo()).getDrone() != this) {
				 output = new AutopilotOutputsImplementation(0, 0, 0, 0, 0, 0, 0, 0);
			 }
			 else {
				 this.getJob().getAirportTo().setDroneAt(this.getJob().getGateTo(), this.autopilot);
				 this.getJob().getAirportFrom().setDroneAt(this.getJob().getGateFrom(), null);
			 }
		 }
		 
		 // cap outputs
		 float curThrust = output.getThrust();
		 if (nextTarget.equals(finalTarget) && finalTarget.calculateDistance(positionDrone) < 300 && positionDrone.getY() > 15)
			 curThrust /= 1.75;
		 if (curThrust > config.getMaxThrust())
		 	curThrust = config.getMaxThrust();
		 
		 output = new AutopilotOutputsImplementation(curThrust,
				 									capInclination(velocityDrone, config, output.getLeftWingInclination()),
				 									capInclination(velocityDrone, config, output.getRightWingInclination()),
				 									output.getHorStabInclination(),				
				 									output.getVerStabInclination(),
				 									output.getFrontBrakeForce(),
				 									output.getLeftBrakeForce(),
				 									output.getRightBrakeForce());
		 
		 
		//
		 
		 return output;
    }
    		
 
    public  AutopilotOutputsImplementation cruising(AutopilotInputs input, Vector velocityDrone, Vector velocityWorld, AutopilotConfig config) {
    	float horStabInclination;
    	float rightWingInclination;
    	float leftWingInclination;
    	float thrust = 0;
    	
    	if (prevtakeoff && input.getPitch() > 0.02) {
    		horStabInclination = -config.getMaxAOA();
    	}
    	else {
      	horStabInclination = -config.getMaxAOA()/4;
    	prevtakeoff = false;
    	}

    	
       	if (input.getPitch() > 0.01f) {
       		horStabInclination = -input.getPitch()*1.5f;
       	}
       	else if (input.getPitch() < -0.01f) {
       		horStabInclination = -input.getPitch()*4f;
       	}
    	
    	float currentProjAirspeed = (float) -Math.atan2(velocityDrone.getY(),-velocityDrone.getZ());
      	rightWingInclination = (float) (-currentProjAirspeed+0.9*config.getMaxAOA());
      	if (prevtakeoff)
      		rightWingInclination /=2;
      	//Log.println("inclination: "+ rightWingInclination);
      	leftWingInclination = rightWingInclination;
    	 
     	float gravityToCompensate = (config.getEngineMass()+config.getTailMass()+2*config.getWingMass())*config.getGravity();
     	float horLift = (float) (config.getHorStabLiftSlope()*velocityDrone.dotProduct(velocityDrone)*(currentProjAirspeed+horStabInclination)*Math.cos(horStabInclination));
     	float cancelY;
 		cancelY = (float) (-velocityWorld.getY()/SIMULATION_PERIOD)*(config.getEngineMass()+2*config.getWingMass()+config.getTailMass());
     	float forceToCompensate = gravityToCompensate - horLift + cancelY;
     	Vector lift = (new Vector(0, (float) ((2*config.getWingLiftSlope()*config.getMaxAOA()*0.9)*Math.cos(rightWingInclination)), 0)).inverseTransform(
     			input.getHeading(), input.getPitch(), input.getRoll());
     	float minSpeed = (float) Math.sqrt((forceToCompensate)/lift.getY());
     	//Log.println("minSpeed: "+ minSpeed);


     	if (-velocityDrone.getZ() < minSpeed) {
     		double acceleration = (minSpeed - Math.abs(velocityDrone.getZ()))/SIMULATION_PERIOD;
     		thrust = (float) (acceleration*(config.getEngineMass()+config.getTailMass()+2*config.getWingMass()));
     	}
//
//		thrust = 1000;
//		//horStabInclination = config.getMaxAOA()*0.4f;
//		if (velocityWorld.getY() > 0.0f && position.getY() > refHeight + 1) {
//			rightWingInclination += 0.02 * config.getMaxAOA();
//			leftWingInclination = rightWingInclination;
//		//	thrust += 150;
//		}
//		if (velocityWorld.getY() < 0.0f  && position.getY() < refHeight -1){
//			rightWingInclination -= 0.02 * config.getMaxAOA();
//			leftWingInclination = rightWingInclination;
//			thrust -= 100;
//		}



		if  (!(turnLeft || turnRight)) {
     		float deltaroll = 0.02f;//(float) 0.02*config.getMaxAOA();
     		if (input.getRoll() > 0.005f){//0.01f
     			rightWingInclination -= deltaroll*6;
     			leftWingInclination += deltaroll;
       	 } else if (input.getRoll() < -0.005f) {
       		rightWingInclination += deltaroll;
 			leftWingInclination -= deltaroll*6;
       	 }
     	}
     	
     	//Log.println("-------------------------------------------------");
     	
     
     	
     	return new AutopilotOutputsImplementation(thrust, leftWingInclination, rightWingInclination, -horStabInclination, 0, 0, 0, 0);
     }

	public  AutopilotOutputsImplementation landing(AutopilotInputs input, Vector velocityDrone, Vector velocityWorld, AutopilotConfig config) {
		float horStabInclination;
		float rightWingInclination;
		float leftWingInclination;
		float thrust = 0;
		float frontBrake = 0;
		float leftBrake = 0;
		float rightBrake = 0;


		horStabInclination = -config.getMaxAOA()/4;
		if (input.getPitch() > 0.03f) {
			horStabInclination = -input.getPitch();
		}
		else if (input.getPitch() < -0.01f) {
			horStabInclination = config.getMaxAOA()*0.6f;
		}

		float currentProjAirspeed = (float) -Math.atan2(velocityDrone.getY(),-velocityDrone.getZ());
		rightWingInclination = (float) (-currentProjAirspeed+0.9*config.getMaxAOA());

		leftWingInclination = rightWingInclination;

		if (velocityDrone.getY() < -3)
			thrust = 1000;

		if (input.getY() < 2) {
			frontBrake = config.getRMax();
			leftBrake = config.getRMax()/1.5f;
			rightBrake = config.getRMax()/1.5f;
			rightWingInclination = (float) (-currentProjAirspeed+0.3*config.getMaxAOA());
			leftWingInclination =(float) (-currentProjAirspeed+0.3*config.getMaxAOA());
		}

		float deltaroll = 0.02f;//(float) 0.02*config.getMaxAOA();
		if (input.getRoll() > 0.05f){//0.01f
			rightWingInclination -= deltaroll*6;
			leftWingInclination += deltaroll;
		} else if (input.getRoll() < -0.05f) {
			rightWingInclination += deltaroll;
			leftWingInclination -= deltaroll*6;

		}

		//System.out.println("desc hstab: " +horStabInclination);
		if (position.getY() > 5) {
			Vector destination = finalTarget;
			float targetHeading = (float) Math.atan2((destination.getX() - input.getX()), (destination.getZ() - input.getZ()));
			float currentHeading = (float) (input.getHeading());
			float ref = targetHeading - currentHeading;

			if ((ref > 0 && ref < Math.PI) || (ref < 0 && ref < -Math.PI)) {
				//turnright
				//System.out.println("right");
				if (Math.abs(ref) > 0.03f) {
					if (input.getRoll() > 0.07f) {//0.01f
						rightWingInclination -= deltaroll * 6;
						leftWingInclination += deltaroll;
					} else if (input.getRoll() < 0.07f) {
						rightWingInclination += deltaroll;
						leftWingInclination -= deltaroll * 6;
					}
				}

			} else if ((ref > 0 && ref > Math.PI) || (ref < 0 && ref > -Math.PI)) {
				//turnleft
				//System.out.println("left");
				if (Math.abs(ref) > 0.03f) {
					if (input.getRoll() > -0.07f) {//0.01f
						rightWingInclination -= deltaroll * 6;
						leftWingInclination += deltaroll;
					} else if (input.getRoll() < -0.07f) {
						rightWingInclination += deltaroll;
						leftWingInclination -= deltaroll * 6;
					}
				}
			}
		}
		return new AutopilotOutputsImplementation(thrust, leftWingInclination, rightWingInclination, -horStabInclination, 0, frontBrake, leftBrake, rightBrake);
	}




    public  AutopilotOutputsImplementation takeoff(AutopilotInputs input, Vector velocityDrone, Vector velocityWorld, AutopilotConfig config) {
    	float horStabInclination;
    	float rightWingInclination;
    	float leftWingInclination;
    	float leftBrake = 0;
    	float rightBrake = 0;
    	float thrust = 0;

    	thrust = config.getMaxThrust();
   	 	horStabInclination = 0;
    	
    	if (input.getPitch() > 0.08f) {
    		horStabInclination = -input.getPitch();
    	}
		Vector destination = new Vector(path.getX()[reachedTargets],path.getY()[reachedTargets],path.getZ()[reachedTargets]);
		float targetHeading = (float) Math.atan2((destination.getX() - input.getX()),(destination.getZ()-input.getZ()));
		float currentHeading = (float) (input.getHeading());
		float ref = targetHeading - currentHeading;

		Log.println("ref" + ref);

		if ((ref > 0 && ref < Math.PI) || (ref < 0 && ref < -Math.PI)) {
			//turnright
			//Log.println("right");
			if (Math.abs(ref) > 0.05f) {
				rightBrake = config.getRMax()/10;
			}

		}
		else if ((ref > 0 && ref > Math.PI) || (ref < 0 && ref > -Math.PI)) {
			//turnleft
			//Log.println("left");
			if (Math.abs(ref) > 0.05f) {
				leftBrake = config.getRMax()/10;
			}
		}
    	
    	float currentProjAirspeed = (float) -Math.atan2(velocityDrone.getY(),-velocityDrone.getZ());
     	rightWingInclination = (float) (-currentProjAirspeed+0.9*config.getMaxAOA());
    	leftWingInclination = rightWingInclination;
    	prevtakeoff = true;
    	
    	return new AutopilotOutputsImplementation(thrust, leftWingInclination, rightWingInclination, -horStabInclination, 0, 0, leftBrake, rightBrake);
    }

    public  AutopilotOutputsImplementation ascending(AutopilotInputs input, Vector velocityDrone, Vector velocityWorld, AutopilotConfig config) {
    	float horStabInclination;
    	float rightWingInclination;
    	float leftWingInclination;
    	float thrust = 0;
    	
    	horStabInclination = 0;
    	
    	if (input.getPitch() > 0.15f) {
    		horStabInclination = -input.getPitch();
    	}

    	float currentProjAirspeed = (float) -Math.atan2(velocityDrone.getY(),-velocityDrone.getZ());
       	rightWingInclination = (float) (-currentProjAirspeed+0.9*config.getMaxAOA());
       	leftWingInclination = rightWingInclination;
     	
      	if (velocityWorld.getY() < 3) {
     		double acceleration = 1.4*(3-velocityDrone.getY());
     		thrust = (float) (acceleration*(config.getEngineMass()+config.getTailMass()+2*config.getWingMass()));
     	}
      	else {
      		thrust = 0;
      	}
      	
      	//Log.println("ascending - thrust: " + thrust);
      	//Log.println("-----------------------------------------");
      	
    	return new AutopilotOutputsImplementation(thrust, leftWingInclination, rightWingInclination, -horStabInclination, 0, 0, 0, 0);
    }

    public  AutopilotOutputsImplementation descending(AutopilotInputs input, Vector velocityDrone, Vector velocityWorld, AutopilotConfig config) {
    	float horStabInclination;
    	float rightWingInclination;
    	float leftWingInclination;
    	float thrust = 0;
    	
    	//Log.println("descending");
   	 
    	if (input.getPitch() < 0.0f) {
    		horStabInclination = -input.getPitch()*3;
    	}
    	else {
    		horStabInclination = -input.getPitch()*3;
    	}

    	float currentProjAirspeed = (float) -Math.atan2(velocityDrone.getY(),-velocityDrone.getZ());
       	rightWingInclination = (float) (-currentProjAirspeed+0.8*config.getMaxAOA());

       	leftWingInclination = rightWingInclination;
       	
       	thrust = 0;
       	
    	return new AutopilotOutputsImplementation(thrust, leftWingInclination, rightWingInclination, -horStabInclination, 0, 0, 0, 0);
    }

    public  AutopilotOutputsImplementation taxi(AutopilotInputs input, Vector velocityDrone, Vector velocityWorld, AutopilotConfig config, AutopilotImplementation autopilotImplementation) {
    	float horStabInclination = 0;
    	float rightWingInclination = 0;
    	float leftWingInclination = 0;
    	float thrust = 0;
    	float frontBrake = 0;
    	float leftBrake = 0;
    	float rightBrake = 0;
    	Vector destination = this.finalTarget;
    	
    	float targetHeading = (float) Math.atan2((destination.getX() - input.getX()),(destination.getZ()-input.getZ()));
    	targetHeading += Math.PI;
    	float currentHeading = (float) (input.getHeading() + Math.PI);
    	float ref = targetHeading - currentHeading;
    	//Log.println("ref" + ref);
    	
    	if ((ref > 0 && ref < Math.PI) || (ref < 0 && ref < -Math.PI)) {
    		//turnright
    		//Log.println("right");
    		if (Math.abs(ref) > 0.01f) {
    			rightBrake = config.getRMax()/2;
    		}
    		
    	}
    	else if ((ref > 0 && ref > Math.PI) || (ref < 0 && ref > -Math.PI)) {
    		//turnleft
    		//Log.println("left");
    		if (Math.abs(ref) > 0.01f) {
    			leftBrake = config.getRMax()/2;
    		}	
    	}
    	if (velocityDrone.getZ() > -10) {
    		thrust = config.getMaxThrust();
    	} else {
    		frontBrake = config.getRMax()/2;
    		leftBrake = config.getRMax()/2;
    		rightBrake = config.getRMax()/2;
    	}
    	//velocityDrone.printVector("vroom");
    	
    	float dist = (float) Math.sqrt(Math.pow(destination.getX() - input.getX(),2) + Math.pow(destination.getZ()-input.getZ(),2));
    	//Log.println("dist: " + dist);
    	if (dist < 100) {
    		float t = 5;
    		float totalmass = config.getEngineMass() + config.getTailMass() + 2* config.getWingMass();
    		float totalBrakeForce = (2*totalmass/(t*t))*(dist - Math.abs(velocityDrone.getZ())*t);
    		if (totalBrakeForce < 0) {
    			//Log.println("brake brake brake: " + totalBrakeForce );
    			rightBrake = Math.abs(totalBrakeForce)/2;
    			leftBrake = Math.abs(totalBrakeForce)/2;
    			thrust = 0;
    			
    			
    		}
    		if (dist < 0.5f) {
    			frontBrake = config.getRMax();
        		leftBrake = config.getRMax();
        		rightBrake = config.getRMax();
        		thrust = 0;
        		if (Math.abs(velocityDrone.getZ()) < 0.1) {
					autopilotImplementation.finishCurrentJob();
					reachedTargets = 0;
        		}
        		
    		}
    		
    		
    		if ((ref > 0 && ref < Math.PI) || (ref < 0 && ref < -Math.PI)) {
        		//turnright
        		//Log.println("right");
        		if (Math.abs(ref) > 0.01f) {
        			rightBrake += config.getRMax()/6;
        		}
        		
        	}
        	else if ((ref > 0 && ref > Math.PI) || (ref < 0 && ref > -Math.PI)) {
        		//turnleft
        		//Log.println("left");
        		if (Math.abs(ref) > 0.01f) {
        			leftBrake += config.getRMax()/6;
        		}	
        	}
    	}
    	
    	return new AutopilotOutputsImplementation(thrust, leftWingInclination, rightWingInclination, -horStabInclination, 0, frontBrake, leftBrake, rightBrake);
    }

    public  AutopilotOutputsImplementation turnLeft(AutopilotInputs input, Vector velocityDrone, Vector velocityWorld, AutopilotConfig config, AutopilotOutputsImplementation output) {
    	float horStabInclination = output.getHorStabInclination();
    	float rightWingInclination = output.getRightWingInclination();
    	float leftWingInclination = output.getLeftWingInclination();
    	float thrust = output.getThrust();
    	
    	float deltaroll = 0.01f;//0.001f
    	
    	if (input.getRoll() > 0.25f){//0.01f
    		rightWingInclination-= 6*deltaroll;// = getMininclination(velocityDrone, config, rightWingInclination)+0.04f;//
    		leftWingInclination += deltaroll;//= getMaxinclination(velocityDrone, config, leftWingInclination)-0.04f;//
    	} else {
    		rightWingInclination += deltaroll;//= getMaxinclination(velocityDrone, config, rightWingInclination)/4;//
        	leftWingInclination -= 6*deltaroll;//= getMininclination(velocityDrone, config, leftWingInclination)/4;//
    	}

		thrust =1000;
		if (velocityWorld.getY() > 0.0f && position.getY() > refHeight )
			thrust =850;
		if (velocityWorld.getY() < 0.0f  && position.getY() < refHeight )
			thrust = 1100;
//    	//System.out.print("turnleft");
//		if (input.getY() < refHeight - 1) {
//			thrust *= 1.5;
//		}
//		else if ((input.getY() > refHeight + 1)) {
//			thrust  = 750;
//		}
		if (position.getY() > refHeight + 10){
			thrust /= 1.5f;
		}


		horStabInclination = 0.05f;
   	 	if (input.getPitch() > 0.15f)
   	 		horStabInclination = -input.getPitch();
   	 	if (input.getPitch() < 0.04)
   	 		horStabInclination *= 3;
   	 	return new AutopilotOutputsImplementation(thrust, leftWingInclination, rightWingInclination, -horStabInclination, 0, 0, 0, 0);
    	}

    public  AutopilotOutputsImplementation turnRight(AutopilotInputs input, Vector velocityDrone, Vector velocityWorld, AutopilotConfig config, AutopilotOutputsImplementation output) {
    	float horStabInclination = output.getHorStabInclination();
    	float rightWingInclination = output.getRightWingInclination();
    	float leftWingInclination = output.getLeftWingInclination();
    	float thrust = output.getThrust();
    	
    	float deltaroll = 0.01f;//0.001f
   	 	
   	 	if (input.getRoll() < -0.25f){//0.01f
   	 		rightWingInclination += deltaroll; //getMaxinclination(velocityDrone, config, rightWingInclination)-0.04f;//
   	 		leftWingInclination -= 6*deltaroll; //getMininclination(velocityDrone, config, leftWingInclination)+0.04f;//
   	 	} else {
   	 		rightWingInclination  -= deltaroll*6;//getMininclination(velocityDrone, config, rightWingInclination)/4; //
   	 		leftWingInclination += deltaroll; //getMaxinclination(velocityDrone, config, leftWingInclination)/4;//
   	 	}







        thrust = 1000;
        if (velocityWorld.getY() > 0.0f && position.getY() > refHeight )
            thrust = 850;
        if (velocityWorld.getY() < 0.0f  && position.getY() < refHeight)
            thrust = 1100;

        if (position.getY() > refHeight + 10){
        	thrust /= 1.5f;
		}
		//System.out.print("turnleft");

//		if (input.getY() < refHeight - 1) {
//			thrust *= 1.5;
//		}
//		else if ((input.getY() > refHeight + 1)) {
//			thrust  = 750;
//		}



    	

   	 
   	 
   	 	horStabInclination = 0.05f;
   	 	if (input.getPitch() > 0.15f)
   	 		horStabInclination = -input.getPitch();
   	 	if (input.getPitch() < 0.04)
   	 		horStabInclination *= 3;
   	 	

   	 	
   	 	return new AutopilotOutputsImplementation(thrust, leftWingInclination, rightWingInclination, -horStabInclination, 0, 0, 0, 0);
    	}
    
    
    public  AutopilotOutputsImplementation homing(AutopilotInputs input, Vector velocityDrone, Vector velocityWorld, AutopilotConfig config, float[] targetVector, Vector nextTarget) {
    	float horStabInclination = 0;
    	float rightWingInclination = 0;
    	float leftWingInclination = 0;
    	float thrust = 0;
    	float currentProjAirspeed = (float) -Math.atan2(velocityDrone.getY(),-velocityDrone.getZ());
       	rightWingInclination = (float) (-currentProjAirspeed+0.9*config.getMaxAOA());
       	leftWingInclination = rightWingInclination;
    	
       	if (velocityWorld.getY() < -0.1f) {
       		thrust = config.getMaxThrust();
       	}else if (velocityWorld.getY() > 0.1f) {
       		thrust = 0;
       	}
       	
       	
     	if (input.getPitch() > 0.01f) {
       		horStabInclination = -input.getPitch()*1.5f;
       	}
       	else if (input.getPitch() < -0.01f) {
       		horStabInclination = -input.getPitch()*4f;
       	}
       	
    	float xError = (float)(-targetVector[0] * Math.cos(input.getRoll()) - targetVector[1] * Math.sin(input.getRoll()));
    	float yError = (float)(-targetVector[0] * Math.sin(input.getRoll()) - targetVector[1] * Math.cos(input.getRoll()));
    	//Log.println("xError: " + xError);
    	//Log.println("yError: " + yError);
    	
    	if (Math.abs(yError) > Math.abs(xError)) {
    		
    		if (yError < -5) {
    			
    			rightWingInclination = (float) (-currentProjAirspeed+0.3*config.getMaxAOA());
    	       	leftWingInclination = rightWingInclination;
    	       	thrust = config.getMaxThrust()/4;
    		}else if (yError > 10 ) {
    			rightWingInclination = (float) (-currentProjAirspeed+0.8*config.getMaxAOA());//0.8
    	       	leftWingInclination = rightWingInclination;
    	       	thrust = config.getMaxThrust()*.75f;//*.75f;
    		}
    	}else {
    		if (xError > 10) {
    			float deltaroll = 0.01f;
    			if (input.getRoll() > 0.15f){
    	   	 		rightWingInclination += deltaroll;
    	   	 		leftWingInclination -= 6*deltaroll;
    	   	 	} else {
    	   	 		rightWingInclination  -= deltaroll*6;
    	   	 		leftWingInclination += deltaroll; 
    	   	 	}
    		
    		}else if (xError < -10){
    			float deltaroll = 0.01f;
    			if (input.getRoll() < -0.15f){
    	   	 		rightWingInclination += deltaroll;
    	   	 		leftWingInclination -= 6*deltaroll;
    	   	 	} else {
    	   	 		rightWingInclination  -= deltaroll*6;
    	   	 		leftWingInclination += deltaroll; 
    	   	 	}
    		}else {
    			float deltaroll = 0.02f;//(float) 0.02*config.getMaxAOA();
         		if (input.getRoll() > 0.005f){//0.01f
         			rightWingInclination -= deltaroll*6;
         			leftWingInclination += deltaroll;
         		} else if (input.getRoll() < -0.005f) {
         			rightWingInclination += deltaroll;
         			leftWingInclination -= deltaroll*6;
         		}
    		}
    	}
    	
   	 	return new AutopilotOutputsImplementation(thrust, leftWingInclination, rightWingInclination, -horStabInclination, 0, 0, 0, 0);
    }
    
    public   void setCruising(float height) {
    	cruising = true;
    	landing = false;
    	takeoff = false;
    	ascending = false;
    	descending = false;
    	taxi = false;
    	turnLeft = false;
    	turnRight = false;
    	homing= false;
    }
    
    public   void setLanding() {
    	Log.println("landing");
    	cruising = false;
    	landing = true;
    	takeoff = false;
    	ascending = false;
    	descending = false;
    	taxi = false;
    	turnLeft = false;
    	turnRight = false;
    	homing= false;
  }
    
    public   void setTakeoff() {
    	Log.println("take off");
    	cruising = false;
    	landing = false;
    	takeoff = true;
    	ascending = false;
    	descending = false;
    	taxi = false;
    	turnLeft = false;
    	turnRight = false;
    	homing= false;
   }
    
    public   void setAscending() {
    	cruising = false;
    	landing = false;
    	takeoff = false;
    	ascending = true;
    	descending = false;
    	taxi = false;
    	turnLeft = false;
    	turnRight = false;
    	homing= false;
  }
    
    public   void setDescending() {
    	cruising = false;
    	landing = false;
    	takeoff = false;
    	ascending = false;
    	descending = true;
    	taxi = false;
    	turnLeft = false;
    	turnRight = false;
    	homing= false;
   }
    
    public   void setTaxi() {
    	cruising = false;
    	landing = false;
    	takeoff = false;
    	ascending = false;
    	descending = false;
    	taxi = true;
    	turnLeft = false;
    	turnRight = false;
    	homing= false;
  }
    
    public   void setTurnLeft() {
    	cruising = true;
    	landing = false;
    	takeoff = false;
    	ascending = false;
    	descending = false;
    	taxi = false;
    	turnLeft = true;
    	turnRight = false;
    	homing= false;
   }
    
    public   void setTurnRight() {
    	cruising = true;
    	landing = false;
    	takeoff = false;
    	ascending = false;
    	descending = false;
    	taxi = false;
    	turnLeft = false;
    	turnRight = true;
    	homing= false;
    }

    public   void setHoming() {
    	cruising = false;
    	landing = false;
    	takeoff = false;
    	ascending = false;
    	descending = false;
    	taxi = false;
    	turnLeft = false;
    	turnRight = false;
    	homing = true;
    }
    

    public   float getMaxinclination(Vector velocityDrone, AutopilotConfig config, float startincl) {
    	float incl = startincl;
    	Vector axisVector = new Vector(1,0,0);
    	Vector projectedAirspeed = new Vector(0,velocityDrone.getY(), velocityDrone.getZ());
    	Vector attackVector = new Vector(0, (float) Math.sin(incl),(float) -Math.cos(incl));
    	Vector normalVector = axisVector.crossProduct(attackVector);
    	
    	double angle = -Math.atan2(normalVector.dotProduct(projectedAirspeed), attackVector.dotProduct(projectedAirspeed) );
    	while (angle < config.getMaxAOA()-0.01) {
    		incl += 0.01;
    		attackVector = new Vector(0, (float) Math.sin(incl),(float) -Math.cos(incl));
        	normalVector = axisVector.crossProduct(attackVector);
        	angle = -Math.atan2(normalVector.dotProduct(projectedAirspeed), attackVector.dotProduct(projectedAirspeed) );
    	}
    	if (angle < config.getMaxAOA()) {
    		return incl;
    	}else {
    		return (float) (incl - 0.01);
    	}
    }
    
    public   float getMininclination(Vector velocityDrone, AutopilotConfig config,float startincl) {
    	float incl = startincl;
    	Vector axisVector = new Vector(1,0,0);
    	Vector projectedAirspeed = new Vector(0,velocityDrone.getY(), velocityDrone.getZ());
    	Vector attackVector = new Vector(0, (float) Math.sin(incl),(float) -Math.cos(incl));
    	Vector normalVector = axisVector.crossProduct(attackVector);
    	
    	double angle = -Math.atan2(normalVector.dotProduct(projectedAirspeed), attackVector.dotProduct(projectedAirspeed) );
    	while (angle > -config.getMaxAOA()+0.01) {
    		incl -= 0.01;
    		attackVector = new Vector(0, (float) Math.sin(incl),(float) -Math.cos(incl));
        	normalVector = axisVector.crossProduct(attackVector);
        	angle = -Math.atan2(normalVector.dotProduct(projectedAirspeed), attackVector.dotProduct(projectedAirspeed) );
    	}
    	if (angle > -config.getMaxAOA()) {
    		return incl;
    	}else {
    		return (float) (incl + 0.01);
    	}
    }
	
	public   float capInclination(Vector velocityDrone, AutopilotConfig config,float startincl) {

		float incl = startincl;
		//Log.println("starting incl: " + incl);
    	Vector axisVector = new Vector(1,0,0);
    	Vector projectedAirspeed = new Vector(0,velocityDrone.getY(), velocityDrone.getZ());
    	Vector attackVector = new Vector(0, (float) Math.sin(incl),(float) -Math.cos(incl));
    	Vector normalVector = axisVector.crossProduct(attackVector);
    	double checkangle = 0;
    	double angle = -Math.atan2(normalVector.dotProduct(projectedAirspeed), attackVector.dotProduct(projectedAirspeed) );
    	while (angle > config.getMaxAOA()*0.85) {
    		incl -= 0.01;
    		checkangle = angle;
    		attackVector = new Vector(0, (float) Math.sin(incl),(float) -Math.cos(incl));
        	normalVector = axisVector.crossProduct(attackVector);
        	angle = -Math.atan2(normalVector.dotProduct(projectedAirspeed), attackVector.dotProduct(projectedAirspeed) );
//        	if (checkangle < angle) {
//        		incl += 0.02;
//        		attackVector = new Vector(0, (float) Math.sin(incl),(float) -Math.cos(incl));
//            	normalVector = axisVector.crossProduct(attackVector);
//            	angle = -Math.atan2(normalVector.dotProduct(projectedAirspeed), attackVector.dotProduct(projectedAirspeed) );
//        	}
    	}
    	while (angle < -config.getMaxAOA()*.85) {
    		incl += 0.01;
    		checkangle = angle;
    		attackVector = new Vector(0, (float) Math.sin(incl),(float) -Math.cos(incl));
        	normalVector = axisVector.crossProduct(attackVector);
        	angle = -Math.atan2(normalVector.dotProduct(projectedAirspeed), attackVector.dotProduct(projectedAirspeed) );
//        	if (checkangle < angle) {
//        		incl -= 0.02;
//        		attackVector = new Vector(0, (float) Math.sin(incl),(float) -Math.cos(incl));
//            	normalVector = axisVector.crossProduct(attackVector);
//            	angle = -Math.atan2(normalVector.dotProduct(projectedAirspeed), attackVector.dotProduct(projectedAirspeed) );
//        	}
    	}
//    	if (incl < 0)
//    		incl += 0.1f;
    	if (angle < config.getMaxAOA()*.85 && angle > -config.getMaxAOA()*.85) {
    	//	Log.println("calc AOA: " + angle);
    	//	Log.println("calc incl " + incl);
    		return incl;
    	}else if (angle > config.getMaxAOA()*.85){
    	//	Log.println("oops: " + angle);
    	//	Log.println("oopsincl : " + incl);
    		return (float) (incl - 0.01);
    	}else {
    		return (float) (incl +0.01);
    	}
	}

	public   boolean checkIfReachable(AutopilotInputs input, Vector nextTarget) {
		Vector center1 = new Vector((float)(input.getX() + 480*Math.cos(input.getHeading())),0,(float)(input.getZ() - 480*Math.sin(input.getHeading())));
		Vector center2 = new Vector((float)(input.getX() - 480*Math.cos(input.getHeading())),0,(float)(input.getZ() + 480*Math.sin(input.getHeading())));
		float dist1 = (float) Math.sqrt(Math.pow(nextTarget.getX() - center1.getX(),2) + Math.pow(nextTarget.getZ() - center1.getZ(),2) );
		float dist2 = (float) Math.sqrt(Math.pow(nextTarget.getX() - center2.getX(),2) + Math.pow(nextTarget.getZ() - center2.getZ(),2) );
		Log.println("dist1: " + dist1 + " dist2: " + dist2);
		return (dist1 > 460 && dist2 > 460);
	}
	
//	public void setCurrentPath(ArrayList<Vector> path) {
//	//	this.path = new PathImplementation(path);
//	}
	
	public Path getCurrentPath() {
		return this.path;
	}

	public void setAirport(Airport airport){
  	    this.airport = airport;
    }
    public void setGate(int gate){
  	    this.gate = gate;
    }

}

