 package interfaces;


class InputToOutput {
 
 	static PIDcontroller PitchControllerxx = new PIDcontroller(4.5f, 0f, 15f);
  	static PIDcontroller HeightController = new PIDcontroller(0.1f, 0f, 0.02f);


  	static PIDcontroller RollController = new PIDcontroller(0.3f, 0f, 12f);
  	static PIDcontroller HorizontalController = new PIDcontroller(0.1f, 0f, 7f);
  	
  	static PIDcontroller SpeedController = new PIDcontroller(5f, 0, 20f); 
  	
  	static PIDcontroller PitchControllerTurning = new PIDcontroller(4.5f, 0f, 15f);
  	static PIDcontroller RollControllerTurning = new PIDcontroller(0.3f, 0f, 12f);
  	static PIDcontroller HeadingController = new PIDcontroller(0.01f, 0f,0.1f); // 0.01, 0, 0.1
  	
  	static float upperbound = 0.1f;
  	static float lowerbound = -0.1f;
  	
  	private static double SIMULATION_PERIOD = 0.01;


  	static boolean ascending = false;
 	static float refHeight = 0;
 	static float refRoll = 0;
 	static float refHeading = 0;
 	static float refPitch = 0;
  	static boolean cruising = true;
  	static boolean descending = false;
  	static boolean turn = false;
  	static boolean turnLeft = false;
  	static boolean turnRight = false;
  	static float maxRoll = 0.1f;
  	static boolean noTurn = true;
  	static boolean takeoff = false;
  	static boolean prevtakeoff = false;
  	static boolean landing = false;


    static AutopilotOutputsImplementation calculate(AutopilotInputs input, float[] targetVector, int nbColumns, int nbRows, AutopilotImplementation autopilot) {
         PreviousInputs prev = autopilot.getPreviousInput();
         float leftWingInclination = 0;
         float rightWingInclination = 0;
         float horStabInclination = 0;
         float verStabInclination = 0;
         float frontBrake = 0;
         float leftBrake = 0;
         float rightBrake = 0;
         float thrust = 0;
         float dt = -prev.getElapsedTime()+input.getElapsedTime();
         Vector velocityWorld;
         Vector velocityDrone = new Vector(0, 0, -32);
         AutopilotConfig config = autopilot.getConfig();
         boolean prevDescending = descending;
         
         
         velocityWorld = new Vector((input.getX()-prev.getX())/dt,(input.getY()-prev.getY())/dt,(input.getZ()-prev.getZ())/dt);
         velocityDrone = velocityWorld.inverseTransform(prev.getHeading(),prev.getPitch(),prev.getRoll());
         Vector angularVelocity = (new Vector((input.getPitch()-prev.getPitch())/dt,(input.getHeading()-prev.getHeading())/dt,(input.getRoll()-prev.getRoll())/dt));
         
         if (targetVector == null) {
        	 if ((ascending)||(descending)) {
        		 refHeight = input.getY();
        	 }
        	 ascending = false;
        	 descending = false;
        	 cruising = true;
         }
         else if (cruising) {
        	 if (targetVector[1]>=15) {
        		 ascending = true;
        		 descending = false;
        		 cruising = false;
        	 }
        	 else if (targetVector[1]<=-15) {
        		 ascending = false;
        		 descending = true;
        		 cruising = false;
        	 }
         }
         else if (ascending) {
        	 if (targetVector[1]<=-15) {
        		 cruising = true;
        		 ascending = false;
        		 descending = false;
        		 refHeight = input.getY();
        	 }
         }
         else if (descending) {
        	 if (targetVector[1]>=5) {
        		 cruising = true;
        		 ascending = false;
        		 descending = false;
        		 refHeight = input.getY();
        	 }
         }
         
         cruising = true;
		 ascending = false;
		 descending = false;
		 

		 if (input.getElapsedTime() > 20 && input.getHeading() > -0.3f) {
			 ascending = true;
			 cruising = false;
			 descending = false;
			 turnRight = false;
		 }
		 
		 if (input.getY() > 30 || input.getElapsedTime()<40) {
			 ascending = false;
			 cruising = true;
			 descending = false;
			 refHeight = 0;
			 turnLeft = false;
			
		 }

		 
		 if (input.getY() < 20 && input.getElapsedTime() < 20){
			 takeoff = true;
		 	ascending = false;
		 	descending = false;
		 	cruising = false;
		 }
		 else
			 takeoff = false;
		 
		 if (input.getElapsedTime() > 40){
			 landing = true;
			 ascending = false;
			 descending = false;
			 cruising = false;
		 }
		 else 
			 landing = false;
         
         if (cruising) {
        	 
        	if (prevtakeoff && input.getPitch() > 0.02) {
        		horStabInclination = -config.getMaxAOA();
        	}
        	else {
          	horStabInclination = -config.getMaxAOA()/4;
        	prevtakeoff = false;
        	}
//          	if (upperbound >= 0.01f)
//        		upperbound -= 0.01;
//        	if (lowerbound >= -0.01f)
//        		lowerbound -= 0.01;
//        	 
//        	
//        	if (upperbound <= 0.01f)
//        		upperbound += 0.01;
//        	if (lowerbound <= -0.01f)
//        		lowerbound += 0.01;
        	
        	
           	if (input.getPitch() > 0.01f) {
           		horStabInclination = -input.getPitch();
           	}
           	else if (input.getPitch() < -0.01f) {
           		horStabInclination = -input.getPitch();
           	}
           	if (prevDescending && input.getPitch() > 0){
           		horStabInclination *= -2;
           		
           	}
           
          	
//        	System.out.println("upper: " + upperbound);
//        	System.out.println("lower : " + lowerbound);
//        	System.out.println("cruising: ");
        	 
        	float currentProjAirspeed = (float) -Math.atan2(velocityDrone.getY(),-velocityDrone.getZ());
          	rightWingInclination = (float) (-currentProjAirspeed+0.9*config.getMaxAOA());
          	if (prevtakeoff)
          		rightWingInclination /=2;
          	System.out.println("inclination: "+ rightWingInclination);
          	leftWingInclination = rightWingInclination;
        	 
         	float gravityToCompensate = (config.getEngineMass()+config.getTailMass()+2*config.getWingMass())*config.getGravity();
         	float horLift = (float) (config.getHorStabLiftSlope()*velocityDrone.dotProduct(velocityDrone)*(currentProjAirspeed+horStabInclination)*Math.cos(horStabInclination));
         	float cancelY;
     		cancelY = (float) (-velocityWorld.getY()/SIMULATION_PERIOD)*(config.getEngineMass()+2*config.getWingMass()+config.getTailMass());
         	float forceToCompensate = gravityToCompensate - horLift + cancelY;
         	Vector lift = (new Vector(0, (float) ((2*config.getWingLiftSlope()*config.getMaxAOA()*0.9)*Math.cos(rightWingInclination)), 0)).inverseTransform(
         			input.getHeading(), input.getPitch(), input.getRoll());
         	float minSpeed = (float) Math.sqrt((forceToCompensate)/lift.getY());
         	System.out.println("minSpeed: "+ minSpeed);
         	
         	
         	if (-velocityDrone.getZ() < minSpeed) {
         		double acceleration = (minSpeed - Math.abs(velocityDrone.getZ()))/SIMULATION_PERIOD;
         		thrust = (float) (acceleration*(config.getEngineMass()+config.getTailMass()+2*config.getWingMass()));
         	}
         	
         	if (input.getY() < refHeight - 1) {
         		thrust *= 2;
         	}
         	if (input.getY() > refHeight + 1) {
         		thrust /= 2;
         	}
         	
         	
//         	System.out.println("NUL PUNT NUL 1");
//         	horStabInclination = (float) (angularVelocity.getX()*Vector.getInertiaTensor(config).getX()/(config.getTailSize()*config.getHorStabLiftSlope()*
//         			velocityDrone.dotProduct(velocityDrone)));
//         	
//         	if ((input.getPitch()>0.01)&&(horStabInclination>0)) {
//         		horStabInclination = (float) (1.5*horStabInclination);
//         	}
         	
         
//         	System.out.println("horStabInclination: " + horStabInclination);
         	
         	System.out.println("-------------------------------------------------");
         	
         }
          
         
         
         else if (takeoff){
        	 thrust = config.getMaxThrust();
        	 horStabInclination = 0;
         	
         	if (input.getPitch() > 0.08f) {
         		horStabInclination = -input.getPitch();
         	}
         	
         	float currentProjAirspeed = (float) -Math.atan2(velocityDrone.getY(),-velocityDrone.getZ());
          	rightWingInclination = (float) (-currentProjAirspeed+0.9*config.getMaxAOA());
         	leftWingInclination = rightWingInclination;
         	prevtakeoff = true;
         }
         
         else if (landing){
//        	 float currentProjAirspeed = (float) -Math.atan2(velocityDrone.getY(),-velocityDrone.getZ());
//
//            horStabInclination = 0.02f;
//   
//            if (input.getPitch() < -0.02f) {
//            	horStabInclination = -input.getPitch()*2;
//            }
//            
//         	rightWingInclination = (float) (-currentProjAirspeed+0.8*config.getMaxAOA());
//         	if (input.getY() < 30) {
//         		rightWingInclination =(float) (-currentProjAirspeed+0.9*config.getMaxAOA());
//         		leftWingInclination = rightWingInclination;
//         		horStabInclination = (float) (-currentProjAirspeed+0.5*config.getMaxAOA());
//         	}		
           	horStabInclination = -config.getMaxAOA()/4;	
            	if (input.getPitch() > 0.03f) {
            		horStabInclination = -input.getPitch();
            	}
            	else if (input.getPitch() < -0.01f) {
            		horStabInclination = config.getMaxAOA()*0.75f;
            	}
            	
            

         	float currentProjAirspeed = (float) -Math.atan2(velocityDrone.getY(),-velocityDrone.getZ());
           	rightWingInclination = (float) (-currentProjAirspeed+0.9*config.getMaxAOA());

           	leftWingInclination = rightWingInclination;
         	 
           	if (velocityDrone.getY() < -3)
           		thrust = 1000;
           	
        	 if (input.getY() < 2) {
        		 frontBrake = config.getRMax()/2;
        		 leftBrake = config.getRMax()/2;
        		 rightBrake = config.getRMax()/2;
        	 }
        	 
         	System.out.println("desc hstab: " +horStabInclination);
         }
         else if (ascending) {
        	 
        	horStabInclination = 0;
        	
        	if (input.getPitch() > 0.15f) {
        		horStabInclination = -input.getPitch();
        	}

//        	if (upperbound < 0.13f)
//        		upperbound += 0.004;
//        	if (lowerbound < 0.11f)
//        		lowerbound += 0.004;
        	 
//        	System.out.println("upper: " + upperbound);
//        	System.out.println("lower : " + lowerbound);
//        	System.out.println("ascending: ");
        	
//           	if (input.getPitch() > upperbound) {
//           		horStabInclination = -input.getPitch() - config.getMaxAOA()/2;
//           	}
//           	else if (input.getPitch() < lowerbound) {
//           		horStabInclination = -input.getPitch() + config.getMaxAOA()/2;
//           	}
        	
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
          	
          	System.out.println("ascending - thrust: " + thrust);
          	System.out.println("-----------------------------------------");
        	 
         }
         
         else if (descending) {
        	 
        	 System.out.println("descending");
        	 
        	 horStabInclination = 0f;
        	 
        	 
          
     	
     	
        	if (input.getPitch() < -0.02f) {
        		horStabInclination = -input.getPitch()*2;
        	}

     	 
     	float currentProjAirspeed = (float) -Math.atan2(velocityDrone.getY(),-velocityDrone.getZ());
       	rightWingInclination = 0;
       	System.out.println("inclination: "+ rightWingInclination);
       	leftWingInclination = rightWingInclination;
     	 
      	float gravityToCompensate = (config.getEngineMass()+config.getTailMass()+2*config.getWingMass())*config.getGravity();
      	float horLift = (float) (config.getHorStabLiftSlope()*velocityDrone.dotProduct(velocityDrone)*(currentProjAirspeed+horStabInclination)*Math.cos(horStabInclination));
    	float forceToCompensate = gravityToCompensate - horLift;
      	Vector lift = (new Vector(0, (float) ((2*config.getWingLiftSlope()*config.getMaxAOA()*0.9)*Math.cos(rightWingInclination)), 0)).inverseTransform(
      			input.getHeading(), input.getPitch(), input.getRoll());
      	float minSpeed = (float) Math.sqrt((forceToCompensate)/lift.getY());
      	System.out.println("minSpeed: "+ minSpeed);
      	
      	
      	if (-velocityDrone.getZ() < minSpeed) {
      		double acceleration = (minSpeed - Math.abs(velocityDrone.getZ()))/SIMULATION_PERIOD;
      		thrust = (float) (acceleration*(config.getEngineMass()+config.getTailMass()+2*config.getWingMass()));
      	}
      	
      	if (input.getY() < refHeight - 1) {
      		thrust *= 2;
      	}
      	if (input.getY() > refHeight + 1) {
      		thrust /= 2;
      	}
      	
        }
         
         if (cruising) {
//        	 if (targetVector == null) {
//        		 turnLeft = false;
//        		 turnRight = false;
//        		 noTurn = true;
//        	 }
//        	 else if (noTurn) {
//        		 if (targetVector[0] > 3) {
//	        		 turnRight = true;
//	        		 turnLeft = false;
//	        		 noTurn = false;
//        		 }
//        		 else if (targetVector[0] < -3) {
//            		 turnRight = false;
//            		 turnLeft = true;
//            		 noTurn = false;
//            	 }
//        	 }
//        	 else if (turnLeft) {
//        		 if (targetVector[0] > 25) {
//            		 turnLeft = false;
//            		 turnRight = false;
//            		 noTurn = true;
//            		 refHeading = input.getHeading();
//            	 }
//        	 }
//        	 else if (turnRight) {
//        		 if (targetVector[0] < -25) {
//            		 turnLeft = false;
//            		 turnRight = false;
//            		 noTurn = true;
//            		 refHeading = input.getHeading();
//            	 }
//        	 }
         }
         if ((targetVector!=null)&&(targetVector[2] >= 500)) {
        	 System.out.println("te groot 500");
        	 turnLeft = false;
        	 turnRight = false;
        	 noTurn = true;
        	 if ((turnLeft)||(turnRight)) {
        		 refHeading = input.getHeading();
        	 }
         }
         
         if (turnLeft) {
        	 float deltaroll = 0.005f;
        	 rightWingInclination += deltaroll;
        	 leftWingInclination -= deltaroll;
        	 if (input.getRoll() > 0.01f){
        		 rightWingInclination -= deltaroll;
        		 leftWingInclination += deltaroll;
        	 }
        	 thrust = 1000;
        	System.out.print("turnleft");
        	 
        	 
        	 horStabInclination = 0.01f;
        	 if (input.getPitch() > 0.05f)
        		 horStabInclination = -input.getPitch(); 
        	 
        	 if (input.getHeading() > 0.3f)
        		 turnLeft = false;
        	 
        	 
         }
         else if (turnRight) {
        	 float deltaroll = 0.005f;
        	 rightWingInclination -= deltaroll;
        	 leftWingInclination += deltaroll;
        	 if (input.getRoll() < -0.01f){
        		 rightWingInclination += deltaroll;
        		 leftWingInclination -= deltaroll;
        	 }
        	 thrust = 1000;
        	System.out.print("turnleft");
        	 
        	 
        	 horStabInclination = 0.01f;
        	 if (input.getPitch() > 0.05f)
        		 horStabInclination = -input.getPitch(); 
        	 
        	 if (input.getHeading() < -0.3f)
        		 turnRight = false;
         }
         else if (noTurn) {
        	 //System.out.println("xxnoTurn: " + refRoll);
        	 if (refRoll>0) {
    			 setRoll(refRoll-0.01f);
    		 }
    		 else if (refRoll<0) {
    			 setRoll(refRoll+0.01f);
    		 }
//        	 if (velocityDrone.getX() > 0.5f){
//        		 setRoll(-0.1f);
//        	 }else if (velocityDrone.getX() < -0.5f){
//        		 setRoll(0.1f);
//        	 }
        	 float deltaRoll = RollController.getOutput(input.getRoll(), refRoll);
             leftWingInclination -= deltaRoll/2;
             rightWingInclination += deltaRoll/2;
             verStabInclination = -HeadingController.getOutput(input.getHeading(), refHeading);
             if (verStabInclination > 0.03f) {
            	 verStabInclination = 0.03f;
             }
             else if (verStabInclination < -0.03f) {
            	 verStabInclination = -0.03f;
             }
         }
         
         if (thrust > 2000) {
        	 thrust = 2000;
         }
         System.out.println("thrust: " + thrust);

         return new AutopilotOutputsImplementation(thrust, leftWingInclination, rightWingInclination, -horStabInclination, verStabInclination, frontBrake, leftBrake, rightBrake);
     }
     
     public static void setRoll(float roll) {
    	 if (roll >= maxRoll) {
    		 refRoll = maxRoll;
    	 }
    	 else if (roll <= -maxRoll) {
    		 refRoll = -maxRoll;
    	 }
    	 else if (Math.abs(roll) <= 0.005) {
    		 refRoll = 0;
    	 }
    	 else {
    		 refRoll = roll;
    	 }
     }
    
     public float bissectie(float a, float b, AutopilotConfig config, Vector velocityDrone, Vector angularVelocity){
    			float c;
    			float fa;
    			float fb;
    			float fc;
    			float Vy = velocityDrone.getY();
    			float Vz = velocityDrone.getZ();
    			c = a+b/2;
    			fc = applyFunction(c, Vy, Vz, config, angularVelocity);
    			if (Math.abs(fc)< 0.001)
    				return c;
    			fa = applyFunction(a, Vy, Vz, config, angularVelocity);
    			fb = applyFunction(b, Vy, Vz, config, angularVelocity);
    			if  (Math.signum(fa) == Math.signum(fb))
    				return Float.NaN;
    			if (Math.signum(fa) == Math.signum(fb))
    				return bissectie(c,b, config, velocityDrone, angularVelocity);
    			if (Math.signum(fb) == Math.signum(fc))
    				return bissectie(a,c, config, velocityDrone, angularVelocity);
    			else 
    			return Float.NaN;
    }
    		
    public float applyFunction(float x, float Vy, float Vz, AutopilotConfig config, Vector angularVelocity){
    			float A = 0;
    			return (float) (-Math.cos(x)*Math.atan2(Vy * Math.cos(x) + Vz * Math.sin(x),Vy*Math.sin(x)- Vz*Math.cos(x))- A);
    }


}

