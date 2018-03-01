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

    static AutopilotOutputsImplementation calculate(AutopilotInputs input, float[] targetVector, int nbColumns, int nbRows, AutopilotImplementation autopilot) {
         PreviousInputs prev = autopilot.getPreviousInput();
         float leftWingInclination = 0;
         float rightWingInclination = 0;
         float horStabInclination = 0;
         float verStabInclination = 0;
         float thrust = 0;
         float dt = -prev.getElapsedTime()+input.getElapsedTime();
         Vector velocityWorld;
         Vector velocityDrone = new Vector(0, 0, -32);
         AutopilotConfig config = autopilot.getConfig();
         
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
		 
		 if (input.getElapsedTime() > 3) {
			 ascending = true;
			 cruising = false;
			 descending = false;
		 }
		 
		 if (input.getElapsedTime() > 10) {
			 cruising = true;
			 ascending = false;
			 descending = false;
			 refHeight = 20;
		 }
		 
         
         if (cruising) {
        	 
          	horStabInclination = 0;
         
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
           		horStabInclination = 0;
           	}
           	else if (input.getPitch() < -0.01f) {
           		horStabInclination = -input.getPitch()*15f;
           	}
          	
          	
//        	System.out.println("upper: " + upperbound);
//        	System.out.println("lower : " + lowerbound);
//        	System.out.println("cruising: ");
        	 
        	float currentProjAirspeed = (float) -Math.atan2(velocityDrone.getY(),-velocityDrone.getZ());
          	rightWingInclination = (float) (-currentProjAirspeed+0.9*config.getMaxAOA());
          	System.out.println("inclination: "+ rightWingInclination);
          	leftWingInclination = rightWingInclination;
        	 
         	float gravityToCompensate = (config.getEngineMass()+config.getTailMass()+2*config.getWingMass())*config.getGravity();
         	float horLift = (float) (config.getHorStabLiftSlope()*velocityDrone.dotProduct(velocityDrone)*(currentProjAirspeed+horStabInclination)*Math.cos(horStabInclination));
         	float cancelY = (float) (-velocityDrone.getY()/2/SIMULATION_PERIOD)*(config.getEngineMass()+2*config.getWingMass()+config.getTailMass());
         	float forceToCompensate = gravityToCompensate - horLift + cancelY;
         	Vector lift = (new Vector(0, (float) ((2*config.getWingLiftSlope()*config.getMaxAOA()*0.9)*Math.cos(rightWingInclination)), 0)).inverseTransform(
         			input.getHeading(), input.getPitch(), input.getRoll());
         	float minSpeed = (float) Math.sqrt((forceToCompensate)/lift.getY());
         	System.out.println("minSpeed: "+ minSpeed);
     
         	
         	if (-velocityDrone.getZ() < minSpeed) {
         		double acceleration = (minSpeed - Math.abs(velocityDrone.getZ()))/SIMULATION_PERIOD/10;
         		thrust = (float) (acceleration*(config.getEngineMass()+config.getTailMass()+2*config.getWingMass()));
         	}
         	
         	if (input.getY() > refHeight) {
         		thrust = thrust/2;
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
         	
          	if (velocityDrone.getY() < 3) {
         		double acceleration = 1.2*(3-velocityDrone.getY());
         		thrust = (float) (acceleration*(config.getEngineMass()+config.getTailMass()+2*config.getWingMass()));
         	}
          	else {
          		thrust = 0;
          	}
          	
          	System.out.println("ascending - thrust: " + thrust);
          	System.out.println("-----------------------------------------");
        	 
         }
         
         else if (descending) {
        	 
        	 if (input.getY() <= refHeight + 10) {
        		 horStabInclination = 0;
                 
               	if (upperbound >= 0.01f)
             		upperbound -= 0.01;
             	if (lowerbound >= -0.01f)
             		lowerbound -= 0.01;
             	 
             	
             	if (upperbound < 0.01f)
             		upperbound += 0.01;
             	if (lowerbound < -0.01f)
             		lowerbound += 0.01;
             	
             	
                if (input.getPitch() > upperbound) {
                	horStabInclination = -input.getPitch() - config.getMaxAOA()/2;
                }
                else if (input.getPitch() < lowerbound) {
                	horStabInclination = -input.getPitch() + config.getMaxAOA()/2;
                }
        	 }
        	 
        	 else {
        	 rightWingInclination = 0;
        	 leftWingInclination = 0;
         	if (upperbound > -0.02f)
        		upperbound -= 0.004;
        	if (lowerbound > -0.03f)
        		lowerbound -= 0.004;
        	 
        	System.out.println("upper: " + upperbound);
        	System.out.println("lower : " + lowerbound);
        	System.out.println("ascending: ");
        	
           	if (input.getPitch() > upperbound) {
           		horStabInclination = -input.getPitch() - config.getMaxAOA()/2;
           	}
           	else if (input.getPitch() < lowerbound) {
           		horStabInclination = -input.getPitch() + config.getMaxAOA()/2;
           	}
        	 }
        	 
           	float currentProjAirspeed = (float) -Math.atan2(velocityDrone.getY(),-velocityDrone.getZ());
           	rightWingInclination = (float) (-currentProjAirspeed+0.9*config.getMaxAOA());
           	System.out.println("inclination: "+ rightWingInclination);
           	leftWingInclination = rightWingInclination; 
         }
         
         else {
            leftWingInclination = input.getPitch();
            rightWingInclination = input.getPitch();
         	horStabInclination = PitchControllerxx.getOutput(input.getPitch(), 0);
 	        if (input.getPitch() + horStabInclination > Math.PI/9){
 	        	horStabInclination = (float) (Math.PI/9);
 	        }
 	        else if(input.getPitch() + horStabInclination < -Math.PI/9){
 	        	horStabInclination = (float) (-Math.PI/9);
 	        }
 	        
         }
         
         if (cruising) {
        	 if (targetVector == null) {
        		 turnLeft = false;
        		 turnRight = false;
        		 noTurn = true;
        	 }
        	 else if (noTurn) {
        		 if (targetVector[0] > 3) {
	        		 turnRight = true;
	        		 turnLeft = false;
	        		 noTurn = false;
        		 }
        		 else if (targetVector[0] < -3) {
            		 turnRight = false;
            		 turnLeft = true;
            		 noTurn = false;
            	 }
        	 }
        	 else if (turnLeft) {
        		 if (targetVector[0] > 25) {
            		 turnLeft = false;
            		 turnRight = false;
            		 noTurn = true;
            		 refHeading = input.getHeading();
            	 }
        	 }
        	 else if (turnRight) {
        		 if (targetVector[0] < -25) {
            		 turnLeft = false;
            		 turnRight = false;
            		 noTurn = true;
            		 refHeading = input.getHeading();
            	 }
        	 }
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
        	 thrust = 2;
        	 setRoll(refRoll+0.01f);
        	 System.out.println("turnLeft: " + refRoll);
        	 verStabInclination = 0.0f;
        	 float deltaRoll = RollController.getOutput(input.getRoll(), refRoll);
             leftWingInclination -= deltaRoll/2;
             rightWingInclination += deltaRoll/2;
             if (verStabInclination > 0.03f) {
            	 verStabInclination = 0.03f;
             }
             else if (verStabInclination < -0.03f) {
            	 verStabInclination = -0.03f;
             }
         }
         else if (turnRight) {
        	 thrust = 2;
        	 setRoll(refRoll-0.01f);
        	 System.out.println("turnRight");
        	 verStabInclination = -0.0f;
        	 float deltaRoll = RollController.getOutput(input.getRoll(), refRoll);
             leftWingInclination -= deltaRoll/2;
             rightWingInclination += deltaRoll/2;
             if (verStabInclination > 0.03f) {
            	 verStabInclination = 0.03f;
             }
             else if (verStabInclination < -0.03f) {
            	 verStabInclination = -0.03f;
             }
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

         return new AutopilotOutputsImplementation(thrust, leftWingInclination, rightWingInclination, -horStabInclination, verStabInclination);
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

