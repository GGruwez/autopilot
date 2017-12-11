 package interfaces;


class InputToOutput {
 
 	static PIDcontroller PitchController = new PIDcontroller(4.5f, 0f, 15f);//new PIDcontroller(3f, 0f, 15f); //3, 0, 4
  	static PIDcontroller HeightController = new PIDcontroller(0.1f, 0f, 0.02f);

//  	static PIDcontroller RollController = new PIDcontroller(1f, 0f, 25f);
//  	
//  	static PIDcontroller SpeedController = new PIDcontroller(5f, 0, 20f); //5, 0, 7
//  	static PIDcontroller HeadingController = new PIDcontroller(1f,0f,10f);

  	static PIDcontroller RollController = new PIDcontroller(0.3f, 0f, 12f);//0.3, 0, 12
  	static PIDcontroller HorizontalController = new PIDcontroller(0.1f, 0f, 7f);
  	
  	static PIDcontroller SpeedController = new PIDcontroller(5f, 0, 20f); //5, 0, 7
  	
  	static PIDcontroller PitchControllerTurning = new PIDcontroller(4.5f, 0f, 15f);
  	static PIDcontroller RollControllerTurning = new PIDcontroller(0.3f, 0f, 12f);
  	static PIDcontroller HeadingController = new PIDcontroller(4f,0.1f,35f);//(4f,0.1f,25f)


  	static boolean ascending = false;
//  static boolean ascendFinished = false;
 	static float refHeight = 0;
 	static float refRoll = 0;
 	static float refHeading = 0;
 	static float refPitch = 0;
  	static boolean cruising = false;
  	static boolean descending = false;
  	static boolean turn = false;
  	static boolean turnLeft = false;
  	static boolean turnRight = false;
  	static float maxRoll = 0.1f;

     static AutopilotOutputsImplementation calculate(AutopilotInputs input, float[] targetVector, int nbColumns, int nbRows, AutopilotImplementation autopilot) {
         PreviousInputs prev = autopilot.getPreviousInput();
         float leftWingInclination = 0;
         float rightWingInclination = 0;
         float horStabInclination = 0;
         float verStabInclination = 0;
         float thrust = 0;
         float dt = -prev.getElapsedTime()+input.getElapsedTime();
         Vector velocityWorld;
         Vector velocityDrone;
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
         
         if (cruising) {
        	System.out.println("cruising: " + refHeight);
        	thrust = 0;
          	leftWingInclination = input.getPitch();
            rightWingInclination = input.getPitch();
           	horStabInclination = PitchController.getOutput(input.getPitch(), 0);
           	
            if (input.getPitch() + horStabInclination > Math.PI/9){
             	horStabInclination = (float) (Math.PI/9);
            }
            else if(input.getPitch() + horStabInclination < -Math.PI/9){
     	        horStabInclination = (float) (-Math.PI/9);
      	    }
              
            if (input.getY()<refHeight) {
 	            thrust = 10*SpeedController.getOutput(input.getY(), refHeight);
 	            horStabInclination = (float) Math.PI/120 - input.getPitch();
            }
            
            if (velocityDrone.getZ()>-20f) {
            	thrust += SpeedController.getOutput(-velocityDrone.getZ(), -20f);
            }
             
        	 if (thrust<0) {
        		 thrust = 0;
        	 }
        	 else if (thrust > 80) {
        		 thrust = 80;
        	 }
          }
          
         else if (ascending) {
        	 System.out.println("ascending: " + input.getY());
         	leftWingInclination = input.getPitch();
             rightWingInclination = input.getPitch();
             horStabInclination = 0;
             verStabInclination = 0;
             thrust = 100;
             
             horStabInclination = PitchController.getOutput(input.getPitch(), (float) Math.PI/15);
 	        if (input.getPitch() + horStabInclination > Math.PI/9){
 	        	horStabInclination = (float) (Math.PI/9);
 	        }
 	        else if(input.getPitch() + horStabInclination < -Math.PI/9){
 	        	horStabInclination = (float) (-Math.PI/9);
 	        }
         }
         
         else if (descending) {
        	 System.out.println("descending");
         	 leftWingInclination = input.getPitch();
             rightWingInclination = input.getPitch();
             horStabInclination = 0;
             verStabInclination = 0;
             thrust = 0;
             
            horStabInclination = PitchController.getOutput(input.getPitch(), (float) -Math.PI/35);
 	        if (input.getPitch() + horStabInclination > Math.PI/9){
 	        	horStabInclination = (float) (Math.PI/9);
 	        }
 	        else if(input.getPitch() + horStabInclination < -Math.PI/9){
 	        	horStabInclination = (float) (-Math.PI/9);
 	        }
         }
         
         else {
            leftWingInclination = input.getPitch();
            rightWingInclination = input.getPitch();
         	horStabInclination = PitchController.getOutput(input.getPitch(), 0);
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
        		 if (refRoll>0) {
        			 setRoll(refRoll-0.01f);
        		 }
        		 else if (refRoll<0) {
        			 setRoll(refRoll+0.01f);
        		 }
        	 }
        	 else if (targetVector[0] > 7) {
        		 turnRight = true;
        		 setRoll(refRoll-0.01f);
        		 turnLeft = false;
        	 }
        	 else if (targetVector[0] < -7) {
        		 turnRight = false;
        		 turnLeft = true;
        		 setRoll(refRoll+0.01f);
        	 }
        	 else {
        		 turnRight = false;
        		 turnLeft = false;
        	 }
         }
         
         else if (! cruising) {
        	 turnLeft = false;
        	 turnRight = false;
         }
         
         if (turnLeft) {
        	 if (targetVector[0] > 25) {
        		 turnLeft = false;
        		 turnRight = false;
        		 refHeading = input.getHeading();
        	 }
         }
         else if (turnRight) {
        	 if (targetVector[0] < -25) {
        		 turnLeft = false;
        		 turnRight = false;
        		 refHeading = input.getHeading();
        	 }
         }
         
         if ((targetVector!=null)&&(targetVector[2] >= 700)) {
        	 System.out.println("te groot 500");
        	 turnLeft = false;
        	 turnRight = false;
        	 if (refRoll>0) {
    			 setRoll(refRoll-0.01f);
    		 }
    		 else if (refRoll<0) {
    			 setRoll(refRoll+0.01f); 
    		 }
         }
         
         if (turnLeft) {
        	 System.out.println("turnLeft: " + refRoll);
        	 verStabInclination = 0.02f;
        	 float deltaRoll = RollController.getOutput(input.getRoll(), refRoll);
             leftWingInclination -= deltaRoll/2;
             rightWingInclination += deltaRoll/2;
         }
         
         else if (turnRight) {
        	 System.out.println("turnRight");
        	 verStabInclination = -0.02f;
        	 float deltaRoll = RollController.getOutput(input.getRoll(), refRoll);
             leftWingInclination -= deltaRoll/2;
             rightWingInclination += deltaRoll/2;
         }
         
         else {
        	 verStabInclination = 0.0f;
        	 float deltaRoll = RollController.getOutput(input.getRoll(), refRoll);
             leftWingInclination -= deltaRoll/2;
             rightWingInclination += deltaRoll/2;
             float error = -HeadingController.getOutput(input.getHeading(), refHeading);
             verStabInclination = error;
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
    	 else {
    		 refRoll = roll;
    	 }
     }
 
 }
