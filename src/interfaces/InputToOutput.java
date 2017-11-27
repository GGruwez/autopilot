 package interfaces;


class InputToOutput {
 
 	static PIDcontroller PitchController = new PIDcontroller(3f, 0f, 4f);
  	static PIDcontroller HeightController = new PIDcontroller(0.1f, 0f, 0.02f);

  	static PIDcontroller RollController = new PIDcontroller(1f, 0f, 25f);
  	static PIDcontroller HeadingController = new PIDcontroller(2f,0f,100f);
  	
  	static PIDcontroller SpeedController = new PIDcontroller(5f, 0, 7f);

  	static boolean ascending = false;
//  static boolean ascendFinished = false;
 	static float refHeight = 20;
 	static float refRoll = 0;
 	static float refHeading = 0;
  	static boolean cruising = false;
  	static boolean descending = false;
	
	//left-right
	static boolean turnLeft = false;
	static boolean turnRight = false;
	static boolean bankTurnLeft = false;
	static boolean bankTurnRight = false;

  	
     static AutopilotOutputsImplementation calculate(AutopilotInputs input, float[] targetVector, int nbColumns, int nbRows, AutopilotImplementation autopilot) {
         PreviousInputs prev = autopilot.getPreviousInput();
         float horizontalError = targetVector[0];
         float verticalError = targetVector[1];
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
         
         if (input.getElapsedTime()<=20) {
        	 refHeight = 0;
         }
         else {
        	 refHeight = 0;
         }
         
         cruising = true;
         
//         if (input.getY()<(refHeight-2)) {
//        	 descending = false;
//        	 cruising = false;
//        	 ascending = true;
//         }
//         else if ((ascending)&&(input.getY()>=refHeight)) {
//        	 descending = false;
//        	 cruising = true;
//        	 ascending = false;
//         }
//         else if (input.getY()>(refHeight+5)) {
//        	 descending = true;
//        	 cruising = false;
//        	 ascending = false;
//         }
//         else if ((descending)&&(input.getY()<=(refHeight+3))) {
//        	 descending = false;
//        	 cruising = true;
//        	 ascending = false;
//         }
//         else {
//        	 descending = false;
//        	 cruising = true;
//        	 ascending = false;
//         }
         
         if (cruising) {
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
 	            thrust = SpeedController.getOutput(velocityDrone.getY(), 0f);
 	            horStabInclination = (float) Math.PI/120 - input.getPitch();
              }
             
        	 thrust += -SpeedController.getOutput(velocityDrone.getZ(), -40f);
//        	 thrust = SpeedController.getOutput(velocityDrone.getY(), 0);
//        	 System.out.println("thrust: " + thrust);
//        	 System.out.println("y: " + velocityDrone.getY());
        	 if (thrust<0) {
        		 thrust = 0;
        	 }
        	 else if (thrust > 80) {
        		 thrust = 80;
        	 }
          }
          
         else if (ascending) {
         	leftWingInclination = input.getPitch();
             rightWingInclination = input.getPitch();
             horStabInclination = 0;
             verStabInclination = 0;
             thrust = 100;
             
             horStabInclination = PitchController.getOutput(input.getPitch(), (float) Math.PI/18);
 	        if (input.getPitch() + horStabInclination > Math.PI/9){
 	        	horStabInclination = (float) (Math.PI/9);
 	        }
 	        else if(input.getPitch() + horStabInclination < -Math.PI/9){
 	        	horStabInclination = (float) (-Math.PI/9);
 	        }
         }
         else if (descending) {
         	leftWingInclination = input.getPitch();
             rightWingInclination = input.getPitch();
             horStabInclination = 0;
             verStabInclination = 0;
             thrust = 0;
             
            horStabInclination = PitchController.getOutput(input.getPitch(), (float) -Math.PI/18);
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
         
//         float error = -HeadingController.getOutput(input.getHeading(), refHeading);
//         System.out.println(error);
//         //verStabInclination = error;
//         
//         refRoll = 0.1f*error;
//         
//         float delta = RollController.getOutput(input.getRoll(), refRoll);
//         //System.out.println(delta/2);
//         leftWingInclination -= delta/2;
//         rightWingInclination += delta/2;
         
         //System.out.println("LW: " + leftWingInclination + "   RW: " + rightWingInclination);

         return new AutopilotOutputsImplementation(thrust, leftWingInclination, rightWingInclination, horStabInclination, verStabInclination);
     }
 
 }
