 package interfaces;


class InputToOutput {
 
 	static PIDcontroller PitchController = new PIDcontroller(3f, 0f, 15f); //3, 0, 4
  	static PIDcontroller HeightController = new PIDcontroller(0.1f, 0f, 0.02f);

  	static PIDcontroller RollController = new PIDcontroller(1f, 0f, 25f);
  	static PIDcontroller HeadingController = new PIDcontroller(2f,0f,100f);
  	
  	static PIDcontroller SpeedController = new PIDcontroller(5f, 0, 10f); //5, 0, 7

  	static boolean ascending = false;
//  static boolean ascendFinished = false;
 	static float refHeight = 0;
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
         
//         if (targetVector == null) {
//        	 ascending = false;
//        	 descending = false;
//        	 cruising = true;
//         }
//         else if (targetVector[1]>=10) {
//        	 if (descending) {
//        		 ascending = false;
//        		 descending = false;
//        		 cruising = false;
//        	 }
//        	 else {
//        		 ascending = true;
//        		 descending = false;
//        		 cruising = false;
//        	 }
//         }
//         else if (targetVector[1]<=-10) {
//        	 if (ascending) {
//        		 ascending = false;
//        		 descending = false;
//        		 cruising = true;
//        		 refHeight = input.getY();
//        	 }
//        	 else {
//	        	 ascending = false;
//	        	 descending = true;
//	        	 cruising = false;
//        	 }
//         }
//         else {
//        	 if (ascending) {
//        		 ascending = true;
//        		 descending = false;
//        		 cruising = false;
//        	 }
//        	 else if (descending) {
//        		 ascending = false;
//        		 descending = true;
//        		 cruising = false;
//        	 }
//        	 else if (cruising) {
//        		 ascending = false;
//        		 descending = false;
//        		 cruising = true;
//        	 }
//         }
//         if ((targetVector!=null)&&(descending)&&(targetVector[1]<-5)) {
//        	 ascending = false;
//        	 descending = false;
//        	 cruising = false;
//         }
         
         cruising = true;
         
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
 	            thrust = 5*SpeedController.getOutput(input.getY(), 0);
 	            horStabInclination = (float) Math.PI/120 - input.getPitch();
            }
             
        	 thrust += -SpeedController.getOutput(velocityDrone.getZ(), -40f);
        	 if (thrust<0) {
        		 thrust = 0;
        	 }
        	 else if (thrust > 80) {
        		 thrust = 80;
        	 }
          }
          
         else if (ascending) {
        	 System.out.println("ascending");
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
         
//			System.out.println("LW: " + leftWingInclination + "   RW: " + rightWingInclination);

         return new AutopilotOutputsImplementation(thrust, leftWingInclination, rightWingInclination, horStabInclination, verStabInclination);
     }
 
 }
