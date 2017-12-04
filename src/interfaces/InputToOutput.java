 package interfaces;


class InputToOutput {
 
 	static PIDcontroller PitchController = new PIDcontroller(4.5f, 0f, 15f);//new PIDcontroller(3f, 0f, 15f); //3, 0, 4
  	static PIDcontroller HeightController = new PIDcontroller(0.1f, 0f, 0.02f);

  	static PIDcontroller RollController = new PIDcontroller(0.3f, 0f, 12f);//new PIDcontroller(1f, 0f, 25f);
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
         

         
         if (targetVector == null) {
        	 if ((ascending)||(descending)) {
        		 refHeight = input.getY();
        	 }
        	 ascending = false;
        	 descending = false;
        	 cruising = true;
         }
         else if (cruising) {
        	 if (targetVector[1]>=10) {
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
         refHeight = 0;
         
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
             
            horStabInclination = PitchController.getOutput(input.getPitch(), (float) -Math.PI/30);
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
         
         
         turn = input.getElapsedTime() >4;
         
         
         if (turn){
        	 
        	
        	 
        	if (input.getHeading()<Math.PI/9){
        		refPitch = 0.0f;
        	}else{
        		refPitch = 0.00f;
        	}
        	 
        	 

         	refRoll = (float) (Math.PI/4);
         	
         	horStabInclination = PitchControllerTurning.getOutput(input.getPitch(), refPitch);
         	System.out.println("pitchError: " + horStabInclination);
// 	        if (input.getPitch() + horStabInclination > Math.PI/9){
// 	        	horStabInclination = (float) (Math.PI/9);
// 	        }
// 	        else if(input.getPitch() + horStabInclination < -Math.PI/9){
// 	        	horStabInclination = (float) (-Math.PI/9);
// 	        }
//         
         	
         	float error = -HeadingController.getOutput(input.getHeading(), refHeading);
         	System.out.println("headingErro: " + error);

         	verStabInclination = error;

        	refRoll= (input.getElapsedTime() > 8? 0f : refRoll );
         	//refRoll = -HorizontalController.getOutput(input.getX(), 10);
        	refRoll = (float) (refRoll > Math.PI/4 ? Math.PI/4 : refRoll);
         	refRoll = (float) (refRoll < -Math.PI/4 ? -Math.PI/4 : refRoll);
         	
         	
         	float deltaRoll = RollControllerTurning.getOutput(input.getRoll(), refRoll);
         	System.out.println("deltaroll: " + deltaRoll);
;
         	leftWingInclination -= deltaRoll/2;
         	rightWingInclination += deltaRoll/2;
         }
         

         return new AutopilotOutputsImplementation(thrust, leftWingInclination, rightWingInclination, horStabInclination, verStabInclination);
     }
 
 }
