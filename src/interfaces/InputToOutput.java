 package interfaces;

import autopilot.PIDcontroller;

class InputToOutput {
 
 	static PIDcontroller PitchController = new PIDcontroller(3f, 0f, 4f);
  	static PIDcontroller HeightController = new PIDcontroller(0.1f, 0f, 0.02f);
  	static PIDcontroller RollController = new PIDcontroller(1f, 0f, 0f);
  	static PIDcontroller HeadingController = new PIDcontroller(1f, 0f, 0.01f);
  	
  	static boolean ascending = false;
//  	static boolean ascendFinished = false;
 	static float refHeight = 20;
  	static boolean cruising = false;
  	static boolean descending = false;
	
	//left-right
	static boolean turnLeft = false;
	static boolean turnRight = false;
	static boolean bankTurnLeft = false;
	static boolean bankTurnRight = false;
	static float refHeading = 0.5f;
  	
     static AutopilotOutputsImplementation calculate(AutopilotInputs input, float[] targetVector, int nbColumns, int nbRows, AutopilotImplementation autopilot) {
         PreviousInputs prev = autopilot.getPreviousInput();
         float horizontalError = targetVector[0];
         float verticalError = targetVector[1];
         float leftWingInclination = 0;
         float rightWingInclination = 0;
         float horStabInclination = 0;
         float verStabInclination = 0;
         float thrust = 0;
         float dt = prev.getElapsedTime()-input.getElapsedTime();
         Vector velocityWorld;
         Vector velocityDrone;
         AutopilotConfig config = autopilot.getConfig();
         
         velocityWorld = new Vector((input.getX()-prev.getX())/dt,(input.getY()-prev.getY())/dt,(input.getZ()-prev.getZ())/dt);
         velocityDrone = velocityWorld.inverseTransform(prev.getHeading(),prev.getPitch(),prev.getRoll());
         Vector angularVelocity = (new Vector((input.getPitch()-prev.getPitch())/dt,(input.getHeading()-prev.getHeading())/dt,(input.getRoll()-prev.getRoll())/dt));
         
         if (input.getElapsedTime()<=20) {
        	 refHeight = 20;
         }
         else {
        	 refHeight = 0;
         }
         
         if (input.getY()<(refHeight-2)) {
        	 descending = false;
        	 cruising = false;
        	 ascending = true;
         }
         else if ((ascending)&&(input.getY()>=refHeight)) {
        	 descending = false;
        	 cruising = true;
        	 ascending = false;
         }
         else if (input.getY()>(refHeight+5)) {
        	 descending = true;
        	 cruising = false;
        	 ascending = false;
         }
         else if ((descending)&&(input.getY()<=(refHeight+3))) {
        	 descending = false;
        	 cruising = true;
        	 ascending = false;
         }
         else {
        	 descending = false;
        	 cruising = true;
        	 ascending = false;
         }
         
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
 	            thrust = 50;
 	            horStabInclination = (float) Math.PI/120 - input.getPitch();
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
         
         //-----------TURNING-----------//
         //turnLeft --> error kleiner dan 30°
         if((Math.abs(refHeading-input.getHeading())<=Math.PI/6) && (refHeading-input.getHeading() >= 0)){
         	System.out.print("turnLeft");
         	System.out.print(input.getHeading());
         	rightWingInclination =  RollController.getOutput(input.getRoll(), 0);
             leftWingInclination = -rightWingInclination;
             verStabInclination = HeadingController.getOutput(input.getHeading(), refHeading);
             
         }
         //turnRight --> error kleiner dan 30°
         else if (((Math.abs(refHeading-input.getHeading())<=Math.PI/6)) && (refHeading-input.getHeading() >= 0)){
         	System.out.print("turnRight");
         	leftWingInclination = 0;
             rightWingInclination = 0;
             verStabInclination = HeadingController.getOutput(input.getHeading(), refHeading);
             rightWingInclination =  RollController.getOutput(input.getRoll(), 0);
             leftWingInclination = -rightWingInclination;
         }
         
         //bankTurnLeft --> error groter dan 30°
         else if((Math.abs(refHeading-input.getHeading())>Math.PI/6) && (refHeading-input.getHeading() >= 0)){
         	bankTurnLeft = true;
         	bankTurnRight = false;
         	turnLeft = false;
         	turnRight = false;
         }
         //bankTurnLeft --> error groter dan 30°
         else if((Math.abs(refHeading-input.getHeading())>Math.PI/6) && (refHeading-input.getHeading() <= 0)){
         	bankTurnLeft = false;
         	bankTurnRight = true;
         	turnLeft = false;
         	turnRight = false;
         }
         //wingLeveller --> roll = 0 --> cruising
         else{
         	
         }
         
         return new AutopilotOutputsImplementation(thrust, leftWingInclination, rightWingInclination, horStabInclination, verStabInclination);
     }
 
 }
