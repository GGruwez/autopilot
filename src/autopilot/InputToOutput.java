package autopilot;
import p_en_o_cw_2017.*;
class InputToOutput {

	static PIDcontroller PitchController = new PIDcontroller(3f, 0f, 4f);
	static PIDcontroller HeightController = new PIDcontroller(0.1f, 0f, 0.02f);
	static boolean ascending = false;
	static boolean ascendFinished = false;
	static float refHeight = 80;
	static boolean cruising = false;
	static boolean descending = false;
	
    static AutopilotOutputs calculate(AutopilotInputs input, float[] targetVector, int nbColumns, int nbRows, Autopilot autopilot) {
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
        
        ascending = (input.getElapsedTime()>3)&&(! cruising);
        if ((input.getY()>=refHeight)&&(ascending)) {
        	ascending = false;
        	ascendFinished = true;
        }
        if (ascendFinished) {
        	ascendFinished = false;
        	cruising = true;
        }
        if (input.getElapsedTime()>20) {
        	cruising = false;
        	ascending = false;
        	ascendFinished = false;
        	descending = true;
        	refHeight = 0;
        }
        if (input.getY()<=0) {
        	cruising = true;
        	ascending = false;
        	ascendFinished = false;
        	descending = false;
        	refHeight = 0;
        }
        if ((descending)&&(input.getY()<=(refHeight+3))) {
        	cruising = true;
        	ascending = false;
        	ascendFinished = false;
        	descending = false;
        }
        
        if (cruising) {
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
        
        return new AutopilotOutputs(thrust, leftWingInclination, rightWingInclination, horStabInclination, verStabInclination);
    }

}