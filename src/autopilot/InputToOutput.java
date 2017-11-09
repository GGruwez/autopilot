package autopilot;
import p_en_o_cw_2017.*;
class InputToOutput {

	static PIDcontroller PitchController = new PIDcontroller(3f, 0f, 4f);
	static boolean ascending;
	static float refHeight = 20;
	static boolean cruising = false;
	
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
        
        ascending = (input.getElapsedTime() > 3)&&(input.getY() < (refHeight));
//        cruising = (input.getElapsedTime() > 3)&&(! ascending);
        cruising = false;
        
        if (cruising) {
        	if (input.getY() < refHeight) {
        		leftWingInclination = 0;
                rightWingInclination = 0;
                horStabInclination = 0;
                verStabInclination = 0;
                thrust = 50;
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
        	
        }
        
        if (ascending) {
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