package autopilot;
import p_en_o_cw_2017.*;
class InputToOutput {

	static PIDcontroller PitchController = new PIDcontroller(0.7f, 0f, 3.5f);
	static PIDcontroller ThrustController = new PIDcontroller(1f, 0f, 0f);
	
    static AutopilotOutputs calculate(AutopilotInputs input, float[] targetVector, int nbColumns, int nbRows, Autopilot autopilot) {
        PreviousInputs prev = autopilot.getPreviousInput();
        float horizontalError = targetVector[0];
        float verticalError = targetVector[1];
        float leftWingInclination = -input.getPitch();
        float rightWingInclination = -input.getPitch();
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
        
        horStabInclination = PitchController.getOutput(input.getPitch(), 0);
        if (input.getPitch() + horStabInclination > Math.PI/9){
        	horStabInclination = (float) (Math.PI/9);
        }
        else if(input.getPitch() + horStabInclination < -Math.PI/9){
        	horStabInclination = (float) (-Math.PI/9);
        }
        
        thrust = ThrustController.getOutput(velocityWorld.getZ(), -30);
        if (thrust > autopilot.getConfig().getMaxThrust()){
        	thrust = (float) autopilot.getConfig().getMaxThrust();
        }
        else if(thrust < 0){
        	thrust = 0;
        }
        
        return new AutopilotOutputs(thrust, leftWingInclination, rightWingInclination, horStabInclination, verStabInclination);
    }

}