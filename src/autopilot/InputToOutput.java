package autopilot;

import p_en_o_cw_2017.*;
public class InputToOutput {

    public AutopilotOutputs calculate(AutopilotInputs input, float[] targetVector, int nbColumns, int nbRows, Autopilot autopilot) {
        float horizontalError = targetVector[0];
        float verticalError = targetVector[1];
        float leftWingInclination;
        float rightWingInclination;
        float horStabInclination;
        Vector velocityWorld;
        Vector velocityDrone;
        AutopilotInputs prev = autopilot.getPreviousInput();
        
        velocityWorld = new Vector((input.getX()-prev.getX())/(prev.getElapsedTime()-input.getElapsedTime()),(input.getY()-prev.getY())/(prev.getElapsedTime()-input.getElapsedTime()),(input.getZ()-prev.getZ())/(prev.getElapsedTime()-input.getElapsedTime()));
        velocityDrone = velocityWorld.inverseTransform(prev.getHeading(),prev.getPitch(),prev.getRoll());
        //eerst draaien
        
        if (horizontalError >= 10) {
        	
        	
            float horizontalAngleError = horizontalError/((nbColumns/2)*autopilot.config.getHorizontalAngleOfView());
            float g = autopilot.config.getGravity();
            float pitch = input.getPitch();
            float targetRoll = (float) (horizontalAngleError*-velocityDrone.getZ()/velocityDrone.getY() - g/velocityDrone.getY()*Math.sin(pitch));
            if (horizontalError>0) {
                //formule Simon
            }
            else {
                //formule Simon
            }
        }
        //daarna omhoog/omlaag
        else if(targetVector[1]!=0) {
            float r = verticalError/(nbRows/2)*autopilot.config.getHorizontalAngleOfView();
            horStabInclination = r; //??
        }
        //anders gewoon blijven verder vliegen in een rechte lijn
        else {

        }


        return new AutopilotOutputs(thrust, leftWingInclination, rightWingInclination, horStabInclination, verStabInclination);
    }


}