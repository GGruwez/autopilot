package autopilot;

import java.util.Vector;

import p_en_o_cw_2017.*;
public class InputToOutput {

    public AutopilotOutputs calculate(AutopilotInputs input, float[] targetVector, int nbColumns, int nbRows, Autopilot autopilot) {
        float horizontalError = targetVector[0];
        float verticalError = targetVector[1];
        float leftWingInclination;
        float rightWingInclination;
        float horStabInclination;
        Vector velocityWorld;
        Vector<E> velocityDrone;
        AutopilotInputs prev = autopilot.getPreviousInput();
        
        velocityWorld = new Vector((input.getX()-prev.getX())/(prev.getElapsedTime()-input.getElapsedTime()),(input.getY()-prev.getY())/(prev.getElapsedTime()-input.getElapsedTime()),(input.getZ()-prev.getZ())/(prev.getElapsedTime()-input.getElapsedTime()));
        velocityDrone = velocityWorld.inverseTransform(prev.getHeading(),prev.getPitch(),prev.getRoll());
        angularVelocity = new Vector((input.getRoll()-prev.getRoll())/(prev.getElapsedTime()-input.getElapsedTime()),(input.getHeading()-prev.getHeading())/(prev.getElapsedTime()-input.getElapsedTime()),(input.getPitch()-prev.getPitch())/(prev.getElapsedTime()-input.getElapsedTime()))
        
        //eerst draaien
        
        if (horizontalError >= 10) {
        	
        	
            float horizontalAngleError = (float) (horizontalError/((nbColumns/2))*autopilot.config.getHorizontalAngleOfView()*(Math.PI/180));
            float g = autopilot.config.getGravity();
            float roll = input.getRoll();
            float tconstant = velocityDrone.getY()/-velocityDrone.getZ() 

            float target = horizontalAngleError + (velocityDrone.get));
            
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