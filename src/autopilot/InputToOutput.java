package autopilot;
import autopilot.Vector;


import p_en_o_cw_2017.*;
public class InputToOutput {

    public static AutopilotOutputs calculate(AutopilotInputs input, float[] targetVector, int nbColumns, int nbRows, Autopilot autopilot) {
        float horizontalError = targetVector[0];
        float verticalError = targetVector[1];
        float leftWingInclination = 0;
        float rightWingInclination = 0;
        float horStabInclination = 0;
        float verStabInclination = 0;
        float thrust = 0;
        Vector velocityWorld;
        Vector velocityDrone;
        AutopilotInputs prev = autopilot.getPreviousInput();
        
        velocityWorld = new Vector((input.getX()-prev.getX())/(prev.getElapsedTime()-input.getElapsedTime()),(input.getY()-prev.getY())/(prev.getElapsedTime()-input.getElapsedTime()),(input.getZ()-prev.getZ())/(prev.getElapsedTime()-input.getElapsedTime()));
        velocityDrone = velocityWorld.inverseTransform(prev.getHeading(),prev.getPitch(),prev.getRoll());
        Vector angularVelocity = new Vector((input.getHeading()-prev.getHeading())/(prev.getElapsedTime()-input.getElapsedTime()),(input.getPitch()-prev.getPitch())/(prev.getElapsedTime()-input.getElapsedTime()),(input.getRoll()-prev.getRoll())/(prev.getElapsedTime()-input.getElapsedTime()));
        
        //eerst draaien
        
        if (horizontalError >= 10){
            float horizontalAngleError = (float) (horizontalError/((nbColumns/2))*autopilot.config.getHorizontalAngleOfView()*(Math.PI/180));
            float g = autopilot.config.getGravity();
            float roll = input.getRoll();
            float tconstant = (float) (velocityDrone.getY()/-velocityDrone.getZ() * angularVelocity.getZ() + g/-velocityDrone.getZ() * Math.sin(input.getRoll()));
            float t2constant = velocityDrone.getY()/-velocityDrone.getZ() * 2;
            float target = horizontalAngleError ;
            
            
        }
        //daarna omhoog/omlaag
        else if(verticalError >=10) {
            float r = verticalError/(nbRows/2)*autopilot.config.getHorizontalAngleOfView();
            horStabInclination = r; //??
        }
        //anders gewoon blijven verder vliegen in een rechte lijn
        else {

        }


        return new AutopilotOutputs(1, leftWingInclination, rightWingInclination, horStabInclination, verStabInclination);
    }


}