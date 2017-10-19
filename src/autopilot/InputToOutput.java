package autopilot;

import p_en_o_cw_2017.*;
public class InputToOutput {

    public AutopilotOutputs calculate(AutopilotInputs input, float[] imageVector, int nbColumns, int nbRows, Autopilot autopilot) {
        float x = imageVector[0];
        float y = imageVector[1];
        float leftWingInclination = 0;
        float rightWingInclination = 0;
        float horStabInclination = 0;
        float verStabInclination = 0;
        float thrust = 0;
        
        AutopilotInputs prev = autopilot.getPreviousInput();
        //eerst draaien
        if (imageVector[0]!=0) {
            float r = x/(nbColumns/2)*autopilot.config.getHorizontalAngleOfView();
            float U0 = -(input.getZ()-prev.getZ())/prev.getElapsedTime()-input.getElapsedTime();
            float W0 = (input.getY()-prev.getY())/prev.getElapsedTime()-input.getElapsedTime();
            float g = autopilot.config.getGravity();
            float omega = input.getPitch();
            float p = (float) (r*U0/W0 - g/W0*Math.sin(omega));
            if (x>0) {
                //formule Simon
            }
            else {
                //formule Simon
            }
        }
        //daarna omhoog/omlaag
        else if(imageVector[1]!=0) {
            float r = y/(nbRows/2)*autopilot.config.getHorizontalAngleOfView();
            horStabInclination = r; //??
        }
        //anders gewoon blijven verder vliegen in een rechte lijn
        else {

        }

        return new AutopilotOutputs(thrust, leftWingInclination, rightWingInclination, horStabInclination, verStabInclination);
    }


}