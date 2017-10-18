<<<<<<< HEAD
import autopilot.p_en_o_cw_2017.Autopilot;
import autopilot.p_en_o_cw_2017.autopilot_src_generated.*;
=======
import p_en_o_cw_2017.*;

>>>>>>> 4a9f7758de3cd800b3c95df128aec3669ebf7849
public class InputToOutput {

    public AutopilotOutputs calculate(AutopilotInputs input, float[] imageVector, int nbColumns, int nbRows, AutopilotInputs prev, Autopilot autopilot) {
        float x = imageVector[0];
        float y = imageVector[1];
        float leftWingInclination;
        float rightWingInclination;
        float horStabInclination;
        //eerst draaien
        if (imageVector[0]!=0) {
            float r = x/(nbColumns/2)*60;
            float U0 = -(input.getZ()-prev.getZ())/prev.getElapsedTime()-input.getElapsedTime();
            float W0 = (input.getY()-prev.getY())/prev.getElapsedTime()-input.getElapsedTime();
            float g = autopilot.config;
            float omega = input.getPitch();
            float p = (float) (r*U0/W0 - g/W0*Math.sin(omega));


        }
        //daarna omhoog/omlaag
        else if(imageVector[1]!=0) {

        }


        return new AutopilotOutputs() {
            public float getThrust() { return 0; }
            public float getLeftWingInclination() { return leftWingInclination; }
            public float getRightWingInclination() { return rightWingInclination; }
            public float getHorStabInclination() { return horStabInclination; }
            public float getVerStabInclination() { return 0; }
        };
    }


}