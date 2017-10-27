package autopilot;
import p_en_o_cw_2017.*;
class InputToOutput {

    static AutopilotOutputs calculate(AutopilotInputs input, float[] targetVector, int nbColumns, int nbRows, Autopilot autopilot) {
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

     //   if (horizontalError >= 10){
            horizontalError = 50;
            double horizontalAngleError =  (horizontalError/((nbColumns/2))*autopilot.getConfig().getHorizontalAngleOfView()*(Math.PI/180));
            float g = autopilot.getConfig().getGravity();
            float roll = input.getRoll();
            float U = -velocityDrone.getZ();
            float W = velocityDrone.getY();
            float enginePlace =autopilot.getConfig().getTailSize()*-autopilot.getConfig().getTailMass()/autopilot.getConfig().getEngineMass();
            double constant = horizontalAngleError*U/W;
            double tcoef = (-g*Math.sin(roll)/W-angularVelocity.getZ());
            double t2coef =  (2*autopilot.getConfig().getWingLiftSlope()*velocityDrone.dotProduct(velocityDrone)*
            				((-autopilot.getConfig().getMaxAOA()/2)/(autopilot.getConfig().getEngineMass()*Math.pow(enginePlace,2) + autopilot.getConfig().getTailMass()*Math.pow(autopilot.getConfig().getTailSize(),2) + 2*autopilot.getConfig().getWingMass()*Math.pow(autopilot.getConfig().getWingX(),2))
            						-(autopilot.getConfig().getMaxAOA()/2 -2)/(4*autopilot.getConfig().getWingMass()*Math.pow(autopilot.getConfig().getWingX(),2))));
            									
            float D = (float) (Math.pow(tcoef, 2)- 4*constant*t2coef);
            float x = (float) ((-tcoef+ Math.sqrt(D))/(2*t2coef));
            System.out.println("time" + input.getX());
            velocityDrone.printVector("vel");
            System.out.println("x : " + x);
            System.out.println("D : " + D);
            System.out.println("constant : " + constant);
            System.out.println("tcoef : " + tcoef);
            System.out.println("t2coef : " + t2coef);
//      //  }
//        //daarna omhoog/omlaag
//        else if(verticalError >=10) {
//            float r = verticalError/(nbRows/2)*autopilot.config.getHorizontalAngleOfView();
//            horStabInclination = r; //??
//        }
//        //anders gewoon blijven verder vliegen in een rechte lijn
//        else {
//
//        }
        return new AutopilotOutputs(thrust, leftWingInclination, rightWingInclination, horStabInclination, verStabInclination);
    }


}