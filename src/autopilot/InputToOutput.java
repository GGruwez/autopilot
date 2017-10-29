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
        PreviousInputs prev = autopilot.getPreviousInput();
        AutopilotConfig config = autopilot.getConfig();
        velocityWorld = new Vector((input.getX()-prev.getX())/(prev.getElapsedTime()-input.getElapsedTime()),(input.getY()-prev.getY())/(prev.getElapsedTime()-input.getElapsedTime()),(input.getZ()-prev.getZ())/(prev.getElapsedTime()-input.getElapsedTime()));
        velocityDrone = velocityWorld.inverseTransform(prev.getHeading(),prev.getPitch(),prev.getRoll());
        Vector angularVelocity = new Vector((input.getHeading()-prev.getHeading())/(prev.getElapsedTime()-input.getElapsedTime()),(input.getPitch()-prev.getPitch())/(prev.getElapsedTime()-input.getElapsedTime()),(input.getRoll()-prev.getRoll())/(prev.getElapsedTime()-input.getElapsedTime()));
        
        //eerst draaien

        double horizontalAngleError =  (horizontalError/((nbColumns/2))*config.getHorizontalAngleOfView()*(Math.PI/180));
        float g = config.getGravity();
        float roll = input.getRoll();
        float U = -velocityDrone.getZ();
        float W = velocityDrone.getY();
        float enginePlace =config.getTailSize()*-config.getTailMass()/config.getEngineMass();
        
        double t = 1;
        double rollRate=(input.getRoll()-prev.getRoll())/(input.getElapsedTime()-prev.getElapsedTime());
        double Ix = config.getEngineMass()*Math.pow(enginePlace,2)+config.getTailMass()*Math.pow(config.getTailSize(),2);
        double Iz = 2*config.getWingMass()*Math.pow(config.getWingX(), 2);
        double Iy = Ix + Iz;
        double s= velocityDrone.euclideanLength();
        double s2=Math.pow(s, 2);
        
        if (Math.abs(horizontalError) >= 5){
            
            

//            double constant = horizontalAngleError*U/W;
//            double tcoef = (-g*Math.sin(roll)/W-angularVelocity.getZ());
//            double t2coef =  (2*config.getWingLiftSlope()*velocityDrone.dotProduct(velocityDrone)*
//            				((-config.getMaxAOA()/2)/(config.getEngineMass()*Math.pow(enginePlace,2) + config.getTailMass()*Math.pow(config.getTailSize(),2) + 2*config.getWingMass()*Math.pow(config.getWingX(),2))
//            						-(config.getMaxAOA()/2 -2)/(4*config.getWingMass()*Math.pow(config.getWingX(),2))));
//            									
//            float D = (float) (Math.pow(tcoef, 2)- 4*constant*t2coef);
//            float x = (float) ((-tcoef+ Math.sqrt(D))/(2*t2coef));
//            
            
            
            double sig = config.getWingLiftSlope()*W*Iy*(2*W*rollRate*t*Math.pow(Iz,2) - U*horizontalAngleError*Iy*Iz - 2*U*horizontalAngleError*Math.pow(Iz,2) + 2*g*t*Math.pow(Iz,2)*Math.sin(roll) + config.getWingLiftSlope()*W*s2*Math.pow(t,2)*Iy + W*rollRate*t*Iy*Iz + g*t*Iy*Iz*Math.sin(roll));
            double sig1 = Math.sqrt(sig);
            double sig3 = config.getWingLiftSlope()*W*s*t*Iy;
            double sig2 = sig3 + config.getWingLiftSlope()*W*s*t*Iz;
            
            double a1 = (sig1+sig3-input.getPitch()*config.getWingLiftSlope()*W*s*t*Iy-2*input.getPitch()*config.getWingLiftSlope()*W*s*t*Iz)/sig2;
            double a2 = -(sig1-sig3+input.getPitch()*config.getWingLiftSlope()*W*s*t*Iy+2*input.getPitch()*config.getWingLiftSlope()*W*s*t*Iz)/sig2;
            
            leftWingInclination = (float) a1;
            rightWingInclination = (float) -a1;
            
            
//            System.out.println("x : " + x);
//            System.out.println("D : " + D);
//            System.out.println("constant : " + constant);
//            System.out.println("tcoef : " + tcoef);
//            System.out.println("t2coef : " + t2coef);
//            System.out.println("a1 : " + a1);
//            System.out.println("a2 : " + a2);
        }
//        //daarna omhoog/omlaag
        if(Math.abs(verticalError) >=5) {
        	double verticalAngleError =  (verticalError/((nbRows/2))*config.getVerticalAngleOfView()*(Math.PI/180));
           
        	double sig = -config.getHorStabLiftSlope()*config.getTailSize()*U*Iy(4*verticalAngleError*Math.pow(Iz, 2)+4*W*t*Math.pow(Iz, 2)+2*verticalAngleError*Iy*Iz + 2*W*t*Iy*Iz - 4*input.getPitch()*U*t*Math.pow(Iz, 2)-2*input.getPitch()*U*t*Iy*Iz - config.getHorStabLiftSlope()*config.getTailSize()*U*s2*Math.pow(t, 2)*Iy);
        	double sig1 = Math.sqrt(sig);
        	double sig3 = config.getHorStabLiftSlope()*config.getTailSize()*U*s*t*Iy;
        	double sig2 = sig3 + 2*config.getHorStabLiftSlope()*config.getTailSize()*U*s*t*Iz;
        	
        	double a1 = (sig1 + sig3- input.getPitch()*config.getHorStabLiftSlope()*config.getTailSize()*U*s*t*Iy - 2*input.getPitch()*config.getTailSize()*config.getHorStabLiftSlope()*U*s*t*Iz)/sig2;
        	double a2 = -(sig1 - sig3+ input.getPitch()*config.getHorStabLiftSlope()*config.getTailSize()*U*s*t*Iy + 2*input.getPitch()*config.getTailSize()*config.getHorStabLiftSlope()*U*s*t*Iz)/sig2;
        
        	horStabInclination = (float) a2;
        
        }
       
        return new AutopilotOutputs(thrust, leftWingInclination, rightWingInclination, horStabInclination, verStabInclination);
    }


}