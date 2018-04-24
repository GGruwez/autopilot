package interfaces;/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

public class AutopilotOutputsImplementation implements AutopilotOutputs {
    
    private float thrust;
    private float leftWingInclination;
    private float rightWingInclination;
    private float horStabInclination;
    private float verStabInclination;
    private float frontBrake;
    private float leftBrake;
    private float rightBrake;
    
    AutopilotOutputsImplementation(float thrust, float leftIncl, float rightIncl, float horIncl, float verIncl, float frontBrake, float leftBrake, float rightBrake) {
    	this.thrust = thrust;
    	this.leftWingInclination = leftIncl;
    	this.rightWingInclination = rightIncl;
    	this.horStabInclination = horIncl;
    	this.verStabInclination = verIncl;
    	this.frontBrake = frontBrake;
    	this.leftBrake = leftBrake;
    	this.rightBrake = rightBrake;
    }
    
    AutopilotOutputsImplementation() {
    	new AutopilotOutputsImplementation(0.0f, 0.0f, 0.0f, 0.0f, 0.0f ,0f,0f,0f);
    }

    @Override
    public float getThrust() {
        return this.thrust;
    }

    @Override
    public float getLeftWingInclination() {
        return this.leftWingInclination;
    }

    @Override
    public float getRightWingInclination() {
        return this.rightWingInclination;
    }

    @Override
    public float getHorStabInclination() {
        return this.horStabInclination;
    }

    @Override
    public float getVerStabInclination() {
        return this.verStabInclination;
    }

	@Override
	public float getFrontBrakeForce() {
		return this.frontBrake;
	}

	@Override
	public float getLeftBrakeForce() {
		return this.leftBrake;
	}

	@Override
	public float getRightBrakeForce() {
		return this.rightBrake;
	}
    
}