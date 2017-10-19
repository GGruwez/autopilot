package autopilot;/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/**
 *
 * @author Stien
 */
public class AutopilotOutputs implements p_en_o_cw_2017.AutopilotOutputs{
    
    private float thrust;
    private float leftWingInclination;
    private float rightWingInclination;
    private float horStabInclination;
    private float verStabInclination;
    
    public AutopilotOutputs(float thrust, float leftIncl, float rightIncl, float horIncl, float verIncl) {
    	this.thrust = thrust;
    	this.leftWingInclination = leftIncl;
    	this.rightWingInclination = rightIncl;
    	this.horStabInclination = horIncl;
    	this.verStabInclination = verIncl;
    }
    
    public AutopilotOutputs() {
    	new AutopilotOutputs(0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
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
    
}