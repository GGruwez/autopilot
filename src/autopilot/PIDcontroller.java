package autopilot;

public class PIDcontroller {
	
	private float K = (float) (Math.PI/9);
	private float Ti = 1;
	private float Td;
	private float processValue;
	private float reference;
	private float alfa = 0.1f;
	private float previousEDf;
	private float previousPreviousEDf;
	private float previousError;
	private float previousOutput;
	private float Ts;
	
	public float getEDf() {
		return previousEDf/(Ts/getTf()+1) + getError()*Ts/getTf()/(Ts/getTf()+1);
	}
	
	public float getTf() {
		return alfa*Td;
	}
	
	public float getDelta() {
		float P = getError()-previousError;
		float I = Ts/Ti*getError();
		float D = Td/Ts*(getEDf()-2*previousEDf+previousPreviousEDf);
		return K*(P+I+D);
	}
	
	public float getError() {
		return processValue-reference;
	}
	
	public void setProcessValue(float value) {
		this.processValue = value;
	}

	public void setTs(float dt) {
		this.Ts = dt;
	}
	
	public float getOutput(float dt, float processValue) {
		setTs(dt);
		setProcessValue(processValue);
		float output = getDelta()+previousOutput;
		previousOutput = output;
		previousError = getError();
		previousPreviousEDf = previousEDf;
		previousEDf = getEDf();
		return output;
	}
	
	
	
}
