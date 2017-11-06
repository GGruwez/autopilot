package autopilot;

public class PIDcontroller {
	
	private float K ;
	private float Ti;
	private float Td;
	private float processValue;
	private float reference;
	private float alfa = 0.1f;
	private float previousEDf;
	private float previousPreviousEDf;
	private float previousError;
	private float previousOutput;
	private float Ts;
	
	PIDcontroller(float k , float ti, float td){
		this.setConfig(k, ti, td);
	}
	
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
	

	private void setConfig(float K, float Ti, float Td){
		this.K = K;
		this.Ti= Ti;
		this.Td = Td;
	}
	
	public void setProcessValue(float value) {
		this.processValue = value;
	}
	
	public void setReference(float reference){
		this.reference = reference;
	}

	public void setTs(float dt) {
		this.Ts = dt;
	}
	
	public float getOutput(float dt, float processValue, float reference) {
		setTs(dt);
		setProcessValue(processValue);
		setReference(reference);
		float output = getDelta()+previousOutput;
		previousOutput = output;
		previousError = getError();
		previousPreviousEDf = previousEDf;
		previousEDf = getEDf();
		return output;
	}
	
	
	
}
