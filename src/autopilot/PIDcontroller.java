package autopilot;

public class PIDcontroller {
	
	private float Kp;
	private float Ki;
	private float Kd;
	private float errorSum;
	private float errorPrevious;
	
	PIDcontroller(float Kp, float Ki, float Kd) {
		this.setControl(Kp, Ki, Kd);
	}
	
	public void setControl(float Kp, float Ki, float Kd) {
		this.Kp = Kp;
		this.Ki = Ki;
		this.Kd = Kd;
	}
	
	public float getErrorSum() {
		return this.errorSum;
	}
	
	public void setErrorSum(float sum) {
		this.errorSum = sum;
	}
	
	public float getPreviousError() {
		return this.errorPrevious;
	}
	
	public void setPreviousError(float error) {
		this.errorPrevious = error;
	}
	
	public float getOutput(float y, float ref) {
		float error = ref-y;
		float delta = error-getPreviousError();
		setErrorSum(getErrorSum()+error);
		setPreviousError(error);
		return Kp*(error) + Ki*getErrorSum() + Kd*(delta);
	}

}
