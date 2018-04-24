package interfaces;

import java.util.ArrayList;

public class PathImplementation implements Path {

	@Override
	public float[] getX() {
		return this.X;
	}

	@Override
	public float[] getY() {
		return this.Y;
	}

	@Override
	public float[] getZ() {
		return this.Z;
	}
	
//	public float[] X = new float[]{0,0,0,0};
//	public float[] Y = new float[]{30,30,30,30};
//	public float[] Z = new float[]{-2000,-600,-500,-400};
	public float[] X = new float[]{0};
	public float[] Y = new float[]{30};
	public float[] Z = new float[]{-800};
	
	public void addPath(Vector[] path) {
		
		float[] tempX = new float[this.X.length+path.length];
		float[] tempY = new float[this.Y.length+path.length];
		float[] tempZ = new float[this.Z.length+path.length];
		for (int i=0;i<this.X.length+path.length;i++) {
			if (i<X.length -1 ) {
				tempX[i] = X[i];
				tempY[i] = Y[i];
				tempZ[i] = Z[i];
				
			}else {
				tempX[i] = path[i].getX();
				tempY[i] = path[i].getY();
				tempZ[i] = path[i].getZ();
			}
			
		}
		
	}
}
