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
	
	
}
