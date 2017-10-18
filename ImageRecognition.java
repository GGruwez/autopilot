package autopilot;

import java.util.ArrayList;

public class ImageRecognition {

	
	public float[] FindTarget(byte[] image, int nbColumns, int nbRows ){
		ArrayList<Integer> positions = new ArrayList<Integer>();
		for (int position=0; position<nbColumns*nbRows; position+=3) {
			byte[] pixel = new byte[3];
			pixel[0] = image[position];
			pixel[1] = image[position+1];
			pixel[2] = image[position+2];
			
			byte[] HSVPixel = RBGToHSV(pixel);
			if (((HSVPixel[0] <= 8) || (HSVPixel[0]>= 172)) && (HSVPixel[1] >= 100) && (HSVPixel[2] >38))
				positions.add(position/3);
		}	
		int maxColumn = 0;
		int minColumn = nbColumns;
		int maxRow = 0;
		int minRow = nbRows;
		for (int position :positions){
			int currentRow = (int) Math.floor(position/nbColumns);
			int currentColumn = position%nbColumns;
			
			if (currentRow > maxRow)
				maxRow = currentRow;
			if (currentRow < minRow)
				minRow = currentRow;
			if (currentColumn > maxColumn)
				maxColumn = currentColumn;
			if (currentColumn < minColumn)
				minColumn = currentColumn;
		}
		
		float xVector = minColumn + (maxColumn - minColumn)/2 - nbColumns/2;
		float yVector = minRow + (maxRow - minRow)/2 - nbRows/2;
		
		
		return new float[]{xVector,yVector} ;
		
	}
	
	public byte[] RBGToHSV(byte[] pixel){
		float R = pixel[0]/255;
		float G = pixel[1]/255;
		float B = pixel[2]/255;
		float Cmax = Max(R,G,B);
		float Cmin = Min(R,G,B);
		float delta = Cmax- Cmin;
		byte[] HSVPixel = new byte[3];
		
		if (delta == 0)
			HSVPixel[0] = 0;
		else if (Cmax == R)
			HSVPixel[0] = (byte) Math.floor((30*((G-B)/delta)%6));
		else if (Cmax == G)
			HSVPixel[0] = (byte) Math.floor((30*((B-R)/delta)+2));
		else if (Cmax == B)
			HSVPixel[0] = (byte) Math.floor((30*((R-G)/delta)+4));
		
		if (Cmax == 0)
			HSVPixel[1] = 0;
		else
			HSVPixel[1] = (byte) Math.floor(delta/Cmax*255);
		
		HSVPixel[2] = (byte) Math.floor(Cmax);
		
		return HSVPixel;
	}
		
	public float Max(float R, float B, float G){
		if (R>B){
			if (R>G)
				return R;
			else
				return G;
			}
		else{
			if (B>G)
				return B;
			else
				return G;	
		}
	}
	
	public float Min(float R, float B, float G){
		
		if (R<B){
			if (R<G)
				return R;
			else
				return G;
			}
		else{
			if (B<G)
				return B;
			else
				return G;
			
		}
		
	}
			
}
