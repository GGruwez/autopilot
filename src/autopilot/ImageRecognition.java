package autopilot;

import java.util.ArrayList;

class ImageRecognition {


	public static float[] FindTarget(byte[] image, int nbColumns, int nbRows ){
		ArrayList<Integer> positions = new ArrayList<Integer>();
//		System.out.println(image.length);
		for (int row=0; row<nbRows; row+=1) {
			for (int column =0; column<nbColumns ; column +=1){
				ArrayList<Integer> pixel = new ArrayList<Integer>();

//				System.out.println((image[3*(row*nbColumns+column)+0] & 0xff) + "   "+ (3*(row*nbColumns+column)+0));
//				System.out.println((image[3*(row*nbColumns+column)+1] & 0xff) + "   " + (3*(row*nbColumns+column)+1));
//				System.out.println((image[3*(row*nbColumns+column)+2] & 0xff)+ "   " + (3*(row*nbColumns+column)+2));
				pixel.add(image[3*(row*nbColumns+column)+0] & 0xff) ;
				pixel.add(image[3*(row*nbColumns+column)+1] & 0xff);
				pixel.add(image[3*(row*nbColumns+column)+2] & 0xff);
//				System.out.println("pixel: "+ pixel.get(0) +", "+ pixel.get(1) + ", " + pixel.get(2));
				ArrayList<Float> HSVPixel = RBGToHSV(pixel);
				if (((HSVPixel.get(0) <= 8) || (HSVPixel.get(0)>= 172)) && (HSVPixel.get(1) >= 0.3) && (HSVPixel.get(2)>0.3)){
					positions.add(row*nbColumns+column);
					//System.out.println("position added: " + position/3);
				}
			}	
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
		//System.out.println(minColumn + "   "+ maxColumn);
		//System.out.println(xVector + "   "+ yVector);
		return new float[]{xVector,yVector};

		
	}
	
	static ArrayList<Float> RBGToHSV(ArrayList<Integer> pixel){
		float R = pixel.get(0)/255f;
		float G = pixel.get(1)/255f;
		float B = pixel.get(2)/255f;
		float Cmax = Max(R,G,B);
		
//		if (Cmax != 0)
//		System.out.println("Cmax: " + Cmax);
		float Cmin = Min(R,G,B);
//		if (Cmax != 0)
//			System.out.println("Cmin: " + (Cmax == R));
		float delta = Cmax- Cmin;
		//if (pixel.get(0) != 0)
		//System.out.println("pixel: "+ pixel.get(0) +", "+ pixel.get(1) + ", " + pixel.get(2));
		//System.out.println("delta: "+ delta);
		ArrayList<Float> HSVPixel = new ArrayList<Float>();
		
		if (delta == 0)
			HSVPixel.add(0f);
		else if (Cmax == R)
			HSVPixel.add((30*((G-B)/delta)%6));
		else if (Cmax == G)
			HSVPixel.add((30*((B-R)/delta)+2));
		else if (Cmax == B)
			HSVPixel.add((30*((R-G)/delta)+4));
//		if (HSVPixel.get(0) != 0)
//		System.out.println(HSVPixel.get(0));
//		
		if (Cmax == 0){
			HSVPixel.add(0f);}
		else{
			HSVPixel.add(delta/Cmax);}		
		
		HSVPixel.add(Cmax);
		
		if (HSVPixel.get(2) != 0)
		System.out.println("V: " + HSVPixel.get(2));
		return HSVPixel;
	}
		
	static float Max(float R, float B, float G){
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
	
	static float Min(float R, float B, float G){
		
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
