package interfaces;

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
				ArrayList<Integer> HSVPixel = RBGToHSV(pixel);
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
	
	static ArrayList<Integer> RBGToHSV(ArrayList<Integer> pixel){
		float R = pixel.get(0)/255;
		float G = pixel.get(1)/255;
		float B = pixel.get(2)/255;
		float Cmax = Max(R,G,B);
		float Cmin = Min(R,G,B);
		float delta = Cmax- Cmin;
		//System.out.println("pixel: "+ pixel.get(0) +", "+ pixel.get(1) + ", " + pixel.get(2));
		//System.out.println("delta: "+ delta);
		ArrayList<Integer> HSVPixel = new ArrayList<Integer>();
		
		if (delta == 0)
			HSVPixel.add(0);
		else if (Cmax == R)
			HSVPixel.add((int) Math.floor((30*((G-B)/delta)%6)));
		else if (Cmax == G)
			HSVPixel.add((int) Math.floor((30*((B-R)/delta)+2)));
		else if (Cmax == B)
			HSVPixel.add((int) Math.floor((30*((R-G)/delta)+4)));
		
		if (Cmax == 0)
			HSVPixel.add(0);
		else
			HSVPixel.add((int) Math.floor(delta/Cmax));
		
		HSVPixel.add((int) Math.floor(Cmax));
//		System.out.println("pixel: "+ HSVPixel.get(0) +", "+ HSVPixel.get(1) + ", " + HSVPixel.get(2));
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
