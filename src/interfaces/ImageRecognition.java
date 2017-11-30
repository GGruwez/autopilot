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
				ArrayList<Float> HSVPixel = RBGToHSV(pixel);
				if (((HSVPixel.get(0) <= 8) || (HSVPixel.get(0)>= 172)) && (HSVPixel.get(1) >= 0.01) && (HSVPixel.get(2)>0.01)){
					positions.add(row*nbColumns+column);
					//System.out.println("position added: " + position/3);
				}
			}	
		}	
		
		if (positions.size()==0) {
			return null;
		}
		
		ArrayList<Integer> filteredPositions = filterValues(positions,image);
		
		
		int maxColumn = 0;
		int minColumn = nbColumns;
		int maxRow = 0;
		int minRow = nbRows;
		for (int position :filteredPositions){
			
			//
//			ArrayList<Integer> pixel = new ArrayList<Integer>();
//			pixel.add(image[3*(position)+0] & 0xff) ;
//			pixel.add(image[3*(position)+1] & 0xff);
//			pixel.add(image[3*(position)+2] & 0xff);
//			ArrayList<Float> HSVPixel = RBGToHSV(pixel);
//			System.out.println("HSV: "+ HSVPixel.get(0) + " " + HSVPixel.get(1) + " " + HSVPixel.get(2));
			//

//			System.out.println("currow: " + Math.floor(position/nbColumns));

			int currentRow = position/nbColumns;
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
	
		
		int size = ((maxRow - minRow) == 0 ? 1 : (maxRow - minRow+1));
		float distance = (float) (0.7*200/size);
//		float theta = (float) (Math.PI/3 * (size)/(nbRows/2));
//		float distance = (float) (1/Math.tan(theta));
//		size = ((maxColumn - minColumn) == 0 ? 1 : (maxColumn - minColumn+1));
//		distance = (distance > (float) (Math.PI/3 * (size)/(nbColumns/2)) ? distance : (float) (Math.PI/3 * (size)/(nbColumns/2)));
//		//System.out.println("nb filtered: " + (positions.size()-filteredPositions.size()));
		//System.out.println("distance: " + 286.06*Math.pow( filteredPositions.size(), -0.638));

//		System.out.println("distance: " + distance);

		float xVector = minColumn + (maxColumn - minColumn)/2 - nbColumns/2;
		float yVector = minRow + (maxRow - minRow)/2 - nbRows/2;
//		System.out.println( maxColumn-minColumn+1);
//		System.out.println(maxRow - minRow+1);
		//System.out.println(xVector + "   "+ yVector);
		//System.out.println(size);
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
		
//		if (HSVPixel.get(2) != 0)
//		System.out.println("V: " + HSVPixel.get(2));
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
			
	
	static ArrayList<Integer> filterValues(ArrayList<Integer> positions, byte[] image){
//		ArrayList<Integer> pX = new ArrayList<Integer>();
//		ArrayList<Integer> nX = new ArrayList<Integer>();
//		ArrayList<Integer> pY = new ArrayList<Integer>();
//		ArrayList<Integer> nY = new ArrayList<Integer>();
//		ArrayList<Integer> pZ = new ArrayList<Integer>();
//		ArrayList<Integer> nZ = new ArrayList<Integer>();
		
		ArrayList<ArrayList<Integer>> pos = new ArrayList<ArrayList<Integer>>();
		for (int i=0; i<6 ;i++){
			ArrayList<Integer> side = new ArrayList<Integer>();
			pos.add(side);
		}
		
		for (Integer position : positions){
			ArrayList<Integer> pixel = new ArrayList<Integer>();
			pixel.add(image[3*(position)+0] & 0xff) ;
			pixel.add(image[3*(position)+1] & 0xff);
			pixel.add(image[3*(position)+2] & 0xff);
			ArrayList<Float> HSVPixel = RBGToHSV(pixel);
			float value= HSVPixel.get(2);

//			System.out.println("HSV: "+ HSVPixel.get(0) + " " + HSVPixel.get(1) + " " + HSVPixel.get(2));

			if (value <=0.225f){
				pos.get(0).add(position); //nY
			}else if (value <= 0.375f){
				pos.get(1).add(position); //nX
			}else if (value <= 0.59f){
				pos.get(2).add(position); //nZ
			}else if (value <= 0.775f){
				pos.get(3).add(position); //pZ
			}else if (value <= 0.925f){
				pos.get(4).add(position);//pX
			}else{
				pos.get(5).add(position);//pY
			}			
		}

//		System.out.println("done");
		ArrayList<Integer> high = new ArrayList<Integer>();
		for (ArrayList<Integer> side :pos){
//			System.out.println(side.size());

			if (side.size() > high.size())
				high = side;
		}
		
		
//		for (Integer position : positions){
//			float value= (image[3*(position)+2] & 0xff)/255f;
//			
//			if (value <=0.225f){
//				nY.add(position);
//			}else if (value <= 0.375f){
//				nX.add(position);
//			}else if (value <= 0.575f){
//				nZ.add(position);
//			}else if (value <= 0.775f){
//				pZ.add(position);
//			}else if (value <= 0.925f){
//				pX.add(position);
//			}else{
//				pY.add(position);
//			}			
//		}
		
		
		return pos.get(3);
	}
}

