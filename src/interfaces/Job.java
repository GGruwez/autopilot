package interfaces;

import java.util.ArrayList;

public class Job {
	
	public Job(Airport from, int gateFrom, Airport to, int gateTo) {
		this.from = from;
		this.to = to;
		this.gateFrom = gateFrom;
		this.gateTo = gateTo;
		this.path = calculatePath();
	}
	
	public Airport getAirportFrom() {
		return this.from;
	}
	
	public Airport getAirportTo() {
		return this.to;
	}
	
	public AutopilotImplementation getDrone() {
		return this.drone;
	}
	
	public PathImplementation getPath(){
		return this.path;
	}
	
	public void setDrone(AutopilotImplementation drone) {
		if (! this.hasDrone()) {
			this.drone = drone;
		}
	}
	
	public boolean hasDrone() {
		return ! (this.getDrone() == null);
	}
	
	public int getGateFrom() {
		return this.gateFrom;
	}
	
	public int getGateTo() {
		return this.gateTo;
	}
	
	public PathImplementation calculatePath(){
		ArrayList<Vector> path = new ArrayList<Vector>();
		float takeoffLenght = 380;
		float turningRadius = 900;
		Vector startingPoint = new Vector(from.getCenterX()+takeoffLenght*from.getCenterToRunway0X(),20,from.getCenterZ()+ takeoffLenght*from.getCenterToRunway0Z());
		Vector centerRStart = new Vector((float)(startingPoint.getX() + turningRadius*from.getCenterToRunway0Z()),0,(float)(startingPoint.getZ() - turningRadius*from.getCenterToRunway0X()));
		Vector centerLStart = new Vector((float)(startingPoint.getX() - turningRadius*from.getCenterToRunway0Z()),0,(float)(startingPoint.getZ() + turningRadius*from.getCenterToRunway0X()));
		
		Vector endPoint = new Vector(to.getCenterX()-takeoffLenght*to.getCenterToRunway0X(),20,to.getCenterZ()- takeoffLenght*to.getCenterToRunway0Z());
		Vector centerREnd = new Vector((float)(endPoint.getX() + turningRadius*to.getCenterToRunway0Z()),0,(float)(endPoint.getZ() - turningRadius*to.getCenterToRunway0X()));
		Vector centerLEnd = new Vector((float)(endPoint.getX() - turningRadius*to.getCenterToRunway0Z()),0,(float)(endPoint.getZ() + turningRadius*to.getCenterToRunway0X()));
		
		if(startingPoint.getX() > endPoint.getX()) {
			//takeLeftStartCircle
			if(centerLStart.calculateDistance(centerLEnd) < centerLStart.calculateDistance(centerREnd)) {
				//takeLeftEndCircle
			}else {
				//takeRightEndCircle
			}
		}else {
			//takeRightCircle
			if(centerRStart.calculateDistance(centerLEnd) < centerRStart.calculateDistance(centerREnd)) {
				//takeLeftEndCircle
			}else {
				//takeRightEndCircle
			}
		}
		
		// if rightCircle to rightCircle or leftCircle to leftCircle -> tangent // lines between centers
		
		// if left to right
		

		float x1 = centerLStart.getX();
		float y1 = centerLStart.getZ();
		float x2 = centerREnd.getX();
		float y2 = centerREnd.getZ();
		float r = turningRadius;
		float d = centerREnd.subtract(centerLStart).euclideanLength()/2;
		
		
		float theta1 =  (-4*d*d + 16*r*r - 3*x1*x1 + 2*x1*x2 + x2*x2 -3*y1*y1 +2*y1*y2 + y2*y2)/(4*(x1-x2));
		float theta = 4*d*d*(y1 - y2) + 16*r*r*(y2-y1) +  x1*x1* (3*y1  +  y2) + x2*x2*(3*y1+y2) +y1*y2*y2 - 5*y1*y1*y2 + 3*y1*y1*y1 + y2*y2*y2 - 6*x1*x2*y1 - 2*x1*x2*y2;
		float theta5 = (float) Math.sqrt(((4*d*d +16 *d*r + 16*r*r - x1*x1 + 2*x1*x2 - x2*x2-y1*y1+2*y1*y2-y2*y2)*(-4*d*d + 16*d*r-16*r*r + x1*x1 -2*x1*x2 + x2*x2 + y1*y1 - 2*y1*y2 + y2*y2))/4);
		float theta6 = 2*(x2-x1)*theta5;
		float theta2 = theta + theta6;
		float theta3 = theta - theta6;
		float theta13 = x1*x1 - 2*x1*x2+x2*x2+y1*y1-2*y1*y2+y2*y2;
		float theta4 = 4*(x1-x2)*theta13;
		
		float tx1 = -theta1 - (y1-y2)*theta2/theta4;
		float ty1 = theta2/(4*theta13);
		float tx2 = -theta1 - (y1-y2)*theta3/theta4;
		float ty2 = theta3/(4*theta13);
		
		Vector t1 = new Vector(tx1,0,ty1);
		Vector offset1 = centerREnd.subtract(t1);
		t1 = centerLStart.add(t1.subtract(centerLStart).constantProduct(.5f));
		Vector t2 = new Vector(tx2,0,ty2);
		Vector offset2 = centerREnd.subtract(t2);
		t2 = centerLStart.add(t2.subtract(centerLStart).constantProduct(.5f));
		
//		centerLStart.printVector("start:  ");
//		centerREnd.printVector("end:  ");
//		t1.printVector("t1:  ");
//		t2.printVector("t2:  ");

		
		
		Vector r1 = t1.add(offset1);
		Vector r2 = t2.add(offset2);
//		r1.printVector("r1:  ");
//		r2.printVector("r2:  ");
		
		
		
		//get path on first circle
		
		Vector a1 = t2.subtract(centerLStart);
		Vector a2 = startingPoint.subtract(centerLStart);
		float angle = a2.angleBetween(a1);
		float arclenght = (float) (turningRadius*angle/(2*Math.PI));
		float nbPathentry = arclenght/20;
		float stepAngle = angle/nbPathentry;
		
		for (int i=0;i<nbPathentry;i++) {
			float x = (float) (centerLStart.getX() - Math.cos(stepAngle * i)*turningRadius);
			float y = (float) (centerLStart.getZ() - Math.sin(stepAngle * i)*turningRadius);
			Vector pathEntry = new Vector(x,20,y);
			path.add(pathEntry);

		}
		float x = (float) (centerLStart.getX() - Math.cos(angle)*turningRadius);
		float y = (float) (centerLStart.getZ() - Math.sin(angle)*turningRadius);
		Vector pathEntry = new Vector(x,20,y);
		path.add(pathEntry);
		
		// get path on tangent
		
		Vector tangent = r2.subtract(t2);
		nbPathentry = tangent.euclideanLength()/20;
		for (int i= 1; i <nbPathentry;i++) {
			float partOfTangent = i/nbPathentry;
			pathEntry = t2.add(tangent.constantProduct(partOfTangent));
			pathEntry = new Vector(pathEntry.getX(),20,pathEntry.getZ());
			path.add(pathEntry);
		}
		
		
		// get path on second circle
		
		a1 = r2.subtract(centerREnd);
		a2 = endPoint.subtract(centerREnd);
		angle = a2.angleBetween(a1);

		arclenght = (float) (turningRadius*angle/(2*Math.PI));
		nbPathentry = arclenght/20;
		stepAngle = angle/nbPathentry;
		
		for (int i=5;i<nbPathentry;i++) {
			x = (float) (centerREnd.getX() - Math.cos(stepAngle * i)*turningRadius);
			y = (float) (centerREnd.getZ() + Math.sin(stepAngle * i)*turningRadius);
			pathEntry = new Vector(x,20,y);
			path.add(pathEntry);
		}
	
		path.add(endPoint);
		
		return new PathImplementation(path);
	}
	
	private Airport from;
	private Airport to;
	private AutopilotImplementation drone = null;
	private int gateFrom;
	private int gateTo;
	private PathImplementation path;
}


