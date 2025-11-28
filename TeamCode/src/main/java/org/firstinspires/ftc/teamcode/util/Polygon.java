package org.firstinspires.ftc.teamcode.util;

public class Polygon {
	public Point.Two[] points;
	public Polygon(Point.Two[] points) {
		this.points = points;
	}
	public boolean contains(Point.Two point) {
		// Code converted to Java from https://stackoverflow.com/a/29915728
		// ray-casting algorithm based on
		// https://wrf.ecse.rpi.edu/Research/Short_Notes/pnpoly.html/pnpoly.html
    
    	int x = point.x; int y = point.y;
        Point.Two[] vs = this.points;
    	boolean inside = false;
    	for (int i = 0, j = vs.length - 1; i < vs.length; j = i++) {
        	int xi = vs[i].x, yi = vs[i].y;
        	int xj = vs[j].x, yj = vs[j].y;
        
        	boolean intersect = ((yi > y) != (yj > y))
            	&& (x < (xj - xi) * (y - yi) / (yj - yi) + xi);
        	if (intersect) inside = !inside;
    	}
    
    	return inside;
	}
}
