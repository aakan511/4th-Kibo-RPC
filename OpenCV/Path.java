package jp.jaxa.iss.kibo.rpc.usa;

import gov.nasa.arc.astrobee.types.Point;
import gov.nasa.arc.astrobee.types.Quaternion;

public class Path {
    //private String ID;
    private double distance;
    private Point[] points = {new Point(0,0,0)};
    private Quaternion quaternion;

    public Path(double d, Point[] p, Quaternion q){
        distance = d;
        points = p;
        quaternion = q;
    }
    public Path(){}

    public Point[] getPoints(){
        return  points;
    }

    public double getDistance(){
        return distance;
    }

    public Quaternion getQuaternion(){
        return quaternion;
    }

    public void setQuaternion(Quaternion q){
        quaternion = q;
    }

    public void setLastPoint(Point pt){
        points[points.length-1] = pt;
    }
}
