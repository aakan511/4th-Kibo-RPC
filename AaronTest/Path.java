package jp.jaxa.iss.kibo.rpc.sampleapk;

import java.util.Arrays;

import gov.nasa.arc.astrobee.types.Point;
import gov.nasa.arc.astrobee.types.Quaternion;

public class Path {
    //private String ID;
    private double distance;
    private Point[] points;
    private Quaternion quaternion;

    public Path(double d, Point[] p, Quaternion q){
        distance = d;
        points = p;
        quaternion = q;
    }
    public Path(){
    }

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

    public static Path reversePath(Path p){
        Point[] points = Arrays.copyOf(p.getPoints(), p.getPoints().length);
        for(int i=0; i<points.length/2; i++){
            Point temp = points[i];
            points[i] = points[points.length -i -1];
            points[points.length -i -1] = temp;
        }
        return new Path(p.getDistance(), points, null);
    }

}
