package jp.jaxa.iss.kibo.rpc.Salcedo;

import android.util.Log;

import gov.nasa.arc.astrobee.Kinematics;
import jp.jaxa.iss.kibo.rpc.api.KiboRpcService;

import gov.nasa.arc.astrobee.types.Point;
import gov.nasa.arc.astrobee.types.Quaternion;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class Path {
    private String ID;
    private double distance;
    private ArrayList<Point> points = new ArrayList<Point>();
    private Quaternion quaternion;

    public Path(String id, double d, ArrayList<Point> p, Quaternion q){
        ID = id;
        distance = d;
        points = p;
        quaternion = q;
    }

    public ArrayList<Point> getPoints(){
        return  points;
    }

    public double getDistance(){
        return distance;
    }

    public Quaternion getQuaternion(){
        return quaternion;
    }

}
