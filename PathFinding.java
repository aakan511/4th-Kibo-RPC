package jp.jaxa.iss.kibo.rpc.Salcedo;

import android.graphics.Bitmap;
import android.graphics.Matrix;
import android.util.Log;

import gov.nasa.arc.astrobee.Kinematics;
import jp.jaxa.iss.kibo.rpc.api.KiboRpcService;

import gov.nasa.arc.astrobee.Result;
import gov.nasa.arc.astrobee.types.Point;
import gov.nasa.arc.astrobee.types.Quaternion;

import org.opencv.calib3d.Calib3d;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.aruco.*;
import org.opencv.core.CvType;
import org.opencv.core.MatOfDouble;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.MatOfPoint3f;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.objdetect.QRCodeDetector;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.core.Rect;
import org.opencv.core.MatOfRect;


import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Map;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.Queue;


public class PathFinding {
    private String path, optimal;
    Map<String,Path> allPaths = new HashMap<String, Path>();

    public PathFinding(List<Integer> targetList){
        path = "0" + intListToString(targetList);
        definePaths();
        findOptimalPath();

    }

    public void definePaths(){

        /*
        PATHS FROM POINT 0
         */

        // Point 0 to Point 1
        ArrayList<Point> p = new ArrayList<Point>();
        p.add(new Point(11.2746d,-9.2284d, 4.988d));
        p.add(new Point(11.2746d,-9.92284d, 5.2988d));
        Quaternion quaternion = new Quaternion(0,0,-0.707f,0.707f);
        allPaths.put("01", new Path("01", 2.84, p, quaternion));

        // Point 0 to Point 2
        p.clear();
        p.add(new Point(10.612d,-9.0709d, 4.48d));
        // Quaternion quaternion = new Quaternion(0,0,-0.707f,0.707f);
        allPaths.put("02", new Path("02", 2.84, p, quaternion));

        // Point 0 to Point 3
        p.clear();
        p.add(new Point(10.71d,-8.2d,1.98d));
        p.add(new Point(10.71d,-7.7d,1.48d));
        // Quaternion quaternion = new Quaternion(0,0,-0.707f,0.707f);
        allPaths.put("02", new Path("02", 2.84, p, quaternion));

         /*
        PATHS FROM POINT 1
         */

    }

    public void findOptimalPath(){
        String options = path.substring(1);
        String add = path.substring(0,1);
        double optimal_distance = 0.0;
        Queue<String> possibles = new LinkedList<>();
        possibles.add(path.substring(0,1));
        // Defines "possibles" with every possible path given the targets
        while (add.length() <= path.length()){
            for (int i = 0; i < options.length(); i++) {
                if (!(add.contains("" + options.charAt(i)))){
                    possibles.add(add + options.charAt(i));
                }
            }
            add = possibles.remove();
        }
        optimal = possibles.remove();
        optimal_distance = getPathDistance(optimal);
        // Sets optimal to the path with the lowest distance
        while (!(possibles.isEmpty())){
            String that = possibles.remove();
            double that_distance = getPathDistance(that);
            if (that_distance < optimal_distance){
                optimal = that;
                optimal_distance = that_distance;
            }
        }

    }

    public double getPathDistance(String p){
        String key;
        double total = 0.0;
        for(int i = 0; i < p.length() - 1; i++){
            key = "" + p.charAt(i) + p.charAt(i + 1);
            total+=allPaths.get(key).getDistance();
        }
        return total;
    }

    private String intListToString(List<Integer> list){
        String ret = "";
        for(Integer i: list){
            ret+=String.valueOf(i);
        }
        return ret;
    }
}
