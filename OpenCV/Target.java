package jp.jaxa.iss.kibo.rpc.usa;

import java.util.Collections;
import java.util.Iterator;
import java.util.List;

import gov.nasa.arc.astrobee.types.Point;
import gov.nasa.arc.astrobee.types.Quaternion;

public class Target {

    //orientations indexes also off by one because start point orientation is not needed
    private static Quaternion[] orientations = new Quaternion[]{ (new Quaternion(0f, 0f, -0.707f, 0.707f)), (new Quaternion(0.5f, 0.5f, -0.5f, 0.5f)), (new Quaternion(0f, 0.707f, 0f, 0.707f)),
            (new Quaternion(0, 0, -1, 0)),  (new Quaternion(0f, 0.707f, 0f, 0.707f)), new Quaternion(0f, 0f, -0.707f, -0.707f)};

    private static Point[] reversePointHelper = {new Point(11.2146d, -9.68, 5.47), new Point(10.45, -9.21, 4.70),
            new Point(10.71, -7.763, 4.75), new Point(10.51, -6.6115, 5.2074),
            new Point(11.38d, -8.56d, 4.85)};

    private static Path path1 = new Path(47, new Point[]{ (new Point(10.6d, -10.0d, 5.2988d)), (new Point(11.2146d, -9.68, 5.47))}, orientations[0]);

    private static Path[] target2 = {new Path(21.6, new Point[]{new Point(10.45, -9.21, 4.70)}, orientations[1]),
            new Path(29, new Point[]{ new Point(10.45, -9.21, 4.70)} , orientations[1])}; //new Point(10.9, -9.2, 4.9),

    private static Path[] target3 = {new Path(50.9, new Point[]{new Point(10.71, -8.5, 4.75), new Point(10.71, -7.763, 4.75)}, orientations[2]),
            new Path(36.7, new Point[]{new Point(10.71, -7.763, 4.75)}, orientations[2]),
            new Path(31.5, new Point[]{new Point(10.71, -7.763, 4.75)}, orientations[2])};

    private static Path[] target4 = {new Path(62.6, new Point[]{new Point(10.5, -9.4, 4.7), new Point(10.51, -6.6115, 5.2074)}, orientations[3]),
            new Path(44, new Point[]{new Point(10.51, -6.6115, 5.2074)}, orientations[3]),
            new Path(40.7, new Point[]{new Point(10.51, -6.6115, 5.2074)}, orientations[3]),
            new Path(29.6, new Point[]{new Point(10.51, -6.6115, 5.2074)}, orientations[3])};


    private static Path[] QR = {new Path(20, new Point[]{new Point(11.28, -8.56, 4.85)}, orientations[4]),
            new Path(27, new Point[]{new Point(11.28, -8.56, 4.85)}, orientations[4]), //x 11.38
            new Path(26.1, new Point[]{new Point(11.28, -8.56, 4.85)}, orientations[4]),
            new Path(36, new Point[]{new Point(11.28, -8.56, 4.85)}, orientations[4])};

    private static Path[] goal = {new Path(40, new Point[]{new Point(11.143d, -6.7607d, 5.14d)}, orientations[5]),
            new Path(37.6, new Point[]{new Point(11.143d, -6.7607d, 4.9654d)}, orientations[5]),
            new Path(25.8, new Point[]{new Point(11.143d, -6.7607d, 4.9654d)}, orientations[5]),
            new Path(21.2, new Point[]{new Point(11.143d, -6.7607d, 4.9654d)}, orientations[5]),
            new Path(37, new Point[]{new Point(11.143d, -6.7607d, 4.9654d)}, orientations[5])};
    
    //QR = target 7
    //goal = target 8
    //QR & goal indexes are off by one since there is no reason to go straight to the qr code from the start

    private static boolean[] targetCalibration = {false, false, false, false};
    private Target() {};

    public static Path getPath(int currTarget, int nextTarget){
        if(currTarget == nextTarget){
            return null;
        }

        if(currTarget > nextTarget){
            Path p = getPath(nextTarget, currTarget);//reversePath(getPath(nextTarget, currTarget));
            //p.setQuaternion(orientations[nextTarget-1]);
            Point[] pts = new Point[p.getPoints().length];

            pts[pts.length-1] = reversePointHelper[nextTarget-1];
            if(pts.length == 2){
                pts[0] = p.getPoints()[0];
            }


            return (new Path(p.getDistance(), pts, orientations[nextTarget-1]));
        }
        switch(nextTarget){
            case 1:
                if(currTarget==0){return path1;}
                else{return null;}
                //break;
            case 2:
                return target2[currTarget];
                //break;
            case 3:
                return target3[currTarget];
                //break;
            case 4:
                return target4[currTarget];
                //break;
            case 5:
                return QR[currTarget-1];
            case 6:
                return goal[currTarget-1];
            default:
                return null;
                //break;
        }
    }

    public static List<Integer> planPath(List<Integer> targets, int currTarget){
        if(targets.size() == 1){
            return targets;
        }
        if(targets.size() == 2){
            Iterator<Integer> it = targets.iterator();
            int nextTarget = it.next();
            int nextNextTarget = it.next();
            double dist1 = getPath(currTarget, nextTarget).getDistance() + getPath(nextTarget, nextNextTarget).getDistance();
            double dist2 = getPath(currTarget, nextNextTarget).getDistance() + getPath(nextNextTarget, nextTarget).getDistance();

            if(dist1<=dist2){
                return targets;
            }if(dist1>dist2){
                Collections.reverse(targets);
                return targets;
            }
        }

        return targets;
    }
    public static List<Integer> planPath(List<Integer> targets, int currTarget, long timeLeft, boolean qr){
        if(targets.size() == 1){
            return targets;
        }
        Iterator<Integer> it = targets.iterator();
        int nextTarget = it.next();
        int nextNextTarget = it.next();
        if(qr && timeLeft < 90000){

            double dist1 = getPath(currTarget, nextTarget).getDistance() + getPath(nextTarget, nextNextTarget).getDistance() + getPath(nextNextTarget, 6).getDistance();
            double dist2 = getPath(currTarget, nextNextTarget).getDistance() + getPath(nextNextTarget, nextTarget).getDistance() + getPath(nextTarget, 6).getDistance();

            if(dist1<=dist2 && dist1<90){
                return targets;
            }if(dist1>dist2 && dist2<90){
                Collections.reverse(targets);
                return targets;
            }
        }
        double dist1 = getPath(currTarget, nextTarget).getDistance() + getPath(nextTarget, nextNextTarget).getDistance();
        double dist2 = getPath(currTarget, nextNextTarget).getDistance() + getPath(nextNextTarget, nextTarget).getDistance();

        if(dist1<=dist2){
            return targets;
        }if(dist1>dist2){
            Collections.reverse(targets);
            return targets;
        }
        return targets;
    }

    public static Long nextTargetTime(int currTarget, int nextTarget){
        if(currTarget == nextTarget)
            return 0L;
        return (long)((getPath(currTarget, nextTarget)).getDistance() * 1000);
    }
    public static Long qrTime(int nextTarget, boolean readQR){
        if(!readQR){
            return (long)(((getPath(nextTarget, 5)).getDistance() + getPath(5, 6).getDistance())* 1000);
        }
        return (long)(getPath(nextTarget, 6).getDistance())* 1000;
    }

    public static double distance(Point pt1, Point pt2){
        double deltaX = pt1.getX()-pt2.getX();
        double deltaY = pt1.getY()-pt2.getY();
        double deltaZ = pt1.getZ()-pt2.getZ();
        return Math.sqrt(deltaX*deltaX + deltaY*deltaY + deltaZ*deltaZ);
    }

//    public boolean acceptableDistance(float point, int targ){
//        int distance = 4;
//        switch(targ){
//            case 1:
//                distance = 0;
//                break;
//
//        }
//
//    }

    public static boolean isTargetCalibrated(int target){return targetCalibration[target-1];}

    public static void calibrateTarget(int target, Point pt){
        //if(targetCalibration[target-1]){return;}
        //targetCalibration[target-1]=true;

        reversePointHelper[target-1] = pt;

        switch(target){
            case 1:
                path1.setLastPoint(pt);
                return;
            case 2:
                for(int i=0; i<target2.length; i++){
                    target2[i].setLastPoint(pt);
                }
                return;
            case 3:
                for(int i=0; i<target3.length; i++){
                    target3[i].setLastPoint(pt);
                }
                return;
            case 4:
                for(int i=0; i<target4.length; i++){
                    target4[i].setLastPoint(pt);
                }
                return;

            default:
                return;

    }}
}
