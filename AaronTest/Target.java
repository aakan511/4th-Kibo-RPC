package jp.jaxa.iss.kibo.rpc.sampleapk;

import gov.nasa.arc.astrobee.types.Point;
import gov.nasa.arc.astrobee.types.Quaternion;

import static jp.jaxa.iss.kibo.rpc.sampleapk.Path.reversePath;

public class Target {

    private static Quaternion[] orientations = new Quaternion[]{(new Quaternion()), (new Quaternion(0f, 0f, -0.707f, 0.707f)), (new Quaternion(0.5f, 0.5f, -0.5f, 0.5f)), (new Quaternion(0f, 0.707f, 0f, 0.707f)),
            (new Quaternion(0, 0, -1, 0)), (new Quaternion(-0.5f, -0.5f, -0.5f, 0.5f)), (new Quaternion(0, 0, 0, 1)), (new Quaternion(0, 0.707f, 0, 0.707f)) };

    private static Path path1 = new Path(0.0, new Point[]{(new Point(10.6d, -10.0d, 5.2988d)), (new Point(11.2146d, -9.92284, 5.47))}, orientations[1]);

    private static Path[] target2 = {new Path(0.0, null, new Quaternion()), new Path()};

    private static Path[] target3 = {new Path(0.0, null, new Quaternion()), new Path(), new Path()};

    private static Path[] target4 = {new Path(0.0, null, new Quaternion()), new Path(), new Path(), new Path()};

    private static Path[] target5 = {new Path(0.0, null, new Quaternion()), new Path(), new Path(), new Path(), new Path()};

    private static Path[] target6 = {new Path(0.0, null, new Quaternion()), new Path(), new Path(), new Path(), new Path(), new Path()};

    private static Path[] QR = {new Path(0.0, null, new Quaternion()), new Path(), new Path(), new Path(), new Path(), new Path()};

    //QR = target 7
    //QR indexes are off by one since there is no reason to go straight to the qr code from the start
    private Target() {};

    public static Path getPath(int currTarget, int nextTarget){
        if(currTarget == nextTarget){
            return null;
        }

        if(currTarget > nextTarget){
            Path p = reversePath(getPath(nextTarget, currTarget));
            p.setQuaternion(orientations[nextTarget]);
            return p;
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
                return target5[currTarget];
                //break;
            case 6:
                return target6[currTarget];
                //break;
            case 7:
                return QR[currTarget-1];
            default:
                return null;
                //break;
        }
    }

}
