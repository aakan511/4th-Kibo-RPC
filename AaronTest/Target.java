package jp.jaxa.iss.kibo.rpc.sampleapk;

import gov.nasa.arc.astrobee.types.Point;
import gov.nasa.arc.astrobee.types.Quaternion;

public class Target {

    //orientations indexes also off by one because start point orientation is not needed
    private static Quaternion[] orientations = new Quaternion[]{ (new Quaternion(0f, 0f, -0.707f, 0.707f)), (new Quaternion(0.5f, 0.5f, -0.5f, 0.5f)), (new Quaternion(0f, 0.707f, 0f, 0.707f)),
            (new Quaternion(0, 0, -1, 0)), (new Quaternion(-0.5f, -0.5f, -0.5f, 0.5f)), (new Quaternion(0, 0, 0, 1)), (new Quaternion(0, 0.707f, 0, 0.707f)) };

    private static Path path1 = new Path(0.0, new Point[]{(new Point(10.6d, -10.0d, 5.2988d)), (new Point(11.2146d, -9.92284, 5.47))}, orientations[0]);

    private static Path[] target2 = {new Path(0.0, new Point[]{new Point(10.5, -9.21, 4.51)}, orientations[1]), new Path(1.602105, new Point[]{new Point(10.5, -9.0709, 4.75), new Point(10.5, -9.21, 4.51)} , orientations[1])};

    private static Path[] target3 = {new Path(0.0, new Point[]{new Point(10.71, -8.5, 4.75), new Point(10.71, -7.763, 4.75)}, orientations[2]), new Path(0.0, new Point[]{new Point(10.71, -8.5, 4.75), new Point(10.71, -7.763, 4.75)}, orientations[2]), new Path(1.515332, new Point[]{new Point(10.71, -8.5, 4.75), new Point(10.71, -7.763, 4.75)}, orientations[2])};

    private static Path[] target4 = {new Path(0.0, new Point[]{new Point(10.5, -9.4, 4.7), new Point(10.51, -6.6115, 5.2074)}, orientations[3]),
            new Path(0.0, new Point[]{new Point(10.51, -6.6115, 5.2074)}, orientations[3]),
            new Path(0.0, new Point[]{new Point(10.51, -8.4, 4.7), new Point(10.51, -6.6115, 5.2074)}, orientations[3]),
            new Path(0.0, new Point[]{new Point(10.51, -6.6115, 5.2074)}, orientations[3])};

    private static Path[] target5 = {new Path(0.0, new Point[]{new Point(10.5, -9.4, 4.7) , new Point(11.047,-7.9156,5.05)}, orientations[4]),
            new Path(0.0, new Point[]{new Point(11.047,-7.9156,5.3393)}, orientations[4]),
            new Path(0.0, new Point[]{new Point(10.76, -8.5, 5), new Point(11.047,-7.9156,5.05)}, orientations[4]),
            new Path(0.0, new Point[]{ new Point(11.047,-7.9156,5.05)},orientations[4]),
            new Path(0.0, new Point[]{new Point(11.047,-7.9156,5.2074)},orientations[4])};

    private static Path[] target6 = {new Path(0.0, new Point[]{new Point( 11.405, -9.05, 4.94)} , orientations[5]),
            new Path(0.0, new Point[]{new Point( 11.405, -9.05, 4.94)}, orientations[5]),
            new Path(0.0, new Point[]{new Point( 11.405, -9.05, 4.94)}, orientations[5]),
            new Path(0.0, new Point[]{new Point( 11.405, -9.05, 4.94)}, orientations[5]),
            new Path(0.0, new Point[]{new Point( 11.405, -9.05, 4.94)}, orientations[5]),
            new Path(0.0, new Point[]{new Point( 11.405, -9.05, 4.94)}, orientations[5])};

    private static Path[] QR = {new Path(0.0, new Point[]{new Point(11.38, -8.56, 4.85)}, orientations[6]), new Path(0.0, new Point[]{new Point(10.7, -8.95, 4.8), new Point(11.38, -8.56, 4.85)}, orientations[6]), new Path(), new Path(), new Path(), new Path()}; //This one is all u Justin

    private static Point[] reversePointHelper = {new Point(11.2146d, -9.92284, 5.47), new Point(10.5, -9.21, 4.51), new Point(10.71, -7.763, 4.75), new Point(10.51, -6.6115, 5.2074), new Point(11.047,-7.9156,5.2), new Point( 11.405, -9.05, 4.94)};
    
    //QR = target 7
    //QR indexes are off by one since there is no reason to go straight to the qr code from the start
    private Target() {};

    public static Path getPath(int currTarget, int nextTarget){
        if(currTarget == nextTarget){
            return null;
        }

        if(currTarget > nextTarget){
            Path p = getPath(nextTarget, currTarget);//reversePath(getPath(nextTarget, currTarget));
            //p.setQuaternion(orientations[nextTarget-1]);
            Point[] pts = new Point[p.getPoints().length];


            for(int i=0; i< pts.length-1; i++){
                pts[i] = p.getPoints()[p.getPoints().length -1 - i];
            }
            pts[pts.length-1] = reversePointHelper[nextTarget-1];

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
