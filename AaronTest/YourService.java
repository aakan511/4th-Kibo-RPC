package jp.jaxa.iss.kibo.rpc.sampleapk;

import android.graphics.Bitmap;
import android.util.Log;

import org.opencv.core.CvType;
import org.opencv.core.Mat;

import java.util.Arrays;
import java.util.List;

import gov.nasa.arc.astrobee.Kinematics;
import gov.nasa.arc.astrobee.Result;
import gov.nasa.arc.astrobee.types.Point;
import gov.nasa.arc.astrobee.types.Quaternion;
import jp.jaxa.iss.kibo.rpc.api.KiboRpcService;
/**
 * Class meant to handle commands from the Ground Data System and execute them in Astrobee
 */

public class YourService extends KiboRpcService {

    final int LOOP_MAX=5;
    final String TAG = this.getClass().getSimpleName();
    final float markerLength = 0.05f;

    @Override
    protected void runPlan1(){


        // the mission starts
        api.startMission();

        double[][] distortion = api.getNavCamIntrinsics(); //gets camera distortion
        Mat camMat = new Mat().zeros(3, 3, CvType.CV_64FC(1));//intrinsic camera matrix initializer
        Mat distortionCoefficients = new Mat().zeros(4, 1, CvType.CV_64FC(1)); //distortion coefficient initializer

        for(int r=0; r<3; r++){ //fills intrinsic camera matrix with correct values
            for(int c=0; c<3; c++) {
                camMat.put(r, c, (distortion[0][3*r+c]));
                Log.i(TAG, "camMat[" + r +", " + c + "] = " + camMat.get(r, c));
                Log.i(TAG, "distortion[" + (3*r+c) + "] = " + distortion[0][3*r+c]);
            }
        }
        for(int i=0; i<distortion[1].length-1; i++){ //fills distorition coefficient array with values
            distortionCoefficients.put(i, 0, (distortion[1][i]));
            Log.i(TAG, "distortionCoefficients[" + i + "] = " + distortionCoefficients.get(0,1));
            Log.i(TAG, "distortion[" + i + "] = " + distortion[1][i]);
        }

        int currTarget=0;
        int[] targets = {2, 3, 1, 2};
        for(int nextTarget : targets){
            List<Integer> list = api.getActiveTargets();
            Log.i(TAG, "active targets(nextTarget=" + nextTarget + "): " + Arrays.toString(list.toArray()));

            Path path = Target.getPath(currTarget, nextTarget);
            for(Point p : path.getPoints()){
                moveAstrobee(p, path.getQuaternion(), 'A', true);
            }

            api.laserControl(true);

            Bitmap bmpimage = api.getBitmapNavCam();
            api.saveBitmapImage(bmpimage, "target" + nextTarget + "Laserbmp.jpg");

            Mat image = api.getMatNavCam();

            api.takeTargetSnapshot(nextTarget);

            api.saveMatImage(image, "target" + nextTarget + "_test");

            currTarget = nextTarget;
        }



//            // get remaining active time and mission time
//            List<Long> timeRemaining = api.getTimeRemaining();
//
//            // check the remaining milliseconds of mission time
//            if (timeRemaining.get(1) < 60000){
//                break;
//            }


        List<Integer> list = api.getActiveTargets();
        Log.i(TAG, "active targets: " + Arrays.toString(list.toArray()));
        // move to a point
//        Point point = new Point(10.4d, -10.2d, 4.47d);
//        Quaternion quaternion = new Quaternion(0f, 0f, -0.707f, 0.707f);//new Quaternion(0f, 0f, 0f, 1f);
//
//        Point point = new Point(10.6d, -10.0d, 5.2988d);
//        moveAstrobee(point, quaternion, 'A', true);
//
//        //point = new Point(11.1d, -9.5d, 5.2988d);
//        //moveAstrobee(point, quaternion, 'A', true);
//
//        //point = new Point(11.2746d, -9.92284, 5.2988); //Astrobee coords
//
//        point = new Point(11.2146d, -9.92284, 5.47);
//        moveAstrobee(point, quaternion, 'A', true);
//        // irradiate the laser
//        api.laserControl(true);
//
//        Bitmap bmpimage = api.getBitmapNavCam();
//        api.saveBitmapImage(bmpimage,"3_target1Laserbmp.jpg");
//
//        // get a camera image
//        Mat image = api.getMatNavCam();
//        Mat image_undistorted = new Mat();
//
//        org.opencv.imgproc.Imgproc.undistort(image, image_undistorted, camMat, distortionCoefficients); //undistorts image
//
//
//        //BEGIN EXPERIMENT WITH TVEC AND RVEC
//        ArrayList<Mat> corners = new ArrayList<Mat>();
//        List<Mat> new_corners = new ArrayList<>() ;
//        Mat ids = new Mat();
//
//        Aruco.detectMarkers(image, Aruco.getPredefinedDictionary(Aruco.DICT_5X5_250), corners, ids);
//
//
//        Mat rvec = new Mat();
//        Mat tvec = new Mat();
//        Aruco.estimatePoseSingleMarkers(corners, markerLength, camMat, distortionCoefficients, rvec, tvec);
//
//        Log.i(TAG, "rvec :"+rvec.dump());
//        Log.i(TAG, "tvec :"+tvec.dump());
//
//
//        //END EXPERIMENT
//
//
//        // take active target snapshots
//        int target_id = 1;
//        //target_id = api.getActiveTargets().iterator().next();
//        Log.i(TAG, "target_id= " + target_id);
//        api.takeTargetSnapshot(target_id);
//        api.saveMatImage(image, "target1_test");
//        api.saveMatImage(image_undistorted, "target1_undsistort");
//
//
////Point 1 - point 2 path
//        quaternion = new Quaternion(0.5f, 0.5f, -0.5f, 0.5f);
//        point = new Point(10.5, -9.0709, 4.75);
//        moveAstrobee(point, quaternion, 'A', true);
//
//        point = new Point(10.5, -9.21, 4.51); //Point(10.5, -9.19, 4.54)
//        moveAstrobee(point, quaternion, 'A', true);
//
//        api.laserControl(true);
//
//        Bitmap bmpimage2 = api.getBitmapNavCam();
//        api.saveBitmapImage(bmpimage2, "target2_bpm.jpg");
//
//        Mat image2 = api.getMatNavCam();
//
//        target_id = 2;
//        Log.i(TAG, "target_id= " + target_id);
//        api.takeTargetSnapshot(target_id);
//        api.saveMatImage(image2, "target2_laser");
////End point 1-2 path
//
//
////Point 2-3 path
//
//        quaternion = new Quaternion(0.f, 0.707f, 0f, 0.707f);
//        point = new Point(10.71, -8.5, 4.75);
//        moveAstrobee(point, quaternion, 'A', true);
//
//        point = new Point(10.71, -7.763, 4.75);
//        moveAstrobee(point, quaternion, 'A', true);
//
//        api.laserControl(true);
//
//        Bitmap bmpimage3 = api.getBitmapNavCam();
//        api.saveBitmapImage(bmpimage3, "target3_bpm.jpg");
//
//        Mat image3 = api.getMatNavCam();
//
//        target_id = 3;
//        Log.i(TAG, "target_id= " + target_id);
//        api.takeTargetSnapshot(target_id);
//        api.saveMatImage(image3, "target3_laser");
//
////END Point 2-3 path


        // turn on the front flash light
        api.flashlightControlFront(0.05f);

        // get QR code content
        String mQrContent = yourMethod();

        // turn off the front flash light
        api.flashlightControlFront(0.00f);

        // notify that astrobee is heading to the goal
        api.notifyGoingToGoal();

        /* ********************************************************** */
        /* write your own code to move Astrobee to the goal positiion */
        /* ********************************************************** */

        // send mission completion
        api.reportMissionCompletion(mQrContent);
    }

    @Override
    protected void runPlan2(){
        // write your plan 2 here
    }

    @Override
    protected void runPlan3(){
        // write your plan 3 here
    }

    // You can add your method
    private String yourMethod(){
        return "your method";
    }

//    private boolean move(Point point1, Quaternion quat1, Boolean print, int tries){
//        Result result = api.moveTo(point1, quat1, print);
//        int i=0;
//        while(!result.hasSucceeded() && i<tries){
//            result = api.moveTo(point1, quat1, print);
//            i++;
//        }
//        return result.hasSucceeded();
//    }

    protected Kinematics.Confidence moveAstrobee(Point point, Quaternion quaternion, char moveType, Boolean printRobotPosition)
    {
        //Qua_x 0.7071068 Astrobee spin right
        //QUa_x -0.7071068 Astrobee spin left
        //Qua_y  1    Astrobee look up
        //Qua_y -1    Astrobee look down
        //Qua_w  1    Astrobee turn right
        //Qua_W -1    Astrobee turn left

        final boolean DO_FOR_R = false;// Do not repeatedly try for relative move

        Result result;

        double dX, dY, dZ;
        float dOX,dOY,dOZ,dOW;

        Point currentPoint ;

        Point startPoint ;
        Point targetPoint ;

        Quaternion currentQuaternion1;
        Kinematics.Confidence currentPositionConfidence;
        Kinematics kinematics = null;

        currentPositionConfidence = Kinematics.Confidence.GOOD;
        final double [] kIZ2_min_data = {9.5, -10.5, 4.02};
        final double [] kIZ2_max_data = {10.5, -9.6, 4.8};
        final double [] kIZ1_min_data = {10.3, -10.2, 4.32};
        final double [] kIZ1_max_data = {11.55 ,-6.0, 5.57};

        final Vector kIZ2_min = new Vector(kIZ2_min_data);
        final Vector  kIZ2_max = new Vector(kIZ2_max_data);
        final Vector  kIZ1_min = new Vector(kIZ1_min_data);
        final Vector  kIZ1_max = new Vector(kIZ1_max_data);

        Vector moveToPoint = new Vector(point);
        if(moveToPoint.distanceTo(kIZ1_max)<0.025){
            point = moveToPoint.minusScalar(0.025).toPoint();
            Log.i(TAG,"Restricted movement due to KIZ 1 max violation");
        }
        if(kIZ1_min.distanceTo(moveToPoint)<0.025){
            point = moveToPoint.minusScalar(-0.025).toPoint();
            Log.i(TAG,"Restricted movement due to KIZ 1 min violation");
        }

        startPoint = api.getRobotKinematics().getPosition();
        currentPoint = startPoint;
        if(moveType=='R') {
            targetPoint = new Point(startPoint.getX()+ point.getX(), startPoint.getY()+point.getY(),
                    startPoint.getZ()+point.getZ());
            result = api.relativeMoveTo(point, quaternion, printRobotPosition);
        }else{
            targetPoint=point;
            result = api.moveTo(point, quaternion, printRobotPosition);
            Log.i(TAG,"Moved");
        }
        int noOfTry = 0;
        if(result.hasSucceeded()){
            kinematics = api.getRobotKinematics();
            if(kinematics.getConfidence() == Kinematics.Confidence.GOOD){
                currentPoint = kinematics.getPosition();
                if(currentPoint.getX() != point.getX() ||
                        currentPoint.getY() != point.getY() ||
                        currentPoint.getZ() != point.getZ()){
                    currentPositionConfidence = Kinematics.Confidence.POOR;
                }
            }
        }
        while((!result.hasSucceeded() || currentPositionConfidence == Kinematics.Confidence.POOR) && noOfTry < LOOP_MAX ){

            if(moveType=='R') {
                if( DO_FOR_R == true) {
                    point = new Point(targetPoint.getX() - currentPoint.getX(), targetPoint.getY() - currentPoint.getY(),
                            targetPoint.getZ() - currentPoint.getZ());
                    result = api.relativeMoveTo(point, quaternion, printRobotPosition);
                }
            }else{
                // point = new Point(point.getX()-0.015f,point.getY()-0.015f,point.getZ()-0.015f);
                result = api.moveTo(point, quaternion, printRobotPosition);
                Log.i(TAG,"Moving Loop " + noOfTry);
            }
            if(result.hasSucceeded()){
                kinematics = api.getRobotKinematics();
                if(kinematics.getConfidence() == Kinematics.Confidence.GOOD){
                    currentPoint = kinematics.getPosition();
                    if(currentPoint.getX() != point.getX() ||
                            currentPoint.getY() != point.getY() ||
                            currentPoint.getZ() != point.getZ()){
                        currentPositionConfidence = Kinematics.Confidence.POOR;
                    }
                }
            }
            ++noOfTry;
        }

        return (Kinematics.Confidence.GOOD);
    }
    /************************************************************/
    /* computeQuaternion()
    /* Uses global values of yaw, pitch, roll to compute the
    /* quaternion.
    /*      rotation x - roll
    /*              y - pitch
     /*             z - yaw
    /************************************************************/
    protected Quaternion computeQuaternion(double roll,double pitch, double yaw)
    {
        //Precompute sine and cosine values.
        //Input Euler Angles are in degrees
        double su = (double) Math.sin(roll  *Math.PI/360);
        double sv = (double)Math.sin(pitch*Math.PI/360);
        double sw = (double)Math.sin(yaw *Math.PI/360);
        double cu = (double)Math.cos(roll  *Math.PI/360);
        double cv = (double)Math.cos(pitch*Math.PI/360);
        double cw = (double)Math.cos(yaw *Math.PI/360);

        //Quaternion
        float q0 = 1.0f;
        float q1 = 0.0f;
        float q2 = 0.0f;
        float q3 = 0.0f;

        q0 = (float)(cu*cv*cw + su*sv*sw);
        q1 = (float)(su*cv*cw - cu*sv*sw);
        q2 = (float)(cu*sv*cw + su*cv*sw);
        q3 = (float)(cu*cv*sw - su*sv*cw);

        //Display Quaternion, rounded to three sig. figs.
        q0 = (float)Math.floor(q0*10000)/10000;
        q1 = (float)Math.floor(q1*10000)/10000;
        q2 = (float)Math.floor(q2*10000)/10000;
        q3 = (float)Math.floor(q3*10000)/10000;


        return(new Quaternion (q1,q2,q3,q0));
    }


    protected static void wait(int sec)
    {
        int ms = sec* 1000;

        try
        {
            Thread.sleep(ms);
        }
        catch(InterruptedException ex)
        {
            Thread.currentThread().interrupt();
        }
    }
}
