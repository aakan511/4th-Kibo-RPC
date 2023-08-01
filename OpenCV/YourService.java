package jp.jaxa.iss.kibo.rpc.usa;

import android.util.Log;

import org.opencv.aruco.Aruco;
import org.opencv.calib3d.Calib3d;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.opencv.objdetect.QRCodeDetector;

import java.util.ArrayList;
import java.util.Iterator;
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
    final int[][] markerIDs = {{1, 4}, {5, 8}, {9, 12}, {13, 16}};
    final float[] radius = {0.05f, 0.06f, 0.04f, 0.06f};

    @Override
    protected void runPlan1(){


        // the mission starts
        api.startMission();
        double[][] navCamIntrinsics = api.getNavCamIntrinsics(); //gets camera distortion
        Mat camMat = new Mat().zeros(3, 3, CvType.CV_64FC(1));//intrinsic camera matrix initializer
        Mat distortionCoefficients = new Mat().zeros(4, 1, CvType.CV_64FC(1)); //distortion coefficient initializer
        for(int r=0; r<3; r++){ //fills intrinsic camera matrix with correct values
            for(int c=0; c<3; c++) {
                camMat.put(r, c, (navCamIntrinsics[0][3*r+c]));
                Log.i(TAG, "camMat[" + r +", " + c + "] = " + camMat.get(r, c));
                Log.i(TAG, "navCamIntrinsics[" + (3*r+c) + "] = " + navCamIntrinsics[0][3*r+c]);
            }
        }
        for(int i=0; i<navCamIntrinsics[1].length-1; i++){ //fills distorition coefficient array with values
            distortionCoefficients.put(i, 0, (navCamIntrinsics[1][i]));
            Log.i(TAG, "distortionCoefficients[" + i + "] = " + distortionCoefficients.get(0,1));
            Log.i(TAG, "navCamIntrinsics[" + i + "] = " + navCamIntrinsics[1][i]);
        }

        boolean qrRead = false;
        String mQrContent = "";

        int currTarget=0;
        List<Integer> targets = api.getActiveTargets();
        //Log.i(TAG, "current target: " + currTarget);
        //Log.i(TAG, "active targets(before planPath): " + currTarget + ", " + Arrays.toString(targets.toArray()));
        targets = Target.planPath(targets, currTarget);
        //Log.i(TAG, "active targets(after planPath):" + Arrays.toString(targets.toArray()));
        Iterator<Integer> it = targets.iterator();
        int nextTarget = it.next();
        while((api.getTimeRemaining().get(1) - (Target.nextTargetTime(currTarget, nextTarget) + Target.qrTime(nextTarget, qrRead))) > 2000) {


                if(!qrRead && ((currTarget==1 && nextTarget==2) || (currTarget==2 && nextTarget == 1))){
                    Path path = Target.getPath(currTarget, 5);
                    for(Point p: path.getPoints()){
                        moveAstrobee(p, path.getQuaternion(), 'A', false);
                    }

                    api.flashlightControlFront(0.08f);
                    Mat image = api.getMatNavCam();
                    //Log.i(TAG, "Distortion Total :" + distortionCoefficients.total());
                    //api.saveMatImage(image, "1_QRCodes.png");
                    Mat image2 = undistort(image, camMat, distortionCoefficients);//getNavCamImage();
                    api.saveMatImage(image2, "2_QRCodes_Undistorted.png");
                    // get QR code content
                    mQrContent = getQRData(image2);
                    api.flashlightControlFront(0.0f);

                    if(mQrContent.equals("")){
                        Point pt = new Point(path.getPoints()[path.getPoints().length-1].getX(), path.getPoints()[path.getPoints().length-1].getY(), path.getPoints()[path.getPoints().length-1].getZ()-0.15f);
                        moveAstrobee(pt, path.getQuaternion(), 'A', false);
                        api.flashlightControlFront(0.08f);
                        image = api.getMatNavCam();
                        //Log.i(TAG, "Distortion Total :" + distortionCoefficients.total());
                        //api.saveMatImage(image, "1_QRCodes.png");
                        image2 = undistort(image, camMat, distortionCoefficients);//getNavCamImage();
                        api.saveMatImage(image2, "closerQR.png");
                        // get QR code content
                        mQrContent = getQRData(image2);
                        Log.i(TAG, "Closer QR read: " + mQrContent);
                        api.flashlightControlFront(0.0f);
                    }


                    qrRead=true;
                    currTarget = 5;
                }
                Path path = Target.getPath(currTarget, nextTarget);
                for (Point p : path.getPoints()) {
                    moveAstrobee(p, path.getQuaternion(), 'A', false);
                }

                //BEGIN OpenCV targeting
                //note points can be abreviated if calculating movement distance based on relative movement and 0 rather than two absolute points

                if(!Target.isTargetCalibrated(nextTarget)) {
                    ArrayList<Mat> corners;// = new ArrayList<Mat>();
                    Mat ids;// = new Mat();
                    Mat rvec;// = new Mat();
                    Mat tvec;// = new Mat();
                    Point currPt = path.getPoints()[path.getPoints().length-1];
                    Point pt1; //= new Point(coords[0] + currPt.getX(), coords[1] + currPt.getY(), coords[2] + currPt.getZ());
                    boolean keepCalibrating = true;

                    for (int counter = 0; counter < 2 && keepCalibrating; counter++) {
                        corners = new ArrayList<Mat>();
                        ids = new Mat();
                        rvec = new Mat();
                        tvec = new Mat();


                        api.flashlightControlFront(0.08f);
                        Mat img = api.getMatNavCam();
                        api.flashlightControlFront(0.0f);
                        Aruco.detectMarkers(img, Aruco.getPredefinedDictionary(Aruco.DICT_5X5_250), corners, ids);
                        Aruco.estimatePoseSingleMarkers(corners, markerLength, camMat, distortionCoefficients, rvec, tvec);
//                        Log.i(TAG, "rvec :" + rvec.dump());
                        Log.i(TAG, "tvec :" + tvec.dump());
                        Log.i(TAG, "ids :" + ids.dump());

                        //Mat axes = img.clone();
                        if (ids.total() > 0) {
                            boolean inRange = false;
                            int id = 0;
                            for (int j = 0; !inRange && j < ids.total(); j++) {
//                                Log.i(TAG, "logging in ID while loop");
//                                Log.i(TAG, "nextTarget-1 = " + (nextTarget - 1));
//                                Log.i(TAG, "nextTarget=" + nextTarget);
//                                Log.i(TAG, "get ID through int cast: " + (int) (ids.get(id, 0)[0]));
//                                Log.i(TAG, "" + ((int) ids.get(j, 0)[0] >= markerIDs[nextTarget - 1][0]));
//                                Log.i(TAG, "" + ((int) ids.get(j, 0)[0] <= markerIDs[nextTarget - 1][1]));
//                                Log.i(TAG, "j=" + j);

                                if ((int) ids.get(j, 0)[0] >= markerIDs[nextTarget - 1][0] && (int) ids.get(j, 0)[0] <= markerIDs[nextTarget - 1][1]) {
                                    inRange = true;
                                    id = j;
                                }
                            }
                            Log.i(TAG, "id from new process = " + id);
                            Mat rotMatrix = new Mat(3, 3, CvType.CV_64FC1, Scalar.all(0.0f));

                            Calib3d.Rodrigues(rvec.row(id), rotMatrix);

                            Log.i(TAG, "rotation matrix " + rotMatrix.dump());
                            float[] coords = cornerAdjust1((int) (ids.get(id, 0))[0], tvec.row(id).get(0, 0), rotMatrix);
                            Log.i(TAG, "counter = " + counter);

                            pt1 = new Point(coords[0] + currPt.getX(), coords[1] + currPt.getY(), coords[2] + currPt.getZ());

                            if (Target.distance(pt1, currPt) >= 0.05f) {//radius[nextTarget-1]
                                Log.i(TAG, "adjustingToTarget" + counter);
                                moveAstrobee(pt1, path.getQuaternion(), 'A', false);
                            } else {
                                Log.i(TAG, "target calibrated in range");
                                keepCalibrating = false;
                            }
                            currPt = new Point(pt1.getX(), pt1.getY(), pt1.getZ());
                        }
                    }
                    Target.calibrateTarget(nextTarget, currPt);//currPt
                }else{
                    Log.i(TAG, "skipped adjustToTarget");
                }

                //END EXPERIMENT

                api.laserControl(true);
                Mat image = api.getMatNavCam();
                api.takeTargetSnapshot(nextTarget);
                api.saveMatImage(image, "target" + nextTarget);
                currTarget = nextTarget;


                if(it.hasNext()){
                    nextTarget = it.next();
                }else {
                    targets = api.getActiveTargets();
                    //Log.i(TAG, "active targets(before planPath):" + currTarget + ", " + Arrays.toString(targets.toArray()));
                    targets = Target.planPath(targets, currTarget, api.getTimeRemaining().get(1), qrRead);
                    //Log.i(TAG, "active targets(after planPath):" + Arrays.toString(targets.toArray()));
                    it = targets.iterator();
                    nextTarget = it.next();
                }
        }



//        int currTarget=0;
//        int[] targets = {1, 2, 3, 4};
//        for(int nextTarget : targets){
//            List<Integer> list = api.getActiveTargets();
//            Log.i(TAG, "active targets(nextTarget=" + nextTarget + "): " + Arrays.toString(list.toArray()));
//
//            Path path = Target.getPath(currTarget, nextTarget);
//            for(Point p : path.getPoints()){
//                moveAstrobee(p, path.getQuaternion(), 'A', false);
//            }
//
//
//
//            //BEGIN OpenCV targeting
//
//            //note points can be abreviated if calculating movement distance based on relative movement and 0 rather than two absolute points
//            if(!Target.isTargetCalibrated(nextTarget)) {
//                ArrayList<Mat> corners;// = new ArrayList<Mat>();
//                Mat ids;// = new Mat();
//                Mat rvec;// = new Mat();
//                Mat tvec;// = new Mat();
//                Point currPt = path.getPoints()[path.getPoints().length-1];
//                Point pt1; //= new Point(coords[0] + currPt.getX(), coords[1] + currPt.getY(), coords[2] + currPt.getZ());
//                boolean keepCalibrating = true;
//
//                for (int counter = 0; counter < 1 && keepCalibrating; counter++) {
//                    corners = new ArrayList<Mat>();
//                    ids = new Mat();
//                    rvec = new Mat();
//                    tvec = new Mat();
//
//
//                    api.flashlightControlFront(0.08f);
//                    Mat img = api.getMatNavCam();
//                    api.flashlightControlFront(0.0f);
//                    Aruco.detectMarkers(img, Aruco.getPredefinedDictionary(Aruco.DICT_5X5_250), corners, ids);
//                    Aruco.estimatePoseSingleMarkers(corners, markerLength, camMat, distortionCoefficients, rvec, tvec);
//                    //solvePnP()
//                    Log.i(TAG, "rvec :" + rvec.dump());
//                    Log.i(TAG, "tvec :" + tvec.dump());
//                    Log.i(TAG, "ids :" + ids.dump());
//
//                    //Mat axes = img.clone();
//                    if (ids.total() > 0) {
//                        boolean inRange = false;
//                        int id = 0;
//                        for (int j = 0; !inRange && j < ids.total(); j++) {
////                            Log.i(TAG, "logging in ID while loop");
////                            Log.i(TAG, "nextTarget-1 = " + (nextTarget - 1));
////                            Log.i(TAG, "nextTarget=" + nextTarget);
////                            Log.i(TAG, "get ID through int cast: " + (int) (ids.get(id, 0)[0]));
////                            Log.i(TAG, "" + ((int) ids.get(j, 0)[0] >= markerIDs[nextTarget - 1][0]));
////                            Log.i(TAG, "" + ((int) ids.get(j, 0)[0] <= markerIDs[nextTarget - 1][1]));
////                            Log.i(TAG, "j=" + j);
//
//                            if ((int) ids.get(j, 0)[0] >= markerIDs[nextTarget - 1][0] && (int) ids.get(j, 0)[0] <= markerIDs[nextTarget - 1][1]) {
//                                //inRange = true;
//                                id = j;
//                                Mat rotMatrix = new Mat(3, 3, CvType.CV_64FC1, Scalar.all(0.0f));
//
//                                Calib3d.Rodrigues(rvec.row(j), rotMatrix);
//
//                                //Log.i(TAG, "rotation matrix " + rotMatrix.dump());
//                                float[] coords = cornerAdjust1((int) (ids.get(j, 0))[0], tvec.row(j).get(0, 0), rotMatrix);
//                                Log.i(TAG, "Begin dump for arcuo marker:" + (int) (ids.get(j, 0))[0]);
//                                Log.i(TAG, "relative coordinates to ar tag are: " + -1*coords[0] + ", " + -1*coords[1] + ", " + coords[2]);
//                                pt1 = new Point(coords[0] + currPt.getX(), coords[1] + currPt.getY(), coords[2] + currPt.getZ());
//                                moveAstrobee(pt1, path.getQuaternion(), 'A', false);
//                                api.laserControl(true);
//                                api.saveMatImage(api.getMatNavCam(), ""+ nextTarget+"," + (int) (ids.get(j, 0))[0] + ".png");
//                                api.laserControl(false);
//
//                            }
//                        }
//                        Log.i(TAG, "" + ids.total());
//                        Log.i(TAG, "id from new process = " + id);
//                        Mat rotMatrix = new Mat(3, 3, CvType.CV_64FC1, Scalar.all(0.0f));
//
//                        Calib3d.Rodrigues(rvec.row(id), rotMatrix);
//
//                        Log.i(TAG, "rotation matrix " + rotMatrix.dump());
//                        float[] coords = cornerAdjust((int) (ids.get(id, 0))[0], tvec.row(id).get(0, 0), rotMatrix);
//                        Log.i(TAG, "counter = " + counter);
//
//                        pt1 = new Point(coords[0] + currPt.getX(), coords[1] + currPt.getY(), coords[2] + currPt.getZ());
//                        if (Target.distance(pt1, currPt) > .015f) {
//                            //moveAstrobee(pt1, path.getQuaternion(), 'A', false);
//                            Log.i(TAG, "adjustingToTarget" + counter);
//                        } else {
//                            Log.i(TAG, "target calibrated in range");
//                            keepCalibrating = false;
//                        }
//                        currPt = api.getRobotKinematics().getPosition();//new Point(pt1.getX(), pt1.getY(), pt1.getZ());
//                    }
//                }
//                Target.calibrateTarget(nextTarget, currPt);
//            }
//
//            //END EXPERIMENT
//
//            api.laserControl(true);
//            Mat image = api.getMatNavCam();
//            api.saveMatImage(image, "target" + nextTarget + "_test");
//            api.takeTargetSnapshot(nextTarget);
//
//
//
//            currTarget = nextTarget;
//        }


        if(!qrRead) {
            Path path = Target.getPath(currTarget, 5);
            for (Point p : path.getPoints()) {
                moveAstrobee(p, path.getQuaternion(), 'A', false);
            }
            api.flashlightControlFront(0.08f);
            Mat image = api.getMatNavCam();
            //Log.i(TAG, "Distortion Total :" + distortionCoefficients.total());
            //api.saveMatImage(image, "1_QRCodes.png");
            Mat image2 = undistort(image, camMat, distortionCoefficients);//getNavCamImage();
            api.saveMatImage(image2, "2_QRCodes_Undistorted.png");
            // get QR code content
            mQrContent = getQRData(image2);
            api.flashlightControlFront(0.0f);


            if(mQrContent.equals("")){
                Point pt = new Point(path.getPoints()[path.getPoints().length-1].getX(), path.getPoints()[path.getPoints().length-1].getY(), path.getPoints()[path.getPoints().length-1].getZ()-0.15f);
                moveAstrobee(pt, path.getQuaternion(), 'A', false);
                api.flashlightControlFront(0.08f);
                image = api.getMatNavCam();
                //Log.i(TAG, "Distortion Total :" + distortionCoefficients.total());
                //api.saveMatImage(image, "1_QRCodes.png");
                image2 = undistort(image, camMat, distortionCoefficients);//getNavCamImage();
                api.saveMatImage(image2, "closerQR.png");
                // get QR code content
                mQrContent = getQRData(image2);
                Log.i(TAG, "Closer QR read: " + mQrContent);
                api.flashlightControlFront(0.0f);
            }
            Log.i(TAG, "Time remaining at end of QR: " + api.getTimeRemaining().get(1));
            currTarget = 5;
        }

        api.notifyGoingToGoal();
        Path goGoal = Target.getPath(currTarget, 6);
        for(Point p : goGoal.getPoints()){
            moveAstrobee(p, goGoal.getQuaternion(), 'A', false);
        }
        Log.i(TAG, "reporting targetCalibration: " + Target.targetCalibration[0]+ Target.targetCalibration[1]+ Target.targetCalibration[2]+ Target.targetCalibration[3]);
        api.reportMissionCompletion(QRDataToReport(mQrContent));
    }

    @Override
    protected void runPlan2(){
        // write your plan 2 here
    }

    @Override
    protected void runPlan3(){
        // write your plan 3 here
    }

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

    protected String getQRData(Mat img)
    {
        Rect rectCrop = new Rect(img.width()/4, img.height()/4, img.width()/2, img.height()/2);
        Mat image = new Mat(img, rectCrop);
        Core.flip(image, image, -1);
        QRCodeDetector decoder = new QRCodeDetector();
        api.saveMatImage(image, "3_QRCodes_Undistort_Flip.png");
        Mat bw = image;//new Mat();

//        com.google.zxing.BinaryBitmap bmp = new BinaryBitmap();
//        Utils.matToBitmap(image, bmp);
//        api.saveBitmapImage(bmp, "bitmaptest.png");
//        com.google.zxing.Result result = new QRCodeReader().decode(bmp);
//        String text = result.getText();
//        Log.i(TAG, "result1: " + text);
//        if(!result.equals("")){
//            return text;
//        }

        Mat thresh = new Mat();
        Imgproc.threshold(bw, thresh, 230, 255, Imgproc.THRESH_BINARY);
        //api.saveMatImage(thresh, "simpleThresh.png");

        List<MatOfPoint> contours = new ArrayList<>();
        Mat heirarchy = new Mat();
        Imgproc.findContours(thresh, contours, heirarchy,Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        rectCrop = Imgproc.boundingRect(contours.get(0));
        Mat img_contourCrop = new Mat(image, rectCrop);
        //api.saveMatImage(img_contourCrop, "contourCrop.png");

//        bmp = Bitmap.createBitmap(image.cols(), image.rows(), Bitmap.Config.ARGB_8888);
//        Utils.matToBitmap(image, bmp);
//        api.saveBitmapImage(bmp, "bitmaptest2.png");
//        result = new QRCodeReader().decode(bmp);
//        String text = result.getText();
//        Log.i(TAG, "result1: " + text);
//        if(!result.equals("")){
//            return text;
//        }

        String data = decoder.detectAndDecode(img_contourCrop);
        Log.i(TAG, "QR DATA: " + data);

        return data;

    }
    protected String QRDataToReport(String data)
    {
        if (data.equals("JEM")){
            return "STAY_AT_JEM";
        }
        if (data.equals("COLUMBUS")){
            return "GO_TO_COLUMBUS";
        }
        if (data.equals("RACK1")){
            return "CHECK_RACK_1";
        }
        if (data.equals("ASTROBEE")){
            return "I_AM_HERE";
        }
        if (data.equals("INTBALL")){
            return "LOOKING_FORWARD_TO_SEE_YOU";
        }
        if (data.equals("BLANK")){
            return "NO_PROBLEM";
        }
        return "LOOKING_FORWARD_TO_SEE_YOU";
    }
    public Mat undistort(Mat src, Mat camMat, Mat distortionCoefficients){
        Mat dst = new Mat();//output image
        org.opencv.imgproc.Imgproc.undistort(src, dst, camMat, distortionCoefficients); //undistorts image
        return dst;
    }

//    protected float[] cornerAdjust(int id, double[] pt, Mat r){
//
//        float[] deltaLaser = {0.09f, 0.0585f};
//
//        float[] orthoVectorX = {(float)r.get(0,0)[0], (float)r.get(0,1)[0]};
//        float[] orthoVectorY = {(float)r.get(1,0)[0], (float)r.get(1,1)[0]};
//
//        if(Math.abs(r.get(0,0)[0])<Math.abs(r.get(0, 1)[0])){
//            orthoVectorX = new float[]{(float)r.get(1,0)[0], (float)r.get(1,1)[0]};
//            orthoVectorY = new float[]{(float)r.get(0,0)[0], (float)r.get(0,1)[0]};
//        }
//
//
//
//
//        if(orthoVectorX[0] < 0){
//            orthoVectorX[0] = -orthoVectorX[0];
//            orthoVectorX[1] = -orthoVectorX[1];
//        }
//        if(orthoVectorY[1] < 0){
//            orthoVectorY[0] = -orthoVectorY[0];
//            orthoVectorY[1] = -orthoVectorY[1];
//        }
//
//        float[] rotAdjust = {0.10f * orthoVectorX[0] + 0.0375f * orthoVectorY[0], 0.10f * orthoVectorX[1] + 0.0375f * orthoVectorY[1]};
//        //float[] rotAdjust = {0.10f * orthoVectorX[0],  0.0375f * orthoVectorY[1]};
//        Log.i(TAG, "rotAdjust: " + rotAdjust[0] + " , " + rotAdjust[1]);
//        Log.i(TAG, "aruco id: " + id);
//        switch(id){
//            //target 1
//            case 1:
//                return new float[]{(float)pt[0]-Math.abs(rotAdjust[0]) - deltaLaser[0]-0.025f, 0, (float)pt[1]+Math.abs(rotAdjust[1]) + deltaLaser[1] };
//            case 2:
//                return new float[]{(float)pt[0]+Math.abs(rotAdjust[0]) - deltaLaser[0]-0.025f, 0, (float)pt[1]+Math.abs(rotAdjust[1]) + deltaLaser[1]};
//            case 3:
//                return new float[]{(float)pt[0]+Math.abs(rotAdjust[0]) - deltaLaser[0]-0.025f, 0, (float)pt[1]-Math.abs(rotAdjust[1]) + deltaLaser[1]};
//            case 4:
//                return new float[]{(float)pt[0]-Math.abs(rotAdjust[0]) - deltaLaser[0]-0.025f, 0, (float)pt[1]-Math.abs(rotAdjust[1]) + deltaLaser[1]};
//
//            //target 2
//            case 5:
//                return new float[]{(float)pt[0]-Math.abs(rotAdjust[0]) - deltaLaser[0]-0.025f, -1*((float)pt[1])-Math.abs(rotAdjust[1]) - deltaLaser[1], 0};
//            case 6:
//                return new float[]{(float)pt[0]+Math.abs(rotAdjust[0]) - deltaLaser[0]-0.025f, -1*((float)pt[1])-Math.abs(rotAdjust[1]) - deltaLaser[1], 0};
//            case 7:
//                return new float[]{(float)pt[0]+Math.abs(rotAdjust[0]) - deltaLaser[0]-0.025f, -1*((float)pt[1])+Math.abs(rotAdjust[1]) - deltaLaser[1], 0};
//            case 8:
//                return new float[]{(float)pt[0]-Math.abs(rotAdjust[0]) - deltaLaser[0]-0.025f, -1*((float)pt[1])+Math.abs(rotAdjust[1]) - deltaLaser[1], 0}; //inverted deltaLaser1
//
//            //target 3
//            case 9:
//                return new float[]{1*((float)pt[1] + deltaLaser[1]+Math.abs(rotAdjust[1]))+0.025f, 1*((float)pt[0] - Math.abs(rotAdjust[0]) - deltaLaser[0])-0.025f, 0}; //inverted laserdelta[1] on all targets
//            case 10:
//                return new float[]{1*((float)pt[1] + deltaLaser[1]+Math.abs(rotAdjust[1]))+0.025f, 1*((float)pt[0] + Math.abs(rotAdjust[0]) - deltaLaser[0])-0.04f, 0};
//            case 11:
//                return new float[]{1*((float)pt[1] + deltaLaser[1]-Math.abs(rotAdjust[1]))+0.025f, 1*((float)pt[0] + Math.abs(rotAdjust[0]) - deltaLaser[0])-0.025f, 0};//changed
//            case 12:
//                return new float[]{1*((float)pt[1] + deltaLaser[1]-Math.abs(rotAdjust[1]))+0.025f, 1*((float)pt[0] - Math.abs(rotAdjust[0]) - deltaLaser[0])-0.04f, 0};
//
//            //target 4
//            case 13:
//                return new float[]{0, -1*((float)pt[0]-Math.abs(rotAdjust[0]) - deltaLaser[0])+0.025f, 1*((float)pt[1]+Math.abs(rotAdjust[1]) + deltaLaser[1])};
//            case 14:
//                return new float[]{0, -1*((float)pt[0]+Math.abs(rotAdjust[0]) - deltaLaser[0])+0.025f, 1*((float)pt[1]+Math.abs(rotAdjust[1]) + deltaLaser[1])};
//            case 15:
//                return new float[]{0, -1*((float)pt[0]+Math.abs(rotAdjust[0]) - deltaLaser[0])+0.025f, 1*((float)pt[1]-Math.abs(rotAdjust[1]) + deltaLaser[1])};
//            case 16:
//                return new float[]{0, -1*((float)pt[0]-Math.abs(rotAdjust[0]) - deltaLaser[0])+0.025f, 1*((float)pt[1]-Math.abs(rotAdjust[1]) + deltaLaser[1])};
//
//
//             default:
//                return new float[]{0, 0, 0};
//
//        }
//    }

    protected float[] cornerAdjust1(int id, double[] pt, Mat r){
        Log.i(TAG, "aruco id: " + id);
        switch(id){
            //target 1
            case 1:
                return new float[]{(float)pt[0]-0.2157991556f, 0, (float)pt[1]+0.0962053633f}; //needs to be fixed
            case 2:
                return new float[]{(float)pt[0]-0.0121166379f, 0, (float)pt[1]+0.0931235607f};
            case 3:
                return new float[]{(float)pt[0]-0.0121166379f, 0, (float)pt[1]+0.016808413f};
            case 4:
                return new float[]{(float)pt[0]-0.21263513f, 0, (float)pt[1]+0.01762434f};

            //target 2
            case 5:
                return new float[]{(float)pt[0]-0.21863572f, -1*((float)pt[1])-0.13f, 0};
            case 6:
                return new float[]{(float)pt[0]-0.006f, -1*((float)pt[1])-0.13f, 0};
            case 7:
                return new float[]{(float)pt[0]-0.006f, -1*((float)pt[1])-0.0474837071f, 0};
            case 8:
                return new float[]{(float)pt[0]-0.21728513f, -1*((float)pt[1])-0.040998265f, 0};

            //target 3
            case 9:
                return new float[]{1*((float)pt[1])+0.097196944f, 1*((float)pt[0])-0.21073888f, 0};
            case 10:
                return new float[]{1*((float)pt[1])+0.09422761f, 1*((float)pt[0])-0.006f, 0};
            case 11:
                return new float[]{1*((float)pt[1])+0.020253867f, 1*((float)pt[0])-0.006f, 0};
            case 12:
                return new float[]{1*((float)pt[1])+0.022121891f, 1*((float)pt[0])-0.2141265f, 0};

            //target 4
            case 13:
                return new float[]{0, -1*((float)pt[0])+0.1936096526f, 1*((float)pt[1])+0.09426758f};
            case 14:
                return new float[]{0, -1*((float)pt[0])-0.05f, 1*((float)pt[1])+0.0926298684f}; //needs to be fixed
            case 15:
                return new float[]{0, -1*((float)pt[0])-0.05f, 1*((float)pt[1])+0.022150304f};
            case 16:
                return new float[]{0, -1*((float)pt[0])+0.1936096526f, 1*((float)pt[1])+0.022150304f};


            default:
                return new float[]{0, 0, 0};

        }
    }




}
