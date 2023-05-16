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


import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**
 * Class meant to handle commands from the Ground Data System and execute them in Astrobee
 */

public class YourService extends KiboRpcService {

    //final String TAG = "Salcedo";
    final int MAX_TRY = 3;
    final float markerLength = 0.05f;
    private final String TAG = this.getClass().getSimpleName();

    @Override
    protected void runPlan1(){
        // the mission starts
        api.startMission();
        int loop_counter = 0;

        /*

        Coordinates / Orientations:

            Start 9.815 -9.806 4.293 / 1 0 0 0
            Goal 11.143 -6.7607 4.9654 / 0 0 -0.707 0.707

            Point1 11.2746 -9.92284 5.2988 / 0 0 -0.707 0.707
            Point2 10.612 -9.0709 4.48 / 0.5 0.5 -0.5 0.5
            Point3 10.71 -7.7 4.48 / 0 0.707 0 0.707
            Point4 10.51 -6.7185 5.1804 / 0 0 -1 0
            Point5 11.114 -7.9756 5.3393 / -0.5 -0.5 -0.5 0.5
            Point6 11.355 -8.9929 4.7818 / 0 0 0 1
            Point7 11.369 -8.5518 4.48 / 0 0.707 0 0.707

            Target1 11.2625 -10.58 5.3625 / 0.707 0 0 0.707
            Target2 10.513384 -9.085172 3.76203 / 0 0 0 1
            Target3 10.6031 -7.71007 3.76093 / 0.707 0 0 0.707
            Target4 9.866984 -6.673972 5.09531 / 0.5 0.5 -0.5 0.5
            Target5 11.102 -8.0304 5.9076 / 1 0 0 0
            Target6 12.023 -8.989 4.8305 / 0.5 0.5 -0.5 -0.5

            QR Code 11.381944 -8.566172 3.76203 / 0 0 0 1



        Target Info:

            Target      AR ID       Target Radius
            Target 1    AR_ID 1–4       4 cm
            Target 2    AR_ID 5–8       5 cm
            Target 3    AR_ID 9–12      3 cm
            Target 4    AR_ID 13–16     5 cm
            Target 5    AR_ID 17–20     4 cm
            Target 6    AR_ID 21–24     4 cm



        Mission Complete Report:

            QR Code String Example      Report Message
            JEM                         STAY_AT_JEM
            COLUMBUS                    GO_TO_COLUMBUS
            RACK1                       CHECK_RACK_1
            ASTROBEE                    I_AM_HERE
            INTBALL                     LOOKING_FORWARD_TO_SEE_YOU
            BLANK                       NO_PROBLEM


        Step 1: getNavCamIntrinsics() -> Camera matrix and distortion coefficients
            Step 1a. get image
        Step 2. undistort -> undistorted image for aruco detect markers
        Step 3. Aruco.detectMarkers() -> corners, ids


         */

        final double navCamIntrinsics[][]  = api.getNavCamIntrinsics();

        final Mat navCamMat = new Mat(3,3,CvType.CV_32FC1,Scalar.all(0.0f));
        final Mat navCamDistCoef = new Mat(1,5,CvType.CV_32FC1,Scalar.all(0.0f));
        final double navCamToLaserDelta[] = {-0.0125f,	-0.0994f,	0.0285f};
        Mat distortionCoefficients4 = new Mat().zeros(4, 1, CvType.CV_64FC(1));

        ArrayList<Mat> corners = new ArrayList<Mat>();
        List<Mat> new_corners = new ArrayList<>() ;
        Mat ids = new Mat();

        final double dockCamIntrinsics [][] = api.getDockCamIntrinsics();

        navCamMat.put(0,0,Arrays.copyOfRange(navCamIntrinsics[0],0,8));
        navCamDistCoef.put(0,0,Arrays.copyOfRange(navCamIntrinsics[1],0,4));

        Log.i(TAG, "NavCAM MAT new :"+navCamMat.dump());
        Log.i(TAG, "navCamDistCoef MAT new :"+navCamDistCoef.dump());

        List<Integer> targetList = api.getActiveTargets();

        Point toQRCodes = new Point(10.381944d, -8.566172d, 3.76203d);
        Quaternion quaternion = new Quaternion(0f, 0f, 0f, 1f);
        api.moveTo(toQRCodes, quaternion, false);



        while (true){
            // get the list of active target id
            List<Integer> list = api.getActiveTargets();

            // get a camera image
            Mat image = api.getMatNavCam();

            // irradiate the laser
            api.laserControl(true);

            // take active target snapshots
            int target_id = 1;
            api.takeTargetSnapshot(target_id);

            /* ************************************************ */
            /* write your own code and repair the ammonia leak! */
            /* ************************************************ */

            // get remaining active time and mission time
            List<Long> timeRemaining = api.getTimeRemaining();

            // check the remaining milliseconds of mission time
            if (timeRemaining.get(1) < 60000){
                break;
            }

            loop_counter++;
            if (loop_counter == 2){
                break;
            }
        }
        // turn on the front flash light
        api.flashlightControlFront(0.05f);

        // get QR code content
        String mQrContent = getQR();

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

    public Quaternion multiply(Quaternion q, Quaternion r){ //proper way to rotate a quaternion. QUAT MULTIPLICATION NOT COMMUNICATIVE
        float w = q.getW()*r.getW() - q.getX()*r.getX() - q.getY()*r.getY() - q.getZ()*r.getZ();
        float x = q.getX()*r.getW() + q.getW()*r.getX() + q.getY()*r.getZ() - q.getZ()*r.getY();
        float y = q.getY()*r.getW() + q.getW()*r.getY() + q.getZ()*r.getX() - q.getX()*r.getZ();
        float z = q.getZ()*r.getW() + q.getW()*r.getZ() + q.getX()*r.getY() + q.getY()*r.getX();

        return new Quaternion(x, y, z, w);
    }
    //creates basic quaternions that can be used to rotate robot orientation
    public Quaternion zRotation(float angle){
        return new Quaternion(0, 0, (float)Math.sin(Math.toRadians(angle/2)), (float)Math.cos(Math.toRadians(angle/2)));
    }
    public Quaternion xRotation(float angle){
        return new Quaternion((float)Math.sin(Math.toRadians(angle/2)), 0, 0, (float)Math.cos(Math.toRadians(angle/2)));
    }
    public Quaternion yRotation(float angle){
        return new Quaternion(0, (float)Math.sin(Math.toRadians(angle/2)), 0, (float)Math.cos(Math.toRadians(angle/2)));
    }
    public float distance(float x1, float y1, float x2, float y2){
        float deltaX = x1-x2;
        float deltaY = y1-y2;
        return (float)(Math.sqrt(deltaX*deltaX + deltaY*deltaY));
    }

    protected Kinematics.Confidence moveAstrobee(Point point, Quaternion quaternion, char moveType, Boolean printRobotPosition)
    {
        //Qua_x 0.7071068 Astrobee spin right
        //QUa_x -0.7071068 Astrobee spin left
        //Qua_y  1    Astrobee look up
        //Qua_y -1    Astrobee look down
        //Qua_w  1    Astrobee turn right
        //Qua_W -1    Astrobee turn left

        Result result;

        double dX, dY, dZ;
        float dOX,dOY,dOZ,dOW;

        Point currentPoint ;
        Quaternion currentQuaternion1;
        Kinematics.Confidence currentPositionConfidence;
        Kinematics kinematics = null;

        final double [] kIZ2_min_data = {9.5, -10.5, 4.02};
        final double [] kIZ2_max_data = {10.5, -9.6, 4.8};
        final double [] kIZ1_min_data = {10.3, -10.2, 4.32};
        final double [] kIZ1_max_data = {11.55 ,-6.4, 5.57};

        final jp.jaxa.iss.kibo.rpc.Salcedo.Vector kIZ2_min = new jp.jaxa.iss.kibo.rpc.Salcedo.Vector(kIZ2_min_data);
        final jp.jaxa.iss.kibo.rpc.Salcedo.Vector kIZ2_max = new jp.jaxa.iss.kibo.rpc.Salcedo.Vector(kIZ2_max_data);
        final jp.jaxa.iss.kibo.rpc.Salcedo.Vector kIZ1_min = new jp.jaxa.iss.kibo.rpc.Salcedo.Vector(kIZ1_min_data);
        final jp.jaxa.iss.kibo.rpc.Salcedo.Vector kIZ1_max = new jp.jaxa.iss.kibo.rpc.Salcedo.Vector(kIZ1_max_data);

        jp.jaxa.iss.kibo.rpc.Salcedo.Vector moveToPoint = new jp.jaxa.iss.kibo.rpc.Salcedo.Vector(point);
        if(moveToPoint.distanceTo(kIZ1_max)<0.05){
            point = moveToPoint.minusScalar(0.05).toPoint();
            Log.i(TAG,"Restricted movement due to KIZ 1 max violation");
        }
        if(kIZ1_min.distanceTo(moveToPoint)<0.05){
            point = moveToPoint.minusScalar(-0.05).toPoint();
            Log.i(TAG,"Restricted movement due to KIZ 1 min violation");
        }


        if(moveType=='R') {
            result = api.relativeMoveTo(point, quaternion, printRobotPosition);
        }else{
            result = api.moveTo(point, quaternion, printRobotPosition);
            Log.i(TAG,"Moved");
        }
        int noOfTry = 0;
        while(!result.hasSucceeded() && noOfTry < MAX_TRY){

            if(moveType=='R') {
                result = api.relativeMoveTo(point, quaternion, printRobotPosition);
            }else{
                result = api.moveTo(point, quaternion, printRobotPosition);
                Log.i(TAG,"Moving Loop " + noOfTry);
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

    protected double[] multiplyQuaternion( double r0, double r1, double r2, double r3, double s0, double s1, double s2, double s3 )
    {
        double[] q = {0f, 0f, 0f, 0f};

        q[0] = r0*s0-r1*s1-r2*s2-r3*s3;
        q[1] = r0*s1+r1*s0-r2*s3+r3*s2;
        q[2] = r0*s2+r1*s3+r2*s0-r3*s1;
        q[3] = r0*s3-r1*s2+r2*s1+r3*s0;
        return( q );
    }

    protected Mat mulMat1(Mat A, Mat B){
        int r1,c1,r2,c2;

        r1 = A.rows(); c1 = A.cols();
        r2 = B.rows(); c2 = B.cols();

        // Check if multiplication is Possible
        if (c1 != r2) {
            Log.i(TAG, "Size doesn't match. A :"+r1+"x"+c1+" B :"+r2+"x"+c2+". Multiplication Not Possible");
            return(A);
        }
        Mat product = new Mat(r1,c2,CvType.CV_64FC1,Scalar.all(0));

        for(int i = 0; i < r1; i++) {
            for (int j = 0; j < c2; j++) {
                for (int k = 0; k < c1; k++) {
                    product.put(i,j,product.get(i,j)[0]+(A.get(i,k)[0]*B.get(k,j)[0]));
                    Log.i(TAG, "product "+product.get(i,j)[0]+",A "+A.get(i,k)[0]+",B "+B.get(k,j)[0]);
                    // product[i][j] += firstMatrix[i][k] * secondMatrix[k][j];
                }
            }
        }
        return product;
    }


    protected Mat mulMat1(Mat A, double s){
        int r1,c1, ch1;

        r1 = A.rows(); c1 = A.cols(); ch1 = A.channels();
        Log.i(TAG,"scalar multiplication "+r1+" "+c1+" "+ch1);

        Mat product = new Mat(r1,c1,CvType.CV_64FC1,Scalar.all(0));
        double temp=0d;

        for(int i = 0; i < r1; i++) {
            for (int j = 0; j < c1; j++) {
                temp = A.get(i,j)[0];
                product.put(i,j,temp*s);
            }
        }
        return product;
    }
    protected Mat subMat(Mat A, Mat B){
        int r1,c1,r2,c2;

        r1 = A.rows(); c1 = A.cols();
        r2 = A.rows(); c2 = A.cols();
        if(r1 != r2 || c1 != c2){
            Log.i(TAG, "Size doesn't match. A :"+r1+"x"+c1+" B :"+r2+"x"+c2+". Subtraction Not Possible");
            return(A);
        }

        Mat result = new Mat(r1,c1,CvType.CV_64FC1,Scalar.all(0.0f));

        for(int i = 0; i < r1; i++) {
            for (int j = 0; j < c1; j++) {
                result.put(i,j,A.get(i,j)[0]-B.get(i,j)[0]);
            }
        }
        Log.i(TAG,"Dump subtract output :"+result.dump());
        return result;
    }
    protected Mat transposeMat(Mat A){
        int r1,c1;

        r1 = A.rows(); c1 = A.cols();

        Log.i(TAG,"Rows :"+A.rows()+"Cols "+A.cols()+" Channels "+A.channels());

        Mat result = new Mat(c1,r1,CvType.CV_64FC1,Scalar.all(0.0f));

        for(int i = 0; i < r1; i++) {
            for (int j = 0; j < c1; j++) {
                result.put(j,i,A.get(i,j)[0]);
            }
        }
        Log.i(TAG,"Transpose Function "+result.dump());
        return result;
    }
    protected double[][] mulMat(double A[][], double B[][]){
        int r1,c1,r2,c2;

        r1 = A.length; c1 = A[0].length;
        r2 = B.length; c2 = B[0].length;

        // Check if multiplication is Possible
        if (c1 != r2) {
            Log.i(TAG, "Size doesn't match. A :"+r1+"x"+c1+" B :"+r2+"x"+c2+". Multiplication Not Possible");
            return(A);
        }
        double[][] product = new double[r1][c2];

        for(int i = 0; i < r1; i++) {
            for (int j = 0; j < c2; j++) {
                for (int k = 0; k < c1; k++) {
                    product[i][j] =product[i][j] +(A[i][k]*B[k][j]);
                }
            }
        }
        return product;

    }

    protected double[][] mulMat(double A[][], double s){
        int r1,c1;

        r1 = A.length; c1 = A[0].length;

        double[][] product = new double[r1][c1];

        for(int i = 0; i < r1; i++) {
            for (int j = 0; j < c1; j++) {
                product[i][j]=product[i][j]*s;
            }
        }
        return product;
    }
    protected double[] mulMat(double A[], double s){
        int r1;

        r1 = A.length;

        double[] product = new double[r1];

        for(int i = 0; i < r1; i++) {
            product[i] = product[i]*s;
        }
        return product;
    }
    protected Mat convertToMat(double A[],int row,int col){

        int len = A.length;
        int k = 0;

        Mat result = new Mat(row,col,CvType.CV_64FC1,Scalar.all(0.0f));

        for(int i = 0; i < row; i++) {
            for (int j = 0; j < col; j++) {
                result.put(i,j,A[k]);
                k++;
            }
        }
        Log.i(TAG,"Converted to Mat"+result.dump());
        return result;
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
    protected String getQR()
    {
        return "";
    }

}
