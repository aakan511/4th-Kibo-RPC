protected String getQRData(Mat img)
    {
        Mat imag = new Mat();
        Core.flip(img, imag, Core.ROTATE_180);
        Rect rectCrop = new Rect(640, 480, 640, 480);
        Mat image = new Mat(imag, rectCrop);
        QRCodeDetector decoder = new QRCodeDetector();
        api.saveMatImage(image, "3_QRCodes_Undistort_Flip.png");
        Mat points = new Mat();
        String data = decoder.detectAndDecode(image, points);
        Log.i(TAG, "QR DATA: " + data);
        if (!points.empty()){
            Log.i(TAG, "points not empty! " + data);
            return data;
        }
        return "";
    }
    protected String QRDataToReport(String data)
    {
        if (data.equals("JEM")){
            return "STAY_AT_JEM";
        }
        if (data.equals("COLOMBUS")){
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
        return "";
    }
