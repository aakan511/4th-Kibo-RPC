        int currTarget=0;
        int[] targets = {1, 2, 3, 4};
        for(int nextTarget : targets){
            List<Integer> list = api.getActiveTargets();
            Log.i(TAG, "active targets(nextTarget=" + nextTarget + "): " + Arrays.toString(list.toArray()));

            Path path = Target.getPath(currTarget, nextTarget);
            for(Point p : path.getPoints()){
                moveAstrobee(p, path.getQuaternion(), 'A', true);
            }



            //BEGIN EXPERIMENT WITH TVEC AND RVEC
            api.flashlightControlFront(0.08f);
            Mat img = api.getMatNavCam();
            api.flashlightControlFront(0.0f);
            ArrayList<Mat> corners = new ArrayList<Mat>();
            Mat ids = new Mat();

            Aruco.detectMarkers(img, Aruco.getPredefinedDictionary(Aruco.DICT_5X5_250), corners, ids);


            Mat rvec = new Mat();
            Mat tvec = new Mat();
            Aruco.estimatePoseSingleMarkers(corners, markerLength, camMat, distortionCoefficients, rvec, tvec);
            //solvePnP()
            Log.i(TAG, "rvec :"+rvec.dump());
            Log.i(TAG, "tvec :"+tvec.dump());
            Log.i(TAG, "ids :" +ids.dump());

            //Mat axes = img.clone();
            if(ids.total()>0){
                boolean inRange = false;
                int id=0;
                while(!inRange && id<ids.total()){
                    if((int)ids.get(id, 0)[0]>=markerIDs[nextTarget-1][0] && (int)ids.get(id, 0)[0]<=markerIDs[nextTarget-1][1]){
                        inRange = true;
                        id--;
                    }
                    id++;
                }
                Log.i(TAG, "id from new process = " + id);
                Mat rotMatrix = new Mat(3,3,CvType.CV_64FC1, Scalar.all(0.0f));
                Calib3d.Rodrigues(rvec.row(id), rotMatrix);
                Log.i(TAG, "rotation matrix " + rotMatrix.dump());
                float[] coords = cornerAdjust((int)(ids.get(id, 0))[0], tvec.row(id).get(0, 0), rotMatrix);
                Point currPt = path.getPoints()[path.getPoints().length-1];
                Point pt1 = new Point(coords[0] + currPt.getX(), coords[1] + currPt.getY(), coords[2] + currPt.getZ());
                moveAstrobee(pt1, path.getQuaternion(), 'A', true);
            }
