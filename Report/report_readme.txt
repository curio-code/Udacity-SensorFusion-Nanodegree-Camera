  Performace Evaluation 1:
    With Detector and Desciptor pair of SHITOMASI & BRISK, at two frame of 3 and 10 major deviation was observed between TTC_Camera and TTC_Lidar
    
    In the 2 scenarios above, there still have some noise or outliers even we implement euclidean clustering algorithm. These noise may come from:

		1. LiDAR and camera is not perfectly synchronized with each other
		3. Removing the ground just by setting a z threshold on point clould might give erroneous results thus we should use RANSAC alogorithm to remove the ground
        
  Performance Evaluation 2:
  	 In report folder I created the spreadsheet which contains  TTC_camera and TTC_lidar for all the pair of the detector and desciptors.
     From the previous evaluation from mid term project and this report I conclude the below three are the best pair for the application.
      1.
    