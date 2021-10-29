#### PoseSLAM

1. User laser scan matcher to get odom between frames and transform into base frame
2. Get minicheetah odometry and transform into base frame (this should already be a preintegrated IMU)
3. Incorporate these factors into an incremental slam factor graph
4. Optimize at each time step t
5. Compare to ground truth for each time step 2
(6). Loop Closure, (Odometry constrains between T_n and T_n + a, a = 2..N)

#### GraphSLAM

1. Convert /scan topic into a point cloud
2. Register each point from point cloud into a global point cloud and get a list of landmarks (bearing observations from each pose) (should be transformed into base frame)
3. Get minicheetah odometry and transform into base frame
4. Incorporate these factors into the incremental SLAM factor graph
5. Optimize at each time step 2
(6). Loop Closure
(7). Replace 1-2 with "feature-based" representations for better performance
