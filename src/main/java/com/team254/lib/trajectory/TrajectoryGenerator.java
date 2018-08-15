package com.team254.lib.trajectory;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import com.team1323.frc2018.Constants;
import com.team1323.frc2018.DriveMotionPlanner;
import com.team1323.frc2018.pathfinder.PathManager;
import com.team1323.frc2018.pathfinder.PathfinderPath;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Pose2dWithCurvature;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;
import com.team254.lib.trajectory.timing.CurvatureVelocityConstraint;
import com.team254.lib.trajectory.timing.TimedState;
import com.team254.lib.trajectory.timing.TimingConstraint;

import edu.wpi.first.wpilibj.Timer;
import jaci.pathfinder.Waypoint;

public class TrajectoryGenerator {
    private static final double kMaxVelocity = 12.5;
    private static final double kMaxAccel = 12.5;
    private static final double kMaxCentripetalAccelElevatorDown = 110.0 / 12.0;
    private static final double kMaxCentripetalAccel = 100.0 / 12.0;
    private static final double kMaxVoltage = 9.0;
    private static final double kFirstPathMaxVoltage = 9.0;
    private static final double kFirstPathMaxAccel = 10.0;
    private static final double kFirstPathMaxVel = 10.0;

    private static final double kSimpleSwitchMaxAccel = 100.0 / 12.0;
    private static final double kSimpleSwitchMaxCentripetalAccel = 80.0 / 12.0;
    private static final double kSimpleSwitchMaxVelocity = 120.0 / 12.0;

    private static TrajectoryGenerator mInstance = new TrajectoryGenerator();
    private final DriveMotionPlanner mMotionPlanner;
    private TrajectorySet mTrajectorySet = null;

    public static TrajectoryGenerator getInstance() {
        return mInstance;
    }

    private TrajectoryGenerator() {
        mMotionPlanner = new DriveMotionPlanner();
    }

    public void generateTrajectories() {
        if(mTrajectorySet == null) {
        	double startTime = Timer.getFPGATimestamp();
            System.out.println("Generating trajectories...");
            mTrajectorySet = new TrajectorySet();
            System.out.println("Finished trajectory generation in: " + (Timer.getFPGATimestamp() - startTime) + " seconds");
        }
    }

    public TrajectorySet getTrajectorySet() {
        return mTrajectorySet;
    }

    public Trajectory<TimedState<Pose2dWithCurvature>> generateTrajectory(
            boolean reversed,
            final List<Pose2d> waypoints,
            final List<TimingConstraint<Pose2dWithCurvature>> constraints,
            double max_vel,  // inches/s
            double max_accel,  // inches/s^2
            double max_decel,
            double max_voltage,
            double default_vel) {
        return mMotionPlanner.generateTrajectory(reversed, waypoints, constraints, max_vel, max_accel, max_decel, max_voltage, default_vel);
    }

    public Trajectory<TimedState<Pose2dWithCurvature>> generateTrajectory(
            boolean reversed,
            final List<Pose2d> waypoints,
            final List<TimingConstraint<Pose2dWithCurvature>> constraints,
            double start_vel,  // inches/s
            double end_vel,  // inches/s
            double max_vel,  // inches/s
            double max_accel,  // inches/s^2
            double max_decel,
            double max_voltage,
            double default_vel) {
        return mMotionPlanner.generateTrajectory(reversed, waypoints, constraints, start_vel, end_vel, max_vel, max_accel, max_decel, max_voltage, default_vel);
    }

    // CRITICAL POSES
    // Origin is the center of the robot when the robot is placed against the middle of the alliance station wall.
    // +x is towards the center of the field.
    // +y is to the left.
    // ALL POSES DEFINED FOR THE CASE THAT ROBOT STARTS ON RIGHT! (mirrored about +x axis for LEFT)
    public static final Pose2d kSideStartPose = Pose2d.fromTranslation(new Translation2d(Constants.kRobotHalfWidth, 27.0 - (5.5 - Constants.kRobotHalfLength)));

    public class TrajectorySet {
        public class MirroredTrajectory {
            public MirroredTrajectory(Trajectory<TimedState<Pose2dWithCurvature>> right) {
                this.right = right;
                this.left = TrajectoryUtil.mirrorTimed(right);
            }

            public Trajectory<TimedState<Pose2dWithCurvature>> get(boolean left) {
                return left ? this.left : this.right;
            }

            public final Trajectory<TimedState<Pose2dWithCurvature>> left;
            public final Trajectory<TimedState<Pose2dWithCurvature>> right;
        }

        public final Trajectory<TimedState<Pose2dWithCurvature>> startToLeftScale;
        public final Trajectory<TimedState<Pose2dWithCurvature>> alternateLeftmostCube;
        public final Trajectory<TimedState<Pose2dWithCurvature>> derpLeftCubeToLeftScale;
        public final Trajectory<TimedState<Pose2dWithCurvature>> alternateLeftScaleToSecondCube;
        public final Trajectory<TimedState<Pose2dWithCurvature>> alternateSecondLeftCubeToScale;

        public final Trajectory<TimedState<Pose2dWithCurvature>> startToRightScale;

        private TrajectorySet() {
            startToLeftScale = getStartToLeftScale();
            alternateLeftmostCube = convertPath(PathManager.mAlternateLeftmostCube, 1.9);
            derpLeftCubeToLeftScale = convertPath(PathManager.mDerpLeftCubeToLeftScale, 3.5);
            alternateLeftScaleToSecondCube = convertPath(PathManager.mAlternateLeftScaleToSecondCube, 4.7);
            alternateSecondLeftCubeToScale = convertPath(PathManager.mAlternateSecondLeftCubeToScale, 4.3);

            //startToRightScale = convertPath(PathManager.mStartToRightScale, 4.0/*, Arrays.asList(new CurvatureVelocityConstraint())*/);
            startToRightScale = generateTrajectory(false, convertWaypoints(PathManager.mStartToRightScale), Arrays.asList(new CurvatureVelocityConstraint()), 10.0, 10.0, 6.0, kMaxVoltage, 8.0);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getStartToLeftScale() {
            return convertPath(PathManager.mStartToLeftScale, 5.5);
        }


    }
    
    public List<Pose2d> convertWaypoints(PathfinderPath path){
    	List<Pose2d> waypoints = new ArrayList<>();
    	for(Waypoint waypoint : path.getWaypoints()){
    		waypoints.add(new Pose2d(new Translation2d(waypoint.x, waypoint.y), Rotation2d.fromRadians(waypoint.angle)));
    	}
    	return waypoints;
    }

    public Trajectory<TimedState<Pose2dWithCurvature>> convertPath(PathfinderPath path, double defaultCook){
        return convertPath(path, defaultCook, Arrays.asList());
    }

    public Trajectory<TimedState<Pose2dWithCurvature>> convertPath(PathfinderPath path, double defaultCook,
    List<TimingConstraint<Pose2dWithCurvature>> constraints){
        return generateTrajectory(false, convertWaypoints(path), constraints, 
            path.maxSpeed, path.maxAccel, path.maxAccel, kMaxVoltage, defaultCook);
    }
}
