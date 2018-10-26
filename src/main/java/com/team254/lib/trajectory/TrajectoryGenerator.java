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
            double default_vel,
            int slowdown_chunks) {
        return mMotionPlanner.generateTrajectory(reversed, waypoints, constraints, max_vel, max_accel, max_decel, max_voltage, 
            default_vel, slowdown_chunks);
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
            double default_vel,
            int slowdown_chunks) {
        return mMotionPlanner.generateTrajectory(reversed, waypoints, constraints, start_vel, end_vel, max_vel, max_accel, max_decel, max_voltage, 
            default_vel, slowdown_chunks);
    }

    // CRITICAL POSES
    // Origin is the center of the robot when the robot is placed against the middle of the alliance station wall.
    // +x is towards the center of the field.
    // +y is to the left.
    // ALL POSES DEFINED FOR THE CASE THAT ROBOT STARTS ON RIGHT! (mirrored about +x axis for LEFT)
    public static final Pose2d kSideStartPose = Pose2d.fromTranslation(new Translation2d(Constants.kRobotHalfWidth, 5.5 - Constants.kRobotHalfLength));

    public static final Pose2d kLeftScaleScorePose = new Pose2d(new Translation2d(22.75, Constants.kLeftSwitchCloseCorner.y() - Constants.kRobotHalfLength - 1.0),
        Rotation2d.fromDegrees(0.0));

    public static final Pose2d kSecondRightCubePose = new Pose2d(new Translation2d(Constants.kRightSwitchFarCorner.x() + 3.0, Constants.kRightSwitchFarCorner.y() + Constants.kRobotHalfLength - 3.25),
        Rotation2d.fromDegrees(90.0));

    //4 Cube Switch Auto?
    public static final Translation2d kLeftSwitchScoringPose = new Translation2d(Constants.kLeftSwitchCloseCorner.x() - Constants.kRobotHalfLength - 4.0, Constants.kLeftSwitchCloseCorner.y() + Constants.kRobotHalfWidth + 1.0);
    public static final Translation2d kOuterCubeIntakingPose = new Translation2d(Constants.kLeftSwitchCloseCorner.x() - (3*Constants.kCubeWidth) - 2.0, Constants.kLeftSwitchCloseCorner.y() + Constants.kRobotHalfWidth + 5.1);
    public static final Translation2d kMiddleCubeIntakingPose = new Translation2d(Constants.kLeftSwitchCloseCorner.x() - (2*Constants.kCubeWidth) - 1.5, Constants.kLeftSwitchCloseCorner.y() + Constants.kRobotHalfWidth + 5.2);
    public static final Translation2d kLeftSwitchScoringPose2 = new Translation2d(Constants.kLeftSwitchCloseCorner.x() - (2*Constants.kCubeWidth) - 1.5, Constants.kLeftSwitchCloseCorner.y() + Constants.kRobotHalfWidth + 1.0);
    public static final Translation2d kBottomLeftIntakingPose = new Translation2d(Constants.kLeftSwitchCloseCorner.x() - (2*Constants.kCubeWidth) - 1.25, Constants.kLeftSwitchCloseCorner.y() + Constants.kRobotHalfWidth + 4.25);

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

        //Left Scale Auto
        public final Trajectory<TimedState<Pose2dWithCurvature>> startToLeftScale;
        public final Trajectory<TimedState<Pose2dWithCurvature>> alternateLeftmostCube;
        public final Trajectory<TimedState<Pose2dWithCurvature>> derpLeftCubeToLeftScale;
        public final Trajectory<TimedState<Pose2dWithCurvature>> alternateLeftScaleToSecondCube;
        public final Trajectory<TimedState<Pose2dWithCurvature>> alternateSecondLeftCubeToScale;

        //Right Scale Auto
        public final Trajectory<TimedState<Pose2dWithCurvature>> startToRightScale;
        public final Trajectory<TimedState<Pose2dWithCurvature>> rightScaleToFirstCube;
        public final Trajectory<TimedState<Pose2dWithCurvature>> alternateRightCubeToRightScale;
        public final Trajectory<TimedState<Pose2dWithCurvature>> alternateRightScaleToSecondCube;
        public final Trajectory<TimedState<Pose2dWithCurvature>> secondCubeToRightScale;

        //Left Switch Auto
        public final Trajectory<TimedState<Pose2dWithCurvature>> frontLeftSwitch;
        public final Trajectory<TimedState<Pose2dWithCurvature>> frontLeftSwitchToOuterCube;
        public final Trajectory<TimedState<Pose2dWithCurvature>> outerCubeToFrontLeftSwitch;
        public final Trajectory<TimedState<Pose2dWithCurvature>> frontLeftSwitchToMiddleCube;
        public final Trajectory<TimedState<Pose2dWithCurvature>> middleCubeToFrontLeftSwitch;
        public final Trajectory<TimedState<Pose2dWithCurvature>> frontLeftSwitchToDropoff;
        public final Trajectory<TimedState<Pose2dWithCurvature>> frontLeftSwitchToBottomMiddle;

        //Right Switch Auto
        public final Trajectory<TimedState<Pose2dWithCurvature>> frontRightSwitch;
        public final Trajectory<TimedState<Pose2dWithCurvature>> frontRightSwitchToOuterCube;
        public final Trajectory<TimedState<Pose2dWithCurvature>> outerCubeToFrontRightSwitch;
        public final Trajectory<TimedState<Pose2dWithCurvature>> frontRightSwitchToMiddleCube;
        public final Trajectory<TimedState<Pose2dWithCurvature>> middleCubeToFrontRightSwitch;
        public final Trajectory<TimedState<Pose2dWithCurvature>> frontRightSwitchToDropoff;
        public final Trajectory<TimedState<Pose2dWithCurvature>> frontRightSwitchToBottomMiddle;

        //4 Cube Left Switch
        public final Trajectory<TimedState<Pose2dWithCurvature>> fourCubeFrontLeftSwitch;
        public final Trajectory<TimedState<Pose2dWithCurvature>> fourCubeFrontLeftToOuterCube;
        public final Trajectory<TimedState<Pose2dWithCurvature>> fourCubeOuterToLeftSwitch;
        public final Trajectory<TimedState<Pose2dWithCurvature>> fourCubeFrontLeftToMiddleCube;
        public final Trajectory<TimedState<Pose2dWithCurvature>> fourCubeMiddleToLeftSwitch;
        public final Trajectory<TimedState<Pose2dWithCurvature>> fourCubeFrontLeftToBottomLeft;
        public final Trajectory<TimedState<Pose2dWithCurvature>> fourCubebottomLeftToSwitch;

        //Poof assist switch + scale
        public final Trajectory<TimedState<Pose2dWithCurvature>> middleCubeToLeftScale;
        public final Trajectory<TimedState<Pose2dWithCurvature>> middleCubeToRightScale;
        public final Trajectory<TimedState<Pose2dWithCurvature>> alternateMiddleCubeToLeftScale;
        public final Trajectory<TimedState<Pose2dWithCurvature>> alternateMiddleCubeToRightScale;

        //Poof assist scale auto
        public final Trajectory<TimedState<Pose2dWithCurvature>> backOffLeftScale;

        private TrajectorySet() {
            //Left Scale Mode
            startToLeftScale = getStartToLeftScale();
            alternateLeftmostCube = /*getAlternateLeftmostCube();*/convertPath(PathManager.mAlternateLeftmostCube, 3.8);
            derpLeftCubeToLeftScale = convertPath(PathManager.mDerpLeftCubeToLeftScale, 3.5);
            alternateLeftScaleToSecondCube = convertPath(PathManager.mAlternateLeftScaleToSecondCube, 4.7);
            alternateSecondLeftCubeToScale = convertPath(PathManager.mAlternateSecondLeftCubeToScale, 4.3);

            //Right Scale Mode
            startToRightScale = generateTrajectory(false, convertWaypoints(PathManager.mStartToRightScale), Arrays.asList(new CurvatureVelocityConstraint()), 10.0, 10.0, 2.0, kMaxVoltage, 9.75, 3);
            rightScaleToFirstCube = convertPath(PathManager.mRightScaleToFirstCube, 3.0);
            alternateRightCubeToRightScale = convertPath(PathManager.mAlternateRightCubeToRightScale, 4.75);
            alternateRightScaleToSecondCube = convertPath(PathManager.mAlternateRightScaleToSecondCube, 5.5);
            secondCubeToRightScale = getSecondCubeToRightScale();

            //Left Switch Mode
            frontLeftSwitch = convertPath(PathManager.mFrontLeftSwitch, 6.0);
            frontLeftSwitchToOuterCube = convertPath(PathManager.mFrontLeftSwitchToOuterCube, 5.75);
            outerCubeToFrontLeftSwitch = convertPath(PathManager.mOuterCubeToFrontLeftSwitch, 5.5);
            frontLeftSwitchToMiddleCube = convertPath(PathManager.mFrontLeftSwitchToMiddleCube, 6.25);
            middleCubeToFrontLeftSwitch = convertPath(PathManager.mMiddleCubeToFrontLeftSwitch, 6.0);
            frontLeftSwitchToDropoff = convertPath(PathManager.mFrontLeftSwitchToDropoff, 2.0);
            frontLeftSwitchToBottomMiddle = convertPath(PathManager.mFrontLeftSwitchToBottomMiddle, 5.75);

            //Right Switch Mode
            frontRightSwitch = convertPath(PathManager.mFrontRightSwitch, 6.75);
            frontRightSwitchToOuterCube = convertPath(PathManager.mFrontRightSwitchToOuterCube, 5.75);
            outerCubeToFrontRightSwitch = convertPath(PathManager.mOuterCubeToFrontRightSwitch, 5.5);
            frontRightSwitchToMiddleCube = convertPath(PathManager.mFrontRightSwitchToMiddleCube, 6.5);
            middleCubeToFrontRightSwitch = convertPath(PathManager.mMiddleCubeToFrontRightSwitch, 5.75);
            frontRightSwitchToDropoff = convertPath(PathManager.mFrontLeftSwitchToDropoff, 2.0);
            frontRightSwitchToBottomMiddle = convertPath(PathManager.mFrontRightSwitchToBottomMiddle, 4.25);

            //4 Cube Switch Mode?
            fourCubeFrontLeftSwitch = getFourCubeFrontLeftSwitch();
            fourCubeFrontLeftToOuterCube = getFourCubeFrontLeftToOuterCube();
            fourCubeOuterToLeftSwitch = getFourCubeOuterToLeftSwitch();
            fourCubeFrontLeftToMiddleCube = getFourCubeFrontLeftToMiddleCube();
            fourCubeMiddleToLeftSwitch = getFourCubeMiddleToLeftSwitch();
            fourCubeFrontLeftToBottomLeft = getFourCubeFrontLeftToBottomLeft();
            fourCubebottomLeftToSwitch = getFourCubeBottomLeftToSwitch();

            //Switch + Scale Assist Modes
            middleCubeToLeftScale = getMiddleCubeToLeftScale();
            middleCubeToRightScale = getMiddleCubeToRightScale();
            alternateMiddleCubeToLeftScale = getAlternateMiddleCubeToLeftScale();
            alternateMiddleCubeToRightScale = getAlternateMiddleCubeToRightScale();

            //Poof Assist Mode
            backOffLeftScale = getBackOffLeftScale();
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getStartToLeftScale() {
            //return convertPath(PathManager.mStartToLeftScale, 6.25);
            return generateTrajectory(false, convertWaypoints(PathManager.mStartToLeftScale), Arrays.asList(), 8.0, 6.0, 2.0, kMaxVoltage, 7.0, 1);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getAlternateLeftmostCube(){
            List<Pose2d> waypoints = new ArrayList<>();
            Pose2d firstTranslation = new Pose2d(new Translation2d(22.75, Constants.kLeftSwitchCloseCorner.y() - Constants.kRobotHalfLength - 1.0), Rotation2d.fromDegrees(65.0)).transformBy(Pose2d.fromTranslation(Constants.kVehicleToModuleThree));
            waypoints.add(new Pose2d(firstTranslation.getTranslation(), Rotation2d.fromDegrees(180.0)));
            Pose2d secondTranslation = new Pose2d(new Translation2d(Constants.kLeftSwitchFarCorner.x() + 3.75, Constants.kLeftSwitchFarCorner.y() + 0.25), Rotation2d.fromDegrees(175.0)).transformBy(Pose2d.fromTranslation(Constants.kVehicleToModuleThree));
            waypoints.add(new Pose2d(secondTranslation.getTranslation(), Rotation2d.fromDegrees(135.0)));

            PathfinderPath path = PathManager.mAlternateLeftmostCube;
            return generateTrajectory(false, waypoints, Arrays.asList(), path.maxSpeed, path.maxAccel, path.maxAccel, kMaxVoltage, 3.8, 1);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getSecondCubeToRightScale(){
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kSecondRightCubePose);
            waypoints.add(Pose2d.fromTranslation(new Translation2d(Constants.kRightScaleCorner.x() - Constants.kRobotHalfLength + 0.05, Constants.kRightScaleCorner.y() + Constants.kRobotHalfWidth + 1.75)));
            
            return generateTrajectory(false, waypoints, Arrays.asList(), 10.0, 10.0, 6.0, kMaxVoltage, 6.0, 1);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getMiddleCubeToRightScale(){
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(new Translation2d(Constants.kRightSwitchCloseCorner.x() - (2*Constants.kCubeWidth) - 1.5, Constants.kRightSwitchCloseCorner.y() - Constants.kRobotHalfWidth - 4.6), Rotation2d.fromDegrees(90.0)));
            waypoints.add(Pose2d.fromTranslation(Constants.kRightSwitchCloseCorner.translateBy(new Translation2d(Constants.kRobotHalfWidth, Constants.kRobotHalfLength + 1.5))));
            waypoints.add(Pose2d.fromTranslation(Constants.kRightScaleCorner.translateBy(new Translation2d(1.0, 5.25))));

            return generateTrajectory(false, waypoints, Arrays.asList(), 8.0, 10.0, 2.0, kMaxVoltage, 7.5, 1);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getAlternateMiddleCubeToRightScale(){
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(new Translation2d(Constants.kLeftSwitchCloseCorner.x() - (2*Constants.kCubeWidth) - 1.5, Constants.kLeftSwitchCloseCorner.y() + Constants.kRobotHalfWidth + 5.2), Rotation2d.fromDegrees(90.0)));
            waypoints.add(Pose2d.fromTranslation(Constants.kRightSwitchCloseCorner.translateBy(new Translation2d(Constants.kRobotHalfWidth, Constants.kRobotHalfLength + 1.5))));
            waypoints.add(Pose2d.fromTranslation(Constants.kRightScaleCorner.translateBy(new Translation2d(1.0, 6.0))));

            return generateTrajectory(false, waypoints, Arrays.asList(), 8.0, 10.0, 2.0, kMaxVoltage, 7.5, 1);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getMiddleCubeToLeftScale(){
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(new Translation2d(Constants.kLeftSwitchCloseCorner.x() - (2*Constants.kCubeWidth) - 1.5, Constants.kLeftSwitchCloseCorner.y() + Constants.kRobotHalfWidth + 5.2), Rotation2d.fromDegrees(-90.0)));
            waypoints.add(Pose2d.fromTranslation(Constants.kLeftSwitchCloseCorner.translateBy(new Translation2d(Constants.kRobotHalfWidth, -Constants.kRobotHalfLength - 1.5))));
            waypoints.add(Pose2d.fromTranslation(Constants.kLeftScaleCorner.translateBy(new Translation2d(1.0, -5.25))));

            return generateTrajectory(false, waypoints, Arrays.asList(), 8.0, 10.0, 2.0, kMaxVoltage, 7.5, 1);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getAlternateMiddleCubeToLeftScale(){
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(new Translation2d(Constants.kRightSwitchCloseCorner.x() - (2*Constants.kCubeWidth) - 1.5, Constants.kRightSwitchCloseCorner.y() - Constants.kRobotHalfWidth - 4.6), Rotation2d.fromDegrees(-90.0)));
            waypoints.add(Pose2d.fromTranslation(Constants.kLeftSwitchCloseCorner.translateBy(new Translation2d(Constants.kRobotHalfWidth, -Constants.kRobotHalfLength - 1.5))));
            waypoints.add(Pose2d.fromTranslation(Constants.kLeftScaleCorner.translateBy(new Translation2d(1.0, -5.5))));

            return generateTrajectory(false, waypoints, Arrays.asList(), 8.0, 10.0, 2.0, kMaxVoltage, 7.5, 1);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getBackOffLeftScale(){
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(kLeftScaleScorePose);
            waypoints.add(kLeftScaleScorePose.transformBy(Pose2d.fromTranslation(new Translation2d(-7.0, -0.5))));

            return generateTrajectory(false, waypoints, Arrays.asList(), 10.0, 10.0, 6.0, kMaxVoltage, 6.0, 1);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getFourCubeFrontLeftSwitch(){
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(Constants.kRobotStartingPose.getTranslation(), Rotation2d.fromDegrees(-60.0)));
            waypoints.add(new Pose2d(kLeftSwitchScoringPose, Rotation2d.fromDegrees(0.0)));

            return generateTrajectory(false, waypoints, Arrays.asList(), 10.0, 10.0, 6.0, kMaxVoltage, 5.0, 1);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getFourCubeFrontLeftToOuterCube(){
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(kLeftSwitchScoringPose, Rotation2d.fromDegrees(180.0)));
            waypoints.add(new Pose2d(kOuterCubeIntakingPose, Rotation2d.fromDegrees(0.0)));

            return generateTrajectory(false, waypoints, Arrays.asList(), 10.0, 10.0, 6.0, kMaxVoltage, 5.0, 1);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getFourCubeOuterToLeftSwitch(){
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(kOuterCubeIntakingPose, Rotation2d.fromDegrees(-90.0)));
            waypoints.add(new Pose2d(kLeftSwitchScoringPose, Rotation2d.fromDegrees(-90.0)));

            return generateTrajectory(false, waypoints, Arrays.asList(), 10.0, 10.0, 6.0, kMaxVoltage, 5.0, 1);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getFourCubeFrontLeftToMiddleCube(){
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(kLeftSwitchScoringPose, Rotation2d.fromDegrees(180.0)));
            waypoints.add(new Pose2d(kMiddleCubeIntakingPose, Rotation2d.fromDegrees(0.0)));

            return generateTrajectory(false, waypoints, Arrays.asList(), 10.0, 10.0, 6.0, kMaxVoltage, 5.0, 1);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getFourCubeMiddleToLeftSwitch(){
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(kMiddleCubeIntakingPose, Rotation2d.fromDegrees(-90.0)));
            waypoints.add(new Pose2d(kLeftSwitchScoringPose2, Rotation2d.fromDegrees(-90.0)));

            return generateTrajectory(false, waypoints, Arrays.asList(), 10.0, 10.0, 6.0, kMaxVoltage, 5.0, 1);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getFourCubeFrontLeftToBottomLeft(){
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(kLeftSwitchScoringPose2, Rotation2d.fromDegrees(90.0)));
            waypoints.add(new Pose2d(kBottomLeftIntakingPose, Rotation2d.fromDegrees(90.0)));

            return generateTrajectory(false, waypoints, Arrays.asList(), 10.0, 10.0, 6.0, kMaxVoltage, 5.0, 1);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getFourCubeBottomLeftToSwitch(){
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(kBottomLeftIntakingPose, Rotation2d.fromDegrees(-90.0)));
            waypoints.add(new Pose2d(kLeftSwitchScoringPose2, Rotation2d.fromDegrees(-90.0)));

            return generateTrajectory(false, waypoints, Arrays.asList(), 10.0, 10.0, 6.0, kMaxVoltage, 3.5, 1);
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
            path.maxSpeed, path.maxAccel, path.maxAccel, kMaxVoltage, defaultCook, 1);
    }

    public Trajectory<TimedState<Pose2dWithCurvature>> recenterPath(PathfinderPath path, double defaultCook,
        Translation2d followingCenter){
        List<Pose2d> waypoints = new ArrayList<>();
    	for(Waypoint waypoint : path.getWaypoints()){
            Pose2d robotCenterPose = new Pose2d(new Translation2d(waypoint.x, waypoint.y), Rotation2d.fromRadians(waypoint.angle));
    		waypoints.add(robotCenterPose.transformBy(Pose2d.fromTranslation(followingCenter)));
        }
        return generateTrajectory(false, waypoints, Arrays.asList(), path.maxSpeed, path.maxAccel,
            path.maxAccel, kMaxVoltage, defaultCook, 1);
    }
}
