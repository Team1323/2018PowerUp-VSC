package com.team1323.frc2018;

import java.text.DecimalFormat;
import java.util.ArrayList;
import java.util.List;

import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Pose2dWithCurvature;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;
import com.team254.lib.trajectory.DistanceView;
import com.team254.lib.trajectory.Trajectory;
import com.team254.lib.trajectory.TrajectoryIterator;
import com.team254.lib.trajectory.TrajectorySamplePoint;
import com.team254.lib.trajectory.TrajectoryUtil;
import com.team254.lib.trajectory.timing.CurvatureVelocityConstraint;
import com.team254.lib.trajectory.timing.TimedState;
import com.team254.lib.trajectory.timing.TimingConstraint;
import com.team254.lib.trajectory.timing.TimingUtil;
import com.team254.lib.util.CSVWritable;
import com.team254.lib.util.Util;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DriveMotionPlanner implements CSVWritable {
    private static final double kMaxDx = 2.0;
    private static final double kMaxDy = 0.25;
    private static final double kMaxDTheta = Math.toRadians(5.0);
    
    private double defaultCook = 0.5;
    private boolean useDefaultCook = true;
    
    private double rotationStartTime = 0.0;
    private double rotationEndTime = 0.0;
    private double rotationTimeDuration = 0.0;
    private double targetHeading = 0.0;
    private double currentHeading = 0.0;
    private double headingDifferential = 0.0;

    public enum FollowerType {
        PURE_PURSUIT
    }

    FollowerType mFollowerType = FollowerType.PURE_PURSUIT;

    public void setFollowerType(FollowerType type) {
        mFollowerType = type;
    }

    TrajectoryIterator<TimedState<Pose2dWithCurvature>> mCurrentTrajectory;
    boolean mIsReversed = false;
    double mLastTime = Double.POSITIVE_INFINITY;
    public TimedState<Pose2dWithCurvature> mSetpoint = new TimedState<>(Pose2dWithCurvature.identity());
    Pose2d mError = Pose2d.identity();
    Translation2d mOutput = Translation2d.identity();
    
    double mDt = 0.0;

    public DriveMotionPlanner() {
    }

    public void setTrajectory(final TrajectoryIterator<TimedState<Pose2dWithCurvature>> trajectory) {
        mCurrentTrajectory = trajectory;
        mSetpoint = trajectory.getState();
        defaultCook = trajectory.trajectory().defaultVelocity();
        for (int i = 0; i < trajectory.trajectory().length(); ++i) {
            if (trajectory.trajectory().getState(i).velocity() > Util.kEpsilon) {
                mIsReversed = false;
                break;
            } else if (trajectory.trajectory().getState(i).velocity() < -Util.kEpsilon) {
                mIsReversed = true;
                break;
            }
        }
    }

    public void reset() {
        mError = Pose2d.identity();
        mOutput = Translation2d.identity();
        mLastTime = Double.POSITIVE_INFINITY;
        useDefaultCook = true;
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
        return generateTrajectory(reversed, waypoints, constraints, 0.0, 0.0, max_vel, max_accel, max_decel, max_voltage, default_vel);
    }

    public Trajectory<TimedState<Pose2dWithCurvature>> generateTrajectory(
            boolean reversed,
            final List<Pose2d> waypoints,
            final List<TimingConstraint<Pose2dWithCurvature>> constraints,
            double start_vel,
            double end_vel,
            double max_vel,  // inches/s
            double max_accel,  // inches/s^2
            double max_decel,
            double max_voltage,
            double default_vel) {
        List<Pose2d> waypoints_maybe_flipped = waypoints;
        final Pose2d flip = Pose2d.fromRotation(new Rotation2d(-1, 0, false));
        // TODO re-architect the spline generator to support reverse.
        if (reversed) {
            waypoints_maybe_flipped = new ArrayList<>(waypoints.size());
            for (int i = 0; i < waypoints.size(); ++i) {
                waypoints_maybe_flipped.add(waypoints.get(i).transformBy(flip));
            }
        }

        // Create a trajectory from splines.
        Trajectory<Pose2dWithCurvature> trajectory = TrajectoryUtil.trajectoryFromSplineWaypoints(
                waypoints_maybe_flipped, kMaxDx, kMaxDy, kMaxDTheta);

        if (reversed) {
            List<Pose2dWithCurvature> flipped = new ArrayList<>(trajectory.length());
            for (int i = 0; i < trajectory.length(); ++i) {
                flipped.add(new Pose2dWithCurvature(trajectory.getState(i).getPose().transformBy(flip), -trajectory
                        .getState(i).getCurvature(), trajectory.getState(i).getDCurvatureDs()));
            }
            trajectory = new Trajectory<>(flipped);
        }
        // Create the constraint that the robot must be able to traverse the trajectory without ever applying more
        // than the specified voltage.
        //final CurvatureVelocityConstraint velocity_constraints = new CurvatureVelocityConstraint();
        List<TimingConstraint<Pose2dWithCurvature>> all_constraints = new ArrayList<>();
        //all_constraints.add(velocity_constraints);
        if (constraints != null) {
            all_constraints.addAll(constraints);
        }
        // Generate the timed trajectory.
        Trajectory<TimedState<Pose2dWithCurvature>> timed_trajectory = TimingUtil.timeParameterizeTrajectory
                (reversed, new
                        DistanceView<>(trajectory), kMaxDx, all_constraints, start_vel, end_vel, max_vel, max_accel, max_decel);
        timed_trajectory.setDefaultVelocity(default_vel / Constants.kSwerveMaxSpeedFeetPerSecond);
        return timed_trajectory;
    }
    
    public void setTargetHeading(double targetHeading, double rotationTimeDuration, Pose2d robotPose){
    	double currentRobotHeading = robotPose.getRotation().getUnboundedDegrees();
    	this.targetHeading = com.team1323.lib.util.Util.placeInAppropriate0To360Scope(currentRobotHeading, targetHeading);
    	headingDifferential = targetHeading - currentRobotHeading;
    	rotationStartTime = mCurrentTrajectory.getProgress();
    	rotationEndTime = rotationStartTime + rotationTimeDuration;
    	this.rotationTimeDuration = rotationTimeDuration;
    }
    
    private double interpolateHeading(){
    	double currentTime = mCurrentTrajectory.getProgress();
    	if(currentTime < rotationEndTime)
    		currentHeading = targetHeading - (((rotationEndTime - currentTime) / rotationTimeDuration) * headingDifferential);
    	else
    		currentHeading = targetHeading;
    	System.out.println(currentHeading);
    	return currentHeading;
    }
    
    public double getHeading(){
    	return currentHeading;
    }

    @Override
    public String toCSV() {
        DecimalFormat fmt = new DecimalFormat("#0.000");
        return mOutput.toCSV();
    }

    protected Translation2d updatePurePursuit(Pose2d current_state) {
        double lookahead_time = Constants.kPathLookaheadTime;
        final double kLookaheadSearchDt = 0.01;
        TimedState<Pose2dWithCurvature> lookahead_state = mCurrentTrajectory.preview(lookahead_time).state();
        double actual_lookahead_distance = mSetpoint.state().distance(lookahead_state.state());
        while (actual_lookahead_distance < Constants.kPathMinLookaheadDistance &&
                mCurrentTrajectory.getRemainingProgress() > lookahead_time) {
            lookahead_time += kLookaheadSearchDt;
            lookahead_state = mCurrentTrajectory.preview(lookahead_time).state();
            actual_lookahead_distance = mSetpoint.state().distance(lookahead_state.state());
        }
        if (actual_lookahead_distance < Constants.kPathMinLookaheadDistance) {
            lookahead_state = new TimedState<>(new Pose2dWithCurvature(lookahead_state.state()
                    .getPose().transformBy(Pose2d.fromTranslation(new Translation2d(
                            (mIsReversed ? -1.0 : 1.0) * (Constants.kPathMinLookaheadDistance -
                                    actual_lookahead_distance), 0.0))), 0.0), lookahead_state.t()
                    , lookahead_state.velocity(), lookahead_state.acceleration());
        }
        
        SmartDashboard.putNumber("Path X", lookahead_state.state().getTranslation().x());
        SmartDashboard.putNumber("Path Y", lookahead_state.state().getTranslation().y());
        SmartDashboard.putNumber("Path Velocity", lookahead_state.velocity() / Constants.kSwerveMaxSpeedFeetPerSecond);

        Translation2d lookaheadTranslation = new Translation2d(current_state.getTranslation(), 
        		lookahead_state.state().getTranslation());
        Rotation2d steeringDirection = lookaheadTranslation.direction();
        double normalizedSpeed = mSetpoint.velocity() / Constants.kSwerveMaxSpeedFeetPerSecond;
        if(normalizedSpeed < defaultCook && useDefaultCook){ 
        	normalizedSpeed = defaultCook;
        }else{
        	useDefaultCook = false;
        }
        
        final Translation2d steeringVector = Translation2d.fromPolar(steeringDirection, normalizedSpeed);
        
        //System.out.println(steeringVector.toString());
        
        return steeringVector;
    }

    public Translation2d update(double timestamp, Pose2d current_state) {
        if (mCurrentTrajectory == null) return Translation2d.identity();

        if (mCurrentTrajectory.getProgress() == 0.0 && !Double.isFinite(mLastTime)) {
            mLastTime = timestamp;
        }

        mDt = timestamp - mLastTime;
        mLastTime = timestamp;
        
        double minimizedDistance = distance(current_state, 0.0);
        double searchStepSize = 1.0;
        double previewQuantity = 0.0;
    	double searchDirection = 1.0;
    	double forwardDistance = distance(current_state, previewQuantity + searchStepSize);
    	double reverseDistance = distance(current_state, previewQuantity - searchStepSize);
    	searchDirection = Math.signum(reverseDistance - forwardDistance);
        while(searchStepSize > 0.001){
/*        	for(double progress = previewQuantity; 
        			(searchDirection == 1.0) ? (progress <= mCurrentTrajectory.getRemainingProgress()) : (-progress < mCurrentTrajectory.getProgress()); 
        			progress += (searchStepSize * searchDirection)){
        		double distance = distance(current_state, progress);
        		if(distance < minimizedDistance) previewQuantity = progress;
        	}/**/
        	if(Util.epsilonEquals(distance(current_state, previewQuantity), 0.0, 0.001)) break;
        	while(/* next point is closer than current point */ distance(current_state, previewQuantity + searchStepSize*searchDirection) < 
        			distance(current_state, previewQuantity)) {
        		/* move to next point */
        		previewQuantity += searchStepSize*searchDirection;
        	}
        	searchStepSize /= 10.0;
        	searchDirection *= -1;
        }
        
        TrajectorySamplePoint<TimedState<Pose2dWithCurvature>> sample_point = mCurrentTrajectory.advance(previewQuantity);
        mSetpoint = sample_point.state();
        /*SmartDashboard.putNumber("Path X", mSetpoint.state().getTranslation().x());
        SmartDashboard.putNumber("Path Y", mSetpoint.state().getTranslation().y());
        SmartDashboard.putNumber("Path Velocity", mSetpoint.velocity() / Constants.kSwerveMaxSpeedFeetPerSecond);*/

        if (!mCurrentTrajectory.isDone()) {
            mError = current_state.inverse().transformBy(mSetpoint.state().getPose());

            if (mFollowerType == FollowerType.PURE_PURSUIT) {
                mOutput = updatePurePursuit(current_state);
                interpolateHeading();
            }
        } else {
            // TODO Possibly switch to a pose stabilizing controller?
            mOutput = Translation2d.identity();
        }
        return mOutput;
    }
    
    private double distance(Pose2d current_state, double additional_progress){
    	return mCurrentTrajectory.preview(additional_progress).state().state().getPose().distance(current_state);
    }

    public boolean isDone() {
        return mCurrentTrajectory != null && mCurrentTrajectory.isDone();
    }

    public Pose2d error() {
        return mError;
    }

    public TimedState<Pose2dWithCurvature> setpoint() {
        return mSetpoint;
    }
}