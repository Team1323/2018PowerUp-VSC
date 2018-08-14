package com.team1323.frc2018.loops;

import java.util.ArrayList;
import java.util.List;

import com.team1323.frc2018.Constants;
import com.team254.lib.geometry.Pose2dWithCurvature;
import com.team254.lib.geometry.Translation2d;
import com.team254.lib.spline.QuinticHermiteSpline;
import com.team254.lib.trajectory.TimedView;
import com.team254.lib.trajectory.Trajectory;
import com.team254.lib.trajectory.TrajectoryIterator;
import com.team254.lib.trajectory.timing.TimedState;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class QuinticPathTransmitter implements Loop{
	private static QuinticPathTransmitter instance = new QuinticPathTransmitter();
	public static QuinticPathTransmitter getInstance(){
		return instance;
	}
	
	public QuinticPathTransmitter(){
	}
	
	private List<TrajectoryIterator<TimedState<Pose2dWithCurvature>>> remainingTrajectories = new ArrayList<>();
	private TrajectoryIterator<TimedState<Pose2dWithCurvature>> currentTrajectory;
	private double t = 0;
	private boolean defaultCookReported = false;
	
	private double startingTime = 0.0;
	
	public void addPath(Trajectory<TimedState<Pose2dWithCurvature>> path){
		remainingTrajectories.add(new TrajectoryIterator<>(new TimedView<>(path)));
		currentTrajectory = null;
	}

	public void addPaths(List<Trajectory<TimedState<Pose2dWithCurvature>>> paths){
		paths.forEach((p) -> addPath(p));
	}

	@Override
	public void onStart(double timestamp) {
		
	}

	@Override
	public void onLoop(double timestamp) {
		if(currentTrajectory == null){
			if(remainingTrajectories.isEmpty()){
				return;
			}
			
			currentTrajectory = remainingTrajectories.remove(0);
			defaultCookReported = false;
			t = 0;
			startingTime = timestamp;
		}
		
		t = timestamp - startingTime;
		TimedState<Pose2dWithCurvature> state = currentTrajectory.preview(t).state();
		Translation2d pos = state.state().getTranslation();
		SmartDashboard.putNumberArray("Path Pose", new double[]{pos.x(), pos.y(), 0.0, state.velocity() / Constants.kSwerveMaxSpeedFeetPerSecond});
		
		if(state.acceleration() < 0.0 && !defaultCookReported){
			System.out.println("Optimal default cook: " + state.velocity());
			defaultCookReported = true;
		}

	    if(t >= currentTrajectory.trajectory().getLastState().t()){
	    	System.out.println("Path should take " + currentTrajectory.trajectory().getLastState().t() + " seconds");
	    	currentTrajectory = null;
	    }
	}

	@Override
	public void onStop(double timestamp) {
		
	}

}
