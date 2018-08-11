package com.team1323.frc2018.loops;

import java.util.ArrayList;
import java.util.List;

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
	
	private List<QuinticHermiteSpline> remainingPaths = new ArrayList<>();
	private QuinticHermiteSpline currentPath;
	private List<QuinticHermiteSpline> cachedPaths = remainingPaths;
	
	private List<TrajectoryIterator<TimedState<Pose2dWithCurvature>>> remainingTrajectories = new ArrayList<>();
	private TrajectoryIterator<TimedState<Pose2dWithCurvature>> currentTrajectory;
	
	private final double stepSize = 0.01;
	private double t = 0;
	
	private double startingTime = 0.0;
	
	private int index = 0;
	
	/*public void addPaths(List<PathfinderPath> paths){
		remainingPaths = new ArrayList<>();
		cachedPaths = remainingPaths;
		
		for(PathfinderPath path : paths){
			List<QuinticHermiteSpline> splines = path.toSplines();
			for(QuinticHermiteSpline spline : splines){
				remainingPaths.add(spline);
			}
		}
		
		currentPath = null;
	}*/
	
	public void addPath(Trajectory<TimedState<Pose2dWithCurvature>> path){
		remainingTrajectories.add(new TrajectoryIterator<>(new TimedView<>(path)));
		currentTrajectory = null;
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
			t = 0;
			index = 0;
			startingTime = timestamp;
		}
		
		t = timestamp - startingTime;
		//Translation2d pos = currentPath.getPoint(t);
		Translation2d pos = currentTrajectory.preview(t).state().state().getTranslation();
	    /*SmartDashboard.putNumber("Path X", pos.x());
	    SmartDashboard.putNumber("Path Y", pos.y());
	    SmartDashboard.putNumber("Path Velocity", currentTrajectory.preview(t).state().velocity() / 12.0);*/
	    SmartDashboard.putNumberArray("Path Pose", new double[]{pos.x(), pos.y(), 0.0, currentTrajectory.preview(t).state().velocity() / 12.5});
	    //t += stepSize;
	    //index++;
	    
	    if(t >= currentTrajectory.trajectory().getLastState().t()){
	    	System.out.println("Path should take " + currentTrajectory.trajectory().getLastState().t() + " seconds");
	    	currentTrajectory = null;
	    }
	}

	@Override
	public void onStop(double timestamp) {
		
	}

}
