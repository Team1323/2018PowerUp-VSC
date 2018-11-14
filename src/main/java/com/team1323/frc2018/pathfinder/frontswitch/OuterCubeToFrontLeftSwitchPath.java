package com.team1323.frc2018.pathfinder.frontswitch;

import com.team1323.frc2018.Constants;
import com.team1323.frc2018.pathfinder.PathfinderPath;

import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Waypoint;

public class OuterCubeToFrontLeftSwitchPath extends PathfinderPath{

	public OuterCubeToFrontLeftSwitchPath(){
		super.points = new Waypoint[]{
			new Waypoint(Constants.kLeftSwitchCloseCorner.x() - (3*Constants.kCubeWidth) - 2.15, Constants.kLeftSwitchCloseCorner.y() + Constants.kRobotHalfWidth + 5.1, Pathfinder.d2r(-120)),
			new Waypoint(Constants.kLeftSwitchCloseCorner.x() - Constants.kRobotHalfLength - 1.4, Constants.kLeftSwitchCloseCorner.y() + Constants.kRobotHalfWidth + 0.6, Pathfinder.d2r(0))
		};
		super.maxAccel = 8.0;
		super.defaultSpeed = 5.0;
		super.rotationScalar = 1.0;
		super.lookaheadPoints = 15;
		super.usePID = true;
	}
	
}
