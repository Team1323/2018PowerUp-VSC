package com.team1323.frc2018.pathfinder;

import com.team1323.frc2018.Constants;

import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Waypoint;

public class RightScaleToSecondCubePath extends PathfinderPath{
	public RightScaleToSecondCubePath(){
		super.points = new Waypoint[]{
			new Waypoint(23.25, 22.5, Pathfinder.d2r(180.0)),
			new Waypoint(Constants.kRightSwitchFarCorner.x() + 3.5, Constants.kRightSwitchFarCorner.y() + Constants.kRobotHalfLength - 3.25, Pathfinder.d2r(-90.0))
		};//3.9 -2.5
		super.maxAccel = 5.0;
		super.defaultSpeed = 4.5;
		super.rotationScalar = 0.5;
		super.lookaheadPoints = 15;
	}
}
