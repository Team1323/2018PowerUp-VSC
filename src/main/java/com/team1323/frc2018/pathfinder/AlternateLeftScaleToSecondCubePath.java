package com.team1323.frc2018.pathfinder;

import com.team1323.frc2018.Constants;

import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Waypoint;

public class AlternateLeftScaleToSecondCubePath extends PathfinderPath{

	public AlternateLeftScaleToSecondCubePath(){
		super.points = new Waypoint[]{
			new Waypoint(21.75, 6.0, Pathfinder.d2r(180)),
			new Waypoint(Constants.kLeftSwitchFarCorner.x() + 1.7, Constants.kLeftSwitchFarCorner.y() + 3.0, Pathfinder.d2r(90))
		};
		super.maxAccel = 5.0;
		super.defaultSpeed = 4.5;
		super.rotationScalar = 0.75;
		super.lookaheadPoints = 15;
	}
	
}
