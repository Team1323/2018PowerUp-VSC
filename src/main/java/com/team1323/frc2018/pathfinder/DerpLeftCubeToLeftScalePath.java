package com.team1323.frc2018.pathfinder;

import com.team1323.frc2018.Constants;

import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Waypoint;

public class DerpLeftCubeToLeftScalePath extends PathfinderPath{

	public DerpLeftCubeToLeftScalePath(){
		super.points = new Waypoint[]{
			new Waypoint(Constants.kLeftSwitchFarCorner.x() + 3.5, Constants.kLeftSwitchFarCorner.y() + 0.5, Pathfinder.d2r(-45)),
			new Waypoint(21.75, 6.0, Pathfinder.d2r(-45))
		};
		super.maxSpeed = 8.0;
		super.maxAccel = 5.0;
		super.defaultSpeed = 3.5;
		super.rotationScalar = 1.25;
		super.lookaheadPoints = 20;
		super.rotationOverride = false;
	}
	
}
