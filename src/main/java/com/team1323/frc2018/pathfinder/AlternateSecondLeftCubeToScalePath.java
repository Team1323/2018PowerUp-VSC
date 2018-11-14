package com.team1323.frc2018.pathfinder;

import com.team1323.frc2018.Constants;

import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Waypoint;

public class AlternateSecondLeftCubeToScalePath extends PathfinderPath{

	public AlternateSecondLeftCubeToScalePath(){
		super.points = new Waypoint[]{
			new Waypoint(Constants.kLeftSwitchFarCorner.x() + 1.7, Constants.kLeftSwitchFarCorner.y() + 3.0, Pathfinder.d2r(-75)),
			new Waypoint(22.6, 5.3, Pathfinder.d2r(0)),
		};
		super.maxAccel = 5.0;
		super.defaultSpeed = 4.5;
		super.rotationScalar = 0.75;
		super.lookaheadPoints = 20;
	}
	
}
