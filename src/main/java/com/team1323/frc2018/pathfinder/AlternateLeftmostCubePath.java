package com.team1323.frc2018.pathfinder;

import com.team1323.frc2018.Constants;

import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Waypoint;

public class AlternateLeftmostCubePath extends PathfinderPath{

	public AlternateLeftmostCubePath(){
		super.points = new Waypoint[]{
				/*new Waypoint(Constants.kLeftSwitchFarCorner.x() + 5.0, Constants.kLeftSwitchFarCorner.y() - Constants.kRobotHalfLength + 0.25, Pathfinder.d2r(135)),
				new Waypoint(Constants.kLeftSwitchFarCorner.x() + 3.65, Constants.kLeftSwitchFarCorner.y() + 0.25, Pathfinder.d2r(0))//3.7*/
				new Waypoint(21.5, Constants.kLeftSwitchCloseCorner.y() - Constants.kRobotHalfLength - 1.5, Pathfinder.d2r(135)),
				new Waypoint(Constants.kLeftSwitchFarCorner.x() + 3.15, Constants.kLeftSwitchFarCorner.y() + 0.15, Pathfinder.d2r(135))
			};
			super.defaultSpeed = 4.0;
			super.maxSpeed = 4.0;//4.0
			super.maxAccel = 10.0;
			super.rotationScalar = 1.0;
			super.lookaheadPoints = 10;
	}
	
}
