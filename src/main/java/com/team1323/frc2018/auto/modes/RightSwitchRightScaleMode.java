package com.team1323.frc2018.auto.modes;

import com.team1323.frc2018.Constants;
import com.team1323.frc2018.auto.AutoModeBase;
import com.team1323.frc2018.auto.AutoModeEndedException;
import com.team1323.frc2018.auto.actions.DriveStraightAction;
import com.team1323.frc2018.auto.actions.FollowPathAction;
import com.team1323.frc2018.auto.actions.ResetPoseAction;
import com.team1323.frc2018.auto.actions.WaitAction;
import com.team1323.frc2018.auto.actions.WaitForElevatorAction;
import com.team1323.frc2018.auto.actions.WaitForHeadingAction;
import com.team1323.frc2018.auto.actions.WaitForWallAction;
import com.team1323.frc2018.auto.actions.WaitToFinishPathAction;
import com.team1323.frc2018.auto.actions.WaitToIntakeCubeAction;
import com.team1323.frc2018.auto.actions.WaitToPassXCoordinateAction;
import com.team1323.frc2018.pathfinder.PathManager;
import com.team1323.frc2018.subsystems.Intake;
import com.team1323.frc2018.subsystems.Intake.IntakeState;
import com.team1323.frc2018.subsystems.Superstructure;
import com.team1323.frc2018.subsystems.Swerve;
import com.team254.lib.geometry.Rotation2d;

import edu.wpi.first.wpilibj.Timer;

public class RightSwitchRightScaleMode extends AutoModeBase{
	Superstructure s;
	Intake intake;
	
	public RightSwitchRightScaleMode(){
		s = Superstructure.getInstance();
		intake = Intake.getInstance();
	}
	
	@Override
	protected void routine() throws AutoModeEndedException {
		double startTime = Timer.getFPGATimestamp();
		runAction(new ResetPoseAction(Constants.kRobotStartingPose));
		s.request(intake.stateRequest(IntakeState.CLAMPING));
		runAction(new FollowPathAction(PathManager.mRightSwitchDropoff, 0.0));
		runAction(new WaitAction(0.5));
		Swerve.getInstance().setAbsolutePathHeading(-90.0);
		s.request(s.elevatorWristConfig(Constants.kElevatorSwitchHeight, 20.0));
		runAction(new WaitToPassXCoordinateAction(Constants.kRightSwitchCloseCorner.x() - 1.0));
		s.request(intake.ejectRequest(Constants.kIntakeEjectOutput));
		runAction(new WaitToPassXCoordinateAction(Constants.kRightSwitchFarCorner.x()));
		runAction(new WaitAction(0.25));
		s.request(s.elevatorWristConfig(Constants.kElevatorIntakingHeight, Constants.kWristIntakingAngle));
		runAction(new WaitToFinishPathAction());
		runAction(new WaitForElevatorAction());
		runAction(new FollowPathAction(PathManager.mRightmostCubePickup, -170.0));
		s.request(intake.stateRequest(IntakeState.INTAKING_WIDE));
		runAction(new WaitForWallAction(2.0));
		s.request(intake.stateRequest(IntakeState.INTAKING));
		runAction(new WaitToIntakeCubeAction(0.75));
		System.out.println("Intaken at: " + (Timer.getFPGATimestamp() - startTime));
		runAction(new FollowPathAction(PathManager.mRightCubeToRightScale, -50.0));//-45
		runAction(new WaitAction(0.25));
		s.request(s.elevatorWristConfig(Constants.kELevatorBalancedScaleHeight, 35.0));
		runAction(new WaitToPassXCoordinateAction(23.5));
		runAction(new WaitForElevatorAction());
		runAction(new WaitForHeadingAction(-60.0, -40.0));
		s.request(intake.ejectRequest(Constants.kIntakeWeakEjectOutput));
		System.out.println("Second Cube scored at: " + (Timer.getFPGATimestamp() - startTime));
		runAction(new WaitAction(0.25));
		runAction(new FollowPathAction(PathManager.mRightScaleToSecondCube, -135.0));
		runAction(new WaitAction(0.5));
		s.request(s.elevatorWristIntakeConfig(
				Constants.kElevatorIntakingHeight, 
				Constants.kWristIntakingAngle, 
				IntakeState.INTAKING));
		runAction(new WaitToIntakeCubeAction(3.5));
		if(!intake.hasCube()){
			s.request(s.elevatorWristIntakeConfig(
					Constants.kElevatorIntakingHeight, 
					Constants.kWristIntakingAngle, 
					IntakeState.INTAKING));
			runAction(new DriveStraightAction(Rotation2d.fromDegrees(-135.0).toTranslation().scale(0.35)));
			runAction(new WaitToIntakeCubeAction(1.5));
		}
		System.out.println("Third Cube intaken at: " + (Timer.getFPGATimestamp() - startTime));
		runAction(new FollowPathAction(PathManager.mSecondRightCubeToScale, -50.0));
		runAction(new WaitAction(0.25));
		s.request(s.elevatorWristConfig(Constants.kELevatorBalancedScaleHeight, 35.0));
		runAction(new WaitToPassXCoordinateAction(23.0));
		runAction(new WaitForHeadingAction(-60.0, -40.0));
		runAction(new WaitForElevatorAction());
		s.request(intake.ejectRequest(Constants.kIntakeWeakEjectOutput));
		System.out.println("Third Cube scored at: " + (Timer.getFPGATimestamp() - startTime));
		runAction(new WaitAction(0.5));
	}

}
