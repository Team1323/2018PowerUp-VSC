package com.team1323.frc2018.auto.modes;

import com.team1323.frc2018.Constants;
import com.team1323.frc2018.auto.AutoModeBase;
import com.team1323.frc2018.auto.AutoModeEndedException;
import com.team1323.frc2018.auto.actions.FollowPathAction;
import com.team1323.frc2018.auto.actions.ResetPoseAction;
import com.team1323.frc2018.auto.actions.WaitAction;
import com.team1323.frc2018.auto.actions.WaitForElevatorAction;
import com.team1323.frc2018.auto.actions.WaitForHeadingAction;
import com.team1323.frc2018.auto.actions.WaitForWallAction;
import com.team1323.frc2018.auto.actions.WaitToFinishPathAction;
import com.team1323.frc2018.auto.actions.WaitToIntakeCubeAction;
import com.team1323.frc2018.auto.actions.WaitToPassXCoordinateAction;
import com.team1323.frc2018.auto.actions.WaitToPassYCoordinateAction;
import com.team1323.frc2018.pathfinder.PathManager;
import com.team1323.frc2018.subsystems.Intake;
import com.team1323.frc2018.subsystems.Intake.IntakeState;
import com.team1323.frc2018.subsystems.Superstructure;
import com.team1323.frc2018.subsystems.Swerve;
import com.team1323.frc2018.subsystems.Wrist;

import edu.wpi.first.wpilibj.Timer;

public class RightSwitchLeftScaleMode extends AutoModeBase {
	Superstructure s;
	Intake intake;
	
	public RightSwitchLeftScaleMode(){
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
		s.request(s.elevatorWristIntakeConfig(
				Constants.kElevatorIntakingHeight, 
				Constants.kWristIntakingAngle, 
				IntakeState.INTAKING));
		runAction(new WaitToFinishPathAction());
		runAction(new WaitForElevatorAction());
		runAction(new FollowPathAction(PathManager.mRightmostCubePickup, -170.0));
		s.request(intake.stateRequest(IntakeState.INTAKING_WIDE));
		runAction(new WaitForWallAction(2.0));
		s.request(intake.stateRequest(IntakeState.INTAKING));
		runAction(new WaitToIntakeCubeAction(0.75));
		System.out.println("Intaken at: " + (Timer.getFPGATimestamp() - startTime));
		runAction(new FollowPathAction(PathManager.mRightCubeToLeftScale, -180.0));
		runAction(new WaitAction(0.25));
		s.request(Wrist.getInstance().angleRequest(Constants.kWristPrimaryStowAngle));
		runAction(new WaitToPassYCoordinateAction(17.5));
		Swerve.getInstance().setAbsolutePathHeading(-330.0);
		runAction(new WaitForHeadingAction(-340.0, -320.0));
		s.request(s.elevatorWristConfig(Constants.kELevatorBalancedScaleHeight, 35.0));
		runAction(new WaitToFinishPathAction());
		//runAction(new WaitToPassXCoordinateAction(22.75));
		runAction(new WaitForElevatorAction());
		s.request(intake.ejectRequest(Constants.kIntakeEjectOutput));
		System.out.println("Second cube scored at: " + (Timer.getFPGATimestamp() - startTime));
		runAction(new WaitAction(0.25));
		runAction(new FollowPathAction(PathManager.mLeftScaleToFirstCube, -225.0));
		runAction(new WaitAction(0.75));
		s.request(s.elevatorWristIntakeConfig(
				Constants.kElevatorIntakingHeight, 
				Constants.kWristIntakingAngle, 
				IntakeState.INTAKING));
		runAction(new WaitToIntakeCubeAction(3.0));
		System.out.println("Third cube intaken at: " + (Timer.getFPGATimestamp() - startTime));
		runAction(new FollowPathAction(PathManager.mLeftCubeToLeftScale, -330.0));
		runAction(new WaitAction(0.25));
		s.request(s.elevatorWristConfig(Constants.kELevatorBalancedScaleHeight, 35.0));
		runAction(new WaitToPassXCoordinateAction(22.75));
		runAction(new WaitForElevatorAction());
		runAction(new WaitForHeadingAction(-340.0 ,-320.0));
		s.request(intake.ejectRequest(Constants.kIntakeWeakEjectOutput));
		System.out.println("Third cube scored at: " + (Timer.getFPGATimestamp() - startTime));
	}

}
