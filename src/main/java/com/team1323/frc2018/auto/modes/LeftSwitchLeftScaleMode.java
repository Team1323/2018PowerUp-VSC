package com.team1323.frc2018.auto.modes;

import com.team1323.frc2018.Constants;
import com.team1323.frc2018.auto.AutoModeBase;
import com.team1323.frc2018.auto.AutoModeEndedException;
import com.team1323.frc2018.auto.actions.DriveStraightAction;
import com.team1323.frc2018.auto.actions.FollowPathAction;
import com.team1323.frc2018.auto.actions.ResetPoseAction;
import com.team1323.frc2018.auto.actions.WaitAction;
import com.team1323.frc2018.auto.actions.WaitForElevatorAction;
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

public class LeftSwitchLeftScaleMode extends AutoModeBase {
	Superstructure s;
	Intake intake;
	
	public LeftSwitchLeftScaleMode(){
		s = Superstructure.getInstance();
		intake = Intake.getInstance();
	}

	@Override
	protected void routine() throws AutoModeEndedException {
		double startTime = Timer.getFPGATimestamp();
		runAction(new ResetPoseAction(Constants.kRobotStartingPose));
		s.request(intake.stateRequest(IntakeState.CLAMPING));
		runAction(new FollowPathAction(PathManager.mLeftSwitchDropoff, 0.0));
		runAction(new WaitAction(0.5));
		Swerve.getInstance().setAbsolutePathHeading(90.0);
		runAction(new WaitAction(0.5));
		s.request(s.elevatorWristConfig(Constants.kElevatorSwitchHeight, 20.0));
		runAction(new WaitToPassXCoordinateAction(Constants.kLeftSwitchCloseCorner.x()));
		s.request(intake.ejectRequest(Constants.kIntakeEjectOutput));
		runAction(new WaitToPassXCoordinateAction(Constants.kLeftSwitchFarCorner.x()));
		runAction(new WaitAction(0.25));
		s.request(s.elevatorWristIntakeConfig(
				Constants.kElevatorIntakingHeight, 
				Constants.kWristIntakingAngle, 
				IntakeState.INTAKING));
		runAction(new WaitToFinishPathAction());
		runAction(new WaitForElevatorAction());
		double pathStartTime = Timer.getFPGATimestamp();
		runAction(new FollowPathAction(PathManager.mLeftmostCubePickup, 160.0));
		s.request(intake.stateRequest(IntakeState.INTAKING_WIDE));
		runAction(new WaitForWallAction(2.0));
		s.request(intake.stateRequest(IntakeState.INTAKING));
		runAction(new WaitToIntakeCubeAction(1.5));
		System.out.println("Intaken at: " + (Timer.getFPGATimestamp() - startTime));
		System.out.println("Intaken in : " + (Timer.getFPGATimestamp() - pathStartTime));
		runAction(new FollowPathAction(PathManager.mLeftCubeToLeftScale, 30.0));
		runAction(new WaitAction(0.5));
		s.request(s.elevatorWristConfig(Constants.kELevatorBalancedScaleHeight, 35.0));
		//runAction(new WaitToPassXCoordinateAction(23.0));
		runAction(new WaitToFinishPathAction());
		runAction(new WaitForElevatorAction());
		//runAction(new WaitForHeadingAction(35.0, 55.0));
		s.request(intake.ejectRequest(Constants.kIntakeEjectOutput));
		System.out.println("Second Cube scored at: " + (Timer.getFPGATimestamp() - startTime));
		runAction(new WaitAction(0.5));
		runAction(new FollowPathAction(PathManager.mLeftScaleToSecondCube, 135.0));
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
			runAction(new DriveStraightAction(Rotation2d.fromDegrees(135.0).toTranslation().scale(0.35)));
			runAction(new WaitToIntakeCubeAction(1.5));
		}
		System.out.println("Third Cube intaken at: " + (Timer.getFPGATimestamp() - startTime));
		runAction(new FollowPathAction(PathManager.mSecondLeftCubeToScale, 45.0));
		runAction(new WaitAction(0.5));
		s.request(s.elevatorWristConfig(Constants.kELevatorBalancedScaleHeight, 35.0));
		//runAction(new WaitToPassXCoordinateAction(23.0));
		runAction(new WaitToFinishPathAction());
		//runAction(new WaitForHeadingAction(35.0, 55.0));
		runAction(new WaitForElevatorAction());
		s.request(intake.ejectRequest(Constants.kIntakeWeakEjectOutput));
		System.out.println("Third Cube scored at: " + (Timer.getFPGATimestamp() - startTime));
		runAction(new WaitAction(0.5));
		runAction(new FollowPathAction(PathManager.mLeftScaleToThirdCube, 135.0));
		runAction(new WaitAction(0.5));
		s.request(s.elevatorWristIntakeConfig(
				Constants.kElevatorIntakingHeight, 
				Constants.kWristIntakingAngle, 
				IntakeState.INTAKING));
		runAction(new WaitToIntakeCubeAction(4.0));
		System.out.println("Fourth Cube intaken at: " + (Timer.getFPGATimestamp() - startTime));
	}

}
