package com.team1323.frc2018.auto.modes;

import java.util.Arrays;
import java.util.List;

import com.team1323.frc2018.Constants;
import com.team1323.frc2018.auto.AutoModeBase;
import com.team1323.frc2018.auto.AutoModeEndedException;
import com.team1323.frc2018.auto.actions.DriveStraightAction;
import com.team1323.frc2018.auto.actions.ResetPoseAction;
import com.team1323.frc2018.auto.actions.SetTrajectoryAction;
import com.team1323.frc2018.auto.actions.WaitAction;
import com.team1323.frc2018.auto.actions.WaitForElevatorAction;
import com.team1323.frc2018.auto.actions.WaitForWallAction;
import com.team1323.frc2018.auto.actions.WaitToFinishPathAction;
import com.team1323.frc2018.auto.actions.WaitToIntakeCubeAction;
import com.team1323.frc2018.auto.actions.WaitToPassYCoordinateAction;
import com.team1323.frc2018.subsystems.Intake;
import com.team1323.frc2018.subsystems.Intake.IntakeState;
import com.team1323.frc2018.subsystems.Superstructure;
import com.team1323.frc2018.subsystems.Swerve;
import com.team1323.frc2018.subsystems.Wrist;
import com.team254.lib.geometry.Pose2dWithCurvature;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.trajectory.Trajectory;
import com.team254.lib.trajectory.timing.TimedState;

import edu.wpi.first.wpilibj.Timer;

public class RightScaleMode extends AutoModeBase{
	Superstructure s;
	Intake intake;
	
	private static List<Trajectory<TimedState<Pose2dWithCurvature>>> paths = Arrays.asList(trajectories.startToRightScale,
		trajectories.rightScaleToFirstCube, trajectories.alternateRightCubeToRightScale, trajectories.alternateRightScaleToSecondCube);
	public static List<Trajectory<TimedState<Pose2dWithCurvature>>> getPaths(){
		return paths;
	}

	public RightScaleMode(){
		s = Superstructure.getInstance();
		intake = Intake.getInstance();
	}

	@Override
	protected void routine() throws AutoModeEndedException {
		double startTime = Timer.getFPGATimestamp();
		runAction(new ResetPoseAction(Constants.kRobotLeftStartingPose));
		s.request(intake.stateRequest(IntakeState.CLAMPING));
		s.request(s.wrist.angleRequest(Constants.kWristPrimaryStowAngle));
		runAction(new SetTrajectoryAction(trajectories.startToRightScale, -90.0, 0.75));
		runAction(new WaitToPassYCoordinateAction(15.0));
		s.request(s.elevatorWristConfig(Constants.kELevatorBalancedScaleHeight, 60.0));
		runAction(new WaitToPassYCoordinateAction(17.0));
		Swerve.getInstance().setAbsolutePathHeading(-40.0);
		runAction(new WaitToFinishPathAction());
		runAction(new WaitForElevatorAction());
		s.request(intake.ejectRequest(Constants.kIntakeEjectOutput));
		System.out.println("First cube scored at: " + (Timer.getFPGATimestamp() - startTime));
		runAction(new WaitAction(0.25));
		runAction(new SetTrajectoryAction(trajectories.rightScaleToFirstCube, -180.0, 1.0));
		runAction(new WaitAction(0.5));
		s.request(s.elevatorWristIntakeConfig(
				Constants.kElevatorIntakingHeight, 
				Constants.kWristIntakingAngle, 
				IntakeState.INTAKING_WIDE));
		runAction(new WaitForWallAction(2.5));
		System.out.println("Right Switch X: " + Swerve.getInstance().getPose().getTranslation().x());
		s.request(intake.stateRequest(IntakeState.INTAKING));
		runAction(new WaitToIntakeCubeAction(0.75));
		if(!intake.hasCube()){
			runAction(new DriveStraightAction(Rotation2d.fromDegrees(180).toTranslation().scale(0.35)));
			runAction(new WaitToIntakeCubeAction(2.0));
			if(intake.hasCube()){
				Swerve.getInstance().setXCoordinate(19.0);
			}
		}
		System.out.println("Second cube intaken at: " + (Timer.getFPGATimestamp() - startTime));
		runAction(new SetTrajectoryAction(trajectories.alternateRightCubeToRightScale, -55.0, 1.25));
		runAction(new WaitAction(0.25));
		s.request(s.elevatorWristConfig(Constants.kELevatorBalancedScaleHeight, 55.0));
		runAction(new WaitToFinishPathAction());
		runAction(new WaitForElevatorAction());
		s.request(intake.ejectRequest(Constants.kIntakeEjectOutput));
		System.out.println("Second cube scored at: " + (Timer.getFPGATimestamp() - startTime));
		runAction(new WaitAction(0.25));
		runAction(new SetTrajectoryAction(trajectories.alternateRightScaleToSecondCube, -135.0, 0.5));
		runAction(new WaitAction(0.75));
		s.request(s.elevatorWristIntakeConfig(
				Constants.kElevatorIntakingHeight, 
				Constants.kWristIntakingAngle, 
				IntakeState.INTAKING));
		runAction(new WaitToIntakeCubeAction(3.5));
		if(!intake.hasCube()){
			s.request(s.elevatorWristIntakeConfig(
					Constants.kElevatorIntakingHeight, 
					Constants.kWristIntakingAngle, 
					IntakeState.INTAKING_WIDE));
			runAction(new DriveStraightAction(Rotation2d.fromDegrees(-135.0).toTranslation().scale(0.35)));
			runAction(new WaitToIntakeCubeAction(1.5));
		}
		System.out.println("Third Cube intaken at: " + (Timer.getFPGATimestamp() - startTime));
		runAction(new SetTrajectoryAction(trajectories.alternateRightCubeToRightScale, -55.0, 1.25));
		runAction(new WaitAction(0.25));
		/*s.request(Wrist.getInstance().angleRequest(Constants.kWristPrimaryStowAngle));
		runAction(new WaitToFinishPathAction());*/
		s.request(s.elevatorWristConfig(Constants.kELevatorBalancedScaleHeight, 55.0));
		runAction(new WaitToFinishPathAction());
		runAction(new WaitForElevatorAction());
		s.request(intake.ejectRequest(Constants.kIntakeEjectOutput));
		System.out.println("Third cube scored at: " + (Timer.getFPGATimestamp() - startTime));
	}

}
