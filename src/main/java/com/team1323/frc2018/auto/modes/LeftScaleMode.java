package com.team1323.frc2018.auto.modes;

import java.util.Arrays;

import com.team1323.frc2018.Constants;
import com.team1323.frc2018.auto.AutoModeBase;
import com.team1323.frc2018.auto.AutoModeEndedException;
import com.team1323.frc2018.auto.actions.DriveStraightAction;
import com.team1323.frc2018.auto.actions.FollowPathAction;
import com.team1323.frc2018.auto.actions.ResetPoseAction;
import com.team1323.frc2018.auto.actions.SetTrajectoryAction;
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
import com.team254.lib.geometry.Pose2dWithCurvature;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.trajectory.timing.TimedState;
import java.util.List;
import java.util.ArrayList;
import com.team254.lib.trajectory.Trajectory;

import edu.wpi.first.wpilibj.Timer;

public class LeftScaleMode extends AutoModeBase{
	Superstructure s;
	Intake intake;
	
	private static List<Trajectory<TimedState<Pose2dWithCurvature>>> paths = Arrays.asList(trajectories.startToLeftScale, trajectories.alternateLeftmostCube,
	trajectories.derpLeftCubeToLeftScale, trajectories.alternateLeftScaleToSecondCube,
	trajectories.alternateSecondLeftCubeToScale);
	public static List<Trajectory<TimedState<Pose2dWithCurvature>>> getPaths(){
		return paths;
	}

	public LeftScaleMode(){
		s = Superstructure.getInstance();
		intake = Intake.getInstance();
	}

	@Override
	protected void routine() throws AutoModeEndedException {
		double startTime = Timer.getFPGATimestamp();
		runAction(new ResetPoseAction(Constants.kRobotLeftStartingPose));
		s.request(intake.stateRequest(IntakeState.CLAMPING));
		runAction(new SetTrajectoryAction(trajectories.startToLeftScale, 50.0, 0.5));
		runAction(new WaitToPassXCoordinateAction(Constants.kLeftSwitchCloseCorner.x()));
		s.request(s.elevatorWristConfig(4.5, 66.0));//66 4.5
		runAction(new WaitToFinishPathAction());
		s.request(intake.ejectRequest(Constants.kIntakeStrongEjectOutput));
		System.out.println("First Cube Scored at: " + (Timer.getFPGATimestamp() - startTime));
		runAction(new WaitAction(0.4));
		runAction(new SetTrajectoryAction(trajectories.alternateLeftmostCube, 175.0, 1.0));
		runAction(new WaitForHeadingAction(80.0, 180.0));
		s.request(s.elevatorWristIntakeConfig(
				Constants.kElevatorIntakingHeight, 
				Constants.kWristIntakingAngle, 
				IntakeState.INTAKING_WIDE));
		runAction(new WaitForWallAction(3.0));
		s.request(intake.stateRequest(IntakeState.INTAKING));
		runAction(new WaitToIntakeCubeAction(1.0));
		System.out.println("Switch X: " + Swerve.getInstance().getPose().getTranslation().x());
		if(!intake.hasCube()){
			System.out.println("Drive straight action initiated");
			runAction(new DriveStraightAction(Rotation2d.fromDegrees(180.0).toTranslation().scale(0.3)));
			runAction(new WaitToIntakeCubeAction(1.5));
			if(intake.hasCube())
				Swerve.getInstance().setXCoordinate(19.0);
		}
		System.out.println("Second cube intaken at: " + (Timer.getFPGATimestamp() - startTime));
		runAction(new SetTrajectoryAction(trajectories.derpLeftCubeToLeftScale, 35.0, 1.25));
		runAction(new WaitAction(0.25));
		s.request(s.elevatorWristConfig(Constants.kELevatorBalancedScaleHeight, 60.0));
		runAction(new WaitToFinishPathAction());
		runAction(new WaitForElevatorAction());
		s.request(intake.ejectRequest(Constants.kIntakeEjectOutput));
		System.out.println("Second Cube scored at: " + (Timer.getFPGATimestamp() - startTime));
		runAction(new WaitAction(0.25));
		runAction(new SetTrajectoryAction(trajectories.alternateLeftScaleToSecondCube, 150.0, 0.75));
		runAction(new WaitAction(0.75));
		s.request(s.elevatorWristIntakeConfig(Constants.kElevatorIntakingHeight, Constants.kWristIntakingAngle, IntakeState.INTAKING_WIDE));
		runAction(new WaitToFinishPathAction());
		s.request(intake.stateRequest(IntakeState.INTAKING));
		runAction(new WaitToIntakeCubeAction(0.75));
		if(!intake.hasCube()){
			System.out.println("Drive straight action initiated");
			s.request(intake.stateRequest(IntakeState.INTAKING));
			runAction(new DriveStraightAction(Rotation2d.fromDegrees(120.0).toTranslation().scale(0.25)));
			runAction(new WaitToIntakeCubeAction(1.5));
		}
		System.out.println("Third Cube intaken at: " + (Timer.getFPGATimestamp() - startTime));
		runAction(new SetTrajectoryAction(trajectories.alternateSecondLeftCubeToScale, 55.0, 0.75));
		runAction(new WaitAction(0.25));
		s.request(s.elevatorWristConfig(Constants.kELevatorBalancedScaleHeight, 60.0));
		runAction(new WaitToFinishPathAction());
		runAction(new WaitForElevatorAction());
		s.request(intake.ejectRequest(Constants.kIntakeEjectOutput));
		System.out.println("Third Cube scored at: " + (Timer.getFPGATimestamp() - startTime));
	}
	
}
