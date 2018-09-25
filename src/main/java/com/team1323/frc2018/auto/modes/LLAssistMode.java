package com.team1323.frc2018.auto.modes;

import java.util.Arrays;
import java.util.List;

import com.team1323.frc2018.Constants;
import com.team1323.frc2018.auto.AutoModeBase;
import com.team1323.frc2018.auto.AutoModeEndedException;
import com.team1323.frc2018.auto.actions.DriveStraightAction;
import com.team1323.frc2018.auto.actions.FollowPathAction;
import com.team1323.frc2018.auto.actions.ResetPoseAction;
import com.team1323.frc2018.auto.actions.SetTrajectoryAction;
import com.team1323.frc2018.auto.actions.WaitAction;
import com.team1323.frc2018.auto.actions.WaitForElevatorAction;
import com.team1323.frc2018.auto.actions.WaitToFinishPathAction;
import com.team1323.frc2018.auto.actions.WaitToIntakeCubeAction;
import com.team1323.frc2018.auto.actions.WaitToPassXCoordinateAction;
import com.team1323.frc2018.pathfinder.PathManager;
import com.team1323.frc2018.subsystems.Elevator;
import com.team1323.frc2018.subsystems.Intake;
import com.team1323.frc2018.subsystems.Intake.IntakeState;
import com.team1323.frc2018.subsystems.RequestList;
import com.team1323.frc2018.subsystems.Superstructure;
import com.team1323.frc2018.subsystems.Wrist;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Pose2dWithCurvature;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;
import com.team254.lib.trajectory.Trajectory;
import com.team254.lib.trajectory.timing.TimedState;

import edu.wpi.first.wpilibj.Timer;

public class LLAssistMode extends AutoModeBase{
	Superstructure s;
	Intake intake;

	private List<Trajectory<TimedState<Pose2dWithCurvature>>> paths = Arrays.asList(trajectories.frontLeftSwitch,
		trajectories.frontLeftSwitchToOuterCube, trajectories.outerCubeToFrontLeftSwitch, trajectories.frontLeftSwitchToMiddleCube,
		trajectories.middleCubeToLeftScale);

	@Override
	public List<Trajectory<TimedState<Pose2dWithCurvature>>> getPaths(){
		return paths;
	}

	public LLAssistMode(){
		s = Superstructure.getInstance();
		intake = Intake.getInstance();
	}
	
	@Override
	protected void routine() throws AutoModeEndedException {
		double startTime = Timer.getFPGATimestamp();
		runAction(new ResetPoseAction(Constants.kRobotStartingPose.transformBy(Pose2d.fromTranslation(new Translation2d(/*-0.25*/0.0, 0.0)))));
		s.request(intake.stateRequest(IntakeState.CLAMPING));
		runAction(new SetTrajectoryAction(trajectories.frontLeftSwitch, 0.0, 1.0));
		runAction(new WaitAction(0.5));
		s.request(s.elevatorWristConfig(Constants.kElevatorSecondCubeHeight, 75.0));
		runAction(new WaitToFinishPathAction());
		s.request(intake.ejectRequest(Constants.kIntakeEjectOutput));
		System.out.println("First Cube Scored at: " + (Timer.getFPGATimestamp() - startTime));
		runAction(new WaitAction(0.25));
		runAction(new SetTrajectoryAction(trajectories.frontLeftSwitchToOuterCube, 0.0, 1.0));
		runAction(new WaitAction(0.35));
		s.request(s.elevatorWristIntakeConfig(
				Constants.kElevatorIntakingHeight, 
				Constants.kWristIntakingAngle,
				IntakeState.INTAKING_WIDE));
		runAction(new WaitToFinishPathAction(3.5));
		s.request(intake.stateRequest(IntakeState.INTAKING));
		runAction(new WaitToIntakeCubeAction(1.0));
		if(!intake.hasCube()){
			s.request(intake.stateRequest(IntakeState.INTAKING));
			runAction(new DriveStraightAction(Rotation2d.fromDegrees(0).toTranslation().scale(0.3)));
			runAction(new WaitToIntakeCubeAction(1.0));
		}
		System.out.println("Second Cube Intaken at: " + (Timer.getFPGATimestamp() - startTime));
		runAction(new SetTrajectoryAction(trajectories.outerCubeToFrontLeftSwitch, 0.0, 1.0));
		runAction(new WaitAction(0.5));
		s.request(s.elevatorWristConfig(Constants.kElevatorSecondCubeHeight, 85.0));
		runAction(new WaitToPassXCoordinateAction(Constants.kLeftSwitchCloseCorner.x() - Constants.kRobotHalfLength - 1.0));
		s.request(intake.ejectRequest(Constants.kIntakeEjectOutput));
		System.out.println("Second Cube Scored at: " + (Timer.getFPGATimestamp() - startTime));
		runAction(new WaitAction(0.25));
		runAction(new SetTrajectoryAction(trajectories.frontLeftSwitchToMiddleCube, 0.0, 1.0));
		runAction(new WaitAction(0.25));
		s.request(s.elevatorWristIntakeConfig(
				Constants.kElevatorSecondCubeHeight, 
				Constants.kWristIntakingAngle,
				IntakeState.INTAKING_WIDE));
		runAction(new WaitToFinishPathAction(3.5));
		s.request(intake.stateRequest(IntakeState.INTAKING));
		runAction(new WaitToIntakeCubeAction(1.0));
		if(!intake.hasCube()){
			s.request(intake.stateRequest(IntakeState.INTAKING));
			runAction(new DriveStraightAction(Rotation2d.fromDegrees(0).toTranslation().scale(0.3)));
			runAction(new WaitToIntakeCubeAction(1.0));
		}
		System.out.println("Third Cube Intaken at: " + (Timer.getFPGATimestamp() - startTime));
        runAction(new SetTrajectoryAction(trajectories.middleCubeToLeftScale, 90.0, 0.5));
        runAction(new WaitAction(0.5));
        s.request(s.wristIntakeConfig(Constants.kWristPrimaryStowAngle, IntakeState.CLAMPING));
        runAction(new WaitToPassXCoordinateAction(Constants.kLeftSwitchFarCorner.x() - 1.0));
        s.request(s.elevatorWristConfig(4.5, 60.0));
        runAction(new WaitToFinishPathAction());
        s.request(intake.ejectRequest(Constants.kIntakeEjectOutput));
        System.out.println("Third cube scored at: " + (Timer.getFPGATimestamp() - startTime));
	}
}
	
