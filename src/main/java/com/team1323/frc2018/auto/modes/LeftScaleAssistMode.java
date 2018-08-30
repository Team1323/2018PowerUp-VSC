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
import com.team1323.frc2018.auto.actions.WaitForHeadingAction;
import com.team1323.frc2018.auto.actions.WaitForWallAction;
import com.team1323.frc2018.auto.actions.WaitToFinishPathAction;
import com.team1323.frc2018.auto.actions.WaitToIntakeCubeAction;
import com.team1323.frc2018.auto.actions.WaitToPassXCoordinateAction;
import com.team1323.frc2018.subsystems.Intake;
import com.team1323.frc2018.subsystems.Intake.IntakeState;
import com.team1323.frc2018.subsystems.Superstructure;
import com.team1323.frc2018.subsystems.Swerve;
import com.team254.lib.geometry.Pose2dWithCurvature;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.trajectory.Trajectory;
import com.team254.lib.trajectory.timing.TimedState;

import edu.wpi.first.wpilibj.Timer;

public class LeftScaleAssistMode extends AutoModeBase{
	Superstructure s;
	Intake intake;
	
	private static List<Trajectory<TimedState<Pose2dWithCurvature>>> paths = Arrays.asList(trajectories.startToLeftScale, trajectories.backOffLeftScale);
	public static List<Trajectory<TimedState<Pose2dWithCurvature>>> getPaths(){
		return paths;
	}

	public LeftScaleAssistMode(){
		s = Superstructure.getInstance();
		intake = Intake.getInstance();
	}

	@Override
	protected void routine() throws AutoModeEndedException {
		double startTime = Timer.getFPGATimestamp();
		runAction(new ResetPoseAction(Constants.kRobotAssistStartingPose));
		s.request(intake.stateRequest(IntakeState.CLAMPING));
		runAction(new SetTrajectoryAction(trajectories.startToLeftScale, 50.0, 0.5));
		runAction(new WaitToPassXCoordinateAction(Constants.kLeftSwitchCloseCorner.x()));
		s.request(s.elevatorWristConfig(4.5, 66.0));
		runAction(new WaitToFinishPathAction());
		s.request(intake.ejectRequest(Constants.kIntakeStrongEjectOutput));
		System.out.println("First Cube Scored at: " + (Timer.getFPGATimestamp() - startTime));
		runAction(new WaitAction(0.4));
		runAction(new SetTrajectoryAction(trajectories.backOffLeftScale, 90.0, 0.5));
		runAction(new WaitForHeadingAction(80.0, 100.0));
		s.request(s.elevatorWristIntakeConfig(
			Constants.kElevatorIntakingHeight, 
			Constants.kWristPrimaryStowAngle,
			IntakeState.OFF));
		runAction(new WaitToFinishPathAction());
	}
	
}
