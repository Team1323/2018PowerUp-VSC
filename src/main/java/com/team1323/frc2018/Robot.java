/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.team1323.frc2018;

import java.util.Arrays;

import com.team1323.frc2018.auto.AutoModeExecuter;
import com.team1323.frc2018.auto.SmartDashboardInteractions;
import com.team1323.frc2018.auto.modes.LLAssistMode;
import com.team1323.frc2018.loops.Looper;
import com.team1323.frc2018.loops.PathTransmitter;
import com.team1323.frc2018.loops.QuinticPathTransmitter;
import com.team1323.frc2018.loops.RobotStateEstimator;
import com.team1323.frc2018.subsystems.Elevator;
import com.team1323.frc2018.subsystems.Intake;
import com.team1323.frc2018.subsystems.Intake.IntakeState;
import com.team1323.frc2018.subsystems.RequestList;
import com.team1323.frc2018.subsystems.SubsystemManager;
import com.team1323.frc2018.subsystems.Superstructure;
import com.team1323.frc2018.subsystems.Swerve;
import com.team1323.frc2018.subsystems.Wrist;
import com.team1323.io.Xbox;
import com.team1323.lib.util.CrashTracker;
import com.team1323.lib.util.Logger;
import com.team1323.lib.util.Util;
import com.team254.lib.geometry.Translation2d;
import com.team254.lib.trajectory.TimedView;
import com.team254.lib.trajectory.TrajectoryGenerator;
import com.team254.lib.trajectory.TrajectoryIterator;

import edu.wpi.cscore.MjpegServer;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.VideoMode.PixelFormat;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.properties file in the
 * project.
 */
public class Robot extends IterativeRobot {
	private Swerve swerve;
	private Superstructure superstructure;
	private SubsystemManager subsystems;
	private Intake intake;
	private Elevator elevator;
	private Wrist wrist;
	
	private AutoModeExecuter autoModeExecuter = null;
	private TrajectoryGenerator generator = TrajectoryGenerator.getInstance();
	private PathTransmitter transmitter = PathTransmitter.getInstance();
	private QuinticPathTransmitter qTransmitter = QuinticPathTransmitter.getInstance();
	private SmartDashboardInteractions smartDashboardInteractions = new SmartDashboardInteractions();

	private Looper enabledLooper = new Looper();
	private Looper disabledLooper = new Looper();
	
	private RobotState robotState = RobotState.getInstance();
	
	private Xbox driver, coDriver;
	private final boolean oneControllerMode = false;
	private boolean flickRotation = false;

	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
		swerve = Swerve.getInstance();
		superstructure = Superstructure.getInstance();
		intake = Intake.getInstance();
		wrist = Wrist.getInstance();
		elevator = Elevator.getInstance();
		subsystems = new SubsystemManager(
				Arrays.asList(Intake.getInstance(), Elevator.getInstance(),
						Wrist.getInstance(), Superstructure.getInstance(),
						Swerve.getInstance()));
		
		driver = new Xbox(0);
		coDriver = new Xbox(1);
		driver.setDeadband(0.0);
		coDriver.setDeadband(0.4);
		
		Logger.clearLog();

		subsystems.registerEnabledLoops(enabledLooper);
		subsystems.registerDisabledLoops(disabledLooper);
		enabledLooper.register(RobotStateEstimator.getInstance());
		enabledLooper.register(PathTransmitter.getInstance());
		enabledLooper.register(QuinticPathTransmitter.getInstance());
		disabledLooper.register(RobotStateEstimator.getInstance());
		disabledLooper.register(PathTransmitter.getInstance());
		disabledLooper.register(QuinticPathTransmitter.getInstance());

		swerve.zeroSensors();
		
		smartDashboardInteractions.initWithDefaults();
		initCamera();
		
		generator.generateTrajectories();

		qTransmitter.addPaths(smartDashboardInteractions.getSelectedAutoMode(DriverStation.getInstance().getGameSpecificMessage().substring(0, 2)).getPaths());
	}
	
	public void allPeriodic(){
		subsystems.outputToSmartDashboard();
		robotState.outputToSmartDashboard();
		enabledLooper.outputToSmartDashboard();
	}

	public void autoConfig(){
		swerve.zeroSensors();
		swerve.setNominalDriveOutput(1.5);
		swerve.requireModuleConfiguration();
		transmitter.transmitCachedPaths();

		superstructure.elevator.setCurrentLimit(15);
		superstructure.elevator.configForAutoSpeed();
		
		superstructure.intake.setHoldingOutput(Constants.kIntakeWeakHoldingOutput);
		superstructure.intake.setCurrentLimit(30);
		
		superstructure.enableCompressor(false);
	}

	public void teleopConfig(){
		superstructure.enableCompressor(true);
		swerve.setNominalDriveOutput(0.0);
		superstructure.elevator.setCurrentLimit(15);
		superstructure.elevator.configForTeleopSpeed();
		superstructure.intake.setHoldingOutput(Constants.kIntakeStrongHoldingOutput);
		superstructure.intake.setCurrentLimit(30);
	}
	
	public void initCamera(){
    	UsbCamera usbCamera = new UsbCamera("USB Camera 0", 0);
    	usbCamera.setVideoMode(PixelFormat.kMJPEG, 320, 240, 30);
    	MjpegServer mjpegServer2 = new MjpegServer("serve_Blur", 1182);
    	mjpegServer2.setSource(usbCamera);
	}
	
	@Override
	public void autonomousInit() {
		try{
			if(autoModeExecuter != null)
				autoModeExecuter.stop();

			autoConfig();
			
			disabledLooper.stop();
			enabledLooper.start();
			
			SmartDashboard.putBoolean("Auto", true);
			
			String gameData = DriverStation.getInstance().getGameSpecificMessage();
			autoModeExecuter = new AutoModeExecuter();
			autoModeExecuter.setAutoMode(smartDashboardInteractions.getSelectedAutoMode(gameData.substring(0, 2)));
			autoModeExecuter.start();
			SmartDashboard.putString("Game Data", gameData);
			System.out.println(gameData);
		}catch(Throwable t){
			CrashTracker.logThrowableCrash(t);
			throw t;
		}
	}

	/**
	 * This function is called periodically during autonomous.
	 */
	@Override
	public void autonomousPeriodic() {
		allPeriodic();
	}
	
	@Override
	public void teleopInit(){
		try{
			disabledLooper.stop();
			enabledLooper.start();
			teleopConfig();
			SmartDashboard.putBoolean("Auto", false);
		}catch(Throwable t){
			CrashTracker.logThrowableCrash(t);
			throw t;
		}
	}

	/**
	 * This function is called periodically during operator control.
	 */
	@Override
	public void teleopPeriodic() {
		try{
			driver.update();
			coDriver.update();
			
			if(oneControllerMode) oneControllerMode();
			else twoControllerMode();
			
			allPeriodic();
		}catch(Throwable t){
			CrashTracker.logThrowableCrash(t);
			throw t;
		}
	}
	
	@Override
	public void disabledInit(){
		try{
			if(autoModeExecuter != null)
				autoModeExecuter.stop();
			enabledLooper.stop();
			subsystems.stop();
			disabledLooper.start();
			elevator.fireLatch(false);
			elevator.fireForks(false);
		}catch(Throwable t){
			CrashTracker.logThrowableCrash(t);
			throw t;
		}
	}
	
	@Override
	public void disabledPeriodic(){
		try{
			allPeriodic();
			smartDashboardInteractions.output();
		}catch(Throwable t){
			CrashTracker.logThrowableCrash(t);
			throw t;
		}
	}
	
	@Override
	public void testInit(){
		Timer.delay(2.0);
		boolean passed = true;
		//passed &= Intake.getInstance().checkSystem();
		//passed &= Wrist.getInstance().checkSystem();
		passed &= Elevator.getInstance().checkSystem();
		if(passed)
			System.out.println("All systems passed");
		else
			System.out.println("Some systems failed, check above output for details");
	}
	
	@Override
	public void testPeriodic() {
		allPeriodic();
	}

	private void twoControllerMode(){
		if(coDriver.backButton.isBeingPressed()){
			superstructure.request(intake.stateRequest(IntakeState.OFF));
		}
		
		double swerveYInput = driver.getX(Hand.kLeft);
		double swerveXInput = -driver.getY(Hand.kLeft);
		double swerveRotationInput = (flickRotation ? 0.0 : driver.getX(Hand.kRight));
		
		swerve.sendInput(swerveXInput, swerveYInput, swerveRotationInput, false, driver.leftTrigger.isBeingPressed());

		if(driver.rightCenterClick.wasPressed()){
			if(flickRotation){
				driver.rumble(3, 1);
			}else{
				driver.rumble(1, 1);
			}
			flickRotation = !flickRotation;
		}

		if(flickRotation){
			swerve.updateControllerDirection(new Translation2d(-driver.getY(Hand.kRight), driver.getX(Hand.kRight)));
			if(!Util.epsilonEquals(Util.placeInAppropriate0To360Scope(swerve.getTargetHeading(), swerve.averagedDirection.getDegrees()), swerve.getTargetHeading(), swerve.rotationDivision / 2.0)){
				swerve.rotate(swerve.averagedDirection.getDegrees());
			}
		}

		if(driver.yButton.isBeingPressed())
			swerve.rotate(0);
		else if(driver.bButton.isBeingPressed())
			swerve.rotate(90);
		else if(driver.aButton.isBeingPressed())
			swerve.rotate(180);
		else if(driver.xButton.isBeingPressed())
			swerve.rotate(270);
		else if(driver.leftCenterClick.isBeingPressed())
			swerve.rotate(-135);
		else if(driver.rightBumper.isBeingPressed())
			swerve.rotate(25);
		if(driver.backButton.wasPressed()){
			swerve.temporarilyDisableHeadingController();
			swerve.zeroSensors(Constants.kRobotStartingPose);
			swerve.resetAveragedDirection();
		}else if(driver.backButton.longPressed()){
			swerve.temporarilyDisableHeadingController();
			swerve.zeroSensors(Constants.kRobotStartingPose);
			swerve.resetAveragedDirection();
		}else if(driver.rightTrigger.wasPressed()){
			swerve.temporarilyDisableHeadingController();
			swerve.zeroSensors(Constants.kRobotLeftStartingPose);
			swerve.setTrajectory(new TrajectoryIterator<>(new TimedView<>(generator.getTrajectorySet().startToRightScale)), -90.0, 0.75);
		}
					
		if(superstructure.driveTrainFlipped() && coDriver.leftTrigger.isBeingPressed())
			superstructure.sendManualInput(-coDriver.getY(Hand.kRight), -coDriver.getY(Hand.kLeft)*0.5);
		else
			superstructure.sendManualInput(-coDriver.getY(Hand.kRight), -coDriver.getY(Hand.kLeft));
		
		if(!superstructure.driveTrainFlipped()){
			if(coDriver.aButton.wasPressed()){
				superstructure.request(superstructure.elevatorWristConfig(Constants.kElevatorIntakingHeight, 
						Constants.kWristIntakingAngle));
				superstructure.replaceQueue(Arrays.asList(new RequestList(intake.waitForCubeRequest()),
						new RequestList(intake.stateRequest(IntakeState.CLAMPING)),
						new RequestList(wrist.angleRequest(Constants.kWristPrimaryStowAngle))));
			}else if(coDriver.aButton.longPressed()){
				if(intake.getState() == IntakeState.INTAKING)
					superstructure.replaceQueue(superstructure.elevatorWristIntakeConfig(0.31, 13.0, IntakeState.CLAMPING));
				else
					superstructure.request(superstructure.elevatorWristIntakeConfig(0.31, 13.0, IntakeState.CLAMPING));
			}else if(coDriver.xButton.wasPressed()){
				superstructure.request(superstructure.elevatorWristIntakeConfig(Constants.kElevatorSwitchHeight, 
						20.0, IntakeState.CLAMPING));
			}else if(coDriver.xButton.longPressed()){
				superstructure.request(superstructure.elevatorWristIntakeConfig(Constants.kElevatorHumanLoadHeight, 
						Constants.kWristIntakingAngle, IntakeState.OPEN));
			}else if(coDriver.bButton.wasPressed()){
				superstructure.request(superstructure.wristIntakeConfig(Constants.kWristPrimaryStowAngle, IntakeState.CLAMPING));
			}else if(coDriver.bButton.longPressed()){
				superstructure.request(superstructure.elevatorWristIntakeConfig(Constants.kElevatorIntakingHeight, 
						Constants.kWristPrimaryStowAngle, IntakeState.OFF));
			}else if(coDriver.yButton.wasPressed()){
				superstructure.request(superstructure.elevatorWristIntakeConfig(Constants.kELevatorBalancedScaleHeight, 
						20.0, IntakeState.CLAMPING));
			}else if(coDriver.POV0.wasPressed()){
				superstructure.request(superstructure.elevatorWristIntakeConfig(Constants.kElevatorHighScaleHeight, 
						60.0, IntakeState.CLAMPING));
			}else if(coDriver.POV180.wasPressed()){
				superstructure.request(superstructure.elevatorWristIntakeConfig(Constants.kElevatorLowScaleHeight, 
						25.0, IntakeState.CLAMPING));
			}else if(coDriver.POV90.wasPressed()){
				superstructure.request(superstructure.elevatorWristIntakeConfig(Constants.kElevatorTippingCubeHeight,
						Constants.kWristIntakingAngle, IntakeState.OFF));
			}else if(coDriver.rightCenterClick.wasPressed()){
				superstructure.request(superstructure.elevatorWristConfig(Constants.kElevatorSecondCubeHeight, 
						Constants.kWristIntakingAngle),
						new RequestList(intake.stateRequest(IntakeState.INTAKING)));
			}else if(coDriver.leftBumper.wasPressed()){
				superstructure.request(superstructure.elevatorWristIntakeConfig(
						Constants.kElevatorIntakingHeight, 
						Constants.kWristIntakingAngle, 
						IntakeState.OFF),
						new RequestList(intake.stateRequest(IntakeState.INTAKING_WIDE)));
			}
		}else{
			if(coDriver.POV0.isBeingPressed()){
				superstructure.requestWinchOpenLoop(0.75);
			}else if(coDriver.POV180.isBeingPressed()){
				superstructure.requestWinchOpenLoop(-0.75);
			}else{
				superstructure.requestWinchOpenLoop(0.0);
			}
		}
		
		if(coDriver.rightBumper.isBeingPressed()){
			superstructure.addForemostActiveRequest(intake.stateRequest(IntakeState.FORCED_INTAKE));
		}else if(intake.getState() == IntakeState.FORCED_INTAKE){
			superstructure.addForemostActiveRequest(intake.stateRequest(IntakeState.OFF));
		}else if(coDriver.leftTrigger.wasPressed() || driver.leftBumper.wasPressed()){
			superstructure.addForemostActiveRequest(intake.stateRequest(IntakeState.OPEN));
		}else if(coDriver.rightTrigger.wasPressed() || driver.rightTrigger.wasPressed()){
			superstructure.request(intake.ejectRequest(Constants.kIntakeEjectOutput));
		}else if(coDriver.rightTrigger.longPressed() || driver.rightTrigger.longPressed()){
			superstructure.addForemostActiveRequest(intake.ejectRequest(Constants.kIntakeWeakEjectOutput));
		}
		
		if(driver.POV0.wasPressed()){
			superstructure.request(superstructure.elevatorWristIntakeConfig(Constants.kELevatorHangingHeight, 
					Constants.kWristPrimaryStowAngle, IntakeState.OPEN),
					new RequestList(elevator.gearShiftRequest(false)));
		}else if(driver.POV180.wasPressed() && !elevator.isHighGear()){
			//superstructure.request(elevator.lowGearHeightRequest(Constants.kElevatorMinimumHangingHeight));
		}else if(driver.POV90.wasPressed() && !elevator.isHighGear()){
			//flip is disabled for now, due to our adoption of forks for CC
			//superstructure.flipDriveTrain();
			elevator.toggleForks();
		}
		
		if(intake.needsToNotifyDrivers()){
			driver.rumble(1.0, 1.0);
			coDriver.rumble(1.0, 1.0);
		}
		
		if(coDriver.startButton.longPressed()){
			elevator.setManualSpeed(0.25);
			superstructure.elevator.enableLimits(false);
		}else if(!superstructure.elevator.limitsEnabled() && coDriver.getY(Hand.kLeft) == 0){
			superstructure.elevator.zeroSensors();
			superstructure.elevator.enableLimits(true);
			elevator.setManualSpeed(Constants.kElevatorTeleopManualSpeed);
		}
	}

	private void oneControllerMode(){
		if(driver.backButton.wasPressed() || coDriver.backButton.isBeingPressed()){
			superstructure.request(intake.stateRequest(IntakeState.OFF));
		}
		
		double swerveYInput = driver.getX(Hand.kLeft);
		double swerveXInput = -driver.getY(Hand.kLeft);
		double swerveRotationInput = (flickRotation ? 0.0 : driver.getX(Hand.kRight));
		
		swerve.sendInput(swerveXInput, swerveYInput, swerveRotationInput, driver.rightCenterClick.isBeingPressed(), driver.leftTrigger.isBeingPressed());

		if(flickRotation){
			swerve.updateControllerDirection(new Translation2d(-driver.getY(Hand.kRight), driver.getX(Hand.kRight)));
			if(!Util.epsilonEquals(Util.placeInAppropriate0To360Scope(swerve.getTargetHeading(), swerve.averagedDirection.getDegrees()), swerve.getTargetHeading(), swerve.rotationDivision / 2.0)){
				swerve.rotate(swerve.averagedDirection.getDegrees());
			}
		}

		if(driver.backButton.longPressed()){
			swerve.temporarilyDisableHeadingController();
			swerve.zeroSensors(Constants.kRobotStartingPose);
			swerve.resetAveragedDirection();
		}
		
		if(!superstructure.driveTrainFlipped()){
			if(driver.aButton.wasPressed() || coDriver.aButton.wasPressed()){
				superstructure.request(superstructure.elevatorWristConfig(Constants.kElevatorIntakingHeight, 
						Constants.kWristIntakingAngle));
				superstructure.replaceQueue(Arrays.asList(new RequestList(intake.waitForCubeRequest()),
						new RequestList(intake.stateRequest(IntakeState.CLAMPING)),
						new RequestList(wrist.angleRequest(Constants.kWristPrimaryStowAngle))));
			}else if(driver.xButton.wasPressed() || coDriver.xButton.wasPressed()){
				superstructure.request(superstructure.elevatorWristIntakeConfig(Constants.kElevatorSwitchHeight, 
						20.0, IntakeState.CLAMPING));
			}else if(driver.bButton.wasPressed() || coDriver.bButton.wasPressed()){
				superstructure.request(superstructure.wristIntakeConfig(Constants.kWristPrimaryStowAngle, IntakeState.CLAMPING));
			}else if(driver.bButton.longPressed() || coDriver.bButton.longPressed()){
				superstructure.request(superstructure.elevatorWristIntakeConfig(Constants.kElevatorIntakingHeight, 
						Constants.kWristPrimaryStowAngle, IntakeState.OFF));
			}else if(driver.yButton.wasPressed() || coDriver.yButton.wasPressed()){
				superstructure.request(superstructure.elevatorWristIntakeConfig(Constants.kELevatorBalancedScaleHeight, 
						20.0, IntakeState.CLAMPING));
			}else if(driver.POV0.wasPressed() || coDriver.POV0.wasPressed()){
				superstructure.request(superstructure.elevatorWristIntakeConfig(Constants.kElevatorHighScaleHeight, 
						60.0, IntakeState.CLAMPING));
			}else if(driver.POV180.wasPressed() || coDriver.POV180.wasPressed()){
				superstructure.request(superstructure.elevatorWristIntakeConfig(Constants.kElevatorLowScaleHeight, 
						25.0, IntakeState.CLAMPING));
			}else if(driver.POV90.wasPressed() || coDriver.POV90.wasPressed()){
				superstructure.request(superstructure.elevatorWristIntakeConfig(Constants.kElevatorTippingCubeHeight,
						Constants.kWristIntakingAngle, IntakeState.OFF));
			}else if(driver.rightCenterClick.wasPressed() || coDriver.rightCenterClick.wasPressed()){
				superstructure.request(superstructure.elevatorWristConfig(Constants.kElevatorSecondCubeHeight, 
						Constants.kWristIntakingAngle),
						new RequestList(intake.stateRequest(IntakeState.INTAKING)));
			}else if(driver.leftBumper.wasPressed() || coDriver.leftBumper.wasPressed()){
				superstructure.request(superstructure.elevatorWristIntakeConfig(
						Constants.kElevatorIntakingHeight, 
						Constants.kWristIntakingAngle, 
						IntakeState.OFF),
						new RequestList(intake.stateRequest(IntakeState.INTAKING_WIDE)));
			}
		}

		if(superstructure.driveTrainFlipped() && coDriver.leftTrigger.isBeingPressed())
			superstructure.sendManualInput(-coDriver.getY(Hand.kRight), -coDriver.getY(Hand.kLeft)*0.5);
		else
			superstructure.sendManualInput(-coDriver.getY(Hand.kRight), -coDriver.getY(Hand.kLeft));
		
		if(driver.rightBumper.isBeingPressed() || coDriver.rightBumper.isBeingPressed()){
			superstructure.addForemostActiveRequest(intake.stateRequest(IntakeState.FORCED_INTAKE));
		}else if(intake.getState() == IntakeState.FORCED_INTAKE){
			superstructure.addForemostActiveRequest(intake.stateRequest(IntakeState.OFF));
		}else if(driver.leftTrigger.wasPressed() || coDriver.leftTrigger.wasPressed()){
			superstructure.addForemostActiveRequest(intake.stateRequest(IntakeState.OPEN));
		}else if(driver.rightTrigger.wasPressed() || coDriver.rightTrigger.wasPressed()){
			superstructure.request(intake.ejectRequest(Constants.kIntakeEjectOutput));
		}else if(driver.rightTrigger.longPressed() || coDriver.rightTrigger.longPressed()){
			superstructure.addForemostActiveRequest(intake.ejectRequest(Constants.kIntakeWeakEjectOutput));
		}
		
		if(intake.needsToNotifyDrivers()){
			driver.rumble(1.0, 1.0);
			coDriver.rumble(1.0, 1.0);
		}

		if(coDriver.startButton.longPressed()){
			elevator.setManualSpeed(0.25);
			superstructure.elevator.enableLimits(false);
		}else if(!superstructure.elevator.limitsEnabled() && coDriver.getY(Hand.kLeft) == 0){
			superstructure.elevator.zeroSensors();
			superstructure.elevator.enableLimits(true);
			elevator.setManualSpeed(Constants.kElevatorTeleopManualSpeed);
		}
	}
}
