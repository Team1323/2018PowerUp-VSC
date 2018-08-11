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
import com.team1323.frc2018.loops.LimelightProcessor;
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
import com.team254.lib.trajectory.TimedView;
import com.team254.lib.trajectory.TrajectoryGenerator;
import com.team254.lib.trajectory.TrajectoryIterator;

import edu.wpi.cscore.MjpegServer;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.VideoMode.PixelFormat;
import edu.wpi.first.wpilibj.CameraServer;
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
	
	private Looper swerveLooper = new Looper();
	private Looper enabledLooper = new Looper();
	private Looper disabledLooper = new Looper();
	
	private RobotState robotState = RobotState.getInstance();
	private LimelightProcessor limelight = LimelightProcessor.getInstance();
	private CameraServer cam;
	
	private Xbox driver, coDriver;
	

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
						Wrist.getInstance(), Superstructure.getInstance()));
		
		driver = new Xbox(0);
		coDriver = new Xbox(1);
		driver.setDeadband(0.0);
		coDriver.setDeadband(0.4);
		
		Logger.clearLog();
		
		swerve.registerEnabledLoops(swerveLooper);
		subsystems.registerEnabledLoops(enabledLooper);
		enabledLooper.register(LimelightProcessor.getInstance());
		enabledLooper.register(RobotStateEstimator.getInstance());
		enabledLooper.register(PathTransmitter.getInstance());
		enabledLooper.register(QuinticPathTransmitter.getInstance());
		disabledLooper.register(LimelightProcessor.getInstance());
		disabledLooper.register(RobotStateEstimator.getInstance());
		disabledLooper.register(PathTransmitter.getInstance());
		disabledLooper.register(QuinticPathTransmitter.getInstance());
		
		subsystems.zeroSensors();
		swerve.zeroSensors();
		
		smartDashboardInteractions.initWithDefaults();
		initCamera();
		
		generator.generateTrajectories();
		
		//PathManager.buildAllPaths();
		
		/*transmitter.addPaths(Arrays.asList(PathManager.mLeftSwitchDropoff, PathManager.mLeftmostCubePickup,
				PathManager.mLeftCubeToLeftScale, PathManager.mLeftScaleToSecondCube, PathManager.mSecondLeftCubeToScale,
				PathManager.mLeftScaleToThirdCube));*/
		/*transmitter.addPaths(Arrays.asList(PathManager.mRightSwitchDropoff, PathManager.mRightmostCubePickup,
				PathManager.mRightCubeToRightScale, PathManager.mRightScaleToSecondCube, PathManager.mSecondRightCubeToScale));*/
		/*transmitter.addPaths(Arrays.asList(PathManager.mRightSwitchDropoff, PathManager.mRightmostCubePickup,
				PathManager.mRightCubeToLeftScale, PathManager.mLeftScaleToFirstCube));*/
		/*transmitter.addPaths(Arrays.asList(PathManager.mLeftSwitchDropoff, PathManager.mLeftmostCubePickup,
				PathManager.mLeftCubeToRightScale, PathManager.mRightScaleToFirstCube));*/
		/*transmitter.addPaths(Arrays.asList(PathManager.mFrontLeftSwitch, PathManager.mFrontLeftSwitchToOuterCube, 
				PathManager.mOuterCubeToFrontLeftSwitch, PathManager.mFrontLeftSwitchToMiddleCube, PathManager.mMiddleCubeToFrontLeftSwitch,
				PathManager.mFrontLeftSwitchToBottomMiddle, PathManager.mFrontLeftSwitchToDropoff));*/
		/*transmitter.addPaths(Arrays.asList(PathManager.mFrontRightSwitch, PathManager.mFrontRightSwitchToOuterCube, 
				PathManager.mOuterCubeToFrontRightSwitch, PathManager.mFrontRightSwitchToMiddleCube, PathManager.mMiddleCubeToFrontRightSwitch,
				PathManager.mFrontRightSwitchToBottomMiddle, PathManager.mFrontRightSwitchToDropoff));*/
		/*transmitter.addPaths(Arrays.asList(PathManager.mStartToLeftScale, PathManager.mAlternateLeftmostCube,
				PathManager.mDerpLeftCubeToLeftScale, PathManager.mAlternateLeftScaleToSecondCube,
				PathManager.mAlternateSecondLeftCubeToScale));*/
		/*transmitter.addPaths(Arrays.asList(PathManager.mStartToRightScale, PathManager.mRightScaleToFirstCube,
				PathManager.mAlternateRightCubeToRightScale, PathManager.mAlternateRightScaleToSecondCube));*/
		
		//qTransmitter.addPaths(Arrays.asList(PathManager.mStartToRightScale));
		
		/*PathfinderPath path = PathManager.mStartToRightScale;
		double maxSpeed = 0.0;
		int points = 0;
		
		for (int i = 0; i < path.getTrajectory().length(); i++) {
		    Trajectory.Segment seg = path.getTrajectory().get(i);
		    String coordinates = "(" + Double.toString(seg.y) + ", " + Double.toString(seg.x) + ")";
		    if(i != (path.getTrajectory().length() - 1))
		    	coordinates += ", ";
		    Logger.log(coordinates);
		    maxSpeed = (seg.velocity > maxSpeed) ? seg.velocity : maxSpeed;
		    points++;
		}
		System.out.println("Max Path Velocity: " + maxSpeed + ", Number of Points: " + points);*/
		qTransmitter.addPath(generator.getTrajectorySet().sideStartToFarScale.get(false));
	}
	
	public void allPeriodic(){
		subsystems.outputToSmartDashboard();
		swerve.outputToSmartDashboard();
		robotState.outputToSmartDashboard();
		//SmartDashboard.putNumber("Swerve dt", swerveLooper.dt_);
		//enabledLooper.outputToSmartDashboard();
		
	}
	
	public void initCamera(){
		cam = CameraServer.getInstance();
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
			
			subsystems.zeroSensors();
			swerve.zeroSensors();
			swerve.setNominalDriveOutput(1.5);
			swerve.requireModuleConfiguration();
			transmitter.transmitCachedPaths();
			
			disabledLooper.stop();
			swerveLooper.start();
			enabledLooper.start();
			
			limelight.setVisionMode();
			limelight.ledOn(true);
			
			superstructure.elevator.setCurrentLimit(20);
			superstructure.elevator.configForAutoSpeed();
			
			superstructure.intake.setHoldingOutput(Constants.kIntakeWeakHoldingOutput);
			
			superstructure.enableCompressor(false);
			
			//SmartDashboard.putBoolean("Auto", true);
			
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
			swerveLooper.stop();
			swerveLooper.start();
			enabledLooper.start();
			superstructure.enableCompressor(true);
			limelight.setDriverMode();
			limelight.ledOn(false);
			swerve.setNominalDriveOutput(0.0);
			superstructure.elevator.setCurrentLimit(30);
			superstructure.elevator.configForTeleopSpeed();
			superstructure.intake.setHoldingOutput(Constants.kIntakeStrongHoldingOutput);
			//limelight.setVisionMode();
			//limelight.ledOn(true);
			//SmartDashboard.putBoolean("Auto", false);
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
			
			if(coDriver.backButton.isBeingPressed()){
				superstructure.request(intake.stateRequest(IntakeState.OFF));
			}
			
			double swerveYInput = driver.getX(Hand.kLeft);
			double swerveXInput = -driver.getY(Hand.kLeft);
			double swerveRotationInput = driver.getX(Hand.kRight);
			
			swerve.sendInput(swerveXInput, swerveYInput, swerveRotationInput, driver.rightCenterClick.isBeingPressed(), driver.leftTrigger.isBeingPressed());
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
			}else if(driver.backButton.longPressed()){
				swerve.temporarilyDisableHeadingController();
				swerve.zeroSensors(Constants.kRobotStartingPose);
			}else if(driver.rightTrigger.wasPressed()){
				/*swerve.setVectorField(
						(new ConstantVectorField(new Translation2d(1,0)))
						.add(new GuidingVectorField(new PolyCone(Arrays.asList(
							Constants.kLeftSwitchCloseCorner,
							Constants.kLeftSwitchFarCorner,
							Constants.kRightSwitchFarCorner,
							Constants.kRightSwitchCloseCorner
						)), here -> 1.0)) // k = 0 to make all followey no approachey
					);*/
				//swerve.toggleEvade();
				swerve.temporarilyDisableHeadingController();
				swerve.zeroSensors(Constants.kRobotLeftStartingPose);
				swerve.setTrajectory(new TrajectoryIterator<>(new TimedView<>(generator.getTrajectorySet().sideStartToFarScale.get(false))), 0.0, 2.0);
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
				superstructure.request(elevator.lowGearHeightRequest(Constants.kElevatorMinimumHangingHeight));
			}else if(driver.POV90.wasPressed() && !elevator.isHighGear()){
				superstructure.flipDriveTrain();
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
			swerveLooper.stop();
			swerveLooper.start();
			disabledLooper.start();
			elevator.fireGasStruts(false);
			elevator.fireLatch(false);
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
	}
}
