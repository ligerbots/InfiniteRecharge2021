/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package frc.robot;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.EntryNotification;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.*;
import frc.robot.commands.InterstellarAccuracy.WaitForSmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  private AutoCommandInterface m_autonomousCommand;
  private RobotContainer m_robotContainer;
  SendableChooser<AutoCommandInterface> chosenAuto = new SendableChooser<>();

  private AutoCommandInterface m_prevAutoCommand = null;

  //returns the time since initialization in seconds as a double
  public static double time() {
    return System.nanoTime() * 1.0e-9;
  }

  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {
    /* Instantiate our RobotContainer. This will perform all our button bindings, 
    and put our autonomous chooser on the dashboard. */
    m_robotContainer = new RobotContainer();

    m_robotContainer.climber.shoulder.setIdleMode(IdleMode.kCoast);

    // Set motors to coast so it's easier to move the robot.
    m_robotContainer.robotDrive.setIdleMode(IdleMode.kCoast);

    //m_robotContainer.shooter.calibratePID(0.000085, 0.000000033, 0, 6.776 * 0.00001);

    SmartDashboard.getEntry("tableUpdateRate").addListener((EntryNotification e)->NetworkTableInstance.getDefault().setUpdateRate(e.value.getDouble()), EntryListenerFlags.kUpdate|EntryListenerFlags.kNew);
    SmartDashboard.putNumber("tableUpdateRate", 0.1); 
    /*creates a smartdashboard value for the time that it takes the network table to refresh its 
    values in seconds, which is 100 milliseconds or 0.1 seconds
    */ 

    // SmartDashboard.putData(new TestTurret(m_robotContainer.shooter));
    /*
    chosenAuto.addDefault("Default Auto", new DriveForwardAuto(m_robotContainer.robotDrive, m_robotContainer.carouselCommand, m_robotContainer.driveCommand));


    chosenAuto.addObject("EightBallAuto", new EightBallAuto(
      m_robotContainer.robotDrive,
      m_robotContainer.shooter,
      m_robotContainer.intake,
      m_robotContainer.climber,
      m_robotContainer.carousel,
      m_robotContainer.driveCommand,
      m_robotContainer.carouselCommand));

    chosenAuto.addObject("ShootAndDrive", new ShootAndDriveAuto(
        m_robotContainer.robotDrive,
        m_robotContainer.shooter,
        m_robotContainer.intake,
        m_robotContainer.climber,
        m_robotContainer.carousel,
        m_robotContainer.driveCommand,
        m_robotContainer.carouselCommand));
    */

    chosenAuto.setDefaultOption("None", null);
    chosenAuto.addOption("RedAAuto", new GalacticSearchAuto(m_robotContainer.robotDrive, m_robotContainer.driveCommand, m_robotContainer.carousel, m_robotContainer.intake, GalacticSearchAuto.Path.RedA));
    chosenAuto.addOption("RedBAuto", new GalacticSearchAuto(m_robotContainer.robotDrive, m_robotContainer.driveCommand, m_robotContainer.carousel, m_robotContainer.intake, GalacticSearchAuto.Path.RedB));
    chosenAuto.addOption("BlueAAuto", new GalacticSearchAuto(m_robotContainer.robotDrive, m_robotContainer.driveCommand, m_robotContainer.carousel, m_robotContainer.intake, GalacticSearchAuto.Path.BlueA));
    chosenAuto.addOption("BlueBAuto", new GalacticSearchAuto(m_robotContainer.robotDrive, m_robotContainer.driveCommand, m_robotContainer.carousel, m_robotContainer.intake, GalacticSearchAuto.Path.BlueB));
    chosenAuto.addOption("Barrel", new AutoNavPaths(m_robotContainer.robotDrive, m_robotContainer.driveCommand, AutoNavPaths.Path.Barrel));
    chosenAuto.addOption("Slalom", new AutoNavPaths(m_robotContainer.robotDrive, m_robotContainer.driveCommand, AutoNavPaths.Path.Slalom));
    chosenAuto.addOption("Bounce", new BounceAuto(m_robotContainer.robotDrive, m_robotContainer.driveCommand));
    chosenAuto.addOption("VisionPath", new GalacticSearchAutoSelector(m_robotContainer.robotDrive, m_robotContainer.driveCommand, m_robotContainer.carousel, m_robotContainer.intake, m_robotContainer.vision));
    chosenAuto.addOption("InterstellarAccuracy", new InterstellarAccuracy(m_robotContainer.robotDrive, m_robotContainer.driveCommand,
        m_robotContainer.shooter, m_robotContainer.carousel, m_robotContainer.carouselCommand));

        SmartDashboard.putData("Chosen Auto", chosenAuto);
    WaitForSmartDashboard.initSmartDashboard();
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for
   * items like diagnostics that you want ran during disabled, autonomous,
   * teleoperated and test.
   *
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    /* Runs the Scheduler. This is responsible for polling buttons, adding newly-scheduled commands, 
    running already-scheduled commands, removing finished or interrupted commands, 
    and running subsystem periodic() methods. 
    This must be called from the
    robot's periodic block in order for anything in the Command-based framework to work.
    */ 
    CommandScheduler.getInstance().run();
  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   */
  @Override
  public void disabledInit() {
    if (RobotBase.isSimulation()) {
      m_robotContainer.robotDrive.setRobotFromFieldPose();
    }

    if (Robot.isReal()) {
      m_robotContainer.climber.shoulder.setIdleMode(IdleMode.kCoast);
      m_robotContainer.climber.winch.setIdleMode(IdleMode.kCoast);
      // Set motors to coast so it's easier to move the robot.
      m_robotContainer.robotDrive.setIdleMode(IdleMode.kCoast);
      m_robotContainer.climber.coastWinch();
    }
  }

  @Override
  public void disabledPeriodic() {
    //m_robotContainer.carouselCommand.schedule();

    /* Do not use the member variable m_autonomousCommand. Setting that signals
    that the command is running, which it is not, yet. */
    AutoCommandInterface autoCommandInterface = chosenAuto.getSelected();
    if (autoCommandInterface != null && autoCommandInterface != m_prevAutoCommand) {
      m_robotContainer.robotDrive.setPose(autoCommandInterface.getInitialPose());
      m_prevAutoCommand = autoCommandInterface;
    }
  }

  /**
   * This autonomous runs the autonomous command selected by your
   * {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    if (RobotBase.isSimulation()) {
      m_robotContainer.robotDrive.setRobotFromFieldPose();
    }

    m_robotContainer.carousel.resetEncoder();
    // Set motors to brake for the drive train
    m_robotContainer.robotDrive.setIdleMode(IdleMode.kBrake);

    m_robotContainer.carouselCommand.schedule();

    // Cancel the DriveCommand so that the joystick can't override this command
    // group
    m_robotContainer.driveCommand.cancel();

    // schedule the autonomous command
    m_autonomousCommand = chosenAuto.getSelected();
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    if (RobotBase.isSimulation()) {
      m_robotContainer.robotDrive.setRobotFromFieldPose();
    }

    /* This makes sure that the autonomous stops running when
    teleop starts running. If you want the autonomous to
    continue until interrupted by another command, remove
    this line or comment it out.
    Do this immediately before changing any motor settings, etc. 
    */

    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
      m_autonomousCommand = null;
    }

    // Set motors to brake for the drive train
    m_robotContainer.robotDrive.setIdleMode(IdleMode.kBrake);

    System.out.println("teleopInit");

    // Reset the winch encoder
    m_robotContainer.climber.resetWinchEncoder();
    m_robotContainer.climber.winch.setIdleMode(IdleMode.kCoast);

    //SmartDashboard.putData(m_robotContainer.testFlup);

    m_robotContainer.driveCommand.schedule();
    //m_robotContainer.testFlup.schedule();
    //m_robotContainer.shooter.testSpin();
    m_robotContainer.carouselCommand.schedule();

    // Cancel the IntakeCommand so it only runs on the bumper buttons
    m_robotContainer.intakeCommand.cancel();
    //m_robotContainer.testFlup.schedule();
    //m_robotContainer.testIntake.schedule();
    //RunWinch aaa = new RunWinch(m_robotContainer.climber, m_robotContainer);
    //aaa.schedule();
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
    if (RobotBase.isSimulation()) {
      m_robotContainer.robotDrive.moveAroundField();
    }
  }

  @Override
  public void simulationInit() {
  }

  @Override
  public void simulationPeriodic() {
  }
}
