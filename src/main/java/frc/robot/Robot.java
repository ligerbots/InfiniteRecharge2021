/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package frc.robot;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.*;

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
  private TrajectoryPlotter m_plotter;

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

    // Set motors to coast so it's easier to move the robot.
    m_robotContainer.robotDrive.setIdleMode(IdleMode.kCoast);

    m_plotter = new TrajectoryPlotter(m_robotContainer.robotDrive.getField2d());

    //SmartDashboard.getEntry("tableUpdateRate").addListener((EntryNotification e)->NetworkTableInstance.getDefault().setUpdateRate(e.value.getDouble()), EntryListenerFlags.kUpdate|EntryListenerFlags.kNew);
    //SmartDashboard.putNumber("tableUpdateRate", 0.1); 
    
    chosenAuto.setDefaultOption("ShootandDrive", 
        new ShootAndDriveAuto(m_robotContainer.robotDrive, m_robotContainer.shooter,
            m_robotContainer.intake, m_robotContainer.carousel, m_robotContainer.driveCommand,
            m_robotContainer.carouselCommand));

    chosenAuto.addOption("MoveFowardAuto",  
      new MoveForwardAuto(m_robotContainer.robotDrive, m_robotContainer.driveCommand));

    //chosenAuto.addOption("DriveForward", (AutoCommandInterface) new DriveForwardAuto(m_robotContainer.robotDrive,
       // m_robotContainer.carouselCommand, m_robotContainer.driveCommand));

    // Disable for now, until it is fixed and tested
    // chosenAuto.addOption("EightBallAuto", 
    //     new EightBallAuto(m_robotContainer.robotDrive, m_robotContainer.shooter,
    //         m_robotContainer.intake, m_robotContainer.deployIntake, m_robotContainer.carousel,
    //         m_robotContainer.driveCommand, m_robotContainer.carouselCommand));

    SmartDashboard.putData("Chosen Auto", chosenAuto);
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

    // Maintain a SD value to know if the robot is enabled
    // Used for timing the 2021 At Home Skills
    SmartDashboard.putBoolean("Enabled", false);

    if (Robot.isReal()) {
      // m_robotContainer.climber.shoulder.setIdleMode(IdleMode.kCoast);
      // m_robotContainer.climber.winch.setIdleMode(IdleMode.kCoast);
      // Set motors to coast so it's easier to move the robot.
      m_robotContainer.robotDrive.setIdleMode(IdleMode.kCoast);
      // m_robotContainer.climber.coastWinch();
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

      if (Robot.isSimulation()) {
        m_plotter.clear();
        autoCommandInterface.plotTrajectory(m_plotter);
      }
    }
  }

  /**
   * This autonomous runs the autonomous command selected by your
   * {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    // Maintain a SD value to know if the robot is enabled
    SmartDashboard.putBoolean("Enabled", true);
    // For At Home Skills, we want to know when the auto starts, so flush for fast response.
    //NetworkTableInstance.getDefault().flush();

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
    // Maintain a SD value to know if the robot is enabled
    SmartDashboard.putBoolean("Enabled", true);

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

    //System.out.println("teleopInit");

    // Reset the winch encoder
    // m_robotContainer.climber.resetWinchEncoder();
    // m_robotContainer.climber.winch.setIdleMode(IdleMode.kCoast);

    m_robotContainer.driveCommand.schedule();
    m_robotContainer.carouselCommand.schedule();

    // Cancel the IntakeCommand so it only runs on the bumper buttons
    m_robotContainer.intakeCommand.cancel();
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void testInit() {
    // Maintain a SD value to know if the robot is enabled
    SmartDashboard.putBoolean("Enabled", false);

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
