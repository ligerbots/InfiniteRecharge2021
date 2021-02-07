/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.commands.*;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Carousel;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision;

import java.util.function.DoubleSupplier;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  //private final DriveTrain robotDrive = new DriveTrain();
  private final Throttle throttle = new Throttle();
  private final Turn turn = new Turn();
  public final DriveTrain robotDrive = new DriveTrain();

  XboxController xbox = new XboxController(0);
  Joystick farm = new Joystick(1);

  public final Vision vision = new Vision(robotDrive);
  public final Intake intake = new Intake();
  public final Carousel carousel = new Carousel();
  public final Shooter shooter = new Shooter(vision);

  // Not needed for At Home. Silence the warning for now
  // private final Shoulder shoulder = new Shoulder();
  public final Climber climber = new Climber(robotDrive);
  public final DriveCommand driveCommand = new DriveCommand(robotDrive, throttle, turn);
  public final PositionRecorder positionRecorder = new PositionRecorder(robotDrive);
  public final CarouselCommand carouselCommand = new CarouselCommand(carousel);
  public final IntakeCommand intakeCommand = new IntakeCommand(intake, Constants.INTAKE_SPEED);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  public class Throttle implements DoubleSupplier{
    @Override
    public double getAsDouble() {
      //return xbox.getY(Hand.kLeft);
      return xbox.getTriggerAxis(Hand.kLeft) - xbox.getTriggerAxis(Hand.kRight);// use trigger for throttle
    }
  }

  public class Turn implements DoubleSupplier{
    @Override
    public double getAsDouble() {
      return xbox.getX(Hand.kLeft);
    }
  }
  
  public class Shoulder implements DoubleSupplier{
    @Override
    public double getAsDouble() {
      //return xbox.getTriggerAxis(Hand.kRight) - xbox.getTriggerAxis(Hand.kLeft);// set shoulder speed 
      return 0.0;
    }
  }

  private void configureButtonBindings() {
    if (Robot.isSimulation()) {
      // for the simulation, silence warnings about missing joysticks
      DriverStation.getInstance().silenceJoystickConnectionWarning(true);
    }

    // JoystickButton xboxA = new JoystickButton(xbox, Constants.XBOX_A);
    // JoystickButton xboxB = new JoystickButton(xbox, Constants.XBOX_B);
    // JoystickButton xboxX = new JoystickButton(xbox, Constants.XBOX_X);
    JoystickButton xboxY = new JoystickButton(xbox, Constants.XBOX_Y);
    xboxY.whenPressed(new TurnAndShoot(robotDrive, shooter, carousel, carouselCommand, driveCommand, true));
    // JoystickButton xbox7 = new JoystickButton(xbox, Constants.XBOX_BACK);
    // JoystickButton xboxStart = new JoystickButton(xbox, Constants.XBOX_START);

    JoystickButton bumperRight = new JoystickButton(xbox, Constants.XBOX_RB);
    bumperRight.whileHeld(new IntakeCommand(intake, Constants.INTAKE_SPEED));

    JoystickButton bumperLeft = new JoystickButton(xbox, Constants.XBOX_LB);
    bumperLeft.whileHeld(new IntakeCommand(intake, -Constants.INTAKE_SPEED));
    
    // JoystickButton farm1 = new JoystickButton(farm, 1);
    // JoystickButton farm2 = new JoystickButton(farm, 2);
    // JoystickButton farm4 = new JoystickButton(farm, 4);
    // JoystickButton farm5 = new JoystickButton (farm, 5);
    // JoystickButton farm6 = new JoystickButton(farm, 6);
    // JoystickButton farm7 = new JoystickButton(farm, 7);
    // JoystickButton farm11 = new JoystickButton(farm, 11);
    // JoystickButton farm14 = new JoystickButton(farm, 14);
    // JoystickButton farm16 = new JoystickButton(farm, 16);
    // JoystickButton farm21 = new JoystickButton(farm, 21);
  }

  // public boolean APressed () {
  //   return xboxA.get();
  // }

  /**
   * LigerBots: we don't use this function. 
   * Autonomous is controlled by a Chooser defined in Robot.
   */
  // public Command getAutonomousCommand() {
  //   return null;
  // }
}