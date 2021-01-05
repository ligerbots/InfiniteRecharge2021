/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.Carousel;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
 //import edu.wpi.first.wpilibj.XboxController; will need later
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.GenericHID.Hand;

import frc.robot.commands.*;
import frc.robot.subsystems.DriveTrain;

@SuppressWarnings("all")
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

  private final Shoulder shoulder = new Shoulder();
  public final Climber climber = new Climber(robotDrive);



  //public final NewEightBallSim newEightBallSimCommand = new NewEightBallSim(robotDrive, driveCommand, climber);

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
      return xbox.getY(Hand.kLeft);
    }
  }

  public class Turn implements DoubleSupplier{

    @Override
    public double getAsDouble() {
      return xbox.getX(Hand.kRight);
    }
  }
  public class Shoulder implements DoubleSupplier{

    @Override
    public double getAsDouble() {
      return xbox.getTriggerAxis(Hand.kRight) - xbox.getTriggerAxis(Hand.kLeft);// set shoulder speed 
    }
  }

  private void configureButtonBindings() {
    JoystickButton xboxA = new JoystickButton(xbox, Constants.XBOX_A);
    JoystickButton xboxB = new JoystickButton(xbox, Constants.XBOX_B);
    JoystickButton xboxX = new JoystickButton(xbox, Constants.XBOX_X);
    JoystickButton xboxY = new JoystickButton(xbox, Constants.XBOX_Y);
    JoystickButton xbox7 = new JoystickButton(xbox, Constants.XBOX_BACK);
    JoystickButton xboxLine = new JoystickButton(xbox, Constants.XBOX_START);
    JoystickButton bumperRight = new JoystickButton(xbox, Constants.XBOX_RB);
    JoystickButton bumperLeft = new JoystickButton(xbox, Constants.XBOX_LB);

    
    JoystickButton xboxStart = new JoystickButton(xbox, Constants.XBOX_START);

    JoystickButton farm1 = new JoystickButton(farm, 1);

    JoystickButton farm6 = new JoystickButton(farm, 6);

    JoystickButton farm2 = new JoystickButton(farm, 2);

    JoystickButton farm7 = new JoystickButton(farm, 7);

    JoystickButton farm4 = new JoystickButton(farm, 4);

    JoystickButton farm5 = new JoystickButton (farm, 5);
    

    JoystickButton farm11 = new JoystickButton(farm, 11);

    JoystickButton farm14 = new JoystickButton(farm, 14);
    JoystickButton farm16 = new JoystickButton(farm, 16);

    JoystickButton farm21 = new JoystickButton(farm, 21);
  }


  /*public boolean APressed () {
    return xboxA.get();
  }*/


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  //EightBallAuto auto = new EightBallAuto(robotDrive, shooter, intake, climber, carousel, driveCommand, carouselCommand);
  public Command _getAutonomousCommand() {
    return null;
  }
}