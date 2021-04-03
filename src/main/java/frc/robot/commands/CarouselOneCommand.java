package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.Carousel;

public class CarouselOneCommand extends CommandBase {
  /**
   * Creates a new CarouselCommand.
   */

  Carousel carousel;

  double targetSlot;
  double sensorStartTime;

  final double sensorWaitTime = 0.04; // seconds
  // TODO find a better value
  // must be the carousel's overshoot ticks divided by the ticks in one fith of a
  // rotation
  final double earlySlotStopDelta = 1830.0 / Constants.CAROUSEL_FIFTH_ROTATION_TICKS;

  private static enum State {
    // There are 4 possible states: Rotating, WaitingForBall, Full, WaitForSensor
    Rotating, WaitingForBall, Full, WaitingForSensor;

  }

  private State state;
  private double currentSlot;

  public CarouselOneCommand(Carousel carousel) {
    this.carousel = carousel;
    addRequirements(carousel);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // save the current slot
    currentSlot = carousel.getSlot();
    targetSlot = Math.round(carousel.getSlot()) + 1.0 - earlySlotStopDelta;
    // check the carousel is on the center of a slot
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // move carousel. It will finish() when it has gone far enough
    carousel.spin(Constants.CAROUSEL_INTAKE_SPEED);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Since isFinshed always returns false, the only way we get here is if we're interrupted.
    // Need to stop the carousel because if it's spinning when we're interrupted, the interrupting
    // command might not expect it to be rotating.
    carousel.spin(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (carousel.getSlot() >= targetSlot);
  }
}