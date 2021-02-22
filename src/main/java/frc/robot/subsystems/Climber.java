/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.commands.DeployIntake;

public class Climber extends SubsystemBase {
    double currentShoulderAngle;
    double requestedShoulderAngle;
    double tempRequestedShoulderAngle;
    double lastShoulderAngle;

    public final CANSparkMax shoulder; // declare new motor
    public final CANSparkMax winch; // declare new motor
    DutyCycleEncoder shoulderEncoder;
    double shoulderSpeedUp = Constants.SHOULDER_SPEED_UP; // set shoulder movement speed
    double shoulderSpeedHold = Constants.SHOULDER_SPEED_HOLD; //This is not enough to lift the intake, but wll hold it in place
    double shoulderRateDown = Constants.SHOULDER_RATE_DOWN; // a little under 2 seconds to get from max height to min height

    boolean deployed = false;
    boolean shoulderMovingDown = false;
    boolean autoLevel = false;
    boolean hookGoingUp = true;
    double currentWinchHeight;
    double requestedWinchHeight;
  
    CANEncoder winchEncoder;
    //SHOULDER ENCODER IS AT 0.44 WHEN DOWN ALL THE WAY

    private DriveTrain driveTrain;

    public Climber(DriveTrain driveTrain) {
        shoulder = new CANSparkMax(Constants.SHOULDER_MOTOR_CAN_ID, MotorType.kBrushless); //init motor type and can id
        shoulder.setIdleMode(IdleMode.kCoast); // set to Coast at startup
        winch = new CANSparkMax(Constants.WINCH_MOTOR_CAN_ID, MotorType.kBrushless); //init motor type and can id
        winch.setIdleMode(IdleMode.kBrake);// set to break when the motor is speed 0
        shoulderEncoder = new DutyCycleEncoder(9);
        currentShoulderAngle = shoulderEncoder.get();
        winchEncoder = new CANEncoder(winch);
        this.driveTrain = driveTrain;
    }

    @Override
    public void periodic(){}

    public void moveWinch(double winchHeight) {
        requestedWinchHeight = winchHeight;
    }

    public void stopWinch() {
        winch.setVoltage(0.0);
        winch.setIdleMode(IdleMode.kBrake);
    }

    public void coastWinch() {
        winch.setIdleMode(IdleMode.kCoast);
    }

    public double getWinchPosition(){
        return winchEncoder.getPosition();
    }

   public void resetWinchEncoder(){
       winchEncoder.setPosition(0);
   }

   public double getShoulderPosition(){
       return shoulderEncoder.get();
   }

   public boolean shoulderAtMinHeight(){
       return shoulderEncoder.get() < Constants.SHOULDER_MIN_VELOCITY_HEIGHT;
   }

   public void autoLevel(boolean autoLevel){
       this.autoLevel = autoLevel;
   }

   public boolean autoLeveling(){
       return autoLevel;
   }

   public boolean shoulderOnTarget () {
       return Math.abs(requestedShoulderAngle - shoulderEncoder.get()) <= 0.05;
   }
}
