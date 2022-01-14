/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package frc.robot.subsystems;

import java.util.Arrays;
import java.util.TreeMap;
import java.util.Map.Entry;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

// suppress unneeded warnings about serialVersionUID in TreeMap
@SuppressWarnings("serial")
public class Shooter extends SubsystemBase {

    CANSparkMax motor1, motor2, motor3;
    CANSparkMax flup;
    RelativeEncoder shooterEncoder;
    Servo hoodServo, turretServo;
    TreeMap<Double, Double[]> distanceLookUp = new TreeMap<Double,Double[]>() {}; //set up lookup table for ranges
    TreeMap<Double, Double> turretAngleLookup = new TreeMap<Double, Double>() {};
    SparkMaxPIDController pidController;

    public Vision vision;
    public int rpmAdjustment = 0;
    public int hoodAdjustment = 0;
    public double angleErrorAfterTurn = 0;

    public Shooter(Vision vision) {
        this.vision = vision;
        motor1 = new CANSparkMax(Constants.SHOOTER_ONE_CAN_ID, MotorType.kBrushless);
        motor2 = new CANSparkMax(Constants.SHOOTER_TWO_CAN_ID, MotorType.kBrushless);
        // motor3 = new CANSparkMax(Constants.SHOOTER_THREE_CAN_ID, MotorType.kBrushless);

        flup = new CANSparkMax(Constants.SHOOTER_FLUP_CAN_ID, MotorType.kBrushless);

        // Set motors to coast when idle. 
        Arrays.asList(motor1, motor2, flup)
            .forEach((CANSparkMax spark) -> spark.setIdleMode(IdleMode.kCoast));

        hoodServo = new Servo(Constants.SHOOTER_SERVO_PWM_ID);
        turretServo = new Servo(Constants.SHOOTER_TURRET_SERVO_ID);

        shooterEncoder = motor2.getEncoder();
        shooterEncoder.setVelocityConversionFactor(2.666);

        pidController = motor2.getPIDController();
        pidController.setFeedbackDevice(shooterEncoder);

        // We want motor2 to be master and motor1 and 3 follow the speed of motor2
        motor1.follow(motor2, true);
        // motor3.follow(motor2);

        motor1.setSmartCurrentLimit(40);
        motor2.setSmartCurrentLimit(40);
        // motor3.setSmartCurrentLimit(40);
        
        // Reset Smart Dashboard for shooter test
        SmartDashboard.putString("shooter/Status", "Idle");

        // always have an entry at 0 so that it has a chance of working at short distances
        distanceLookUp.put(0.0, new Double[] { 5500.0, 103.0 });
        distanceLookUp.put(50.0, new Double[] { 5500.0, 103.0 });
        distanceLookUp.put(110.0, new Double[] { 6500.0, 87.0 });
        distanceLookUp.put(170.0, new Double[] { 8500.0, 70.0 });
        distanceLookUp.put(230.0, new Double[] { 9000.0, 60.0 });
        distanceLookUp.put(318.1, new Double[] { 9000.0, 55.0 });
        // extra far, to make sure the table work at the long end
        distanceLookUp.put(400.0, new Double[] { 9000.0, 50.0 });

        // The relative setting for non-zero angles needs to be recomputed if the zero setting chenges,
        // but at least we'll be close
        turretAngleLookup.put(-5.0, Constants.TURRET_ANGLE_ZERO_SETTING - 23.0);
        turretAngleLookup.put(-4.0, Constants.TURRET_ANGLE_ZERO_SETTING - 18.0);
        turretAngleLookup.put(-3.0, Constants.TURRET_ANGLE_ZERO_SETTING - 11.0);
        turretAngleLookup.put(-2.0, Constants.TURRET_ANGLE_ZERO_SETTING - 6.0);
        turretAngleLookup.put(-1.0, Constants.TURRET_ANGLE_ZERO_SETTING - 4.0);
        turretAngleLookup.put( 0.0, Constants.TURRET_ANGLE_ZERO_SETTING);
        turretAngleLookup.put( 1.0, Constants.TURRET_ANGLE_ZERO_SETTING + 4.0);
        turretAngleLookup.put( 2.0, Constants.TURRET_ANGLE_ZERO_SETTING + 7.0);
        turretAngleLookup.put( 3.0, Constants.TURRET_ANGLE_ZERO_SETTING + 10.5);
        turretAngleLookup.put( 4.0, Constants.TURRET_ANGLE_ZERO_SETTING + 13.0);
        turretAngleLookup.put( 5.0, Constants.TURRET_ANGLE_ZERO_SETTING + 15.0);

        // used in ShooterPIDTuner
        SmartDashboard.putNumber("shooter/P", 0.000145);
        SmartDashboard.putNumber("shooter/I", 1e-8);
        SmartDashboard.putNumber("shooter/D", 0);
        SmartDashboard.putNumber("shooter/F", 6.6774 * 0.00001);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("shooter/RPM", getSpeed());
        SmartDashboard.putNumber("shooter/current", motor2.getOutputCurrent());
        SmartDashboard.putNumber("shooter/distance", vision.getDistance());
        SmartDashboard.putNumber("shooter/Hood_Adjustment", hoodAdjustment);
        SmartDashboard.putNumber("shooter/RPM_Adjustment", rpmAdjustment);
        SmartDashboard.putNumber("shooter/Output_Voltage", motor2.getAppliedOutput());
        SmartDashboard.putNumber("shooter/turretAngleRaw", turretServo.getAngle());
    }

    public double getVoltage() {
        return motor2.getBusVoltage();
    }

    public void setHood(double angle) {
        System.out.println("hood angle SET!!!!");
        if (angle < 40) {
            angle = 40;
        }
        if (angle > 160) {
            angle = 160;
        }
        hoodServo.setAngle(angle);
    }

    public double getSpeed() {
        return -shooterEncoder.getVelocity();
    }

    public void prepareShooter(double distance) {
        // Set the shooter and hood based on the distance
        setShooterRpm(calculateShooterSpeed(distance));
        setHood(calculateShooterHood(distance));
    }

    // public void setShooterVoltage (double voltage) {
    //     pidController.setReference(voltage, ControlType.kVoltage);
    // }

    public void shoot() {
        //if (flup.getOutputCurrent() < Constants.FLUP_STOP_CURRENT) {
            flup.set(-0.5);
        //} else {
        //    flup.set(0);
        //}*/
    }

    public void testSpin() {
        setShooterRpm(4000.0);
        SmartDashboard.putString("shooter/Status", "Shooting");
    }

    public void setShooterRpm(double rpm) {
        System.out.println("Shooter RPM SET!!!!!");
        // for the shooter to run the right direction, rpm values passed to setReference must be negative
        // passing the negative absolute value causes the passed value to always be negative, 
        // while allowing the function argument to be positive or negative  
        if (rpm < 0) System.out.println("warning: shooter rpm argument should be positive");
        pidController.setReference(-Math.abs(rpm), ControlType.kVelocity, 0, -0.8);
    }

    public double calculateShooterSpeed(double distance) {
        Entry<Double, Double[]> floorEntry = distanceLookUp.floorEntry(distance);
        Entry<Double, Double[]> ceilingEntry = distanceLookUp.higherEntry(distance);
        if (floorEntry != null && ceilingEntry != null) {

            // Charles' calculation
            double ratio = 1 - (ceilingEntry.getKey() - distance) / (ceilingEntry.getKey() - floorEntry.getKey());
            double rpmAdjustment = floorEntry.getValue()[0] + ratio * (ceilingEntry.getValue()[0] - floorEntry.getValue()[0]);

            System.out.format("Shooter: ratio %3.2f, floor %4.1f, dist %4.1f, ceiling %4.1f, RPM %4.1f",
                ratio, floorEntry.getKey(), distance,  ceilingEntry.getKey(), rpmAdjustment);
            return rpmAdjustment;
        }
        else {
            System.out.println("Shooter: floorEntry or ceilingEntry was null");
            // Typical speed. Not sure this will work for much, but it won't break anything.
            return 4000;
        }
    }

    public double calculateShooterHood(double distance) {
        Entry<Double, Double[]> floorEntry = distanceLookUp.floorEntry(distance);
        Entry<Double, Double[]> ceilingEntry = distanceLookUp.higherEntry(distance);

        if (floorEntry != null && ceilingEntry != null) {
            // Charles calculation
            double ratio = 1 - (ceilingEntry.getKey() - distance) / (ceilingEntry.getKey() - floorEntry.getKey());
            double hoodAdjustment = floorEntry.getValue()[1] + ratio * (ceilingEntry.getValue()[1] - floorEntry.getValue()[1]);
            System.out.format(" hood %3.0f%n", hoodAdjustment);

            return hoodAdjustment;
        }
        else {
            return 60;
        }
    }

    public void warmUp() {
        setShooterRpm(Constants.WARM_UP_RPM);
    }

    public boolean speedOnTarget(final double targetVelocity, final double percentAllowedError) {
        final double max = targetVelocity * (1.0 + (percentAllowedError / 100.0));
        final double min = targetVelocity * (1.0 - (percentAllowedError / 100.0));
        return shooterEncoder.getVelocity() > max && shooterEncoder.getVelocity() < min;  //this is wack cause it's negative
    }

    public boolean hoodOnTarget(final double targetAngle) {
        System.out.println("ServoPosition: " + hoodServo.getPosition());
        return hoodServo.getAngle() > targetAngle - 0.5 && hoodServo.getAngle() < targetAngle + 0.5;
    }

    public void calibratePID(final double p, final double i, final double d, final double f) {
        pidController.setIAccum(0);
        pidController.setP(p);
        pidController.setI(i);
        pidController.setD(d);
        pidController.setFF(f);
        pidController.setIZone(1000);
    }

    public void stopAll() {
        setShooterRpm(0.0);
        flup.set(0);
        setHood(160);
        SmartDashboard.putString("shooter/Status", "Idle");
    }

    public double getTurretAngle() {
        return turretServo.get() *  Constants.TURRET_ANGLE_COEFFICIENT;
    }

    private void setTurret(double angle) {
        System.out.println("Moving turret to " + angle);
        turretServo.setAngle(angle);
    }

    public void setTurretAdjusted(double adjustedAngle) {
        SmartDashboard.putNumber("shooter/turretAngle", adjustedAngle);
        if (adjustedAngle > 5) {
            adjustedAngle = 5;
        }
        if (adjustedAngle < -5) {
            adjustedAngle = -5;
        }
        Entry<Double, Double> floorEntry = adjustedAngle < 0 ? turretAngleLookup.higherEntry(adjustedAngle) : 
                                                                turretAngleLookup.floorEntry(adjustedAngle);
        Entry<Double, Double> ceilingEntry = adjustedAngle < 0 ? turretAngleLookup.floorEntry(adjustedAngle) :  
                                                                turretAngleLookup.higherEntry(adjustedAngle);
                                                        
        if (floorEntry != null && ceilingEntry != null) {
            // Charles calculation
            double ratio = 1 - (ceilingEntry.getKey() - adjustedAngle) / (ceilingEntry.getKey() - floorEntry.getKey());
            double result = floorEntry.getValue() + ratio * (ceilingEntry.getValue() - floorEntry.getValue());

            setTurret(result);
            // System.out.println("Turret Adjustment should be working: " + result + "    " + adjustedAngle);
        }
        else {
            System.out.println("Turret Adjustment not successful      " + adjustedAngle);
            // This must equal the zero setting in turretAngleLookup
            setTurret(Constants.TURRET_ANGLE_ZERO_SETTING);
        }
    }
}
