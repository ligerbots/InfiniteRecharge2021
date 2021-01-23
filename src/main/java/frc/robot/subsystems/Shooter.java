/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package frc.robot.subsystems;

import java.util.Arrays;
import java.io.BufferedReader;
import java.io.FileReader;
import java.util.TreeMap;
import java.util.Map.Entry;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

// suppress unneeded warnings about serialVerionUID in TreeMap
@SuppressWarnings("serial")
public class Shooter extends SubsystemBase {

    CANSparkMax motor1, motor2, motor3;
    CANSparkMax flup;
    CANEncoder shooterEncoder;
    Servo hoodServo, turretServo;
    TreeMap<Double, Double[]> distanceLookUp = new TreeMap<Double,Double[]>() {}; //set up lookup table for ranges
    TreeMap<Double, Double> turretAngleLookup = new TreeMap<Double, Double>() {};
    CANPIDController pidController;

    public Vision vision;
    public int RPMAdjustment = 0;
    public int HoodAdjustment = 0;
    public double angleErrorAfterTurn = 0;

    public Shooter(Vision vision) {
        this.vision = vision;
        motor1 = new CANSparkMax(Constants.SHOOTER_ONE_CAN_ID, MotorType.kBrushless);
        motor2 = new CANSparkMax(Constants.SHOOTER_TWO_CAN_ID, MotorType.kBrushless);
        motor3 = new CANSparkMax(Constants.SHOOTER_THREE_CAN_ID, MotorType.kBrushless);

        flup = new CANSparkMax(Constants.SHOOTER_FLUP_CAN_ID, MotorType.kBrushless);

        // Set motors to coast when idle. 
        Arrays.asList(motor1, motor2, motor3, flup)
            .forEach((CANSparkMax spark) -> spark.setIdleMode(IdleMode.kCoast));

        hoodServo = new Servo(Constants.SHOOTER_SERVO_PWM_ID);
        turretServo = new Servo(Constants.SHOOTER_TURRET_SERVO_ID);

        shooterEncoder = new CANEncoder(motor2);
        shooterEncoder.setVelocityConversionFactor(2.666);

        pidController = new CANPIDController(motor2);
        pidController.setFeedbackDevice(shooterEncoder);

        // We want motor2 to be master and motor1 and 3 follow the speed of motor2
        motor1.follow(motor2, true);
        motor3.follow(motor2);

        motor1.setSmartCurrentLimit(40);
        motor2.setSmartCurrentLimit(40);
        motor3.setSmartCurrentLimit(40);
        
        // Reset Smart Dashboard for shooter test
        SmartDashboard.putString("Shooting", "Idle");

        SmartDashboard.putNumber("Turret Angle", 75);
        SmartDashboard.putNumber("Target Hood Angle", 60);
        SmartDashboard.putNumber("TSR", -5500);

        try (BufferedReader br = new BufferedReader(new FileReader("/home/lvuser/ShooterData.csv"))){
            String line;
            while ((line = br.readLine()) != null) {
                if (line.trim().length() == 0) continue;
                String values_str[] = line.split(",");

                double values[] = new double[values_str.length];
                for (int i = 0; i < values.length; i++) {
                    values[i] = Double.parseDouble(values_str[i].trim());
                }               

                distanceLookUp.put(values[0], new Double[] {values[1], values[2]});
            }
        }
        catch (Exception e) {
            System.err.println("Error trying to read or parse ShooterData.csv: " + e.getMessage()); 
            System.err.println("Using original hard-coded table instead");

            distanceLookUp.put(112.6, new Double[] {5500.0, 90.0});
            distanceLookUp.put(137.1, new Double[] {5500.0, 80.0});
            distanceLookUp.put(168.9, new Double[] {6000.0, 70.0});
            distanceLookUp.put(227.0, new Double[] {7000.0, 65.0});
            distanceLookUp.put(235.2, new Double[] {7500.0, 55.0});            
            distanceLookUp.put(253.4, new Double[] {7500.0, 60.0});
            distanceLookUp.put(318.1, new Double[] {8000.0, 60.0});
        }
      
        turretAngleLookup.put(-5.0, 57.5);
        turretAngleLookup.put(-4.0, 61.0);
        turretAngleLookup.put(-3.0, 64.0);
        turretAngleLookup.put(-2.0, 68.0);
        turretAngleLookup.put(-1.0, 71.5);
        turretAngleLookup.put(0.0, 79.5);
        turretAngleLookup.put(1.0, 85.0);
        turretAngleLookup.put(2.0, 89.0);
        turretAngleLookup.put(3.0, 93.0);
        turretAngleLookup.put(4.0, 97.0);
        turretAngleLookup.put(5.0, 101.0);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Shooter RPM", getSpeed());
        SmartDashboard.putNumber("Shooter motor current", motor2.getOutputCurrent());
        SmartDashboard.putNumber("Hood Adjustment", HoodAdjustment);
        SmartDashboard.putNumber("RPM Adjustment", RPMAdjustment);
        SmartDashboard.putNumber("Output Voltage", motor2.getAppliedOutput());
    }

    public double getVoltage() {
        return motor2.getBusVoltage();
    }

    public void setHood (double angle) {
        System.out.println("hood angle SET!!!!");
        if (angle < 40) {
            angle = 40;
        }
        if (angle > 160) {
            angle = 160;
        }
        hoodServo.setAngle(angle);
    }

    public double getSpeed () {
        return -shooterEncoder.getVelocity();
    }

    public void prepareShooter(double distance) {
        pidController.setReference(-calculateShooterSpeed(distance), ControlType.kVelocity);
        hoodServo.setAngle(calculateShooterHood(distance));
        /* The idea was that this would set the shooter speed and hoodServo value 
        based on the input distance. 
        */
    }

    // public void setShooterVoltage (double voltage) {
    //     pidController.setReference(voltage, ControlType.kVoltage);
    // }

    public void shoot () {
        //if (flup.getOutputCurrent() < Constants.FLUP_STOP_CURRENT) {
            flup.set(-0.5);
        //} else {
        //    flup.set(0);
        //}*/
    }

    public void testSpin () {
        pidController.setReference(-4000, ControlType.kVelocity);
        SmartDashboard.putString("Shooting", "Shooting");
    }

    public void setShooterRpm (double rpm) {
        System.out.println("Shooter RPM SET!!!!!");
        if (rpm < 0) System.out.println("warning: rpm values should be positive");
        pidController.setReference(-Math.abs(rpm), ControlType.kVelocity, 0, -0.8);
    }

    public double calculateShooterSpeed(double distance) {
        Entry<Double, Double[]> floorEntry = distanceLookUp.floorEntry(distance);
        Entry<Double, Double[]> ceilingEntry = distanceLookUp.higherEntry(distance);
        if (floorEntry != null && ceilingEntry != null) {

            // Charles' calculation
            double ratio = 1 - (ceilingEntry.getKey() - distance) / (ceilingEntry.getKey() - floorEntry.getKey());
            double result = floorEntry.getValue()[0] + ratio * (ceilingEntry.getValue()[0] - floorEntry.getValue()[0]);

            System.out.format("Shooter: ratio %3.2f, floor %4.1f, dist %4.1f, ceiling %4.1f, RPM %4.1f",
                ratio, floorEntry.getKey(), distance,  ceilingEntry.getKey(), result);
            return result/* + Robot.RPMAdjustment*/;
        }
        else {
            System.out.println("Shooter: floorEntry or celingEntry was null");
            return -1000;
        }
    }

    public double calculateShooterHood(double distance) {
        Entry<Double, Double[]> floorEntry = distanceLookUp.floorEntry(distance);
        Entry<Double, Double[]> ceilingEntry = distanceLookUp.higherEntry(distance);

        if (floorEntry != null && ceilingEntry != null) {
            // Charles calculation
            double ratio = 1 - (ceilingEntry.getKey() - distance) / (ceilingEntry.getKey() - floorEntry.getKey());
            double result = floorEntry.getValue()[1] + ratio * (ceilingEntry.getValue()[1] - floorEntry.getValue()[1]);
            System.out.format(" hood %3.0f%n", result);

            // Mark's calculation
            // double result = (ceilingEntry.getValue()[1] - floorEntry.getValue()[1]) / (ceilingEntry.getKey() - floorEntry.getKey()) * (ceilingEntry.getKey() - distance) / (ceilingEntry.getKey() - floorEntry.getKey())  + floorEntry.getValue()[1];

            return result + HoodAdjustment;
        }
        else {
            return 60;
        }
    }

    public void warmUp () {
        setShooterRpm(3000);
    }

    public boolean speedOnTarget (final double targetVelocity, final double percentAllowedError) {
        final double max = targetVelocity * (1.0 + (percentAllowedError / 100.0));
        final double min = targetVelocity * (1.0 - (percentAllowedError / 100.0));
        return shooterEncoder.getVelocity() > max && shooterEncoder.getVelocity() < min;  //this is wack cause it's negative
    }

    public boolean hoodOnTarget (final double targetAngle) {
        System.out.println("ServoPosition: " + hoodServo.getPosition());
        return hoodServo.getAngle() > targetAngle - 0.5 && hoodServo.getAngle() < targetAngle + 0.5;
    }

    public void calibratePID (final double p, final double i, final double d, final double f) {
        pidController.setIAccum(0);
        pidController.setP(p);
        pidController.setI(i);
        pidController.setD(d);
        pidController.setFF(f);
        pidController.setIZone(1000);
    }

    public void stopAll () {
        pidController.setReference(0, ControlType.kVoltage);
        flup.set(0);
        hoodServo.setAngle(160);
    }

    public double getTurretAngle () {
        return turretServo.get() *  Constants.TURRET_ANGLE_COEFFICIENT;
    }

    public void setTurret (double angle) {
        System.out.println("Moving turret to " + angle);
        turretServo.setAngle(angle);
    }

    public void setTurretAdjusted(double adjustedAngle) {
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

            // Mark's calculation
            // double result = (ceilingEntry.getValue()[1] - floorEntry.getValue()[1]) / (ceilingEntry.getKey() - floorEntry.getKey()) * (ceilingEntry.getKey() - distance) / (ceilingEntry.getKey() - floorEntry.getKey())  + floorEntry.getValue()[1];

            turretServo.setAngle(result);
            System.out.println("Turret Adjustment should be working: " + result + "    " + adjustedAngle);
        }
        else {
            System.out.println("Turret Adjustment not successful      " + adjustedAngle);
            turretServo.setAngle(72);
        }
    }

    
}
