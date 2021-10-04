// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class Drivetrain extends SubsystemBase {
  public static CANCoder frontLeftEncoder = new CANCoder(0);
  public static CANCoder frontRightEncoder = new CANCoder(3);
  public static CANCoder backLeftEncoder = new CANCoder(1);
  public static CANCoder backRightEncoder = new CANCoder(2);

  public TalonFX frontLeftDrive = new TalonFX(5);
  public TalonFX frontLeftAngle = new TalonFX(4);
  
  public TalonFX frontRightDrive = new TalonFX(3);
  public TalonFX frontRightAngle = new TalonFX(2);

  public TalonFX backLeftDrive = new TalonFX(1);
  public TalonFX backLeftAngle = new TalonFX(0);

  public TalonFX backRightDrive = new TalonFX(7);
  public TalonFX backRightAngle = new TalonFX(6);
  /** Creates a new Drivetrain. */
  public Drivetrain() {}

/**
 * Sets the wheels to either 0 or 180 so we can drive in forward/reverse
 * @param angleMotor - TalonFX passed in for the wheel we are currently resetting
 * @param encoder - angle of the wheel we are resetting
 */

  public void resetWheel(){
    double angle = frontLeftEncoder.getAbsolutePosition();
    frontRightAngle.set(ControlMode.Follower, 4);
    backLeftAngle.set(ControlMode.Follower, 4);
    backRightAngle.set(ControlMode.Follower, 4);
    
    if(angle < 180){
      frontLeftAngle.set(ControlMode.PercentOutput, .3);
      if(angle < 90)
        frontLeftAngle.set(ControlMode.PercentOutput, .2);
      if(angle < 45)
        frontLeftAngle.set(ControlMode.PercentOutput, .08);
    }else{
      frontLeftAngle.set(ControlMode.PercentOutput, -.3);
      if(angle > 275)
        frontLeftAngle.set(ControlMode.PercentOutput, -.2);
      if(angle > 315)
        frontLeftAngle.set(ControlMode.PercentOutput, -.08);
    }
  }

  /**
   * Returns whether or not the current wheel is is at 180 or 0/360 (or at least within a range of a few degrees set in this command)
   * @param encoder - gives angle of the wheel we are resetting
   * @return - returns true if the wheel has been reset, returns false otherwise. 
   */
  public boolean isReset(CANCoder encoder){
    if(encoder.getAbsolutePosition() < 2 || encoder.getAbsolutePosition() > 358){
      return true;
    }else{
      return false;
    }
  }


  public void stopWheel(TalonFX motor){
    motor.set(ControlMode.PercentOutput, 0);
  }

  public void joystickDrive(){
    frontRightDrive.set(ControlMode.Follower, 5);
    backLeftDrive.set(ControlMode.Follower, 5);
    backRightDrive.set(ControlMode.Follower, 5);

    frontRightAngle.set(ControlMode.Follower, 4);
    backLeftAngle.set(ControlMode.Follower, 4);
    backRightAngle.set(ControlMode.Follower, 4);

    frontLeftDrive.set(ControlMode.PercentOutput, RobotContainer.getJoystick().getRawAxis(1));
    frontLeftAngle.set(ControlMode.PercentOutput, RobotContainer.getJoystick().getRawAxis(5) / 2);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
