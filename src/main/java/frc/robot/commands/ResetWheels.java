// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drivetrain;

public class ResetWheels extends CommandBase {
  private TalonFX angleMotor;
  private CANCoder encoder;

  /** Creates a new ResetWheels. */

  public ResetWheels(TalonFX angleMotor, CANCoder encoder){
    this.angleMotor = angleMotor;
    this.encoder = encoder;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(encoder.getAbsolutePosition() < 1 || encoder.getAbsolutePosition() > 359)
      return;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double angle = encoder.getAbsolutePosition();
    if(angle < 180){
      angleMotor.set(ControlMode.PercentOutput, -.25);
      if(angle < 90)
        angleMotor.set(ControlMode.PercentOutput, -.2);
      if(angle < 45)
        angleMotor.set(ControlMode.PercentOutput, -.08);
    }else{
      angleMotor.set(ControlMode.PercentOutput, .25);
      if(angle > 275)
        angleMotor.set(ControlMode.PercentOutput, .2);
      if(angle > 315)
        angleMotor.set(ControlMode.PercentOutput, .08);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    angleMotor.set(ControlMode.PercentOutput, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    
    if(encoder.getAbsolutePosition() < 1 || encoder.getAbsolutePosition() > 359){
      return true;
    }else{
      return false;
    }
  }
}
