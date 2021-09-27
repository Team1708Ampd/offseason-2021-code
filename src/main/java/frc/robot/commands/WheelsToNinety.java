// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class WheelsToNinety extends CommandBase {
  /** Creates a new WheelsToNinety. */
  private TalonFX angleMotor;
  private CANCoder encoder;
  public WheelsToNinety(TalonFX angleMotor, CANCoder encoder) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.angleMotor = angleMotor; 
    this.encoder = encoder; 
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(encoder.getAbsolutePosition()> 89 && encoder.getAbsolutePosition() < 91){
      return;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double angle = encoder.getAbsolutePosition();
    
    if(angle >270)
      angleMotor.set(ControlMode.PercentOutput, .3);
    else if(angle >0 && angle  < 45)
     angleMotor.set(ControlMode.PercentOutput, .2); 
    else if(angle >45 && angle < 90)
      angleMotor.set(ControlMode.PercentOutput, .08);
    else if(angle <135)
      angleMotor.set(ControlMode.PercentOutput, -.08);
    else if(angle <180)
      angleMotor.set(ControlMode.PercentOutput, -.2);
    else if(angle <270)
      angleMotor.set(ControlMode.PercentOutput, -.3);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    angleMotor.set(ControlMode.PercentOutput, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(encoder.getAbsolutePosition()> 89 && encoder.getAbsolutePosition() < 91)
      return true;
    else return false;
  }
}
