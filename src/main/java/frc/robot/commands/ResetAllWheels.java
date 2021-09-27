// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drivetrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ResetAllWheels extends ParallelCommandGroup {
  /** Creates a new ResetAllWheels. */
  private Drivetrain drive;
  public ResetAllWheels(Drivetrain drivetrain) {
    this.drive = drivetrain;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new ResetWheels(drive.frontLeftAngle, drive.frontLeftEncoder), 
    new ResetWheels(drive.frontRightAngle, drive.frontRightEncoder),
    new ResetWheels(drive.backLeftAngle, drive.backLeftEncoder),
    new ResetWheels(drive.backRightAngle, drive.backRightEncoder));
  }
}
