// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  public static final Drivetrain drivetrain = new Drivetrain();
  
  private static final XboxController joystick = new XboxController(0);

  public final static CANCoder frontLeftEncoder = new CANCoder(0);
  public final static CANCoder frontRightEncoder = new CANCoder(3);
  public final static CANCoder backLeftEncoder = new CANCoder(1);
  public final static CANCoder backRightEncoder = new CANCoder(2);

  public final static TalonFX frontLeftDrive = new TalonFX(5);
  public final static TalonFX frontLeftAngle = new TalonFX(4);
  
  public final static TalonFX frontRightDrive = new TalonFX(3);
  public final static TalonFX frontRightAngle = new TalonFX(2);

  public final static TalonFX backLeftDrive = new TalonFX(1);
  public final static TalonFX backLeftAngle = new TalonFX(0);

  public final static TalonFX backRightDrive = new TalonFX(7);
  public final static TalonFX backRightAngle = new TalonFX(6);

  

  private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);
  //private final ResetWheels resetWheels = new ResetWheels(drivetrain);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
   //   new JoystickButton(joystick, Button.kA.value).whenPressed(new ResetAllWheels(drivetrain));
   //   new JoystickButton(joystick, Button.kB.value).whenPressed(new AllWheelsToNinety(drivetrain));
    
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_autoCommand;
  }

  public static XboxController getJoystick(){
    return joystick;
  }
}
