// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
//import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.alignDistanceWithTagCommand;
import frc.robot.subsystems.DriveSubsystem;
//import frc.robot.subsystems.cameraSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  
  private final SendableChooser<Command> autoChooser;
  //private final cameraSubsystem m_CameraSubsystem = new cameraSubsystem(m_robotDrive);
  // The driver's controller
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
    
    configureButtonBindings();
    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband)*m_speedMultiplier,
                -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband)*m_speedMultiplier,
                -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband)*m_speedMultiplier,
                fieldRelative),
            m_robotDrive));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  boolean fieldRelative = true;
  private void configureButtonBindings() {
    
    /*new JoystickButton(m_driverController, XboxController.Button.kA.value)
        .whileTrue(new RunCommand(
            () -> m_robotDrive.drive(m_CameraSubsystem.driveTo(),0,m_CameraSubsystem.setAngle(1),false),
            m_robotDrive));
    */
    new JoystickButton(m_driverController, XboxController.Button.kStart.value)
        .onTrue(new InstantCommand(
            () -> m_robotDrive.zeroHeading(),
            m_robotDrive));

    new JoystickButton(m_driverController, XboxController.Button.kY.value).onTrue(new InstantCommand(() -> fieldRelative = false));
    new JoystickButton(m_driverController, XboxController.Button.kX.value).onTrue(new InstantCommand(() -> fieldRelative = true));
    new JoystickButton(m_driverController, XboxController.Button.kLeftBumper.value).whileTrue(new InstantCommand(() -> m_speedMultiplier = 0.5));
    new JoystickButton(m_driverController,XboxController.Button.kLeftBumper.value).whileFalse(new InstantCommand(() -> m_speedMultiplier = 1.0));
    
    new JoystickButton(m_driverController,XboxController.Button.kA.value).onTrue(new alignDistanceWithTagCommand(m_robotDrive));
    //new JoystickButton(m_driverController, XboxController.Button.kRightBumper.value).onTrue(new InstantCommand(() -> m_robotDrive.setX()));
  }

  private double m_speedMultiplier = 1.0;
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
