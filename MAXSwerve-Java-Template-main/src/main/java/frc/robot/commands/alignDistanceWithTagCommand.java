// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimelightHelpers;
import edu.wpi.first.math.controller.PIDController;
public class alignDistanceWithTagCommand extends Command {


  private DriveSubsystem m_driveSubsystem;
  private double m_targetZ; 
  private double m_targetX;
  private double m_targetRotation;
  private double m_tid;
  private double m_Rotation;
  private double m_Forward;
  private double kSpeedKp = 0.125;
  private double kRotationKp = 0.125;
  private double kSpeedKi = 0;
  private double kRotationKi = 0;
  private double kSpeedKd = 0;
  private double kRotationKd = 0;

  private double m_driveRotTarget;
  private double m_driveForwardTarget;
  PIDController m_rotationController = new PIDController(kRotationKp, kRotationKi, kRotationKd);
  PIDController m_xSpeedController = new PIDController(kSpeedKp, kSpeedKi, kSpeedKd);
  PIDController m_zSpeedController = new PIDController(kSpeedKp, kSpeedKi, kSpeedKd);

  private boolean m_tidFound = false;

  
  private NetworkTable m_table;

  public alignDistanceWithTagCommand(DriveSubsystem driveSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_driveSubsystem = driveSubsystem;

    addRequirements(m_driveSubsystem);

        NetworkTableInstance inst = NetworkTableInstance.getDefault();
    m_table = inst.getTable("ReefAlignment");

   

    

   
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_table=NetworkTableInstance.getDefault().getTable("limelight");
    double tid = LimelightHelpers.getFiducialID("limelight");
    m_tid=tid;
    
    //change later to actual angle (tweak it)
    m_targetRotation = 0;

    m_tidFound = m_targetRotation >= 0;
    if (m_tidFound == false)
    {
      return;
    }
    //tweak on a per id basis, just a generic value for april tag
    m_targetZ =0;

    

    m_xSpeedController.setSetpoint(m_targetZ);
    m_zSpeedController.setSetpoint(m_targetZ);
    m_rotationController.setSetpoint(m_targetRotation);
    m_Rotation=LimelightHelpers.getTX("limelight");

  }

  private double rotate(){
    double kP=.035;
    m_Rotation=LimelightHelpers.getTX("limelight") *kP;

    return m_rotationController.calculate(m_Rotation)*(Math.PI);
  }
  private double forward(){
    double kP=.1;
    m_Forward=LimelightHelpers.getTY("limelight") *kP;

    return m_xSpeedController.calculate(m_Forward)*(1);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    m_tid = LimelightHelpers.getFiducialID("limelight");
  
    SmartDashboard.putNumber(" command tid", m_tid);
   
    if (m_tid!=-1 && m_tid!=0) {
    

     
      //rotSpeed
      m_driveRotTarget=rotate();
     

      //FORWARD SPEED
      m_driveForwardTarget=forward(); 

      SmartDashboard.putBoolean("isValidId", true);
      SmartDashboard.putNumber("rotError", Math.abs(m_targetRotation-m_driveRotTarget));
      SmartDashboard.putNumber("drivespeed", m_driveForwardTarget);

      if(Math.abs(m_targetRotation-m_driveRotTarget )>0.05){
          m_driveSubsystem.drive( 0, 0, m_driveRotTarget, false);
      }
      else{
          m_driveSubsystem.drive(-1* m_driveForwardTarget, 0, 0, false);
      }
    
    } else {
      SmartDashboard.putBoolean("isValidId", false);
      m_tidFound=false;
      m_driveSubsystem.drive(0, 0, 0, true);
    }
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_driveSubsystem.drive(0, 0, 0, true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_tidFound == false;
  }
}
