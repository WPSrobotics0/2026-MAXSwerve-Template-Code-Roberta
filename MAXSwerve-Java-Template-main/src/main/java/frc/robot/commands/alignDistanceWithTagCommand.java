// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimelightHelpers;
import edu.wpi.first.math.controller.PIDController;
/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class alignDistanceWithTagCommand extends Command {
  /** Creates a new AlignWithIntakeCommand. */

  private DriveSubsystem m_driveSubsystem;
  private double m_targetZ; 
  private double m_targetX;
  private double m_targetRotation;
  private double m_Z;
  private double m_X;
  private double m_Rotation;
  private double kSpeedKp = 0.75;
  private double kRotationKp = 0.035;
  private double kSpeedKi = 0;
  private double kRotationKi = 0;
  private double kSpeedKd = 0;
  private double kRotationKd = 0;
  private double m_driveZTarget;
  private double m_driveXTarget;
  private double m_driveRotTarget;
  PIDController m_rotationController = new PIDController(kRotationKp, kRotationKi, kRotationKd);
  PIDController m_xSpeedController = new PIDController(kSpeedKp, kSpeedKi, kSpeedKd);
  PIDController m_zSpeedController = new PIDController(kSpeedKp, kSpeedKi, kSpeedKd);

  private boolean m_tidFound = false;

  private final DoublePublisher m_alignmentProgressPub;
  private final BooleanPublisher m_alignmentCompletePub;
  private final DoublePublisher m_targetAnglePub;
  private final DoublePublisher m_tidPub;
  private NetworkTable m_table;

   //constants for progress calculation
   private static final double MAX_ROTATION_ERROR = 10.0; //degrees
   private static final double MAX_POSITION_ERROR = 1.0; //units

  public alignDistanceWithTagCommand(DriveSubsystem driveSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_driveSubsystem = driveSubsystem;

    addRequirements(m_driveSubsystem);

        NetworkTableInstance inst = NetworkTableInstance.getDefault();
    NetworkTable table = inst.getTable("ReefAlignment");

    var progressPub = table.getDoubleTopic("progress");
    m_alignmentProgressPub = progressPub.publish();
    m_alignmentProgressPub.set(0.0);

    var completePub = table.getBooleanTopic("Complete");
    m_alignmentCompletePub = completePub.publish();
    m_alignmentCompletePub.set(false);

    var targetAnglePub = table.getDoubleTopic("target Angle");
    m_targetAnglePub = targetAnglePub.publish();
    m_targetAnglePub.set(0.0);

    var tidPub = table.getDoubleTopic("tid");
    m_tidPub = tidPub.publish();
    m_tidPub.set(0.0);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_table=NetworkTableInstance.getDefault().getTable("limelight");
    var tid = LimelightHelpers.getFiducialID("limelight");

    //change later to actual angle (tweak it)
    m_targetRotation = 45;

    m_tidFound = m_targetRotation >= 0;
    if (m_tidFound == false)
    {
      return;
    }
    //tweak on a per id basis, just a generic value for april tag
    m_targetZ =3.5;
    m_targetX =3.5;

    m_targetAnglePub.set(m_targetRotation);
    m_tidPub.set(tid);

    //m_AprilTagPID.setTargetPosition(m_targetX, m_targetZ, m_targetRotation);
    m_xSpeedController.setSetpoint(m_targetX);
    m_zSpeedController.setSetpoint(m_targetZ);
    m_rotationController.setSetpoint(m_targetRotation);
    m_X=m_targetX;
    m_Z=m_targetZ;
    m_Rotation=m_targetRotation;


    //reset dashboard values
    m_alignmentProgressPub.set(0);
    m_alignmentCompletePub.set(false);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
        double[] robotSpace = LimelightHelpers.getTargetPose_RobotSpace("limelight");
    if (robotSpace != null && robotSpace.length >= 3) {
      //m_AprilTagPID.driveToTarget();
      
      
  
      

      //zSpeed
      m_Z=robotSpace[2];
      double maxDriveSpeed=4.8;
      m_driveZTarget=m_zSpeedController.calculate(m_Z)* maxDriveSpeed;

      //xSpeed
      m_X=robotSpace[0];
      m_driveXTarget=m_xSpeedController.calculate(m_X)* maxDriveSpeed;

      //rotSpeed
      m_Rotation=m_driveSubsystem.getAngle().getDegrees() % 360;
      if (m_Rotation < m_targetRotation)
        {
            if (Math.abs(m_Rotation - m_targetRotation) > 90)
                m_Rotation += 360;
        }
        else if (m_Rotation > m_targetRotation)
        {
            if (Math.abs(m_Rotation - m_targetRotation) > 90)
                m_Rotation -= 360;
        }
      //double remapAngle=m_Rotation % 360;
      //if (remapAngle< m_targetRotation)

      m_driveRotTarget=m_rotationController.calculate(m_Rotation)*Math.PI;



      m_driveSubsystem.drive(-1* m_driveZTarget, -1*m_driveXTarget, m_driveRotTarget, false);
      // Calculate alignment progress and update dashboard
      // updateDashboard();

    } else {
      // If no AprilTag data is available, stop the robot and show 0% progress
      m_driveSubsystem.drive(0, 0, 0, true);
    }
  }

  /*private void updateDashboard() {
    boolean isComplete = m_AprilTagPID.atTargetPosition();
    m_alignmentCompletePub.set(isComplete);
    
    // Get the current errors
    double deltaX =Math.abs(m_targetX-m_X);
    double deltaZ =Math.abs(m_targetZ-m_Z);
    double deltaRotation = Math.abs(m_driveSubsystem.getAngle().getDegrees()-);
    
    // Ensure the PID controller has valid AprilTag data
    double[] robotSpace = LimelightHelpers.getTargetPose_RobotSpace("limelight");
    if (robotSpace == null || robotSpace.length < 3) {
      // Invalid data, return early without updating progress
      return;
        }

              // Calculate the total distance from target (Euclidean distance in 3D space)
    // Scale rotation by a factor to make it comparable to position values
    double rotationScale = MAX_POSITION_ERROR / MAX_ROTATION_ERROR;
    double scaledRotation = deltaRotation * rotationScale;
    
    // Calculate the total distance using 3D Euclidean distance formula
    double totalDistance = Math.sqrt(deltaX * deltaX + deltaZ * deltaZ + scaledRotation * scaledRotation);
    
    // Calculate maximum possible distance for normalization
    double maxDistance = Math.sqrt(
        MAX_POSITION_ERROR * MAX_POSITION_ERROR + 
        MAX_POSITION_ERROR * MAX_POSITION_ERROR + 
        (MAX_ROTATION_ERROR * rotationScale) * (MAX_ROTATION_ERROR * rotationScale)
    );
    
    // Calculate progress as percentage (inverse of distance)
    double progressPercentage = Math.max(0, Math.min(100, (1.0 - totalDistance / maxDistance) * 100));
    
    // If we're complete, progress is 100%
    if (isComplete) {
      progressPercentage = 100.0;
    }
    
    m_alignmentProgressPub.set(progressPercentage);
  }*/


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_driveSubsystem.drive(0, 0, 0, true);
    m_alignmentCompletePub.set(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_tidFound == false || (Math.abs(m_driveRotTarget-m_Rotation) < 3.0 && Math.abs(m_targetX-m_X) < 0.05 && Math.abs(m_targetZ-m_Z) < 0.05);
  }
}
