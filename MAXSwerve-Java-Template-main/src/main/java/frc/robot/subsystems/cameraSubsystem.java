package frc.robot.subsystems;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.IntegerPublisher;

//Far from complete
public class cameraSubsystem  extends SubsystemBase{
    NetworkTable m_table;
    NetworkTableEntry m_ty;
    NetworkTableEntry m_tx;
    private double m_targetOffsetAngleVertical;

    // how many degrees back is your limelight rotated from perfectly vertical?
    private double m_limelightMountAngleDegrees = 25.0; 
    // distance from the center of the Limelight lens to the floor
    private double m_limelightLensHeightInches = 20.0; 
    //private DriveSubsystem m_DriveSubsystem;
    // distance from the target to the floor
    private double m_goalHeightInches = 60.0; 
    private DriveSubsystem m_driveSubsystem;
    public cameraSubsystem(DriveSubsystem driveSubsystem) {
        m_table=NetworkTableInstance.getDefault().getTable("limelight");
        m_ty = m_table.getEntry("ty");
        m_tx = m_table.getEntry("ty");
        m_targetOffsetAngleVertical=m_ty.getDouble(0.0);
        m_driveSubsystem=driveSubsystem;
    }
    @Override
    public void periodic() {
        
    }
    //WIP
    public double driveTo(){
        
        double KpDist=-0.1;
        double currentDist=getDist(1);

        double distError= getDesiredDist(1)-currentDist;
        double distDriveForward=KpDist * distError;
        return distDriveForward;
        
    }

    private double getDist(int aprilTagID){
        
        m_ty = m_table.getEntry("ty");
        m_goalHeightInches=getAprilTagHeight(1);
        m_targetOffsetAngleVertical=m_ty.getDouble(0.0);
        
        //distance
        double angleToGoalRadians =  (m_limelightMountAngleDegrees + m_targetOffsetAngleVertical)* (3.14159 / 180.0);
        return (m_goalHeightInches - m_limelightLensHeightInches) / Math.tan(angleToGoalRadians);
    }
    public double setAngle(int aprilTagID){
        float KpAim = 0.035f;
        m_tx = m_table.getEntry("tx");
        double targetAngle = m_tx.getDouble(0.0)*KpAim;
        double maxAngSpeed=Math.PI;
        targetAngle *= maxAngSpeed;
        targetAngle *= -1.0;

        return targetAngle;
    }

    private double getDesiredDist(int aprilTagID){
        if(aprilTagID==1){
            return 10.0;
        }
        return 0.0;
    }

    private double getAprilTagHeight(int aprilTagID){
        if(aprilTagID==1){
            return 60.0;
        }
        return 0.0;
    }
    
}
