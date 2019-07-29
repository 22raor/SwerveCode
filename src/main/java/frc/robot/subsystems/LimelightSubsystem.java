/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package frc.robot.subsystems;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.networktables.*;


import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * Add your docs here.
 */
public class LimelightSubsystem extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();


  private boolean m_LimelightHasValidTarget = false;
  private double m_LimelightDriveCommand = 0.0;
  private double m_LimelightSteerCommand = 0.0;

public void a() {

  Update_Limelight_Tracking();


  boolean auto = Robot.oi.stick3.getRawButton(1);



  if (auto)
  {
    if (m_LimelightHasValidTarget)
    {
          Robot.driveSubsystem.drive.arcadeDrive(m_LimelightDriveCommand,m_LimelightSteerCommand);
    }
    else
    {

      Robot.driveSubsystem.drive.arcadeDrive(0.0,0.0);
    }
 
   // m_Drive.arcadeDrive(drive,steer);
  }
}

public void Update_Limelight_Tracking() {

  // These numbers must be tuned for your Robot!  Be careful!
  final double STEER_K = 0.03;                    // how hard to turn toward the target
  final double DRIVE_K = 0.26;                    // how hard to drive fwd toward the target
  final double DESIRED_TARGET_AREA = 13.0;        // Area of the target when the robot reaches the wall
  final double MAX_DRIVE = 0.7;                   // Simple speed limit so we don't drive too fast

  double tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
  double tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
  double ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
  double ta = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0);

  if (tv < 1.0)
  {
    m_LimelightHasValidTarget = false;
    m_LimelightDriveCommand = 0.0;
    m_LimelightSteerCommand = 0.0;
    return;
  }

  m_LimelightHasValidTarget = true;

  // Start with proportional steering
  double steer_cmd = tx * STEER_K;
  m_LimelightSteerCommand = steer_cmd;

  // try to drive forward until the target area reaches our desired area
  double drive_cmd = (DESIRED_TARGET_AREA - ta) * DRIVE_K;

  // don't let the robot drive too fast into the goal
  if (drive_cmd > MAX_DRIVE)
  {
    drive_cmd = MAX_DRIVE;
  }
  m_LimelightDriveCommand = drive_cmd;
}
  


  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}