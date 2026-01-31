// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.logging.LogManager;

import org.tinylog.TaggedLogger;
import org.usfirst.frc3620.logger.LoggingMaster;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.datalog.StructLogEntry;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SetIMUFromMegaTag1Command extends Command {

  boolean resetIMU;
  TaggedLogger logger = LoggingMaster.getLogger(getClass());
  /** Creates a new SetIMUFromMegaTag1Command. */
  public SetIMUFromMegaTag1Command() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    resetIMU = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    int mostTargets = 0; 
    Pose2d pose = null; 
    String used = null; 
    for (var cameraData : RobotContainer.limelightSubsystem.getAllCameraData()) {
      if (cameraData.shouldUseThisCamera()) {
        PoseEstimate pe = cameraData.megaTag1.getPoseEstimate();
        if (pe !=null && pe.tagCount > mostTargets){
          mostTargets = pe.tagCount;
          pose = pe.pose;
          used = cameraData.getLimelightName();
        }
      }
    }
    if (pose != null) {
      if (RobotContainer.drivetrain != null){
        CommandSwerveDrivetrain sd = RobotContainer.drivetrain;
        Rotation3d r3d = new Rotation3d(0, 0, pose.getRotation().getRadians());

        Rotation2d before = r3d.toRotation2d(); // log info
        sd.getPigeon2().setYaw(pose.getRotation().getRadians());
        Rotation2d after = r3d.toRotation2d(); // log info
        logger.info("Swerve yaw changed from {} to {}", before.getDegrees(), after.getDegrees());
        resetIMU = true;
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return resetIMU;
  }

  @Override
  public boolean runsWhenDisabled(){
    return true;
  }
}
