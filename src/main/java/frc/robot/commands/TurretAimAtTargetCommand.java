// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.TurretSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TurretAimAtTargetCommand extends Command {
  /** Creates a new TurretAimAtTargetCommand. */
  private final CommandSwerveDrivetrain drivetrain;
  private final TurretSubsystem turret;
  private final Translation2d targetPosition;

  public TurretAimAtTargetCommand(CommandSwerveDrivetrain drivetrain, TurretSubsystem turret,
    Translation2d targetPosition) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drivetrain = drivetrain;
    this.turret = turret;
    this.targetPosition = targetPosition;

    addRequirements(turret);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    Pose2d robotPose = drivetrain.getPose();
    // Compute the turret angle needed to aim at the target position
    Rotation2d turretAngle = frc.robot.helpers.ShooterCalculator.turretAngleForFieldTarget(robotPose, targetPosition);
    SmartDashboard.putNumber("TargetTurretAngle", turretAngle.getDegrees());
    // Command the turret to the desired angle
    turret.setAngle(Degrees.of(turretAngle.getDegrees()));

    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
