// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Seconds;

import java.util.function.Supplier;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.PivotConfig;
import yams.mechanisms.positional.Pivot;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.remote.TalonFXWrapper;

public class TurretSubsystem extends SubsystemBase {
  TalonFX turretMotor = new TalonFX(20);

  SmartMotorControllerConfig motorConfig = new SmartMotorControllerConfig(this)
      .withControlMode(ControlMode.CLOSED_LOOP)
      .withClosedLoopController(4, 0, 0, DegreesPerSecond.of(360), DegreesPerSecondPerSecond.of(180))
      // Configure Motor and Mechanism properties
      .withGearing(new MechanismGearing(GearBox.fromReductionStages(1,50)))
      .withIdleMode(MotorMode.BRAKE)
      .withMotorInverted(false)
      // Setup Telemetry
      .withTelemetry("TurretMotor", TelemetryVerbosity.HIGH)
      // Power Optimization
      .withStatorCurrentLimit(Amps.of(40))
      .withClosedLoopRampRate(Seconds.of(0.25))
      .withOpenLoopRampRate(Seconds.of(0.25));

  SmartMotorController turret = new TalonFXWrapper(turretMotor,
      DCMotor.getKrakenX60(1),
      motorConfig);

  PivotConfig turret_config = new PivotConfig(turret)
      .withStartingPosition(Degrees.of(0)) // Starting position of the Pivot
      .withHardLimit(Degrees.of(-135), Degrees.of(135)) // Hard limit bc wiring prevents infinite spinning
      .withSoftLimits(Degrees.of(-130), Degrees.of(130))
      .withTelemetry("PivotExample", TelemetryVerbosity.HIGH) // Telemetry
      .withMOI(Meters.of(0.25), Pounds.of(2)); // MOI Calculation

  private Pivot turretPivot = new Pivot(turret_config);


  /** Creates a new TurretSubsystem. */
  public TurretSubsystem() {
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    turretPivot.updateTelemetry();
  }

  @Override
  public void simulationPeriodic(){
    turretPivot.simIterate();
  }

  public Command setAngle(Angle angle) {
    return turretPivot
        .setAngle(() -> angle)
        .withName("TurretSetAngle(" + angle.in(Degrees) + ")");

  }

  public Command aimAtFieldTargetCommand(
      CommandSwerveDrivetrain drivetrain,
      Translation2d targetPosition) {

    Supplier<Angle> angleSupplier = () -> {
      Pose2d robotPose = drivetrain.getPose();
      Rotation2d turretAngle = frc.robot.helpers.ShooterCalculator.turretAngleForFieldTarget(robotPose, targetPosition);
      SmartDashboard.putNumber("TargetTurretAngle", turretAngle.getDegrees());
      return Degrees.of(turretAngle.getDegrees());
    };

    return turretPivot
        .setAngle(angleSupplier)
        .withName("TurretAimAtFieldTarget");

  }
}
