/* (C)2026 */
package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.Supplier;
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
import yams.units.EasyCRT;
import yams.units.EasyCRTConfig;

public class TurretSubsystem extends SubsystemBase {

  /** Manually rerun CRT seeding. */
  private static final String RERUN_SEED = "Turret/CRT/RerunSeed";

  private final CANcoder absA = new CANcoder(31);
  private final CANcoder absB = new CANcoder(32);
  private final Pivot turretPivot;
  private final EasyCRTConfig easyCrtConfig;
  private final SmartMotorController turret;
  private boolean rotorSeededFromAbs = false;
  private double lastSeededTurretDeg = Double.NaN;
  private double lastSeedError = Double.NaN;
  private double lastAbsA = Double.NaN;
  private double lastAbsB = Double.NaN;
  private String lastSeedStatus = "NOT_ATTEMPTED";

  private Timer startUpTimer = new Timer();
  private boolean startTimer = false;
  private boolean delayForCRTDone = false;

  TalonFX turretMotor = new TalonFX(20);

  /** Creates a new TurretSubsystem. */
  public TurretSubsystem() {

    SmartMotorControllerConfig motorConfig =
        new SmartMotorControllerConfig(this)
            .withControlMode(ControlMode.CLOSED_LOOP)
            .withClosedLoopController(
                4, 0, 0, DegreesPerSecond.of(360), DegreesPerSecondPerSecond.of(180))
            // Configure Motor and Mechanism properties
            .withGearing(new MechanismGearing(GearBox.fromReductionStages(1, 50)))
            .withIdleMode(MotorMode.BRAKE)
            .withMotorInverted(false)
            // Setup Telemetry
            .withTelemetry("TurretMotor", TelemetryVerbosity.HIGH)
            // Power Optimization
            .withStatorCurrentLimit(Amps.of(40))
            .withClosedLoopRampRate(Seconds.of(0.25))
            .withOpenLoopRampRate(Seconds.of(0.25));

    turret = new TalonFXWrapper(turretMotor, DCMotor.getKrakenX60(1), motorConfig);

    PivotConfig turret_config =
        new PivotConfig(turret)
            .withStartingPosition(Degrees.of(0)) // Starting position of the Pivot
            .withHardLimit(
                Degrees.of(-135),
                Degrees.of(135)) // Hard limit bc wiring prevents infinite spinning
            .withSoftLimits(Degrees.of(-130), Degrees.of(130))
            .withTelemetry("PivotExample", TelemetryVerbosity.HIGH) // Telemetry
            .withMOI(Meters.of(0.25), Pounds.of(2)); // MOI Calculation

    turretPivot = new Pivot(turret_config);
    easyCrtConfig = buildEasyCRTConfig();
    logCrtConfigTelemetry();
    SmartDashboard.putBoolean(RERUN_SEED, false);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    if (!startTimer) {
      startUpTimer.reset();
      startUpTimer.start();
      startTimer = true;
    }

    if (startUpTimer.hasElapsed(5)) {
      delayForCRTDone = true;
    }

    if (SmartDashboard.getBoolean(RERUN_SEED, false) && delayForCRTDone) {
      SmartDashboard.putBoolean(RERUN_SEED, false);
      rerunCrtSeed();
    }
    SmartDashboard.putNumber(
        "Turret/CRT/CurrentPositionDeg", turret.getMechanismPosition().in(Degrees));
    if (!rotorSeededFromAbs && delayForCRTDone) {
      attemptRotorSeedFromCANCoders();
    }
    turret.updateTelemetry();

    SmartDashboard.putNumber("Encoder A Raw", absA.getAbsolutePosition().getValueAsDouble());
    SmartDashboard.putNumber("Encoder B", absB.getPosition().getValueAsDouble());
    SmartDashboard.putBoolean("Encoder A Raw", rotorSeededFromAbs);
    SmartDashboard.putNumber("Position", getAngle().in(Rotations));
  }

  @Override
  public void simulationPeriodic() {
    turretPivot.simIterate();
  }

  /** Returns the current angle of the turret */
  public Angle getAngle() {
    return turretPivot.getAngle();
  }

  public Command setAngle(Angle angle) {
    return turretPivot.setAngle(() -> angle).withName("TurretSetAngle(" + angle.in(Degrees) + ")");
  }

  public Command aimAtFieldTargetCommand(
      CommandSwerveDrivetrain drivetrain, Translation2d targetPosition) {

    Supplier<Angle> angleSupplier =
        () -> {
          Pose2d robotPose = drivetrain.getPose();
          Rotation2d turretAngle =
              frc.robot.helpers.ShooterCalculator.turretAngleForFieldTarget(
                  robotPose, targetPosition);
          SmartDashboard.putNumber("TargetTurretAngle", turretAngle.getDegrees());
          return Degrees.of(turretAngle.getDegrees());
        };

    return turretPivot.setAngle(angleSupplier).withName("TurretAimAtFieldTarget");
  }

  /** Forces a CRT reseed attempt */
  public void rerunCrtSeed() {
    rotorSeededFromAbs = false;
    SmartDashboard.putNumber("Turret/CRT/ManualRerunTimestampSec", Timer.getFPGATimestamp());
    attemptRotorSeedFromCANCoders();
  }

  private AbsSensorRead readAbsSensors() {
    boolean haveDevices = absA.isConnected() && absB.isConnected();

    if (haveDevices) {
      return new AbsSensorRead(
          true,
          absA.getAbsolutePosition().getValueAsDouble(),
          absB.getAbsolutePosition().getValueAsDouble(),
          "OK");
    }
    return new AbsSensorRead(false, Double.NaN, Double.NaN, "NO_DEVICES");
  }

  private EasyCRTConfig buildEasyCRTConfig() {
    return new EasyCRTConfig(
            () -> absA.getAbsolutePosition().getValue(),
            () -> absB.getAbsolutePosition().getValue())
        // .withCommonDriveGear(1, 95, 19, 21)
        .withAbsoluteEncoder1GearingStages(140, 18)
        .withAbsoluteEncoder2GearingStages(140, 18, 24, 23)
        .withAbsoluteEncoderOffsets(Rotations.of(0), Rotations.of(0))
        .withMechanismRange(
            Rotations.of(-0.4166667), Rotations.of(0.4166667)) // 150 degrees total range
        .withMatchTolerance(Rotations.of(0.03))
        .withCrtGearRecommendationInputs(24, 140 / 18.0)
        .withCrtGearRecommendationConstraints(1.1, 11, 60, 40);
  }

  /**
   * Tries to solve turret position via CRT and seed the relative encoder with the result. Reads
   * both CANCoder values, runs the solver, updates the SmartMotorController encoder, and publishes
   * CRT status to the dashboard.
   */
  private void attemptRotorSeedFromCANCoders() {
    AbsSensorRead absRead = readAbsSensors();
    if (!absRead.ok()) {
      if (!"NO_DEVICES".equals(absRead.status())) {
        SmartDashboard.putString("Turret/CRT/SeedStatus", absRead.status());
      }
      lastSeedStatus = absRead.status();
      return;
    }

    double absA = absRead.absA();
    double absB = absRead.absB();
    lastAbsA = absA;
    lastAbsB = absB;

    var solver = new EasyCRT(easyCrtConfig);
    var solvedAngle = solver.getAngleOptional();

    SmartDashboard.putNumber("Turret/CRT/AbsA", absA);
    SmartDashboard.putNumber("Turret/CRT/AbsB", absB);
    SmartDashboard.putString("Turret/CRT/SolverStatus", solver.getLastStatus());
    SmartDashboard.putNumber("Turret/CRT/SolverErrorRot", solver.getLastErrorRotations());
    SmartDashboard.putNumber("Turret/CRT/SolverIterations", solver.getLastIterations());

    if (solvedAngle.isEmpty()) {
      SmartDashboard.putBoolean("Turret/CRT/SolutionFound", false);
      lastSeedStatus = solver.getLastStatus();
      return;
    }

    Angle turretRotations = solvedAngle.get();
    turret.setEncoderPosition(turretRotations);
    rotorSeededFromAbs = true;
    lastSeededTurretDeg = turretRotations.in(Degrees);
    lastSeedError = solver.getLastErrorRotations();
    SmartDashboard.putBoolean("Turret/CRT/SolutionFound", true);
    SmartDashboard.putNumber("Turret/CRT/SeededTurretDeg", lastSeededTurretDeg);
    SmartDashboard.putNumber("Turret/CRT/MatchErrorRot", lastSeedError);

    lastSeedStatus = "OK";
    SmartDashboard.putString("Turret/CRT/SeedStatus", lastSeedStatus);
    SmartDashboard.putBoolean("Turret/CRT/Seeded", rotorSeededFromAbs);
  }

  private void logCrtConfigTelemetry() {
    double mechanismRangeRot = easyCrtConfig.getMechanismRange().in(Rotations);
    double uniqueCoverageRot =
        easyCrtConfig.getUniqueCoverage().map(angle -> angle.in(Rotations)).orElse(Double.NaN);
    SmartDashboard.putNumber(
        "Turret/CRT/Config/RatioA", easyCrtConfig.getEncoder1RotationsPerMechanismRotation());
    SmartDashboard.putNumber(
        "Turret/CRT/Config/RatioB", easyCrtConfig.getEncoder2RotationsPerMechanismRotation());
    SmartDashboard.putNumber("Turret/CRT/Config/UniqueCoverageRot", uniqueCoverageRot);
    SmartDashboard.putBoolean(
        "Turret/CRT/Config/CoverageSatisfiesRange", easyCrtConfig.coverageSatisfiesRange());
    SmartDashboard.putNumber("Turret/CRT/Config/RequiredRangeRot", mechanismRangeRot);

    var configPair = easyCrtConfig.getRecommendedCrtGearPair();
    SmartDashboard.putBoolean("Turret/CRT/Config/RecommendedPairFound", configPair.isPresent());
    if (configPair.isPresent()) {
      var pair = configPair.get();
      SmartDashboard.putNumber("Turret/CRT/Config/Reccomender/RecommendedGearA", pair.gearA());
      SmartDashboard.putNumber("Turret/CRT/Config/Reccomender/RecommendedGearB", pair.gearB());
      SmartDashboard.putNumber(
          "Turret/CRT/Config/Reccomender/RecommendedCoverageRot", pair.coverage().in(Rotations));
      SmartDashboard.putNumber("Turret/CRT/Config/Reccomender/RecommendedLcm", pair.lcm());
      SmartDashboard.putBoolean(
          "Turret/CRT/Config/Reccomender/RecommendedCoprime",
          EasyCRTConfig.isCoprime(pair.gearA(), pair.gearB()));
      SmartDashboard.putNumber(
          "Turret/CRT/Config/Reccomender/RecommendedIterations", pair.theoreticalIterations());
    }
  }

  private static record AbsSensorRead(boolean ok, double absA, double absB, String status) {}
}
