package frc.robot.subsystems.intake;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  private static final int LEADER_MOTOR_ID = 19;
  private static final int FOLLOWER_MOTOR_ID = 20;

  private final SparkMax leaderMotor = new SparkMax(LEADER_MOTOR_ID, MotorType.kBrushless);
  private final SparkMax followerMotor = new SparkMax(FOLLOWER_MOTOR_ID, MotorType.kBrushless);

  private final DCMotorSim simulation = new DCMotorSim(
      LinearSystemId.createDCMotorSystem(
          DCMotor.getNEO(2), 
          0.001,
          36.0 / 25.0),
      DCMotor.getNEO(2));

  public Intake() {
    SparkMaxConfig leaderConfig = new SparkMaxConfig();
    SparkMaxConfig followerConfig = new SparkMaxConfig();

    leaderConfig
        .inverted(true)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(90);
    leaderConfig.openLoopRampRate(0.25);

    followerConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(90)
        .follow(LEADER_MOTOR_ID, true);

    leaderMotor.configure(leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    followerMotor.configure(followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public Command set(double output) {
    return run(() -> {
      double clamped = Math.max(-1.0, Math.min(1.0, output));
      leaderMotor.set(clamped);
    });
  }


  public double getOutput() {
    return leaderMotor.getAppliedOutput();
  }

  public double getRollerRPM() {
    return leaderMotor.getEncoder().getVelocity() / (36.0 / 25.0);
  }

  @Override
  public void simulationPeriodic() {
    if (!RobotBase.isSimulation()) {
      return;
    }

    double simVoltage = leaderMotor.getAppliedOutput() * RobotController.getBatteryVoltage();
    simulation.setInputVoltage(simVoltage);
    simulation.update(0.020);
  }
}