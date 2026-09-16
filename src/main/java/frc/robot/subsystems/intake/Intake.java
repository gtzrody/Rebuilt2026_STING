package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

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

  private final CANSparkMax leaderMotor = new CANSparkMax(LEADER_MOTOR_ID, MotorType.kBrushless);
  private final CANSparkMax followerMotor = new CANSparkMax(FOLLOWER_MOTOR_ID, MotorType.kBrushless);

  // Updated simulation to simulate a gearbox with 2 NEOs
  private final DCMotorSim simulation = new DCMotorSim(
      LinearSystemId.createDCMotorSystem(
          DCMotor.getNEO(2), 
          0.001,
          36.0 / 25.0),
      DCMotor.getNEO(2));

  public Intake() {
    leaderMotor.restoreFactoryDefaults();
    followerMotor.restoreFactoryDefaults();

    leaderMotor.setInverted(true); 
    leaderMotor.setIdleMode(IdleMode.kBrake);
    leaderMotor.setSmartCurrentLimit(90);
    leaderMotor.setOpenLoopRampRate(0.25);

    followerMotor.setIdleMode(IdleMode.kBrake);
    followerMotor.setSmartCurrentLimit(90);
    

    followerMotor.follow(leaderMotor, false);

    // Save configurations to the SPARK MAX flash memory
    leaderMotor.burnFlash(); 
    followerMotor.burnFlash(); 
  }

  /**
   * Run the intake roller.
   *
   * @param output motor output from -1 to +1
   */
  public Command set(double output) {
    return run(() -> {
      double clamped = Math.max(-1.0, Math.min(1.0, output));
      // You only need to set the leader; the follower will automatically copy it
      leaderMotor.set(clamped);
    });
  }

  /** Stop the intake immediately. */
  public void stop() {
    leaderMotor.stopMotor();
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