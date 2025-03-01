// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class Elevator extends SubsystemBase {
  //No intentes entender esto aza, solo cambia las constantes si quieres cambiar la altura
  /** Creates a new Elevator. */
  private PeriodicIO mPeriodicIO;
  private static Elevator mInstance;

  public static Elevator getInstance() {
    if (mInstance == null) {
      mInstance = new Elevator();
    }
    return mInstance;
  }

  private SparkMax mLeftMotor;
  private RelativeEncoder mLeftEncoder;
  private SparkClosedLoopController mLeftPIDController;

  private SparkMax mRightMotor;

  private TrapezoidProfile mProfile;
  private TrapezoidProfile.State mCurState = new TrapezoidProfile.State();
  private TrapezoidProfile.State mGoalState = new TrapezoidProfile.State();
  private double prevUpdateTime = Timer.getFPGATimestamp();

  public Elevator() {
    super("Elevator");
    mPeriodicIO = new PeriodicIO();
    SparkMaxConfig elevatorConfig = new SparkMaxConfig();

    elevatorConfig.closedLoop
        .pid(ElevatorConstants.k_elevatorKP, ElevatorConstants.k_elevatorKD, ElevatorConstants.k_elevatorKI)
        .iZone(ElevatorConstants.k_elevatorKIZone);

    elevatorConfig.smartCurrentLimit(ElevatorConstants.k_supplyLimit);

    elevatorConfig.idleMode(IdleMode.kBrake);
    elevatorConfig.limitSwitch.reverseLimitSwitchEnabled(true);

    // LEFT ELEVATOR MOTOR
    mLeftMotor = new SparkMax(ElevatorConstants.k_leftMotor, MotorType.kBrushless);
    mLeftEncoder = mLeftMotor.getEncoder();
    mLeftPIDController = mLeftMotor.getClosedLoopController();
    mLeftMotor.configure(elevatorConfig, com.revrobotics.spark.SparkBase.ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // RIGHT ELEVATOR MOTOR
    mRightMotor = new SparkMax(ElevatorConstants.k_rightMotor, MotorType.kBrushless);
    mRightMotor.configure(elevatorConfig.follow(mLeftMotor, true), com.revrobotics.spark.SparkBase.ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    mProfile = new TrapezoidProfile(
        new TrapezoidProfile.Constraints(
            ElevatorConstants.k_maxVelocity,
            ElevatorConstants.k_maxAcceleration));
  }
  public enum ElevatorState {
    NONE,
    STOW,
    L2,
    L3,
    L4,
    A1,
    A2
  }

  private static class PeriodicIO {
    double elevator_target = 0.0;
    double elevator_power = 0.0;

    boolean is_elevator_pos_control = false;

    ElevatorState state = ElevatorState.STOW;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
       double curTime = Timer.getFPGATimestamp();
      double dt = curTime - prevUpdateTime;
      prevUpdateTime = curTime;
      if (mPeriodicIO.is_elevator_pos_control) {
        // Update goal
        mGoalState.position = mPeriodicIO.elevator_target;
  
        // Calculate new state
        prevUpdateTime = curTime;
        mCurState = mProfile.calculate(dt, mCurState, mGoalState);
  
        // Set PID controller to new state
        mLeftPIDController.setReference(
            mCurState.position,
            SparkBase.ControlType.kPosition,
            ClosedLoopSlot.kSlot0,
            ElevatorConstants.kG,
            ArbFFUnits.kVoltage);
      } else {
        mCurState.position = mLeftEncoder.getPosition();
        mCurState.velocity = 0;
        mLeftMotor.set(mPeriodicIO.elevator_power);}
      
      SmartDashboard.putNumber("Elevador", mLeftEncoder.getPosition());
  }

  //ni puta idea que hace lo de abajo
   public void writePeriodicOutputs() {
    double curTime = Timer.getFPGATimestamp();
    double dt = curTime - prevUpdateTime;
    prevUpdateTime = curTime;
    if (mPeriodicIO.is_elevator_pos_control) {
      // Update goal
      mGoalState.position = mPeriodicIO.elevator_target;

      // Calculate new state
      prevUpdateTime = curTime;
      mCurState = mProfile.calculate(dt, mCurState, mGoalState);

      // Set PID controller to new state
      mLeftPIDController.setReference(
          mCurState.position,
          SparkBase.ControlType.kPosition,
          ClosedLoopSlot.kSlot0,
          ElevatorConstants.kG,
          ArbFFUnits.kVoltage);
    } else {
      mCurState.position = mLeftEncoder.getPosition();
      mCurState.velocity = 0;
      mLeftMotor.set(mPeriodicIO.elevator_power);
    }
  }

  public void stop() {
    mPeriodicIO.is_elevator_pos_control = false;
    mPeriodicIO.elevator_power = 0.0;

    mLeftMotor.set(0.0);
  }

  public ElevatorState getState() {
    return mPeriodicIO.state;
  }

  public void setElevatorPower(double power) {
    mPeriodicIO.is_elevator_pos_control = false;
    mPeriodicIO.elevator_power = power;
  }

  public void goToElevatorStow() {
    mPeriodicIO.is_elevator_pos_control = true;
    mPeriodicIO.elevator_target = ElevatorConstants.kStowHeight;
    mPeriodicIO.state = ElevatorState.STOW;
  }

  public void goToElevatorL2() {
    mPeriodicIO.is_elevator_pos_control = true;
    mPeriodicIO.elevator_target = ElevatorConstants.kL2Height;
    mPeriodicIO.state = ElevatorState.L2;
  }

  public void goToElevatorL3() {
    mPeriodicIO.is_elevator_pos_control = true;
    mPeriodicIO.elevator_target = ElevatorConstants.kL3Height;
    mPeriodicIO.state = ElevatorState.L3;
  }

  public void goToElevatorL4() {
    mPeriodicIO.is_elevator_pos_control = true;
    mPeriodicIO.elevator_target = ElevatorConstants.kL4Height;
    mPeriodicIO.state = ElevatorState.L4;
  }

  public void goToAlgaeLow() {
    mPeriodicIO.is_elevator_pos_control = true;
    mPeriodicIO.elevator_target = ElevatorConstants.kLowAlgaeHeight;
    mPeriodicIO.state = ElevatorState.A1;
  }

  public void goToAlgaeHigh() {
    mPeriodicIO.is_elevator_pos_control = true;
    mPeriodicIO.elevator_target = ElevatorConstants.kHighAlgaeHeight;
    mPeriodicIO.state = ElevatorState.A2;
  }

  
}
