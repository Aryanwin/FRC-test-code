// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderStatusFrame;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

import static frc.robot.utility.Constants.Unit.*;

public class DriveTrain extends SubsystemBase {
  private static final int LEFT_FRONT_MOTOR_PORT = 40;
  private static final int LEFT_BACK_MOTOR_PORT = 43;
  private static final int RIGHT_FRONT_MOTOR_PORT = 41;
  private static final int RIGHT_BACK_MOTOR_PORT = 42;

  /** Creates a new DriveTrain. */
  public DriveTrain() {}
  
  private final WPI_TalonFX[] motors = new WPI_TalonFX[] {
    new WPI_TalonFX(LEFT_FRONT_MOTOR_PORT),
    new WPI_TalonFX(LEFT_BACK_MOTOR_PORT),
    new WPI_TalonFX(RIGHT_FRONT_MOTOR_PORT),
    new WPI_TalonFX(RIGHT_BACK_MOTOR_PORT)
  };
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
