// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.drive.*; 
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/**import static frc.robot.utility.Constants.Unit.*; */

public class DriveTrain extends SubsystemBase {
  private static final int LEFT_FRONT_MOTOR_PORT = 40;
  private static final int LEFT_BACK_MOTOR_PORT = 43;
  private static final int RIGHT_FRONT_MOTOR_PORT = 41;
  private static final int RIGHT_BACK_MOTOR_PORT = 42;

  private static final int LEFT_FRONT_ENCODER_ROTATOR_PORT = 50;
    private static final int LEFT_BACK_ENCODER_ROTATOR_PORT = 53;
    private static final int RIGHT_FRONT_ENCODER_ROTATOR_PORT = 51;
    private static final int RIGHT_BACK_ENCODER_ROTATOR_PORT = 52;

  /** Creates a new DriveTrain. */
  public DriveTrain() {}
  
  private final TalonFX[] motors = new TalonFX[] {
    new TalonFX(LEFT_FRONT_MOTOR_PORT),
    new TalonFX(LEFT_BACK_MOTOR_PORT),
    new TalonFX(RIGHT_FRONT_MOTOR_PORT),
    new TalonFX(RIGHT_BACK_MOTOR_PORT)
  };
  

  private final CANcoder[] encoders = new CANcoder[] {
    new CANcoder(LEFT_FRONT_ENCODER_ROTATOR_PORT),
    new CANcoder(LEFT_BACK_ENCODER_ROTATOR_PORT),
    new CANcoder(RIGHT_FRONT_ENCODER_ROTATOR_PORT),
    new CANcoder(RIGHT_BACK_ENCODER_ROTATOR_PORT)
};
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
