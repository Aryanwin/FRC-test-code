// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.sim.CommandBase;
import frc.robot.sim.TimedRobot;
import frc.robot.sim.WPI_TalonFX;
import frc.robot.sim.graphics.XboxController;

/** Add your docs here. */
public class TankControlCommand{
    private TimedRobot robot;

    new Command(this,xbox).schedule(this);
    WPI_TalonFX motor0 = new WPI_TalonFX(0,robot);
    WPI_TalonFX motor1 = new WPI_TalonFX(0,robot);
    WPI_TalonFX motor2 = new WPI_TalonFX(0,robot);
    WPI_TalonFX motor3 = new WPI_TalonFX(0,robot);

}
