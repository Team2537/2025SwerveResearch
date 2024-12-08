package frc.robot

import edu.wpi.first.math.MathUtil
import edu.wpi.first.wpilibj2.command.CommandScheduler
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import frc.robot.commands.swerve.AccelerationControlledDriveCommand
import frc.robot.subsystem.swerve.Drivebase
import org.littletonrobotics.junction.LogFileUtil
import org.littletonrobotics.junction.LoggedRobot
import org.littletonrobotics.junction.Logger
import org.littletonrobotics.junction.networktables.NT4Publisher
import org.littletonrobotics.junction.wpilog.WPILOGReader
import org.littletonrobotics.junction.wpilog.WPILOGWriter
import kotlin.math.pow

object Robot : LoggedRobot() {
    val driverController = CommandXboxController(0)

    val drivebase =
        Drivebase().apply {
            defaultCommand =
                AccelerationControlledDriveCommand(
                    this,
                    { -MathUtil.applyDeadband(driverController.leftY, 0.05).pow(3) * Drivebase.maxVelocity },
                    { -MathUtil.applyDeadband(driverController.leftX, 0.05).pow(3) * Drivebase.maxVelocity },
                    { -MathUtil.applyDeadband(driverController.rightX, 0.05) * 2 * Math.PI },
                    { false },
                )
        }

    init {
        Logger.recordMetadata("RobotType", RobotType.robot.toString())
        when (RobotType.robot) {
            RobotType.Robot.BOUNTY -> {
                Logger.addDataReceiver(WPILOGWriter())
                Logger.addDataReceiver(NT4Publisher())
            }
            RobotType.Robot.SIM -> {
                // Initialize the simulation here
                Logger.addDataReceiver(WPILOGWriter())
                Logger.addDataReceiver(NT4Publisher())
            }
            RobotType.Robot.REPLAY -> {
                val file = LogFileUtil.findReplayLog()
                Logger.setReplaySource(WPILOGReader(file))

                Logger.addDataReceiver(WPILOGWriter(LogFileUtil.addPathSuffix(file, "_replay")))
                Logger.addDataReceiver(NT4Publisher())
            }
        }

        Logger.start()

        configureBindings()
    }

    private fun configureBindings() {
    }

    override fun robotPeriodic() {
        CommandScheduler.getInstance().run()
    }
}
