package frc.robot

import edu.wpi.first.apriltag.AprilTagFieldLayout
import edu.wpi.first.apriltag.AprilTagFields
import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.units.Units.Volts
import edu.wpi.first.units.measure.Voltage
import edu.wpi.first.wpilibj2.command.CommandScheduler
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import frc.robot.subsystem.swerve.Drivebase
import lib.commands.not
import lib.math.units.measuredIn
import org.littletonrobotics.junction.LogFileUtil
import org.littletonrobotics.junction.LoggedRobot
import org.littletonrobotics.junction.Logger
import org.littletonrobotics.junction.networktables.NT4Publisher
import org.littletonrobotics.junction.wpilog.WPILOGReader
import org.littletonrobotics.junction.wpilog.WPILOGWriter
import kotlin.math.pow

object Robot : LoggedRobot() {
    val driverController = CommandXboxController(0)

    val field = AprilTagFieldLayout.loadField(AprilTagFields.k2024Crescendo)

    val drivebase =
        Drivebase().apply {
            defaultCommand =
                getDriveCommand(
                    { MathUtil.applyDeadband(-driverController.leftY, 0.05).pow(3) },
                    { MathUtil.applyDeadband(-driverController.leftX, 0.05).pow(3) },
                    { MathUtil.applyDeadband(-driverController.rightX, 0.05) },
                    !driverController.leftBumper(),
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
        println((12.0 measuredIn Volts) as Voltage)
    }

    private fun configureBindings() {
        driverController.y().whileTrue(drivebase.getSendToPoseCmd { Pose2d(2.0, 2.0, Rotation2d.kZero) })
        driverController.a().whileTrue(drivebase.getSendToPoseCmd { Pose2d(2.0, 8.0, Rotation2d.kZero) })
        driverController.x().whileTrue(drivebase.getSendToPoseCmd { Pose2d(8.0, 2.0, Rotation2d.kZero) })
        driverController.b().whileTrue(drivebase.getSendToPoseCmd { Pose2d(8.0, 8.0, Rotation2d.kZero) })
    }

    override fun robotPeriodic() {
        CommandScheduler.getInstance().run()
    }
}
