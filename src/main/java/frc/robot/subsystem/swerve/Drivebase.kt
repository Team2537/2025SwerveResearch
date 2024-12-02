package frc.robot.subsystem.swerve

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.kinematics.SwerveDriveKinematics
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.math.util.Units.inchesToMeters
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.RobotType
import frc.robot.subsystem.swerve.module.SwerveModule
import frc.robot.subsystems.swerve.gyro.GyroIO
import frc.robot.subsystems.swerve.gyro.GyroIOPigeon2
import frc.robot.subsystems.swerve.gyro.GyroIOSim
import lib.controllers.ControllerGains
import lib.controllers.pathfollowing.SimplePathFollower
import org.littletonrobotics.junction.Logger
import java.util.function.DoubleSupplier
import javax.naming.ldap.Control

class Drivebase : SubsystemBase("drivebase") {
    val moduleTranslations =
        arrayOf(
            Translation2d(moduleOffset, moduleOffset),
            Translation2d(moduleOffset, -moduleOffset),
            Translation2d(-moduleOffset, moduleOffset),
            Translation2d(-moduleOffset, -moduleOffset),
        )

    val modules: List<SwerveModule> =
        ModuleConfig.entries.mapIndexed { index, config ->
            SwerveModule(config, moduleTranslations[index])
        }

    val gyro: GyroIO =
        when (RobotType.robot) {
            RobotType.Robot.BOUNTY -> GyroIOPigeon2("rio", 13)
            RobotType.Robot.SIM -> GyroIOSim(::robotRelativeSpeeds)
            else -> object : GyroIO {}
        }

    val gyroInputs = GyroIO.GyroInputs()

    val kinematics: SwerveDriveKinematics =
        SwerveDriveKinematics(*moduleTranslations)

    val poseEstimator: SwerveDrivePoseEstimator =
        SwerveDrivePoseEstimator(kinematics, gyroInputs.yaw, modulePositions, Pose2d())

    val modulePositions: Array<SwerveModulePosition>
        get() = modules.map { it.position }.toTypedArray()

    val measuredModuleStates: Array<SwerveModuleState>
        get() = modules.map { it.state }.toTypedArray()

    val targetModuleStates = Array(4) { SwerveModuleState() }

    val robotRelativeSpeeds: ChassisSpeeds
        get() =
            kinematics.toChassisSpeeds(*measuredModuleStates)
    
    val pose: Pose2d
        get() = poseEstimator.estimatedPosition
    
    val autoController = SimplePathFollower(
        ControllerGains(),
        ControllerGains(),
        ControllerGains(),
        ::applyChassisSpeeds,
        ::pose
    )

    fun applyChassisSpeeds(speeds: ChassisSpeeds) {
        val moduleStates = kinematics.toSwerveModuleStates(speeds)

        modules.forEachIndexed { index, module ->
            module.apply(moduleStates[index])
            targetModuleStates[index] = moduleStates[index]
        }
    }

    fun robotRelativeDriveCommand(
        xVel: DoubleSupplier,
        yVel: DoubleSupplier,
        thetaVel: DoubleSupplier,
    ): Command {
        return this.run {
            applyChassisSpeeds(
                ChassisSpeeds(
                    xVel.asDouble * maxVelocity,
                    yVel.asDouble * maxVelocity,
                    thetaVel.asDouble * maxVelocity,
                ),
            )
        }
    }

    override fun periodic() {
        modules.forEachIndexed { index, module ->
            module.updateInputs()
            Logger.processInputs("Module [$index]", module.inputs)
        }

        gyro.updateInputs(gyroInputs)
        Logger.processInputs("Gyro", gyroInputs)

        poseEstimator.update(
            gyroInputs.yaw,
            modulePositions,
        )

        Logger.recordOutput("$name/targetModuleStates", *targetModuleStates)
        Logger.recordOutput("$name/measuredModuleStates", *measuredModuleStates)
        Logger.recordOutput("$name/pose", Pose2d.struct, poseEstimator.estimatedPosition)
        Logger.recordOutput("$name/robotRelativeSpeeds", robotRelativeSpeeds)
    }

    companion object {
        val moduleOffset: Double =
            when (RobotType.robot) {
                else -> inchesToMeters(9.7859)
            }

        val maxVelocity = 3.0

        /**
         * Enum for storing the configuration of each swerve module.
         *
         * @param driveID The ID of the drive motor.
         * @param turnID The ID of the turn motor.
         * @param encoderID The ID of the encoder.
         * @param encoderOffset The offset of the encoder.
         * @param driveInverted Whether the drive motor is inverted.
         * @param turnInverted Whether the turn motor is inverted.
         */
        enum class ModuleConfig(
            val driveID: Int,
            val turnID: Int,
            val encoderID: Int,
            val encoderOffset: Double,
            val driveInverted: Boolean,
            val turnInverted: Boolean,
        ) {
            FRONT_LEFT(1, 2, 3, 0.0, false, false),
            FRONT_RIGHT(4, 5, 6, 0.0, false, false),
            BACK_LEFT(7, 8, 9, 0.0, false, false),
            BACK_RIGHT(10, 11, 12, 0.0, false, false),
        }

        /**
         * Enum for storing the gearings of different kinds of swerve modules.
         *
         * @param drive The drive gearing.
         * @param steer The steer gearing.
         */
        enum class ModuleGearings(val drive: Double, val steer: Double) {
            MK4I_L2(6.75, 150.0 / 7.0),
        }
    }
}
