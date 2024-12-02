package lib.math

import edu.wpi.first.math.VecBuilder
import edu.wpi.first.math.Vector
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.numbers.N2

val Rotation2d.vector: Vector<N2>
    get() = VecBuilder.fill(cos, sin)

fun rotationFromVector(vec: Vector<N2?>): Rotation2d {
    return Rotation2d(vec[0], vec[1])
}

fun vectorFromRotation(rot: Rotation2d): Vector<N2> {
    return VecBuilder.fill(rot.cos, rot.sin)
}
