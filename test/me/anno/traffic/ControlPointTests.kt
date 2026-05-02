package me.anno.traffic

import me.anno.maths.Maths.PIf
import me.anno.traffic.utils.SplineMaths.computeControlPoint
import me.anno.utils.assertions.assertEquals
import me.anno.utils.assertions.assertTrue
import org.joml.Quaternionf
import org.joml.Vector3d
import org.junit.jupiter.api.Test
import java.util.*

class ControlPointTests {

    @Test
    fun testStraightLineForward() {
        val p0 = Vector3d(0.0, 0.0, 0.0)
        val p2 = Vector3d(0.0, 0.0, 10.0)

        val q0 = Quaternionf()
        val q2 = Quaternionf() // same direction

        val p1 = computeControlPoint(p0, q0, p2, q2)
        assertEquals(Vector3d(0.0, 0.0, 5.0), p1, 1e-10)
    }

    @Test
    fun testRightAngleTurn() {
        val p0 = Vector3d(0.0, 0.0, 0.0)
        val p2 = Vector3d(10.0, 0.0, 10.0)

        val q0 = Quaternionf() // forward Z
        val q2 = Quaternionf().rotateY(-PIf * 0.5f) // forward X

        val p1 = computeControlPoint(p0, q0, p2, q2)
        assertEquals(Vector3d(0.0, 0.0, 10.0), p1, 1e-10)
    }

    @Test
    fun testRandomizedStability() {
        val rand = Random(1)

        repeat(200) {
            val p0 = Vector3d(
                rand.nextGaussian(),
                rand.nextGaussian(),
                rand.nextGaussian()
            ).mul(10.0)

            val p2 = Vector3d(
                rand.nextGaussian(),
                rand.nextGaussian(),
                rand.nextGaussian()
            ).mul(10.0)

            val q0 = Quaternionf().rotateYXZ(
                rand.nextFloat(),
                rand.nextFloat(),
                rand.nextFloat()
            )

            val q2 = Quaternionf().rotateYXZ(
                rand.nextFloat(),
                rand.nextFloat(),
                rand.nextFloat()
            )

            val p1 = computeControlPoint(p0, q0, p2, q2)
            assertTrue(p1.isFinite)
            assertTrue(p1.distance(p0) < 1e4)
            assertTrue(p1.distance(p2) < 1e4)
        }
    }
}