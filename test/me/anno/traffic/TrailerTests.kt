package me.anno.traffic

import me.anno.traffic.VehicleTest.Companion.createStraight
import me.anno.traffic.utils.f2
import me.anno.utils.assertions.assertEquals
import me.anno.utils.assertions.assertTrue
import me.anno.utils.types.Floats.f2
import org.joml.Quaternionf
import org.joml.Vector3d
import org.junit.jupiter.api.Test
import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.atan2

class TrailerTests {
    @Test
    fun testTrailerMaintainsDistance() {
        val straight = createStraight(500.0)
        val engine = Vehicle()
        val trailer = Vehicle()
        engine.route.add(straight)

        engine.maxVelocity = 10f
        trailer.maxVelocity = 10f

        trailer.maxAcceleration *= 2f
        trailer.maxDeceleration *= 2f

        engine.attachTrailer(trailer, 2f, 2f)
        println(engine.position.distance(trailer.position))

        val dt = 0.1f
        val vs = listOf(engine, trailer)
        repeat(200) {
            println("\n--------------------")
            vs.update(dt)

            val pos = trailer.calculateTrailingPosition(trailer.linkToEngine!!)
            val dist = pos.distance(trailer.position)
            println("dist: ${dist.f2()}, vel: ${engine.velocity.length().f2()}, ${trailer.velocity.length().f2()}")
            println("  pos: ${engine.position.f2()} vs ${trailer.position.f2()}, target: ${pos.f2()}, ${engine.rotation.f2()} vs ${trailer.rotation.f2()}")

            // todo it would be nice, if we could be stricter...
            assertEquals(0.0, dist, 0.5, "Link distance should remain stable [$it]")
        }
    }

    @Test
    fun testTrailerDoesntDrivingWithoutEngine() {
        val engine = Vehicle()
        val trailer = Vehicle()
        engine.route.add(createStraight())
        engine.attachTrailer(trailer, 2f, 2f)

        engine.maxVelocity = 0f

        val dt = 0.1f
        val vs = listOf(engine, trailer)
        repeat(50) {
            vs.update(dt)
        }

        assertTrue(trailer.velocity.length() < 0.01f, "Trailer should not move on its own")
    }

    @Test
    fun testTrailerFollowsEngineMotion() {
        val engine = Vehicle()
        val trailer = Vehicle()
        engine.route.add(createStraight(200.0))

        engine.attachTrailer(trailer, 2f, 2f)

        engine.velocity.set(0f, 0f, 8f)

        val dt = 0.1f
        val vs = listOf(engine, trailer)
        repeat(20) {
            vs.update(dt)

            println("follows[$it]: ${engine.position.f2()} -> ${trailer.position.f2()}, ${trailer.velocity.f2()}")
        }

        assertTrue(trailer.velocity.z > 0.5f, "Trailer should be pulled forward")
        assertTrue(trailer.position.z > 0.0)
    }

    @Test
    fun testTrailerAlignsWithMotionDirection() {
        val engine = Vehicle()
        val trailer = Vehicle()
        engine.route.add(createStraight())

        engine.attachTrailer(trailer, 2f, 2f)

        // Start misaligned
        trailer.rotation.rotationY((PI * 0.5).toFloat())

        engine.velocity.set(0f, 0f, 10f)

        val dt = 0.05f
        val vs = listOf(engine, trailer)
        repeat(200) {
            vs.update(dt)
        }

        val forward = trailer.rotation.transform(Vector3d(0.0, 0.0, 1.0))
        assertTrue(forward.z > 0.7, "Trailer should align with pull direction")
    }

    @Test
    fun testHighSpeedStability() {
        val engine = Vehicle()
        val trailer = Vehicle()
        engine.route.add(createStraight())

        engine.attachTrailer(trailer, 2f, 2f)

        engine.velocity.set(0f, 0f, 40f)

        val dt = 0.02f
        val vs = listOf(engine, trailer)
        repeat(300) {
            vs.update(dt)

            assertTrue(engine.position.isFinite)
            assertTrue(trailer.position.isFinite)
            assertTrue(engine.velocity.isFinite)
            assertTrue(trailer.velocity.isFinite)
        }
    }

    @Test
    fun testTrailerFollowsCurve() {
        val p0 = LanePoint(Vector3d(0.0, 0.0, 0.0), Quaternionf(), 0.0, 0.0)
        val p1 = LanePoint(Vector3d(0.0, 0.0, 50.0), Quaternionf().rotationY((PI * 0.25).toFloat()), 0.0, 0.0)
        val p2 = LanePoint(Vector3d(50.0, 0.0, 50.0), Quaternionf().rotationY((PI * 0.5).toFloat()), 0.0, 0.0)
        val lane = Lane(p0, p1, p2)

        val engine = Vehicle()
        val trailer = Vehicle()

        engine.attachTrailer(trailer, 2f, 2f)

        engine.route.add(lane)
        engine.maxVelocity = 10f

        val dt = 0.1f
        val vs = listOf(engine, trailer)
        repeat(200) {
            vs.update(dt)

            if (it % 5 == 0) {
                val engineForward = engine.rotation.transform(Vector3d(0.0, 0.0, 1.0))
                val trailerForward = trailer.rotation.transform(Vector3d(0.0, 0.0, 1.0))
                println("forward[$it]: ${engineForward.f2()}, ${trailerForward.f2()}, positions: ${engine.position.f2()} vs ${trailer.position.f2()}")
            }
        }

        val forward = trailer.rotation.transform(Vector3d(0.0, 0.0, 1.0))
        assertTrue(forward.x > 0.5) {
            "Trailer should follow curve direction, forward: $forward"
        }
    }

    @Test
    fun testMultiTrailerChainStability() {
        val engine = Vehicle()
        val t1 = Vehicle()
        val t2 = Vehicle()

        engine.route.add(createStraight())

        engine.attachTrailer(t1, 2f, 2f)
        t1.attachTrailer(t2, 2f, 2f)

        engine.velocity.set(0f, 0f, 12f)

        val dt = 0.05f
        val vs = listOf(engine, t1, t2)
        repeat(300) {
            vs.update(dt)

            if (it % 10 == 0) {
                println("${engine.position.f2()}, ${t1.position.f2()}, ${t2.position.f2()}")
            }
        }

        val d1 = engine.position.distance(t1.position)
        val d2 = t1.position.distance(t2.position)

        assertEquals(4.0, d1, 0.3)
        assertEquals(4.0, d2, 0.3)
    }

    @Test
    fun testNoExtremeJackknife() {
        val engine = Vehicle()
        val trailer = Vehicle()

        engine.route.add(createStraight())
        engine.attachTrailer(trailer, 2f, 2f)

        // Force extreme initial angle
        trailer.rotation.rotationY((PI * 0.9).toFloat())

        engine.velocity.set(0f, 0f, 10f)

        val dt = 0.05f
        var maxAngle = 0.0

        val vs = listOf(engine,trailer)
        repeat(200) {
            vs.update(dt)

            val forward = trailer.rotation.transform(Vector3d(0.0, 0.0, 1.0))
            val angle = atan2(forward.x, forward.z)
            maxAngle = maxOf(maxAngle, abs(angle))
        }

        assertTrue(maxAngle < PI * 0.95, "Trailer should not fully flip/jackknife")
    }

    @Test
    fun testTrailerCollisionDoesNotBreakConstraint() {
        val engine = Vehicle()
        val trailer = Vehicle()
        val obstacle = Vehicle()
        engine.route.add(createStraight())

        engine.attachTrailer(trailer, 2f, 2f)

        obstacle.position.set(0.0, 0.0, 10.0)
        obstacle.maxVelocity = 0f

        engine.nearby.add(obstacle)
        trailer.nearby.add(obstacle)
        obstacle.nearby.add(engine)
        obstacle.nearby.add(trailer)

        engine.velocity.set(0f, 0f, 10f)

        val dt = 0.05f
        val vs = listOf(engine, trailer, obstacle)
        repeat(200) {
            vs.update(dt)
        }

        val dist = engine.position.distance(trailer.position)
        assertEquals(4.0, dist, 0.5, "Constraint should survive collision")
    }
}