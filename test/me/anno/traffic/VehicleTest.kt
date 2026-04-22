package me.anno.traffic

import org.joml.Quaternionf
import org.joml.Vector3d
import org.junit.jupiter.api.Assertions.*
import org.junit.jupiter.api.Test
import kotlin.math.PI
import kotlin.math.abs

class VehicleTest {

    @Test
    fun testStraightLineFollowing() {
        val p0 = LanePoint(Vector3d(0.0, 0.0, 0.0), Quaternionf(), 0.0, 0.0)
        val p1 = LanePoint(Vector3d(0.0, 0.0, 50.0), Quaternionf(), 0.0, 0.0)
        val p2 = LanePoint(Vector3d(0.0, 0.0, 100.0), Quaternionf(), 0.0, 0.0)
        val lane = Lane(p0, p1, p2)

        val v1 = Vehicle()
        v1.route.add(lane)
        v1.position.set(0.0, 0.0, 0.0)
        v1.maxVelocity = 10.0

        val dt = 0.1f
        for (i in 0 until 100) {
            v1.update(dt)
        }

        assertTrue(v1.velocity.z > 0)
        assertTrue(v1.position.z > 0)
        assertEquals(0.0, v1.position.x, 0.1)
    }

    @Test
    fun testStoppingBehindVehicle() {
        val p0 = LanePoint(Vector3d(0.0, 0.0, 0.0), Quaternionf(), 0.0, 0.0)
        val p1 = LanePoint(Vector3d(0.0, 0.0, 50.0), Quaternionf(), 0.0, 0.0)
        val p2 = LanePoint(Vector3d(0.0, 0.0, 500.0), Quaternionf(), 0.0, 0.0)
        val lane = Lane(p0, p1, p2)

        val slow = Vehicle()
        slow.route.add(lane)
        slow.position.set(0.0, 0.0, 50.0)
        slow.velocity.set(0.0, 0.0, 2.0)
        slow.maxVelocity = 2.0

        val fast = Vehicle()
        fast.route.add(lane)
        fast.position.set(0.0, 0.0, 0.0)
        fast.maxVelocity = 20.0

        // Manual nearby injection for test
        fast.nearby.add(slow)
        slow.nearby.add(fast)

        val dt = 0.1f
        for (i in 0 until 500) {
            slow.update(dt)
            fast.update(dt)
            
            val dist = slow.position.distance(fast.position)
            assertTrue(dist > 3.8, "Vehicles collided at step $i: dist=$dist")
        }

        assertTrue(fast.velocity.length() <= slow.velocity.length() + 0.5)
        assertFalse(fast.isCrashed)
        assertFalse(slow.isCrashed)
    }

    @Test
    fun testRectangleSATCollisionResolution() {
        // Test that overlapping rectangles are pushed apart correctly
        val v1 = Vehicle()
        v1.position.set(0.0, 0.0, 0.0)
        v1.rotation.rotationY(0f)

        val v2 = Vehicle()
        // Place v2 slightly overlapping v1 on the side
        // v1 width is ~1.86 (-0.93 to 0.93). Place v2 at x=1.5 (overlap of ~0.36)
        v2.position.set(1.5, 0.0, 0.1)
        v2.rotation.rotationY(0f)

        v1.nearby.add(v2)
        v2.nearby.add(v1)

        val dt = 0.01f
        val initialDist = v1.position.distance(v2.position)
        
        // Resolve for a few frames
        for (i in 0 until 10) {
            v1.update(dt)
            v2.update(dt)
        }

        val finalDist = v1.position.distance(v2.position)
        assertTrue(finalDist > initialDist, "Vehicles should be pushed apart")
        // Pushing should be primarily on X axis for this configuration
        assertTrue(abs(v1.position.x) > 0.0, "v1 should have been pushed on X")
    }

    @Test
    fun testDeterministicCrashSpin() {
        val v1 = Vehicle()
        v1.position.set(0.0, 0.0, 0.0)
        v1.velocity.set(0.0, 0.0, 20.0) // Moving fast forward

        val v2 = Vehicle()
        // Offset v2 so it's a glancing blow (T-bone style)
        v2.position.set(1.0, 0.0, 2.0)
        v2.rotation.rotationY(PI.toFloat() * 0.5f) // Sideways

        v1.nearby.add(v2)
        v2.nearby.add(v1)

        val dt = 0.01f
        // Run update to trigger crash
        v1.update(dt)
        v2.update(dt)

        assertTrue(v1.isCrashed)
        assertTrue(v2.isCrashed)
        assertNotEquals(0.0, v1.angularVelocity, "Collision should have induced spin")
        
        val spinBefore = v1.angularVelocity
        // Re-run the exact same setup to verify determinism
        val v1b = Vehicle()
        v1b.position.set(0.0, 0.0, 0.0)
        v1b.velocity.set(0.0, 0.0, 20.0)
        val v2b = Vehicle()
        v2b.position.set(1.0, 0.0, 2.0)
        v2b.rotation.rotationY(PI.toFloat() * 0.5f)
        v1b.nearby.add(v2b)
        v2b.nearby.add(v1b)
        
        v1b.update(dt)
        assertEquals(spinBefore, v1b.angularVelocity, "Physics should be deterministic")
    }

    @Test
    fun testSidewaysResistance() {
        val v1 = Vehicle()
        v1.position.set(0.0, 0.0, 0.0)
        v1.rotation.rotationY(0f)

        // Force a velocity that is purely lateral
        v1.velocity.set(5.0, 0.0, 0.0)

        val dt = 0.1f
        v1.update(dt)

        // Sideways velocity should be heavily damped by tire grip (maxLateralG = 1.0)
        // 1.0G * 0.1s = 0.981 m/s reduction
        val expectedReduction = 0.981
        assertEquals(5.0 - expectedReduction, v1.velocity.x, 0.01)
    }

    @Test
    fun testAckermannSteeringOnCurve() {
        val p0 = LanePoint(Vector3d(0.0, 0.0, 0.0), Quaternionf().rotationY(0f), 0.0, 0.0)
        val p1 = LanePoint(Vector3d(0.0, 0.0, 100.0), Quaternionf().rotationY(PI.toFloat() * 0.25f), 0.0, 0.0)
        val p2 = LanePoint(Vector3d(100.0, 0.0, 100.0), Quaternionf().rotationY(PI.toFloat() * 0.5f), 0.0, 0.0)
        val lane = Lane(p0, p1, p2)

        val v = Vehicle()
        v.route.add(lane)
        v.maxVelocity = 10.0

        val dt = 0.1f
        for (i in 0 until 150) {
            v.update(dt)
        }

        // Vehicle should have turned and be moving roughly in +X direction
        val forward = v.rotation.transform(Vector3d(0.0, 0.0, 1.0))
        assertTrue(forward.x > 0.7, "Vehicle should be facing mostly +X")
        assertTrue(v.angularVelocity > 0, "Vehicle should have had positive angular velocity during turn")
    }

    @Test
    fun testReversingPrevention() {
        val v = Vehicle()
        // Give it some forward momentum then try to set backward target
        v.velocity.set(0.0, 0.0, 5.0)
        
        // No route or target velocity means it wants to stop
        // Let's manually inject a backward push
        v.velocity.set(0.0, 0.0, -2.0)
        
        val dt = 0.1f
        v.update(dt)
        
        // vF < -0.1 should trigger stopping force
        assertTrue(v.velocity.z > -0.1, "Reversing should be prevented")
    }
}