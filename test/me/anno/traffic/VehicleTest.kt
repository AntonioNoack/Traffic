package me.anno.traffic

import org.joml.Quaternionf
import org.joml.Vector3d
import org.junit.jupiter.api.Assertions.*
import org.junit.jupiter.api.Test
import kotlin.math.PI

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
            assertTrue(dist > 3.0, "Vehicles collided at step $i: dist=$dist")
        }

        assertTrue(fast.velocity.length() <= slow.velocity.length() + 0.5)
        assertFalse(fast.isCrashed)
        assertFalse(slow.isCrashed)
    }

    @Test
    fun testStoppingBehindVehicleOnCurve() {
        // 90 degree turn from (0,0,0) towards (100,0,100)
        val p0 = LanePoint(Vector3d(0.0, 0.0, 0.0), Quaternionf().rotationY(0f), 0.0, 0.0)
        val p1 = LanePoint(Vector3d(0.0, 0.0, 100.0), Quaternionf().rotationY(PI.toFloat() * 0.25f), 0.0, 0.0)
        val p2 = LanePoint(Vector3d(100.0, 0.0, 100.0), Quaternionf().rotationY(PI.toFloat() * 0.5f), 0.0, 0.0)
        val lane = Lane(p0, p1, p2)

        val slow = Vehicle()
        slow.route.add(lane)
        slow.position.set(p0.position)
        slow.maxVelocity = 2.0

        val fast = Vehicle()
        fast.route.add(lane)
        fast.position.set(p0.position)
        fast.maxVelocity = 10.0

        // Give slow a head start
        val dt = 0.1f
        for (i in 0 until 100) {
            slow.update(dt)
        }

        fast.nearby.add(slow)
        slow.nearby.add(fast)

        for (i in 0 until 1000) {
            slow.update(dt)
            fast.update(dt)
            
            val dist = slow.position.distance(fast.position)
            assertTrue(dist > 3.0, "Vehicles collided on curve at step $i: dist=$dist")
            
            if (slow.routeIndexF >= 0.9f && fast.routeIndexF >= 0.8f) break
        }

        assertFalse(fast.isCrashed)
    }

    @Test
    fun testCrashSituation() {
        val p0 = LanePoint(Vector3d(0.0, 0.0, 0.0), Quaternionf(), 0.0, 0.0)
        val p1 = LanePoint(Vector3d(0.0, 0.0, 50.0), Quaternionf(), 0.0, 0.0)
        val p2 = LanePoint(Vector3d(0.0, 0.0, 100.0), Quaternionf(), 0.0, 0.0)
        val lane = Lane(p0, p1, p2)

        val obstacle = Vehicle()
        obstacle.route.add(lane)
        obstacle.position.set(0.0, 0.0, 10.0)
        obstacle.velocity.set(0.0, 0.0, 0.0)
        obstacle.maxVelocity = 0.0

        val speeder = Vehicle()
        speeder.route.add(lane)
        speeder.position.set(0.0, 0.0, 0.0)
        speeder.velocity.set(0.0, 0.0, 30.0)
        speeder.maxVelocity = 30.0

        speeder.nearby.add(obstacle)

        val dt = 0.01f
        var crashed = false
        for (i in 0 until 200) {
            speeder.update(dt)
            if (speeder.isCrashed) {
                crashed = true
                break
            }
        }

        assertTrue(crashed, "Speeder should have crashed into obstacle")
    }

    @Test
    fun testCurveFollowing() {
        val p0 = LanePoint(Vector3d(0.0, 0.0, 0.0), Quaternionf().rotationY(0f), 0.0, 0.0)
        val p1 = LanePoint(Vector3d(0.0, 0.0, 10.0), Quaternionf().rotationY(PI.toFloat()*0.25f), 0.0, 0.0)
        val p2 = LanePoint(Vector3d(10.0, 0.0, 10.0), Quaternionf().rotationY(PI.toFloat()*0.5f), 0.0, 0.0)
        val lane = Lane(p0, p1, p2)

        val v = Vehicle()
        v.route.add(lane)
        v.maxVelocity = 5.0

        val dt = 0.1f
        for (i in 0 until 100) {
            v.update(dt)
        }

        assertTrue(v.position.x > 5.0)
        assertTrue(v.position.z > 5.0)
    }
}