package me.anno.traffic

import me.anno.traffic.VehicleTest.Companion.createLane
import me.anno.traffic.vehicle.setOn
import me.anno.traffic.vehicle.updateS
import org.joml.Vector3d
import org.junit.jupiter.api.Test

class VehicleHillTests {

    fun runTest(dy: Double, v: Vehicle = Vehicle()): Vehicle {
        val lane = createLane(
            Vector3d(0.0, 0.0, 0.0),
            Vector3d(0.0, dy * 0.5, 50.0).normalize(50.0),
            Vector3d(0.0, dy, 100.0).normalize(100.0)
        )

        v.setOn(lane, 0f)

        val dt = 0.1f
        for (i in 0 until 50) {
            v.updateS(dt)
        }

        return v
    }

    @Test
    fun testCompareHillSpeeds() {
        // todo we should test acceleration/decelerating, straight/up/down
        val uphill = runTest(20.0)
        val flat = runTest(0.0)
        val downhill = runTest(-20.0)
        println(uphill.routeIndexF)
        println(flat.routeIndexF)
        println(downhill.routeIndexF)

        println()
        println(uphill.velocity)
        println(flat.velocity)
        println(downhill.velocity)
    }

}