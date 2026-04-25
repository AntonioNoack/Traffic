package me.anno.traffic

import me.anno.traffic.vehicle.update
import org.joml.Quaternionf
import org.joml.Vector3d
import org.junit.jupiter.api.Assertions.assertFalse
import org.junit.jupiter.api.Assertions.assertTrue
import org.junit.jupiter.api.Test
import kotlin.math.PI

class RightTurnPredictionTest {

    private fun lanePoint(x: Double, z: Double, angle: Double): LanePoint {
        return LanePoint(Vector3d(x, 0.0, z), Quaternionf().rotationY(angle.toFloat()), angle, 2.0)
    }

    @Test
    fun testRightTurnDoesNotOverBrakeForStoppedOpposingTraffic() {
        val crossing = object : Crossing(Vector3d(0.0, 0.0, 0.0), 15.0) {
            override fun mayDrive(from: Int, to: Int): Boolean = true
        }

        // Entry lane from north, exit lane to east.
        val entry0 = lanePoint(0.0, -100.0, PI * 0.5)
        val entry1 = lanePoint(0.0, -15.0, PI * 0.5)
        val exit0 = lanePoint(15.0, 0.0, 0.0)
        val exit1 = lanePoint(100.0, 0.0, 0.0)
        val entryLane = Lane(entry0, lanePoint(0.0, -50.0, PI * 0.5), entry1)
        val exitLane = Lane(exit0, lanePoint(50.0, 0.0, 0.0), exit1)

        val entryDir = Vector3d(entryLane.getPosition(1.01, 0.0, 0.0, Vector3d()))
            .sub(entry1.position)
            .normalize()
            .mul(10.0)
        val exitDir = Vector3d(exitLane.getPosition(-0.01, 0.0, 0.0, Vector3d()))
            .sub(exit0.position)
            .normalize()
            .mul(10.0)
        val centerPoint = Vector3d(entry1.position).add(entryDir).mix(Vector3d(exit0.position).add(exitDir), 0.5)
        val angle = (entry1.angle + exit0.angle) * 0.5
        val turnLane = Lane(entry1, LanePoint(centerPoint, Quaternionf().rotationY(angle.toFloat()), angle, 2.0), exit0)

        val csEntry = CrossingSection(crossing, 1)
        val csTurn = CrossingSection(crossing, 0)
        val csExit = CrossingSection(crossing, 3)
        entryLane.crossingSection = csEntry
        turnLane.crossingSection = csTurn
        exitLane.crossingSection = csExit
        crossing.sections.addAll(listOf(csEntry, csTurn, csExit))

        val vehicle = Vehicle()
        vehicle.route.add(entryLane)
        vehicle.route.add(turnLane)
        vehicle.route.add(exitLane)
        vehicle.position.set(0.0, 0.0, -60.0)
        vehicle.rotation.rotationY((PI * 0.5).toFloat())
        vehicle.updateDirections()
        vehicle.velocity.set(0.0, 0.0, 8.0)
        vehicle.maxVelocity = 13f

        // Stopped traffic in the opposing through lane.
        val blockers = listOf(
            Vehicle().apply {
                position.set(0.0, 0.0, 18.0)
                rotation.rotationY((-PI * 0.5).toFloat())
            },
            Vehicle().apply {
                position.set(0.0, 0.0, 28.0)
                rotation.rotationY((-PI * 0.5).toFloat())
            },
            Vehicle().apply {
                position.set(0.0, 0.0, 38.0)
                rotation.rotationY((-PI * 0.5).toFloat())
            }
        )

        blockers.forEach { blocker ->
            blocker.velocity.set(0.0, 0.0, 0.0)
            blocker.maxVelocity = 0f
            vehicle.nearby.add(blocker)
            blocker.nearby.add(vehicle)
        }

        val dt = 0.05f
        var minSpeedBeforeTurnCompleted = Float.POSITIVE_INFINITY
        val vehicles = listOf(vehicle) + blockers
        for (i in 0 until 400) {
            vehicles.update(dt)
            if (vehicle.routeIndex < 2 || vehicle.position.x <= 20.0) {
                minSpeedBeforeTurnCompleted = minOf(minSpeedBeforeTurnCompleted, vehicle.velocity.length())
            }
            if (i % 20 == 0) {
                println(
                    "step=$i pos=${vehicle.position} speed=${vehicle.velocity.length()} " +
                            "routeIndex=${vehicle.routeIndex} routeIndexF=${vehicle.routeIndexF}"
                )
            }
        }

        assertFalse(vehicle.isCrashed)
        assertTrue(vehicle.routeIndex >= 2, "Turning vehicle should complete the turn")
        assertTrue(vehicle.position.x > 20.0, "Turning vehicle should not stall behind stopped opposing traffic")
        assertTrue(
            minSpeedBeforeTurnCompleted > 0.2f,
            "Turning vehicle should not brake to a full stop before clearing the intersection, minSpeed=$minSpeedBeforeTurnCompleted"
        )
    }

    @Test
    fun testRightTurnUsesOtherRouteInsteadOfSteeringForPrediction() {
        val crossing = object : Crossing(Vector3d(0.0, 0.0, 0.0), 15.0) {
            override fun mayDrive(from: Int, to: Int): Boolean = true
        }

        val lanePoints = arrayOf(
            lanePoint(0.0, -100.0, PI * 0.5),
            lanePoint(0.0, -15.0, PI * 0.5),
            lanePoint(15.0, 0.0, 0.0),
            lanePoint(100.0, 0.0, 0.0)
        )

        val entryLane = Lane(lanePoints[0], lanePoint(0.0, -50.0, PI * 0.5), lanePoints[1])
        val exitLane = Lane(lanePoints[2], lanePoint(50.0, 0.0, 0.0), lanePoints[3])

        val entryDir = Vector3d(entryLane.getPosition(1.01, 0.0, 0.0, Vector3d()))
            .sub(lanePoints[1].position)
            .normalize()
            .mul(10.0)
        val exitDir = Vector3d(exitLane.getPosition(-0.01, 0.0, 0.0, Vector3d()))
            .sub(lanePoints[2].position)
            .normalize()
            .mul(10.0)
        val centerPoint =
            Vector3d(lanePoints[1].position).add(entryDir).mix(Vector3d(lanePoints[2].position).add(exitDir), 0.5)
        val angle = (lanePoints[1].angle + lanePoints[2].angle) * 0.5
        val turnLane = Lane(
            lanePoints[1],
            LanePoint(centerPoint, Quaternionf().rotationY(angle.toFloat()), angle, 2.0),
            lanePoints[2]
        )
        val blockerLane = Lane(
            lanePoint(0.0, 100.0, -PI * 0.5),
            lanePoint(0.0, 50.0, -PI * 0.5),
            lanePoint(0.0, -100.0, -PI * 0.5)
        )

        val csEntry = CrossingSection(crossing, 1)
        val csTurn = CrossingSection(crossing, 0)
        val csExit = CrossingSection(crossing, 3)
        entryLane.crossingSection = csEntry
        turnLane.crossingSection = csTurn
        exitLane.crossingSection = csExit
        crossing.sections.addAll(listOf(csEntry, csTurn, csExit))

        val turnVehicle = Vehicle()
        turnVehicle.route.add(entryLane)
        turnVehicle.route.add(turnLane)
        turnVehicle.route.add(exitLane)
        turnVehicle.position.set(0.0, 0.0, -60.0)
        turnVehicle.rotation.rotationY((PI * 0.5).toFloat())
        turnVehicle.updateDirections()
        turnVehicle.velocity.set(0.0, 0.0, 8.0)
        turnVehicle.maxVelocity = 13f

        val blockers = listOf(0, 1, 2).map { idx ->
            Vehicle().apply {
                route.add(blockerLane)
                position.set(0.0, 0.0, 18.0 + idx * 10.0)
                rotation.rotationY((-PI * 0.5).toFloat())
                velocity.set(0.0, 0.0, 0.0)
                steeringAngle = 0.55f
                maxVelocity = 0f
            }
        }

        blockers.forEach { blocker ->
            turnVehicle.nearby.add(blocker)
            blocker.nearby.add(turnVehicle)
        }

        val dt = 0.05f
        var minSpeedBeforeTurnCompleted = Float.POSITIVE_INFINITY
        val vehicles = listOf(turnVehicle) + blockers
        for (i in 0 until 400) {
            vehicles.update(dt)
            if (turnVehicle.routeIndex < 2 || turnVehicle.position.x <= 20.0) {
                minSpeedBeforeTurnCompleted = minOf(minSpeedBeforeTurnCompleted, turnVehicle.velocity.length())
            }
            if (i % 20 == 0) {
                println(
                    "routeStep=$i pos=${turnVehicle.position} speed=${turnVehicle.velocity.length()} " +
                            "routeIndex=${turnVehicle.routeIndex} routeIndexF=${turnVehicle.routeIndexF}"
                )
            }
        }

        assertFalse(turnVehicle.isCrashed)
        assertTrue(turnVehicle.routeIndex >= 2, "Turning vehicle should complete the turn")
        assertTrue(turnVehicle.position.x > 20.0, "Turning vehicle should not stall behind stopped opposing traffic")
        assertTrue(
            minSpeedBeforeTurnCompleted > 0.2f,
            "Route-aware prediction should ignore steering-only false positives, minSpeed=$minSpeedBeforeTurnCompleted"
        )
    }
}
