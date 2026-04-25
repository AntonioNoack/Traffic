package me.anno.traffic

import me.anno.maths.Maths.TAUf
import me.anno.traffic.VehicleTest.Companion.createStraight
import me.anno.utils.types.Floats.f2
import org.junit.jupiter.api.Test
import java.util.*
import kotlin.math.abs
import kotlin.math.min

class CrashPreventionTests {
    @Test
    fun testCrashPrevention() {
        // todo spawn two vehicles in random positions, velocities and orientations, and make sure both reach the goal
        val random = Random(1324)
        for (i in 0 until 1000) {
            val va = Vehicle()
            val vb = Vehicle()

            va.rotation.rotationY(random.nextFloat() * TAUf)
            vb.rotation.rotationY(random.nextFloat() * TAUf)
            va.updateDirections()
            vb.updateDirections()

            vb.position.set(10.0, 0.0, 0.0)
            va.updateStrictBounds()
            vb.updateStrictBounds()

            val distance = va.boundsDistance(vb)
            val overlap = distance.minOverlap
            vb.position.x += overlap + 5f

            // they may be on different routes
            va.route.add(createStraight())
            vb.route.add(createStraight())

            va.nearby.add(vb)
            vb.nearby.add(va)

            val dt = 0.1f
            var j = 0
            while (++j < 200) {
                va.update(dt)
                vb.update(dt)

                check(!va.isCrashed)
                check(!vb.isCrashed)

                val progress = min(va.routeIndexF, vb.routeIndexF)
                if (progress > 0.5f) break
            }

            println("Result[$i]: ${va.routeIndexF}, ${vb.routeIndexF} @$j")

            check(va.routeIndexF > 0.5f)
            check(vb.routeIndexF > 0.5f)

        }
    }

    @Test
    fun safetyDistanceCheck() {

        val random = kotlin.random.Random(1324)

        val numVehicles = 10
        val sample = Vehicle()
        val safetyDistance = 1.5 + sample.localBounds.deltaZ
        val startDistance = safetyDistance * 2.0
        val totalLength = startDistance * 1.2 * numVehicles + 50.0
        val route = createStraight(totalLength)

        val shuffledIndices = IntArray(numVehicles) { it }
        shuffledIndices.shuffle(random)

        val deshuffledIndices = IntArray(numVehicles)
        for (i in 0 until numVehicles) {
            deshuffledIndices[shuffledIndices[i]] = i
        }

        val vehicles = List(numVehicles) { index ->
            val vehicle = Vehicle()
            vehicle.position.z = index * startDistance
            vehicle.route.add(route)
            vehicle
        }

        val randomUpdateOrder = ArrayList(vehicles)
        randomUpdateOrder.shuffle(random)

        for (i in 1 until numVehicles) {
            val va = vehicles[i - 1]
            val vb = vehicles[i]
            va.nearby.add(vb)
            vb.nearby.add(va)
        }

        val dt = 0.1f
        var k = 0
        while (++k < 1000) {
            // yes, we need much less than 100s to reach the end,
            //  it's only ~50m + numVehicles * ~3m or so after all,
            //  but we want to confirm they are stable, too

            for (i in vehicles.indices) {
                randomUpdateOrder[i].update(dt)
            }

            if (k % 10 == 0 || k == 463) {
                val distances = List(vehicles.size - 1) {
                    val delta = (vehicles[it + 1].position.z - vehicles[it].position.z)
                    (delta / safetyDistance).f2()
                }
                println("[$k] Positions: ${vehicles.map { it.position.z.f2() }}")
                println("[$k] Velocities: ${vehicles.map { it.velocity.z.f2() }}")
                println("[$k] Distances: $distances")
                check(distances.all { it.toFloat() > 0.95 })
            }

            for (i in vehicles.indices) {
                val va = vehicles[i]
                check(abs(va.position.x) < 1.0)
                check(abs(va.position.y) < 1.0)
                check(!va.isCrashed)
                check(va.timeSinceCollision < 0f) {
                    println("Positions: ${vehicles.map { it.position.z.f2() }}")
                    println("Velocities: ${vehicles.map { it.velocity.z.f2() }}")
                    val crashedVehicleIds = vehicles.withIndex()
                        .filter { it.value.timeSinceCollision >= 0f }
                        .map { it.index }
                    "Vehicles $crashedVehicleIds collided at step $k"
                }
            }

            // validate distance to previous and after...
            for (i in 1 until numVehicles) {
                val va = vehicles[i - 1]
                val vb = vehicles[i]

                val xa = va.position.z
                val xb = vb.position.z
                val distance = xb - xa

                // todo we should be able to calculate the ideal safety distance at each velocity... do that... and compare to it
                check(distance in safetyDistance * 0.7..startDistance * 5.0) {
                    println("Positions: ${vehicles.map { it.position.z.f2() }}")
                    println("Velocities: ${vehicles.map { it.velocity.z.f2() }}")
                    "Vehicles $i are too close at step $k, ${distance.f2()} vs ${safetyDistance.f2()} .. ${startDistance.f2()}"
                }
            }
        }

        println("Total: $totalLength")
        println("Progress: ${vehicles.map { it.position.z }}")

        // todo validate all vehicles have driven to the end...
        // validate distance to previous and after...
        for (i in vehicles.indices) {
            val va = vehicles[i]
            check(va.position.z < totalLength)
            check(va.position.z > totalLength - i * safetyDistance * 1.05)
        }

        // validate distance to previous and after...
        for (i in 1 until numVehicles) {
            val va = vehicles[i - 1]
            val vb = vehicles[i]

            val xa = va.position.z
            val xb = vb.position.z
            val distance = xb - xa

            check(distance in safetyDistance * 0.99..safetyDistance * 1.01) {
                "Vehicles $i are too close at step $k, $distance vs $safetyDistance"
            }
        }

    }
}