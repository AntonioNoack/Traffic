package me.anno.traffic

import me.anno.traffic.utils.VehicleTree
import me.anno.utils.structures.tuples.IntPair
import org.joml.AABBd
import org.joml.Vector3d
import org.junit.jupiter.api.Assertions.assertEquals
import org.junit.jupiter.api.Assertions.assertTrue
import org.junit.jupiter.api.Test
import java.util.*
import kotlin.math.cbrt
import kotlin.math.max
import kotlin.math.min
import kotlin.math.sqrt

class NetworkTest {

    /**
     * this was an actual engine bug, as expected
     * */
    @Test
    fun testVehicleTreeMatchesQuadraticBaselineAfterTeleport() {
        val n = 10
        for (i in 0 until 1000) {
            val seed = 80L + i
            // println("Seed: $seed")
            val random = Random(seed)
            val initialPositions = List(n) {
                Vector3d(
                    random.nextGaussian(),
                    random.nextGaussian(),
                    random.nextGaussian()
                ).mul(6.0 * cbrt(n.toDouble()))
            }

            val treeNetwork = createNetwork(initialPositions)
            val baselineNetwork = createNetwork(initialPositions)

            // Warm up the cached bounds so the second frame tests the tree rebuild path.
            treeNetwork.useVehicleTree = false
            baselineNetwork.useVehicleTree = false
            treeNetwork.update(0.1f)
            baselineNetwork.update(0.1f)

            for (index in 0 until n) {
                val delta = Vector3d(
                    random.nextGaussian(),
                    random.nextGaussian(),
                    random.nextGaussian()
                ).mul(20.0)
                treeNetwork.vehicles[index].position.add(delta)
                baselineNetwork.vehicles[index].position.add(delta)
            }

            treeNetwork.useVehicleTree = true
            baselineNetwork.useVehicleTree = false

            treeNetwork.update(0f)
            baselineNetwork.update(0f)

            val expectedPairs = nearbyPairs(baselineNetwork.vehicles)
            val actualPairs = nearbyPairs(treeNetwork.vehicles)

            validateTree(treeNetwork.vehicleTree)

            // println(expectedPairs.size)
            assertTrue(expectedPairs.isNotEmpty(), "Test setup must create at least one close pair")
            assertEquals(expectedPairs, actualPairs) {
                val tooMuch = actualPairs - expectedPairs
                val tooLittle = expectedPairs - actualPairs
                val goodDistance = expectedPairs.map { (a, b) ->
                    val va = baselineNetwork.vehicles[a].position
                    val vb = baselineNetwork.vehicles[b].position
                    va.distance(vb) / Network.scanDistance
                }.sorted()
                val tooLittleDistance = tooLittle.map { (a, b) ->
                    val va = baselineNetwork.vehicles[a].position
                    val vb = baselineNetwork.vehicles[b].position
                    va.distance(vb) / Network.scanDistance
                }
                val tooLittleDistanceCheck = tooLittle.map { (a, b) ->
                    val va = baselineNetwork.vehicles[a].run { AABBd(treeBoundsMin, treeBoundsMax) }
                    val vb = baselineNetwork.vehicles[b].run { AABBd(treeBoundsMin, treeBoundsMax) }
                    if (a < b) {
                        println("entry[${Network.scanDistance}]:")
                        println("  va: $va @${baselineNetwork.vehicles[a].position}")
                        println("  vb: $vb @${baselineNetwork.vehicles[b].position}")
                    }
                    va.distance(vb) / Network.scanDistance
                }
                println("goodDistance: $goodDistance")
                println("tooLittleCheck: $tooLittleDistanceCheck")

                // print hierarchy
                printTree(treeNetwork)

                "Too Much: $tooMuch, Too Little: $tooLittle, $tooLittleDistance"
            }
        }
    }

    private fun AABBd.distance(other: AABBd): Double {
        return signedDistance(this, other)
    }


    fun signedDistance(a: AABBd, b: AABBd): Double {
        // Separation / overlap per axis
        val dx = axisDistance(a.minX, a.maxX, b.minX, b.maxX)
        val dy = axisDistance(a.minY, a.maxY, b.minY, b.maxY)
        val dz = axisDistance(a.minZ, a.maxZ, b.minZ, b.maxZ)

        // If any axis is separated → outside distance (Euclidean)
        val sx = max(dx, 0.0)
        val sy = max(dy, 0.0)
        val sz = max(dz, 0.0)

        val outsideDist = sqrt(sx * sx + sy * sy + sz * sz)

        // If fully overlapping → return penetration (largest negative)
        val insideDist = max(dx, max(dy, dz))

        return if (outsideDist > 0.0) outsideDist else insideDist
    }

    private fun axisDistance(aMin: Double, aMax: Double, bMin: Double, bMax: Double): Double {
        return when {
            aMax < bMin -> bMin - aMax              // A left of B
            bMax < aMin -> aMin - bMax              // B left of A
            else -> -min(aMax - bMin, bMax - aMin)  // overlap (negative penetration)
        }
    }

    private fun printTree(network: Network) {
        printTree(network, network.vehicleTree, 0)
    }

    private fun printTree(network: Network, tree: VehicleTree, depth: Int) {
        val left = tree.left
        val right = tree.right
        val tabs = "  ".repeat(depth)
        println(tree.values?.map { it.id - network.vehicles[0].id } ?: "N")
        if (left is VehicleTree) {
            print("$tabs  left: ")
            printTree(network, left, depth + 1)
            right as VehicleTree
            print("$tabs  right: ")
            printTree(network, right, depth + 1)
        }
    }

    private fun validateTree(tree: VehicleTree) {
        val targetBounds = AABBd()
        tree.forEach { vehicle -> targetBounds.union(AABBd(vehicle.treeBoundsMin, vehicle.treeBoundsMax)) }
        assertEquals(targetBounds.getMin(Vector3d()), tree.min)
        assertEquals(targetBounds.getMax(Vector3d()), tree.max)

        val left = tree.left
        val right = tree.right
        if (left is VehicleTree) validateTree(left)
        if (right is VehicleTree) validateTree(right)
    }

    private fun createNetwork(positions: List<Vector3d>): Network {
        return Network().apply {
            positions.forEach { position ->
                addVehicle(Vehicle().apply {
                    this.position.set(position)
                    maxVelocity = 0f
                })
            }
        }
    }

    private fun nearbyPairs(vehicles: List<Vehicle>): Set<IntPair> {
        val offset = vehicles.first().id
        val pairs = HashSet<IntPair>()
        for (vehicle in vehicles) {
            for (other in vehicle.nearby) {
                pairs.add(IntPair(vehicle.id - offset, other.id - offset))
            }
        }
        return pairs
    }
}
