package me.anno.traffic.visuals

import me.anno.ecs.Entity
import me.anno.maths.Maths.posMod
import me.anno.traffic.Network
import me.anno.traffic.Street
import me.anno.traffic.visuals.StreetMeshBuilder.addStreetMesh
import org.joml.Vector3d
import java.lang.Math.TAU

fun buildFivePointIntersection(network: Network, scene: Entity): List<Street> {

    val raise = 20.0

    val n = 5
    val builder = StreetBuilder(network)

    val outer0 = Vector3d(110.0, raise * 0.1 + 1.0, -100.0)
    val streets = ArrayList<Street>()
    for (i in 0 until n) {
        val angle = i * TAU / n

        builder.position0.set(30.0, raise + 1.0, 0.0).rotateY(angle)
        builder.position1.set(110.0, raise * 0.5 + 1.0, 0.0).rotateY(angle)
        builder.position2.set(outer0).rotateY(angle)

        val street = builder.placeStreet()
        addStreetMesh(street, scene)
        streets.add(street)
    }

    val outer1 = Vector3d(130.0, 1.0, -130.0)
    for (i in 0 until n) {
        val angle0 = (i + 0.2) * TAU / n
        val angle1 = (i + 0.5) * TAU / n
        val angle2 = (i + 0.9) * TAU / n

        builder.position0.set(outer1).rotateY(angle0)
        builder.position1.set(outer1).rotateY(angle1)
        builder.position2.set(outer1).rotateY(angle2)

        builder.extrudeCenter(1.0)

        val street = builder.placeStreet()
        addStreetMesh(street, scene)
        streets.add(street)
    }

    createIntersection(
        network, scene,
        streets.subList(0, n),
        Vector3d(0.0, raise, 0.0), 40.0,
        builder
    )

    for (i in 0 until n) {
        val angle = (i + 0.05) * TAU / n
        val center = Vector3d(outer1)
            .mix(outer0, 0.3)
            .rotateY(angle)
        val j = posMod(i - 1, n)
        createIntersection(
            network, scene,
            listOf(streets[i], streets[i + n], streets[j + n]),
            center, 50.0,
            builder
        )
    }

    return streets
}