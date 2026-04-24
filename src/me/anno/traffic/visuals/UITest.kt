package me.anno.traffic.visuals

import me.anno.ecs.Entity
import me.anno.ecs.components.mesh.MeshComponent
import me.anno.ecs.components.mesh.material.Material
import me.anno.ecs.systems.Systems
import me.anno.engine.DefaultAssets
import me.anno.engine.OfficialExtensions
import me.anno.engine.debug.DebugLine
import me.anno.engine.debug.DebugShapes
import me.anno.engine.ui.render.SceneView.Companion.testSceneWithUI
import me.anno.io.files.Reference.getReference
import me.anno.maths.Maths.TAUf
import me.anno.maths.Maths.posMod
import me.anno.traffic.Network
import me.anno.traffic.Street
import me.anno.traffic.visuals.StreetMeshBuilder.addStreetMesh
import org.joml.Vector3d
import java.lang.Math.TAU
import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.min

fun main() {

    OfficialExtensions.initForTests()

    val network = Network()
    val scene = Entity()

    Entity(scene)
        .add(MeshComponent(DefaultAssets.plane, Material.diffuse(0xaaff99)))
        .setScale(200f)

    Systems.registerSystem(network)

    // build some initial streets
    //  and build traffic meshes

    // plus an intersection? would be great for testing :D

    val n = 5
    val builder = StreetBuilder(network)

    val outer0 = Vector3d(110.0, 1.0, -100.0)
    val streets = ArrayList<Street>()
    for (i in 0 until n) {
        val angle = i * TAU / n

        builder.position0.set(30.0, 1.0, 0.0).rotateY(angle)
        builder.position1.set(110.0, 1.0, 0.0).rotateY(angle)
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
        Vector3d(0.0), 40.0,
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

    val carRef = getReference("/media/antonio/4TB WDRed/Assets/Quaternius/Cars.zip/SportsCar2.fbx/Scene.json")
    scene.add(VehicleRenderer(carRef, network))
    scene.add(RandomNavigator(network))

    spawnVehicles(network, streets)

    testSceneWithUI("Network Builder", scene) { sceneView ->
        sceneView.editControls = TrafficBuilderControls(sceneView, network)
    }
}

fun debugDrawCircle(center: Vector3d, radius: Double) {
    // debug-draw circle
    val n = 30
    val pts = List(n) {
        Vector3d(radius, 0.0, 0.0)
            .rotateY(it * TAU / n)
            .add(center)
    }
    for (i in 0 until n) {
        val line = DebugLine(pts[i], pts[posMod(i + 1, n)], -1, 1e3f)
        DebugShapes.debugLines.add(line)
    }
}

fun absAngleDiff(angle: Float): Float {
    val v = posMod(abs(angle), TAUf)
    return min(v, TAUf - v)
}

fun absAngleDiff(angle: Double): Double {
    val v = posMod(abs(angle), TAU)
    return min(v, TAU - v)
}

fun angleDiff(angle: Double): Double {
    val v = posMod(abs(angle), TAU) // 0 .. TAU
    return if (v < PI) v else v - TAU // -PI .. +PI
}
