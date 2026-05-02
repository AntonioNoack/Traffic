package me.anno.traffic.testing

import me.anno.ecs.Entity
import me.anno.ecs.components.mesh.MeshComponent
import me.anno.ecs.components.mesh.material.Material
import me.anno.ecs.systems.Systems
import me.anno.engine.DefaultAssets
import me.anno.engine.OfficialExtensions
import me.anno.engine.debug.DebugLine
import me.anno.engine.debug.DebugShapes
import me.anno.engine.ui.render.SceneView.Companion.testSceneWithUI
import me.anno.traffic.Network
import me.anno.traffic.simulation.RandomNavigator
import me.anno.traffic.visuals.TrafficBuilderControls
import me.anno.traffic.visuals.VehicleRenderer
import me.anno.ui.UIColors
import me.anno.utils.OS.res
import org.joml.Vector3d

/**
 * build some initial streets
 *  and build traffic meshes
 *
 *  plus an intersection? would be great for testing :D
 * */
fun main() {

    OfficialExtensions.initForTests()

    val network = Network()
    val scene = Entity()

    Entity(scene)
        .add(MeshComponent(DefaultAssets.plane, Material.diffuse(0xaaff99)))
        .setScale(200f)

    Systems.registerSystem(network)

    val carRef = res.getChild("meshes/SportsCar2.fbx/Scene.json")
    scene.add(VehicleRenderer(carRef, network))
    scene.add(RandomNavigator(network))

    val streets = buildFivePointIntersectionInPlannerMode(network, scene)
    spawnVehicles(network, streets)

    // show gizmos on all lane ends
    for (street in streets) {
        for (lane in street.lanes) {
            for (point in listOf(lane.from, lane.to)) {
                val position = Vector3d(point.position).apply { y += 10.0 }
                fun showArrow(x: Float, y: Float, z: Float, color: Int) {
                    val arrow = DebugLine(
                        position,
                        Vector3d(x, y, z)
                            .rotate(point.rotation)
                            .mul(3.0)
                            .add(position),
                        color, 1e3f
                    )
                    DebugShapes.showDebugArrow(arrow)
                }
                showArrow(1f, 0f, 0f, UIColors.axisXColor)
                showArrow(0f, 1f, 0f, UIColors.axisYColor)
                showArrow(0f, 0f, 1f, UIColors.axisZColor)
            }
        }
    }

    testSceneWithUI("Network Builder", scene) { sceneView ->
        sceneView.editControls = TrafficBuilderControls(sceneView, network)
    }
}
