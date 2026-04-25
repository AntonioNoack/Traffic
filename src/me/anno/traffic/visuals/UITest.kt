package me.anno.traffic.visuals

import me.anno.ecs.Entity
import me.anno.ecs.components.mesh.MeshComponent
import me.anno.ecs.components.mesh.material.Material
import me.anno.ecs.systems.Systems
import me.anno.engine.DefaultAssets
import me.anno.engine.OfficialExtensions
import me.anno.engine.ui.render.SceneView.Companion.testSceneWithUI
import me.anno.traffic.Network
import me.anno.utils.OS.res

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

    val streets = buildFivePointIntersection(network, scene)

    val carRef = res.getChild("meshes/SportsCar2.fbx/Scene.json")
    scene.add(VehicleRenderer(carRef, network))
    scene.add(RandomNavigator(network))

    spawnVehicles(network, streets)

    testSceneWithUI("Network Builder", scene) { sceneView ->
        sceneView.editControls = TrafficBuilderControls(sceneView, network)
    }
}
