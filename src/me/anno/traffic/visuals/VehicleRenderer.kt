package me.anno.traffic.visuals

import me.anno.ecs.Transform
import me.anno.ecs.components.mesh.IMesh
import me.anno.ecs.components.mesh.MeshCache
import me.anno.ecs.components.mesh.MeshSpawner
import me.anno.ecs.components.mesh.material.Material
import me.anno.ecs.components.mesh.material.MaterialBase
import me.anno.gpu.pipeline.Pipeline
import me.anno.io.files.FileReference
import me.anno.traffic.Network

class VehicleRenderer(
    val carMesh: FileReference,
    val network: Network
) : MeshSpawner() {

    val crashedMat = Material.diffuse(0x333333)
    val trailerMat = Material.diffuse(0xffff99)

    override fun forEachMesh(
        pipeline: Pipeline?,
        callback: (IMesh, MaterialBase?, Transform) -> Boolean
    ) {
        val mesh = MeshCache[carMesh] ?: return

        val vehicles = network.vehicles
        for (i in vehicles.indices) {
            val vehicle = vehicles[i]
            val transform = getTransform(i).apply {
                localPosition = vehicle.position
                localRotation = vehicle.rotation
            }
            val material =
                if (vehicle.isTrailer) trailerMat
                else if (vehicle.isCrashed) crashedMat
                else null
            callback(mesh, material, transform)
        }
    }
}