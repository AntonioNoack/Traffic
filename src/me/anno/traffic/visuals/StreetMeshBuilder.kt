package me.anno.traffic.visuals

import me.anno.ecs.Entity
import me.anno.ecs.components.mesh.Mesh
import me.anno.ecs.components.mesh.MeshComponent
import me.anno.ecs.components.mesh.material.Material
import me.anno.ecs.components.mesh.spline.SplineMesh
import me.anno.ecs.components.mesh.spline.SplineProfile
import me.anno.ecs.components.mesh.utils.MeshJoiner
import me.anno.gpu.CullMode
import me.anno.traffic.Lane
import me.anno.traffic.Street
import me.anno.utils.structures.arrays.FloatArrayList
import org.joml.Matrix4x3f
import org.joml.Vector2f
import org.joml.Vector3d

object StreetMeshBuilder {

    fun addStreetMesh(street: Street, entity: Entity) {
        val fixMaterial = Material().apply { cullMode = CullMode.BOTH }
        entity.add(MeshComponent(createStreetMesh(street), fixMaterial))
    }

    fun createStreetMesh(street: Street): Mesh {
        val meshes = street.lanes.map { lane -> createLaneMesh(lane) }
        return object : MeshJoiner<Mesh>(false, false, mayHaveUVs = true) {
            override fun getMesh(element: Mesh): Mesh = element
            override fun getTransform(element: Mesh, dst: Matrix4x3f) {}
        }.join(meshes)
    }

    fun createLaneMesh(lane: Lane): Mesh {
        val profile = SplineProfile(
            listOf(
                Vector2f(-3f, -1f),
                Vector2f(-2f, 0f),
                Vector2f(+2f, 0f),
                Vector2f(+3f, -1f),
            ).map { it * 0.8f }, FloatArrayList(
                floatArrayOf(
                    -1f, -0.2f, 0.2f, +1f
                )
            ), null, false
        )
        val n = 10
        val splinePoints = List(2 * n) { pointIndex ->
            val t = pointIndex.shr(1) / (n - 1.0)
            val lrx = pointIndex.and(1) * 2.0 - 1.0
            lane.getPosition(t, lrx, 0.0, Vector3d())
        }
        return SplineMesh.generateSplineMesh(
            Mesh(), profile, false, true, true, true,
            splinePoints
        )
    }
}