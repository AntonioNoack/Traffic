package me.anno.traffic.visuals

import me.anno.ecs.Entity
import me.anno.ecs.components.mesh.MeshComponent
import me.anno.ecs.systems.Systems
import me.anno.engine.DefaultAssets
import me.anno.engine.raycast.Raycast
import me.anno.engine.ui.LineShapes
import me.anno.engine.ui.control.DraggingControls
import me.anno.engine.ui.render.SceneView
import me.anno.engine.ui.render.SceneView.Companion.testSceneWithUI
import me.anno.gpu.pipeline.Pipeline
import me.anno.input.Key
import me.anno.maths.Maths.mixAngle
import me.anno.traffic.Lane
import me.anno.traffic.LanePoint
import me.anno.traffic.Network
import me.anno.traffic.Road
import me.anno.ui.UIColors
import org.joml.Quaternionf
import org.joml.Vector3d
import kotlin.math.cos
import kotlin.math.sin

fun main() {
    val network = Network()
    val scene = Entity()
    Entity(scene)
        .add(MeshComponent(DefaultAssets.plane))
        .setScale(200f)

    testSceneWithUI("Network Builder", scene) { sceneView ->
        sceneView.editControls = UITest(sceneView, network)
    }
}

class UITest(sceneView: SceneView, val network: Network) :
    DraggingControls(sceneView.renderView) {

    val scene get() = Systems.world as Entity

    val position0 = Vector3d()
    val position1 = Vector3d()
    val position2 = Vector3d()

    enum class State {
        NO_POINTS,
        FIRST_POINT,
        SECOND_POINT,
    }

    var state = State.NO_POINTS

    val laneWidth = 4.0

    fun getPoint(i: Int, j: Int, n: Int): LanePoint {
        val jr = (j - (n - 1) * 0.5) * laneWidth

        // calculate ideal position
        val angle0 = position0.angleYTo(position1)
        val angle1 = position1.angleYTo(position2)
        val angle = when (i) {
            0 -> angle0
            1 -> mixAngle(angle0, angle1, 0.5)
            2 -> angle1
            else -> throw IllegalStateException()
        }

        val base = when (i) {
            0 -> position0
            1 -> position1
            2 -> position2
            else -> throw IllegalStateException()
        }

        val idealPosition = Vector3d(base)
            .add(cos(angle) * jr, 0.0, sin(angle) * jr)
        val point0 = network.getPoint(idealPosition, laneWidth * 0.5)
        if (point0 != null) return point0

        val point1 = LanePoint(idealPosition, Quaternionf().rotateY(angle.toFloat()), angle, laneWidth * 0.5)
        network.addPoint(point1)
        return point1
    }

    override fun fill(pipeline: Pipeline) {
        super.fill(pipeline)

        // todo draw active points
        // todo draw potential street

        // draw existing streets
        for (lane in network.lanes) {
            drawLine(lane, 0.0, 0.0)
        }
    }

    fun drawLine(lane: Lane, x: Double, y: Double) {
        // draw bezier shape
        val n = 10
        for (i in 0 until n) {
            val f0 = (i + 0.0) / n
            val f1 = (i + 1.0) / n
            val p0 = lane.getPosition(f0, x, y, Vector3d())
            val p1 = lane.getPosition(f1, x, y, Vector3d())
            drawLine(p0, p1)
        }
    }

    fun drawLine(from: Vector3d, to: Vector3d) {
        LineShapes.drawArrowZ(from, to, UIColors.dodgerBlue)
    }

    override fun onMouseClicked(x: Float, y: Float, button: Key, long: Boolean) {
        when (button) {
            Key.BUTTON_LEFT -> {
                val query = renderView.rayQuery()
                if (!Raycast.raycast(scene, query)) return

                val dst = when (state) {
                    State.NO_POINTS -> position0
                    State.FIRST_POINT -> position1
                    State.SECOND_POINT -> position2
                }

                dst.set(query.result.positionWS)
                if (state == State.SECOND_POINT) {
                    placeStreet()
                }

                state = when (state) {
                    State.NO_POINTS -> State.FIRST_POINT
                    State.FIRST_POINT -> State.SECOND_POINT
                    State.SECOND_POINT -> State.FIRST_POINT // don't reuse middle
                }
            }
            Key.BUTTON_RIGHT -> {
                state = when (state) {
                    State.NO_POINTS -> State.NO_POINTS
                    State.FIRST_POINT -> State.NO_POINTS
                    State.SECOND_POINT -> State.FIRST_POINT
                }
            }
            else -> {}
        }
    }

    fun placeStreet() {
        val numLanes = 6
        val numReversed = 3
        val from = List(numLanes) { idx ->
            getPoint(0, idx, numLanes)
        }
        val control = List(numLanes) { idx ->
            getPoint(1, idx, numLanes)
        }
        val to = List(numLanes) { idx ->
            getPoint(2, idx, numLanes)
        }
        val lanes = List(6) {
            if (it <= numReversed) {
                Lane(to[it], control[it], from[it])
            } else {
                Lane(from[it], control[it], to[it])
            }
        }
        network.addRoad(Road(lanes))
    }
}