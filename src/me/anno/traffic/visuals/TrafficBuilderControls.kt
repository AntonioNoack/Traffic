package me.anno.traffic.visuals

import me.anno.ecs.Entity
import me.anno.ecs.systems.Systems
import me.anno.engine.raycast.Raycast
import me.anno.engine.ui.LineShapes
import me.anno.engine.ui.control.DraggingControls
import me.anno.engine.ui.render.SceneView
import me.anno.gpu.pipeline.Pipeline
import me.anno.input.Key
import me.anno.traffic.Lane
import me.anno.traffic.Network
import me.anno.traffic.Street
import me.anno.traffic.editor.StreetBuilder
import me.anno.ui.UIColors
import org.joml.Vector3d

class TrafficBuilderControls(sceneView: SceneView, val network: Network) :
    DraggingControls(sceneView.renderView) {

    val scene get() = Systems.world as Entity

    var state = TrafficBuilderState.NO_POINTS

    val builder = StreetBuilder(network)

    override fun fill(pipeline: Pipeline) {
        super.fill(pipeline)

        // todo draw active points

        drawPotentialStreet()

        // draw existing streets
        if (false) for (lane in network.lanes) {
            drawLine(lane)
        }
        for (street in network.streets) {
            drawLine(street)
        }
    }

    fun drawPotentialStreet() {
        if (state == TrafficBuilderState.NO_POINTS) return
        val query = renderView.rayQuery()
        val numPoints = if (Raycast.raycast(scene, query)) {
            val dst = when (state) {
                TrafficBuilderState.NO_POINTS -> builder.position0
                TrafficBuilderState.FIRST_POINT -> builder.position1
                TrafficBuilderState.SECOND_POINT -> builder.position2
            }

            dst.set(query.result.positionWS)

            when (state) {
                TrafficBuilderState.NO_POINTS -> 1
                TrafficBuilderState.FIRST_POINT -> 2
                TrafficBuilderState.SECOND_POINT -> 3
            }
        } else {
            when (state) {
                TrafficBuilderState.NO_POINTS -> 0
                TrafficBuilderState.FIRST_POINT -> 1
                TrafficBuilderState.SECOND_POINT -> 2
            }
        }

        if (numPoints < 2) return
        if (numPoints == 2) builder.position0.mix(builder.position1, 2.0, builder.position2)
        for (lane in builder.createStreetInExpertMode(builder.createStreetInPlannerMode()).lanes) {
            drawLine(lane)
        }
    }

    fun drawLine(lane: Lane) {
        // draw bezier shape
        val n = 10
        val positions = List(n + 1) { index ->
            lane.getPosition(index.toDouble() / n, Vector3d())
        }
        for (i in 0 until n) {
            val p0 = positions[i]
            val p1 = positions[i + 1]
            drawLine(p0, p1)
        }
    }

    fun drawLine(street: Street) {
        // draw bezier shape
        val n = 10
        val positions = List(n + 1) { index ->
            street.getPosition(index.toDouble() / n, Vector3d())
        }
        for (i in 0 until n) {
            val p0 = positions[i]
            val p1 = positions[i + 1]
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
                    TrafficBuilderState.NO_POINTS -> builder.position0
                    TrafficBuilderState.FIRST_POINT -> builder.position1
                    TrafficBuilderState.SECOND_POINT -> builder.position2
                }

                dst.set(query.result.positionWS)
                if (state == TrafficBuilderState.SECOND_POINT) {
                    builder.placeStreetInExpertMode()
                }

                state = when (state) {
                    TrafficBuilderState.NO_POINTS -> TrafficBuilderState.FIRST_POINT
                    TrafficBuilderState.FIRST_POINT -> TrafficBuilderState.SECOND_POINT
                    TrafficBuilderState.SECOND_POINT -> {
                        // don't reuse middle
                        builder.position0.set(builder.position2)
                        TrafficBuilderState.FIRST_POINT
                    }
                }
            }
            Key.BUTTON_RIGHT -> {
                state = when (state) {
                    TrafficBuilderState.NO_POINTS -> TrafficBuilderState.NO_POINTS
                    TrafficBuilderState.FIRST_POINT -> TrafficBuilderState.NO_POINTS
                    TrafficBuilderState.SECOND_POINT -> TrafficBuilderState.FIRST_POINT
                }
            }
            else -> {}
        }
    }

}