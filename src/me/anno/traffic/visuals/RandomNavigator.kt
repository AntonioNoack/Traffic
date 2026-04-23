package me.anno.traffic.visuals

import me.anno.ecs.Component
import me.anno.ecs.systems.OnUpdate
import me.anno.traffic.Network

class RandomNavigator(val network: Network) : Component(), OnUpdate {
    override fun onUpdate() {
        for (vehicle in network.vehicles) {
            if (vehicle.routeIndex > 0) {
                vehicle.route.removeFirst()
                vehicle.routeIndex--
            }
            if (vehicle.routeIndex + 3 > vehicle.route.size) {
                extendRoute(vehicle)
            }
        }
    }
}