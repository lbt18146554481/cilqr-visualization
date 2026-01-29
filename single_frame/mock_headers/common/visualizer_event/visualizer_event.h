/*
 * Mock header for VisualizerEvent
 */

#pragma once

#include <string>

// FLAGS for visualization
extern bool FLAGS_planning_enable_vis_event;

namespace ceshi {
namespace planning {
namespace visualizer {

class Event {
public:
    enum Type {
        k3D = 0
    };
    enum Attribute {
        kOdom = 0
    };
    
    void set_type(Type type) {}
    void add_attribute(Attribute attr) {}
    
    class SphereContainer {
    public:
        class Sphere {
        public:
            class Center {
            public:
                void set_x(double x) {}
                void set_y(double y) {}
                void set_z(double z) {}
            };
            Center* mutable_center() {
                static Center center;
                return &center;
            }
            void set_radius(double r) {}
        };
        Sphere* Add() {
            static Sphere sphere;
            return &sphere;
        }
    };
    
    SphereContainer* mutable_sphere() {
        static SphereContainer container;
        return &container;
    }
};

class EventSender {
public:
    static EventSender* Instance() {
        static EventSender instance;
        return &instance;
    }
    
    Event* GetEvent(const std::string& name) {
        static Event event;
        return &event;
    }
};

} // namespace visualizer
} // namespace planning
} // namespace ceshi

// Alias for vis namespace (used in trajectory_smoother)
namespace vis = ceshi::planning::visualizer;
