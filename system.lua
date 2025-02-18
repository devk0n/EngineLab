system = {
    ground = {
        position = {0, 0, 0}, -- Origin of the ground
    },

    bodies = {
        link1 = {
            position = {0.5, 0.0, 0.0}, -- x, y, z, position in meters
            orientation = {0.0, 0.0, 0.0}, -- x, y, z, euler rotation in degrees
            size = {0.50, 0.10, 0.05}, -- x, y, z distance in meters
            mass = 13.6, -- mass in kg
            inertia = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}, -- local inertia tensor
        }
    },

    joints = {
        {
            type = "fixed", -- Ground is fixed, so this joint anchors link1 to it
            body1 = "ground",
            body2 = "link1",
            position = {0, 0, 0} -- Joint location (relative to ground)
        },
        {
            type = "revolute",
            body1 = "link1",
            body2 = "link2",
            axis = {0, 0, 1}, -- Rotation around Z
            position = {system.bodies.link1.length, 0, 0}
        }
    }
}
