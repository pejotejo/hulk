use super::*;

pub(super) fn add_landscape(arena: &mut Layout, end: f32, side: f32) {
    arena.block(
        Vec3::new(0.0, -0.29, -8.0),
        Vec3::new(110.0, 0.12, 100.0),
        GRASS,
    );

    // The Elbe and both quays sit beyond the far stands, clear of camera paths.
    let river = -side - 11.0;
    arena.block(
        Vec3::new(0.0, -0.16, river),
        Vec3::new(106.0, 0.06, 7.0),
        WATER,
    );
    for z in [river - 4.0, river + 4.0] {
        arena.block(Vec3::new(0.0, -0.13, z), Vec3::new(106.0, 0.18, 0.9), SAND);
        arena.block(
            Vec3::new(0.0, -0.01, z),
            Vec3::new(106.0, 0.08, 0.16),
            CONCRETE,
        );
    }
    for index in 0..19 {
        let x = (index as f32 - 9.0) * 4.8;
        arena.block(
            Vec3::new(x, -0.126, river + (index % 4) as f32 * 1.25 - 2.0),
            Vec3::new(1.1 + (index % 3) as f32 * 0.6, 0.005, 0.045),
            WATER_LIGHT,
        );
        arena.block(
            Vec3::new(x, 0.12, river + 3.9),
            Vec3::new(0.12, 0.3, 0.12),
            STEEL,
        );
    }

    let warehouse_z = river - 7.5;
    for (index, height) in [4.3, 5.0, 4.5, 5.3, 4.1, 4.8, 4.4].into_iter().enumerate() {
        warehouse(
            arena,
            Vec3::new((index as f32 - 3.0) * 6.2, 0.0, warehouse_z),
            height,
            if index % 3 == 0 { BRICK_LIGHT } else { BRICK },
        );
    }
    crane(arena, Vec3::new(-end - 8.0, 0.0, river - 3.8), 1.0);
    crane(arena, Vec3::new(end + 7.0, 0.0, river - 3.8), 0.8);
    michaelis(arena, Vec3::new(end + 15.0, 0.0, warehouse_z - 5.0));

    for sign in [-1.0, 1.0] {
        let park_x = sign * (end + 6.0);
        arena.block(
            Vec3::new(park_x, -0.15, 0.5),
            Vec3::new(4.2, 0.08, 2.0 * side + 13.0),
            SAND,
        );
        for index in 0..6 {
            let z = -side - 3.0 + index as f32 * 3.1;
            let x = park_x + sign * if index % 2 == 0 { 2.5 } else { -2.5 };
            tree(
                arena,
                Vec3::new(x, 0.0, z),
                0.85 + (index % 3) as f32 * 0.12,
                index % 2 == 0,
            );
        }
        for z in [-side + 1.0, side + 2.0] {
            let x = park_x + sign * 1.2;
            arena.block(Vec3::new(x, 0.36, z), Vec3::new(0.42, 0.12, 1.25), BARK);
            arena.block(
                Vec3::new(x + sign * 0.19, 0.57, z),
                Vec3::new(0.08, 0.36, 1.25),
                BARK,
            );
            for offset in [-0.45, 0.45] {
                arena.block(
                    Vec3::new(x, 0.15, z + offset),
                    Vec3::new(0.32, 0.3, 0.07),
                    STEEL,
                );
            }
        }
    }
    arena.shape(
        Vec3::new(-end - 21.0, 19.0, -side - 35.0),
        Vec3::splat(4.2),
        SUN,
        Shape::Sphere,
    );
}

fn warehouse(arena: &mut Layout, base: Vec3, height: f32, brick: usize) {
    let width = 4.9;
    let depth = 4.0;
    arena.block(
        base + Vec3::Y * height / 2.0,
        Vec3::new(width, height, depth),
        brick,
    );
    arena.shape(
        base + Vec3::new(0.0, height + 0.75, 0.0),
        Vec3::new(width + 0.3, 1.5, depth + 0.3),
        ROOF,
        Shape::Gable,
    );
    arena.shape(
        base + Vec3::new(0.0, height + 0.62, depth / 2.0 + 0.05),
        Vec3::new(width - 0.15, 1.24, 0.1),
        brick,
        Shape::Gable,
    );
    let front = depth / 2.0 + 0.08;
    for floor in 0..4 {
        arena.block(
            base + Vec3::new(0.0, floor as f32 * height / 4.0 + 0.12, front),
            Vec3::new(width + 0.12, 0.09, 0.14),
            SAND,
        );
        for bay in 0..4 {
            let x = (bay as f32 - 1.5) * 1.13;
            arena.block(
                base + Vec3::new(x, (floor as f32 + 0.55) * height / 4.0, front),
                Vec3::new(0.58, height / 4.0 * 0.62, 0.09),
                WINDOW,
            );
        }
    }
    for x in [-2.25, 0.0, 2.25] {
        arena.block(
            base + Vec3::new(x, height / 2.0, front + 0.02),
            Vec3::new(0.13, height, 0.13),
            BRICK_LIGHT,
        );
    }
    arena.block(
        base + Vec3::new(0.0, 0.7, front + 0.03),
        Vec3::new(0.8, 1.4, 0.12),
        ROOF,
    );
    arena.block(
        base + Vec3::new(0.0, height + 0.42, front + 0.03),
        Vec3::new(0.36, 0.48, 0.13),
        WINDOW,
    );
    arena.block(
        base + Vec3::new(width * 0.3, height + 1.2, -0.6),
        Vec3::new(0.36, 1.05, 0.5),
        brick,
    );
}

fn crane(arena: &mut Layout, base: Vec3, scale: f32) {
    let point = |x, y, z| base + Vec3::new(x, y, z) * scale;
    for x in [-0.7, 0.7] {
        for z in [-0.6, 0.6] {
            arena.beam(
                point(x, 0.0, z),
                point(x * 0.35, 5.1, z * 0.35),
                0.13 * scale,
                COPPER,
            );
        }
        arena.beam(
            point(x, 0.8, -0.5),
            point(-x * 0.6, 3.5, -0.3),
            0.065 * scale,
            STEEL,
        );
    }
    arena.block(
        point(0.0, 5.2, 0.0),
        Vec3::new(1.1, 0.75, 0.9) * scale,
        COPPER,
    );
    arena.block(
        point(0.0, 5.3, 0.46),
        Vec3::new(0.8, 0.4, 0.04) * scale,
        WINDOW,
    );
    arena.beam(
        point(-4.5, 7.7, 0.0),
        point(2.2, 5.8, 0.0),
        0.16 * scale,
        COPPER,
    );
    arena.beam(
        point(-4.5, 7.3, 0.0),
        point(2.2, 5.4, 0.0),
        0.1 * scale,
        STEEL,
    );
    for index in 0..7 {
        let x = -4.5 + index as f32 * 0.95;
        let y = 7.7 - index as f32 * 0.27;
        arena.beam(
            point(x, y, 0.0),
            point(x + 0.95, y - 0.67, 0.0),
            0.06 * scale,
            STEEL,
        );
    }
    arena.beam(
        point(0.0, 5.6, 0.0),
        point(0.0, 8.6, 0.0),
        0.11 * scale,
        COPPER,
    );
    arena.beam(
        point(0.0, 8.6, 0.0),
        point(-4.5, 7.7, 0.0),
        0.035 * scale,
        STEEL,
    );
    arena.beam(
        point(0.0, 8.6, 0.0),
        point(2.2, 5.8, 0.0),
        0.035 * scale,
        STEEL,
    );
    arena.beam(
        point(-4.3, 7.5, 0.0),
        point(-4.3, 2.0, 0.0),
        0.025 * scale,
        STEEL,
    );
    arena.block(
        point(-4.3, 1.9, 0.0),
        Vec3::new(0.25, 0.2, 0.22) * scale,
        STEEL,
    );
    arena.block(
        point(1.8, 5.5, 0.0),
        Vec3::new(0.7, 0.6, 0.65) * scale,
        CONCRETE,
    );
}

fn tree(arena: &mut Layout, base: Vec3, scale: f32, light: bool) {
    arena.block(
        base + Vec3::Y * 0.85 * scale,
        Vec3::new(0.18, 1.7, 0.18) * scale,
        BARK,
    );
    arena.shape(
        base + Vec3::Y * 2.1 * scale,
        Vec3::new(2.0, 2.6, 1.8) * scale,
        if light { LEAF_LIGHT } else { LEAF },
        Shape::Crown,
    );
    arena.shape(
        base + Vec3::new(0.3, 2.1, 0.15) * scale,
        Vec3::new(1.5, 1.8, 1.5) * scale,
        if light { LEAF } else { LEAF_LIGHT },
        Shape::Crown,
    );
}

fn michaelis(arena: &mut Layout, base: Vec3) {
    arena.block(base + Vec3::Y * 1.8, Vec3::new(5.0, 3.6, 5.5), BRICK);
    arena.shape(
        base + Vec3::Y * 4.1,
        Vec3::new(5.4, 1.0, 5.9),
        ROOF,
        Shape::Gable,
    );
    arena.block(base + Vec3::Y * 4.1, Vec3::new(2.2, 8.2, 2.2), BRICK_LIGHT);
    for height in [5.2, 7.8, 8.5] {
        arena.block(base + Vec3::Y * height, Vec3::new(2.45, 0.16, 2.45), SAND);
    }
    arena.block(
        base + Vec3::new(0.0, 7.0, 1.12),
        Vec3::new(0.85, 0.85, 0.04),
        SAND,
    );
    arena.block(
        base + Vec3::new(0.0, 7.1, 1.15),
        Vec3::new(0.035, 0.34, 0.03),
        ROOF,
    );
    arena.block(
        base + Vec3::new(0.15, 6.95, 1.15),
        Vec3::new(0.3, 0.035, 0.03),
        ROOF,
    );
    arena.block(base + Vec3::Y * 8.6, Vec3::new(1.1, 1.0, 1.1), ROOF);
    arena.shape(
        base + Vec3::Y * 9.4,
        Vec3::new(1.8, 1.9, 1.8),
        COPPER,
        Shape::Sphere,
    );
    arena.shape(
        base + Vec3::Y * 11.0,
        Vec3::new(1.15, 4.6, 1.15),
        COPPER,
        Shape::Crown,
    );
    arena.beam(base + Vec3::Y * 13.0, base + Vec3::Y * 13.9, 0.045, STEEL);
    arena.beam(
        base + Vec3::new(-0.23, 13.65, 0.0),
        base + Vec3::new(0.23, 13.65, 0.0),
        0.045,
        STEEL,
    );
}
