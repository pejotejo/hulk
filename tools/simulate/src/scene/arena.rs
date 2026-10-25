//! Static arena decoration shared by the Bevy viewport and headless MuJoCo renderer.
mod landscape;

use std::{f32::consts::PI, fmt::Write};

use bevy::{
    asset::RenderAssetUsages,
    image::{CompressedImageFormats, ImageSampler, ImageType},
    light::NotShadowCaster,
    mesh::PrimitiveTopology,
    prelude::*,
};
use mujoco_rs::prelude::MjSpec;
use types::field_dimensions::FieldDimensions;

use crate::{
    parameters::CurrentSimulatorParameters,
    presentation::{Presentation, PresentationUpdateSet},
    scene::SceneParameterUpdateSet,
};

const ASPHALT: usize = 0;
const CONCRETE: usize = 1;
const NAVY: usize = 2;
const TEAL: usize = 3;
const WHITE: usize = 4;
const STEEL: usize = 5;
const TURF: usize = 6;
const TURF_LIGHT: usize = 7;
const LAMP: usize = 8;
const BRICK: usize = 9;
const BRICK_LIGHT: usize = 10;
const ROOF: usize = 11;
const WINDOW: usize = 12;
const WATER: usize = 13;
const WATER_LIGHT: usize = 14;
const GRASS: usize = 15;
const LEAF: usize = 16;
const LEAF_LIGHT: usize = 17;
const BARK: usize = 18;
const COPPER: usize = 19;
const SAND: usize = 20;
const SUN: usize = 21;
const COLORS: [[f32; 3]; 22] = [
    [0.055, 0.075, 0.095],
    [0.32, 0.37, 0.40],
    [0.035, 0.075, 0.14],
    [0.025, 0.47, 0.53],
    [0.88, 0.91, 0.92],
    [0.16, 0.21, 0.25],
    [0.09, 0.32, 0.17],
    [0.115, 0.365, 0.195],
    [0.93, 0.97, 1.0],
    [0.42, 0.22, 0.17],
    [0.55, 0.31, 0.23],
    [0.19, 0.25, 0.27],
    [0.18, 0.31, 0.35],
    [0.24, 0.43, 0.49],
    [0.36, 0.55, 0.59],
    [0.31, 0.40, 0.31],
    [0.19, 0.34, 0.25],
    [0.30, 0.44, 0.29],
    [0.31, 0.26, 0.21],
    [0.31, 0.49, 0.44],
    [0.65, 0.63, 0.55],
    [1.0, 0.88, 0.65],
];
pub(crate) const BRAND_COUNT: usize = 9;
pub(crate) const BRAND_CADENCE_SECONDS: f64 = 12.0;
const BRANDS: [(&str, &[u8]); BRAND_COUNT] = [
    (
        "hulks_panel.png",
        include_bytes!("../../assets/branding/hulks_panel.png"),
    ),
    (
        "kdab_panel.png",
        include_bytes!("../../assets/branding/kdab_panel.png"),
    ),
    (
        "cc_panel.png",
        include_bytes!("../../assets/branding/cc_panel.png"),
    ),
    (
        "draeger_panel.png",
        include_bytes!("../../assets/branding/draeger_panel.png"),
    ),
    (
        "gea_panel.png",
        include_bytes!("../../assets/branding/gea_panel.png"),
    ),
    (
        "haspa_panel.png",
        include_bytes!("../../assets/branding/haspa_panel.png"),
    ),
    (
        "tk_panel.png",
        include_bytes!("../../assets/branding/tk_panel.png"),
    ),
    (
        "gegenstrom_panel.png",
        include_bytes!("../../assets/branding/gegenstrom_panel.png"),
    ),
    (
        "tuhh_panel.png",
        include_bytes!("../../assets/branding/tuhh_panel.png"),
    ),
];
const PANEL_HEIGHT: f32 = 0.48;

/// View-only rotation, staggered across board groups and deterministic when seeking replay.
pub(crate) fn brand_index(initial: usize, display_time: f64) -> usize {
    let initial = initial % BRAND_COUNT;
    let time = if display_time.is_finite() && display_time > 0.0 {
        display_time % (BRAND_CADENCE_SECONDS * BRAND_COUNT as f64)
    } else {
        0.0
    };
    let offset = (initial * 5 % BRAND_COUNT) as f64;
    (initial + ((time + offset) / BRAND_CADENCE_SECONDS).floor() as usize) % BRAND_COUNT
}

#[derive(Clone, Copy)]
enum Shape {
    Box,
    Gable,
    Crown,
    Sphere,
}

struct Block {
    position: Vec3,
    size: Vec3,
    finish: usize,
    rotation: Quat,
    shape: Shape,
}

struct Panel {
    position: Vec3,
    width: f32,
    yaw: f32,
    brand: usize,
}

#[derive(Default)]
struct Layout {
    blocks: Vec<Block>,
    panels: Vec<Panel>,
}

impl Layout {
    fn block(&mut self, position: Vec3, size: Vec3, finish: usize) {
        self.shape(position, size, finish, Shape::Box);
    }

    fn shape(&mut self, position: Vec3, size: Vec3, finish: usize, shape: Shape) {
        self.blocks.push(Block {
            position,
            size,
            finish,
            rotation: Quat::IDENTITY,
            shape,
        });
    }

    fn beam(&mut self, start: Vec3, end: Vec3, thickness: f32, finish: usize) {
        let direction = end - start;
        self.blocks.push(Block {
            position: (start + end) / 2.0,
            size: Vec3::new(thickness, direction.length(), thickness),
            rotation: Quat::from_rotation_arc(Vec3::Y, direction.normalize()),
            finish,
            shape: Shape::Box,
        });
    }
}

fn layout(field: &FieldDimensions) -> Layout {
    let mut arena = Layout::default();
    // Keep all furniture beyond both the carpet runoff and the backs of the goals.
    let end = field.length / 2.0 + field.border_strip_width.max(field.goal_depth) + 0.55;
    let side = field.width / 2.0 + field.border_strip_width + 0.55;
    arena.block(
        Vec3::new(0.0, -0.10, 0.0),
        Vec3::new(2.0 * end + 2.4, 0.16, 2.0 * side + 5.0),
        ASPHALT,
    );
    arena.block(
        Vec3::new(0.0, -0.025, 0.0),
        Vec3::new(2.0 * end + 0.6, 0.04, 2.0 * side + 0.6),
        CONCRETE,
    );
    arena.block(
        Vec3::new(0.0, 0.00155, 0.0),
        Vec3::new(
            field.length + 2.0 * field.border_strip_width,
            0.0001,
            field.width + 2.0 * field.border_strip_width,
        ),
        TURF,
    );
    for index in (0..12).step_by(2) {
        arena.block(
            Vec3::new(
                -field.length / 2.0 + (index as f32 + 0.5) * field.length / 12.0,
                0.0017,
                0.0,
            ),
            Vec3::new(field.length / 12.0, 0.0001, field.width),
            TURF_LIGHT,
        );
    }

    for sign in [-1.0, 1.0] {
        arena.block(
            Vec3::new(0.0, 0.31, sign * side),
            Vec3::new(end * 2.0, 0.62, 0.12),
            NAVY,
        );
        arena.block(
            Vec3::new(0.0, 0.64, sign * side),
            Vec3::new(end * 2.0 + 0.12, 0.045, 0.17),
            TEAL,
        );
        arena.block(
            Vec3::new(sign * end, 0.31, 0.0),
            Vec3::new(0.12, 0.62, side * 2.0),
            NAVY,
        );
        arena.block(
            Vec3::new(sign * end, 0.64, 0.0),
            Vec3::new(0.17, 0.045, side * 2.0),
            TEAL,
        );

        let long_count = ((2.0 * end / 1.7).round() as usize).max(8);
        for index in 0..long_count {
            let spacing = 2.0 * end / long_count as f32;
            arena.panels.push(Panel {
                position: Vec3::new(
                    -end + (index as f32 + 0.5) * spacing,
                    0.33,
                    sign * (side - 0.066),
                ),
                width: (spacing - 0.07).min(PANEL_HEIGHT * 3.0),
                yaw: if sign < 0.0 { 0.0 } else { PI },
                brand: index % BRANDS.len(),
            });
        }
        let end_count = ((2.0 * side / 1.7).round() as usize).max(6);
        for index in 0..end_count {
            let spacing = 2.0 * side / end_count as f32;
            arena.panels.push(Panel {
                position: Vec3::new(
                    sign * (end - 0.066),
                    0.33,
                    -side + (index as f32 + 0.5) * spacing,
                ),
                width: (spacing - 0.07).min(PANEL_HEIGHT * 3.0),
                yaw: -sign * PI / 2.0,
                brand: (index + 3) % BRANDS.len(),
            });
        }

        // Three low rows leave the match visible from the default elevated camera.
        for row in 0..3 {
            let z = sign * (side + 0.75 + row as f32 * 0.52);
            let height = 0.18 + row as f32 * 0.20;
            arena.block(
                Vec3::new(0.0, height / 2.0, z),
                Vec3::new(end * 1.72, height, 0.52),
                CONCRETE,
            );
            let seats = ((end * 1.72 / 0.48).floor() as usize).max(4);
            for seat in 0..seats {
                if seat == seats / 2 || seat == seats / 2 - 1 {
                    continue;
                }
                let x = -end * 0.86 + (seat as f32 + 0.5) * end * 1.72 / seats as f32;
                let finish = match (seat / 4 + row) % 5 {
                    0 => WHITE,
                    1 | 2 => TEAL,
                    _ => NAVY,
                };
                arena.block(
                    Vec3::new(x, height + 0.05, z),
                    Vec3::new(0.36, 0.09, 0.34),
                    finish,
                );
                arena.block(
                    Vec3::new(x, height + 0.22, z + sign * 0.14),
                    Vec3::new(0.36, 0.30, 0.06),
                    finish,
                );
            }
        }
        for x_sign in [-1.0, 1.0] {
            let x = x_sign * (end + 0.40);
            let z = sign * (side + 0.15);
            arena.block(Vec3::new(x, 1.6, z), Vec3::new(0.07, 3.2, 0.07), STEEL);
            arena.block(Vec3::new(x, 3.2, z), Vec3::new(0.64, 0.12, 0.28), NAVY);
            arena.block(Vec3::new(x, 3.13, z), Vec3::new(0.56, 0.025, 0.22), LAMP);
        }
    }
    landscape::add_landscape(&mut arena, end, side);
    arena
}

#[derive(Resource)]
struct ArenaAssets {
    surfaces: Vec<Handle<StandardMaterial>>,
    brands: Vec<Handle<StandardMaterial>>,
}

impl FromWorld for ArenaAssets {
    fn from_world(world: &mut World) -> Self {
        let surfaces = COLORS
            .iter()
            .enumerate()
            .map(|(index, color)| {
                world
                    .resource_mut::<Assets<StandardMaterial>>()
                    .add(StandardMaterial {
                        base_color: Color::srgb(color[0], color[1], color[2]),
                        perceptual_roughness: 0.9,
                        unlit: index == LAMP || index == SUN,
                        ..default()
                    })
            })
            .collect();
        let brands = BRANDS
            .iter()
            .map(|(_, bytes)| {
                let image = Image::from_buffer(
                    bytes,
                    ImageType::Extension("png"),
                    CompressedImageFormats::NONE,
                    true,
                    ImageSampler::default(),
                    RenderAssetUsages::default(),
                )
                .expect("decode bundled arena branding");
                let texture = world.resource_mut::<Assets<Image>>().add(image);
                world
                    .resource_mut::<Assets<StandardMaterial>>()
                    .add(StandardMaterial {
                        base_color_texture: Some(texture),
                        perceptual_roughness: 0.9,
                        ..default()
                    })
            })
            .collect();
        Self { surfaces, brands }
    }
}

#[derive(Component)]
struct Arena;

#[derive(Component)]
struct BrandPanel(usize);

#[derive(Default, Resource)]
struct ArenaDimensions(Option<[f32; 4]>);

pub(super) struct ArenaPlugin;

impl Plugin for ArenaPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<ArenaAssets>()
            .init_resource::<ArenaDimensions>()
            .insert_resource(ClearColor(Color::srgb(0.70, 0.81, 0.84)))
            .insert_resource(GlobalAmbientLight {
                color: Color::srgb(0.80, 0.89, 1.0),
                brightness: 160.0,
                ..default()
            })
            .add_systems(PreUpdate, update_arena.in_set(SceneParameterUpdateSet))
            .add_systems(PreUpdate, configure_sun)
            .add_systems(Update, rotate_brands.after(PresentationUpdateSet));
    }
}

fn configure_sun(
    mut lights: Query<(&mut DirectionalLight, &mut Transform), Added<DirectionalLight>>,
) {
    for (mut light, mut transform) in &mut lights {
        light.illuminance = 15_000.0;
        light.color = Color::srgb(1.0, 0.96, 0.87);
        light.shadow_maps_enabled = false;
        *transform = Transform::from_xyz(-8.0, 14.0, -12.0).looking_at(Vec3::ZERO, Vec3::Y);
    }
}

fn update_arena(
    mut commands: Commands,
    parameters: Res<CurrentSimulatorParameters>,
    assets: Res<ArenaAssets>,
    mut dimensions: ResMut<ArenaDimensions>,
    roots: Query<Entity, With<Arena>>,
    mut meshes: ResMut<Assets<Mesh>>,
    presentation: Res<Presentation>,
) {
    let field = &parameters.field_dimensions;
    let current = [
        field.length,
        field.width,
        field.border_strip_width,
        field.goal_depth,
    ];
    if dimensions.0 == Some(current) {
        return;
    }
    dimensions.0 = Some(current);
    for root in &roots {
        commands.entity(root).despawn();
    }
    let batches = arena_meshes(field);
    commands
        .spawn((Arena, Transform::default(), Visibility::default()))
        .with_children(|parent| {
            for (index, mesh) in batches.into_iter().enumerate() {
                let brand = index.checked_sub(COLORS.len());
                let material = match brand {
                    Some(initial) => {
                        &assets.brands[brand_index(initial, presentation.display_time())]
                    }
                    None => &assets.surfaces[index],
                };
                let mut entity = parent.spawn((
                    Mesh3d(meshes.add(mesh.into_bevy())),
                    MeshMaterial3d(material.clone()),
                    Transform::default(),
                    NotShadowCaster,
                ));
                if let Some(initial) = brand {
                    entity.insert(BrandPanel(initial));
                }
            }
        });
}

fn rotate_brands(
    presentation: Res<Presentation>,
    assets: Res<ArenaAssets>,
    mut panels: Query<(&BrandPanel, &mut MeshMaterial3d<StandardMaterial>)>,
) {
    let time = presentation.display_time();
    for (panel, mut material) in &mut panels {
        let next = &assets.brands[brand_index(panel.0, time)];
        if material.0 != *next {
            material.0 = next.clone();
        }
    }
}

/// Non-colliding decoration, including turf; attach the returned `arena` root body.
pub(crate) fn arena_spec(field: FieldDimensions) -> Result<MjSpec, String> {
    MjSpec::from_xml_string(&arena_xml(&field)).map_err(|error| error.to_string())
}

fn arena_meshes(field: &FieldDimensions) -> Vec<ArenaMesh> {
    let arena = layout(field);
    let mut meshes: Vec<ArenaMesh> = (0..COLORS.len() + BRANDS.len())
        .map(|_| ArenaMesh::default())
        .collect();
    for block in arena.blocks {
        meshes[block.finish].add_block(&block);
    }
    for panel in arena.panels {
        meshes[COLORS.len() + panel.brand].add_box(
            panel.position,
            Vec3::new(panel.width, panel.width / 3.0, 0.004),
            panel.yaw,
        );
    }
    meshes
}

fn arena_xml(field: &FieldDimensions) -> String {
    let meshes = arena_meshes(field);
    let mut xml = String::from(
        "<mujoco><asset><texture name=\"arena_sky\" type=\"skybox\" builtin=\"gradient\" rgb1=\"0.36 0.57 0.70\" rgb2=\"0.83 0.90 0.90\" width=\"128\" height=\"768\"/>",
    );
    let directory = concat!(env!("CARGO_MANIFEST_DIR"), "/assets/branding/");
    let [r, g, b] = COLORS[SUN];
    write!(
        xml,
        "<material name=\"arena_sun_glow\" rgba=\"{r} {g} {b} 1\" emission=\"1\" specular=\"0\"/>"
    )
    .unwrap();
    for (index, (file, _)) in BRANDS.iter().enumerate() {
        let path = format!("{directory}{file}")
            .replace('&', "&amp;")
            .replace('"', "&quot;");
        write!(xml, "<texture name=\"brand_{index}\" type=\"2d\" file=\"{path}\"/><material name=\"brand_{index}\" texture=\"brand_{index}\" texuniform=\"false\" specular=\"0\" shininess=\"0\"/>").unwrap();
    }
    // Both renderers use one static mesh per finish or initial board brand.
    for (index, mesh) in meshes.iter().enumerate() {
        write!(xml, "<mesh name=\"part_{index}\" vertex=\"").unwrap();
        for [x, y, z] in &mesh.vertices {
            // Bevy is Y-up; MuJoCo uses field coordinates (X, -Z, Y).
            write!(xml, "{x} {} {y} ", -z).unwrap();
        }
        xml.push_str("\" face=\"");
        for [a, b, c] in &mesh.faces {
            write!(xml, "{a} {b} {c} ").unwrap();
        }
        xml.push_str("\" texcoord=\"");
        for [u, v] in &mesh.texcoords {
            write!(xml, "{u} {v} ").unwrap();
        }
        xml.push_str("\"/>");
    }
    xml.push_str("</asset><worldbody><body name=\"arena\"><light name=\"arena_sun\" directional=\"true\" castshadow=\"false\" pos=\"-8 12 14\" dir=\"8 -12 -14\" ambient=\"0.08 0.09 0.10\" diffuse=\"0.70 0.672 0.609\" specular=\"0.15 0.14 0.12\"/>");
    for index in 0..meshes.len() {
        let (name, finish) = if index == SUN {
            (
                format!("arena_finish_{index}"),
                "material=\"arena_sun_glow\"".to_owned(),
            )
        } else if index < COLORS.len() {
            let [r, g, b] = COLORS[index];
            (
                format!("arena_finish_{index}"),
                format!("rgba=\"{r} {g} {b} 1\""),
            )
        } else {
            let initial = index - COLORS.len();
            (
                format!("arena_brand_{initial}"),
                format!("material=\"brand_{initial}\""),
            )
        };
        write!(
            xml,
            "<geom name=\"{name}\" type=\"mesh\" mesh=\"part_{index}\" {finish} contype=\"0\" conaffinity=\"0\" group=\"5\"/>"
        )
        .unwrap();
    }
    xml.push_str("</body></worldbody></mujoco>");
    xml
}

#[derive(Default)]
struct ArenaMesh {
    vertices: Vec<[f32; 3]>,
    faces: Vec<[usize; 3]>,
    texcoords: Vec<[f32; 2]>,
}

impl ArenaMesh {
    fn add_box(&mut self, position: Vec3, size: Vec3, yaw: f32) {
        self.add_block(&Block {
            position,
            size,
            rotation: Quat::from_rotation_y(yaw),
            shape: Shape::Box,
            finish: 0,
        });
    }

    fn add_block(&mut self, block: &Block) {
        match block.shape {
            Shape::Box => {
                let base = self.vertices.len();
                self.add_polyhedron(
                    block,
                    &[
                        [-0.5, -0.5, 0.5],
                        [0.5, -0.5, 0.5],
                        [0.5, 0.5, 0.5],
                        [-0.5, 0.5, 0.5],
                        [-0.5, -0.5, -0.5],
                        [0.5, -0.5, -0.5],
                        [0.5, 0.5, -0.5],
                        [-0.5, 0.5, -0.5],
                    ],
                    &[
                        [0, 1, 2],
                        [0, 2, 3],
                        [5, 4, 7],
                        [5, 7, 6],
                        [4, 0, 3],
                        [4, 3, 7],
                        [1, 5, 6],
                        [1, 6, 2],
                        [3, 2, 6],
                        [3, 6, 7],
                        [4, 5, 1],
                        [4, 1, 0],
                    ],
                );
                self.texcoords[base..].copy_from_slice(&[
                    [0.0, 1.0],
                    [1.0, 1.0],
                    [1.0, 0.0],
                    [0.0, 0.0],
                    [1.0, 1.0],
                    [0.0, 1.0],
                    [0.0, 0.0],
                    [1.0, 0.0],
                ]);
            }
            Shape::Gable => self.add_polyhedron(
                block,
                &[
                    [-0.5, -0.5, -0.5],
                    [0.5, -0.5, -0.5],
                    [0.0, 0.5, -0.5],
                    [-0.5, -0.5, 0.5],
                    [0.5, -0.5, 0.5],
                    [0.0, 0.5, 0.5],
                ],
                &[
                    [0, 2, 1],
                    [3, 4, 5],
                    [0, 1, 4],
                    [0, 4, 3],
                    [0, 3, 5],
                    [0, 5, 2],
                    [1, 2, 5],
                    [1, 5, 4],
                ],
            ),
            Shape::Crown => self.add_polyhedron(
                block,
                &[
                    [0.0, 0.5, 0.0],
                    [0.5, 0.0, 0.0],
                    [0.0, 0.0, 0.5],
                    [-0.5, 0.0, 0.0],
                    [0.0, 0.0, -0.5],
                    [0.0, -0.5, 0.0],
                ],
                &[
                    [0, 2, 1],
                    [0, 3, 2],
                    [0, 4, 3],
                    [0, 1, 4],
                    [5, 1, 2],
                    [5, 2, 3],
                    [5, 3, 4],
                    [5, 4, 1],
                ],
            ),
            Shape::Sphere => self.add_sphere(block),
        }
    }

    fn add_polyhedron(&mut self, block: &Block, vertices: &[[f32; 3]], faces: &[[usize; 3]]) {
        let base = self.vertices.len();
        self.vertices.extend(vertices.iter().map(|vertex| {
            (block.position + block.rotation * (Vec3::from_array(*vertex) * block.size)).to_array()
        }));
        self.texcoords.resize(self.vertices.len(), [0.0, 0.0]);
        self.faces
            .extend(faces.iter().map(|face| face.map(|index| base + index)));
    }

    fn add_sphere(&mut self, block: &Block) {
        const SEGMENTS: usize = 16;
        let mut vertices = vec![[0.0, 0.5, 0.0], [0.0, -0.5, 0.0]];
        for latitude in 1..8 {
            let (sin, cos) = (latitude as f32 * PI / 8.0).sin_cos();
            for longitude in 0..SEGMENTS {
                let (azimuth_sin, azimuth_cos) =
                    (longitude as f32 * 2.0 * PI / SEGMENTS as f32).sin_cos();
                vertices.push([sin * azimuth_cos / 2.0, cos / 2.0, sin * azimuth_sin / 2.0]);
            }
        }
        let mut faces = Vec::new();
        for index in 0..SEGMENTS {
            let next = (index + 1) % SEGMENTS;
            faces.push([0, 2 + next, 2 + index]);
            faces.push([1, 2 + 6 * SEGMENTS + index, 2 + 6 * SEGMENTS + next]);
            for ring in 0..6 {
                let a = 2 + ring * SEGMENTS + index;
                let b = 2 + ring * SEGMENTS + next;
                faces.push([a, b, b + SEGMENTS]);
                faces.push([a, b + SEGMENTS, a + SEGMENTS]);
            }
        }
        self.add_polyhedron(block, &vertices, &faces);
    }

    fn into_bevy(self) -> Mesh {
        let positions: Vec<_> = self
            .faces
            .iter()
            .flatten()
            .map(|index| self.vertices[*index])
            .collect();
        let texcoords: Vec<_> = self
            .faces
            .iter()
            .flatten()
            .map(|index| self.texcoords[*index])
            .collect();
        let mut mesh = Mesh::new(
            PrimitiveTopology::TriangleList,
            RenderAssetUsages::RENDER_WORLD,
        )
        .with_inserted_attribute(Mesh::ATTRIBUTE_POSITION, positions)
        .with_inserted_attribute(Mesh::ATTRIBUTE_UV_0, texcoords);
        mesh.compute_flat_normals();
        mesh
    }
}

#[cfg(test)]
mod tests {
    use mujoco_rs::prelude::MjtObj;

    use super::*;

    fn half_extents(block: &Block) -> Vec3 {
        ((block.rotation * Vec3::X).abs() * block.size.x
            + (block.rotation * Vec3::Y).abs() * block.size.y
            + (block.rotation * Vec3::Z).abs() * block.size.z)
            / 2.0
    }

    #[test]
    fn furniture_stays_outside_runoff_and_goal_depth() {
        for length in [9.0, 14.0, 22.0] {
            let field = FieldDimensions {
                length,
                width: length * 2.0 / 3.0,
                ..FieldDimensions::SPL_2025
            };
            let arena = layout(&field);
            let x = field.length / 2.0 + field.border_strip_width.max(field.goal_depth);
            let z = field.width / 2.0 + field.border_strip_width;
            for block in &arena.blocks {
                let half = half_extents(block);
                if block.position.y + half.y <= 0.01 {
                    continue;
                }
                assert!(
                    block.position.x.abs() - half.x > x || block.position.z.abs() - half.z > z,
                    "furniture intrudes into runoff: {:?}",
                    block.position,
                );
            }
            assert!(arena.panels.iter().all(|panel| panel.width > 0.0));
            for panel in &arena.panels {
                let normal = Quat::from_rotation_y(panel.yaw) * Vec3::Z;
                assert!(normal.dot(-panel.position) > 0.0);
            }
            for brand in 0..BRANDS.len() {
                assert!(arena.panels.iter().any(|panel| panel.brand == brand));
            }
        }
    }

    #[test]
    fn landscape_leaves_sideline_cameras_clear() {
        for field in [
            FieldDimensions::SPL_2025,
            FieldDimensions {
                length: 22.0,
                width: 14.0,
                ..FieldDimensions::SPL_2025
            },
        ] {
            let mut arena = Layout::default();
            let end = field.length / 2.0 + field.border_strip_width.max(field.goal_depth) + 0.55;
            let side = field.width / 2.0 + field.border_strip_width + 0.55;
            landscape::add_landscape(&mut arena, end, side);
            for block in &arena.blocks {
                let half = half_extents(block);
                assert!(
                    block.position.y + half.y < 2.3
                        || block.position.x.abs() - half.x > field.length / 2.0 + 1.0
                        || block.position.z + half.z < field.width / 2.0 + 2.0
                        || block.position.z - half.z > field.width / 2.0 + 4.0,
                    "landscape occupies sideline camera corridor: {:?}",
                    block.position,
                );
            }
        }
    }

    #[test]
    fn brands_rotate_at_staggered_twelve_second_boundaries() {
        for initial in 0..BRAND_COUNT {
            let first_change = BRAND_CADENCE_SECONDS - (initial * 5 % BRAND_COUNT) as f64;
            assert_eq!(brand_index(initial, 0.0), initial);
            assert_eq!(brand_index(initial, first_change - 1e-6), initial);
            assert_eq!(
                brand_index(initial, first_change),
                (initial + 1) % BRAND_COUNT
            );
            assert_eq!(
                brand_index(initial, first_change + BRAND_CADENCE_SECONDS),
                (initial + 2) % BRAND_COUNT
            );
            assert_eq!(
                brand_index(initial, BRAND_CADENCE_SECONDS * BRAND_COUNT as f64),
                initial
            );
            // Seeking backward and returning live require no mutable animation state.
            for time in [40.5, 0.0, 7.25, 40.5] {
                assert_eq!(
                    brand_index(initial, time),
                    brand_index(initial, time + BRAND_CADENCE_SECONDS * BRAND_COUNT as f64)
                );
            }
        }
    }

    #[test]
    fn invalid_brand_times_preserve_the_initial_brand() {
        for initial in [0, 1, BRAND_COUNT - 1, BRAND_COUNT, usize::MAX] {
            for time in [-1.0, f64::NEG_INFINITY, f64::INFINITY, f64::NAN] {
                assert_eq!(brand_index(initial, time), initial % BRAND_COUNT);
            }
            assert!(brand_index(initial, f64::MAX) < BRAND_COUNT);
        }
    }

    #[test]
    fn shared_batches_have_finite_non_degenerate_triangles() {
        for batch in arena_meshes(&FieldDimensions::SPL_2025) {
            assert_eq!(batch.vertices.len(), batch.texcoords.len());
            assert!(!batch.faces.is_empty());
            for [a, b, c] in &batch.faces {
                let [a, b, c] = [a, b, c].map(|index| Vec3::from_array(batch.vertices[*index]));
                let normal = (b - a).cross(c - a);
                assert!(normal.is_finite() && normal.length_squared() > 0.0);
            }
            let vertex_count = batch.faces.len() * 3;
            let mesh = batch.into_bevy();
            assert_eq!(mesh.count_vertices(), vertex_count);
            assert!(mesh.attribute(Mesh::ATTRIBUTE_NORMAL).is_some());
        }
    }

    #[test]
    fn headless_arena_compiles_without_contacts_or_dofs() {
        let model = arena_spec(FieldDimensions::SPL_2025)
            .unwrap()
            .compile()
            .unwrap();
        assert!(model.geom_contype().iter().all(|value| *value == 0));
        assert!(model.geom_conaffinity().iter().all(|value| *value == 0));
        assert!(model.geom_group().iter().all(|value| *value == 5));
        assert_eq!(model.nv(), 0);
        assert_eq!(model.ngeom() as usize, COLORS.len() + BRANDS.len());
        assert_eq!(model.nlight(), 1);
        for initial in 0..BRAND_COUNT {
            let geom = model
                .name_to_id(MjtObj::mjOBJ_GEOM, &format!("arena_brand_{initial}"))
                .unwrap();
            let material = model
                .name_to_id(MjtObj::mjOBJ_MATERIAL, &format!("brand_{initial}"))
                .unwrap();
            assert_eq!(model.geom_matid()[geom] as usize, material);
        }
    }
}
