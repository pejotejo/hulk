use std::{
    convert::Into, env::current_dir, iter::once, net::Ipv4Addr, path::PathBuf, str::FromStr,
    sync::Arc, time::SystemTime,
};

use argument_parsers::NaoAddress;
use clap::Parser;
use color_eyre::{
    eyre::{bail, eyre, Context as _, ContextCompat},
    Report, Result,
};
use eframe::{
    egui::{
        CentralPanel, Context, CornerRadius, Id, Label, Layout, Sense, StrokeKind, TopBottomPanel,
        Ui, Widget, WidgetText,
    },
    emath::Align,
    epaint::Color32,
    run_native, App, CreationContext, Frame, NativeOptions, Storage,
};
use egui_dock::{DockArea, DockState, Node, NodeIndex, Split, SurfaceIndex, TabAddAlign, TabIndex};
use fern::{colors::ColoredLevelConfig, Dispatch, InitError};
use itertools::chain;
use serde_json::{from_str, to_string, Value};

use communication::client::Status;
use configuration::{
    keybind_plugin::{self, KeybindSystem},
    keys::KeybindAction,
    Configuration,
};
use hulk_widgets::CompletionEdit;
use log::{error, warn};
use nao::Nao;
use panel::Panel;
use panels::{
    BallCandidatePanel, BehaviorSimulatorPanel, CameraCalibrationExportPanel, EnumPlotPanel,
    ImageColorSelectPanel, ImagePanel, ImageSegmentsPanel, LookAtPanel, ManualCalibrationPanel,
    MapPanel, ParameterPanel, PlotPanel, RemotePanel, SemiAutomaticCameraCalibrationPanel,
    TextPanel, VisionTunerPanel,
};
use reachable_naos::ReachableNaos;
use repository::{inspect_version::check_for_update, Repository};
use visuals::Visuals;

use crate::panels::WalkPanel;

mod change_buffer;
mod configuration;
mod log_error;
mod nao;
mod panel;
mod panels;
mod players_buffer_handle;
mod reachable_naos;
mod selectable_panel_macro;
mod twix_painter;
mod value_buffer;
mod visuals;
mod zoom_and_pan;

#[derive(Debug, Parser)]
struct Arguments {
    /// Nao address to connect to (overrides the address saved in the configuration file)
    pub address: Option<String>,
    /// Alternative repository root
    #[arg(long)]
    repository_root: Option<PathBuf>,
    /// Delete the current panel setup
    #[arg(long)]
    pub clear: bool,
}

fn setup_logger() -> Result<(), InitError> {
    Dispatch::new()
        .format(|out, message, record| {
            let colors = ColoredLevelConfig::new();
            out.finish(format_args!(
                "[{}] {}",
                colors.color(record.level()),
                message
            ))
        })
        .level(log::LevelFilter::Info)
        .chain(std::io::stdout())
        .apply()?;
    Ok(())
}

fn main() -> Result<(), eframe::Error> {
    setup_logger().unwrap();

    let arguments = Arguments::parse();
    let repository = arguments
        .repository_root
        .clone()
        .map(Repository::new)
        .map(Ok)
        .unwrap_or_else(|| {
            let current_directory = current_dir().wrap_err("failed to get current directory")?;
            Repository::find_root(current_directory).wrap_err("failed to find repository root")
        });
    match &repository {
        Ok(repository) => {
            if let Err(error) = check_for_update(
                env!("CARGO_PKG_VERSION"),
                repository.root.join("tools/twix/Cargo.toml"),
                "twix",
            ) {
                error!("{error:#?}");
            }
        }
        Err(error) => {
            warn!("{error:#?}");
        }
    }

    let configuration = Configuration::load()
        .unwrap_or_else(|error| panic!("failed to load configuration: {error}"));

    run_native(
        "Twix",
        NativeOptions::default(),
        Box::new(|creation_context| {
            egui_extras::install_image_loaders(&creation_context.egui_ctx);
            Ok(Box::new(TwixApp::create(
                creation_context,
                arguments,
                configuration,
                repository.ok(),
            )))
        }),
    )
}

impl_selectable_panel!(
    BallCandidatePanel,
    BehaviorSimulatorPanel,
    CameraCalibrationExportPanel,
    EnumPlotPanel,
    ImageColorSelectPanel,
    ImagePanel,
    ImageSegmentsPanel,
    LookAtPanel,
    ManualCalibrationPanel,
    MapPanel,
    ParameterPanel,
    PlotPanel,
    RemotePanel,
    SemiAutomaticCameraCalibrationPanel,
    TextPanel,
    VisionTunerPanel,
    WalkPanel,
);

struct TwixApp {
    nao: Arc<Nao>,
    possible_addresses: Vec<Ipv4Addr>,
    address: String,
    reachable_naos: ReachableNaos,
    connection_intent: bool,
    panel_selection: String,
    last_focused_tab: (NodeIndex, TabIndex),
    dock_state: DockState<Tab>,
    visual: Visuals,
}

impl TwixApp {
    fn create(
        creation_context: &CreationContext,
        arguments: Arguments,
        configuration: Configuration,
        repository: Option<Repository>,
    ) -> Self {
        let nao_range = configuration.naos.lowest..=configuration.naos.highest;
        let possible_addresses: Vec<_> = chain!(
            once(Ipv4Addr::LOCALHOST),
            nao_range.clone().map(|id| Ipv4Addr::new(10, 0, 24, id)),
            nao_range.map(|id| Ipv4Addr::new(10, 1, 24, id)),
        )
        .collect();
        let address = arguments
            .address
            .and_then(|address| {
                NaoAddress::from_str(&address)
                    .map(|nao| nao.ip.to_string())
                    .ok()
            })
            .or_else(|| creation_context.storage?.get_string("address"))
            .unwrap_or(Ipv4Addr::LOCALHOST.to_string());

        let nao = Arc::new(Nao::new(
            match address.split_once(":") {
                None | Some((_, "")) => {
                    format!("ws://{address}:1337")
                }
                Some((ip, port)) => {
                    format!("ws://{ip}:{port}")
                }
            },
            repository,
        ));

        let connection_intent = creation_context
            .storage
            .and_then(|storage| storage.get_string("connection_intent"))
            .map(|stored| stored == "true")
            .unwrap_or(false);

        if connection_intent {
            nao.connect();
        }

        let dock_state: Option<DockState<Value>> = if arguments.clear {
            None
        } else {
            creation_context
                .storage
                .and_then(|storage| storage.get_string("dock_state"))
                .and_then(|string| from_str(&string).ok())
        };

        let dock_state = match dock_state {
            Some(dock_state) => dock_state.map_tabs(|value| Tab::new(nao.clone(), value)),
            None => DockState::new(vec![SelectablePanel::TextPanel(TextPanel::new(
                nao.clone(),
                None,
            ))
            .into()]),
        };

        let context = creation_context.egui_ctx.clone();

        keybind_plugin::register(&context);
        context.set_keybinds(Arc::new(configuration.keys));

        let reachable_naos = ReachableNaos::new(context.clone());
        nao.on_change(move || context.request_repaint());

        let visual = creation_context
            .storage
            .and_then(|storage| storage.get_string("style"))
            .and_then(|theme| Visuals::from_str(&theme).ok())
            .unwrap_or(Visuals::Dark);
        visual.set_visual(&creation_context.egui_ctx);

        let panel_selection = "".to_string();

        Self {
            nao,
            reachable_naos,
            connection_intent,
            panel_selection,
            dock_state,
            last_focused_tab: (0.into(), 0.into()),
            visual,
            possible_addresses,
            address,
        }
    }

    fn focus_left(&mut self, node_id: NodeIndex, surface_index: SurfaceIndex) -> Option<()> {
        let parent_id = node_id.parent()?;
        let parent = &self.dock_state[surface_index][parent_id];
        if node_id.is_left() || parent.is_vertical() {
            return self.focus_left(parent_id, surface_index);
        }
        let mut left_id = parent_id.left();

        loop {
            let node = &self.dock_state[surface_index][left_id];
            match node {
                Node::Empty => unreachable!("cannot hit an empty node while digging down"),
                Node::Leaf { .. } => break,
                Node::Vertical { .. } => {
                    left_id = left_id.left();
                }
                Node::Horizontal { .. } => {
                    left_id = left_id.right();
                }
            };
        }

        self.dock_state
            .set_focused_node_and_surface((surface_index, left_id));
        Some(())
    }

    fn focus_right(&mut self, node_id: NodeIndex, surface_index: SurfaceIndex) -> Option<()> {
        let parent_id = node_id.parent()?;
        let parent = &self.dock_state[surface_index][parent_id];
        if node_id.is_right() || parent.is_vertical() {
            return self.focus_right(parent_id, surface_index);
        }
        let mut child = parent_id.right();

        loop {
            let node = &self.dock_state[surface_index][child];
            match node {
                Node::Empty => unreachable!("cannot hit an empty node while digging down"),
                Node::Leaf { .. } => break,
                Node::Vertical { .. } => {
                    child = child.left();
                }
                Node::Horizontal { .. } => {
                    child = child.left();
                }
            };
        }

        self.dock_state
            .set_focused_node_and_surface((surface_index, child));
        Some(())
    }

    fn focus_above(&mut self, node_id: NodeIndex, surface_index: SurfaceIndex) -> Option<()> {
        let parent_id = node_id.parent()?;
        let parent = &self.dock_state[surface_index][parent_id];
        if node_id.is_left() || parent.is_horizontal() {
            return self.focus_above(parent_id, surface_index);
        }
        let mut left_id = parent_id.left();

        loop {
            let node = &self.dock_state[surface_index][left_id];
            match node {
                Node::Empty => unreachable!("cannot hit an empty node while digging down"),
                Node::Leaf { .. } => break,
                Node::Vertical { .. } => {
                    left_id = left_id.right();
                }
                Node::Horizontal { .. } => {
                    left_id = left_id.left();
                }
            };
        }

        self.dock_state
            .set_focused_node_and_surface((surface_index, left_id));
        Some(())
    }

    fn focus_below(&mut self, node_id: NodeIndex, surface_index: SurfaceIndex) -> Option<()> {
        let parent_id = node_id.parent()?;
        let parent = &self.dock_state[surface_index][parent_id];
        if node_id.is_right() || parent.is_horizontal() {
            return self.focus_below(parent_id, surface_index);
        }
        let mut child = parent_id.right();

        loop {
            let node = &self.dock_state[surface_index][child];
            match node {
                Node::Empty => unreachable!("cannot hit an empty node while digging down"),
                Node::Leaf { .. } => break,
                Node::Vertical { .. } => {
                    child = child.left();
                }
                Node::Horizontal { .. } => {
                    child = child.left();
                }
            };
        }

        self.dock_state
            .set_focused_node_and_surface((surface_index, child));
        Some(())
    }
}

impl App for TwixApp {
    fn update(&mut self, context: &Context, _frame: &mut Frame) {
        self.reachable_naos.update();

        TopBottomPanel::top("top_bar").show(context, |ui| {
            ui.horizontal(|ui| {
                ui.with_layout(Layout::left_to_right(Align::Center), |ui| {
                    let address_input = CompletionEdit::new(
                        ui.id().with("nao-selector"),
                        &self.possible_addresses,
                        &mut self.address,
                    )
                    .ui(ui, |ui, selected, ip| {
                        let show_green = self.reachable_naos.is_reachable(*ip);
                        let color = if show_green {
                            Color32::GREEN
                        } else {
                            Color32::WHITE
                        };
                        ui.selectable_label(selected, WidgetText::from(ip.to_string()).color(color))
                    });

                    if address_input.gained_focus() {
                        self.reachable_naos.query_reachability();
                    }
                    if context.keybind_pressed(KeybindAction::FocusAddress) {
                        address_input.request_focus();
                    }
                    if address_input.changed() || address_input.lost_focus() {
                        match &self.address.split_once(":") {
                            None | Some((_, "")) => {
                                let address = &self.address;
                                self.nao.set_address(format!("ws://{address}:1337"));
                            }
                            Some((ip, port)) => {
                                self.nao.set_address(format!("ws://{ip}:{port}"));
                            }
                        }
                        self.connection_intent = true;
                        self.nao.connect();
                    }
                    let (connect_text, color) = match self.nao.connection_status() {
                        Status::Disconnected => ("Disconnected", Color32::RED),
                        Status::Connecting => ("Connecting", Color32::YELLOW),
                        Status::Connected => ("Connected", Color32::GREEN),
                    };
                    let connect_text = WidgetText::from(connect_text).color(color);
                    if ui
                        .checkbox(&mut self.connection_intent, connect_text)
                        .changed()
                    {
                        if self.connection_intent {
                            self.nao.connect();
                        } else {
                            self.nao.disconnect();
                        }
                    }
                    if context.keybind_pressed(KeybindAction::Reconnect) {
                        self.nao.disconnect();
                        self.connection_intent = true;
                        self.nao.connect();
                    }

                    if self.active_tab_index() != Some(self.last_focused_tab) {
                        self.last_focused_tab =
                            self.active_tab_index().unwrap_or((0.into(), 0.into()));
                        if let Some(name) = self
                            .active_tab()
                            .and_then(|tab| tab.panel.as_ref().ok())
                            .map(|panel| format!("{panel}"))
                        {
                            self.panel_selection = name
                        }
                    }
                    let panels = SelectablePanel::registered();
                    let panel_input = ui.add(CompletionEdit::new(
                        ui.id().with("panel-selector"),
                        &panels,
                        &mut self.panel_selection,
                    ));

                    if context.keybind_pressed(KeybindAction::FocusPanel) {
                        panel_input.request_focus();
                    }
                    if panel_input.changed() {
                        match SelectablePanel::try_from_name(
                            &self.panel_selection,
                            self.nao.clone(),
                            None,
                        ) {
                            Ok(panel) => {
                                if let Some(active_tab) = self.active_tab() {
                                    active_tab.panel = Ok(panel);
                                }
                            }
                            Err(err) => error!("{err:?}"),
                        }
                    }
                });
                ui.with_layout(Layout::right_to_left(Align::Center), |ui| {
                    ui.menu_button("⚙", |ui| {
                        ui.menu_button("Theme", |ui| {
                            ui.vertical(|ui| {
                                for visual in Visuals::iter() {
                                    if ui.button(visual.to_string()).clicked() {
                                        self.visual = visual;
                                        self.visual.set_visual(context);
                                    }
                                }
                            })
                        });
                    })
                });
            })
        });
        CentralPanel::default().show(context, |ui| {
            if context.keybind_pressed(KeybindAction::OpenSplit) {
                let tab = SelectablePanel::TextPanel(TextPanel::new(self.nao.clone(), None));
                if let Some((surface_index, node_id)) = self.dock_state.focused_leaf() {
                    let node = &mut self.dock_state[surface_index][node_id];
                    if node.tabs_count() == 0 {
                        node.append_tab(tab.into());
                    } else {
                        let rect = node.rect().unwrap();
                        let direction = if rect.height() > rect.width() {
                            Split::Below
                        } else {
                            Split::Right
                        };
                        self.dock_state.split(
                            (surface_index, node_id),
                            direction,
                            0.5,
                            Node::leaf(tab.into()),
                        );
                    }
                }
            }
            if context.keybind_pressed(KeybindAction::OpenTab) {
                let tab = SelectablePanel::TextPanel(TextPanel::new(self.nao.clone(), None));
                self.dock_state.push_to_focused_leaf(tab.into());
            }

            if context.keybind_pressed(KeybindAction::FocusLeft) {
                if let Some((surface_index, node_id)) = self.dock_state.focused_leaf() {
                    self.focus_left(node_id, surface_index);
                }
            }
            if context.keybind_pressed(KeybindAction::FocusBelow) {
                if let Some((surface_index, node_id)) = self.dock_state.focused_leaf() {
                    self.focus_below(node_id, surface_index);
                }
            }
            if context.keybind_pressed(KeybindAction::FocusAbove) {
                if let Some((surface_index, node_id)) = self.dock_state.focused_leaf() {
                    self.focus_above(node_id, surface_index);
                }
            }
            if context.keybind_pressed(KeybindAction::FocusRight) {
                if let Some((surface_index, node_id)) = self.dock_state.focused_leaf() {
                    self.focus_right(node_id, surface_index);
                }
            }

            if context.keybind_pressed(KeybindAction::DuplicateTab) {
                if let Some((_, tab)) = self.dock_state.find_active_focused() {
                    let new_tab = tab.save();
                    self.dock_state.push_to_focused_leaf(Tab::from(
                        SelectablePanel::new(self.nao.clone(), Some(&new_tab)).unwrap(),
                    ));
                }
            }

            if context.keybind_pressed(KeybindAction::CloseTab) {
                if let Some((surface_index, node_id)) = self.dock_state.focused_leaf() {
                    let active_node = &mut self.dock_state[surface_index][node_id];
                    if let Node::Leaf { active, tabs, .. } = active_node {
                        if !tabs.is_empty() {
                            tabs.remove(active.0);

                            active.0 = active.0.saturating_sub(1);

                            if tabs.is_empty() && node_id != NodeIndex(0) {
                                self.dock_state[surface_index].remove_leaf(node_id);
                            }
                        }
                    }
                }
            }

            if context.keybind_pressed(KeybindAction::CloseAll) {
                self.dock_state = DockState::new(vec![SelectablePanel::TextPanel(TextPanel::new(
                    self.nao.clone(),
                    None,
                ))
                .into()]);
                self.last_focused_tab = (0.into(), 0.into());
                self.dock_state
                    .set_focused_node_and_surface((0.into(), 0.into()));
            }

            let mut style = egui_dock::Style::from_egui(ui.style().as_ref());
            style.buttons.add_tab_align = TabAddAlign::Left;
            let mut tab_viewer = TabViewer::default();
            DockArea::new(&mut self.dock_state)
                .style(style)
                .show_add_buttons(true)
                .show_inside(ui, &mut tab_viewer);

            for (surface_index, node_id) in tab_viewer.nodes_to_add_tabs_to {
                let tab = SelectablePanel::TextPanel(TextPanel::new(self.nao.clone(), None));
                let index = self.dock_state[surface_index][node_id].tabs_count();
                self.dock_state[surface_index][node_id].insert_tab(index.into(), tab.into());
                self.dock_state
                    .set_focused_node_and_surface((surface_index, node_id));
            }

            if let Some((surface_index, node_id)) = self.dock_state.focused_leaf() {
                let node = &self.dock_state[surface_index][node_id];
                let rect = node.rect().unwrap();
                ui.painter().rect_stroke(
                    rect,
                    CornerRadius::same(4),
                    ui.visuals().widgets.active.bg_stroke,
                    StrokeKind::Outside,
                );
            }
        });
    }

    fn save(&mut self, storage: &mut dyn Storage) {
        let dock_state = self.dock_state.map_tabs(|tab| tab.save());

        storage.set_string("dock_state", to_string(&dock_state).unwrap());
        storage.set_string("address", self.address.to_string());
        storage.set_string(
            "connection_intent",
            if self.connection_intent {
                "true"
            } else {
                "false"
            }
            .to_string(),
        );
        storage.set_string("style", self.visual.to_string());
    }
}

impl TwixApp {
    fn active_tab(&mut self) -> Option<&mut Tab> {
        let (_viewport, tab) = self.dock_state.find_active_focused()?;
        Some(tab)
    }

    fn active_tab_index(&self) -> Option<(NodeIndex, TabIndex)> {
        let (surface, node) = self.dock_state.focused_leaf()?;
        if let Node::Leaf { active, .. } = &self.dock_state[surface][node] {
            Some((node, *active))
        } else {
            None
        }
    }
}

struct Tab {
    id: Id,
    panel: Result<SelectablePanel, (Report, Value)>,
}

impl From<SelectablePanel> for Tab {
    fn from(panel: SelectablePanel) -> Self {
        Self {
            id: Id::new(SystemTime::now()),
            panel: Ok(panel),
        }
    }
}

impl Tab {
    fn new(nao: Arc<Nao>, value: &Value) -> Self {
        Self {
            id: Id::new(SystemTime::now()),
            panel: SelectablePanel::new(nao, Some(value)).map_err(|error| (error, value.clone())),
        }
    }

    fn save(&self) -> Value {
        match &self.panel {
            Ok(panel) => panel.save(),
            Err((_report, value)) => value.clone(),
        }
    }
}

#[derive(Default)]
struct TabViewer {
    nodes_to_add_tabs_to: Vec<(SurfaceIndex, NodeIndex)>,
}

impl egui_dock::TabViewer for TabViewer {
    type Tab = Tab;

    fn ui(&mut self, ui: &mut Ui, tab: &mut Self::Tab) {
        match &mut tab.panel {
            Ok(panel) => panel.ui(ui),

            Err((error, value)) => {
                ui.label(format!("Error loading panel: {error}"));
                ui.collapsing("JSON", |ui| {
                    let content = match serde_json::to_string_pretty(value) {
                        Ok(pretty_string) => pretty_string,
                        Err(error) => error.to_string(),
                    };
                    let label = ui.add(Label::new(&content).sense(Sense::click()));
                    if label.clicked() {
                        ui.ctx().copy_text(content);
                    }
                    label.on_hover_ui_at_pointer(|ui| {
                        ui.label("Click to copy");
                    });
                })
                .header_response
            }
        };
    }

    fn title(&mut self, tab: &mut Self::Tab) -> eframe::egui::WidgetText {
        match &mut tab.panel {
            Ok(panel) => format!("{panel}").into(),
            Err((error, _value)) => WidgetText::from(format!("{error}")).color(Color32::LIGHT_RED),
        }
    }

    fn id(&mut self, tab: &mut Self::Tab) -> Id {
        tab.id
    }

    fn on_add(&mut self, surface_index: SurfaceIndex, node: NodeIndex) {
        self.nodes_to_add_tabs_to.push((surface_index, node));
    }
}
