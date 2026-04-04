use bincode;
use osmpbf::{Element, ElementReader};
use rayon::prelude::*;
use rstar::{AABB, RTree, RTreeObject};
use rustc_hash::FxHashSet;
use serde::{Deserialize, Serialize};
use std::cmp::Ordering;
use std::collections::BinaryHeap;
use std::collections::HashMap;
use std::error::Error as StdError;
use std::fs::File; // Added File
use std::io::{BufReader, BufWriter}; // Added Buf-utilities
use std::path::{Path, PathBuf}; // Added PathBuf

#[derive(Serialize, Deserialize)]
pub struct Node {
    pub osm_id: i64,
    pub lon: f32,
    pub lat: f32,
}

#[derive(Serialize, Deserialize)]
pub struct Edge {
    pub from: usize,
    pub to: usize,
    pub weight: f32,
    pub speed_limit: f32,
    pub osm_way_id: i64,
    pub is_oneway: bool,
    pub lod_priority: u8, // 0 = Motorway (Always show), 5 = Alleyway (Hidden when zoomed out)
}

#[derive(Serialize, Deserialize, Clone, Debug, PartialEq)]
pub struct SpatialEdge {
    pub edge_index: usize,
    pub envelope: AABB<[f32; 2]>,
}

impl RTreeObject for SpatialEdge {
    type Envelope = AABB<[f32; 2]>;
    fn envelope(&self) -> Self::Envelope {
        self.envelope
    }
}

#[derive(Serialize, Deserialize, Clone, Copy, PartialEq)]
pub enum FeatureType {
    Coastline,
    CountryBorder,
    ProvinceBorder,
}

#[derive(Serialize, Deserialize)]
pub struct FeatureLine {
    pub from: usize,
    pub to: usize,
    pub feature_type: FeatureType,
}

#[derive(Copy, Clone, PartialEq)]
struct State {
    cost: f32, // This is f(n) = g(n) + h(n) used for the Priority Queue
    g: f32,    // This is g(n), the actual distance from the start
    node: usize,
}

#[derive(Serialize, Deserialize)]
pub struct Graph {
    pub nodes: Vec<Node>,
    pub edges: Vec<Edge>,
    pub adj_list: Vec<Vec<usize>>,
    pub reverse_adj_list: Vec<Vec<usize>>,
    pub osm_id_to_index: HashMap<i64, usize>,
    pub rtree: RTree<SpatialEdge>,
    pub feature_lines: Vec<FeatureLine>,
}

impl Graph {
    pub fn new() -> Self {
        Graph {
            nodes: Vec::new(),
            edges: Vec::new(),
            adj_list: Vec::new(),
            reverse_adj_list: Vec::new(),
            osm_id_to_index: HashMap::new(),
            rtree: RTree::new(),
            feature_lines: Vec::new(),
        }
    }

    pub fn add_node(&mut self, osm_id: i64, lon: f32, lat: f32) -> usize {
        let id = self.nodes.len();
        self.nodes.push(Node { osm_id, lon, lat });
        self.adj_list.push(Vec::new());
        self.reverse_adj_list.push(Vec::new());
        self.osm_id_to_index.insert(osm_id, id);
        id
    }

    pub fn add_edge(
        &mut self,
        from: usize,
        to: usize,
        speed_limit: f32,
        osm_way_id: i64,
        is_oneway: bool,
        lod_priority: u8,
    ) {
        let weight = self.calculate_weight(from, to, speed_limit);

        // 1. Forward Edge
        let edge_idx = self.edges.len();
        self.edges.push(Edge {
            from,
            to,
            weight,
            speed_limit,
            osm_way_id,
            is_oneway,
            lod_priority,
        });
        self.adj_list[from].push(edge_idx);

        // 2. Backward Edge (Crucial for A* Connectivity!)
        if !is_oneway {
            let rev_edge_idx = self.edges.len();
            self.edges.push(Edge {
                from: to, // Flipped
                to: from, // Flipped
                weight,
                speed_limit,
                osm_way_id,
                is_oneway,
                lod_priority,
            });
            self.adj_list[to].push(rev_edge_idx); // Put directly in adj_list so A* can see it
        }
    }

    pub fn get_edges_from(&self, from: usize) -> Vec<&Edge> {
        self.adj_list[from]
            .iter()
            .map(|&edge_idx| &self.edges[edge_idx])
            .collect()
    }

    pub fn get_edges_to(&self, to: usize) -> Vec<&Edge> {
        self.reverse_adj_list[to]
            .iter()
            .map(|&edge_idx| &self.edges[edge_idx])
            .collect()
    }

    pub fn get_all_edges(&self, node: usize) -> Vec<&Edge> {
        let mut all_edges = Vec::new();

        all_edges.extend(self.get_edges_from(node));
        all_edges.extend(self.get_edges_to(node));

        all_edges
    }

    pub fn add_feature_line(&mut self, from: usize, to: usize, feature_type: FeatureType) {
        self.feature_lines.push(FeatureLine {
            from,
            to,
            feature_type,
        });
    }

    pub fn get_node_position(&self, osm_id: usize) -> (f32, f32) {
        (self.nodes[osm_id].lon, self.nodes[osm_id].lat)
    }

    pub fn get_nodes_distance(&self, id0: usize, id1: usize) -> f32 {
        let pos0 = self.get_node_position(id0);
        let pos1 = self.get_node_position(id1);

        let delta_x = pos0.0 - pos1.0;
        let delta_y = pos0.1 - pos1.1;
        (delta_x.powi(2) + delta_y.powi(2)).sqrt()
    }

    pub fn calculate_weight(&self, from: usize, to: usize, speed_limit: f32) -> f32 {
        let distance = self.get_nodes_distance(from, to);
        distance / speed_limit
    }

    pub fn find_closest_node(&self, lon: f32, lat: f32) -> Option<usize> {
        let mut closest_node = None;
        let mut min_dist = f32::MAX;

        for (i, node) in self.nodes.iter().enumerate() {
            if self.adj_list[i].is_empty() {
                continue;
            }

            let dist = ((node.lon - lon).powi(2) + (node.lat - lat).powi(2)).sqrt();
            if dist < min_dist {
                min_dist = dist;
                closest_node = Some(i);
            }
        }
        closest_node
    }
    pub fn from_pbf<P: AsRef<Path>>(path: P) -> Result<Self, Box<dyn std::error::Error>> {
        let mut graph = Graph::new();

        // --- Pass 1: Identify required nodes (Parallel) ---
        println!("Pass 1: Identifying road nodes...");
        let required_nodes: FxHashSet<i64> = ElementReader::from_path(&path)?.par_map_reduce(
            |element| {
                let mut local_set = FxHashSet::default();
                if let Element::Way(way) = element {
                    let mut is_needed = false;
                    let mut is_maritime = false;
                    for (k, v) in way.tags() {
                        if k == "maritime" && v == "yes" {
                            is_maritime = true;
                        }
                        if k == "highway"
                            || (k == "natural" && v == "coastline")
                            || (k == "boundary" && v == "administrative")
                        {
                            is_needed = true;
                        }
                    }
                    if is_needed && !is_maritime {
                        for node_id in way.refs() {
                            local_set.insert(node_id);
                        }
                    }
                }
                local_set
            },
            FxHashSet::default,
            |mut a, b| {
                a.extend(b);
                a
            },
        )?;

        // --- Pass 2: Extract coordinates (Parallel) ---
        println!(
            "Pass 2: Extracting coordinates for {} nodes...",
            required_nodes.len()
        );

        let reader = ElementReader::from_path(&path)?;
        // 1. Collect all nodes into a flat Vec in parallel
        let mut all_nodes: Vec<(i64, f32, f32)> = reader.par_map_reduce(
            |element| {
                let mut local_nodes = Vec::new();
                match element {
                    Element::Node(node) if required_nodes.contains(&node.id()) => {
                        local_nodes.push((node.id(), node.lon() as f32, node.lat() as f32));
                    }
                    Element::DenseNode(node) if required_nodes.contains(&node.id()) => {
                        local_nodes.push((node.id(), node.lon() as f32, node.lat() as f32));
                    }
                    _ => {}
                }
                local_nodes
            },
            Vec::new,
            |mut a, mut b| {
                a.append(&mut b);
                a
            },
        )?;

        println!(
            "Finalizing node indexing for {} extracted nodes...",
            all_nodes.len()
        );

        // 2. Pre-allocate to prevent the "len is 0" error
        graph.nodes.reserve(all_nodes.len());
        graph.osm_id_to_index.reserve(all_nodes.len());

        // 3. Move data into the graph
        // We do this sequentially because 'graph.nodes' and 'osm_id_to_index'
        // must stay in sync for the indices to be valid in Pass 3.
        for (osm_id, lon, lat) in all_nodes {
            let index = graph.nodes.len();
            graph.nodes.push(Node { osm_id, lon, lat });
            graph.osm_id_to_index.insert(osm_id, index);
        }

        if graph.nodes.is_empty() {
            panic!("Error: No nodes were loaded into the graph! Check if required_nodes is empty.");
        }

        // --- Pass 3: Build edges and features (Parallel) ---
        println!("Pass 3: Building edges and features...");

        struct EdgeProto {
            from: usize,
            to: usize,
            speed: f32,
            id: i64,
            oneway: bool,
            lod: u8,
        }
        struct FeatureProto {
            from: usize,
            to: usize,
            f_type: FeatureType,
        }

        let reader = ElementReader::from_path(&path)?;
        let (all_edges, all_features): (Vec<EdgeProto>, Vec<FeatureProto>) = reader
            .par_map_reduce(
                |element| {
                    let mut local_edges = Vec::new();
                    let mut local_features = Vec::new();

                    if let Element::Way(way) = element {
                        let mut is_highway = false;
                        let mut is_oneway = false;
                        let mut speed_limit: Option<f32> = None;
                        let mut highway_type = "other";
                        let mut is_maritime = false;
                        let mut feature_type = None;

                        // 1. Parse Tags inside the thread closure
                        for (k, v) in way.tags() {
                            match k {
                                "highway" => {
                                    is_highway = true;
                                    highway_type = v;
                                }
                                "oneway" => is_oneway = v == "yes",
                                "maxspeed" => {
                                    if let Ok(s) = v.parse::<f32>() {
                                        speed_limit = Some(s);
                                    }
                                }
                                "maritime" if v == "yes" => is_maritime = true,
                                "natural" if v == "coastline" => {
                                    feature_type = Some(FeatureType::Coastline);
                                }
                                "boundary" if v == "administrative" => {
                                    let level = way
                                        .tags()
                                        .find(|(tk, _)| *tk == "admin_level")
                                        .map(|(_, tv)| tv)
                                        .unwrap_or("");
                                    feature_type = match level {
                                        "2" => Some(FeatureType::CountryBorder),
                                        "4" | "6" => Some(FeatureType::ProvinceBorder),
                                        _ => None,
                                    };
                                }
                                _ => {}
                            }
                        }

                        let refs: Vec<i64> = way.refs().collect();

                        if is_highway {
                            let pedestrian_types = [
                                "footway",
                                "pedestrian",
                                "path",
                                "steps",
                                "cycleway",
                                "bridleway",
                                "sidewalk",
                                "corridor",
                                "platform",
                                "stairs",
                                "track",
                            ];

                            if !pedestrian_types.contains(&highway_type) {
                                // 2. Assign LOD and Speeds
                                let (lod_priority, default_speed) = match highway_type {
                                    "motorway" => (0, 110.0),
                                    "trunk" => (0, 90.0),
                                    "primary" => (1, 70.0),
                                    "secondary" => (2, 60.0),
                                    "tertiary" => (3, 50.0),
                                    "residential" | "unclassified" => (4, 30.0),
                                    "living_street" => (5, 15.0),
                                    "service" => (5, 20.0),
                                    _ => (5, 30.0),
                                };

                                let final_speed = speed_limit.unwrap_or(default_speed);

                                for i in 0..refs.len().saturating_sub(1) {
                                    if let (Some(&f_idx), Some(&t_idx)) = (
                                        graph.osm_id_to_index.get(&refs[i]),
                                        graph.osm_id_to_index.get(&refs[i + 1]),
                                    ) {
                                        local_edges.push(EdgeProto {
                                            from: f_idx,
                                            to: t_idx,
                                            speed: final_speed,
                                            id: way.id(),
                                            oneway: is_oneway,
                                            lod: lod_priority,
                                        });
                                    }
                                }
                            }
                        } else if let Some(f_type) = feature_type {
                            if !is_maritime {
                                for i in 0..refs.len().saturating_sub(1) {
                                    if let (Some(&f_idx), Some(&t_idx)) = (
                                        graph.osm_id_to_index.get(&refs[i]),
                                        graph.osm_id_to_index.get(&refs[i + 1]),
                                    ) {
                                        local_features.push(FeatureProto {
                                            from: f_idx,
                                            to: t_idx,
                                            f_type,
                                        });
                                    }
                                }
                            }
                        }
                    }
                    (local_edges, local_features)
                },
                || (Vec::new(), Vec::new()),
                |mut a, mut b| {
                    a.0.append(&mut b.0);
                    a.1.append(&mut b.1);
                    a
                },
            )?;

        // 3. Sequential application to the graph
        println!(
            "Initializing adjacency lists for {} nodes...",
            graph.nodes.len()
        );
        graph.adj_list = vec![Vec::new(); graph.nodes.len()];
        graph.reverse_adj_list = vec![Vec::new(); graph.nodes.len()];

        println!("Adding {} edges to the graph...", all_edges.len());
        for e in all_edges {
            graph.add_edge(e.from, e.to, e.speed, e.id, e.oneway, e.lod);
        }

        for f in all_features {
            graph.add_feature_line(f.from, f.to, f.f_type);
        }

        // Parallelize Pass 4 prep
        let spatial_edges: Vec<SpatialEdge> = graph
            .edges
            .par_iter()
            .enumerate()
            .map(|(idx, edge)| {
                let start = &graph.nodes[edge.from];
                let end = &graph.nodes[edge.to];
                let envelope = AABB::from_corners(
                    [start.lon.min(end.lon), start.lat.min(end.lat)],
                    [start.lon.max(end.lon), start.lat.max(end.lat)],
                );
                SpatialEdge {
                    edge_index: idx,
                    envelope,
                }
            })
            .collect();

        // --- Pass 4: Spatial Index ---
        println!("Pass 4: Building Spatial Index...");
        let spatial_edges: Vec<SpatialEdge> = graph
            .edges
            .iter()
            .enumerate()
            .map(|(idx, edge)| {
                let start = &graph.nodes[edge.from];
                let end = &graph.nodes[edge.to];
                let envelope = AABB::from_corners(
                    [start.lon.min(end.lon), start.lat.min(end.lat)],
                    [start.lon.max(end.lon), start.lat.max(end.lat)],
                );
                SpatialEdge {
                    edge_index: idx,
                    envelope,
                }
            })
            .collect();
        graph.rtree = RTree::bulk_load(spatial_edges);

        Ok(graph)
    }

    pub fn save_to_file<P: AsRef<Path>>(
        &self,
        path: P,
    ) -> Result<(), Box<dyn StdError + Send + Sync>> {
        let path = path.as_ref();
        if !path.exists() {
            std::fs::create_dir_all(path)
                .map_err(|e| Box::new(e) as Box<dyn StdError + Send + Sync>)?;
        }

        println!("Saving graph components in parallel...");

        // We wrap in a block to catch results if needed,
        // or just use unwrap if you are certain the disk is writable.
        rayon::scope(|s| {
            s.spawn(|_| {
                self.save_component(path.join("nodes.bin"), &self.nodes)
                    .expect("Failed to save nodes")
            });
            s.spawn(|_| {
                self.save_component(path.join("edges.bin"), &self.edges)
                    .expect("Failed to save edges")
            });
            s.spawn(|_| {
                self.save_component(path.join("adj.bin"), &self.adj_list)
                    .expect("Failed to save adj")
            });
            s.spawn(|_| {
                self.save_component(path.join("rev_adj.bin"), &self.reverse_adj_list)
                    .expect("Failed to save rev_adj")
            });
            s.spawn(|_| {
                self.save_component(path.join("osm_map.bin"), &self.osm_id_to_index)
                    .expect("Failed to save osm_map")
            });
            s.spawn(|_| {
                self.save_component(path.join("features.bin"), &self.feature_lines)
                    .expect("Failed to save features")
            });
            s.spawn(|_| {
                self.save_component(path.join("rtree.bin"), &self.rtree)
                    .expect("Failed to save rtree")
            });
        });

        Ok(())
    }

    fn save_component<T: Serialize, P: AsRef<Path>>(
        &self,
        path: P,
        data: &T,
    ) -> Result<(), Box<dyn StdError + Send + Sync>> {
        let file = File::create(path)?;
        let mut writer = BufWriter::new(file); // Needs to be mut
        let config = bincode::config::standard();

        // Use &mut writer here
        bincode::serde::encode_into_std_write(data, &mut writer, config)
            .map_err(|e| format!("Serialization failed: {}", e))?;
        Ok(())
    }

    pub fn load_from_file<P: AsRef<Path>>(
        path: P,
    ) -> Result<Self, Box<dyn StdError + Send + Sync>> {
        let path = path.as_ref();
        println!("Loading graph components in parallel...");

        let (nodes_edges, others) = rayon::join(
            || {
                rayon::join(
                    || Self::load_component::<Vec<Node>>(path.join("nodes.bin")),
                    || Self::load_component::<Vec<Edge>>(path.join("edges.bin")),
                )
            },
            || {
                rayon::join(
                    || {
                        rayon::join(
                            || Self::load_component::<Vec<Vec<usize>>>(path.join("adj.bin")),
                            || Self::load_component::<Vec<Vec<usize>>>(path.join("rev_adj.bin")),
                        )
                    },
                    || {
                        rayon::join(
                            || {
                                rayon::join(
                                    || {
                                        Self::load_component::<HashMap<i64, usize>>(
                                            path.join("osm_map.bin"),
                                        )
                                    },
                                    || {
                                        Self::load_component::<Vec<FeatureLine>>(
                                            path.join("features.bin"),
                                        )
                                    },
                                )
                            },
                            || Self::load_component::<RTree<SpatialEdge>>(path.join("rtree.bin")),
                        )
                    },
                )
            },
        );

        Ok(Graph {
            nodes: nodes_edges.0?,
            edges: nodes_edges.1?,
            adj_list: others.0.0?,
            reverse_adj_list: others.0.1?,
            osm_id_to_index: others.1.0.0?,
            feature_lines: others.1.0.1?,
            rtree: others.1.1?,
        })
    }

    // Ensure T implements serde::de::DeserializeOwned for simplicity with bincode
    fn load_component<T: serde::de::DeserializeOwned>(
        path: PathBuf,
    ) -> Result<T, Box<dyn StdError + Send + Sync>> {
        let file = File::open(path)?;
        let mut reader = BufReader::new(file); // Needs to be mut
        let config = bincode::config::standard();

        // Use &mut reader here
        let decoded: T = bincode::serde::decode_from_std_read(&mut reader, config)
            .map_err(|e| format!("Deserialization failed: {}", e))?;
        Ok(decoded)
    }
}

impl Eq for State {}
impl Ord for State {
    fn cmp(&self, other: &Self) -> Ordering {
        other
            .cost
            .partial_cmp(&self.cost)
            .unwrap_or(Ordering::Equal)
    }
}
impl PartialOrd for State {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

pub fn a_star(graph: &Graph, start: usize, goal: usize) -> Option<Vec<usize>> {
    // 1. Find the maximum speed limit in the graph to keep the heuristic admissible.
    // If the graph has no edges yet, fall back to a standard highway speed like 120.0.
    let max_speed = graph
        .edges
        .iter()
        .map(|e| e.speed_limit)
        .fold(120.0, f32::max);

    let mut dist = vec![f32::MAX; graph.nodes.len()];
    let mut parent = vec![None; graph.nodes.len()];
    let mut pq = BinaryHeap::new();

    dist[start] = 0.0;
    pq.push(State {
        cost: 0.0,
        g: 0.0,
        node: start,
    });

    while let Some(State { cost, g, node }) = pq.pop() {
        if node == goal {
            let mut path = Vec::new();
            let mut curr = Some(goal);
            while let Some(n) = curr {
                path.push(n);
                curr = parent[n];
            }
            return Some(path);
        }

        if g > dist[node] {
            continue;
        }

        for &edge_idx in &graph.adj_list[node] {
            let edge = &graph.edges[edge_idx];
            let next = edge.to;
            let n2 = &graph.nodes[next];

            // Use the edge's stored weight (distance / speed limit)
            let weight = edge.weight;

            let new_dist = dist[node] + weight;
            if new_dist < dist[next] {
                dist[next] = new_dist;
                parent[next] = Some(node);

                // Heuristic: Straight line distance to goal
                let goal_node = &graph.nodes[goal];
                let h_dist =
                    ((n2.lon - goal_node.lon).powi(2) + (n2.lat - goal_node.lat).powi(2)).sqrt();

                // Estimated time = distance / max_speed
                let h = h_dist / max_speed;

                pq.push(State {
                    cost: new_dist + h,
                    g: new_dist,
                    node: next,
                });
            }
        }
    }
    None
}
