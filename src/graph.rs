use bincode;
use osmpbf::{Element, ElementReader};
use rstar::{AABB, RTree, RTreeObject};
use rustc_hash::FxHashSet;
use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use std::path::Path;

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

#[derive(Serialize, Deserialize)]
pub struct Graph {
    pub nodes: Vec<Node>,
    pub edges: Vec<Edge>,
    pub adj_list: Vec<Vec<usize>>,
    pub reverse_adj_list: Vec<Vec<usize>>,
    pub osm_id_to_index: HashMap<i64, usize>,
    pub rtree: RTree<SpatialEdge>,
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
        if !is_oneway {
            self.reverse_adj_list[to].push(edge_idx);
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

    pub fn from_pbf<P: AsRef<Path>>(path: P) -> Result<Self, Box<dyn std::error::Error>> {
        let mut graph = Graph::new();
        let mut required_nodes = FxHashSet::default();

        // Pass 1: Identify required nodes

        println!("Pass 1: Identifying road nodes...");

        let reader = ElementReader::from_path(&path)?;
        reader.for_each(|element| {
            if let Element::Way(way) = element {
                if way.tags().any(|(k, _)| k == "highway") {
                    for node_id in way.refs() {
                        required_nodes.insert(node_id);
                    }
                }
            }
        })?;

        // Pre-allocate: Reduce reallocations based on Pass 1 results
        graph.nodes.reserve(required_nodes.len());
        graph.osm_id_to_index.reserve(required_nodes.len());

        // Pass 2: Extract coordinates

        println!(
            "Pass 2: Extracting coordinates for {} nodes...",
            required_nodes.len()
        );

        let reader = ElementReader::from_path(&path)?;
        reader.for_each(|element| match element {
            Element::Node(node) if required_nodes.contains(&node.id()) => {
                graph.add_node(node.id(), node.lon() as f32, node.lat() as f32);
            }
            Element::DenseNode(node) if required_nodes.contains(&node.id()) => {
                graph.add_node(node.id(), node.lon() as f32, node.lat() as f32);
            }
            _ => {}
        })?;

        // Pass 3: Build edges

        println!("Pass 3: Building edges...");

        let reader = ElementReader::from_path(&path)?;
        reader.for_each(|element| {
            if let Element::Way(way) = element {
                let mut is_highway = false;
                let mut is_oneway = false;
                let mut speed_limit = 50.0;
                let mut highway_type = "other"; // Track the type for LOD

                for (k, v) in way.tags() {
                    match k {
                        "highway" => {
                            is_highway = true;
                            highway_type = v;
                        }
                        "oneway" => is_oneway = v == "yes",
                        "maxspeed" => {
                            if let Ok(s) = v.parse() {
                                speed_limit = s
                            }
                        }
                        _ => {}
                    }
                }

                if is_highway {
                    let lod_priority = match highway_type {
                        "motorway" | "trunk" => 0,
                        "primary" => 1,
                        "secondary" => 2,
                        "tertiary" => 3,
                        "residential" | "unclassified" => 4,
                        _ => 5,
                    };

                    let refs: Vec<i64> = way.refs().collect();

                    for i in 0..refs.len().saturating_sub(1) {
                        if let (Some(&from_idx), Some(&to_idx)) = (
                            graph.osm_id_to_index.get(&refs[i]),
                            graph.osm_id_to_index.get(&refs[i + 1]),
                        ) {
                            graph.add_edge(
                                from_idx,
                                to_idx,
                                speed_limit,
                                way.id(),
                                is_oneway,
                                lod_priority,
                            );
                        }
                    }
                }
            }
        })?;

        println!("Pass 4: Building Spatial Index (R-Tree)...");
        let mut spatial_edges = Vec::with_capacity(graph.edges.len());

        for (idx, edge) in graph.edges.iter().enumerate() {
            let start = &graph.nodes[edge.from];
            let end = &graph.nodes[edge.to];

            let min_lon = start.lon.min(end.lon);
            let max_lon = start.lon.max(end.lon);
            let min_lat = start.lat.min(end.lat);
            let max_lat = start.lat.max(end.lat);

            let envelope = AABB::from_corners([min_lon, min_lat], [max_lon, max_lat]);

            spatial_edges.push(SpatialEdge {
                edge_index: idx,
                envelope,
            });
        }

        graph.rtree = RTree::bulk_load(spatial_edges);

        Ok(graph)
    }

    pub fn save_to_file(&self, path: &str) -> Result<(), Box<dyn std::error::Error>> {
        let config = bincode::config::standard();

        let encoded = bincode::serde::encode_to_vec(self, config)
            .map_err(|e| format!("Serialization failed: {}", e))?;

        std::fs::write(path, encoded)?;
        Ok(())
    }

    pub fn load_from_bin(path: &str) -> Result<Self, Box<dyn std::error::Error>> {
        let bytes = std::fs::read(path)?;
        let config = bincode::config::standard();

        // decode_from_slice returns (T, usize) - the data and bytes read
        let (decoded, _): (Self, usize) = bincode::serde::decode_from_slice(&bytes, config)
            .map_err(|e| format!("Deserialization failed: {}", e))?;

        Ok(decoded)
    }
}
