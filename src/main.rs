mod graph;
use graph::*;
use macroquad::prelude::*;
use std::env;
use std::path::Path;

fn window_conf() -> Conf {
    Conf {
        window_title: "A* Pathfinding".to_owned(),
        sample_count: 4, // 4x MSAA
        ..Default::default()
    }
}

fn draw_arrow_head(start: (f32, f32), end: (f32, f32), size: f32, color: Color) {
    let dx = end.0 - start.0;
    let dy = end.1 - start.1;
    let length = (dx * dx + dy * dy).sqrt();

    // Avoid drawing for tiny segments to prevent clutter
    if length < 15.0 {
        return;
    }

    let ux = dx / length;
    let uy = dy / length;

    let mid_x = start.0 + dx * 0.5;
    let mid_y = start.1 + dy * 0.5;

    let angle: f32 = 0.5;

    let x1 = mid_x - size * (ux * angle.cos() - uy * angle.sin());
    let y1 = mid_y - size * (ux * angle.sin() + uy * angle.cos());

    let x2 = mid_x - size * (ux * angle.cos() + uy * angle.sin());
    let y2 = mid_y - size * (-ux * angle.sin() + uy * angle.cos());

    draw_line(mid_x, mid_y, x1, y1, 1.5, color);
    draw_line(mid_x, mid_y, x2, y2, 1.5, color);
}

#[macroquad::main(window_conf)]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args: Vec<String> = env::args().collect();

    if args.len() < 2 {
        panic!("No map file provided.")
    }

    let map_path = &args[1];

    let bin_path = format!("{}.bin", map_path);

    let config = bincode::config::standard();

    let graph = if Path::new(&bin_path).exists() {
        println!("Loading cached binary map: {}...", bin_path);

        // Read the pre-processed bytes
        let bytes = std::fs::read(&bin_path)?;
        let (decoded, _): (Graph, usize) = bincode::serde::decode_from_slice(&bytes, config)
            .map_err(|e| format!("Bincode load error: {}", e))?;
        decoded
    } else {
        println!("No cache found. Parsing PBF (this will take a while)...");

        if !Path::new(map_path).exists() {
            panic!("The map file was not found at: '{}'.", map_path);
        }

        let decoded = Graph::from_pbf(map_path)?;
        println!(
            "Graph loaded with {} nodes and {} edges",
            decoded.nodes.len(),
            decoded.edges.len()
        );

        println!("Saving to cache for next time...");
        let encoded = bincode::serde::encode_to_vec(&decoded, config)
            .map_err(|e| format!("Bincode save error: {}", e))?;
        std::fs::write(&bin_path, encoded)?;

        decoded
    };

    println!(
        "Graph ready: {} nodes, {} edges",
        graph.nodes.len(),
        graph.edges.len()
    );

    // Map geographical bounds
    let mut min_lon = f32::MAX;
    let mut max_lon = f32::MIN;
    let mut min_lat = f32::MAX;
    let mut max_lat = f32::MIN;

    for node in &graph.nodes {
        min_lon = min_lon.min(node.lon);
        max_lon = max_lon.max(node.lon);
        min_lat = min_lat.min(node.lat);
        max_lat = max_lat.max(node.lat);
    }

    // Camera state
    let mut center_lon = (min_lon + max_lon) / 2.0;
    let mut center_lat = (min_lat + max_lat) / 2.0;
    let mut zoom = 1.0_f32;
    let mut last_mouse_pos = mouse_position();

    // Zoom OUT limit
    let min_zoom = 0.8_f32;

    // Zoom in limit
    // Earth's latitude is roughly 111,320 meters per degree.
    let meters_per_degree = 111320.0_f32;
    let map_width_degrees = max_lon - min_lon;
    let map_width_meters = map_width_degrees * meters_per_degree;

    let max_zoom = map_width_meters / 50.0;

    let mut start_node: Option<usize> = None;
    let mut end_node: Option<usize> = None;
    let mut path: Vec<usize> = Vec::new();
    let mut mouse_down_pos = (0.0, 0.0);

    // -- RENDERING LOOP --
    loop {
        clear_background(BLACK);

        // Handle Inputs
        let (mx, my) = mouse_position();

        let lon_range = max_lon - min_lon;
        let lat_range = max_lat - min_lat;
        let base_scale = (screen_width() / lon_range).min(screen_height() / lat_range);

        let mut current_scale = base_scale * zoom;

        let scale_text = format!("Scale: {}", &current_scale.to_string());
        draw_text(
            &scale_text,
            screen_width() - 150.0,
            screen_height() - 20.0,
            20.0,
            WHITE,
        );

        let lod_threshold = if current_scale < 500.0 {
            0 // Only Motorways/Trunk roads
        } else if current_scale < 1000.0 {
            1 // Add primary
        } else if current_scale < 3000.0 {
            2 // Add Secondary
        } else if current_scale < 8000.0 {
            3 // Add Tertiary
        } else if current_scale < 15000.0 {
            4
        } else {
            5 // Show everything (Alleys/Residential) when zoomed in
        };

        if is_mouse_button_down(MouseButton::Left) {
            let dx = mx - last_mouse_pos.0;
            let dy = my - last_mouse_pos.1;

            // Convert pixel delta to geo-coordinate delta
            center_lon -= dx / current_scale;
            center_lat += dy / current_scale; // Positive because Y is flipped

            // Boundary limits
            center_lon = center_lon.clamp(min_lon, max_lon);
            center_lat = center_lat.clamp(min_lat, max_lat);
        }

        last_mouse_pos = (mx, my);

        // Zooming (Mouse Wheel) into mouse position
        let wheel = mouse_wheel().1;
        if wheel != 0.0 {
            let mouse_lon = center_lon + (mx - screen_width() / 2.0) / current_scale;
            let mouse_lat = center_lat - (my - screen_height() / 2.0) / current_scale;

            let zoom_factor = 1.2_f32;
            let mut new_zoom = if wheel > 0.0 {
                zoom * zoom_factor
            } else {
                zoom / zoom_factor
            };

            // Clamp the zoom between our min and max limits
            new_zoom = new_zoom.clamp(min_zoom, max_zoom);

            // Only apply shift if the zoom actually changed (not clamped)
            if new_zoom != zoom {
                let new_scale = base_scale * new_zoom;

                center_lon = mouse_lon - (mx - screen_width() / 2.0) / new_scale;
                center_lat = mouse_lat + (my - screen_height() / 2.0) / new_scale;

                zoom = new_zoom;
                current_scale = new_scale;
            }
        }

        // Track where mouse started pressing
        if is_mouse_button_pressed(MouseButton::Left) || is_mouse_button_pressed(MouseButton::Right)
        {
            mouse_down_pos = mouse_position();
        }

        // Handle release (Select Points)
        if is_mouse_button_released(MouseButton::Left)
            || is_mouse_button_released(MouseButton::Right)
        {
            let curr_pos = mouse_position();
            let dist = ((curr_pos.0 - mouse_down_pos.0).powi(2)
                + (curr_pos.1 - mouse_down_pos.1).powi(2))
            .sqrt();

            if dist < 5.0 {
                // It was a click, not a drag
                let (mx, my) = curr_pos;
                // Convert screen pixel to geo coordinate
                let click_lon = center_lon + (mx - screen_width() / 2.0) / current_scale;
                let click_lat = center_lat - (my - screen_height() / 2.0) / current_scale;

                if let Some(closest) = graph.find_closest_node(click_lon, click_lat) {
                    if is_mouse_button_released(MouseButton::Right) {
                        start_node = Some(closest);
                    } else {
                        end_node = Some(closest);
                    }

                    // Recalculate path if both exist
                    if let (Some(s), Some(e)) = (start_node, end_node) {
                        match a_star(&graph, s, e) {
                            Some(found_path) => path = found_path,
                            None => {
                                println!("No path could be found between node {} and {}!", s, e);
                                path.clear();
                            }
                        }
                    }
                }
            }
        }

        // Convert geographical coordinates into pixels
        let to_screen = |lon: f32, lat: f32| -> (f32, f32) {
            let x = (lon - center_lon) * current_scale + screen_width() / 2.0;
            let y = screen_height() / 2.0 - (lat - center_lat) * current_scale;
            (x, y)
        };

        // Draw the graph edges
        // Calculate the screen's geographic bounding box
        let view_width_geo = screen_width() / current_scale;
        let view_height_geo = screen_height() / current_scale;

        let left = center_lon - view_width_geo / 2.0;
        let right = center_lon + view_width_geo / 2.0;
        let top = center_lat + view_height_geo / 2.0;
        let bottom = center_lat - view_height_geo / 2.0;

        // Extract edge drawing into a closure to avoid code duplication
        let draw_edge = |edge: &Edge, start_pos: (f32, f32), end_pos: (f32, f32)| {
            let start = to_screen(start_pos.0, start_pos.1);
            let end = to_screen(end_pos.0, end_pos.1);

            // Determine color and thickness based on road type (lod_priority)
            let (color, thickness) = match edge.lod_priority {
                0 => (Color::new(0.5, 0.1, 0.1, 1.0), 3.0),
                1 => (Color::new(0.6, 0.3, 0.1, 1.0), 2.5),
                2 => (Color::new(0.5, 0.5, 0.1, 1.0), 2.0),
                3 => (Color::new(0.7, 0.7, 0.7, 0.8), 1.5),
                _ => (Color::new(0.7, 0.7, 0.7, 0.7), 1.0), // Normal/Local: Dim Gray
            };

            let final_color = if edge.is_oneway && edge.lod_priority >= 3 {
                Color::new(0.4, 0.6, 1.0, 0.7) // Keep the bright blue for one-ways
            } else {
                color
            };

            draw_line(start.0, start.1, end.0, end.1, thickness, final_color);

            if edge.is_oneway {
                draw_arrow_head(start, end, thickness * 2.0 + 3.0, final_color);
            }
        };

        // Draw feature lines
        for feature in &graph.feature_lines {
            let start_pos = graph.get_node_position(feature.from);
            let end_pos = graph.get_node_position(feature.to);

            // Frustum culling for features
            if (start_pos.0 < left && end_pos.0 < left)
                || (start_pos.0 > right && end_pos.0 > right)
                || (start_pos.1 < bottom && end_pos.1 < bottom)
                || (start_pos.1 > top && end_pos.1 > top)
            {
                continue;
            }

            let start = to_screen(start_pos.0, start_pos.1);
            let end = to_screen(end_pos.0, end_pos.1);

            let (color, thickness) = match feature.feature_type {
                FeatureType::Coastline => (Color::new(0.0, 0.6, 0.8, 1.0), 2.0), // Teal
                FeatureType::CountryBorder => (Color::new(1.0, 0.2, 0.2, 0.8), 2.5), // Thick Red
                FeatureType::ProvinceBorder => (Color::new(1.0, 0.5, 0.0, 0.6), 1.5), // Orange
            };

            draw_line(start.0, start.1, end.0, end.1, thickness, color);
        }

        // -- LOD + RTREE --
        if lod_threshold <= 2 {
            for edge in graph.edges.iter() {
                if edge.lod_priority > lod_threshold {
                    continue;
                }

                let start_pos = graph.get_node_position(edge.from);
                let end_pos = graph.get_node_position(edge.to);

                // Quick boundary check to ensure we only draw what's visible
                if (start_pos.0 < left && end_pos.0 < left)
                    || (start_pos.0 > right && end_pos.0 > right)
                    || (start_pos.1 < bottom && end_pos.1 < bottom)
                    || (start_pos.1 > top && end_pos.1 > top)
                {
                    continue;
                }

                draw_edge(edge, start_pos, end_pos);
            }
        } else {
            let screen_rect = rstar::AABB::from_corners([left, bottom], [right, top]);

            for spatial_edge in graph.rtree.locate_in_envelope_intersecting(&screen_rect) {
                let edge = &graph.edges[spatial_edge.edge_index];

                if edge.lod_priority > lod_threshold {
                    continue;
                }

                let start_pos = graph.get_node_position(edge.from);
                let end_pos = graph.get_node_position(edge.to);

                draw_edge(edge, start_pos, end_pos);
            }
        }

        // Draw Search Path
        for i in 0..path.len().saturating_sub(1) {
            let p1 = graph.get_node_position(path[i]);
            let p2 = graph.get_node_position(path[i + 1]);
            let s = to_screen(p1.0, p1.1);
            let e = to_screen(p2.0, p2.1);
            draw_line(s.0, s.1, e.0, e.1, 5.0, BLUE);
        }

        // Draw Start/End Markers
        if let Some(s) = start_node {
            let p = graph.get_node_position(s);
            let scr = to_screen(p.0, p.1);
            draw_circle(scr.0, scr.1, 8.0, GREEN);
        }
        if let Some(e) = end_node {
            let p = graph.get_node_position(e);
            let scr = to_screen(p.0, p.1);
            draw_circle(scr.0, scr.1, 8.0, RED);
        }

        next_frame().await
    }
}
