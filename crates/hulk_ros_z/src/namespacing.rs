pub fn normalize_robot_namespace(robot: &str) -> String {
    if robot.starts_with('/') {
        robot.to_owned()
    } else {
        format!("/{robot}")
    }
}
