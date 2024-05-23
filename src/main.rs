use safe_drive::{
    context::Context, error::DynError, logger::Logger, pr_info
};
use std::time::Duration;

fn main() -> Result<(), DynError> {
    // Create a context.
    let ctx = Context::new()?;

    // Create a node.
    let node = ctx.create_node("amcl", None, Default::default())?;
    
    //create a subscriber
    let scan_subscriber = node.create_subscriber::<sensor_msgs::msg::LaserScan>("scan", None)?;
    let tf_subscriber = node.create_subscriber::<geometry_msgs::msg::TransformStamped>("tf",None)?;
    let init_subscriber = node.create_subscriber::<geometry_msgs::msg::PoseWithCovarianceStamped>("initialpose",None)?;
    let map_subscriber = node.create_subscriber::<nav_msgs::msg::OccupancyGrid>("map", None)?;

    // Create a publisher.
    let particle_publisher = node.create_publisher::<geometry_msgs::msg::PoseArray>("particle", None)?;
    let pose_publisher = node.create_publisher::<geometry_msgs::msg::PoseWithCovarianceStamped>("amcl_pose",None)?;
    let tf_publisher = node.create_publisher::<geometry_msgs::msg::TransformStamped>("tf",None)?;

    // Create a logger.
    let logger = Logger::new("amcl");

    let mut particles = geometry_msgs::msg::PoseArray::new().unwrap();
    particles.poses = geometry_msgs::msg::PoseSeq::<0>::new(1000).unwrap();
    loop {
        pr_info!(logger,"ParticleData: {:?}",particles.poses);
        
        // Send a message.
        particle_publisher.send(&particles)?;

        std::thread::sleep(Duration::from_secs(1));
    }
}
