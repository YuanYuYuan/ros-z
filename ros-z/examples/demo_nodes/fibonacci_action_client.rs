#[cfg(all(feature = "external_msgs", has_action_tutorials_interfaces))]
use ros_z::{Builder, Result, context::ZContext};
#[cfg(all(feature = "external_msgs", has_action_tutorials_interfaces))]
use ros_z_msgs::action_tutorials_interfaces::{FibonacciGoal, action::Fibonacci};

/// Fibonacci action client node that sends goals to compute Fibonacci sequences
///
/// # Arguments
/// * `ctx` - The ROS-Z context
/// * `order` - The order of the Fibonacci sequence to compute
#[cfg(all(feature = "external_msgs", has_action_tutorials_interfaces))]
pub async fn run_fibonacci_action_client(ctx: ZContext, order: i32) -> Result<Vec<i32>> {
    // Create a node named "fibonacci_action_client"
    let node = ctx.create_node("fibonacci_action_client").build()?;

    // Create an action client
    let client = node
        .create_action_client::<Fibonacci>("fibonacci")
        .build()?;

    // Wait a bit for the server to be discovered
    tokio::time::sleep(tokio::time::Duration::from_secs(1)).await;

    println!(
        "Fibonacci action client started, sending goal with order {}",
        order
    );

    // Send the goal
    let mut goal_handle = client.send_goal(FibonacciGoal { order }).await?;
    println!("Goal sent and accepted!");

    // Set up feedback monitoring
    if let Some(mut feedback_stream) = goal_handle.feedback() {
        tokio::spawn(async move {
            while let Some(fb) = feedback_stream.recv().await {
                println!("Feedback: {:?}", fb.partial_sequence);
            }
        });
    }

    // Wait for the result
    println!("Waiting for result...");
    let result = goal_handle.result().await?;
    println!("Final result: {:?}", result.sequence);

    Ok(result.sequence)
}

// Only compile main when building as a binary (not when included as a module)
#[cfg(all(
    not(any(test, doctest)),
    feature = "external_msgs",
    has_action_tutorials_interfaces
))]
fn main() -> Result<()> {
    use clap::Parser;
    use ros_z::context::ZContextBuilder;

    let args = Args::parse();

    // Initialize logging
    zenoh::init_log_from_env_or("error");

    // Create the ROS-Z context with optional configuration
    let mut builder = ZContextBuilder::default();
    if let Some(e) = args.endpoint {
        builder = builder.with_connect_endpoints([e]);
    } else {
        // Connect to local zenohd for testing
        builder = builder.with_connect_endpoints(["tcp/127.0.0.1:7447"]);
    }
    let ctx = builder.build()?;

    // Run the client
    let result = tokio::runtime::Runtime::new()
        .unwrap()
        .block_on(run_fibonacci_action_client(ctx, args.order))?;

    println!("Action completed with sequence: {:?}", result);

    Ok(())
}

// Stub main when action_tutorials_interfaces is not available
#[cfg(all(
    not(any(test, doctest)),
    not(all(feature = "external_msgs", has_action_tutorials_interfaces))
))]
fn main() {
    eprintln!("Error: This example requires the action_tutorials_interfaces ROS 2 package.");
    eprintln!("Please install it or ensure your ROS 2 environment is properly set up.");
    std::process::exit(1);
}

#[cfg(all(
    not(any(test, doctest)),
    feature = "external_msgs",
    has_action_tutorials_interfaces
))]
#[derive(Debug, clap::Parser)]
#[command(
    name = "demo_nodes_fibonacci_action_client",
    about = "ROS 2 demo fibonacci action client node - sends goals to compute Fibonacci sequences"
)]
struct Args {
    /// Order of the Fibonacci sequence to compute
    #[arg(short, long, default_value = "10")]
    order: i32,

    /// Zenoh router endpoint to connect to (e.g., tcp/localhost:7447)
    #[arg(short, long)]
    endpoint: Option<String>,
}
