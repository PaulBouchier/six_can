#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from six_can_interfaces.srv import CanChooserRpst
import time
import math # For math.degrees

class CanChooserTestClient(Node):
    def __init__(self):
        super().__init__('can_chooser_test_client')
        self.client = self.create_client(CanChooserRpst, '/can_chooser_rqst')
        
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service /can_chooser_rqst not available, waiting again...')
        
        self.timer = self.create_timer(10.0, self.run_test_cycle_callback)
        self.get_logger().info("CanChooserTestClient started, calling service every 10 seconds.")

    async def call_service_and_log(self, choose_flag: bool):
        req = CanChooserRpst.Request()
        req.choose_can = choose_flag
        
        self.get_logger().info(f"Calling service with choose_can = {choose_flag}")
        future = self.client.call_async(req)
        
        try:
            # Wait for the service call to complete with a timeout
            # This will spin the node enough for the future to complete
            await future # This is a more modern way to await futures with async callbacks
            
            if future.done(): # Redundant if await future succeeded, but good for clarity/error handling
                response = future.result()
                if response.can_chosen:
                    self.get_logger().info(
                        f"Response (choose_can={choose_flag}): Can chosen. "
                        f"X: {response.can_x:.2f}, Y: {response.can_y:.2f}, "
                        f"Range: {response.range:.2f}, Bearing: {math.degrees(response.bearing_rad):.1f} deg"
                    )
                else:
                    self.get_logger().info(f"Response (choose_can={choose_flag}): No can chosen.")
            # else: # await future would raise an exception if it didn't complete successfully (e.g. timeout if configured)
            #    self.get_logger().warn(f"Service call (choose_can={choose_flag}) did not complete (future not done).")

        except Exception as e:
            self.get_logger().error(f"Service call (choose_can={choose_flag}) failed: {e!r}")

    async def run_test_cycle_callback(self): # Renamed to be more descriptive for a timer callback
        self.get_logger().info("--- Starting Test Cycle ---")
        
        # Call 1: choose_can = True
        await self.call_service_and_log(True)
        
        # Brief pause using rclpy's async sleep if needed, or just proceed.
        # For this test, direct sequential calls are fine.
        # await self.create_timer(0.5, lambda: None).timer_ready_event.wait() # Example async sleep

        # Call 2: choose_can = False
        await self.call_service_and_log(False)
        
        self.get_logger().info("--- Test Cycle Complete ---")


def main(args=None):
    rclpy.init(args=args)
    test_client_node = CanChooserTestClient()
    
    # Use MultiThreadedExecutor for async callbacks
    executor = MultiThreadedExecutor()
    executor.add_node(test_client_node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        test_client_node.get_logger().info("CanChooserTestClient shutting down...")
    finally:
        executor.shutdown()
        if test_client_node.get_node_name() != "": # Check if node was successfully created
            test_client_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
