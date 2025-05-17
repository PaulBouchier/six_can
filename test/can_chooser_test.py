#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor # Import SingleThreadedExecutor
from six_can_interfaces.srv import CanChooserRqst
import time
import math # For math.degrees

class CanChooserTestClient(Node):
    def __init__(self):
        super().__init__('can_chooser_test_client')
        self.client = self.create_client(CanChooserRqst, '/can_chooser_rqst')
        
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service /can_chooser_rqst not available, waiting again...')
        
        self.timer = self.create_timer(10.0, self.run_test_cycle_callback)
        self.get_logger().info("CanChooserTestClient started, calling service every 10 seconds.")

    async def call_service_and_log(self, choose_flag: bool):
        req = CanChooserRqst.Request()
        req.choose_can = choose_flag
        
        self.get_logger().info(f"call_service_and_log: Calling service with choose_can = {choose_flag}")
        future = self.client.call_async(req)
        self.get_logger().info(f"call_service_and_log: Service call initiated for choose_can = {choose_flag}. Future: {future}")
        
        try:
            self.get_logger().info(f"call_service_and_log: Awaiting future for choose_can = {choose_flag}...")
            response = await future # If successful, `response` gets the result.
                                    # If the service call fails (e.g. server exception), this raises an exception.
            self.get_logger().info(f"call_service_and_log: Future completed for choose_can = {choose_flag}. Response: {response}")
            
            if response.can_chosen:
                self.get_logger().info(
                    f"Response (choose_can={choose_flag}): Can chosen. "
                    f"X: {response.can_x:.2f}, Y: {response.can_y:.2f}, "
                    f"Range: {response.range:.2f}, Bearing: {math.degrees(response.bearing_rad):.1f} deg"
                )
            else:
                self.get_logger().info(f"Response (choose_can={choose_flag}): No can chosen.")

        except Exception as e:
            self.get_logger().error(f"call_service_and_log: Service call (choose_can={choose_flag}) failed or future error: {e!r}")

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
    
    # Use SingleThreadedExecutor for async callbacks (changed from MultiThreadedExecutor for testing)
    executor = SingleThreadedExecutor()
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
