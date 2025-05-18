#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.task import Future
import math # For math.degrees
import time

from six_can_interfaces.srv import CanChooserRqst


class CanChooserTestClient(Node):
    def __init__(self):
        super().__init__('can_chooser_test_client')
        self.client = self.create_client(CanChooserRqst, '/can_chooser_rqst')
        
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service /can_chooser_rqst not available, waiting again...')
        self.req = CanChooserRqst.Request()
        self.future: Future = None
        self.choose_can = True
        
        self.timer = self.create_timer(5.0, self.run_test_cycle_callback)
        self.get_logger().info("CanChooserTestClient started, calling service every 5 seconds.")

    def call_service_and_log(self, choose_flag: bool):
        if self.future is not None and not self.future.done():
            self.future.cancel()  # Cancel the future. The callback will be called with Future.result == None.
            self.get_logger().info("Service Future canceled. The Node took too long to process the service call."
                                   "Is the Service Server still alive?")

        self.req.choose_can = choose_flag
        self.get_logger().info(f"call_service_and_log: Calling service with choose_can = {choose_flag}")
        self.future = self.client.call_async(self.req)
        self.future.add_done_callback(self.service_response_callback)
        
    def service_response_callback(self, future: Future):
        try:
            response = future.result()  # If successful, `response` gets the result.
                                        # If the service call fails (e.g. server exception), this raises an exception.
            if response is None:
                self.get_logger().error("service_response_callback: Service call failed or future error: response was None.")
            else:
                if response.can_chosen:
                    self.get_logger().info(
                        f"Response (choose_can={self.req.choose_can}): Can chosen. "
                        f"X: {response.can_x:.2f}, Y: {response.can_y:.2f}, "
                        f"Range: {response.range:.2f}, Bearing: {math.degrees(response.bearing_rad):.1f} deg"
                    )
                else:
                    self.get_logger().info(f"Response (choose_can={self.req.choose_can}): No can chosen.")

        except Exception as e:
            self.get_logger().error(f"service_response_callback: Service call failed or future error: {e!r}")


    def run_test_cycle_callback(self): # Renamed to be more descriptive for a timer callback
        self.get_logger().info("--- Starting Test Cycle ---")
        
        self.call_service_and_log(self.choose_can)
        self.choose_can = not self.choose_can  # Toggle the flag for the next call
        

def main(args=None):
    rclpy.init(args=args)
    test_client_node = CanChooserTestClient()
    
    try:
        rclpy.spin(test_client_node)
    except KeyboardInterrupt:
        test_client_node.get_logger().info("CanChooserTestClient shutting down...")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
