#!/usr/bin/env python3
"""
Converts keyboard input to sensor_msgs/Joy and publishes to /joy_keyboard.
simple_mux subscribes to both /joy (physical pad) and /joy_keyboard, and honours
one of them based on the /ego/use_keyboard trigger from the pitwall panel.
Uses pynput to detect simultaneous key presses; speed and steering are fully independent.

Controls:
  Up / Down    Forward / Reverse (held only, resets to 0 on release)
  Left / Right Steer left / right (held only, resets to 0 on release)
  Space        Stop (speed and steering = 0)
  H            Human Drive mode
  A            Auto Drive mode
  Ctrl+C       Quit (in the node's terminal)
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy

from pynput import keyboard as kb



class KeyboardJoyNode(Node):

    def __init__(self):
        super().__init__('keyboard_joy')

        self.declare_parameter('joy_topic', '/joy_keyboard')
        self.pub = self.create_publisher(Joy, self.get_parameter('joy_topic').value, 10)
        self.create_timer(0.02, self._loop)  # 50 Hz

        self.speed = 0.0
        self.steer = 0.0
        self.mode  = 'human'

        self._held = set()   # set of currently held keys (managed by listener thread)

        self.listener = kb.Listener(
            on_press=self._on_press,
            on_release=self._on_release,
        )
        self.listener.start()

        print("=== Keyboard Joy Node ===")
        print("  Up / Down   : Forward / Reverse (resets on release)")
        print("  Left / Right: Steer left / right (resets on release)")
        print("  Space       : Stop (speed and steering = 0)")
        print("  H           : Human Drive mode")
        print("  A           : Auto Drive mode")
        print("  Ctrl+C      : Quit (in this terminal)")

    # ------------------------------------------------------------------ #

    def _on_press(self, key):
        self._held.add(key)

        if key == kb.Key.space:
            self.speed = 0.0
            self.steer = 0.0
        elif hasattr(key, 'char') and key.char is not None:
            c = key.char.lower()
            if c == 'h':
                self.mode = 'human'
            elif c == 'a':
                self.mode = 'auto'
            # pynput hooks keys globally (any window), so a stray 'q' typed
            # anywhere would kill the node - disabled. Stop the node with
            # Ctrl+C in its terminal instead.
            # elif c == 'q':
            #     raise KeyboardInterrupt

    def _on_release(self, key):
        self._held.discard(key)

    # ------------------------------------------------------------------ #

    def _loop(self):
        # speed and steering are checked independently from the held set
        self.speed = 0.8 if kb.Key.up   in self._held else \
                    -0.8 if kb.Key.down  in self._held else 0.0
        self.steer = 1.0 if kb.Key.left  in self._held else \
                    -1.0 if kb.Key.right in self._held else 0.0

        self._publish()

    def _publish(self):
        msg = Joy()
        msg.header.stamp = self.get_clock().now().to_msg()

        # per simple_mux_node._joy_cb: axes[1] = speed, axes[3] = steering
        msg.axes = [0.0, self.speed, 0.0, self.steer]

        # buttons[4] = humandrive,  buttons[5] = autodrive
        msg.buttons = [
            0, 0, 0, 0,
            1 if self.mode == 'human' else 0,
            1 if self.mode == 'auto'  else 0,
        ]

        self.pub.publish(msg)

    def destroy_node(self):
        self.listener.stop()
        super().destroy_node()


# ------------------------------------------------------------------ #

def main(args=None):
    rclpy.init(args=args)
    node = KeyboardJoyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
