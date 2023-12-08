import rclpy
from rclpy.node import Node
import rosidl_runtime_py
from pong_msgs.msg import PlayerAction
import requests
from enum import Enum
from pong_msgs.msg import GameStatus

# Enum to keep track player actions
class PlayerActions(Enum):
    STAY = 0
    UP = 1
    DOWN = 2
# global variables
current_player_action : PlayerActions = PlayerActions.STAY
current_player_position = 0


class PongMobilePlayerNode(Node):
    """
    Pong Mobile Player Node
    """

    def __init__(self) -> None:
        super().__init__("pong_mobile_player_node")
        self.player_action_pub_ = self.create_publisher(
            PlayerAction, "/player_action", 1
        )
        self.timer_ = self.create_timer(0.025, self.publish_player_action_callback_)
        self.game_status_subs_ = self.create_subscription(
            GameStatus, "/game_status", self.game_status_callback_, 1
        )

    def publish_player_action_callback_(self) -> None:
        # create ROS message
        player_action_msg = PlayerAction()
        player_action_msg.player_name = "player2"
        player_action_msg.action = current_player_action.value
        # publish
        self.player_action_pub_.publish(player_action_msg)
    
    def game_status_callback_(self, msg):
        # mark global
        global current_player_position
        # convert and extract and save player position
        current_player_position = rosidl_runtime_py.convert.message_to_ordereddict(msg)["player_2"]["y_position"]


def read_acceleration_data(phyphox_url) -> dict:
    """
    Poll the acceleration data through the HTTP interface of phyphox.
    """
    acceleration = {
        "x": 0.0,
        "y": 0.0,
        "z": 0.0,
    }

    try:
        # make a request to the API http://...IP.../get?accX=&accY=&accZ=&acc=
        response = requests.get(
            phyphox_url,
            headers={"Content-Type": "application/json"},
            params={"accX": "", "accY": "", "accZ": "", "acc": ""},
            verify=False,
            timeout=0.5,
        )
    except requests.exceptions.RequestException:
        return acceleration

    if response.ok:
        json_res = response.json()
        acceleration["x"] = json_res["buffer"]["accX"]["buffer"][0]
        acceleration["y"] = json_res["buffer"]["accY"]["buffer"][0]
        acceleration["z"] = json_res["buffer"]["accZ"]["buffer"][0]

    return acceleration


def translate_acceleration_to_player_action(acceleration) -> PlayerActions:
    """
    Simplification: Track acceleration data on y axis. Decide if the phone pointing up, down or middle.
    """
    action = PlayerActions.STAY

    tilt_ratio = acceleration["y"] / 7.0
    position_to_be = tilt_ratio * 500 # 500 is the half of screen height
    position_diff = position_to_be - current_player_position

    if position_diff < - 10:
        action =  PlayerActions.DOWN
    if 10 < position_diff:
        action =  PlayerActions.UP
    
    return action


def main() -> None:
    # mark global
    global current_player_action
    
    # init ros
    rclpy.init()
    # create Pong Display ROS node
    pong_mobile_player_node = PongMobilePlayerNode()

    # use ros param for url (ugly solution)
    pong_mobile_player_node.declare_parameter("phyphox_url", "192.168.1.187")
    phyphox_url_param = pong_mobile_player_node.get_parameter("phyphox_url").value

    acceleration = {}
    try:
        while True:
            # read acceleration data
            acceleration = read_acceleration_data(phyphox_url_param)
            # select player action
            current_player_action = translate_acceleration_to_player_action(
                acceleration
            )
            # spin ros node
            rclpy.spin_once(pong_mobile_player_node)
    except KeyboardInterrupt:
        pass

    # destroy node
    pong_mobile_player_node.destroy_node()
    # shutdown ros
    rclpy.shutdown()


if __name__ == "__main__":
    main()
