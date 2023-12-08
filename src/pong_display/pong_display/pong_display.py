import rclpy
from rclpy.node import Node
import rosidl_runtime_py
from pong_msgs.msg import GameStatus
import turtle

pong_game_status = {}


class PongDisplayNode(Node):
    """
    Pong Display ROS Node
    """

    def __init__(self):
        super().__init__("pong_display_node")
        self.game_status_sub_ = self.create_subscription(
            GameStatus, "/game_status", self.game_status_callback_, 1
        )

    def game_status_callback_(self, msg):
        """
        Handle pong game status update from ROS interface
        """
        # mark global
        global pong_game_status
        # convert and save message
        pong_game_status = rosidl_runtime_py.convert.message_to_ordereddict(msg)


def main(args=None) -> None:
    # init ros
    rclpy.init(args=args)
    # create Pong Display ROS node
    pong_display_node = PongDisplayNode()

    # Create screen
    sc = turtle.Screen()
    sc.title("Pong game")
    sc.bgcolor("black")
    sc.setup(width=1500, height=1000)

    # Left paddle
    left_pad = turtle.Turtle()
    left_pad.speed(0)
    left_pad.shape("square")
    left_pad.color("blue")
    left_pad.shapesize(stretch_wid=6, stretch_len=1)
    left_pad.penup()
    left_pad.goto(-700, 0)

    # Right paddle
    right_pad = turtle.Turtle()
    right_pad.speed(0)
    right_pad.shape("square")
    right_pad.color("red")
    right_pad.shapesize(stretch_wid=6, stretch_len=1)
    right_pad.penup()
    right_pad.goto(700, 0)
    print(right_pad.shapesize())

    # Ball of circle shape
    ball = turtle.Turtle()
    ball.shape("circle")
    ball.color("white")
    ball.penup()
    ball.speed(0)
    ball.goto(0, 0)

    # Displays the score
    score_display = turtle.Turtle()
    score_display.color("white")
    score_display.penup()
    score_display.hideturtle()
    score_display.speed(0)
    score_display.goto(0, 450)
    score_display.write(
        "0 : 0",
        align="center",
        font=("Courier", 24, "normal"),
    )

    # scores
    player_1_score = 0
    player_2_score = 0

    while rclpy.ok():
        # update from ROS
        rclpy.spin_once(pong_display_node)

        # update UI
        sc.update()
        # move ball
        ball.goto(
            pong_game_status["ball"]["x_position"],
            pong_game_status["ball"]["y_position"],
        )
        # move left paddle
        left_pad.goto(-700, pong_game_status["player_1"]["y_position"])
        # move right paddle
        right_pad.goto(700, pong_game_status["player_2"]["y_position"])

        # update score display
        if (
            player_1_score != pong_game_status["player_1"]["score"]
            or player_2_score != pong_game_status["player_2"]["score"]
        ):
            player_1_score = pong_game_status["player_1"]["score"]
            player_2_score = pong_game_status["player_2"]["score"]
            score_display.clear()
            score_display.write(
                "{} : {}".format(
                    pong_game_status["player_1"]["score"],
                    pong_game_status["player_2"]["score"],
                ),
                align="center",
                font=("Courier", 24, "normal"),
            )

    # destroy node
    pong_display_node.destroy_node()
    # shutdown ros
    rclpy.shutdown()


if __name__ == "__main__":
    main()
