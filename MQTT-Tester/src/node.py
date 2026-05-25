import time
import rclpy
import random
import string
import math

from rclpy.node import Node
from ros2node.api import get_node_names
from array import array

import importlib

class PubSubNode(Node):
    def __init__(self, node_name_raw):
        self.node_name = node_name_raw + "_node"
        Node.__init__(self, self.node_name)
        # self.RobotState_pub = self.create_publisher(RobotStatus, "/to/RobotStatus", rclpy.qos.qos_profile_parameters)
        # self.RobotState_sub = self.create_subscription(RobotStatus, "/from/RobotStatus", self.checker_callback , rclpy.qos.qos_profile_parameters),
        # self.base_msg = RobotStatus()
        self.make_pubsub("../topics.txt")
        self.create_timer(0.5, self.publisher),

        self.last_send_msg = None
        self.need_pub = True
        self.sub_count = 0
        self.COMPARE_NUM = 5

        self.type_count = 0

        self.is_browser_active = False
        with open("../topics.txt", 'r', encoding='utf-8') as f:
            checker_text = f.read()
            self.checker_list = checker_text.split("\n")
            for i in range(len(self.checker_list)):
                self.checker_list[i] = "- " + self.checker_list[i]


    def make_pubsub(self, fname):
        self.pubs = []
        self.subs = []
        self.base_msgs = []
        msg_types = read_msg_types_from_file(fname)

        for msg_full_path in msg_types:
            pkg, _, msg_name = msg_full_path.strip().partition('/msg/')
            if not pkg or not msg_name:
                print(f"Invalid message path: {msg_full_path}")
                continue

            try:
                # メッセージ型をインポート
                module = importlib.import_module(f'{pkg}.msg')
                msg_class = getattr(module, msg_name)

                pub_topic_name = "/to/"   + msg_name
                sub_topic_name = "/from/" + msg_name
                # Publisher
                pub = self.create_publisher(msg_class, pub_topic_name, rclpy.qos.qos_profile_parameters)
                self.pubs.append(pub)

                # # Subscriber
                sub = self.create_subscription(msg_class, sub_topic_name, self.checker_callback, rclpy.qos.qos_profile_parameters)
                self.subs.append(sub)

                self.base_msgs.append(msg_class())
            except (ImportError, AttributeError) as e:
                # self.get_logger().error(f'Failed to import: {msg_full_path} ({e})')
                print(f'Failed to import: {msg_full_path} ({e})')

    def checker_callback(self, msg):
        self.is_browser_active = True
        # if self.last_send_msg == msg:
        if compare_ros2_msgs(self.last_send_msg, msg, rel_tol=1e-4, abs_tol=1e-4): # 小数点以下の微小誤差を考慮して比較
            self.sub_count += 1
            print("Received same message {} / {}".format(self.sub_count, self.COMPARE_NUM))
            if self.sub_count < self.COMPARE_NUM:
                self.need_pub = True
            else:
                print(type(self.base_msgs[self.type_count]), "Success!")
                self.next_msg(True)
        else:
            print("not same")
            print(str(msg))
            print(str(self.last_send_msg))
            print("Failed...")
            self.next_msg(False)
        
    def next_msg(self, TF):
        tf = ["✗ ", "✓ "][TF]
        self.checker_list[self.type_count] = self.checker_list[self.type_count].replace("- ", tf)
        with open("../check_result.txt", 'w', encoding='utf-8') as f:
            f.write("\n".join(self.checker_list))

        if self.type_count < len(self.base_msgs) - 1:
            self.type_count += 1
            self.need_pub = True
            self.sub_count = 0
        else:
            print("All messages sent and received.")
            exit()
    
    def publisher(self):
        if not self.is_browser_active:
            self.need_pub = True
        
        if self.need_pub:
            i = self.type_count
            base_msg = assign_random_to_ros2_msg(self.base_msgs[i])

            # if "MultiArray" in base_msg.__str__():
            #     self.next_msg(False)
            #     return

            self.pubs[i].publish(base_msg)
            # self.get_logger().info("Published: " + str(base_msg))
            self.last_send_msg = base_msg
            self.need_pub = False




def read_msg_types_from_file(file_path: str):
    with open(file_path, 'r', encoding='utf-8') as file:
        data = file.read()
    
    topics = data.split("\n")
    if topics[-1] == "":
        topics = topics[:-1]
    return topics


def random_primitive(field_type):
    """基本型のランダム値を生成"""
    if field_type in ['int', 'int32', 'int64', 'int16']:
        return random.randint(-10000, 10000)
    elif field_type in ['uint', 'uint32', 'uint64', 'uint16']:
        return random.randint(0, 20000)
    elif field_type in ['uint8']:
        return random.randint(0, 255)
    elif field_type in ['int8']:
        return random.randint(-128, 127)
    elif field_type in ['float', 'float32', 'float64', 'double']:
        return random.uniform(-1000.0, 1000.0)
    elif field_type in ['bool', 'boolean']:
        return random.choice([True, False])
    elif field_type in ['str', 'string']:
        return ''.join(random.choices(string.ascii_letters, k=8))
    else:
        print(field_type, "is not a recognized primitive type.")
        return None

def assign_random_to_ros2_msg(msg):
    """ROS2 メッセージの全フィールドにランダム値を代入"""
    for field_name, field_type in zip(msg.__slots__, msg._fields_and_field_types.values()):
        value = getattr(msg, field_name)
        # print(f"Assigning random value to field: {field_name} of type {field_type}")

        # 配列型かどうかを判定
        if field_type.endswith(']'):
            base_type = field_type[:field_type.index('[')]
            array_len = random.randint(1, 5)
            setattr(msg, field_name, [random_primitive(base_type) for _ in range(array_len)])
        elif "sequence<" in field_type:
            base_type = field_type[field_type.index('<') + 1:field_type.index('>')]
            array_len = random.randint(1, 5)
            val = random_primitive(base_type)
            is_primitive = val is not None
            if is_primitive:
                del getattr(msg, field_name)[:]
                for _ in range(array_len):
                    val = random_primitive(base_type)
                    getattr(msg, field_name).append(val)
            else:
                pkg_name, msg_name = base_type.split('/')
                module = importlib.import_module(f'{pkg_name}.msg')
                msg_class = getattr(module, msg_name)
                del getattr(msg, field_name)[:]
                for _ in range(array_len):
                    val = assign_random_to_ros2_msg(msg_class())
                    getattr(msg, field_name).append(val)

        elif 'msg' in str(type(value)):
            # ネストされたメッセージ（geometry_msgsなど）なら再帰呼び出し
            assign_random_to_ros2_msg(value)
        else:
            rand_value = random_primitive(field_type)
            if rand_value is not None:
                setattr(msg, field_name, rand_value)

    return msg
    

### 小数点以下の誤差を許容して比較する関数
def isclose_floats(a, b, rel_tol=1e-9, abs_tol=1e-9):
    return math.isclose(a, b, rel_tol=rel_tol, abs_tol=abs_tol)

### ROS2 メッセージの比較関数
def compare_ros2_msgs(msg1, msg2, rel_tol=1e-9, abs_tol=1e-9):
    if type(msg1) != type(msg2):
        return False

    for field_name in msg1.__slots__:
        val1 = getattr(msg1, field_name)
        val2 = getattr(msg2, field_name)

        # 配列の場合（List[float] など）
        if isinstance(val1, list) and isinstance(val2, list):
            if len(val1) != len(val2):
                return False
            for v1, v2 in zip(val1, val2):
                if isinstance(v1, float) and isinstance(v2, float):
                    if not isclose_floats(v1, v2, rel_tol, abs_tol):
                        return False
                elif hasattr(v1, '__slots__'):
                    if not compare_ros2_msgs(v1, v2, rel_tol, abs_tol):
                        return False
                elif v1 != v2:
                    return False

        # 配列型の配列（array.array など）
        elif isinstance(val1, array) and isinstance(val2, array):
            if len(val1) != len(val2):
                return False
            for v1, v2 in zip(val1, val2):
                if isinstance(v1, float) and isinstance(v2, float):
                    if not isclose_floats(v1, v2, rel_tol, abs_tol):
                        return False
                elif hasattr(v1, '__slots__'):
                    if not compare_ros2_msgs(v1, v2, rel_tol, abs_tol):
                        return False
                elif v1 != v2:
                    return False

        # ネストされたメッセージ型（Vector3, Pose など）
        elif hasattr(val1, '__slots__') and hasattr(val2, '__slots__'):
            if not compare_ros2_msgs(val1, val2, rel_tol, abs_tol):
                return False

        # float 型の誤差許容比較
        elif isinstance(val1, float) and isinstance(val2, float):
            if not isclose_floats(val1, val2, rel_tol, abs_tol):
                return False

        # その他の型は通常比較
        elif val1 != val2:
            return False

    return True

def main(args=None):
    rclpy.init(args=args)
    node = PubSubNode("mqtt_tester")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    print('EOF')

if __name__ == '__main__':
    main()
