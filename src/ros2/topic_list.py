import rclpy
from rclpy.node import Node

class ActiveTopicsInspector(Node):
    def __init__(self):
        super().__init__('active_topics_inspector')
        self.find_topics_with_subscribers()

    def find_topics_with_subscribers(self):
        topic_names_and_types = self.get_topic_names_and_types()
        for topic, types in topic_names_and_types:
            info = self.get_subscriptions_info_by_topic(topic)
            if info:  # only if there are subscribers
                print(f"Topic: {topic}")
                for sub in info:
                    print(f"  Subscribed by: {sub.node_name} ({sub.node_namespace})")

def main():
    rclpy.init()
    node = ActiveTopicsInspector()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
