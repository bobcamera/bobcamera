import rclpy
from rclpy.node import Node

class ActiveTopicsInspector(Node):
    def __init__(self):
        super().__init__('active_topics_inspector')
        self.find_topics_with_subscribers()

    def find_topics_with_subscribers(self):
        topic_names_and_types = self.get_topic_names_and_types()
        for topic, types in topic_names_and_types:
            # Get subscriber information
            sub_info = self.get_subscriptions_info_by_topic(topic)
            # Get publisher information
            pub_info = self.get_publishers_info_by_topic(topic)
            
            if sub_info:  # only if there are subscribers
                print(f"Topic: {topic}")
                print("  Publishers:")
                if pub_info:
                    for pub in pub_info:
                        print(f"    - {pub.node_name} ({pub.node_namespace})")
                else:
                    print("    - None")
                    
                print("  Subscribers:")
                for sub in sub_info:
                    print(f"    - {sub.node_name} ({sub.node_namespace})")
                print()

def main():
    rclpy.init()
    node = ActiveTopicsInspector()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
