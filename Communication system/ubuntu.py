from flask import Flask, request, jsonify
import rospy
from std_msgs.msg import String

app = Flask(__name__)

# Initialize the ROS node
rospy.init_node('flask_ros_bridge', anonymous=True)

# Initialize the ROS publisher
pub = rospy.Publisher('/sensor_topic', String, queue_size=10)

@app.route('/upload', methods=['POST'])
def upload_json():
    """ 接收 Pi3 发送的 JSON 数据 """
    try:
        json_data = request.get_json()
        print("✅ Received JSON:", json_data)
        
        if 'bin_full' in json_data:
            message_to_send = json_data['bin_full']
            
            ros_message = String()
            ros_message.data = message_to_send
       
            # Publish to the ROS topic
            pub.publish(ros_message)
            print("✅ Published to ROS topic:", ros_message.data)

        # 这里可以直接处理 JSON 数据，比如存数据库或前端调用
        return jsonify({"status": "success", "message": "JSON received!"})

    except Exception as e:
        return jsonify({"status": "error", "message": str(e)})

if __name__ == '__main__':
    app.run(host="0.0.0.0", port=5000)  # 监听所有 IP 地址
