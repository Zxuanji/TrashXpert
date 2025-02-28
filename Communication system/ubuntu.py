from flask import Flask, request, jsonify
from flask_cors import CORS
import rospy
from std_msgs.msg import String
import time

app = Flask(__name__)
CORS(app)

# 初始化 ROS 节点
rospy.init_node('flask_ros_bridge', anonymous=True)
pub = rospy.Publisher('/sensor_topic', String, queue_size=10)

# 存储最新的 Pi3 & Pi4 数据
latest_data = {
    "general": 0,   # Pi3 提供
    "plastic": 3,   # Pi4 提供
    "metal": 0,     # Pi4 提供
    "paper": 0,     # Pi4 提供
}

# 记录最近的 bin_full 事件
bin_full_status = {
    "status": False,  # 当前是否有满载
    "timestamp": 0    # 记录最近一次满载的时间戳
}


@app.route('/upload', methods=['POST'])
def upload_json():
    """ 接收 Pi3 或 Pi4 发送的 JSON 数据，并更新最新数据 """
    try:
        json_data = request.get_json()
        print(json_data)
	
        if json_data.get('sensor_data'):
            sensor_data = json_data['sensor_data']

            if json_data.get("device") == "Pi4":
                latest_data["plastic"] = sensor_data.get("plastic_level", 0)
                latest_data["metal"] = sensor_data.get("metal_level", 0)
                latest_data["paper"] = sensor_data.get("paper_level", 0)

            elif json_data.get("device") == "Pi3":
                latest_data["general"] = sensor_data.get("waste_level", 0)

            # 发送数据到 ROS 话题
        print('bin_full' in json_data.keys())
        if 'bin_full' in json_data.keys():
            current_time = time.time()
            bin_full_status["status"] = True
            ros_message = String()
            ros_message.data = json_data['bin_full']
            print("✅ Published to ROS topic:", ros_message.data)

        return jsonify({"status": "success", "message": "JSON received!"})


    except Exception as e:
        return jsonify({"status": "error", "message": str(e)})

@app.route('/api/bin-status', methods=['GET'])
def check_bin_status():
    if bin_full_status["status"]:
        bin_full_status["status"] = False
        return jsonify({"bin_full": "Bin is full"})   
    return jsonify({"bin_full": "false"})  

    	
@app.route('/api/barchartdata', methods=['GET'])
def get_data():
    """ 格式化数据为 mockBarData 格式 """
    formatted_data = [
        {
            "bin_type": "General",
            "general": latest_data["general"],
            "generalColor": "hsl(340, 70%, 50%)",  # 可以修改为你喜欢的颜色
        },
        {
            "bin_type": "Plastic",
            "plastic": latest_data["plastic"],
            "plasticColor": "hsl(96, 70%, 50%)",  # 可以修改为你喜欢的颜色
        },
        {
            "bin_type": "Mental",
            "mental": latest_data["metal"],  # Metal 在这里显示为 Mental
            "mentalColor": "hsl(257, 70%, 50%)",  # 可以修改为你喜欢的颜色
        },
        {
            "bin_type": "paper",
            "paper": latest_data["paper"],
            "paperColor": "hsl(106, 70%, 50%)",  # 可以修改为你喜欢的颜色
        }
    ]
    return jsonify(formatted_data)  # 返回 JSON 数据
    
if __name__ == '__main__':
    app.run(host="0.0.0.0", port=5000)  # 监听所有 IP 地址
