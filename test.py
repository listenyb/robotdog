import requests
import json
import os

BASE_URL = "http://127.0.0.1:9020"


def test_go_to_point():
    print("\n--- 开始发送 go_to_point 请求 ---")
    url = f"{BASE_URL}/api/v1/robot/go_to_0"
    try:
        response = requests.post(url)
        data = response.json()
        print('成功前往0点:', data)
    except requests.exceptions.RequestException as e:
        print('post failed',e)
    except json.JSONDecodeError:
        print(f'响应不是有效的 JSON: {response.text}')

test_go_to_point()
