# robot_api_server

此套件提供一個 Web API，用於使用 FastAPI 和 ROS2 Nav2 堆疊控制機器人導航。

## 安裝

1.  **安裝 Python 依賴項：**
    ```bash
    pip install fastapi "uvicorn[standard]"
    ```
2.  **建置 ROS2 工作區：**
    導航到您的 ROS2 工作區根目錄 (`/home/jetson/base_dev`) 並執行建置腳本：
    ```bash
    ./build_ros2.sh
    ```

## 使用方式

1.  **載入 ROS2 環境：**
    ```bash
    source install/setup.bash
    ```
2.  **啟動 Nav2 堆疊：**
    確保您的 Nav2 堆疊正在運行。您通常可以使用以下命令啟動它：
    ```bash
    ros2 launch nav2 autonomous_navigation.launch.py
    ```
    （請確保您已載入地圖並定位正常工作。）

3.  **啟動 API 伺服器：**
    在新的終端機中，載入 ROS2 環境後，啟動 API 伺服器：
    ```bash
    ros2 launch robot_api_server api_server.launch.py
    ```
    API 伺服器將在 `http://0.0.0.0:8000` 上啟動。

4.  **存取 API 文件：**
    您可以透過開啟網頁瀏覽器並導航到 `http://<您的機器人IP>:8000/docs` 來存取互動式 API 文件 (Swagger UI)。

5.  **發送導航目標：**
    您可以向 `/navigate_to_goal` 端點發送 POST 請求，其中包含所需的 `x`、`y` 座標和 `yaw_deg`（偏航角，以度為單位）。

    **使用 `curl` 的範例：**
    ```bash
    curl -X POST "http://<您的機器人IP>:8000/navigate_to_goal" \
         -H "Content-Type: application/json" \
         -d '{"x": 2.0, "y": 1.0, "yaw_deg": 90.0}'
    ```
    將 `<您的機器人IP>` 替換為您機器人的實際 IP 位址。

    **使用 Python `requests` 的範例：**
    ```python
    import requests

    robot_ip = "<您的機器人IP>" # 例如, "192.168.1.100"
    url = f"http://{robot_ip}:8000/navigate_to_goal"
    headers = {"Content-Type": "application/json"}
    data = {"x": 2.0, "y": 1.0, "yaw_deg": 90.0}

    try:
        response = requests.post(url, headers=headers, json=data)
        response.raise_for_status() # 對於 HTTP 錯誤拋出異常
        print("Response:", response.json())
    except requests.exceptions.RequestException as e:
        print(f"發送請求時發生錯誤: {e}")
    ```

## API 端點

### `POST /navigate_to_goal`

*   **描述：** 向 Nav2 堆疊發送導航目標。
*   **請求主體 (JSON)：**
    ```json
    {
      "x": float,         // 目標 X 座標 (公尺)
      "y": float,         // 目標 Y 座標 (公尺)
      "yaw_deg": float    // 目標偏航角 (度, 0-360)
    }
    ```
*   **回應：**
    *   `200 OK`：`{"message": "目標已接收，導航已啟動。"}`
    *   `500 Internal Server Error`：發送目標時發生問題。
    *   `503 Service Unavailable`：如果 ROS2 節點尚未準備好。

