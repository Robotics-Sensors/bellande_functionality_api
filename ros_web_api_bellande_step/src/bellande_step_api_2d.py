import json
import requests
import os

# Get the parameters from the launch file
x1 = float(os.getenv('x1'))
y1 = float(os.getenv('y1'))
x2 = float(os.getenv('x2'))  # Read x2 from launch file
y2 = float(os.getenv('y2'))  # Read y2 from launch file
limit = int(os.getenv('limit'))

# URL and endpoint path
url = "https://bellanderoboticssensorsresearchinnovationcenter-kot42qxp.b4a.run"
endpoint_path = "/api/Bellande_Step/bellande_step_2d"

# JSON payload
payload = {
    "node0": {"x": x1, "y": y1},
    "node1": {"x": x2, "y": y2}
}

# Headers
headers = {
    'accept': 'application/json',
    'Content-Type': 'application/json'
}

# Make POST request
try:
    response = requests.post(
        url + endpoint_path + '?limit=' + str(limit),
        json=payload,
        headers=headers
    )
    response.raise_for_status()  # Raise an error for unsuccessful responses
    data = response.json()
    print("Next Step:", data['next_step'])
except requests.exceptions.RequestException as e:
    print("Error:", e)
