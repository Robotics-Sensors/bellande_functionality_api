import json
import os
import requests
import sys

def main():
    # Get the absolute path to the config file
    config_file_path = os.path.join(os.path.dirname(__file__), '../config/config2d.json')

    # Check if the config file exists
    if not os.path.exists(config_file_path):
        print("Config file not found:", config_file_path)
        return

    # Read configuration from config.json
    with open(config_file_path, 'r') as config_file:
        config = json.load(config_file)
        url = config['url']
        endpoint_path = config['endpoint_path']
    
    # Get the parameters from the launch file
    x1_str = os.getenv('x1')
    y1_str = os.getenv('y1')
    x2_str = os.getenv('x2')
    y2_str = os.getenv('y2')
    limit_str = os.getenv('limit')

    # Check if any of the environment variables are not set
    if any(v is None for v in [x1_str, y1_str, x2_str, y2_str, limit_str]):
        print("One or more required environment variables are not set.")
        return

    # Convert the parameters to float or int
    x1 = float(x1_str)
    y1 = float(y1_str)
    x2 = float(x2_str)
    y2 = float(y2_str)
    limit = int(limit_str)
    
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

if __name__ == '__main__':
    main()
