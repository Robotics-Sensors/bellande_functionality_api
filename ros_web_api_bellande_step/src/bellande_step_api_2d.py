import json
import os
import requests

def main():
    # Read configuration from config.json
    with open('config.json', 'r') as config_file:
        config = json.load(config_file)
        url = config['url']
        endpoint_path = config['endpoint_path']

    # Get the parameters from the launch file
    x1 = float(os.getenv('x1'))
    y1 = float(os.getenv('y1'))
    x2 = float(os.getenv('x2'))
    y2 = float(os.getenv('y2'))
    limit = int(os.getenv('limit'))

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
