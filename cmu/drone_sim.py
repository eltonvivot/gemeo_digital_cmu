import requests
import json
import random
import time

SYNC_ENDPOINT = "http://127.0.0.1:5000/sync"
current_position = {
    "x": 1.0,
  	"y": 1.0,
  	"z": 0.1
}
battery_level = 300
deactivate = []

def call_rest_api(method, address, content=None):
    try:
        response = None

        if method == 'GET':
            response = requests.get(address)
        elif method == 'POST':
            response = requests.post(address, json=json.loads(content))
        elif method == 'DELETE':
            response = requests.delete(address)
        else:
            raise ValueError(f"Invalid HTTP method: {method}")

        return response

    except requests.exceptions.RequestException as e:
        print(f"Error occurred while making the request: {e}")
        return None

def get_power_consumption():
    power_consumptions = {
        "DronePowerConsumption": random.uniform(18, 24),
      	"Peripherical1PowerConsumption": random.uniform(4.5, 6),
      	"Peripherical2PowerConsumption": random.uniform(3.0, 4),
      	"Peripherical3PowerConsumption": random.uniform(2.5, 3.2)
    }
    for key in deactivate:
        power_consumptions.pop(key, None)
    return "PowerConsumptions", power_consumptions


def get_sensors_data():
    sensors_data = {}
    key, value = get_power_consumption()
    sensors_data[key] = value
    sensors_data["BatteryLevel"] = battery_level
    sensors_data["DroneLocation"] = current_position
    return sensors_data

def sync_simulator():
    global battery_level, current_position, deactivate
    for i in range(1, 10):
        sensors_data = get_sensors_data()
        print(f"======= SYNC {i} =======")
        print(sensors_data)
        response = call_rest_api("POST", SYNC_ENDPOINT, json.dumps(sensors_data))
        result = json.loads(response.text)
        battery_level *= 0.9
        current_position["x"] = result["location"]["x"]*0.1 * i
        current_position["y"] = result["location"]["x"]*0.1 * i
        deactivate = result["deactivate"]
        print("<- WHAT IF RESULTS ->")
        print(
            f"Remaining power: {result['battery']} | Peripherials off: {deactivate}")
        print(f"Turning off periphericals: {deactivate}")
        print(f"======================")

if __name__ == '__main__':
    sync_simulator()
