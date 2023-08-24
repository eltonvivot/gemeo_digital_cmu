import requests
import json
from subprocess import Popen
import time

example = {
	"PowerConsumptions": {
		"DronePowerConsumption": 5.0,
		"Peripherical1PowerConsumption": 3.3,
		"Peripherical2PowerConsumption": 1.5,
		"Peripherical3PowerConsumption": 1.8
	},
	"DroneLocation": {
		"x": 1.0,
		"y": 1.0,
		"z": 0.1
	},
	"BatteryLevel": 300
}


def read_template(path="scenario_template.json"):
    with open(path, 'r') as file:
        data = json.load(file)
        return data

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
        if address.endswith("kill"):
            print("Killed.")
        else: print(f"Error occurred while making the request: {e}")
        return None

def create_scenario_description(input_values=example):
    scenario_template = read_template()
    print(f'Setting {scenario_template["drones"][0]["battery"]["attributes"][0]["name"]} to {input_values["BatteryLevel"]}')
    scenario_template["drones"][0]["battery"]["attributes"][0]["values"] = input_values["BatteryLevel"]
    newPowerConsumption = [0.0, 0.0, input_values["PowerConsumptions"]["DronePowerConsumption"]]
    print(f'Setting {scenario_template["drones"][0]["peripherals"][0]["attributes"][0]["name"]} to {newPowerConsumption}')
    scenario_template["drones"][0]["peripherals"][0]["attributes"][0]["value"] = newPowerConsumption
    new_position = [input_values["DroneLocation"]["x"], input_values["DroneLocation"]["y"], input_values["DroneLocation"]["z"], ]
    print(f'Setting {scenario_template["drones"][0]["mobilityModel"]["attributes"][1]["name"]} to {new_position}')
    scenario_template["drones"][0]["mobilityModel"]["attributes"][1]["value"][0]["position"] = new_position
    return scenario_template

def start_iodsim():
    return Popen('cd ../ns3 && ./ns3 run iodsim', shell=True)

def predict_scenario(drone_values=example):
    content = create_scenario_description(drone_values)
    iod = start_iodsim()
    time.sleep(1)
    result = call_rest_api(method="POST", address="http://127.0.0.1:18080/whatif", content=json.dumps(content))
    call_rest_api(method="DELETE", address="http://127.0.0.1:18080/kill", content=json.dumps(content))
    iod.wait()

    obj = json.loads(result.text)
    deativate_periphericals, battery = calculate_optional_periphericals(
        obj["battery"], drone_values["PowerConsumptions"])
    obj["deactivate"] = deativate_periphericals
    obj["battery"] = battery
    return obj

def calculate_optional_periphericals(battery_level, drone_power_consumption):
    deaticvate_peripherical = []
    for key, val in drone_power_consumption.items():
        if key.startswith("Peripherical"):
            aux = battery_level - val*11
            if aux > 0:
                battery_level = aux
            else:
                deaticvate_peripherical.append(key)
    return deaticvate_peripherical, battery_level
