from flask import Blueprint, request, jsonify
from controller import predict_scenario

dt_bp = Blueprint('digital-twin', '__name__')

# GET all and POST


@dt_bp.route('/sync', methods=['POST'])
def handle_users():
    if request.method == 'POST':
        if request.is_json:
            result = predict_scenario(drone_values=request.get_json())
            result.pop("network", None)
            result.pop("time", None)
            result.pop("node-id", None)
            return result
        else:
            return {"error": "The request payload is not in JSON format"}

    elif request.method == 'GET':
        pass
