from flask import Flask
from controller import *


def create_app():
    import routes
    app = Flask(__name__)
    app.register_blueprint(routes.dt_bp)
    # routes.init_app(app)
    return app


if __name__ == '__main__':
    app = create_app()
    app.run(debug=True)


    # predict_scenario()
