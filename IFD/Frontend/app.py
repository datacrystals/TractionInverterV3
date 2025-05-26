from flask import Flask, jsonify

app = Flask(__name__)

@app.route('/')
def index():
    return app.send_static_file('index.html')

@app.route('/api/data', methods=['GET'])
def get_data():
    data = {
        'speed': 67,
        'temperature': 25,
    }
    return jsonify(data)

if __name__ == '__main__':
    app.run(debug=True)
