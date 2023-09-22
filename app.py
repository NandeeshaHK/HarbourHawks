from flask import Flask, render_template, request, send_file, jsonify
from flask_sse import sse
import os
import pandas as pd

app = Flask(__name__)
alert_flag = 0

# Load the CSV data into a Pandas DataFrame
df = pd.read_csv('your_data.csv')

# Define the directory where the images are stored locally
IMAGE_FOLDER = '/home/nerdnhk/HarbourHawks/Images/copy_img'

def get_newest_image_path():
    image_extensions = ["png", "jpg", "jpeg", "gif"]
    images = []

    for ext in image_extensions:
        images.extend([os.path.join(IMAGE_FOLDER, image) for image in os.listdir(IMAGE_FOLDER) if image.lower().endswith(ext)])

    if not images:
        return None

    newest_image = max(images, key=os.path.getmtime)
    return newest_image

@app.route('/')
def index():
    latest_image_path = get_newest_image_path()
    return render_template('index.html', latest_image_path=latest_image_path)

@app.route('/search', methods=['POST'])
def search():
    global alert_flag
    # Get the IMO number from the form
    imo_input = request.form['imo_number']
    
    # Remove any non-numeric characters from the input
    imo_number = ''.join(filter(str.isdigit, imo_input))

    # Search for the corresponding data in the DataFrame
    result = df[df['IMO'] == 'IMO' +str(imo_number)]

    if not result.empty:
        # Display the search result
        search_result = result.to_html(classes='table', escape=False, index=False)
    else:
        search_result = "No data found for the given IMO number.\n Required Details is fetched from the Specific Drone. \n And the relevent information will be sent to the Station"
        alert_flag = True
    # Render the template and return the search result
    return render_template('index.html', latest_image_path=get_newest_image_path(), search_result=search_result)

@app.route('/alert')
def alert():
    global alert_flag
    alert_flag = 1
    print(alert_flag)  # Replace with your condition
    sse.publish({"message": "Event triggered from Flask!"}, type='event-type')
    return render_template('index.html', alert_flag=alert_flag)

@app.route('/alertOff')
def alertOff():
    global alert_flag
    alert_flag = 0 # Replace with your condition
    print(alert_flag)
    return render_template('index.html', alert_flag=alert_flag)

@app.route('/api/latest_image')
def api_latest_image():
    latest_image_path = get_newest_image_path()
    if latest_image_path:
        return send_file(latest_image_path, mimetype='image/jpeg')
    else:
        return jsonify({'error': 'No latest image found'})

if __name__ == '__main__':
    app.run(debug=True)
