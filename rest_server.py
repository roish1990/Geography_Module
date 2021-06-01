from flask import Flask, jsonify, request
import create_poly_image

# polygon_finder = create_poly_image.polygon_handler("api")

app = Flask(__name__)
app.config["DEBUG"] = True

@app.route('/', methods=['GET'])
def home():
    return "<h1>Distant Reading Archive</h1><p>please enter latitude and longitude of desired point</p>"

@app.route('/api/v1/resources/polygons', methods=['GET'])
def api_location():
    # Check if an location was provided as part of the URL.
    # If location is provided, assign it to a variable.
    # If location ID is provided, display an error in the browser.
    if 'latitude' and 'longitude' in request.args:
        try:
            latitude = float(request.args['latitude'])
            longitude = float(request.args['longitude'])
        except:
            return "Error: Not Valid location. Please specify valid location."
    else:
        return "Error: No location field provided. Please specify a location."

    # Create an empty list for our results
    #todo: get polygon id logic

    # polygon_finder.get_pixel_value(latitude, longitude)
    results = {'exist': True, 'id': 123} # enter here polygon id and flag as dictionary
    return jsonify(results)


app.run()
