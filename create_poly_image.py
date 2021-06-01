from PIL import Image
from PIL import ImageDraw
import utm
import json
import numpy as np
from scipy.spatial import ConvexHull
class polygon_handler:
    def __init__(self, polygons_api_url):

        #todo: create api get polygon method
        #todo: create reordering of polygon priority
        polygon_list = self.get_polygons(polygons_api_url)
        area_of_interest_polygon = polygon_list["interest"]  # change to whatever in json
        area_of_interest_rectangle = self.get_area_of_interest_rectangle(area_of_interest_polygon)
        self.rotated_rectangle_points, self.shift_point, self.alpha = self.map_rotation(area_of_interest_rectangle)
        self.polygon_image = self.create_image(self.rotated_rectangle_points[2][0], self.rotated_rectangle_points[2][1])
        self.polygon_image = self.draw_polygons(polygon_list, self.polygon_image)





    def get_pixel_value(self, x, y):
        ''' find the value of grey level in a given coordinate

        :param x: int, hoizontal pixel value
        :param y: int, vertical pixel value
        :param image: 2D array, image of polygons
        :return: grey level value of given pixel coordinate
        '''

        # todo: project point using point projection method
        # todo: convert to utm
        point = self.point_projection((x,y), self.shift_point, self.alpha)
        return self.polygon_image(point[0], point[1])

    def draw_polygons(self, polygons, image):
        ''' draw greyscale polygons by ID on a blank image

        :param polygons: list of mapped polygons
        :param image: blank greyscale image
        :return: image with polygons
        '''
        draw = ImageDraw.Draw(image)
        for polygon in polygons:
            draw.polygon(polygon["points"], fill=255) # fill = polygon["id"] # change color to id
        return image

    def create_image(self, width, height):
        """ creates image the size of the map used
        :param width: int, number of horizontal pixels
        :param height: int, number of vertical pixels
        :return: blank image with given dimensions
        """

        size = (width, height)
        color = 0
        mode = 'L'
        image = Image.new(mode=mode, size=size, color=color)
        return image

    def convert_to_utm(self, latitude, longitude):
        ''' convert geo to utm coordinates

        :param latitude: latitude geo coordinates
        :param longitude: longitude geo coordinates
        :return: 2D int tuple, utm coordinates
        '''

        UTMCoordinates = utm.from_latlon(latitude=latitude, longitude=longitude) # must check if calculation is accurate enough
        return UTMCoordinates

    def get_polygons(self, polyJSON):
        ''' turn json string to polygon object

        :param polyJSON: json string of polygons
        :return: array of dictionary of polygons coordinates
        '''

        polyDIC = []
        ind = 1
        data = json.load(polyJSON)
        #for poly in data:
        points = []
        for pt in data["Points"]: #for pt in poly["Points"]:
            point = convert_to_utm(pt["Latitude"], pt["Longitude"])
            points.append(int(point[0])-412400)
            points.append(int(point[1])-3857500)
        polyDIC.append({"id": ind, "points": points.copy()})
        ind += 1
        return polyDIC

    def map_rotation(self, points):
        '''
        take a rectangle shift and rotate it so it is parallel to the x,y axis
        :param points: coordinates of a rect angle
        :return points: new coordinates of a rectangle
        :return shifter: the original most left point
        :return alpha: angle of rotation
        '''

        x = []
        for p in points:
            x.append(p[0])

        # find left coordinate
        p = None
        ind = x.index(min(x))
        new_points = []
        for i in range(len(points)):
            new_points.append(points[((ind + i) % len(points))])

        shifter = (new_points[0])
        shifted_points = []
        for p in new_points:
            shifted_points.append((p[0] - new_points[0][0], p[1] - new_points[0][1]))

        beta = np.atan(shifted_points[1][1] / shifted_points[1][0])
        alpha = np.pi / 2 - beta

        rotated_rectangle_points = []
        for p in shifted_points:
            if p[0] == 0:
                rotated_rectangle_points.append(p)
            else:
                d = np.sqrt(np.pow(p[0], 2) + np.pow(p[1], 2))
                teta = np.atan(p[1] / p[0])
                rotated_rectangle_points.append((d * np.cos(teta + alpha), d * np.sin(teta + alpha)))

        # todo: add check rectangle and ceil

        return rotated_rectangle_points, shifter, alpha

    def point_projection(self, point, shifter, alpha):
        '''
        project a point to the correct map coordinate
        :param point: a latitude and longitude point tuple
        :param shifter: original point of projected rectangle
        :param alpha: angle of rotation
        :return: projected point
        '''
        point[0] = point[0] - shifter[0]
        point[1] = point[1] - shifter[1]
        d = np.sqrt(np.pow(point[0], 2) + np.pow(point[1], 2))
        teta = np.atan(point[1] / point[0])
        projected_point = (d * np.cos(teta + alpha), d * np.sin(teta + alpha))
        return projected_point


    def get_area_of_interest_rectangle(self, points_tuples):
        """
        Find the smallest bounding rectangle for a set of points.
        Returns a set of points representing the corners of the bounding box.

        :param points_tuples: an nx2 matrix of coordinates
        :min_bounding_box_vertices: a list of coordinates tuples
        """
        points = np.zeros(len(points_tuples,2))
        i = 0
        for p in points_tuples:
            points[i][0] = p[0]
            points[i][1] = p[1]


        pi2 = np.pi / 2

        # get the convex hull for the points
        hull_points = points[ConvexHull(points).vertices]

        # calculate edge angles
        edges = np.zeros((len(hull_points) - 1, 2))
        edges = hull_points[1:] - hull_points[:-1]

        angles = np.zeros((len(edges)))
        angles = np.arctan2(edges[:, 1], edges[:, 0])

        angles = np.abs(np.mod(angles, pi2))
        angles = np.unique(angles)

        # find rotation matrices
        rotations = np.vstack([
            np.cos(angles),
            np.cos(angles - pi2),
            np.cos(angles + pi2),
            np.cos(angles)]).T
        rotations = rotations.reshape((-1, 2, 2))

        # apply rotations to the hull
        rot_points = np.dot(rotations, hull_points.T)

        # find the bounding points
        min_x = np.nanmin(rot_points[:, 0], axis=1)
        max_x = np.nanmax(rot_points[:, 0], axis=1)
        min_y = np.nanmin(rot_points[:, 1], axis=1)
        max_y = np.nanmax(rot_points[:, 1], axis=1)

        # find the box with the best area
        areas = (max_x - min_x) * (max_y - min_y)
        best_idx = np.argmin(areas)

        # return the best box
        x1 = max_x[best_idx]
        x2 = min_x[best_idx]
        y1 = max_y[best_idx]
        y2 = min_y[best_idx]
        r = rotations[best_idx]

        rval = np.zeros((4, 2))
        rval[0] = np.dot([x1, y2], r)
        rval[1] = np.dot([x2, y2], r)
        rval[2] = np.dot([x2, y1], r)
        rval[3] = np.dot([x1, y1], r)

        min_bounding_box_vertices = []
        for r in rval:
            min_bounding_box_vertices.append(r[0], r[1])
        return min_bounding_box_vertices

#    def main(self):
#        with open('Polygon.json') as f:
#            polygons = get_polygons(f)

#        im = create_image(1000, 1000) # change to map size
#        im = draw_polygons(polygons, im)
#        im.show()

