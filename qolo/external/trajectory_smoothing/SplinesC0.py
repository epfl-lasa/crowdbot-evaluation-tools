import Bezier_curves


class InvalidData(Exception):
    pass


class SplineC0:
    NEGATIVE_DATA_MESSAGE = "Please insert a positive degree!"
    NOT_INT_DATA_MESSAGE = "Please insert integers!"
    NONE_INTERVALS_MESSAGE = "Please insert intervals!"
    INCORRECT_COUNT_CONTROL_POINTS = "Please insert more control points!"
    NEGATIVE_INTERVAL_LENGTH = "Negative interval lengths inserted."

    def __init__(self, degree, intervals):
        if degree < 0:
            raise InvalidData(self.NEGATIVE_DATA_MESSAGE)

        if not isinstance(degree, int):
            raise InvalidData(self.NOT_INT_DATA_MESSAGE)

        if not intervals:
            raise InvalidData(self.NONE_INTERVALS_MESSAGE)

        for interval in intervals:
            if interval < 0:
                raise InvalidData(self.NEGATIVE_INTERVAL_LENGTH)

        self._degree = degree
        self.points_count = len(intervals) * degree + 1
        self.control_points = []
        self.partial_curves = []
        self.intervals = intervals
        self._spline_points = []

        for count in range(0, len(intervals)):
            self.partial_curves.append(Bezier_curves.BezierCurve())

        self.to_be_drawn = False
        self.control_polygon_to_be_drawn = False
        self.are_points_calculated = False

    def append_point(self, point):
        if len(self.control_points) == self.points_count:
            raise IndexError

        self.control_points.append(point)

        partial_curves_count = (len(self.control_points) - 1) // self._degree
        condition = (self.points_count == len(self.control_points))

        if ((len(self.control_points) - 1) % self._degree == 0 and
                (not condition) and len(self.control_points) > 1):
            self.partial_curves[partial_curves_count - 1].append_point(point)

        self.partial_curves[partial_curves_count -
                            condition].append_point(point)

    def replace_point(self, index, point):
        self._nullify()

        self.control_points[index] = point

        if index == 0:
            self.partial_curves[0].replace_point(index, point)
            return

        if index == len(self.partial_curves) * self._degree:
            self.partial_curves[len(self.partial_curves) - 1].replace_point(
                self._degree, point)
            return

        for curve_count in range(1, len(self.partial_curves) + 1):
            if index < curve_count * self._degree:
                if curve_count == 1:
                    self.partial_curves[curve_count - 1].replace_point(index,
                                                                       point)
                else:
                    self.partial_curves[curve_count - 1].replace_point(
                        index % ((curve_count - 1) * self._degree), point)
                return

            if index == curve_count * self._degree:
                self.partial_curves[curve_count - 1].replace_point(
                    self._degree, point)
                self.partial_curves[curve_count].replace_point(0, point)

                return

    def _nullify(self):
        self.are_points_calculated = False
        self._spline_points = []
        for curve in self.partial_curves:
            curve._nullify()

    def draw(self):
        if len(self.control_points) < self.points_count:
            raise InvalidData(self.INCORRECT_COUNT_CONTROL_POINTS)

        if not self.are_points_calculated:
            for curve in self.partial_curves:
                for point in curve.draw():
                    self._spline_points.append(point)
            self.are_points_calculated = True

        return self._spline_points
