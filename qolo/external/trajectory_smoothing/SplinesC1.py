import SplinesC0


class SplineC1:
    NONE_EXISTING_SPLINE_MESSAGE = "Please increase the degree!"
    INCORRECT_COUNT_DEBOOR_POINTS_MESSAGE = "Please insert more deBoor points!"

    def __init__(self, degree, intervals):
        if degree in (0, 1):
            raise SplinesC0.InvalidData(self.NONE_EXISTING_SPLINE_MESSAGE)

        self.splineC0 = SplinesC0.SplineC0(degree, intervals)
        self.deBoor_points = []
        self.control_points = []
        self.partial_curve_counter = 0

        self.to_be_drawn = False
        self.control_polygon_Bezier_to_be_drawn = False
        self.control_polygon_deBoor_to_be_drawn = False
        self.are_points_calculated = False

    def append_deBoor_point(self, point):
        if len(self.deBoor_points) == (self.splineC0.points_count -
                                       len(self.splineC0.intervals) + 1):
            raise IndexError

        self.deBoor_points.append(point)

    def _calculate_Bezier_points(self, counter, index):
        denominator = (self.splineC0.intervals[counter - 1] +
                       self.splineC0.intervals[counter])
        numerator = (self.splineC0.intervals[counter] *
                     self.deBoor_points[index] +
                     self.splineC0.intervals[counter - 1] *
                     self.deBoor_points[index + 1])
        calculated_point = numerator * (1 / float(denominator))

        return calculated_point

    def _append_Bezier_points(self, movement=None):
        counter = 0
        if movement:
            len_points = 0

        for index in range(0, len(self.deBoor_points)):
            if not movement:
                self.splineC0.append_point(self.deBoor_points[index])
                len_points = len(self.splineC0.control_points)
            else:
                len_points += 1

            if (len_points % self.splineC0._degree == 0 and
                    len_points < (self.splineC0.points_count - 1) and
                    index > 0):
                counter += 1
                calculated_point = self._calculate_Bezier_points(counter,
                                                                 index)
                if not movement:
                    self.splineC0.append_point(calculated_point)
                else:
                    self.splineC0.replace_point(index + counter,
                                                calculated_point)
                    len_points += 1

        self.control_points = self.splineC0.control_points

    def replace_point(self, index, point):
        for partial_curve_count in range(
                1, len(self.splineC0.partial_curves) + 1):
            if ((index <= partial_curve_count * self.splineC0._degree -
                    partial_curve_count) or
                    partial_curve_count == len(self.splineC0.partial_curves)):
                index_for_splineC0 = index + partial_curve_count - 1
                break

        self.deBoor_points[index] = point
        self.splineC0.replace_point(index_for_splineC0, point)
        self._append_Bezier_points(True)
        self.control_points = self.splineC0.control_points

    def draw(self):
        if len(self.deBoor_points) < (self.splineC0.points_count -
                                      len(self.splineC0.intervals) + 1):
            raise SplinesC0.InvalidData(
                self.INCORRECT_COUNT_DEBOOR_POINTS_MESSAGE)

        if not self.are_points_calculated:
            self._append_Bezier_points()
            self.are_points_calculated = True

        return self.splineC0.draw()
